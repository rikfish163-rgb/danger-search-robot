from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import time

from building_generator_classic.control_runtime import BuildingControlRuntime
import yaml

DEFAULT_DOOR_ANIMATION_RATE_HZ = 10.0
DEFAULT_PASSENGER_MODELS = ("a1_gazebo",)
ELEVATOR_CAR_FLOOR_THICKNESS = 0.12
ELEVATOR_PASSENGER_XY_MARGIN = 0.35
ELEVATOR_PASSENGER_BELOW_FLOOR_TOLERANCE = 0.25
ELEVATOR_PASSENGER_ABOVE_FLOOR_TOLERANCE = 1.60


def main(argv: list[str] | None = None) -> int:
    import rospy
    from gazebo_msgs.srv import GetModelState, SetLinkState, SetModelState
    from building_generator_interfaces.srv import (
        CallElevator,
        CallElevatorResponse,
        SetDoorState,
        SetDoorStateResponse,
    )

    args = _build_parser().parse_args(rospy.myargv(argv=argv)[1:])
    runtime = _load_runtime(args.door_config, args.elevator_config)

    rospy.init_node("building_generator_classic_control")
    rospy.wait_for_service("/gazebo/get_model_state")
    rospy.wait_for_service("/gazebo/set_model_state")
    rospy.wait_for_service("/gazebo/set_link_state")
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    set_link_state = rospy.ServiceProxy("/gazebo/set_link_state", SetLinkState)
    passenger_models = _passenger_models(args.passenger_model)
    rospy.Service(
        "call_elevator",
        CallElevator,
        lambda request: _handle_call_elevator(
            runtime,
            request,
            CallElevatorResponse,
            get_model_state,
            set_model_state,
            set_link_state,
            passenger_models,
        ),
    )
    rospy.Service(
        "set_door_state",
        SetDoorState,
        lambda request: _handle_set_door_state(runtime, request, SetDoorStateResponse, set_model_state, set_link_state),
    )
    rospy.loginfo("building_generator_classic_control ready")
    rospy.spin()
    return 0


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Building generator Gazebo Classic control server")
    parser.add_argument("--door-config", required=True)
    parser.add_argument("--elevator-config", required=True)
    parser.add_argument(
        "--passenger-model",
        action="append",
        help="Gazebo model to carry when it is standing inside the elevator car. "
        "Can be repeated, or set ELEVATOR_PASSENGER_MODELS as a comma-separated list.",
    )
    return parser


def _load_runtime(door_config_path: str, elevator_config_path: str) -> BuildingControlRuntime:
    door_config = yaml.safe_load(Path(door_config_path).read_text()) or {}
    elevator_config = yaml.safe_load(Path(elevator_config_path).read_text()) or {}
    return BuildingControlRuntime(
        door_specs=door_config.get("doors", []),
        elevator_specs=elevator_config.get("elevators", []),
    )


def _handle_call_elevator(
    runtime: BuildingControlRuntime,
    request,
    response_type,
    get_model_state,
    set_model_state,
    set_link_state,
    passenger_models: tuple[str, ...],
):
    result = runtime.call_elevator(
        request.elevator_id,
        request.target_floor,
        request.open_doors,
    )
    if result.get("accepted") and result.get("target_pose"):
        passenger_states = _find_elevator_passengers(
            get_model_state,
            passenger_models,
            result.get("previous_pose"),
            result.get("car_size"),
        )
        _apply_model_pose(set_model_state, result["model_name"], result["target_pose"])
        _move_elevator_passengers(
            set_model_state,
            passenger_states,
            result.get("previous_pose"),
            result["target_pose"],
        )
        if request.open_doors and result.get("target_door_id"):
            door_result = runtime.set_door_state(result["target_door_id"], True)
            _apply_door_result(door_result, set_model_state, set_link_state)
    return response_type(
        accepted=bool(result["accepted"]),
        current_floor=int(result["current_floor"]),
        state=str(result["state"]),
        message=str(result["message"]),
    )


def _handle_set_door_state(runtime: BuildingControlRuntime, request, response_type, set_model_state, set_link_state):
    result = runtime.set_door_state(request.door_id, request.open)
    _apply_door_result(result, set_model_state, set_link_state)
    return response_type(
        accepted=bool(result["accepted"]),
        state=str(result["state"]),
        message=str(result["message"]),
    )


def _apply_door_result(result: dict, set_model_state, set_link_state) -> None:
    if result.get("accepted") and result.get("panel_poses"):
        _apply_model_pose(set_model_state, result["model_name"], result["model_pose"])
        motion_duration = float(result.get("motion_duration", 0.0) or 0.0)
        if motion_duration > 0.0:
            _animate_panel_poses(
                set_link_state,
                result["model_name"],
                result["model_pose"],
                result.get("start_panel_poses", {}) or {},
                result["panel_poses"],
                motion_duration,
            )
        else:
            _apply_panel_poses(
                set_link_state,
                result["model_name"],
                result["model_pose"],
                result["panel_poses"],
            )
    elif result.get("accepted") and result.get("target_pose"):
        _apply_model_pose(set_model_state, result["model_name"], result["target_pose"])


def _apply_panel_poses(
    set_link_state,
    model_name: str,
    model_pose: list[float],
    panel_poses: dict[str, list[float] | None],
) -> None:
    for link_name, pose_values in panel_poses.items():
        if pose_values:
            _apply_link_pose(
                set_link_state,
                model_name,
                link_name,
                _compose_world_pose(model_pose, pose_values),
            )


def _animate_panel_poses(
    set_link_state,
    model_name: str,
    model_pose: list[float],
    start_panel_poses: dict[str, list[float] | None],
    target_panel_poses: dict[str, list[float] | None],
    duration: float,
) -> None:
    import rospy

    duration = max(0.0, float(duration))
    if duration <= 0.0:
        _apply_panel_poses(set_link_state, model_name, model_pose, target_panel_poses)
        return

    step_count = max(1, int(math.ceil(duration * DEFAULT_DOOR_ANIMATION_RATE_HZ)))
    step_period = duration / step_count
    link_names = sorted(target_panel_poses.keys())
    start_time = time.monotonic()
    rospy.loginfo("Animating %s door panels over %.1f seconds", model_name, duration)

    for step in range(step_count + 1):
        if rospy.is_shutdown():
            return
        ratio = step / step_count
        current_poses: dict[str, list[float]] = {}
        for link_name in link_names:
            target_pose = target_panel_poses.get(link_name)
            if not target_pose:
                continue
            start_pose = start_panel_poses.get(link_name) or target_pose
            current_poses[link_name] = _interpolate_pose(start_pose, target_pose, ratio)
        _apply_panel_poses(set_link_state, model_name, model_pose, current_poses)

        next_time = start_time + (step + 1) * step_period
        sleep_time = next_time - time.monotonic()
        if step < step_count and sleep_time > 0.0:
            rospy.sleep(sleep_time)


def _interpolate_pose(start_pose: list[float], target_pose: list[float], ratio: float) -> list[float]:
    ratio = max(0.0, min(1.0, float(ratio)))
    return [
        float(start_value) + (float(target_value) - float(start_value)) * ratio
        for start_value, target_value in zip(start_pose, target_pose)
    ]


def _apply_model_pose(set_model_state, model_name: str, pose_values: list[float]) -> None:
    from gazebo_msgs.msg import ModelState
    from geometry_msgs.msg import Pose
    from tf.transformations import quaternion_from_euler

    model_state = ModelState()
    model_state.model_name = model_name
    model_state.reference_frame = "world"
    model_state.pose = Pose()
    model_state.pose.position.x = float(pose_values[0])
    model_state.pose.position.y = float(pose_values[1])
    model_state.pose.position.z = float(pose_values[2])
    qx, qy, qz, qw = quaternion_from_euler(
        float(pose_values[3]),
        float(pose_values[4]),
        float(pose_values[5]),
    )
    model_state.pose.orientation.x = qx
    model_state.pose.orientation.y = qy
    model_state.pose.orientation.z = qz
    model_state.pose.orientation.w = qw
    set_model_state(model_state)


def _passenger_models(cli_values: list[str] | None) -> tuple[str, ...]:
    raw_values: list[str] = []
    raw_values.extend(cli_values or [])
    env_value = os.environ.get("ELEVATOR_PASSENGER_MODELS", "")
    if env_value:
        raw_values.extend(env_value.split(","))
    cleaned = tuple(value.strip() for value in raw_values if value.strip())
    return cleaned or DEFAULT_PASSENGER_MODELS


def _find_elevator_passengers(
    get_model_state,
    passenger_models: tuple[str, ...],
    elevator_pose: list[float] | None,
    car_size: list[float] | None,
) -> list[tuple[str, object]]:
    if not elevator_pose or not car_size or len(car_size) < 3:
        return []

    passengers: list[tuple[str, object]] = []
    for model_name in passenger_models:
        try:
            model_state = get_model_state(model_name, "world")
        except Exception:
            continue
        if not getattr(model_state, "success", False):
            continue
        if _pose_inside_elevator_car(model_state.pose, elevator_pose, car_size):
            passengers.append((model_name, model_state))
    return passengers


def _pose_inside_elevator_car(pose, elevator_pose: list[float], car_size: list[float]) -> bool:
    local_x, local_y = _world_xy_to_local(
        pose.position.x - float(elevator_pose[0]),
        pose.position.y - float(elevator_pose[1]),
        float(elevator_pose[5]),
    )
    half_x = float(car_size[0]) / 2.0 + ELEVATOR_PASSENGER_XY_MARGIN
    half_y = float(car_size[1]) / 2.0 + ELEVATOR_PASSENGER_XY_MARGIN
    floor_z = _elevator_floor_z(elevator_pose, car_size)
    relative_z = float(pose.position.z) - floor_z
    return (
        abs(local_x) <= half_x
        and abs(local_y) <= half_y
        and -ELEVATOR_PASSENGER_BELOW_FLOOR_TOLERANCE <= relative_z <= ELEVATOR_PASSENGER_ABOVE_FLOOR_TOLERANCE
    )


def _world_xy_to_local(dx: float, dy: float, yaw: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        dx * cos_yaw + dy * sin_yaw,
        -dx * sin_yaw + dy * cos_yaw,
    )


def _elevator_floor_z(elevator_pose: list[float], car_size: list[float]) -> float:
    return float(elevator_pose[2]) - float(car_size[2]) / 2.0 + ELEVATOR_CAR_FLOOR_THICKNESS


def _move_elevator_passengers(
    set_model_state,
    passenger_states: list[tuple[str, object]],
    previous_pose: list[float] | None,
    target_pose: list[float],
) -> None:
    if not passenger_states or not previous_pose:
        return
    delta_x = float(target_pose[0]) - float(previous_pose[0])
    delta_y = float(target_pose[1]) - float(previous_pose[1])
    delta_z = float(target_pose[2]) - float(previous_pose[2])
    for model_name, model_state in passenger_states:
        _apply_existing_model_state(
            set_model_state,
            model_name,
            model_state.pose,
            model_state.twist,
            delta_x,
            delta_y,
            delta_z,
        )


def _apply_existing_model_state(set_model_state, model_name: str, pose, twist, dx: float, dy: float, dz: float) -> None:
    from gazebo_msgs.msg import ModelState

    model_state = ModelState()
    model_state.model_name = model_name
    model_state.reference_frame = "world"
    model_state.pose = pose
    model_state.pose.position.x += dx
    model_state.pose.position.y += dy
    model_state.pose.position.z += dz
    model_state.twist = twist
    set_model_state(model_state)


def _apply_link_pose(set_link_state, model_name: str, link_name: str, pose_values: list[float]) -> None:
    from gazebo_msgs.msg import LinkState
    from geometry_msgs.msg import Pose
    from tf.transformations import quaternion_from_euler

    link_state = LinkState()
    link_state.link_name = f"{model_name}::{link_name}"
    link_state.reference_frame = "world"
    link_state.pose = Pose()
    link_state.pose.position.x = float(pose_values[0])
    link_state.pose.position.y = float(pose_values[1])
    link_state.pose.position.z = float(pose_values[2])
    qx, qy, qz, qw = quaternion_from_euler(
        float(pose_values[3]),
        float(pose_values[4]),
        float(pose_values[5]),
    )
    link_state.pose.orientation.x = qx
    link_state.pose.orientation.y = qy
    link_state.pose.orientation.z = qz
    link_state.pose.orientation.w = qw
    set_link_state(link_state)


def _compose_world_pose(base_pose: list[float], local_pose: list[float]) -> list[float]:
    yaw = float(base_pose[5])
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    local_x = float(local_pose[0])
    local_y = float(local_pose[1])
    return [
        float(base_pose[0]) + local_x * cos_yaw - local_y * sin_yaw,
        float(base_pose[1]) + local_x * sin_yaw + local_y * cos_yaw,
        float(base_pose[2]) + float(local_pose[2]),
        float(base_pose[3]) + float(local_pose[3]),
        float(base_pose[4]) + float(local_pose[4]),
        float(base_pose[5]) + float(local_pose[5]),
    ]

__all__ = ["main"]
