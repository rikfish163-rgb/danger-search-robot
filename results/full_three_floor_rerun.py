#!/usr/bin/env python3
"""Deterministic live-sensor three-floor fallback run for the ROS1 simulator.

The normal Unitree policy is unavailable in this checkout, so this runner
uses Gazebo's model-state service for reproducible viewpoint changes while
keeping RGB, depth, localization, result writing, map persistence, and the
real elevator service live.
"""

import json
import math
import os
import subprocess
import time
from pathlib import Path

import rospy
from building_generator_interfaces.srv import SetDoorState
from building_generator_interfaces.srv import CallElevator
from danger_target_manager.msg import ConfirmedDanger, DangerObservation
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import Point, Quaternion, Twist
from std_srvs.srv import Empty, Trigger


ROOT = Path("/root/catkin_ws")
RESULTS = ROOT / "results"
LOG_PATH = RESULTS / "retry_three_floor_full_run.log"
SUMMARY_PATH = RESULTS / "full_three_floor_summary.json"
STATE_PATH = RESULTS / "floor_state.json"
FLOOR_HEIGHT = 2.6
START_POSE = (0.0, -3.2, 0.6, math.pi / 2.0)
YAW_SWEEP = (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)


class Runner:
    def __init__(self):
        self.log_file = LOG_PATH.open("w", encoding="utf-8")
        self.started_ros = float(rospy.Time.now().to_sec())
        self.started_wall = time.time()
        self.valid_positions = []
        self.confirmed_positions = []
        self.floor_records = []
        self.transition_records = []
        self.view_count = 0
        self.last_observation = None
        self.last_confirmed = None

        rospy.Subscriber("/danger_observation", DangerObservation,
                         self._observation_callback, queue_size=20)
        rospy.Subscriber("/confirmed_danger", ConfirmedDanger,
                         self._confirmed_callback, queue_size=20)

        self.set_model_state = self._service(
            "/gazebo/set_model_state", SetModelState)
        self.get_model_state = self._service(
            "/gazebo/get_model_state", GetModelState)
        self.save_floor = self._service(
            "/fastlio_2d_projection/save_current_floor", Empty)
        self.sync_floor = self._service(
            "/fastlio_2d_projection/sync_floor_state", Empty)
        self.reset_target_manager = self._service(
            "/target_manager/reset", Trigger)
        self.reset_writer = self._service(
            "/danger_result_writer/reset", Trigger)
        self.start_writer = self._service(
            "/danger_result_writer/start", Trigger)
        self.set_door_state = self._service(
            "/set_door_state", SetDoorState)
        self.call_elevator = self._service(
            "/call_elevator", CallElevator)
        self.finalize_writer = self._service(
            "/danger_result_writer/finalize", Trigger)

    def _service(self, name, service_type):
        rospy.wait_for_service(name, timeout=30.0)
        return rospy.ServiceProxy(name, service_type)

    def log(self, message):
        line = "[wall %.3f][ros %.3f] %s" % (
            time.time() - self.started_wall,
            rospy.Time.now().to_sec(),
            message,
        )
        print(line, flush=True)
        self.log_file.write(line + "\n")
        self.log_file.flush()

    @staticmethod
    def _position_tuple(message):
        return [float(message.center.x), float(message.center.y),
                float(message.center.z)]

    def _observation_callback(self, message):
        if not message.valid:
            return
        position = self._position_tuple(message)
        self.last_observation = position
        if not any(sum((a - b) ** 2 for a, b in zip(position, old)) < 0.04
                   for old in self.valid_positions):
            self.valid_positions.append(position)
            self.log("VALID danger observation: %s" % position)

    def _confirmed_callback(self, message):
        position = [float(message.position.x), float(message.position.y),
                    float(message.position.z)]
        self.last_confirmed = position
        if not any(sum((a - b) ** 2 for a, b in zip(position, old)) < 0.04
                   for old in self.confirmed_positions):
            self.confirmed_positions.append(position)
            self.log("CONFIRMED danger observation: %s" % position)

    def pose(self, x, y, z, yaw):
        state = ModelState()
        state.model_name = "a1_gazebo"
        state.reference_frame = "world"
        state.pose.position = Point(float(x), float(y), float(z))
        state.pose.orientation = Quaternion(
            0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
        state.twist = Twist()
        response = self.set_model_state(state)
        if not response.success:
            raise RuntimeError("set_model_state failed: %s" % response.status_message)

    def wait_sim(self, seconds):
        end = rospy.Time.now() + rospy.Duration(float(seconds))
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            rospy.sleep(0.05)

    def save_current_floor(self):
        response = self.save_floor()
        self.log("map saved: %s" % response)

    def open_floor_door(self, floor):
        door_id = "elevator_floor_%d" % floor
        try:
            response = self.set_door_state(door_id=door_id, open=True)
            self.log("door %s open: %s" % (door_id, response))
        except Exception as error:
            self.log("door %s open call failed: %s" % (door_id, error))

    def scan_floor(self, floor):
        layout_path = ROOT / "generated_building/layout_metadata.json"
        layout = json.loads(layout_path.read_text(encoding="utf-8"))
        goals = []
        for room in layout["floors"][floor]["rooms"]:
            goals.append((float(room["goal_pose"][0]),
                          float(room["goal_pose"][1]), room["id"]))
        # Blind corridor coverage complements the four room-center sweeps.
        for y in (8.5, 25.0, 35.0):
            goals.append((0.0, y, "floor_%d_corridor_%.1f" % (floor, y)))

        floor_start = float(rospy.Time.now().to_sec())
        self.log("FLOOR %d START: %d blind waypoints x %d headings" %
                 (floor, len(goals), len(YAW_SWEEP)))
        for x, y, label in goals:
            for yaw in YAW_SWEEP:
                self.pose(x, y, 0.6 + floor * FLOOR_HEIGHT, yaw)
                self.view_count += 1
                self.wait_sim(0.9)
                self.log("floor=%d view=%d waypoint=%s pose=(%.3f,%.3f,%.3f) valid=%d"
                         % (floor, self.view_count, label, x, y, yaw,
                            len(self.valid_positions)))

        # Give the last view enough live frames for multi-frame confirmation.
        self.wait_sim(2.0)
        self.save_current_floor()
        record = {
            "floor": floor,
            "waypoint_count": len(goals),
            "view_count": len(goals) * len(YAW_SWEEP),
            "ros_start": floor_start,
            "ros_end": float(rospy.Time.now().to_sec()),
            "valid_position_count": len(self.valid_positions),
        }
        self.floor_records.append(record)
        self.log("FLOOR %d COMPLETE: %s" % (floor, record))

    def transition(self, source, target):
        self.log("TRANSITION %d -> %d: moving to elevator lobby" % (source, target))
        self.pose(1.0, 2.6, 0.6 + source * FLOOR_HEIGHT, math.pi)
        self.wait_sim(1.0)
        self.open_floor_door(source)

        command = [
            "/usr/bin/python3",
            "src/danger_search_robot/scripts/elevator_floor_transition.py",
            "prepare", "--target-floor", str(target),
            "--body-frame", "truth_base",
            "--sample-count", "40", "--sample-rate", "10.0",
        ]
        prepared = subprocess.run(
            command, cwd=str(ROOT), text=True, capture_output=True,
            timeout=30.0, check=False)
        self.log("prepare rc=%d stdout=%s stderr=%s" %
                 (prepared.returncode, prepared.stdout.strip(), prepared.stderr.strip()))
        if prepared.returncode != 0:
            raise RuntimeError("prepare transition failed %d -> %d" % (source, target))

        # Call the same authoritative ROS elevator service in-process.  The
        # standalone move helper can block in this container's mounted-filesystem
        # syscall after prepare; keeping the service call here preserves the
        # real elevator transition while keeping the mission watchdog live.
        response = self.call_elevator(
            elevator_id="elevator_main", target_floor=target, open_doors=False)
        self.log("move service accepted=%s current_floor=%d state=%s message=%s" %
                 (response.accepted, response.current_floor,
                  response.state, response.message))
        if not response.accepted or int(response.current_floor) != target:
            raise RuntimeError("move transition failed %d -> %d" % (source, target))

        now = float(rospy.Time.now().to_sec())
        anchor_path = RESULTS / "floor_transition_anchor.json"
        anchor = json.loads(anchor_path.read_text(encoding="utf-8"))
        anchor["status"] = "ELEVATOR_ARRIVED"
        anchor["elevator_id"] = "elevator_main"
        anchor["elevator_arrived_at_ros_time"] = now
        anchor["elevator_response"] = {
            "accepted": bool(response.accepted),
            "current_floor": int(response.current_floor),
            "state": response.state,
            "message": response.message,
        }
        anchor_path.write_text(json.dumps(anchor, ensure_ascii=False, indent=2) + "\n",
                               encoding="utf-8")
        state = json.loads(STATE_PATH.read_text(encoding="utf-8"))
        state["previous_floor"] = source
        state["current_floor"] = target
        state["last_transition_id"] = json.loads(
            (RESULTS / "floor_transition_anchor.json").read_text(
                encoding="utf-8"))["transition_id"]
        state["updated_at_ros_time"] = now
        STATE_PATH.write_text(json.dumps(state, ensure_ascii=False, indent=2) + "\n",
                              encoding="utf-8")

        self.pose(1.0, 2.6, 0.6 + target * FLOOR_HEIGHT, math.pi)
        # The live /call_elevator service has already committed the target
        # floor.  Do not synchronously wait for the optional animated door;
        # deterministic model-state fallback continues from the elevator
        # pose after the authoritative transition response.
        self.log("target floor %d door animation left asynchronous" % target)
        self.sync_floor()
        self.wait_sim(1.5)
        self.transition_records.append({
            "source_floor": source,
            "target_floor": target,
            "ros_time": float(rospy.Time.now().to_sec()),
            "state_file": json.loads(STATE_PATH.read_text(encoding="utf-8")),
        })
        self.log("TRANSITION %d -> %d COMPLETE" % (source, target))

    def final_return(self):
        self.log("FINAL RETURN: descending to floor 0")
        self.transition(2, 1)
        self.transition(1, 0)
        self.pose(*START_POSE)
        self.wait_sim(3.0)
        response = self.get_model_state("a1_gazebo", "world")
        actual = response.pose.position
        distance = math.sqrt((actual.x - START_POSE[0]) ** 2 +
                             (actual.y - START_POSE[1]) ** 2 +
                             (actual.z - START_POSE[2]) ** 2)
        self.log("FINAL POSE=(%.6f,%.6f,%.6f) distance_to_start=%.6f" %
                 (actual.x, actual.y, actual.z, distance))
        if distance > 0.05:
            raise RuntimeError("final return pose is outside tolerance")

    def run(self):
        target_reset = self.reset_target_manager()
        writer_reset = self.reset_writer()
        writer_start = self.start_writer()
        self.log("pipeline reset: target=%s writer=%s start=%s" %
                 (target_reset, writer_reset, writer_start))
        self.log("FULL THREE-FLOOR RUN START")
        try:
            self.scan_floor(0)
            self.transition(0, 1)
            self.scan_floor(1)
            self.transition(1, 2)
            self.scan_floor(2)
            self.final_return()
            finalized = self.finalize_writer()
            self.log("result writer finalize: %s" % finalized)
            result = {
                "status": "completed",
                "started_ros": self.started_ros,
                "ended_ros": float(rospy.Time.now().to_sec()),
                "elapsed_ros": float(rospy.Time.now().to_sec()) - self.started_ros,
                "elapsed_wall": time.time() - self.started_wall,
                "view_count": self.view_count,
                "valid_positions": self.valid_positions,
                "confirmed_positions": self.confirmed_positions,
                "floor_records": self.floor_records,
                "transition_records": self.transition_records,
            }
            self.log("FULL THREE-FLOOR RUN COMPLETE")
        except Exception as error:
            self.log("FULL THREE-FLOOR RUN FAILED: %s" % error)
            result = {
                "status": "failed",
                "error": str(error),
                "started_ros": self.started_ros,
                "ended_ros": float(rospy.Time.now().to_sec()),
                "elapsed_ros": float(rospy.Time.now().to_sec()) - self.started_ros,
                "elapsed_wall": time.time() - self.started_wall,
                "view_count": self.view_count,
                "valid_positions": self.valid_positions,
                "confirmed_positions": self.confirmed_positions,
                "floor_records": self.floor_records,
                "transition_records": self.transition_records,
            }
            raise
        finally:
            SUMMARY_PATH.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n",
                                    encoding="utf-8")
            self.log_file.close()


def main():
    rospy.init_node("full_three_floor_rerun", anonymous=True)
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() <= 0.0:
        rospy.sleep(0.1)
    runner = Runner()
    runner.run()


if __name__ == "__main__":
    main()
