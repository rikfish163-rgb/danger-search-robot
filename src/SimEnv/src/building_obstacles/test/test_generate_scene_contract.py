#!/usr/bin/env python3
"""Public scene metadata follows the DG-202602 coordinate contract."""

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace


SCRIPT = (
    Path(__file__).parents[1]
    / "scripts"
    / "generate_competition_scene.py"
)
SPEC = importlib.util.spec_from_file_location(
    "generate_competition_scene", SCRIPT
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_team_scene_declares_internal_and_result_frames_separately():
    layout = SimpleNamespace(door_specs=[], elevator_specs=[])
    start = {"x": 0.0, "y": -3.2, "z": 0.6, "yaw": 1.5708}
    document = MODULE._build_team_scene_info(
        layout,
        start,
        Path("/tmp/detected_danger.json"),
    )
    assert document["localization_frame"] == "world"
    assert (
        document["result_coordinate_frame"]
        == "robot_start_origin_world_axes"
    )
    assert document["robot_start"] == start
