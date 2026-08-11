#!/usr/bin/env python3
"""DG-202602 evaluator defaults and coordinate invariance."""

import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "evaulate_danger.py"
SPEC = importlib.util.spec_from_file_location("evaulate_danger", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_scene_ratio_is_the_default_threshold_mode():
    args = MODULE._build_parser().parse_args([])
    assert args.threshold_mode == "scene-ratio"
    assert args.scene_ratio == 0.05
    assert args.detected_coordinate_frame == "robot-start-relative"


def test_fixed_threshold_requires_explicit_opt_in():
    args = MODULE._build_parser().parse_args(
        ["--threshold-mode", "fixed", "--threshold", "1.0"]
    )
    assert args.threshold_mode == "fixed"
    assert args.threshold == 1.0


def test_translation_to_spawn_relative_coordinates_preserves_distance():
    truth = MODULE.np.asarray([[7.0, 8.0, 2.0]])
    detected = MODULE.np.asarray([[7.1, 7.9, 2.0]])
    origin = MODULE.np.asarray([0.0, -3.2, 0.6])
    absolute = float(MODULE.np.linalg.norm(truth[0] - detected[0]))
    relative = float(
        MODULE.np.linalg.norm(
            (truth[0] - origin) - (detected[0] - origin)
        )
    )
    assert abs(absolute - relative) < 1e-12
