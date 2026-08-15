#!/usr/bin/env python3
import math
import pathlib
import sys
import unittest


SCRIPT_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from graph_nbv_policy import (  # noqa: E402
    failure_budget_exhausted,
    freshness_age,
    gate_exhaustion_reached,
    pose_is_close,
    pose_is_reasonable,
    pose_samples_stable,
)


class GraphNBVPolicyTest(unittest.TestCase):
    def test_gate_requires_configured_stable_cycles(self):
        self.assertFalse(gate_exhaustion_reached(7, 8))
        self.assertTrue(gate_exhaustion_reached(8, 8))
        self.assertTrue(gate_exhaustion_reached(20, 8))

    def test_gate_pose_must_be_stable_in_position_and_yaw(self):
        reference = (1.0, 2.0, 0.5)
        close = (1.04, 2.03, 0.53)
        far = (1.20, 2.03, 0.53)
        turned = (1.04, 2.03, 0.5 + math.radians(12.0))

        self.assertTrue(pose_is_close(reference, close, 0.10, math.radians(5.0)))
        self.assertFalse(pose_is_close(reference, far, 0.10, math.radians(5.0)))
        self.assertFalse(pose_is_close(reference, turned, 0.10, math.radians(5.0)))
        self.assertTrue(
            pose_samples_stable(
                [reference, close], 0.10, math.radians(5.0)
            )
        )

    def test_failure_budget_is_bounded(self):
        self.assertFalse(failure_budget_exhausted(4, 5))
        self.assertTrue(failure_budget_exhausted(5, 5))
        self.assertTrue(failure_budget_exhausted(6, 5))

    def test_freshness_prefers_receipt_time_over_delayed_header(self):
        self.assertAlmostEqual(
            freshness_age(stamped_at=90.0, received_at=99.0, now=100.0),
            1.0,
        )
        self.assertAlmostEqual(
            freshness_age(stamped_at=0.0, received_at=99.0, now=100.0),
            1.0,
        )
        self.assertIsNone(freshness_age(0.0, 0.0, 100.0))

    def test_pose_guard_rejects_numeric_divergence(self):
        self.assertTrue(pose_is_reasonable((10.0, -20.0, 0.5), 200.0))
        self.assertFalse(pose_is_reasonable((10000.0, 0.0, 0.5), 200.0))
        self.assertFalse(pose_is_reasonable((float("nan"), 0.0, 0.5), 200.0))


if __name__ == "__main__":
    unittest.main()
