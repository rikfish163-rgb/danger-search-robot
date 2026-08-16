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
    mapping_health_ready,
    path_point_at_distance,
    path_cost_allowed,
    point_is_blacklisted,
    pose_is_close,
    pose_is_reasonable,
    pose_samples_stable,
    return_retry_available,
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

    def test_return_retry_budget_is_bounded(self):
        self.assertTrue(return_retry_available(0, 2))
        self.assertTrue(return_retry_available(1, 2))
        self.assertFalse(return_retry_available(2, 2))
        self.assertFalse(return_retry_available(0, 0))

    def test_local_path_cost_budget_rejects_unbounded_targets(self):
        self.assertTrue(path_cost_allowed(4.5, 5.0))
        self.assertFalse(path_cost_allowed(5.01, 5.0))
        self.assertFalse(path_cost_allowed(float("inf"), 5.0))

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

    def test_mapping_health_lease_is_fail_closed(self):
        self.assertTrue(mapping_health_ready(True, True, 0.4, 2.5))
        self.assertFalse(mapping_health_ready(False, True, 0.4, 2.5))
        self.assertFalse(mapping_health_ready(True, False, 0.4, 2.5))
        self.assertFalse(mapping_health_ready(True, True, 2.6, 2.5))
        self.assertFalse(mapping_health_ready(True, True, None, 2.5))

    def test_pose_guard_rejects_numeric_divergence(self):
        self.assertTrue(pose_is_reasonable((10.0, -20.0, 0.5), 200.0))
        self.assertFalse(pose_is_reasonable((10000.0, 0.0, 0.5), 200.0))
        self.assertFalse(pose_is_reasonable((float("nan"), 0.0, 0.5), 200.0))

    def test_long_global_path_is_bounded_to_one_safe_leg(self):
        point = path_point_at_distance(
            [(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)],
            2.0,
        )
        self.assertEqual(point, (2.0, 0.0))

    def test_blacklist_covers_the_bounded_leg_endpoint(self):
        self.assertTrue(
            point_is_blacklisted(
                (4.50, 0.02),
                [(4.50, 0.0)],
                1.0,
            )
        )
        self.assertFalse(
            point_is_blacklisted(
                (5.60, 0.0),
                [(4.50, 0.0)],
                1.0,
            )
        )

    def test_short_global_path_keeps_final_frontier_target(self):
        point = path_point_at_distance(
            [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
            5.0,
        )
        self.assertEqual(point, (1.0, 1.0))


if __name__ == "__main__":
    unittest.main()
