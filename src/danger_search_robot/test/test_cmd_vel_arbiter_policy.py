#!/usr/bin/env python3
import pathlib
import sys
import unittest


SCRIPT_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from cmd_vel_arbiter_policy import choose_source  # noqa: E402


class CmdVelArbiterPolicyTest(unittest.TestCase):
    def test_safety_preempts_navigation(self):
        leases = {
            "navigation": (10, 10.0, 1.0),
            "safety": (100, 10.2, 1.5),
        }
        self.assertEqual(choose_source(10.3, leases), "safety")

    def test_expired_recovery_returns_to_navigation(self):
        leases = {
            "navigation": (10, 10.0, 1.0),
            "exploration": (60, 10.0, 0.5),
        }
        self.assertEqual(choose_source(10.7, leases), "navigation")

    def test_no_source_fails_to_zero(self):
        self.assertIsNone(choose_source(10.0, {}))

    def test_newer_tie_wins(self):
        leases = {
            "first": (20, 10.1, 1.0),
            "second": (20, 10.2, 1.0),
        }
        self.assertEqual(choose_source(10.3, leases), "second")


if __name__ == "__main__":
    unittest.main()
