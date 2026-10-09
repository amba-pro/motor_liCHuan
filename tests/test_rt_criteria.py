import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

import config


class RealtimeCriteriaTest(unittest.TestCase):
    def test_wake_limit_stays_250us_on_a_1ms_cycle(self):
        header = (ROOT / "src" / "rt_setup.hpp").read_text()
        self.assertIn("constexpr int64_t kPeriodNs = 1000000;", header)
        self.assertIn("constexpr int64_t kLateLimitNs = 250000;", header)
        self.assertIn("constexpr int kMinAcceptCycles = 30000;", header)
        self.assertFalse(config.CSP_TIMING_ACCEPTED)
        self.assertFalse(config.MOTION_ARMED)
