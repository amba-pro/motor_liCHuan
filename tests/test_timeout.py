import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from timing import wait_until


class TimeoutTest(unittest.TestCase):
    def test_returns_when_the_condition_becomes_true(self):
        clock = {"t": 0.0}
        seen = {"n": 0}

        def now():
            return clock["t"]

        def sleep(delay):
            clock["t"] += delay

        def ready():
            seen["n"] += 1
            return seen["n"] >= 3

        wait_until(ready, 1.0, sleep, now, interval_s=0.1)
        self.assertEqual(seen["n"], 3)
        self.assertAlmostEqual(clock["t"], 0.2)

    def test_raises_when_the_deadline_passes(self):
        clock = {"t": 10.0}

        def now():
            return clock["t"]

        def sleep(delay):
            clock["t"] += delay

        with self.assertRaises(TimeoutError):
            wait_until(lambda: False, 0.25, sleep, now, interval_s=0.1)
        self.assertGreaterEqual(clock["t"], 10.25)
