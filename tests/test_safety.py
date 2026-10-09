import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

import pysoem

import config
from cia402 import evaluate_scaling
from safety import move_blockers, safety_blockers
from sdo import SdoClient, UnsupportedSdo


class FakeSlave:
    def sdo_read(self, index, subindex, size=0, ca=False):
        raise pysoem.SdoError(0, index, subindex, 0x06020000, "object does not exist")


class SafetyTest(unittest.TestCase):
    def test_default_config_blocks_motion(self):
        known = evaluate_scaling(1, 1, 131072, 1)
        blockers = safety_blockers(known)
        self.assertTrue(blockers)
        self.assertFalse(config.MOTION_ARMED)
        self.assertIn("MOTION_ARMED is false in python/config.py", blockers)

    def test_unknown_scaling_blocks_even_if_other_flags_were_set(self):
        saved = (
            config.MOTION_ARMED,
            config.BRAKE_CIRCUIT_VERIFIED,
            config.ESTOP_AVAILABLE,
            config.MECHANICALLY_SAFE,
            config.AXIS_LIMITS_KNOWN,
            config.PP_BITS_CONFIRMED,
        )
        try:
            config.MOTION_ARMED = True
            config.BRAKE_CIRCUIT_VERIFIED = True
            config.ESTOP_AVAILABLE = True
            config.MECHANICALLY_SAFE = True
            config.AXIS_LIMITS_KNOWN = True
            config.PP_BITS_CONFIRMED = True
            blockers = move_blockers(1.0, 5.0, evaluate_scaling(1, 1))
            self.assertTrue(any("scaling" in item for item in blockers))
        finally:
            (
                config.MOTION_ARMED,
                config.BRAKE_CIRCUIT_VERIFIED,
                config.ESTOP_AVAILABLE,
                config.MECHANICALLY_SAFE,
                config.AXIS_LIMITS_KNOWN,
                config.PP_BITS_CONFIRMED,
            ) = saved

    def test_oversized_move_is_rejected(self):
        blockers = move_blockers(90.0, 5.0, evaluate_scaling(1, 1, 131072, 1))
        self.assertTrue(any("MAX_MOVE_DEGREES" in item for item in blockers))

    def test_unsupported_sdo_is_reported(self):
        client = SdoClient(FakeSlave())
        value, err = client.optional(client.read_u16, 0xFFFF, 0)
        self.assertIsNone(value)
        self.assertIn("0xFFFF", err)
        with self.assertRaises(UnsupportedSdo):
            client.read_u16(0xFFFF, 0)


if __name__ == "__main__":
    unittest.main()
