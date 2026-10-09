import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from cia402 import (
    FAULT,
    FAULT_REACTION,
    OPERATION_ENABLED,
    QUICK_STOP,
    READY,
    SWITCH_ON_DISABLED,
    SWITCHED_ON,
    decode_statusword,
)


class StatuswordTest(unittest.TestCase):
    def test_manual_examples(self):
        # Values from LC-E manual section 7.1, including the extra remote/voltage bits.
        self.assertEqual(decode_statusword(0x0250), SWITCH_ON_DISABLED)
        self.assertEqual(decode_statusword(0x0231), READY)
        self.assertEqual(decode_statusword(0x0233), SWITCHED_ON)
        self.assertEqual(decode_statusword(0x0237), OPERATION_ENABLED)
        self.assertEqual(decode_statusword(0x0217), QUICK_STOP)
        self.assertEqual(decode_statusword(0x021F), FAULT_REACTION)
        self.assertEqual(decode_statusword(0x0218), FAULT)

    def test_mask_ignores_vendor_bits(self):
        self.assertEqual(decode_statusword(0x0237 | 0x0400 | 0x1000), OPERATION_ENABLED)
        self.assertEqual(decode_statusword(0x0040), SWITCH_ON_DISABLED)
        self.assertNotEqual(decode_statusword(0x0237), SWITCHED_ON)
