import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from cia402 import (
    OPERATION_ENABLED,
    SHUTDOWN_CONTROLWORDS,
    SWITCH_ON_DISABLED,
    decode_statusword,
    enables_operation,
)
from pdo import assess_raw_pdo


class ShutdownTest(unittest.TestCase):
    def test_shutdown_words_do_not_enable_operation(self):
        self.assertEqual(SHUTDOWN_CONTROLWORDS, (0x0006, 0x0000))
        for word in SHUTDOWN_CONTROLWORDS:
            self.assertFalse(enables_operation(word))
        self.assertTrue(enables_operation(0x000F))
        self.assertFalse(enables_operation(0x000F | 0x0080))

    def test_disabled_status_is_not_operation_enabled(self):
        self.assertEqual(decode_statusword(0x0250), SWITCH_ON_DISABLED)
        self.assertNotEqual(decode_statusword(0x0250), OPERATION_ENABLED)
        self.assertEqual(decode_statusword(0x0000), "Not Ready to Switch On")

    def test_ambiguous_pdo_blocks_the_operational_request(self):
        report = assess_raw_pdo([bytes.fromhex("0000"), bytes.fromhex("10004060")])
        self.assertFalse(report["safe"])
        self.assertFalse(report["safe"] and True)
