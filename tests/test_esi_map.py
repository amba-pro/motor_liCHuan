import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from esi_lc10e import (
    MANUAL_RX_1702,
    V104_RX_1702,
    V104_TX_1B02,
    build_disabled_image,
    can_use_soem_buffer,
    match_widths,
    payload_bytes,
    RX_CANDIDATES,
    TX_CANDIDATES,
)
from pdo import soem_assumed_bits

# SDO sizes captured from the connected drive before this recheck.
CAPTURED_RX_WIDTHS = [16, 32, 16, 16, 32, 8]
CAPTURED_TX_WIDTHS = [16, 16, 32, 16, 16, 16, 32, 32, 32]
RX_1702 = [
    bytes.fromhex("0000"),
    bytes.fromhex("10004060"),
    bytes.fromhex("2000"),
    bytes.fromhex("7a60"),
    bytes.fromhex("1000b860"),
    bytes.fromhex("10"),
]


class EsiMapTest(unittest.TestCase):
    def test_v104_and_manual_rx_sizes(self):
        self.assertEqual(payload_bytes(V104_RX_1702), 15)
        self.assertEqual(payload_bytes(MANUAL_RX_1702), 19)
        self.assertEqual(payload_bytes(V104_TX_1B02), 28)

    def test_captured_rx_widths_match_v104_only(self):
        self.assertEqual(match_widths(CAPTURED_RX_WIDTHS, RX_CANDIDATES), "LC10E V1.04 0x1702")
        self.assertIsNone(match_widths(CAPTURED_TX_WIDTHS, TX_CANDIDATES))

    def test_truncated_records_do_not_select_a_layout(self):
        self.assertIsNone(match_widths([len(item) * 8 for item in RX_1702[:4]], RX_CANDIDATES))

    def test_disabled_image_echoes_limits_and_holds_position(self):
        echoes = {0x60B8: 0x0020, 0x6071: 10, 0x607F: 1000, 0x6060: 0}
        image = build_disabled_image(V104_RX_1702, actual_position=-2, echoes=echoes)
        self.assertEqual(len(image), 15)
        self.assertEqual(image[:2], b"\x00\x00")
        self.assertEqual(int.from_bytes(image[2:6], "little", signed=True), -2)
        self.assertEqual(int.from_bytes(image[6:8], "little"), 0x0020)
        self.assertEqual(int.from_bytes(image[8:10], "little"), 10)
        self.assertEqual(int.from_bytes(image[10:14], "little"), 1000)
        self.assertEqual(image[14], 0)

    def test_missing_max_velocity_is_refused(self):
        with self.assertRaises(ValueError):
            build_disabled_image(V104_RX_1702, 0, {0x60B8: 0, 0x6060: 0})

    def test_soem_buffer_cannot_hold_the_larger_v104_input(self):
        soem_tx = (soem_assumed_bits([
            bytes.fromhex("0000"),
            bytes.fromhex("1000"),
            bytes.fromhex("10004160"),
            bytes.fromhex("2000"),
            bytes.fromhex("10007760"),
            bytes.fromhex("2000"),
            bytes.fromhex("1000b960"),
            bytes.fromhex("2000ba60"),
            bytes.fromhex("2000bc60"),
        ]) + 7) // 8
        self.assertEqual(soem_tx, 24)
        self.assertFalse(can_use_soem_buffer(payload_bytes(V104_TX_1B02), soem_tx))
        self.assertTrue(can_use_soem_buffer(payload_bytes(V104_RX_1702), 26))
