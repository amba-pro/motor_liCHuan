import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from pdo import assess_raw_pdo, build_disabled_output, decode_mapping_record, soem_assumed_bits

# Captured from the connected LC-E, active RxPDO 0x1702 and TxPDO 0x1B02.
RX_1702 = [
    bytes.fromhex("0000"),
    bytes.fromhex("10004060"),
    bytes.fromhex("2000"),
    bytes.fromhex("7a60"),
    bytes.fromhex("1000b860"),
    bytes.fromhex("10"),
]
TX_1B02 = [
    bytes.fromhex("0000"),
    bytes.fromhex("1000"),
    bytes.fromhex("10004160"),
    bytes.fromhex("2000"),
    bytes.fromhex("10007760"),
    bytes.fromhex("2000"),
    bytes.fromhex("1000b960"),
    bytes.fromhex("2000ba60"),
    bytes.fromhex("2000bc60"),
]


class MappingRecordTest(unittest.TestCase):
    def test_bit_length_comes_from_the_low_byte(self):
        record = decode_mapping_record(bytes.fromhex("10004060"))
        self.assertEqual(record["index"], 0x6040)
        self.assertEqual(record["subindex"], 0)
        self.assertEqual(record["bits"], 16)
        wide = decode_mapping_record(bytes.fromhex("20007a60"))
        self.assertEqual(wide["index"], 0x607A)
        self.assertEqual(wide["bits"], 32)

    def test_short_entries_are_not_mapping_records(self):
        self.assertIsNone(decode_mapping_record(bytes.fromhex("7a60")))
        self.assertIsNone(decode_mapping_record(bytes.fromhex("10")))
        self.assertIsNone(decode_mapping_record(b""))

    def test_live_rxpdo_matches_soem_output_size(self):
        report = assess_raw_pdo(RX_1702)
        self.assertTrue(report["ambiguous"])
        self.assertFalse(report["safe"])
        self.assertEqual(soem_assumed_bits(RX_1702), 202)
        self.assertEqual(report["soem_bytes"], 26)

    def test_live_txpdo_matches_soem_input_size(self):
        report = assess_raw_pdo(TX_1B02)
        self.assertTrue(report["ambiguous"])
        self.assertEqual(report["soem_bits"], 192)
        self.assertEqual(report["soem_bytes"], 24)

    def test_probe_object_is_refused_even_when_the_record_is_standard(self):
        raw = [
            bytes.fromhex("10004060"),
            bytes.fromhex("20007a60"),
            bytes.fromhex("1000b860"),
        ]
        report = assess_raw_pdo(raw)
        self.assertFalse(report["safe"])
        self.assertIn("0x60B8", report["reason"])

    def test_disabled_image_keeps_position_and_clears_controlword(self):
        raw = [bytes.fromhex("10004060"), bytes.fromhex("20007a60"), bytes.fromhex("2000ff60")]
        image = build_disabled_output(assess_raw_pdo(raw)["records"], actual_position=-2)
        self.assertEqual(image[:2], b"\x00\x00")
        self.assertEqual(int.from_bytes(image[2:6], "little", signed=True), -2)
        self.assertEqual(image[6:], b"\x00\x00\x00\x00")
