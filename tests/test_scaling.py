import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from cia402 import (
    R23_COUNTS_PER_REV,
    confirm_r23_scale,
    counts_to_degrees,
    degrees_to_counts,
    encoder_pulse_count,
    evaluate_scaling,
    unpack_i32,
    unpack_u16,
)


class ScalingTest(unittest.TestCase):
    def test_gear_alone_is_not_counts_per_revolution(self):
        scaling = evaluate_scaling(1, 1)
        self.assertFalse(scaling.known)
        self.assertIsNone(scaling.command_units_per_rev)

    def test_does_not_invent_common_encoder_sizes(self):
        scaling = evaluate_scaling(1, 1, None, None)
        self.assertNotEqual(scaling.command_units_per_rev, 10000)
        self.assertNotEqual(scaling.command_units_per_rev, 131072)
        self.assertNotEqual(scaling.command_units_per_rev, 8388608)

    def test_derives_units_from_608f_and_gear(self):
        scaling = evaluate_scaling(2, 1, 131072, 1)
        self.assertTrue(scaling.known)
        self.assertEqual(scaling.command_units_per_rev, 65536)
        self.assertAlmostEqual(scaling.command_units_per_degree, 65536 / 360.0)

    def test_r23_registers_match_the_nameplate_and_the_gear(self):
        self.assertEqual(R23_COUNTS_PER_REV, 8388608)
        self.assertEqual(encoder_pulse_count(0, 0x0080), 8388608)
        scaling = confirm_r23_scale(1, 1, 0, 0x0080, -4228982, -4228982)
        self.assertTrue(scaling.known)
        self.assertEqual(scaling.command_units_per_rev, 8388608)
        self.assertEqual(degrees_to_counts(1.0, scaling.command_units_per_rev), 23302)

    def test_r23_scale_is_refused_when_the_registers_disagree(self):
        scaling = confirm_r23_scale(1, 1, 0, 0, -100, -100)
        self.assertFalse(scaling.known)
        self.assertIsNone(scaling.command_units_per_rev)

    def test_r23_scale_is_refused_when_positions_ignore_the_gear(self):
        scaling = confirm_r23_scale(2, 1, 0, 0x0080, -100, -100)
        self.assertFalse(scaling.known)

    def test_one_degree_round_trip(self):
        units = 360000.0
        counts = degrees_to_counts(1.0, units)
        self.assertEqual(counts, 1000)
        self.assertAlmostEqual(counts_to_degrees(counts, units), 1.0)


class EndianTest(unittest.TestCase):
    def test_statusword_little_endian(self):
        self.assertEqual(unpack_u16(bytes((0x50, 0x02))), 0x0250)

    def test_position_little_endian(self):
        raw = struct.pack("<i", -2)
        self.assertEqual(unpack_i32(raw), -2)
        self.assertEqual(raw, bytes((0xFE, 0xFF, 0xFF, 0xFF)))
