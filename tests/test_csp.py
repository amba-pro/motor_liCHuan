import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

import config
from cia402 import R23_COUNTS_PER_REV, enables_operation
from csp import RX_BYTES, TX_BYTES, assess_periods, disabled_hold_image, future_csp_image, pack_rx, unpack_rx, unpack_tx
from csp_safety import Estop, assess_cycle, controlled_stop_words, csp_blockers
from servo_controller import LichuanServo
from trajectory import generate, one_degree_counts, preview_one_degree


class CspImageTest(unittest.TestCase):
    def test_verified_pdo_sizes_and_offsets(self):
        self.assertEqual((RX_BYTES, TX_BYTES), (15, 28))
        image = disabled_hold_image(-4_228_982, 0, 10, 104_857_600, 0)
        fields = unpack_rx(image)
        self.assertEqual(fields["controlword"], 0)
        self.assertEqual(fields["target"], -4_228_982)
        self.assertEqual(fields["torque"], 10)
        self.assertEqual(fields["max_velocity"], 104_857_600)
        self.assertEqual(fields["mode"], 0)
        self.assertEqual(image[14], 0)
        tx = unpack_tx(bytes(TX_BYTES))
        self.assertEqual(tx["position"], 0)
        self.assertEqual(tx["following"], 0)

    def test_enable_controlword_is_refused(self):
        with self.assertRaises(RuntimeError):
            pack_rx(0x000F, 0, 0, 0, 0, 8)
        self.assertTrue(enables_operation(0x000F))

    def test_mode_8_is_not_released(self):
        self.assertFalse(config.MOTION_ARMED)
        self.assertFalse(config.CSP_TIMING_ACCEPTED)
        with self.assertRaises(RuntimeError):
            future_csp_image(0, 0, 0, 0)

    def test_pysoem_op_path_stays_closed(self):
        servo = LichuanServo.__new__(LichuanServo)
        with self.assertRaises(Exception) as caught:
            servo._enter_op()
        self.assertIn("15/28", str(caught.exception))


class TimingTest(unittest.TestCase):
    def test_deadline_misses_are_counted_and_not_accepted(self):
        result = assess_periods([1_000_000, 1_100_000, 2_000_000])
        self.assertEqual(result["misses"], 1)
        self.assertFalse(result["accepted"])
        self.assertFalse(config.CSP_TIMING_ACCEPTED)


class TrajectoryTest(unittest.TestCase):
    def test_one_degree_uses_the_r23_scale(self):
        self.assertEqual(one_degree_counts(), 23302)
        self.assertEqual(R23_COUNTS_PER_REV, 8_388_608)

    def test_offline_one_degree_profile_stays_inside_the_limits(self):
        start = -4_228_982
        samples = preview_one_degree(start)
        self.assertEqual(samples[0], start)
        self.assertEqual(samples[-1], start + 23302)
        self.assertEqual(samples, sorted(samples))
        steps = [samples[i + 1] - samples[i] for i in range(len(samples) - 1)]
        self.assertLessEqual(max(steps), 700)
        self.assertGreater(len(samples), 10)

    def test_zero_move_and_reverse_move(self):
        self.assertEqual(generate(5, 5, 100, 1000, 10000), [5])
        reverse = generate(100, 0, 500, 5000, 50000, cycle_s=0.001)
        self.assertEqual(reverse[0], 100)
        self.assertEqual(reverse[-1], 0)
        self.assertEqual(reverse, sorted(reverse, reverse=True))


class CspSafetyTest(unittest.TestCase):
    def test_latches_block_csp_motion(self):
        blockers = csp_blockers()
        self.assertTrue(blockers)
        self.assertFalse(config.PP_BITS_CONFIRMED)
        self.assertIn("software disable does not engage it", " ".join(blockers))

    def test_controlled_stop_does_not_enable(self):
        words = controlled_stop_words()
        self.assertEqual(words, (0x0006, 0x0000))
        self.assertFalse(any(enables_operation(word) for word in words))

    def test_status_following_watchdog_and_motion(self):
        self.assertEqual(assess_cycle(0x0250, 10, 0, 10, 3, 0, 0.0, 8.0), "hold")
        self.assertEqual(assess_cycle(0x0237, 10, 0, 10, 3, 0, 0.0, 8.0), "fault")
        self.assertEqual(assess_cycle(0x0218, 10, 0, 10, 3, 0, 0.0, 8.0), "fault")
        self.assertEqual(assess_cycle(0x0250, 10, 0, 10, 0, 3, 0.0, 8.0), "watchdog")
        self.assertEqual(assess_cycle(0x0250, 10, 0, 10, 3, 0, 9.0, 8.0), "stop")
        self.assertEqual(assess_cycle(0x0250, 5000, 0, 0, 3, 0, 0.0, 8.0), "unexpected")
        self.assertEqual(assess_cycle(0x0250, 10, 30000, 10, 3, 0, 0.0, 8.0), "stop")

    def test_estop_interface_cannot_arm_itself(self):
        estop = Estop()
        with self.assertRaises(RuntimeError):
            estop.confirm()
        self.assertFalse(estop.permits_motion)
