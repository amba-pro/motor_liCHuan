import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

import config
from cia402 import enables_operation
from jog import confirmation_blockers, preflight
from profile_position import (
    ACKNOWLEDGED,
    DISABLE_VOLTAGE,
    ENABLE_OPERATION,
    FAULT,
    PENDING,
    REACHED,
    TIMED_OUT,
    UNEXPECTED,
    PpHandshake,
    ProfileBitsUnverified,
    SHUTDOWN,
    SWITCH_ON,
    absolute_setpoint_words,
    fault_shutdown_words,
    note_wkc,
    profile_units,
    target_reached,
)


class ProfilePositionTest(unittest.TestCase):
    def test_manual_state_commands_match_section_7_1(self):
        self.assertEqual((SHUTDOWN, SWITCH_ON, ENABLE_OPERATION, DISABLE_VOLTAGE), (0x0006, 0x0007, 0x000F, 0x0000))
        self.assertFalse(enables_operation(SHUTDOWN))
        self.assertFalse(enables_operation(DISABLE_VOLTAGE))
        self.assertTrue(enables_operation(ENABLE_OPERATION))

    def test_setpoint_bits_stay_unverified(self):
        self.assertFalse(config.PP_BITS_CONFIRMED)
        with self.assertRaises(ProfileBitsUnverified):
            absolute_setpoint_words()

    def test_confirmed_setpoint_is_absolute_and_enabled(self):
        saved = config.PP_BITS_CONFIRMED
        try:
            config.PP_BITS_CONFIRMED = True
            armed, immediate = absolute_setpoint_words()
        finally:
            config.PP_BITS_CONFIRMED = saved
        self.assertEqual(armed, 0x001F)
        self.assertEqual(immediate, 0x003F)
        self.assertFalse(immediate & 0x0040)

    def test_target_reached_is_status_bit_10(self):
        self.assertTrue(target_reached(0x0250 | 0x0400))
        self.assertFalse(target_reached(0x0250))

    def test_profile_limits_use_only_a_supplied_scale(self):
        velocity, acceleration = profile_units(5.0, 3600.0)
        self.assertEqual(velocity, 300)
        self.assertEqual(acceleration, 1200)
        with self.assertRaises(ValueError):
            profile_units(5.0, 0)
        with self.assertRaises(ValueError):
            profile_units(6.0, 3600.0)

    def test_watchdog_trips_after_three_misses(self):
        misses, tripped = note_wkc(3, 3, 2)
        self.assertEqual((misses, tripped), (0, False))
        misses, tripped = note_wkc(0, 3, 0)
        misses, tripped = note_wkc(0, 3, misses)
        misses, tripped = note_wkc(0, 3, misses)
        self.assertTrue(tripped)

    def test_fault_shutdown_does_not_enable_or_touch_a_brake(self):
        self.assertEqual(fault_shutdown_words(), (0x0006, 0x0000))

    def test_r23_speed_and_acceleration_use_command_units_per_second(self):
        velocity, acceleration = profile_units(5.0, 8_388_608)
        self.assertEqual(velocity, 699_051)
        self.assertEqual(acceleration, 2_796_203)

    def test_handshake_is_blocked_for_this_firmware(self):
        self.assertFalse(config.PP_BITS_CONFIRMED)
        with self.assertRaises(ProfileBitsUnverified):
            PpHandshake(lambda: 0.0)

    def test_handshake_acknowledges_then_reaches_the_target(self):
        clock = [0.0]
        machine = self._machine(clock)
        self.assertEqual(machine.controlword(), 0x003F)
        self.assertEqual(machine.poll(0x0237), PENDING)
        self.assertEqual(machine.poll(0x1237), PENDING)
        self.assertEqual(machine.controlword(), 0x002F)
        self.assertEqual(machine.poll(0x0237), ACKNOWLEDGED)
        self.assertEqual(machine.poll(0x0637), REACHED)
        self.assertFalse(config.PP_BITS_CONFIRMED)

    def test_handshake_times_out(self):
        clock = [0.0]
        machine = self._machine(clock, timeout_s=1.0)
        self.assertEqual(machine.poll(0x0237), PENDING)
        clock[0] = 1.1
        self.assertEqual(machine.poll(0x0237), TIMED_OUT)

    def test_handshake_fault(self):
        clock = [0.0]
        machine = self._machine(clock)
        self.assertEqual(machine.poll(0x0218), FAULT)

    def test_handshake_rejects_an_unexpected_statusword(self):
        clock = [0.0]
        machine = self._machine(clock)
        self.assertEqual(machine.poll(0x0250), UNEXPECTED)

    def _machine(self, clock, timeout_s: float = 1.0) -> PpHandshake:
        saved = config.PP_BITS_CONFIRMED
        config.PP_BITS_CONFIRMED = True
        try:
            return PpHandshake(lambda: clock[0], timeout_s=timeout_s)
        finally:
            config.PP_BITS_CONFIRMED = saved


class JogPreflightTest(unittest.TestCase):
    def test_one_degree_command_is_refused_before_the_bus(self):
        blockers = preflight(1.0, 5.0, arm=False, confirm="")
        self.assertTrue(any("MOVE" in item for item in blockers))
        self.assertTrue(any("brake" in item for item in blockers))
        self.assertTrue(confirmation_blockers(True, "MOVE") == [])
        refused = preflight(1.0, 5.0, arm=True, confirm="MOVE")
        self.assertTrue(refused)
        self.assertFalse(any("MOVE" in item for item in refused))
