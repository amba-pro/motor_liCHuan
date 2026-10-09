"""Profile Position helpers.

Device-specific sources for LC10E V1.04, product 0x00000402 revision 0x00000204:

- The LC-E manual (20250705) section 7.1.1 defines controlword bits 0-3, 7,
  and 8. Bits 4-6 and 9 are only "operation mode specific". Section 7.6 lists
  the PP objects and does not assign those bits.
- Section 7.1.2 defines status bit 10 as target reached. Bits 12-13 are only
  "operation mode specific".
- The LC10E V1.04 ESI for this identity contains the PDO map and no controlword
  or statusword bit text.
- A different Lichuan manual (manufacturer 0x0A79, product 0x1000) shows
  controlword 0x1F and 0x3F. That is not this drive.

PP_BITS_CONFIRMED therefore stays false. The state machine below is the
candidate sequence and cannot be started on the drive until that flag is set
from device-specific evidence. It has not been run on this hardware.
"""

from __future__ import annotations

import config
from cia402 import enables_operation

PP_MODE = 1
TARGET_REACHED = 0x0400

# Manual section 7.1, rows 2, 3, 4, and 7.
SHUTDOWN = 0x0006
SWITCH_ON = 0x0007
ENABLE_OPERATION = 0x000F
DISABLE_VOLTAGE = 0x0000

# Candidate bits. Not confirmed for this firmware.
NEW_SETPOINT = 0x0010
CHANGE_SET_IMMEDIATELY = 0x0020
ABSOLUTE = 0x0000
SETPOINT_ACK = 0x1000

PENDING = "pending"
ACKNOWLEDGED = "acknowledged"
REACHED = "target_reached"
TIMED_OUT = "timeout"
FAULT = "fault"
UNEXPECTED = "unexpected"


class ProfileBitsUnverified(RuntimeError):
    """The PP setpoint bits are not defined for this drive."""


def absolute_setpoint_words() -> tuple[int, int]:
    """Return the absolute new-setpoint pair once the bits are confirmed.

    The pair is enable-operation with bit 4, then the same word with bit 5.
    Bit 6 stays clear, which is absolute rather than relative. Neither word is
    produced while PP_BITS_CONFIRMED is false.
    """
    if not config.PP_BITS_CONFIRMED:
        raise ProfileBitsUnverified(
            "LC-E manual 7.1.1 does not define Profile Position controlword "
            "bits 4, 5, 6, and 9; the setpoint handshake is not verified"
        )
    armed = ENABLE_OPERATION | NEW_SETPOINT
    immediate = armed | CHANGE_SET_IMMEDIATELY
    if not enables_operation(armed) or not enables_operation(immediate):
        raise ProfileBitsUnverified("setpoint words lost the enable-operation command")
    if immediate & 0x0040:
        raise ProfileBitsUnverified("relative positioning bit is set")
    return armed, immediate


def target_reached(statusword: int) -> bool:
    return (statusword & TARGET_REACHED) != 0


def profile_units(speed_rpm: float, units_per_rev: float, accel_rpm_per_s: float | None = None) -> tuple[int, int]:
    """Profile velocity and acceleration in command units, from a known scale."""
    if units_per_rev <= 0:
        raise ValueError("command units per revolution are unknown")
    if speed_rpm <= 0 or speed_rpm > config.MAX_SPEED_RPM:
        raise ValueError(f"speed {speed_rpm} rpm exceeds the configured limit")
    accel = config.MAX_ACCEL_RPM_PER_S if accel_rpm_per_s is None else accel_rpm_per_s
    if accel <= 0 or accel > config.MAX_ACCEL_RPM_PER_S:
        raise ValueError(f"acceleration {accel} rpm/s exceeds the configured limit")
    velocity = max(1, int(round(speed_rpm / 60.0 * units_per_rev)))
    acceleration = max(1, int(round(accel / 60.0 * units_per_rev)))
    return velocity, acceleration


def note_wkc(wkc: int, expected: int, misses: int, limit: int = 3) -> tuple[int, bool]:
    """Count consecutive working-counter misses. Trip at the limit."""
    misses = 0 if wkc == expected else misses + 1
    return misses, misses >= limit


def _operation_enabled(statusword: int) -> bool:
    return (statusword & 0x006F) == 0x0027


def _faulted(statusword: int) -> bool:
    return (statusword & 0x004F) in (0x0008, 0x000F)


class PpHandshake:
    """Candidate absolute-position handshake. Refuses to start while unconfirmed.

    The candidate sequence, for a single absolute target, is: raise controlword
    bit 4, wait for status bit 12, clear bit 4, wait for bit 12 to fall, then
    wait for status bit 10. Bit 5 selects immediate change. Bit 6 stays clear.
    """

    def __init__(self, now, timeout_s: float = 1.0, change_immediately: bool = True) -> None:
        if not config.PP_BITS_CONFIRMED:
            raise ProfileBitsUnverified(
                "Profile Position setpoint bits are not confirmed for LC10E V1.04"
            )
        if timeout_s <= 0:
            raise ValueError("timeout must be positive")
        self._now = now
        self._timeout_s = timeout_s
        self._immediate = change_immediately
        self._phase = "wait_ack"
        self._phase_started = now()
        self.outcome = PENDING

    def controlword(self) -> int:
        word = ENABLE_OPERATION
        if self._immediate:
            word |= CHANGE_SET_IMMEDIATELY
        if self._phase == "wait_ack":
            word |= NEW_SETPOINT
        if word & 0x0040:
            raise ProfileBitsUnverified("relative bit must stay clear")
        return word

    def poll(self, statusword: int) -> str:
        if self.outcome not in (PENDING, ACKNOWLEDGED):
            return self.outcome
        if _faulted(statusword):
            self.outcome = FAULT
            return self.outcome
        if not _operation_enabled(statusword):
            self.outcome = UNEXPECTED
            return self.outcome
        if self._now() - self._phase_started > self._timeout_s:
            self.outcome = TIMED_OUT
            return self.outcome
        acknowledged = (statusword & SETPOINT_ACK) != 0
        if self._phase == "wait_ack" and acknowledged:
            self._phase = "wait_ack_fall"
            self._phase_started = self._now()
        elif self._phase == "wait_ack_fall" and not acknowledged:
            self._phase = "wait_reached"
            self._phase_started = self._now()
            self.outcome = ACKNOWLEDGED
        elif self._phase == "wait_reached" and target_reached(statusword):
            self.outcome = REACHED
        return self.outcome


def fault_shutdown_words() -> tuple[int, int]:
    """Shutdown, then disable voltage. Does not command the external brake."""
    if enables_operation(SHUTDOWN) or enables_operation(DISABLE_VOLTAGE):
        raise RuntimeError("fault shutdown would enable operation")
    return SHUTDOWN, DISABLE_VOLTAGE
