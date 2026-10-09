"""CSP guards. None of these set a latch or enable the servo.

The holding brake is powered from an external 24 V supply and is released.
A disable controlword does not engage it.
"""

from __future__ import annotations

import config
from cia402 import OPERATION_ENABLED, SWITCHED_ON, enables_operation
from cia402 import decode_statusword

# Shutdown, then disable voltage. Not a brake command, and not transmitted
# while the hardware latches are false.
CONTROLLED_STOP = (0x0006, 0x0000)

# About 0.04 degree at 8388608 counts/rev. Larger than the 2-count OP step.
UNEXPECTED_MOTION_COUNTS = 1000
FOLLOWING_LIMIT_COUNTS = 23302


def csp_blockers() -> list[str]:
    blockers = []
    if not config.MOTION_ARMED:
        blockers.append("MOTION_ARMED is false in python/config.py")
    if not config.BRAKE_CIRCUIT_VERIFIED:
        blockers.append(
            "external 24 V brake is released and is not controlled by the drive; "
            "software disable does not engage it"
        )
    if not config.ESTOP_AVAILABLE:
        blockers.append("an independent emergency stop is not marked available")
    if not config.MOTOR_SECURED:
        blockers.append("the unloaded motor is not marked secured")
    if not config.SHAFT_CLEAR:
        blockers.append("the shaft is not marked free of obstructions")
    if not config.MECHANICALLY_SAFE:
        blockers.append("the axis is not marked mechanically safe to move")
    if not config.AXIS_LIMITS_KNOWN:
        blockers.append("axis travel limits are not marked known")
    if not config.CSP_TIMING_ACCEPTED:
        blockers.append("the 1 ms CSP cycle is not marked accepted")
    return blockers


class Estop:
    """Hardware emergency stop. Confirming it does not set ESTOP_AVAILABLE."""

    def __init__(self) -> None:
        self.confirmed = False

    def confirm(self) -> None:
        if not config.ESTOP_AVAILABLE:
            raise RuntimeError("ESTOP_AVAILABLE is false; the circuit is not confirmed")
        self.confirmed = True

    def trip(self) -> None:
        self.confirmed = False

    @property
    def permits_motion(self) -> bool:
        return bool(config.ESTOP_AVAILABLE and self.confirmed and not csp_blockers())


def controlled_stop_words() -> tuple[int, int]:
    if any(enables_operation(word) for word in CONTROLLED_STOP):
        raise RuntimeError("controlled stop would enable operation")
    return CONTROLLED_STOP


def assess_cycle(
    status: int,
    position: int,
    following: int,
    hold: int,
    wkc: int,
    misses: int,
    elapsed_s: float,
    timeout_s: float,
    estop: Estop | None = None,
) -> str:
    """Return hold, stop, fault, watchdog, or unexpected. Never enable."""
    if estop is not None and not estop.confirmed and config.ESTOP_AVAILABLE:
        return "stop"
    if misses >= 3 or wkc != 3:
        return "watchdog" if misses >= 3 else "hold"
    name = decode_statusword(status)
    if name in ("Fault", "Fault Reaction Active"):
        return "fault"
    if name in (OPERATION_ENABLED, SWITCHED_ON):
        return "fault"
    if name not in ("Switch On Disabled", "Ready To Switch On", "Not Ready to Switch On"):
        return "fault"
    if abs(position - hold) > UNEXPECTED_MOTION_COUNTS:
        return "unexpected"
    if abs(following) > FOLLOWING_LIMIT_COUNTS:
        return "stop"
    if elapsed_s > timeout_s:
        return "stop"
    return "hold"
