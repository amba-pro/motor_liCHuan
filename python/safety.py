"""Gates that keep motion disabled until the operator confirms the hardware."""

from __future__ import annotations

import config
from cia402 import PositionScaling, counts_to_degrees, evaluate_scaling


def safety_blockers(scaling: PositionScaling | None = None) -> list[str]:
    blockers: list[str] = []
    if not config.MOTION_ARMED:
        blockers.append("MOTION_ARMED is false in python/config.py")
    if not config.BRAKE_CIRCUIT_VERIFIED:
        blockers.append(
            "external 24 V brake is released and is not controlled by the drive; "
            "independent brake control is not implemented"
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
    if not config.PP_BITS_CONFIRMED:
        blockers.append(
            "Profile Position controlword bits are not confirmed for this LC-E drive"
        )
    if scaling is None or not scaling.known or not scaling.command_units_per_rev:
        blockers.append("position scaling is unknown")
    elif scaling.command_units_per_rev < 180:
        blockers.append(
            "one command unit is more than 2 degrees; refusing even a 1-count move"
        )
    return blockers


def hardware_blockers() -> list[str]:
    """Latches that do not depend on encoder scaling."""
    return [
        item for item in safety_blockers(evaluate_scaling(1, 1, 1_000_000, 1))
        if "scaling" not in item and "command unit" not in item
    ]


def move_blockers(degrees: float, speed_rpm: float, scaling: PositionScaling | None) -> list[str]:
    blockers = safety_blockers(scaling)
    if abs(degrees) > config.MAX_MOVE_DEGREES + 1e-9:
        blockers.append(
            f"requested {degrees} deg exceeds MAX_MOVE_DEGREES={config.MAX_MOVE_DEGREES}"
        )
    if speed_rpm <= 0 or speed_rpm > config.MAX_SPEED_RPM:
        blockers.append(
            f"speed {speed_rpm} rpm is outside 0 < speed <= {config.MAX_SPEED_RPM}"
        )
    if scaling and scaling.known and scaling.command_units_per_rev:
        span = abs(counts_to_degrees(1, scaling.command_units_per_rev))
        if span > config.MAX_MOVE_DEGREES:
            blockers.append("a single count exceeds the configured move limit")
    return blockers
