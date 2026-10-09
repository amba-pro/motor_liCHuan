"""CiA402 status decode and position-scaling math. No hardware access."""

from __future__ import annotations

import struct
from dataclasses import dataclass


NOT_READY = "Not Ready to Switch On"
SWITCH_ON_DISABLED = "Switch On Disabled"
READY = "Ready To Switch On"
SWITCHED_ON = "Switched On"
OPERATION_ENABLED = "Operation Enabled"
QUICK_STOP = "Quick Stop Active"
FAULT_REACTION = "Fault Reaction Active"
FAULT = "Fault"
UNKNOWN = "Unknown"


def decode_statusword(status: int) -> str:
    """Decode a CiA402 statusword with the standard bit masks."""
    status &= 0xFFFF
    masked_4f = status & 0x004F
    if masked_4f == 0x0008:
        return FAULT
    if masked_4f == 0x000F:
        return FAULT_REACTION
    if masked_4f == 0x0000:
        return NOT_READY
    if masked_4f == 0x0040:
        return SWITCH_ON_DISABLED
    masked_6f = status & 0x006F
    if masked_6f == 0x0021:
        return READY
    if masked_6f == 0x0023:
        return SWITCHED_ON
    if masked_6f == 0x0027:
        return OPERATION_ENABLED
    if masked_6f == 0x0007:
        return QUICK_STOP
    return UNKNOWN


def ethercat_state_name(state: int) -> str:
    base = {
        0x01: "INIT",
        0x02: "PREOP",
        0x03: "BOOT",
        0x04: "SAFEOP",
        0x08: "OP",
    }.get(state & 0x0F, f"0x{state & 0x0F:02x}")
    if state & 0x10:
        base += "+ERROR"
    return base


def unpack_u8(data: bytes) -> int:
    if len(data) < 1:
        raise ValueError("need 1 byte")
    return data[0]


def unpack_i8(data: bytes) -> int:
    if len(data) < 1:
        raise ValueError("need 1 byte")
    return struct.unpack_from("<b", data)[0]


def unpack_u16(data: bytes) -> int:
    if len(data) < 2:
        raise ValueError("need 2 bytes")
    return struct.unpack_from("<H", data)[0]


def unpack_i16(data: bytes) -> int:
    if len(data) < 2:
        raise ValueError("need 2 bytes")
    return struct.unpack_from("<h", data)[0]


def unpack_u32(data: bytes) -> int:
    if len(data) < 4:
        raise ValueError("need 4 bytes")
    return struct.unpack_from("<I", data)[0]


def unpack_i32(data: bytes) -> int:
    if len(data) < 4:
        raise ValueError("need 4 bytes")
    return struct.unpack_from("<i", data)[0]


def pack_u16(value: int) -> bytes:
    return struct.pack("<H", value & 0xFFFF)


def pack_i8(value: int) -> bytes:
    return struct.pack("<b", value)


def pack_i32(value: int) -> bytes:
    return struct.pack("<i", value)


def pack_u32(value: int) -> bytes:
    return struct.pack("<I", value & 0xFFFFFFFF)


@dataclass
class PositionScaling:
    motor_resolution: int | None
    axis_resolution: int | None
    encoder_increments: int | None
    encoder_revolutions: int | None
    command_units_per_rev: float | None
    command_units_per_degree: float | None
    known: bool
    reason: str


def evaluate_scaling(
    motor_resolution: int | None,
    axis_resolution: int | None,
    encoder_increments: int | None = None,
    encoder_revolutions: int | None = None,
) -> PositionScaling:
    """Command units per revolution from 0x6091 and 0x608F only.

    The LC-E manual defines 0x6091:01/:02 as the electronic gear
    (encoder units = command units * gear). Both default to 1, which does
    not reveal encoder counts per revolution. 10000, 131072, and 8388608
    are never assumed.
    """
    if not motor_resolution or not axis_resolution:
        return PositionScaling(
            motor_resolution, axis_resolution, encoder_increments, encoder_revolutions,
            None, None, False,
            "0x6091 motor or axis resolution is missing or zero",
        )
    if not encoder_increments or not encoder_revolutions:
        return PositionScaling(
            motor_resolution, axis_resolution, encoder_increments, encoder_revolutions,
            None, None, False,
            "0x608F encoder increments per revolution were not read; "
            "refusing to assume a counts-per-revolution value",
        )
    gear = motor_resolution / axis_resolution
    units_per_rev = (encoder_increments / encoder_revolutions) / gear
    if units_per_rev <= 0:
        return PositionScaling(
            motor_resolution, axis_resolution, encoder_increments, encoder_revolutions,
            None, None, False, "computed command units per revolution is not positive",
        )
    return PositionScaling(
        motor_resolution, axis_resolution, encoder_increments, encoder_revolutions,
        units_per_rev, units_per_rev / 360.0, True,
        "derived from 0x608F and 0x6091",
    )


# Nameplate LCMT-01SLR23ZB-40M00330B. Manual section 2.3 defines R23 as a
# 23-bit absolute magnetic encoder. 2^23 is 8388608. This value is accepted
# only when the drive's own pulse registers contain the same number.
R23_COUNTS_PER_REV = 1 << 23
MOTOR_MODEL = "LCMT-01SLR23ZB-40M00330B"


def encoder_pulse_count(low: int, high: int) -> int:
    """Combine 0x2000:32 and 0x2000:33, the ESI encoder-pulse low and high words."""
    return ((int(high) & 0xFFFF) << 16) | (int(low) & 0xFFFF)


def confirm_r23_scale(
    motor_resolution: int | None,
    axis_resolution: int | None,
    pulse_low: int | None,
    pulse_high: int | None,
    internal_position: int | None,
    actual_position: int | None,
) -> PositionScaling:
    """Command units per revolution for this R23 motor, or unknown.

    The manual sets encoder units = command units × 0x6091 motor/axis.
    0x6063 is encoder units and 0x6064 is command units. 0x607A uses the
    0x6064 unit. A match requires the nameplate count, the drive registers,
    the gear, and the two position objects to agree.
    """
    if pulse_low is None or pulse_high is None:
        return evaluate_scaling(motor_resolution, axis_resolution, None, None)
    counts = encoder_pulse_count(pulse_low, pulse_high)
    if counts != R23_COUNTS_PER_REV:
        return PositionScaling(
            motor_resolution, axis_resolution, counts, 1,
            None, None, False,
            f"0x2000:32/33 is {counts}, not the R23 value {R23_COUNTS_PER_REV}",
        )
    if not motor_resolution or not axis_resolution:
        return PositionScaling(
            motor_resolution, axis_resolution, counts, 1,
            None, None, False,
            "0x6091 gear is missing, so encoder counts are not command units",
        )
    gear = motor_resolution / axis_resolution
    if internal_position is None or actual_position is None:
        return PositionScaling(
            motor_resolution, axis_resolution, counts, 1,
            None, None, False,
            "0x6063 or 0x6064 was not read",
        )
    expected_internal = actual_position * gear
    if abs(internal_position - expected_internal) > 1:
        return PositionScaling(
            motor_resolution, axis_resolution, counts, 1,
            None, None, False,
            "0x6063 and 0x6064 do not follow the 0x6091 gear",
        )
    units = counts / gear
    return PositionScaling(
        motor_resolution, axis_resolution, counts, 1,
        units, units / 360.0, True,
        "R23 nameplate 8388608 matches 0x2000:32/33; "
        "0x6064 and 0x607A are command units and 0x6063 follows 0x6091",
    )


def degrees_to_counts(degrees: float, units_per_rev: float) -> int:
    if units_per_rev <= 0:
        raise ValueError("command units per revolution must be positive")
    return int(round(degrees * units_per_rev / 360.0))


# Disable voltage, then shutdown. Neither word has enable-operation set.
SHUTDOWN_CONTROLWORDS = (0x0006, 0x0000)


def enables_operation(controlword: int) -> bool:
    """True only for the CiA402 enable-operation command, ignoring mode bits."""
    return (controlword & 0x008F) == 0x000F


def counts_to_degrees(counts: float, units_per_rev: float) -> float:
    if units_per_rev <= 0:
        raise ValueError("command units per revolution must be positive")
    return counts * 360.0 / units_per_rev
