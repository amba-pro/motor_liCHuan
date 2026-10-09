"""Cyclic Synchronous Position helpers for the LC10E V1.04 PDO image.

The 15-byte RxPDO and 28-byte TxPDO match ESI LC10E_V1.04, product
0x00000402 revision 0x00000204. These functions pack and check that image.
They do not open a socket and they do not enable the servo.
"""

from __future__ import annotations

import struct

import config
from cia402 import enables_operation

CSP_MODE = 8
CYCLE_NS = 1_000_000
DEADLINE_LATE_NS = 250_000

# 0x1702: 6040, 607A, 60B8, 6071, 607F, 6060.
RX_IMAGE = struct.Struct("<HiHhIb")
# 0x1B02: 603F, 6041, 6064, 6077, 60F4, 60B9, 60BA, 60BC, 60FD.
TX_IMAGE = struct.Struct("<HHihiHiiI")

RX_BYTES = RX_IMAGE.size
TX_BYTES = TX_IMAGE.size


def pack_rx(controlword: int, target: int, probe: int, torque: int, max_velocity: int, mode: int) -> bytes:
    if enables_operation(controlword):
        raise RuntimeError("CSP image refuses the enable-operation controlword")
    if not 0 <= controlword <= 0xFFFF:
        raise ValueError("controlword is out of range")
    image = RX_IMAGE.pack(controlword, target, probe & 0xFFFF, torque, max_velocity & 0xFFFFFFFF, mode)
    if image[0] != 0 or image[1] != 0:
        raise RuntimeError("disabled CSP image must keep controlword 0")
    return image


def unpack_rx(image: bytes) -> dict:
    if len(image) != RX_BYTES:
        raise ValueError(f"RxPDO is {len(image)} bytes, not {RX_BYTES}")
    controlword, target, probe, torque, max_velocity, mode = RX_IMAGE.unpack(image)
    return {
        "controlword": controlword,
        "target": target,
        "probe": probe,
        "torque": torque,
        "max_velocity": max_velocity,
        "mode": mode,
    }


def unpack_tx(image: bytes) -> dict:
    if len(image) != TX_BYTES:
        raise ValueError(f"TxPDO is {len(image)} bytes, not {TX_BYTES}")
    error, status, position, torque, following, probe, probe1, probe2, inputs = TX_IMAGE.unpack(image)
    return {
        "error": error,
        "status": status,
        "position": position,
        "torque": torque,
        "following": following,
        "probe": probe,
        "probe1": probe1,
        "probe2": probe2,
        "inputs": inputs,
    }


def disabled_hold_image(actual: int, probe: int, torque: int, max_velocity: int, mode: int) -> bytes:
    """Constant target copied from feedback. Mode is echoed, not forced to CSP."""
    return pack_rx(0, actual, probe, torque, max_velocity, mode)


def future_csp_image(actual: int, probe: int, torque: int, max_velocity: int) -> bytes:
    """Offline image with mode 8. It is not a transmit permit."""
    if not config.MOTION_ARMED or not config.CSP_TIMING_ACCEPTED:
        raise RuntimeError("CSP mode 8 is not released for the drive")
    return pack_rx(0, actual, probe, torque, max_velocity, CSP_MODE)


def assess_periods(periods_ns: list[int], period_ns: int = CYCLE_NS, late_limit_ns: int = DEADLINE_LATE_NS) -> dict:
    if not periods_ns:
        raise ValueError("no cycle periods")
    misses = sum(1 for period in periods_ns if period > period_ns + late_limit_ns)
    return {
        "cycles": len(periods_ns),
        "min_ns": min(periods_ns),
        "max_ns": max(periods_ns),
        "misses": misses,
        "accepted": False,
    }
