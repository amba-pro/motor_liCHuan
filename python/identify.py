#!/usr/bin/env python3
"""Read motor identity and scaling objects. Does not write and does not enable."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import config
import nic
from bus import request_preop
from cia402 import (
    MOTOR_MODEL,
    R23_COUNTS_PER_REV,
    confirm_r23_scale,
    decode_statusword,
    degrees_to_counts,
    ethercat_state_name,
    evaluate_scaling,
)
from profile_position import profile_units
from sdo import SdoClient

# ESI MT_parameter subindexes. Values are read, never written.
MOTOR_OBJECTS = (
    (1, "MT_MotorModel"),
    (9, "MT_AbsEncType"),
    (31, "MT_EncoderSel"),
    (32, "MT_EncoderPensL"),
    (33, "MT_EncoderPensH"),
    (36, "MT_AbsRomMotorModel"),
)


def main() -> int:
    nic.ensure_root()
    import pysoem

    iface = config.IFACE
    was_managed = False
    master = pysoem.Master()
    try:
        was_managed = nic.claim(iface)
        master.open(iface)
        if master.config_init() != 1:
            print("BLOCKED: expected one slave")
            return 1
        request_preop(master)
        slave = master.slaves[0]
        client = SdoClient(slave)
        print("LICHUAN IDENTIFY")
        print(
            f"EtherCAT {ethercat_state_name(slave.state)} AL 0x{slave.al_status:04X} "
            f"vendor 0x{slave.man:08X} product 0x{slave.id:08X} rev 0x{slave.rev:08X}"
        )
        for index, label in ((0x1008, "0x1008"), (0x1009, "0x1009"), (0x100A, "0x100A")):
            try:
                print(f"{label}: {client.read_visible_string(index)}")
            except Exception as exc:
                print(f"{label}: {exc}")
        _print(client, "0x603F", 0x603F, 0, client.read_u16, hex_value=True)
        status, err = client.optional(client.read_u16, 0x6041, 0)
        if err:
            print(f"0x6041: {err}")
        else:
            print(f"0x6041: 0x{status:04X} {decode_statusword(status)}")
        for label, index, sub, reader in (
            ("0x6040", 0x6040, 0, client.read_u16),
            ("0x6060", 0x6060, 0, client.read_i8),
            ("0x607A", 0x607A, 0, client.read_i32),
            ("0x6071", 0x6071, 0, client.read_i16),
            ("0x607F", 0x607F, 0, client.read_u32),
            ("0x60B8", 0x60B8, 0, client.read_u16),
            ("0x6061", 0x6061, 0, client.read_i8),
            ("0x6063", 0x6063, 0, client.read_i32),
            ("0x6064", 0x6064, 0, client.read_i32),
            ("0x606C", 0x606C, 0, client.read_i32),
            ("0x6081", 0x6081, 0, client.read_u32),
            ("0x6083", 0x6083, 0, client.read_u32),
            ("0x6084", 0x6084, 0, client.read_u32),
            ("0x608F:01", 0x608F, 1, client.read_u32),
            ("0x608F:02", 0x608F, 2, client.read_u32),
            ("0x6091:01", 0x6091, 1, client.read_u32),
            ("0x6091:02", 0x6091, 2, client.read_u32),
            ("0x6092:01", 0x6092, 1, client.read_u32),
            ("0x2002:02 encoder type", 0x2002, 2, client.read_u16),
            ("0x2005:03 pulse/rev low", 0x2005, 3, client.read_u16),
            ("0x2005:04 pulse/rev high", 0x2005, 4, client.read_u16),
            ("0x2005:12 P05.17", 0x2005, 0x12, client.read_u16),
            ("0x6502", 0x6502, 0, client.read_u32),
        ):
            _print(client, label, index, sub, reader)
        print("0x2000 motor objects:")
        for sub, name in MOTOR_OBJECTS:
            try:
                raw = bytes(slave.sdo_read(0x2000, sub, 0))
            except Exception as exc:
                print(f"  :{sub:02d} {name}: {getattr(exc, 'desc', exc)}")
                continue
            value = int.from_bytes(raw, "little") if raw else None
            print(f"  :{sub:02d} {name}: {raw.hex()} value {value}")
        motor, _ = client.optional(client.read_u32, 0x6091, 1)
        axis, _ = client.optional(client.read_u32, 0x6091, 2)
        inc, _ = client.optional(client.read_u32, 0x608F, 1)
        rev, _ = client.optional(client.read_u32, 0x608F, 2)
        pos63, _ = client.optional(client.read_i32, 0x6063, 0)
        pos64, _ = client.optional(client.read_i32, 0x6064, 0)
        low, _ = client.optional(client.read_u16, 0x2000, 32)
        high, _ = client.optional(client.read_u16, 0x2000, 33)
        generic = evaluate_scaling(motor, axis, inc, rev)
        scaling = generic if generic.known else confirm_r23_scale(motor, axis, low, high, pos63, pos64)
        print(f"nameplate: {MOTOR_MODEL} R23 nominal {R23_COUNTS_PER_REV}")
        print(f"scaling known: {'YES' if scaling.known else 'NO'}")
        print(f"  {scaling.reason}")
        if scaling.known and scaling.command_units_per_rev:
            one = degrees_to_counts(1.0, scaling.command_units_per_rev)
            velocity, acceleration = profile_units(5.0, scaling.command_units_per_rev)
            print(f"command units per revolution: {scaling.command_units_per_rev}")
            print(f"1 degree is {one} command units")
            print(f"5 rpm is {velocity} command units/s")
            print(f"20 rpm/s is {acceleration} command units/s^2")
        print("motor enable: not sent")
        return 0
    finally:
        try:
            master.state = pysoem.INIT_STATE
            master.write_state()
            master.close()
        except Exception:
            pass
        nic.release(iface, was_managed)


def _print(client, label, index, sub, reader, hex_value=False) -> None:
    value, err = client.optional(reader, index, sub)
    if err:
        print(f"{label}: {err}")
    elif hex_value:
        print(f"{label}: 0x{value:04X}")
    else:
        print(f"{label}: {value}")


if __name__ == "__main__":
    raise SystemExit(main())
