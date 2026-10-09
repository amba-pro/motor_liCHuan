#!/usr/bin/env python3
"""Read-only LC-E object dictionary dump. Does not enable the servo."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import pysoem

import config
import nic
from bus import request_preop
from cia402 import decode_statusword, ethercat_state_name, evaluate_scaling
from pdo import assigned_pdos, pdo_entries
from sdo import SdoClient


def _text(value) -> str:
    if isinstance(value, (bytes, bytearray)):
        return bytes(value).split(b"\x00", 1)[0].decode("utf-8", "replace")
    return str(value)


def _show(label: str, value, error: str | None, hex_width: int | None = None) -> None:
    if error or value is None:
        print(f"  {label}: UNSUPPORTED ({error})")
        return
    if hex_width and isinstance(value, int):
        print(f"  {label}: {value}  0x{value & ((1 << (hex_width * 4)) - 1):0{hex_width}X}")
    else:
        print(f"  {label}: {value}")


def _print_pdos(title: str, pdos: list[dict]) -> None:
    print(title)
    for pdo in pdos:
        if "error" in pdo and "pdo" not in pdo:
            print(f"  {pdo['error']}")
            continue
        print(f"  slot {pdo.get('slot')} PDO 0x{pdo.get('pdo', 0):04X}")
        for entry in pdo.get("entries", []):
            if "error" in entry:
                print(f"    sub {entry.get('sub', '?')}: {entry['error']}")
                continue
            if entry.get("empty"):
                print(f"    sub {entry['sub']}: empty")
                continue
            if "raw" in entry:
                print(f"    sub {entry['sub']}: {entry['length']} bytes {entry['raw']}")
                continue
            print(
                f"    0x{entry['index']:04X}:{entry['subindex']:02X} "
                f"{entry['bits']} bits at bit {entry['bit_offset']}"
            )


def main() -> int:
    nic.ensure_root()
    iface = config.IFACE
    was_managed = False
    master = pysoem.Master()
    try:
        was_managed = nic.claim(iface)
        carrier = Path(f"/sys/class/net/{iface}/carrier").read_text().strip()
        print(f"interface: {iface}  carrier: {carrier}")
        if carrier != "1":
            print("BLOCKED: carrier is down")
            return 1
        master.open(iface)
        if master.config_init() <= 0:
            print("BLOCKED: no slave")
            return 1
        request_preop(master)
        slave = master.slaves[0]
        print(f"slave: {_text(slave.name)}")
        print(f"vendor: 0x{slave.man:08X}  product: 0x{slave.id:08X}  revision: 0x{slave.rev:08X}")
        print(f"EtherCAT state: {ethercat_state_name(slave.state)}")
        client = SdoClient(slave)

        print("identity:")
        for index, sub, label, reader, width in (
            (0x1000, 0, "0x1000 device type", client.read_u32, 8),
            (0x1018, 1, "0x1018:01 vendor", client.read_u32, 8),
            (0x1018, 2, "0x1018:02 product", client.read_u32, 8),
            (0x1018, 3, "0x1018:03 revision", client.read_u32, 8),
            (0x1018, 4, "0x1018:04 serial", client.read_u32, 8),
        ):
            value, err = client.optional(reader, index, sub)
            _show(label, value, err, width)
        for index, label in ((0x1008, "0x1008 name"), (0x1009, "0x1009 hardware"), (0x100A, "0x100A software")):
            try:
                print(f"  {label}: {client.read_visible_string(index)}")
            except Exception as exc:
                print(f"  {label}: UNSUPPORTED ({exc})")

        print("drive:")
        error, error_err = client.optional(client.read_u16, 0x603F, 0)
        status, status_err = client.optional(client.read_u16, 0x6041, 0)
        mode, mode_err = client.optional(client.read_i8, 0x6061, 0)
        position, pos_err = client.optional(client.read_i32, 0x6064, 0)
        velocity, vel_err = client.optional(client.read_i32, 0x606C, 0)
        torque, torque_err = client.optional(client.read_i16, 0x6077, 0)
        digital, di_err = client.optional(client.read_u32, 0x60FD, 0)
        _show("0x603F error", error, error_err, 4)
        if status is None:
            print(f"  0x6041 status: UNSUPPORTED ({status_err})")
        else:
            print(f"  0x6041 status: {status}  0x{status:04X}  {decode_statusword(status)}")
        _show("0x6061 mode display", mode, mode_err)
        _show("0x6064 position", position, pos_err, 8)
        _show("0x606C velocity", velocity, vel_err, 8)
        if torque is None:
            print(f"  0x6077 torque: UNSUPPORTED ({torque_err})")
        else:
            print(f"  0x6077 torque: {torque} (0.1% rated)")
        _show("0x60FD digital inputs", digital, di_err, 8)

        print("scaling:")
        motor, motor_err = client.optional(client.read_u32, 0x6091, 1)
        axis, axis_err = client.optional(client.read_u32, 0x6091, 2)
        enc_inc, enc_inc_err = client.optional(client.read_u32, 0x608F, 1)
        enc_rev, enc_rev_err = client.optional(client.read_u32, 0x608F, 2)
        encoder_pos, enc_pos_err = client.optional(client.read_i32, 0x6063, 0)
        _show("0x6091:01 motor resolution", motor, motor_err)
        _show("0x6091:02 axis resolution", axis, axis_err)
        _show("0x608F:01 encoder increments", enc_inc, enc_inc_err)
        _show("0x608F:02 encoder revolutions", enc_rev, enc_rev_err)
        _show("0x6063 encoder position", encoder_pos, enc_pos_err, 8)
        save_policy, save_err = client.optional(client.read_u16, 0x200C, 0x0E)
        _show("0x200C:0E P0C.13 EEPROM store policy", save_policy, save_err)
        scaling = evaluate_scaling(motor, axis, enc_inc, enc_rev)
        if scaling.known:
            print(f"  command units/rev: {scaling.command_units_per_rev}")
            print(f"  command units/degree: {scaling.command_units_per_degree}")
        else:
            print("  POSITION_SCALING_UNKNOWN")
            print(f"  {scaling.reason}")

        print("watchdog:")
        for kind in ("pdi", "processdata"):
            try:
                print(f"  {kind}: {slave.get_watchdog(kind)} ms")
            except Exception as exc:
                print(f"  {kind}: UNSUPPORTED ({exc})")
        try:
            print(f"  max: {slave.get_max_watchdog_time()} ms")
        except Exception as exc:
            print(f"  max: UNSUPPORTED ({exc})")

        _print_pdos("RPDO assignment 0x1C12:", assigned_pdos(client, 0x1C12))
        _print_pdos("TPDO assignment 0x1C13:", assigned_pdos(client, 0x1C13))
        _print_pdos("Variable RPDO 0x1600 (present, not assigned):", [{"slot": 1, "pdo": 0x1600, "entries": pdo_entries(client, 0x1600)}])
        _print_pdos("Variable TPDO 0x1A00 (present, not assigned):", [{"slot": 1, "pdo": 0x1A00, "entries": pdo_entries(client, 0x1A00)}])
        print("read-only diagnostics finished; servo was not enabled")
        return 0 if status is not None else 1
    except Exception as exc:
        print(f"BLOCKED: {exc}")
        return 1
    finally:
        try:
            master.close()
        except Exception:
            pass
        nic.release(iface, was_managed)


if __name__ == "__main__":
    raise SystemExit(main())
