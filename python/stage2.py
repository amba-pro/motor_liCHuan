#!/usr/bin/env python3
"""Inspect SyncManagers and the active PDO, then stop before SAFEOP if the map is ambiguous.

This tool does not write CoE parameters, does not enable distributed clock,
and does not send a servo-enable control word.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import pysoem

import config
import nic
from bus import request_preop
from cia402 import decode_statusword, ethercat_state_name, evaluate_scaling
from pdo import SM_TYPE, assess_raw_pdo, read_raw_entries
from sdo import SdoClient

ENCODER_TYPE = {0: "incremental (P02.01=0)", 1: "absolute (P02.01=1)"}


def _text(value) -> str:
    if isinstance(value, (bytes, bytearray)):
        return bytes(value).split(b"\x00", 1)[0].decode("utf-8", "replace")
    return str(value)


def _u16(slave, index, sub):
    return int.from_bytes(bytes(slave.sdo_read(index, sub, 2))[:2], "little")


def _optional(slave, index, sub, size):
    try:
        return bytes(slave.sdo_read(index, sub, size)), None
    except Exception as exc:
        return None, getattr(exc, "desc", None) or str(exc)


def main() -> int:
    nic.ensure_root()
    iface = config.IFACE
    was_managed = False
    master = pysoem.Master()
    try:
        was_managed = nic.claim(iface)
        carrier = Path(f"/sys/class/net/{iface}/carrier").read_text().strip()
        print("LICHUAN ETHERCAT STAGE 2")
        print(f"interface: {iface}")
        print(f"carrier: {carrier}")
        if carrier != "1":
            print("BLOCKED: carrier is down")
            return 1
        master.open(iface)
        count = master.config_init()
        print(f"slaves: {count}")
        if count <= 0:
            print("BLOCKED: no slave")
            return 1
        request_preop(master)
        slave = master.slaves[0]
        initial = ethercat_state_name(slave.state)
        print(f"initial state: {initial} AL 0x{slave.al_status:04X}")
        print(f"slave: {_text(slave.name)} vendor 0x{slave.man:08X} product 0x{slave.id:08X} rev 0x{slave.rev:08X}")

        print("SyncManagers 0x1C00:")
        sm_count = bytes(slave.sdo_read(0x1C00, 0, 1))[0]
        for sub in range(1, sm_count + 1):
            kind = bytes(slave.sdo_read(0x1C00, sub, 1))[0]
            print(f"  SM{sub - 1}: type {kind} {SM_TYPE.get(kind, 'unknown')}")

        indexes = {obj.index for obj in slave.od}
        print(f"DC parameter object 0x1C32 present: {0x1C32 in indexes}")
        print(f"DC parameter object 0x1C33 present: {0x1C33 in indexes}")
        sync_type, sync_err = _optional(slave, 0x1C32, 1, 2)
        if sync_type is None:
            print(f"0x1C32:01 sync type: UNSUPPORTED ({sync_err})")
        else:
            print(f"0x1C32:01 sync type: {int.from_bytes(sync_type[:2], 'little')} (read only, DC not activated)")

        rx_assign = _u16(slave, 0x1C12, 1)
        tx_assign = _u16(slave, 0x1C13, 1)
        print(f"active RxPDO: 0x{rx_assign:04X}")
        print(f"active TxPDO: 0x{tx_assign:04X}")
        _rx_count, rx_raw = read_raw_entries(slave, rx_assign)
        _tx_count, tx_raw = read_raw_entries(slave, tx_assign)
        rx = assess_raw_pdo(rx_raw)
        tx = assess_raw_pdo(tx_raw)
        print("RxPDO records:")
        _print_records(rx)
        print("TxPDO records:")
        _print_records(tx)
        print(
            f"SOEM would size RxPDO at {rx['soem_bytes']} bytes and TxPDO at {tx['soem_bytes']} bytes"
        )

        mapped = master.config_map()
        master.read_state()
        print(
            f"config_map bytes: {mapped} output {len(slave.output)} input {len(slave.input)} "
            f"state still {ethercat_state_name(slave.state)}"
        )
        if len(slave.output) != rx["soem_bytes"] or len(slave.input) != tx["soem_bytes"]:
            print("NOTE: live SOEM sizes differ from the low-byte reconstruction")
        if not rx["safe"] or not tx["safe"]:
            print("SAFEOP/OP NOT REQUESTED")
            print(f"RxPDO: {rx['reason']}")
            print(f"TxPDO: {tx['reason'] or 'standard records, but exchange stays down because RxPDO is not safe'}")
        else:
            print("PDO assessed safe; this drive path is not taken in the captured map")

        client = SdoClient(slave)
        print("scaling objects:")
        for label, index, sub, reader in (
            ("0x6063", 0x6063, 0, client.read_i32),
            ("0x6064", 0x6064, 0, client.read_i32),
            ("0x6091:01", 0x6091, 1, client.read_u32),
            ("0x6091:02", 0x6091, 2, client.read_u32),
            ("0x608F:01", 0x608F, 1, client.read_u32),
            ("0x2002:02 encoder type", 0x2002, 2, client.read_u16),
            ("0x2005:12 P05.17 pulse division", 0x2005, 0x12, client.read_u16),
        ):
            value, err = client.optional(reader, index, sub)
            if err:
                print(f"  {label}: UNSUPPORTED ({err})")
            else:
                extra = ""
                if index == 0x2002 and sub == 2:
                    extra = " " + ENCODER_TYPE.get(value, "unlisted value")
                if index == 0x2005:
                    extra = " frequency-division output pulses/rev, not command units"
                print(f"  {label}: {value}{extra}")
        motor, _ = client.optional(client.read_u32, 0x6091, 1)
        axis, _ = client.optional(client.read_u32, 0x6091, 2)
        scaling = evaluate_scaling(motor, axis, None, None)
        print(f"scaling verified: {'YES' if scaling.known else 'NO'}")
        print(f"  {scaling.reason}")

        print("CiA402 while process data is not running:")
        for label, index, sub, reader in (
            ("0x603F", 0x603F, 0, client.read_u16),
            ("0x6041", 0x6041, 0, client.read_u16),
            ("0x6061", 0x6061, 0, client.read_i8),
            ("0x6064", 0x6064, 0, client.read_i32),
            ("0x606C", 0x606C, 0, client.read_i32),
            ("0x6077", 0x6077, 0, client.read_i16),
        ):
            value, err = client.optional(reader, index, sub)
            if err:
                print(f"  {label}: UNSUPPORTED ({err})")
            elif index == 0x6041:
                print(f"  {label}: 0x{value:04X} {decode_statusword(value)}")
            else:
                print(f"  {label}: {value}")
        print(f"final state: {ethercat_state_name(slave.state)}")
        print("motor enable: not sent")
        return 0 if not rx["safe"] else 0
    finally:
        try:
            master.state = pysoem.INIT_STATE
            master.write_state()
        except Exception:
            pass
        try:
            master.close()
        except Exception:
            pass
        nic.release(iface, was_managed)


def _print_records(report: dict) -> None:
    for record in report["records"]:
        if record.get("ambiguous"):
            print(f"  sub {record['sub']}: {record['length']} bytes {record['raw']} (not a 32-bit mapping record)")
        elif record.get("empty"):
            print(f"  sub {record['sub']}: empty")
        else:
            print(
                f"  sub {record['sub']}: 0x{record['index']:04X}:{record['subindex']:02X} "
                f"{record['bits']} bits"
            )
    print(f"  safe: {report['safe']}")


if __name__ == "__main__":
    raise SystemExit(main())
