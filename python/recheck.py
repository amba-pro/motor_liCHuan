#!/usr/bin/env python3
"""Re-read the LC-E after the panel alarm clear, then reach OP only with a verified disabled image.

No CoE parameter writes, no encoder reset, and no controlword that enables the drive.
"""

from __future__ import annotations

import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import pysoem

import config
import nic
from bus import request_preop
from cia402 import decode_statusword, enables_operation, ethercat_state_name
from esi_lc10e import (
    RX_CANDIDATES,
    TX_CANDIDATES,
    build_disabled_image,
    can_use_soem_buffer,
    match_widths,
    payload_bytes,
)
from pdo import assess_raw_pdo, read_raw_entries, soem_assumed_bits
from sdo import SdoClient

ECT_REG_SM0 = 0x0800
ECT_REG_FMMU0 = 0x0600
SM_ENABLE = 0x00010000
ER731 = 0x7305


def main() -> int:
    nic.ensure_root()
    iface = config.IFACE
    was_managed = False
    master = pysoem.Master()
    try:
        was_managed = nic.claim(iface)
        carrier = Path(f"/sys/class/net/{iface}/carrier").read_text().strip()
        print("LICHUAN RECHECK")
        print(f"interface: {iface}")
        print(f"carrier: {carrier}")
        if carrier != "1":
            print("BLOCKED: carrier is down")
            return 1
        master.open(iface)
        count = master.config_init()
        print(f"slaves: {count}")
        if count != 1:
            print("BLOCKED: expected one slave")
            return 1
        request_preop(master)
        slave = master.slaves[0]
        _report_identity(slave)
        _report_sync_managers(slave)
        client = SdoClient(slave)
        status = _report_cia402(client)
        rx_assign, tx_assign, rx_widths, tx_widths = _report_pdo(slave)
        rx_name = match_widths(rx_widths, RX_CANDIDATES)
        tx_name = match_widths(tx_widths, TX_CANDIDATES)
        print(f"Rx width match: {rx_name or 'NONE'}")
        print(f"Tx width match: {tx_name or 'NONE'}")
        reached = False
        if rx_name and tx_name and rx_assign == 0x1702 and tx_assign == 0x1B02:
            reached = _attempt_disabled_op(master, slave, client, rx_name, tx_name, status)
        else:
            print("OP NOT ATTEMPTED: active PDO widths do not match one known layout")
        master.read_state()
        print(f"final EtherCAT state: {ethercat_state_name(slave.state)} AL 0x{slave.al_status:04X}")
        print(f"OP reached: {'YES' if reached else 'NO'}")
        print("motor enable: not sent")
        return 0
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


def _report_identity(slave) -> None:
    name = slave.name
    if isinstance(name, (bytes, bytearray)):
        name = bytes(name).split(b"\x00", 1)[0].decode("utf-8", "replace")
    print(
        f"slave: {name} vendor 0x{slave.man:08X} product 0x{slave.id:08X} "
        f"rev 0x{slave.rev:08X}"
    )
    print(f"EtherCAT state: {ethercat_state_name(slave.state)} AL 0x{slave.al_status:04X} "
          f"{pysoem.al_status_code_to_string(slave.al_status)}")
    identity_ok = slave.man == 0x00000766 and slave.id == 0x00000402 and slave.rev == 0x00000204
    print(f"identity is LC10E V1.04 0x00000766/0x00000402/0x00000204: {'YES' if identity_ok else 'NO'}")


def _report_sync_managers(slave) -> None:
    print("SyncManager registers before config_map:")
    for number in range(4):
        raw = bytes(slave._fprd(ECT_REG_SM0 + 8 * number, 8))
        start, length, flags = struct.unpack_from("<HHI", raw)
        print(f"  SM{number}: addr 0x{start:04X} length {length} flags 0x{flags:08X}")


def _report_cia402(client: SdoClient) -> dict:
    print("CiA402 SDO:")
    values = {}
    for label, index, sub, reader in (
        ("0x603F", 0x603F, 0, client.read_u16),
        ("0x6041", 0x6041, 0, client.read_u16),
        ("0x6061", 0x6061, 0, client.read_i8),
        ("0x6063", 0x6063, 0, client.read_i32),
        ("0x6064", 0x6064, 0, client.read_i32),
        ("0x606C", 0x606C, 0, client.read_i32),
        ("0x60B8", 0x60B8, 0, client.read_u16),
        ("0x607F", 0x607F, 0, client.read_u32),
        ("0x6060", 0x6060, 0, client.read_i8),
        ("0x6071", 0x6071, 0, client.read_i16),
    ):
        value, err = client.optional(reader, index, sub)
        values[index] = value
        if err:
            print(f"  {label}: UNSUPPORTED ({err})")
        elif index == 0x6041:
            print(f"  {label}: 0x{value:04X} {decode_statusword(value)}")
        elif index == 0x603F:
            alarm = "Er.731 absolute-encoder alarm" if value == ER731 else "not Er.731"
            print(f"  {label}: 0x{value:04X} {alarm}")
        else:
            print(f"  {label}: {value}")
    if values.get(0x603F) == 0:
        print("encoder alarm 0x603F: CLEARED")
    elif values.get(0x603F) == ER731:
        print("encoder alarm 0x603F: STILL Er.731")
    else:
        print("encoder alarm 0x603F: not the previous Er.731 code")
    return values


def _report_pdo(slave) -> tuple[int, int, list[int], list[int]]:
    rx_assign = int.from_bytes(bytes(slave.sdo_read(0x1C12, 1, 2))[:2], "little")
    tx_assign = int.from_bytes(bytes(slave.sdo_read(0x1C13, 1, 2))[:2], "little")
    print(f"active RxPDO: 0x{rx_assign:04X}")
    print(f"active TxPDO: 0x{tx_assign:04X}")
    rx_widths = _print_pdo(slave, rx_assign, "Rx")
    tx_widths = _print_pdo(slave, tx_assign, "Tx")
    return rx_assign, tx_assign, rx_widths, tx_widths


def _print_pdo(slave, index: int, label: str) -> list[int]:
    count, raw_entries = read_raw_entries(slave, index)
    report = assess_raw_pdo(raw_entries)
    print(f"{label} 0x{index:04X} subindexes: {count}")
    print(f"  raw SDO safe: {report['safe']} ({report['reason'] or 'standard records'})")
    print(f"  SOEM low-byte size: {report['soem_bytes']} bytes")
    widths = []
    for obj in slave.od:
        if obj.index != index:
            continue
        for entry in obj.entries[1:]:
            widths.append(int(entry.bit_length))
            print(f"  OD sub bit length {entry.bit_length} name {entry.name!r}")
    print("  forced 4-byte reads:")
    for sub in range(1, count + 1):
        try:
            data = bytes(slave.sdo_read(index, sub, 4))
            print(f"    :{sub:02d} {len(data)} bytes {data.hex()}")
        except Exception as exc:
            print(f"    :{sub:02d} ERROR {getattr(exc, 'desc', exc)}")
    if not widths:
        widths = [len(item) * 8 for item in raw_entries]
        print("  OD widths unavailable; using returned SDO sizes")
    return widths


def _attempt_disabled_op(master, slave, client, rx_name: str, tx_name: str, status: dict) -> bool:
    rx_map = RX_CANDIDATES[rx_name]
    tx_map = TX_CANDIDATES[tx_name]
    rx_bytes = payload_bytes(rx_map)
    tx_bytes = payload_bytes(tx_map)
    _count, rx_raw = read_raw_entries(slave, 0x1702)
    _count, tx_raw = read_raw_entries(slave, 0x1B02)
    soem_rx = (soem_assumed_bits(rx_raw) + 7) // 8
    soem_tx = (soem_assumed_bits(tx_raw) + 7) // 8
    if not can_use_soem_buffer(rx_bytes, soem_rx) or not can_use_soem_buffer(tx_bytes, soem_tx):
        print(
            "OP NOT ATTEMPTED: verified PDO does not fit the buffer SOEM allocates "
            f"(Rx {rx_bytes}/{soem_rx} bytes, Tx {tx_bytes}/{soem_tx} bytes)"
        )
        return False
    echoes = {index: status[index] for index in (0x60B8, 0x607F, 0x6060)}
    if any(value is None for value in echoes.values()) or status.get(0x6064) is None:
        print("OP NOT ATTEMPTED: a value that must be echoed could not be read")
        return False
    image = build_disabled_image(rx_map, status[0x6064], echoes)
    if enables_operation(int.from_bytes(image[:2], "little")):
        print("OP NOT ATTEMPTED: image failed the disabled check")
        return False
    print(f"disabled image ({len(image)} bytes): {image.hex()}")
    mapped = master.config_map()
    master.read_state()
    print(
        f"config_map bytes {mapped} output {len(slave.output)} input {len(slave.input)} "
        f"state {ethercat_state_name(slave.state)}"
    )
    if len(slave.output) < rx_bytes or len(slave.input) < tx_bytes:
        print("OP NOT ATTEMPTED: config_map buffer is smaller than the verified PDO")
        return False
    if ethercat_state_name(slave.state) != "PREOP":
        print("OP NOT ATTEMPTED: config_map left PREOP")
        return False
    _shrink_channel(slave, 2, 0, rx_bytes, expected_type=2)
    _shrink_channel(slave, 3, 1, tx_bytes, expected_type=1)
    slave.output = image + bytes(len(slave.output) - len(image))
    if bytes(slave.output)[:2] != b"\x00\x00":
        print("OP NOT ATTEMPTED: output controlword is not zero")
        return False
    reached = _request_with_process_data(master, slave, pysoem.SAFEOP_STATE, "SAFEOP")
    if not reached:
        return False
    reached = _request_with_process_data(master, slave, pysoem.OP_STATE, "OP")
    if reached:
        _print_feedback(slave, tx_map, tx_bytes)
        after, _err = client.optional(client.read_u32, 0x607F, 0)
        print(f"0x607F after OP: {after} (before {status[0x607F]})")
    return reached


def _shrink_channel(slave, sm_number: int, fmmu_number: int, length: int, expected_type: int) -> None:
    sm_addr = ECT_REG_SM0 + 8 * sm_number
    raw = bytearray(slave._fprd(sm_addr, 8))
    start, current, flags = struct.unpack_from("<HHI", raw)
    if current < length:
        raise RuntimeError(f"SM{sm_number} is {current} bytes, shorter than verified {length}")
    struct.pack_into("<H", raw, 2, length)
    flags |= SM_ENABLE
    struct.pack_into("<I", raw, 4, flags)
    slave._fpwr(sm_addr, bytes(raw))
    fmmu_addr = ECT_REG_FMMU0 + 16 * fmmu_number
    fmmu = bytearray(slave._fprd(fmmu_addr, 16))
    log_length = struct.unpack_from("<H", fmmu, 4)[0]
    if fmmu[11] != expected_type or fmmu[12] != 1:
        raise RuntimeError(
            f"FMMU{fmmu_number} type {fmmu[11]} active {fmmu[12]} is not the expected channel"
        )
    if log_length < length:
        raise RuntimeError(f"FMMU{fmmu_number} length {log_length} is shorter than {length}")
    struct.pack_into("<H", fmmu, 4, length)
    fmmu[6] = 0
    fmmu[7] = 7
    slave._fpwr(fmmu_addr, bytes(fmmu))
    print(f"SM{sm_number} addr 0x{start:04X} length {current} -> {length}")


def _request_with_process_data(master, slave, state: int, label: str) -> bool:
    master.state = state
    master.write_state()
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        master.send_processdata()
        master.receive_processdata(2000)
        master.read_state()
        if (slave.state & 0x0F) == state and slave.al_status == 0:
            print(f"{label} reached, AL 0x{slave.al_status:04X}")
            return True
        time.sleep(0.01)
    master.read_state()
    print(
        f"{label} NOT reached: state {ethercat_state_name(slave.state)} "
        f"AL 0x{slave.al_status:04X} {pysoem.al_status_code_to_string(slave.al_status)}"
    )
    return False


def _print_feedback(slave, tx_map, tx_bytes: int) -> None:
    data = bytes(slave.input)[:tx_bytes]
    print(f"TxPDO ({len(data)} bytes): {data.hex()}")
    offset = 0
    for index, _sub, bits in tx_map:
        size = bits // 8
        field = data[offset:offset + size]
        if index in (0x603F, 0x6041):
            value = int.from_bytes(field, "little")
            extra = f" {decode_statusword(value)}" if index == 0x6041 else ""
            print(f"  0x{index:04X}: 0x{value:04X}{extra}")
        elif index in (0x6064, 0x606C, 0x6077):
            print(f"  0x{index:04X}: {int.from_bytes(field, 'little', signed=True)}")
        offset += size


if __name__ == "__main__":
    raise SystemExit(main())
