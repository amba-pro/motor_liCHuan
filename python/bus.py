"""Shared EtherCAT state requests. PREOP is required for SDO and does not enable the motor."""

from __future__ import annotations

import pysoem

from cia402 import ethercat_state_name


def request_preop(master, timeout_us: int = 5_000_000) -> None:
    master.read_state()
    if master.slaves and all((slave.state & 0x0F) == pysoem.PREOP_STATE for slave in master.slaves):
        return
    master.state = pysoem.PREOP_STATE
    master.write_state()
    master.state_check(pysoem.PREOP_STATE, timeout_us)
    master.read_state()


def describe_slave(slave, position: int) -> str:
    lines = [
        f"slave {position}:",
        f"  name: {_text(slave.name)}",
        f"  vendor_id: 0x{slave.man:08X}",
        f"  product_code: 0x{slave.id:08X}",
        f"  revision: 0x{slave.rev:08X}",
        f"  EtherCAT state: {ethercat_state_name(slave.state)} (0x{slave.state:02X})",
        f"  AL status: 0x{slave.al_status:04X} {pysoem.al_status_code_to_string(slave.al_status)}",
    ]
    return "\n".join(lines)


def _text(value) -> str:
    if isinstance(value, (bytes, bytearray)):
        return bytes(value).split(b"\x00", 1)[0].decode("utf-8", "replace")
    return str(value)
