#!/usr/bin/env python3
"""Discover EtherCAT slaves. Read-only: no controlword and no motor enable."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import pysoem

import config
import nic
from bus import describe_slave, request_preop


def main() -> int:
    nic.ensure_root()
    iface = config.IFACE
    was_managed = False
    master = pysoem.Master()
    try:
        was_managed = nic.claim(iface)
        carrier = Path(f"/sys/class/net/{iface}/carrier").read_text().strip()
        print("ETHERCAT_DISCOVERY")
        print(f"interface: {iface}")
        print(f"carrier: {carrier}")
        if carrier != "1":
            print("slaves detected: 0")
            print("BLOCKED: Ethernet carrier is down")
            return 1
        master.open(iface)
        count = master.config_init()
        print(f"slaves detected: {count}")
        if count <= 0:
            print("BLOCKED: no EtherCAT slave answered")
            print("Check drive control power and that the cable is in CN1 IN.")
            return 1
        request_preop(master)
        for position, slave in enumerate(master.slaves, start=1):
            print(describe_slave(slave, position))
        if any((slave.state & 0x0F) != pysoem.PREOP_STATE for slave in master.slaves):
            print("NOTE: slave did not reach PREOP, so SDO access may fail")
            return 1
        return 0
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
