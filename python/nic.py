"""Claim only the dedicated EtherCAT NIC. Never touch Wi-Fi or the default route."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


def ensure_root() -> None:
    if os.geteuid() == 0:
        return
    script = str(Path(sys.argv[0]).resolve())
    os.execvp("pkexec", ["pkexec", sys.executable, script, *sys.argv[1:]])


def _run(args: list[str]) -> str:
    result = subprocess.run(args, check=False, text=True, capture_output=True)
    return (result.stdout or "") + (result.stderr or "")


def assert_dedicated_ethernet(iface: str) -> None:
    if iface.startswith(("wl", "ww", "tailscale", "docker", "veth", "br-", "vop-", "virbr")):
        raise RuntimeError(f"refusing to claim non-dedicated interface {iface}")
    if not iface.startswith(("en", "eth")):
        raise RuntimeError(f"refusing to claim {iface}")
    routes = _run(["ip", "-4", "route", "show", "default"])
    for line in routes.splitlines():
        parts = line.split()
        if "dev" in parts and parts[parts.index("dev") + 1] == iface:
            raise RuntimeError(f"default route uses {iface}; refusing to change it")


def claim(iface: str) -> bool:
    """Drop IPv4 on iface and stop NetworkManager from putting it back.

    Returns whether NetworkManager was managing the device.
    """
    assert_dedicated_ethernet(iface)
    managed = "yes" in _run(["nmcli", "-g", "GENERAL.NM-MANAGED", "device", "show", iface])
    _run(["ip", "link", "set", iface, "up"])
    _run(["ip", "-4", "addr", "flush", "dev", iface])
    _run(["nmcli", "device", "set", iface, "managed", "no"])
    return managed


def release(iface: str, was_managed: bool) -> None:
    if was_managed:
        _run(["nmcli", "device", "set", iface, "managed", "yes"])
