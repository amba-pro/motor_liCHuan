#!/usr/bin/env python3
"""Run the disabled 1 ms CSP-ready OP hold. Does not enable the servo."""

from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import config
import nic
from csp import CYCLE_NS, assess_periods

BINARY = Path(__file__).resolve().parents[1] / "build" / "lc_e_csp_hold"


def python_sleep_periods(cycles: int = 400) -> list[int]:
    """Show that a Python sleep loop is not the 1 ms EtherCAT scheduler."""
    period_s = CYCLE_NS / 1_000_000_000
    next_tick = time.perf_counter() + period_s
    periods = []
    previous = time.perf_counter()
    for _ in range(cycles):
        remaining = next_tick - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)
        now = time.perf_counter()
        periods.append(int(round((now - previous) * 1_000_000_000)))
        previous = now
        next_tick += period_s
    return periods


def main() -> int:
    nic.ensure_root()
    periods = python_sleep_periods()
    measured = assess_periods(periods)
    print(
        "PYTHON_SLEEP cycles "
        f"{measured['cycles']} min_ns {measured['min_ns']} max_ns {measured['max_ns']} "
        f"misses {measured['misses']}"
    )
    print("PYTHON_SLEEP accepted NO")
    if not BINARY.is_file():
        print(f"BLOCKED: {BINARY} is not built")
        return 1
    iface = config.IFACE
    was_managed = False
    try:
        was_managed = nic.claim(iface)
        result = subprocess.run([str(BINARY), "--if", iface, *sys.argv[1:]], check=False)
        return result.returncode
    finally:
        nic.release(iface, was_managed)


if __name__ == "__main__":
    raise SystemExit(main())
