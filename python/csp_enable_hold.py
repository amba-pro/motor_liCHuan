#!/usr/bin/env python3
"""Enable the servo and hold the current position. Does not command a move.

Requires the operator argument --confirm HOLD. Any other argument leaves the
servo disabled.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import config
import nic

BINARY = Path(__file__).resolve().parents[1] / "build" / "lc_e_csp_enable"


def main() -> int:
    if "--confirm" not in sys.argv or "HOLD" not in sys.argv:
        print("BLOCKED: missing operator confirmation HOLD")
        return 2
    nic.ensure_root()
    if not BINARY.is_file():
        print(f"BLOCKED: {BINARY} is not built")
        return 1
    iface = config.IFACE
    was_managed = False
    try:
        was_managed = nic.claim(iface)
        result = subprocess.run([str(BINARY), "--if", iface, "--confirm", "HOLD"], check=False)
        return result.returncode
    finally:
        nic.release(iface, was_managed)


if __name__ == "__main__":
    raise SystemExit(main())
