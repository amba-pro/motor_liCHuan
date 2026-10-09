#!/usr/bin/env python3
"""Claim enp37s0 and run the verified disabled-image OP test.

The C++ tool re-checks the identity, PDO widths, and error code before
requesting SAFEOP. It does not write an SDO or EEPROM.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import config
import nic

BINARY = Path(__file__).resolve().parents[1] / "build" / "lc_e_op_disabled"


def main() -> int:
    nic.ensure_root()
    if not BINARY.is_file():
        print(f"BLOCKED: {BINARY} is not built")
        return 1
    iface = config.IFACE
    was_managed = False
    try:
        was_managed = nic.claim(iface)
        result = subprocess.run([str(BINARY), "--if", iface], check=False)
        return result.returncode
    finally:
        nic.release(iface, was_managed)


if __name__ == "__main__":
    raise SystemExit(main())
