#!/bin/sh
# Give only the read-only diagnostic binary raw-socket capability.
# The Qt window stays an ordinary user process.
set -eu
BIN="${1:-}"
if [ -z "$BIN" ] || [ "$(basename "$BIN")" != "lc_e_diag" ]; then
  echo "Usage: $0 /path/to/lc_e_diag" >&2
  exit 2
fi
if [ ! -x "$BIN" ]; then
  echo "Not an executable: $BIN" >&2
  exit 2
fi
exec pkexec setcap cap_net_raw,cap_net_admin+ep "$BIN"
