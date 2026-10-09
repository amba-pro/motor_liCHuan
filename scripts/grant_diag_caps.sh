#!/bin/sh
# Give raw-socket capability only to a read-only EtherCAT tool.
# The Qt window stays an ordinary user process.
set -eu
BIN="${1:-}"
NAME="$(basename "$BIN")"
if [ -z "$BIN" ] || { [ "$NAME" != "lc_e_diag" ] && [ "$NAME" != "lc_e_csp_hold" ]; }; then
  echo "Usage: $0 /path/to/lc_e_diag|lc_e_csp_hold" >&2
  exit 2
fi
if [ ! -x "$BIN" ]; then
  echo "Not an executable: $BIN" >&2
  exit 2
fi
if [ "$NAME" = "lc_e_csp_hold" ]; then
  # The cyclic tool also needs the rights its existing realtime setup already uses.
  exec pkexec setcap cap_net_raw,cap_net_admin,cap_sys_nice,cap_ipc_lock+ep "$BIN"
fi
exec pkexec setcap cap_net_raw,cap_net_admin+ep "$BIN"
