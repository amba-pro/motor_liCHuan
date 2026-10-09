#!/bin/sh
# Give raw-socket capability only to the named EtherCAT tools.
# The Qt window stays an ordinary user process and never receives these rights.
set -eu
BIN="${1:-}"
NAME="$(basename "$BIN")"
if [ -z "$BIN" ] || { [ "$NAME" != "lc_e_diag" ] && [ "$NAME" != "lc_e_csp_hold" ] && [ "$NAME" != "lc_e_csp_svc" ]; }; then
  echo "Usage: $0 /path/to/lc_e_diag|lc_e_csp_hold|lc_e_csp_svc" >&2
  exit 2
fi
if [ ! -x "$BIN" ]; then
  echo "Not an executable: $BIN" >&2
  exit 2
fi
if [ "$NAME" = "lc_e_csp_hold" ] || [ "$NAME" = "lc_e_csp_svc" ]; then
  # The cyclic tool also needs the rights its existing realtime setup already uses.
  exec pkexec setcap cap_net_raw,cap_net_admin,cap_sys_nice,cap_ipc_lock+ep "$BIN"
fi
exec pkexec setcap cap_net_raw,cap_net_admin+ep "$BIN"
