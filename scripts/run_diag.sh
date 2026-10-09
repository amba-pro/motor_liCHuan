#!/bin/sh
# Read-only LC-E diagnostic. Does not enable the servo.
set -eu
ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
IF="${1:-enp37s0}"

if [ "$(id -u)" -ne 0 ]; then
  echo "Re-running with pkexec so a password dialog can authorize the raw socket."
  exec pkexec "$0" "$IF"
fi

# Leave Wi-Fi and the default route alone. Only the named Ethernet port is touched.
WAS_MANAGED=unknown
if [ -d "/sys/class/net/$IF" ]; then
  ip link set "$IF" up || true
  if ip -4 addr show dev "$IF" | grep -q 'inet '; then
    echo "Removing IPv4 addresses from $IF only."
    ip addr flush dev "$IF" || true
  fi
  if command -v nmcli >/dev/null 2>&1; then
    WAS_MANAGED="$(nmcli -g GENERAL.NM-MANAGED device show "$IF" 2>/dev/null || echo unknown)"
    nmcli device set "$IF" managed no || true
  fi
fi

set +e
"$ROOT/build/lc_e_diag" --if "$IF" --od-scan
STATUS=$?
set -e

if [ "$WAS_MANAGED" = "yes" ] && command -v nmcli >/dev/null 2>&1; then
  nmcli device set "$IF" managed yes || true
fi
exit "$STATUS"
