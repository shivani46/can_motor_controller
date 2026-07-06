#!/usr/bin/env bash
# Bring up (or tear down) the vcan0 virtual CAN interface used by can_motor_controller.
# WSL2 does not persist network interfaces across restarts, so re-run this after reboot.
#
# Usage:
#   ./setup_vcan.sh up      # load vcan module and bring up vcan0 (default)
#   ./setup_vcan.sh down    # remove vcan0
#   ./setup_vcan.sh status  # show current state

set -euo pipefail

IFACE="vcan0"
ACTION="${1:-up}"

case "$ACTION" in
  up)
    if ip link show "$IFACE" &>/dev/null; then
      echo "$IFACE already exists:"
      ip -details link show "$IFACE"
      exit 0
    fi
    sudo modprobe vcan
    sudo ip link add dev "$IFACE" type vcan
    sudo ip link set up "$IFACE"
    echo "$IFACE is up:"
    ip -details link show "$IFACE"
    ;;
  down)
    if ip link show "$IFACE" &>/dev/null; then
      sudo ip link delete "$IFACE"
      echo "$IFACE removed"
    else
      echo "$IFACE does not exist"
    fi
    ;;
  status)
    ip -details link show "$IFACE" 2>&1 || echo "$IFACE does not exist"
    ;;
  *)
    echo "Usage: $0 {up|down|status}" >&2
    exit 1
    ;;
esac
