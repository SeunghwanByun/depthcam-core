#!/usr/bin/env bash
set -euo pipefail

RULE_FILE="/etc/udev/rules.d/99-orbbec-astra.rules"
RULE='SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="0401", MODE:="0666", GROUP:="video"'

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this script as root: sudo $0"
  exit 1
fi

printf '%s\n' "${RULE}" > "${RULE_FILE}"
udevadm control --reload-rules
udevadm trigger

echo "Installed ${RULE_FILE}"
echo "Reconnect the camera, then add your user to the video group if needed:"
echo "  sudo usermod -aG video ${SUDO_USER:-$USER}"
