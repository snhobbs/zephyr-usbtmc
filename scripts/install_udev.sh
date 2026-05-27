#!/usr/bin/env bash
# install_udev.sh — Install udev rules for Zephyr Instrument USB access.
#
# After running this once (with sudo), the device is plug-and-play for
# any user logged in locally: pyvisa, python-usbtmc, and /dev/usbtmc* all
# work without extra permission setup.
#
# Usage:  sudo ./scripts/install_udev.sh

set -euo pipefail

RULES_SRC="$(dirname "$0")/99-zephyr-instrument.rules"
RULES_DST="/etc/udev/rules.d/99-zephyr-instrument.rules"

if [[ $EUID -ne 0 ]]; then
    echo "Run with sudo: sudo $0"
    exit 1
fi

cp -v "$RULES_SRC" "$RULES_DST"
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb --attr-match=idVendor=2e8a

echo ""
echo "Done. Unplug and replug the device — it will work for all local users."
echo "VISA resource name: USB0::11914::9::<serial>::8::INSTR"
