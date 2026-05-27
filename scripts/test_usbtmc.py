# udev rules for Zephyr Instrument (VID 0x2E8A / PID 0x0009)
#
# Install:
#   sudo cp scripts/99-zephyr-instrument.rules /etc/udev/rules.d/
#   sudo udevadm control --reload-rules
#   sudo udevadm trigger
# Then unplug and replug the device.
#
# TAG+="uaccess" lets any user logged in locally access the device
# automatically — no group membership needed (requires systemd/logind).

# ── Generic rule: all USBTMC instruments ─────────────────────────────────────
# The Linux kernel usbtmc driver auto-binds to any USB class 0xFE/0x03
# device and creates /dev/usbtmc*.  Grant the logged-in user access to all
# such devices so this rule works for any instrument, not just this one.
KERNEL=="usbtmc*", SUBSYSTEM=="usbmisc", TAG+="uaccess", MODE="0666"

# ── VID/PID rule: raw USB access (pyvisa-py libusb backend) ──────────────────
# pyvisa-py can also bypass the kernel driver and talk to /dev/bus/usb/...
# directly via libusb.  This rule covers that path for this specific device.
SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0009", \
    TAG+="uaccess", MODE="0666"
