# zephyr-usbtmc

A standalone Zephyr module that provides a USB Test and Measurement Class
(USBTMC 1.0 / USB488 subclass) device-side driver for the
[new Zephyr USBD stack](https://docs.zephyrproject.org/latest/connectivity/usb/device/usb_device.html)
(`CONFIG_USB_DEVICE_STACK_NEXT`).

## What it provides

| Symbol | Description |
|--------|-------------|
| `CONFIG_USBD_TMC` | Enable the class driver |
| `CONFIG_USBD_TMC_RX_BUF_SIZE` | Bulk-OUT receive buffer (bytes, default 512) |
| `CONFIG_USBD_TMC_LOG_LEVEL` | Log verbosity 0–4 (default 3 = info) |

```c
#include <usbd_tmc.h>

// Register an RX callback before USB starts
usbd_tmc_set_rx_cb(my_cb, NULL);

// In the callback:
void my_cb(const uint8_t *data, size_t len, bool eom, void *ctx) {
    // data is the raw SCPI/instrument command (header already stripped)
    // Call usbd_tmc_write() to respond
    usbd_tmc_write(response, resp_len, true, K_MSEC(5000));
}

// Check connection state
if (usbd_tmc_connected()) { ... }
```

The class is registered via `USBD_DEFINE_CLASS` and is automatically included
by `usbd_register_all_classes()` — no extra init call is needed.

## Installation

### Option A — ZEPHYR_EXTRA_MODULES (simplest)

Clone (or symlink) this repo alongside your application, then add one line to
your application's `CMakeLists.txt` **before** `find_package(Zephyr)`:

```cmake
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_LIST_DIR}/../zephyr-usbtmc)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
```

Or pass it on the command line:

```bash
west build -b rpi_pico -- -DZEPHYR_EXTRA_MODULES=/path/to/zephyr-usbtmc
```

### Option B — west manifest

Add to your `west.yml` under `projects:`:

```yaml
- name: usbtmc
  url: https://github.com/example/zephyr-usbtmc   # replace with real URL
  revision: main
  path: modules/usbtmc
```

Then `west update`.

### Option C — copy into Zephyr tree

Copy (or symlink) the repo into `<west-workspace>/modules/usbtmc/`.
Zephyr automatically discovers modules under `modules/`.

```bash
cp -r zephyr-usbtmc  <west-workspace>/modules/usbtmc
# or
ln -s $(pwd)/zephyr-usbtmc  <west-workspace>/modules/usbtmc
```

## Enabling in your project

In your application `prj.conf` (or board overlay):

```kconfig
CONFIG_USB_DEVICE_STACK_NEXT=y
CONFIG_USBD_TMC=y
CONFIG_USBD_TMC_RX_BUF_SIZE=512
```

## Host-side testing

`scripts/test_usbtmc.py` is a standalone Python test script that speaks raw
USBTMC 1.0 over PyUSB (no kernel driver required).

```bash
pip install pyusb
python3 scripts/test_usbtmc.py --list           # enumerate USBTMC devices
python3 scripts/test_usbtmc.py                  # auto-detect and run test suite
python3 scripts/test_usbtmc.py --interactive    # interactive SCPI shell
```

On Linux you need a one-time udev rule so a non-root user can access the
device. See your application's `scripts/install_udev.sh` or add:

```
SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", MODE="0666", TAG+="uaccess"
```

## Protocol notes

- Implements USBTMC 1.0 Table 1–37 and USB488 1.0 subclass.
- Supports: `GET_CAPABILITIES`, `INITIATE_CLEAR`, `CHECK_CLEAR_STATUS`,
  `INITIATE_ABORT_BULK_OUT/IN`, `CHECK_ABORT_*_STATUS`, `READ_STATUS_BYTE`.
- `INDICATOR_PULSE`, `TRIGGER`, `REN_CONTROL` not implemented (advertised
  as unsupported in capabilities).
- No Interrupt-IN endpoint; `READ_STATUS_BYTE` is answered on EP0.
- Requires `CONFIG_USB_DEVICE_STACK_NEXT` (Zephyr's new USBD stack).
  The legacy `CONFIG_USB_DEVICE_STACK` is not supported.
