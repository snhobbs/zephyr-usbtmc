# SPDX-License-Identifier: Apache-2.0
#
# Zephyr module CMakeLists for the USBTMC class driver.
#
# This file is called by the Zephyr build system when the module is present in
# the workspace (via ZEPHYR_EXTRA_MODULES or west.yml).  Do not invoke it
# directly.

zephyr_library_sources_ifdef(CONFIG_USBD_TMC src/usbd_tmc.c)

# Expose include/ to all targets so callers can use #include <usbd_tmc.h>
zephyr_include_directories_ifdef(CONFIG_USBD_TMC include)
