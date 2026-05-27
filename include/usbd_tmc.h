/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB Test and Measurement Class (USBTMC) — USB488 subclass
 *
 * Implements the USBTMC 1.0 / USB488 1.0 class on the new Zephyr USBD stack
 * (CONFIG_USB_DEVICE_STACK_NEXT).  Exposes a lightweight callback API that
 * higher-level code (e.g. the SCPI USB-TMC backend) can use without dealing
 * with USB framing.
 *
 * Usage:
 *   1. Call usbd_tmc_set_rx_cb() before the USB stack starts to register a
 *      callback for incoming DEV_DEP_MSG_OUT payloads.
 *   2. When the host sends a query command followed by REQUEST_DEV_DEP_MSG_IN,
 *      call usbd_tmc_write() with the response data.  The function blocks
 *      until either the transfer is queued or a timeout occurs.
 */

#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Callback invoked (from the TMC work-queue) when the host sends a
 *        DEV_DEP_MSG_OUT bulk transfer.
 *
 * @param data  Pointer to the payload (USBTMC header already stripped).
 * @param len   Payload byte count.
 * @param eom   True when the EOM (End-Of-Message) flag was set in the header.
 * @param ctx   Opaque context pointer passed to usbd_tmc_set_rx_cb().
 */
typedef void (*usbd_tmc_rx_cb_t)(const uint8_t *data, size_t len,
				 bool eom, void *ctx);

/**
 * @brief Register a callback for received TMC messages.
 *
 * Must be called before the USB stack is enabled (e.g. from an application
 * init function or SYS_INIT level).  Only one callback is supported.
 *
 * @param cb   Callback function.
 * @param ctx  Context pointer forwarded to the callback.
 */
void usbd_tmc_set_rx_cb(usbd_tmc_rx_cb_t cb, void *ctx);

/**
 * @brief Send a response to the host.
 *
 * Queues a DEV_DEP_MSG_IN bulk-IN transfer.  The function waits up to
 * @p timeout for a pending REQUEST_DEV_DEP_MSG_IN from the host (instruments
 * only respond to explicit requests).
 *
 * @param data     Response payload (must remain valid until the function
 *                 returns).
 * @param len      Payload byte count.
 * @param eom      Set the EOM flag in the USBTMC header.
 * @param timeout  How long to wait for a pending REQUEST.
 *
 * @return 0 on success, -EAGAIN if no REQUEST arrived within the timeout,
 *         -ENOTCONN if USB is not currently configured, or other negative
 *         errno on transfer error.
 */
int usbd_tmc_write(const uint8_t *data, size_t len, bool eom,
		   k_timeout_t timeout);

/**
 * @brief Return true when the USB device is configured (enumerated by host).
 */
bool usbd_tmc_connected(void);

#ifdef __cplusplus
}
#endif
