/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB Test and Measurement Class (USBTMC) — USB488 subclass
 * New USBD stack (CONFIG_USB_DEVICE_STACK_NEXT) driver.
 *
 * Spec references:
 *   USBTMC revision 1.0  (USB-IF, April 2003)
 *   USBTMC USB488 Subclass Specification revision 1.0  (USB-IF, April 2003)
 *
 * Architecture:
 *
 *   Host                    Bulk-OUT ──► request() cb
 *   DEV_DEP_MSG_OUT ──────────────────────────────────► rx_cb(data, len, eom)
 *   REQUEST_DEV_DEP_MSG_IN ────────────► pending state (btag, max_size)
 *                                          └──► req_sem
 *
 *   usbd_tmc_write() ─────► k_sem_take(req_sem) ──► format header ──► Bulk-IN
 *
 * The class is registered as a single static instance via USBD_DEFINE_CLASS
 * and is automatically included by usbd_register_all_classes().
 *
 * Only one USB TMC interface is defined (no IAD needed for a single interface).
 * Endpoints:
 *   Bulk-OUT  0x01  (host → device)  SCPI commands from host
 *   Bulk-IN   0x81  (device → host)  SCPI responses to host
 *   Interrupt-IN  0x82  (device → host, optional, not implemented)
 */

#include <usbd_tmc.h>

#include <errno.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(usbd_tmc, CONFIG_USBD_TMC_LOG_LEVEL);

/* ── USBTMC constants ────────────────────────────────────────────────────── */

/* MsgID values (USBTMC Table 1) */
#define TMC_MSG_DEV_DEP_MSG_OUT            0x01U
#define TMC_MSG_REQUEST_DEV_DEP_MSG_IN     0x02U
#define TMC_MSG_DEV_DEP_MSG_IN             0x02U
#define TMC_MSG_VENDOR_SPECIFIC_OUT        0x7EU
#define TMC_MSG_REQUEST_VENDOR_SPECIFIC_IN 0x7FU
#define TMC_MSG_VENDOR_SPECIFIC_IN         0x7FU
#define TMC_MSG_TRIGGER                    0x80U  /* USB488 only */

/* bmTransferAttributes flags */
#define TMC_ATTR_EOM                       BIT(0)
#define TMC_ATTR_TERM_CHAR_ENABLED         BIT(1)

/* Class-specific request codes (USBTMC Table 15) */
#define TMC_REQ_INITIATE_ABORT_BULK_OUT        0x01U
#define TMC_REQ_CHECK_ABORT_BULK_OUT_STATUS    0x02U
#define TMC_REQ_INITIATE_ABORT_BULK_IN         0x03U
#define TMC_REQ_CHECK_ABORT_BULK_IN_STATUS     0x04U
#define TMC_REQ_INITIATE_CLEAR                 0x05U
#define TMC_REQ_CHECK_CLEAR_STATUS             0x06U
#define TMC_REQ_GET_CAPABILITIES               0x07U
#define TMC_REQ_INDICATOR_PULSE                0x40U

/* USBTMC_STATUS values (USBTMC Table 16) */
#define TMC_STATUS_SUCCESS                     0x01U
#define TMC_STATUS_PENDING                     0x02U
#define TMC_STATUS_FAILED                      0x80U
#define TMC_STATUS_TRANSFER_NOT_IN_PROGRESS    0x81U
#define TMC_STATUS_SPLIT_NOT_IN_PROGRESS       0x82U
#define TMC_STATUS_SPLIT_IN_PROGRESS           0x83U

/* USBTMC bulk header size (bytes) */
#define TMC_HEADER_SIZE                        12U

/* Bulk transfer sizes */
#define TMC_BULK_EP_MPS  64U
#define TMC_HS_BULK_MPS  512U

/* Maximum payload bytes to allocate for a single Bulk-OUT receive buffer */
#define TMC_OUT_BUF_PAYLOAD CONFIG_USBD_TMC_RX_BUF_SIZE

/* Atomic state bits */
#define TMC_STATE_ENABLED  0
#define TMC_STATE_IN_BUSY  1

/* ── Descriptor layout ───────────────────────────────────────────────────── */

struct usbd_tmc_desc {
	struct usb_if_descriptor if0;
	struct usb_ep_descriptor if0_out_ep;
	struct usb_ep_descriptor if0_in_ep;
#if USBD_SUPPORTS_HIGH_SPEED
	struct usb_ep_descriptor if0_hs_out_ep;
	struct usb_ep_descriptor if0_hs_in_ep;
#endif
	struct usb_desc_header nil_desc;
};

/* ── Per-instance private data ───────────────────────────────────────────── */

struct tmc_data {
	struct usbd_tmc_desc *const desc;
	const struct usb_desc_header **const fs_desc;
	const struct usb_desc_header **const hs_desc;

	/* Pending REQUEST_DEV_DEP_MSG_IN state */
	struct k_sem req_sem;
	uint8_t req_btag;
	uint32_t req_max_size;

	/* bTag of the most recent Bulk-OUT / Bulk-IN transfer — echoed in
	 * INITIATE_ABORT responses (USBTMC Tables 19 and 25). */
	uint8_t last_out_btag;
	uint8_t last_in_btag;

	/* Application RX callback */
	usbd_tmc_rx_cb_t rx_cb;
	void *rx_cb_ctx;

	atomic_t state;
};

/* ── Helpers: get endpoint addresses ────────────────────────────────────── */

static uint8_t tmc_out_ep(struct tmc_data *d,
			  struct usbd_context *ctx)
{
#if USBD_SUPPORTS_HIGH_SPEED
	if (usbd_bus_speed(ctx) == USBD_SPEED_HS) {
		return d->desc->if0_hs_out_ep.bEndpointAddress;
	}
#else
	ARG_UNUSED(ctx);
#endif
	return d->desc->if0_out_ep.bEndpointAddress;
}

static uint8_t tmc_in_ep(struct tmc_data *d,
			 struct usbd_context *ctx)
{
#if USBD_SUPPORTS_HIGH_SPEED
	if (usbd_bus_speed(ctx) == USBD_SPEED_HS) {
		return d->desc->if0_hs_in_ep.bEndpointAddress;
	}
#else
	ARG_UNUSED(ctx);
#endif
	return d->desc->if0_in_ep.bEndpointAddress;
}

/* ── Arm next Bulk-OUT receive ───────────────────────────────────────────── */

static int tmc_arm_out(struct usbd_class_data *c_data)
{
	struct tmc_data *d = usbd_class_get_private(c_data);
	struct usbd_context *ctx = usbd_class_get_ctx(c_data);
	struct net_buf *buf;
	int err;

	if (!atomic_test_bit(&d->state, TMC_STATE_ENABLED)) {
		return -EPERM;
	}

	/* Allocate a buffer large enough for one full USBTMC message
	 * (12-byte header + up to TMC_RX_BUF_SIZE bytes of payload).
	 */
	buf = usbd_ep_buf_alloc(c_data, tmc_out_ep(d, ctx),
				TMC_HEADER_SIZE + TMC_OUT_BUF_PAYLOAD);
	if (buf == NULL) {
		LOG_ERR("tmc: failed to allocate OUT buf");
		return -ENOMEM;
	}

	err = usbd_ep_enqueue(c_data, buf);
	if (err) {
		LOG_ERR("tmc: OUT enqueue error %d", err);
		net_buf_unref(buf);
	}

	return err;
}

/* ── Parse and dispatch a Bulk-OUT transfer ─────────────────────────────── */

static void tmc_process_out(struct usbd_class_data *c_data,
			    struct net_buf *buf)
{
	struct tmc_data *d = usbd_class_get_private(c_data);

	if (buf->len < TMC_HEADER_SIZE) {
		LOG_WRN("tmc: short Bulk-OUT (%u bytes), ignoring", buf->len);
		return;
	}

	const uint8_t msg_id   = buf->data[0];
	const uint8_t btag     = buf->data[1];
	const uint8_t btag_inv = buf->data[2];

	if ((btag ^ btag_inv) != 0xFF) {
		LOG_WRN("tmc: bad bTag/~bTag (0x%02x/0x%02x)", btag, btag_inv);
		return;
	}

	if (msg_id == TMC_MSG_DEV_DEP_MSG_OUT) {
		/* Bytes 4-7: transfer size (LE); byte 8: attributes */
		uint32_t size = sys_get_le32(&buf->data[4]);
		bool eom = !!(buf->data[8] & TMC_ATTR_EOM);
		uint32_t avail = buf->len > TMC_HEADER_SIZE
				 ? (uint32_t)(buf->len - TMC_HEADER_SIZE) : 0U;
		uint32_t payload_len = MIN(size, avail);

		LOG_DBG("tmc: DEV_DEP_MSG_OUT bTag=%u size=%u eom=%d",
			btag, size, (int)eom);

		/* Track for INITIATE_ABORT_BULK_OUT response (Table 19) */
		d->last_out_btag = btag;

		if (payload_len > 0U && d->rx_cb != NULL) {
			d->rx_cb(&buf->data[TMC_HEADER_SIZE],
				 (size_t)payload_len, eom, d->rx_cb_ctx);
		}

	} else if (msg_id == TMC_MSG_REQUEST_DEV_DEP_MSG_IN) {
		/* Bytes 4-7: max transfer size; byte 8: attributes */
		uint32_t max_size = sys_get_le32(&buf->data[4]);

		LOG_DBG("tmc: REQUEST_DEV_DEP_MSG_IN bTag=%u max=%u",
			btag, max_size);

		/* Track for INITIATE_ABORT_BULK_IN response (Table 25) */
		d->last_in_btag = btag;
		d->req_btag     = btag;
		d->req_max_size = max_size;
		k_sem_give(&d->req_sem);

	} else {
		/* USBTMC §4.2.1: unknown MsgID — device must halt Bulk-OUT.
		 * (Only relevant when D0=0, i.e. "listen-only" not set, which
		 * is always the case here since we clear that capability bit.) */
		LOG_WRN("tmc: unknown MsgID 0x%02x, halting Bulk-OUT", msg_id);
		struct usbd_context *ctx = usbd_class_get_ctx(c_data);

		usbd_ep_set_halt(ctx, tmc_out_ep(d, ctx));
	}
}

/* ── USBD class API callbacks ────────────────────────────────────────────── */

static int tmc_request(struct usbd_class_data *c_data,
		       struct net_buf *buf, int err)
{
	struct tmc_data *d = usbd_class_get_private(c_data);
	struct usbd_context *ctx = usbd_class_get_ctx(c_data);
	const struct udc_buf_info *bi =
		(const struct udc_buf_info *)net_buf_user_data(buf);
	const uint8_t ep = bi->ep;

	if (err == -ECONNABORTED) {
		LOG_DBG("tmc: ep 0x%02x transfer aborted", ep);
		net_buf_unref(buf);
		return 0;
	}

	if (err != 0) {
		LOG_ERR("tmc: ep 0x%02x transfer error %d", ep, err);
		net_buf_unref(buf);
		return err;
	}

	if (ep == tmc_out_ep(d, ctx)) {
		/* Bulk-OUT completed — dispatch the message */
		tmc_process_out(c_data, buf);
		net_buf_unref(buf);
		/* Re-arm for the next message.  tmc_arm_out() returns -EPERM if
		 * disabled (expected at disconnect) or if the Bulk-OUT endpoint
		 * was just halted by tmc_process_out() for an unknown MsgID — the
		 * host will send CLEAR_FEATURE(HALT) which triggers feature_halt()
		 * to re-arm.  Suppress those two "expected" codes. */
		int arm_err = tmc_arm_out(c_data);

		if (arm_err && arm_err != -EPERM && arm_err != -ENOTSUP) {
			LOG_ERR("tmc: tmc_arm_out failed after OUT (%d)", arm_err);
		}

	} else {
		/* Bulk-IN completed */
		atomic_clear_bit(&d->state, TMC_STATE_IN_BUSY);
		LOG_DBG("tmc: Bulk-IN ep 0x%02x complete", ep);
		net_buf_unref(buf);
	}

	return 0;
}

/* USB488 subclass request code (Table 9) */
#define TMC_REQ_READ_STATUS_BYTE               0x80U

static int tmc_control_to_host(struct usbd_class_data *c_data,
				const struct usb_setup_packet *const setup,
				struct net_buf *const buf)
{
	struct tmc_data *d = usbd_class_get_private(c_data);
	struct usbd_context *ctx = usbd_class_get_ctx(c_data);
	const uint8_t req = setup->bRequest;

	LOG_DBG("tmc: control_to_host bRequest=0x%02x wIndex=0x%04x wValue=0x%04x",
		req, setup->wIndex, setup->wValue);

	switch (req) {
	case TMC_REQ_GET_CAPABILITIES: {
		/*
		 * USBTMC capabilities structure (24 bytes for USB488 subclass).
		 * See USBTMC Table 37 (base) + USB488 Table 8 (extension).
		 *
		 * We advertise the minimum honest capability set:
		 *   - No INDICATOR_PULSE (D2=0)
		 *   - Not talk-only (D1=0), not listen-only (D0=0) — base caps
		 *   - No TermChar (D0=0) — device cannot send TermChar
		 *   - No USB488.2 (D2=0), no REN_CONTROL (D1=0), no TRIGGER (D0=0)
		 *   - No DT1/RL1/SR1/SCPI bits — USB488 device caps all zero
		 *
		 * USB488 Table 8 rules: advertising DT1(D0) requires TRIGGER(D0),
		 * RL1(D1) requires REN_CONTROL(D1), SCPI(D3) requires SR1+488.2.
		 * Setting all to 0x00 avoids these mandatory requirements.
		 */
		static const uint8_t caps[24] = {
			TMC_STATUS_SUCCESS,   /* USBTMC_STATUS */
			0x00,                 /* Reserved */
			0x00, 0x01,           /* bcdUSBTMC = 1.00 (LE) */
			/* bmInterfaceCapabilities:
			 *   bit0=0 INDICATOR_PULSE not supported
			 *   bit1=0 not talk-only
			 *   bit2=0 not listen-only
			 */
			0x00,
			/* bmDeviceCapabilities:
			 *   bit0=0 device cannot send TermChar
			 *   bit1=0 TermChar not supported
			 */
			0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* Reserved */
			/* USB488 extension (bytes 12-23, USB488 Table 8): */
			0x00, 0x01,           /* bcdUSB488 = 1.00 (LE) */
			/* bmInterfaceCapabilities488:
			 *   bit0=0 TRIGGER not supported
			 *   bit1=0 REN_CONTROL/GO_TO_LOCAL/LOCAL_LOCKOUT not supported
			 *   bit2=0 not USB488.2 interface
			 */
			0x00,
			/* bmDeviceCapabilities488:
			 *   bit0=0 DT1 not set
			 *   bit1=0 RL1 not set
			 *   bit2=0 SR1 not set
			 *   bit3=0 SCPI not set
			 */
			0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		};

		size_t copy = MIN(setup->wLength, sizeof(caps));

		net_buf_add_mem(buf, caps, copy);
		return 0;
	}

	case TMC_REQ_INITIATE_CLEAR: {
		/*
		 * USBTMC §4.2.1.7 / Table 31: response is exactly 1 byte.
		 * The device MUST also halt the Bulk-OUT endpoint so the host
		 * knows no more data will be accepted until CLEAR_FEATURE clears
		 * it.  The host sends CHECK_CLEAR_STATUS polling until we say
		 * SUCCESS and bmClear.D0=0 (no partial packet pending).
		 */
		uint8_t resp[1] = {TMC_STATUS_SUCCESS};

		net_buf_add_mem(buf, resp, MIN(setup->wLength, sizeof(resp)));
		/* Halt Bulk-OUT (MUST per spec) */
		(void)usbd_ep_set_halt(ctx, tmc_out_ep(d, ctx));
		return 0;
	}

	case TMC_REQ_CHECK_CLEAR_STATUS: {
		/* USBTMC Table 34: 2 bytes — {status, bmClear}.
		 * bmClear D0=0: no partial Bulk-OUT packet is buffered. */
		static const uint8_t resp[2] = {TMC_STATUS_SUCCESS, 0x00};
		size_t copy = MIN(setup->wLength, sizeof(resp));

		net_buf_add_mem(buf, resp, copy);
		return 0;
	}

	case TMC_REQ_INITIATE_ABORT_BULK_OUT: {
		/* USBTMC Table 19: {USBTMC_status, bTag_of_current_transfer} */
		uint8_t resp[2] = {TMC_STATUS_SUCCESS, d->last_out_btag};
		size_t copy = MIN(setup->wLength, sizeof(resp));

		net_buf_add_mem(buf, resp, copy);
		return 0;
	}

	case TMC_REQ_INITIATE_ABORT_BULK_IN: {
		/* USBTMC Table 25: {USBTMC_status, bTag_of_current_IN_transfer} */
		uint8_t resp[2] = {TMC_STATUS_SUCCESS, d->last_in_btag};
		size_t copy = MIN(setup->wLength, sizeof(resp));

		net_buf_add_mem(buf, resp, copy);
		return 0;
	}

	case TMC_REQ_CHECK_ABORT_BULK_OUT_STATUS:
	case TMC_REQ_CHECK_ABORT_BULK_IN_STATUS: {
		/* USBTMC Table 22 / Table 28: 8 bytes — {status, rsvd[3], NBYTES[4]} */
		static const uint8_t resp[8] = {TMC_STATUS_SUCCESS, 0x00,
						0x00, 0x00, 0x00, 0x00,
						0x00, 0x00};
		size_t copy = MIN(setup->wLength, sizeof(resp));

		net_buf_add_mem(buf, resp, copy);
		return 0;
	}

	case TMC_REQ_READ_STATUS_BYTE: {
		/*
		 * USB488 §4.3.1 / Table 11-12: Required when no Interrupt-IN EP.
		 * wValue D6:D0 carries the host's bTag (2-127); we echo it back.
		 * Response: {USBTMC_status(1), bTag(1), StatusByte(1)}.
		 * RQS bit MUST be 0 when responding via EP0 (no Interrupt-IN).
		 * Return the IEEE 488.1 Status Byte with RQS cleared.
		 */
		uint8_t btag_echo = (uint8_t)(setup->wValue & 0x7FU);
		uint8_t resp[3] = {TMC_STATUS_SUCCESS, btag_echo, 0x00};
		size_t copy = MIN(setup->wLength, sizeof(resp));

		net_buf_add_mem(buf, resp, copy);
		return 0;
	}

	default:
		/* INDICATOR_PULSE (0x40): not advertised in capabilities (D2=0),
		 * so we MUST STALL per USBTMC §4.2.1.9.
		 * All other undefined requests also STALL (return -ENOTSUP causes
		 * the USBD stack to send a STALL handshake on EP0). */
		LOG_WRN("tmc: unsupported class request 0x%02x → STALL", req);
		return -ENOTSUP;
	}
}

static void *tmc_get_desc(struct usbd_class_data *const c_data,
			  const enum usbd_speed speed)
{
	struct tmc_data *d = usbd_class_get_private(c_data);

#if USBD_SUPPORTS_HIGH_SPEED
	if (speed == USBD_SPEED_HS) {
		return d->hs_desc;
	}
#else
	ARG_UNUSED(speed);
#endif
	return d->fs_desc;
}

static void tmc_enable(struct usbd_class_data *const c_data)
{
	struct tmc_data *d = usbd_class_get_private(c_data);

	LOG_INF("tmc: enabled");
	atomic_set_bit(&d->state, TMC_STATE_ENABLED);

	int err = tmc_arm_out(c_data);

	if (err) {
		LOG_ERR("tmc: initial tmc_arm_out failed (%d)", err);
	}
}

static void tmc_disable(struct usbd_class_data *const c_data)
{
	struct tmc_data *d = usbd_class_get_private(c_data);

	LOG_INF("tmc: disabled");
	atomic_clear_bit(&d->state, TMC_STATE_ENABLED);
	atomic_clear_bit(&d->state, TMC_STATE_IN_BUSY);
}

static int tmc_init(struct usbd_class_data *const c_data)
{
	LOG_DBG("tmc: init instance %p", c_data);
	return 0;
}

/*
 * feature_halt — called by the USBD stack when the host sends SET_FEATURE or
 * CLEAR_FEATURE for an endpoint halt (stall).
 *
 * After INITIATE_CLEAR halts Bulk-OUT, the host sends CLEAR_FEATURE(HALT) to
 * un-stall it once the device signals completion via CHECK_CLEAR_STATUS.
 * We must re-arm the OUT buffer so we can receive the next transfer.
 */
static void tmc_feature_halt(struct usbd_class_data *const c_data,
			     const uint8_t ep, const bool halted)
{
	struct tmc_data *d = usbd_class_get_private(c_data);
	struct usbd_context *ctx = usbd_class_get_ctx(c_data);

	LOG_DBG("tmc: feature_halt ep=0x%02x halted=%d", ep, (int)halted);

	if (!halted && ep == tmc_out_ep(d, ctx)) {
		/* Bulk-OUT was un-stalled — re-arm for the next transfer */
		int err = tmc_arm_out(c_data);

		if (err) {
			LOG_ERR("tmc: tmc_arm_out failed after halt clear (%d)", err);
		}
	}
}

static struct usbd_class_api tmc_api = {
	.feature_halt    = tmc_feature_halt,
	.control_to_host = tmc_control_to_host,
	.request         = tmc_request,
	.get_desc        = tmc_get_desc,
	.enable          = tmc_enable,
	.disable         = tmc_disable,
	.init            = tmc_init,
};

/* Forward-declared so usbd_tmc_write() can reference it before the
 * USBD_DEFINE_CLASS macro emits the full definition below. */
static struct usbd_class_data usbd_tmc_0;

/* ── Descriptor and class instantiation ─────────────────────────────────── */

static struct usbd_tmc_desc tmc_desc_0 = {
	.if0 = {
		.bLength            = sizeof(struct usb_if_descriptor),
		.bDescriptorType    = USB_DESC_INTERFACE,
		.bInterfaceNumber   = 0,
		.bAlternateSetting  = 0,
		.bNumEndpoints      = 2,
		/* USBTMC: bInterfaceClass=0xFE, SubClass=0x03, Protocol=0x01 */
		.bInterfaceClass    = USB_BCC_APPLICATION,
		.bInterfaceSubClass = 0x03,
		.bInterfaceProtocol = 0x01,  /* USB488 subclass */
		.iInterface         = 0,
	},
	.if0_out_ep = {
		.bLength          = sizeof(struct usb_ep_descriptor),
		.bDescriptorType  = USB_DESC_ENDPOINT,
		.bEndpointAddress = 0x01,   /* OUT, address reassigned by stack */
		.bmAttributes     = USB_EP_TYPE_BULK,
		.wMaxPacketSize   = sys_cpu_to_le16(TMC_BULK_EP_MPS),
		.bInterval        = 0x00,
	},
	.if0_in_ep = {
		.bLength          = sizeof(struct usb_ep_descriptor),
		.bDescriptorType  = USB_DESC_ENDPOINT,
		.bEndpointAddress = 0x81,   /* IN, address reassigned by stack */
		.bmAttributes     = USB_EP_TYPE_BULK,
		.wMaxPacketSize   = sys_cpu_to_le16(TMC_BULK_EP_MPS),
		.bInterval        = 0x00,
	},
#if USBD_SUPPORTS_HIGH_SPEED
	.if0_hs_out_ep = {
		.bLength          = sizeof(struct usb_ep_descriptor),
		.bDescriptorType  = USB_DESC_ENDPOINT,
		.bEndpointAddress = 0x01,
		.bmAttributes     = USB_EP_TYPE_BULK,
		.wMaxPacketSize   = sys_cpu_to_le16(TMC_HS_BULK_MPS),
		.bInterval        = 0x00,
	},
	.if0_hs_in_ep = {
		.bLength          = sizeof(struct usb_ep_descriptor),
		.bDescriptorType  = USB_DESC_ENDPOINT,
		.bEndpointAddress = 0x81,
		.bmAttributes     = USB_EP_TYPE_BULK,
		.wMaxPacketSize   = sys_cpu_to_le16(TMC_HS_BULK_MPS),
		.bInterval        = 0x00,
	},
#endif
	.nil_desc = {.bLength = 0, .bDescriptorType = 0},
};

static const struct usb_desc_header *tmc_fs_desc_0[] = {
	(struct usb_desc_header *)&tmc_desc_0.if0,
	(struct usb_desc_header *)&tmc_desc_0.if0_out_ep,
	(struct usb_desc_header *)&tmc_desc_0.if0_in_ep,
	(struct usb_desc_header *)&tmc_desc_0.nil_desc,
};

#if USBD_SUPPORTS_HIGH_SPEED
static const struct usb_desc_header *tmc_hs_desc_0[] = {
	(struct usb_desc_header *)&tmc_desc_0.if0,
	(struct usb_desc_header *)&tmc_desc_0.if0_hs_out_ep,
	(struct usb_desc_header *)&tmc_desc_0.if0_hs_in_ep,
	(struct usb_desc_header *)&tmc_desc_0.nil_desc,
};
#endif

static struct tmc_data tmc_data_0 = {
	.desc    = &tmc_desc_0,
	.fs_desc = tmc_fs_desc_0,
#if USBD_SUPPORTS_HIGH_SPEED
	.hs_desc = tmc_hs_desc_0,
#else
	.hs_desc = NULL,
#endif
	.rx_cb     = NULL,
	.rx_cb_ctx = NULL,
};

/* Initialise the semaphore at system init time. */
static int tmc_data_init(void)
{
	k_sem_init(&tmc_data_0.req_sem, 0, 1);
	return 0;
}
SYS_INIT(tmc_data_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

USBD_DEFINE_CLASS(usbd_tmc_0, &tmc_api, &tmc_data_0, NULL);

/* ── Public API ──────────────────────────────────────────────────────────── */

void usbd_tmc_set_rx_cb(usbd_tmc_rx_cb_t cb, void *ctx)
{
	tmc_data_0.rx_cb     = cb;
	tmc_data_0.rx_cb_ctx = ctx;
}

bool usbd_tmc_connected(void)
{
	return atomic_test_bit(&tmc_data_0.state, TMC_STATE_ENABLED);
}

int usbd_tmc_write(const uint8_t *data, size_t len, bool eom,
		   k_timeout_t timeout)
{
	struct tmc_data *d = &tmc_data_0;

	if (!usbd_tmc_connected()) {
		return -ENOTCONN;
	}

	/* Wait for a REQUEST_DEV_DEP_MSG_IN from the host */
	int ret = k_sem_take(&d->req_sem, timeout);

	if (ret != 0) {
		return -EAGAIN;
	}

	if (atomic_test_and_set_bit(&d->state, TMC_STATE_IN_BUSY)) {
		/* Another Bulk-IN is in flight */
		k_sem_give(&d->req_sem);  /* put it back */
		return -EBUSY;
	}

	struct usbd_class_data *c_data = &usbd_tmc_0;
	struct usbd_context *ctx = usbd_class_get_ctx(c_data);
	uint8_t ep = tmc_in_ep(d, ctx);

	/* Clamp to what the host is willing to receive */
	size_t send_len = MIN(len, (size_t)d->req_max_size);

	/* Allocate a net_buf for the Bulk-IN transfer (header + payload) */
	size_t total = TMC_HEADER_SIZE + send_len;
	struct net_buf *buf = usbd_ep_buf_alloc(c_data, ep, total);

	if (buf == NULL) {
		LOG_ERR("tmc: failed to allocate IN buf");
		atomic_clear_bit(&d->state, TMC_STATE_IN_BUSY);
		return -ENOMEM;
	}

	/* Build the 12-byte DEV_DEP_MSG_IN header */
	uint8_t hdr[TMC_HEADER_SIZE] = {
		TMC_MSG_DEV_DEP_MSG_IN,  /* MsgID */
		d->req_btag,             /* bTag (echo) */
		(uint8_t)~d->req_btag,  /* bTagInverse */
		0x00,                    /* Reserved */
		0x00, 0x00, 0x00, 0x00, /* TransferSize (LE) — filled below */
		eom ? TMC_ATTR_EOM : 0U, /* bmTransferAttributes */
		0x00, 0x00, 0x00,        /* Reserved */
	};
	sys_put_le32((uint32_t)send_len, &hdr[4]);

	net_buf_add_mem(buf, hdr, TMC_HEADER_SIZE);
	if (send_len > 0U) {
		net_buf_add_mem(buf, data, send_len);
	}

	/* USBTMC requires payload padded to 4-byte alignment */
	size_t pad = (4U - (send_len % 4U)) % 4U;

	for (size_t i = 0; i < pad; i++) {
		net_buf_add_u8(buf, 0x00);
	}

	ret = usbd_ep_enqueue(c_data, buf);
	if (ret != 0) {
		LOG_ERR("tmc: Bulk-IN enqueue failed (%d)", ret);
		net_buf_unref(buf);
		atomic_clear_bit(&d->state, TMC_STATE_IN_BUSY);
	}

	return ret;
}
