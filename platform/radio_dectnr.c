/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 *   This file implements the OpenThread platform abstraction
 *   for radio communication.
 *
 */

#define LOG_MODULE_NAME net_otPlat_radio

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_OPENTHREAD_L2_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/net/net_pkt.h>
#include <openthread/ip6.h>
#include <openthread-system.h>
#include <openthread/instance.h>
#include <openthread/platform/radio.h>
#include <openthread/platform/diag.h>
#include <openthread/platform/time.h>
#include <openthread/message.h>

#include <openthread/thread.h>

#include <nrf_modem_dect_phy.h>
#include <modem/nrf_modem_lib.h>
#include <zephyr/drivers/hwinfo.h>

#include <zephyr/random/random.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/openthread.h>

#if defined(CONFIG_OPENTHREAD_NAT64_TRANSLATOR)
#include <openthread/nat64.h>
#endif

#define PKT_IS_IPv6(_p) ((NET_IPV6_HDR(_p)->vtc & 0xf0) == 0x60)

#if defined(CONFIG_NET_TC_THREAD_COOPERATIVE)
#define OT_WORKER_PRIORITY K_PRIO_COOP(CONFIG_OPENTHREAD_THREAD_PRIORITY)
#else
#define OT_WORKER_PRIORITY K_PRIO_PREEMPT(CONFIG_OPENTHREAD_THREAD_PRIORITY)
#endif

//sub GHz 915
//#define DECT_CARRIER 538
#define DECT_CARRIER			1677
#define DECT_TX_HANDLE			0
#define DECT_RX_HANDLE			1
#define DECT_TXRX_TX_HANDLE		2
#define DECT_TXRX_RX_HANDLE		3
#define DECT_HARQ_FEEDBACK_TX_HANDLE 4
#define DEFAULT_SENSITIVITY		-105
#define DECT_RSSI_TARGET		-80
#define DECT_802154_RX_BUFFERS	16
#define DECT_RX_STACK_SIZE		800
#define OPENTHREAD_MTU			1280
//#define OT_PHR_LENGTH			2
#define OT_PHR_LENGTH			1

#define TEST_NETWORK_ID 0x12345678

//HARQ parameters - refer to DECT shell HARQ example
#define HARQ_FEEDBACK_RX_DELAY_SUBSLOTS 2
#define HARQ_FEEDBACK_RX_SUBSLOTS       3

#define DECT_RADIO_FRAME_DURATION_US		(10000)
#define DECT_RADIO_FRAME_DURATION_MS		(10)
#define DECT_RADIO_SLOT_DURATION_US		((double)DECT_RADIO_FRAME_DURATION_US / 24)

#define US_TO_MODEM_TICKS(x) ((uint64_t)(((x) * NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ) / 1000))
#define DECT_RADIO_SLOT_DURATION_IN_MODEM_TICKS (US_TO_MODEM_TICKS(DECT_RADIO_SLOT_DURATION_US))
#define DECT_RADIO_SUBSLOT_DURATION_IN_MODEM_TICKS                                                 \
	((DECT_RADIO_SLOT_DURATION_IN_MODEM_TICKS) / 2) /* Note: assumes that mu = 1 */

//#define US_TO_MODEM_TICKS(x) ((uint64_t)(((x) * NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ) / 1000))

enum pending_events {
    PENDING_EVENT_FRAME_TO_SEND, /* There is a tx frame to send  */
    PENDING_EVENT_FRAME_RECEIVED, /* Radio has received new frame */
    PENDING_EVENT_RX_FAILED, /* The RX failed */
    PENDING_EVENT_TX_STARTED, /* Radio has started transmitting */
    PENDING_EVENT_TX_DONE, /* Radio transmission finished */
    PENDING_EVENT_DETECT_ENERGY, /* Requested to start Energy Detection procedure */
    PENDING_EVENT_DETECT_ENERGY_DONE, /* Energy Detection finished */
    PENDING_EVENT_SLEEP, /* Sleep if idle */
    PENDING_EVENT_COUNT /* Keep last */
};

typedef enum {
	DECT_PHY_HEADER_TYPE1 = 0,
	DECT_PHY_HEADER_TYPE2,
} dect_phy_header_type_t;

typedef enum {
	DECT_PHY_HEADER_FORMAT_000 = 0,
	DECT_PHY_HEADER_FORMAT_001 = 1,
} dect_phy_header_format;

enum dect_phy_packet_length_type {
	DECT_PHY_HEADER_PKT_LENGTH_TYPE_SUBSLOTS = 0,
	DECT_PHY_HEADER_PKT_LENGTH_TYPE_SLOTS = 1,
};


/* Header type 1, due to endianness the order is different than in the specification. */
struct dect_phy_header_type1_format0_t {
    uint32_t packet_length : 4;
    uint32_t packet_length_type : 1;
    uint32_t header_format : 3;
    uint32_t short_network_id : 8;
    uint32_t transmitter_id_hi : 8;
    uint32_t transmitter_id_lo : 8;
    uint32_t df_mcs : 3;
    uint32_t reserved : 1;
    uint32_t transmit_power : 4;
    uint32_t pad : 24;
};

typedef struct {
	uint8_t transmission_feedback0: 1;
	uint8_t harq_process_number0: 3;
	uint8_t format: 4;

	uint8_t CQI: 4;
	uint8_t buffer_status: 4;
} dect_phy_feedback_format_1_t;

typedef struct {
	uint8_t transmission_feedback0: 1;
	uint8_t harq_process_number0: 3;
	uint8_t format: 4;

	uint8_t CQI: 4;
	uint8_t transmission_feedback1: 1;
	uint8_t harq_process_number1: 3;
} dect_phy_feedback_format_3_t;

typedef struct {
	uint8_t harq_feedback_bitmap_proc3: 1;
	uint8_t harq_feedback_bitmap_proc2: 1;
	uint8_t harq_feedback_bitmap_proc1: 1;
	uint8_t harq_feedback_bitmap_proc0: 1;
	uint8_t format: 4;

	uint8_t CQI: 4;
	uint8_t harq_feedback_bitmap_proc7: 1;
	uint8_t harq_feedback_bitmap_proc6: 1;
	uint8_t harq_feedback_bitmap_proc5: 1;
	uint8_t harq_feedback_bitmap_proc4: 1;
} dect_phy_feedback_format_4_t;

typedef struct {
	uint8_t transmission_feedback: 1;
	uint8_t harq_process_number: 3;
	uint8_t format: 4;

	uint8_t codebook_index: 6;
	uint8_t mimo_feedback: 2;
} dect_phy_feedback_format_5_t;

typedef struct {
	uint8_t reserved: 1;
	uint8_t harq_process_number: 3;
	uint8_t format: 4;
	uint8_t CQI: 4;
	uint8_t buffer_status: 4;
} dect_phy_feedback_format_6_t;

typedef union {
	dect_phy_feedback_format_1_t format1;
	dect_phy_feedback_format_3_t format3;
	dect_phy_feedback_format_4_t format4;
	dect_phy_feedback_format_5_t format5;
	dect_phy_feedback_format_6_t format6;
} dect_phy_feedback_t;

struct dect_phy_header_type2_format0_t {
	uint8_t packet_length: 4;
	uint8_t packet_length_type: 1;
	uint8_t format: 3;

	uint8_t short_network_id;

	uint8_t transmitter_identity_hi;
	uint8_t transmitter_identity_lo;

	uint8_t df_mcs: 4;
	uint8_t transmit_power: 4;

	uint8_t receiver_identity_hi;
	uint8_t receiver_identity_lo;

	uint8_t df_harq_process_number: 3;
	uint8_t df_new_data_indication_toggle: 1;
	uint8_t df_redundancy_version: 2;
	uint8_t spatial_streams: 2;

	dect_phy_feedback_t feedback;
};

/* Table 6.2.1-2a: Physical Layer Control Field: Type 2, Header Format: 001 */
struct dect_phy_header_type2_format1_t {
	uint8_t packet_length: 4;
	uint8_t packet_length_type: 1;
	uint8_t format: 3;

	uint8_t short_network_id;

	uint8_t transmitter_identity_hi;
	uint8_t transmitter_identity_lo;

	uint8_t df_mcs: 4;
	uint8_t transmit_power: 4;

	uint8_t receiver_identity_hi;
	uint8_t receiver_identity_lo;

	uint8_t reserved: 6;
	uint8_t spatial_streams: 2;

	dect_phy_feedback_t feedback;
};

//K_SEM_DEFINE(radio_sem, 0, 1);

/* Semaphore to synchronize modem calls. */
K_SEM_DEFINE(operation_sem, 0, 1);

ATOMIC_DEFINE(pending_events, PENDING_EVENT_COUNT);

K_KERNEL_STACK_DEFINE(ot_task_stack,
              CONFIG_OPENTHREAD_RADIO_WORKQUEUE_STACK_SIZE);

K_FIFO_DEFINE(rx_pkt_fifo);
K_FIFO_DEFINE(tx_pkt_fifo);

static struct net_pkt *net_tx_pkt;
static struct net_buf *net_tx_buf;

static struct nrf_modem_dect_phy_tx_params harq_tx_params;
static struct dect_phy_header_type2_format1_t harp_header;
static bool waiting_for_ack;

/* TODO: enable OT_RADIO_CAPS_TRANSMIT_TIMING and use start_time to send frame in given time */
/* TODO: enable OT_RADIO_CAPS_SLEEP_TO_TX for SED */
/* TODO: disable OT_RADIO_CAPS_RX_ON_WHEN_IDLE for SED */
/* DECT radio supports ACK handing (HARQ) and retry */
//static otRadioCaps ot_radio_caps = OT_RADIO_CAPS_RX_ON_WHEN_IDLE | OT_RADIO_CAPS_ACK_TIMEOUT | OT_RADIO_CAPS_TRANSMIT_RETRIES | OT_RADIO_CAPS_SLEEP_TO_TX | OT_RADIO_CAPS_ENERGY_SCAN;
static otRadioCaps ot_radio_caps = OT_RADIO_CAPS_RX_ON_WHEN_IDLE | OT_RADIO_CAPS_ACK_TIMEOUT | OT_RADIO_CAPS_SLEEP_TO_TX | OT_RADIO_CAPS_ENERGY_SCAN;
static otRadioState ot_state = OT_RADIO_STATE_DISABLED;
static otRadioFrame ot_transmit_frame;
static otRadioFrame ot_ack_frame;
static otPanId        ot_pan_id;
static otExtAddress   ot_ext_addr;
static otShortAddress ot_short_addr;
static otError ot_tx_result;
static otError ot_rx_result;
static uint8_t ot_channel;

static int8_t last_rssi = INT8_MAX;
static int8_t tx_power = CONFIG_OPENTHREAD_DEFAULT_TX_POWER;

static uint16_t dect_device_id;
static enum nrf_modem_dect_phy_err dect_op_result;

static struct k_work_q ot_work_q;

static void transmit_message(struct k_work *tx_job);

static K_WORK_DEFINE(tx_job, transmit_message);
struct dect_802154_rx_frame {
    void *fifo_reserved; /* 1st word reserved for use by fifo. */
    uint64_t time; /* RX timestamp. */
    uint8_t lqi; /* Last received frame LQI value. */
    int8_t rssi; /* Last received frame RSSI value. */
    bool ack_fpb; /* FPB value in ACK sent for the received frame. */
    bool ack_seb; /* SEB value in ACK sent for the received frame. */
    uint8_t rx_psdu[OT_PHR_LENGTH + OT_RADIO_FRAME_MAX_SIZE];
};

struct dectnr_ot_l2_ctx {
    struct net_if *iface;
    uint8_t eui64[8];

    /* RX thread stack. */
    K_KERNEL_STACK_MEMBER(rx_stack, DECT_RX_STACK_SIZE);

    /* RX thread control block. */
    struct k_thread rx_thread;

    /* RX fifo queue. */
    struct k_fifo rx_fifo;

    /* Buffers for passing received frame pointers and data to the
     * RX thread via rx_fifo object.
     */
    struct dect_802154_rx_frame rx_frames[DECT_802154_RX_BUFFERS];

    /* Frame pending bit value in ACK sent for the last received frame. */
    bool last_frame_ack_fpb;

    /* Security Enabled bit value in ACK sent for the last received frame. */
    bool last_frame_ack_seb;
};

static struct dectnr_ot_l2_ctx dectnr_l2_ctx;

/* TX Buffer for DECT PHY transmition */
static uint8_t dect_tx_psdu[OT_PHR_LENGTH + OT_RADIO_FRAME_MAX_SIZE];

static inline bool is_pending_event_set(enum pending_events event)
{
    return atomic_test_bit(pending_events, event);
}

static void set_pending_event(enum pending_events event)
{
    atomic_set_bit(pending_events, event);
    otSysEventSignalPending();
}

static void reset_pending_event(enum pending_events event)
{
    atomic_clear_bit(pending_events, event);
}

static inline void clear_pending_events(void)
{
    atomic_clear(pending_events);
}

#if defined(CONFIG_NET_PKT_TXTIME) || defined(CONFIG_OPENTHREAD_CSL_RECEIVER)
/**
 * @brief Convert 32-bit (potentially wrapped) OpenThread microsecond timestamps
 * to 64-bit Zephyr network subsystem nanosecond timestamps.
 *
 * This is a workaround until OpenThread is able to schedule 64-bit RX/TX time.
 *
 * @param target_time_ns_wrapped time in nanoseconds referred to the radio clock
 * modulo UINT32_MAX.
 *
 * @return 64-bit nanosecond timestamp
 */
static net_time_t convert_32bit_us_wrapped_to_64bit_ns(uint32_t target_time_us_wrapped)
{
    /**
     * OpenThread provides target time as a (potentially wrapped) 32-bit
     * integer defining a moment in time in the microsecond domain.
     *
     * The target time can point to a moment in the future, but can be
     * overdue as well. In order to determine what's the case and correctly
     * set the absolute (non-wrapped) target time, it's necessary to compare
     * the least significant 32 bits of the current 64-bit network subsystem
     * time with the provided 32-bit target time. Let's assume that half of
     * the 32-bit range can be used for specifying target times in the
     * future, and the other half - in the past.
     */
    uint64_t now_us = otPlatTimeGet();
    uint32_t now_us_wrapped = (uint32_t)now_us;
    uint32_t time_diff = target_time_us_wrapped - now_us_wrapped;
    uint64_t result = UINT64_C(0);

    if (time_diff < 0x80000000) {
        /**
         * Target time is assumed to be in the future. Check if a 32-bit overflow
         * occurs between the current time and the target time.
         */
        if (now_us_wrapped > target_time_us_wrapped) {
            /**
             * Add a 32-bit overflow and replace the least significant 32 bits
             * with the provided target time.
             */
            result = now_us + UINT32_MAX + 1;
            result &= ~(uint64_t)UINT32_MAX;
            result |= target_time_us_wrapped;
        } else {
            /**
             * Leave the most significant 32 bits and replace the least significant
             * 32 bits with the provided target time.
             */
            result = (now_us & (~(uint64_t)UINT32_MAX)) | target_time_us_wrapped;
        }
    } else {
        /**
         * Target time is assumed to be in the past. Check if a 32-bit overflow
         * occurs between the target time and the current time.
         */
        if (now_us_wrapped > target_time_us_wrapped) {
            /**
             * Leave the most significant 32 bits and replace the least significant
             * 32 bits with the provided target time.
             */
            result = (now_us & (~(uint64_t)UINT32_MAX)) | target_time_us_wrapped;
        } else {
            /**
             * Subtract a 32-bit overflow and replace the least significant
             * 32 bits with the provided target time.
             */
            result = now_us - UINT32_MAX - 1;
            result &= ~(uint64_t)UINT32_MAX;
            result |= target_time_us_wrapped;
        }
    }

    __ASSERT_NO_MSG(result <= INT64_MAX / NSEC_PER_USEC);
    return (net_time_t)result * NSEC_PER_USEC;
}
#endif /* CONFIG_NET_PKT_TXTIME || CONFIG_OPENTHREAD_CSL_RECEIVER */

/* Callback after init operation. */
static void init(const uint64_t *time, int16_t temp, enum nrf_modem_dect_phy_err err,
      const struct nrf_modem_dect_phy_modem_cfg *cfg)
{
    if (err) {
        LOG_ERR("DECT init operation failed, err %d", err);
    }
    dect_op_result = err;
    k_sem_give(&operation_sem);
}

/* Callback after deinit operation. */
static void deinit(const uint64_t *time, enum nrf_modem_dect_phy_err err)
{
    if (err) {
        LOG_ERR("Deinit failed, err %d", err);
        return;
    }
    k_sem_give(&operation_sem);
}

/* Operation complete notification. */
static void op_complete(const uint64_t *time, int16_t temperature,
         enum nrf_modem_dect_phy_err err, uint32_t handle)
{
	//if (err != 0) {
		LOG_INF("op_complete cb time %"PRIu64" handle: %d status %X ot_state: %d", *time, handle, err, ot_state);
	//}
    if (handle == DECT_TX_HANDLE) {
		/*
		* OpenThread handles the following errors:
		* - OT_ERROR_NONE
		* - OT_ERROR_NO_ACK
		* - OT_ERROR_CHANNEL_ACCESS_FAILURE
		* - OT_ERROR_ABORT
		* Any other error passed to `otPlatRadioTxDone` will result in assertion.
		*/
		if (err == 0) {
			ot_tx_result = OT_ERROR_NONE;
		} else {
            LOG_INF("ot_cca failure");
			ot_tx_result = OT_ERROR_CHANNEL_ACCESS_FAILURE;
		}
		set_pending_event(PENDING_EVENT_TX_DONE);
    } else if (handle == DECT_TXRX_TX_HANDLE) {
		if (err == 0) {
			ot_tx_result = OT_ERROR_NONE;
		} else {
            LOG_INF("ot_cca failure");
			ot_tx_result = OT_ERROR_CHANNEL_ACCESS_FAILURE;
            //set_pending_event(PENDING_EVENT_TX_DONE);
		}
		//set_pending_event(PENDING_EVENT_TX_DONE);
    } else if (handle == DECT_RX_HANDLE) {
        if (err) {
            LOG_ERR("DECT Reception failed in op_complete, err %d", err);
        }
        //ot_state = OT_RADIO_STATE_RECEIVE;
    } else if (handle == DECT_TXRX_RX_HANDLE) {
        if (err) {
            LOG_ERR("DECT Ack Reception failed in op_complete, err %d", err);
        }
        if (waiting_for_ack == true) {
            if (ot_state == OT_RADIO_STATE_TRANSMIT) {
                //LOG_INF("Retransmit!");
                //k_work_submit_to_queue(&ot_work_q, &tx_job);
                ot_tx_result = OT_ERROR_CHANNEL_ACCESS_FAILURE;
                waiting_for_ack = false;
                set_pending_event(PENDING_EVENT_TX_DONE);
            }
        }
        //ot_state = OT_RADIO_STATE_RECEIVE;
    }
    else if (handle == DECT_HARQ_FEEDBACK_TX_HANDLE) {
#if 1
        err = dect_receive();
        if (err) {
            LOG_ERR("DECT Reception failed in op_complete, err %d", err);
        }
		ot_state = OT_RADIO_STATE_RECEIVE;
#endif
    }
}

/* Callback after receive stop operation. */
static void rx_stop(const uint64_t *time, enum nrf_modem_dect_phy_err err, uint32_t handle)
{
    LOG_INF("rx_stop cb time %"PRIu64" status %d handle %d", *time, err, handle);
    //k_sem_give(&radio_sem);
    if (ot_state == OT_RADIO_STATE_TRANSMIT){
        k_work_submit_to_queue(&ot_work_q, &tx_job);
    }
}

/* Physical Control Channel reception notification. */
static void pcc(
    const uint64_t *time,
    const struct nrf_modem_dect_phy_rx_pcc_status *status,
    const union nrf_modem_dect_phy_hdr *hdr)
{
    //struct phy_ctrl_field_common *header = (struct phy_ctrl_field_common *)hdr->type_1;
/*
    LOG_INF("Received header from device ID %d",
        header->transmitter_id_hi << 8 |  header->transmitter_id_lo);
*/
    LOG_INF("PCC");
	/* Provide HARQ feedback if requested */
	if (status->header_status == NRF_MODEM_DECT_PHY_HDR_STATUS_VALID &&
        status->phy_type == DECT_PHY_HEADER_TYPE2) {
        struct dect_phy_header_type2_format0_t *header = (void *)hdr;
        if (waiting_for_ack) {
            struct dect_phy_header_type2_format1_t *header2 = (void *)hdr;
            if (header2->format != DECT_PHY_HEADER_FORMAT_001) {
                LOG_INF("Don't handle Type 2 Format 000 header");
            }
            ot_tx_result = OT_ERROR_NONE;
            set_pending_event(PENDING_EVENT_TX_DONE);
            waiting_for_ack = false;
            return;
        }
        if (header->format != DECT_PHY_HEADER_FORMAT_000) {
            LOG_INF("Don't handle Type 2 Format 001 header");
            return;
        }
        const otIp6Address *address = otThreadGetLinkLocalIp6Address(openthread_get_default_instance());
        const otIp6Address *addressRLOC = otThreadGetRloc(openthread_get_default_instance());
        if (address == NULL) {
            LOG_ERR("fail to get ipv6 addr");
            return;
        }
        if (addressRLOC == NULL) {
            LOG_ERR("fail to get rloc addr");
        } else {
            LOG_INF("RLOC address  0x%02X%02X", addressRLOC->mFields.m8[14], addressRLOC->mFields.m8[15]);
        }
        //LOG_INF("Local address  0x%02X%02X", address->mFields.m8[14], address->mFields.m8[15]);
        if (((address->mFields.m8[14] == header->receiver_identity_hi) && (address->mFields.m8[15] == header->receiver_identity_lo)) ||
            ((addressRLOC->mFields.m8[14] == header->receiver_identity_hi) && (addressRLOC->mFields.m8[15] == header->receiver_identity_lo))) {
            struct dect_phy_header_type2_format1_t feedback_header;
			uint16_t len_slots = header->packet_length + 1;

			/* HARQ feedback requested */
			union nrf_modem_dect_phy_hdr phy_header;
            struct nrf_modem_dect_phy_tx_params harq_tx = harq_tx_params;

			feedback_header = harp_header;
			feedback_header.format = DECT_PHY_HEADER_FORMAT_001;
			feedback_header.df_mcs = 0;
			feedback_header.transmit_power = tx_power;
			feedback_header.receiver_identity_hi = header->transmitter_identity_hi;
			feedback_header.receiver_identity_lo = header->transmitter_identity_lo;
            feedback_header.transmitter_identity_hi = header->receiver_identity_hi;
            feedback_header.transmitter_identity_lo = header->receiver_identity_lo;
			feedback_header.spatial_streams = header->spatial_streams;
			feedback_header.feedback.format1.format = 1;
			feedback_header.feedback.format1.CQI = 1;
			feedback_header.feedback.format1.harq_process_number0 =
				header->df_harq_process_number;

			/* ACK/NACK: According to CRC: */
			feedback_header.feedback.format1.transmission_feedback0 = 0;
			feedback_header.feedback.format1.buffer_status =
				0; /* No meaning in our case */
			memcpy(&phy_header.type_2, &feedback_header, sizeof(phy_header.type_2));
			harq_tx.phy_header = &phy_header;
#if 0
			harq_feedback_start_time =
				p_rx_status->stf_start_time +
				(len_slots * DECT_RADIO_SLOT_DURATION_IN_MODEM_TICKS) +
				(current_settings->harq.harq_feedback_tx_delay_subslot_count *
				 DECT_RADIO_SUBSLOT_DURATION_IN_MODEM_TICKS);
#endif
			harq_tx.start_time = status->stf_start_time +
				(len_slots * DECT_RADIO_SUBSLOT_DURATION_IN_MODEM_TICKS) + 
                2 * DECT_RADIO_SUBSLOT_DURATION_IN_MODEM_TICKS;
            int err = nrf_modem_dect_phy_tx_harq(&harq_tx);
            if (err) {
                printk("nrf_modem_dect_phy_tx_harq() failed: %d\n", err);
            } else {
                ot_state = OT_RADIO_STATE_TRANSMIT;
            }
        }

    }
}

/* Physical Control Channel CRC error notification. */
static void pcc_crc_err(const uint64_t *time,
         const struct nrf_modem_dect_phy_rx_pcc_crc_failure *crc_failure)
{
    LOG_DBG("pcc_crc_err cb time %"PRIu64"", *time);
    ot_rx_result = OT_ERROR_FCS;
    set_pending_event(PENDING_EVENT_RX_FAILED);
}

static void dect_rx_thread(void *arg1, void *arg2, void *arg3)
{
    struct net_pkt *pkt;
    struct dect_802154_rx_frame *rx_frame;
    uint16_t pkt_len;
    //uint8_t *msdu;

    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    while (1) {
        pkt = NULL;
        rx_frame = NULL;

        rx_frame = k_fifo_get(&dectnr_l2_ctx.rx_fifo, K_FOREVER);

        pkt_len = rx_frame->rx_psdu[0];
        //pkt_len = *(uint16_t *)rx_frame->rx_psdu;

#if defined(CONFIG_NET_BUF_DATA_SIZE)
        __ASSERT_NO_MSG(pkt_len <= CONFIG_NET_BUF_DATA_SIZE);
#endif

        LOG_INF("Frame received: %d", pkt_len);

        /* Block the RX thread until net_pkt is available, so that we
         * don't drop already ACKed frame in case of temporary net_pkt
         * scarcity. The nRF 802154 radio driver will accumulate any
         * incoming frames until it runs out of internal buffers (and
         * thus stops acknowledging consecutive frames).
         */
        pkt = net_pkt_rx_alloc_with_buffer(dectnr_l2_ctx.iface, pkt_len,
                           AF_UNSPEC, 0, K_FOREVER);

        if (net_pkt_write(pkt, rx_frame->rx_psdu + OT_PHR_LENGTH, pkt_len)) {
            goto drop;
        }

        net_pkt_set_ieee802154_lqi(pkt, rx_frame->lqi);
        net_pkt_set_ieee802154_rssi_dbm(pkt, rx_frame->rssi);
        net_pkt_set_ieee802154_ack_fpb(pkt, rx_frame->ack_fpb);

#if defined(CONFIG_NET_PKT_TIMESTAMP)
        net_pkt_set_timestamp_ns(pkt, rx_frame->time * NSEC_PER_USEC);
#endif
        net_pkt_set_ieee802154_ack_seb(pkt, rx_frame->ack_seb);

        LOG_INF("Caught a packet (%u) (LQI: %u)",
             pkt_len, rx_frame->lqi);
            LOG_HEXDUMP_DBG(rx_frame->rx_psdu + OT_PHR_LENGTH, pkt_len, "rx frame:");

        if (net_recv_data(dectnr_l2_ctx.iface, pkt) < 0) {
            LOG_ERR("Packet dropped by NET stack");
            goto drop;
        }

        rx_frame->rx_psdu[0] = 0;
        //*(uint16_t *)rx_frame->rx_psdu = 0;
        continue;

drop:
        rx_frame->rx_psdu[0] = 0;
        //*(uint16_t *)rx_frame->rx_psdu = 0;

        net_pkt_unref(pkt);
    }
}

/* Physical Data Channel reception notification. */
static void pdc(const uint64_t *time,
        const struct nrf_modem_dect_phy_rx_pdc_status *status,
        const void *data, uint32_t len)
{
    //uint16_t psdu_len;
    uint8_t psdu_len;
    int8_t power, lqi;

    if(len == 0) {
        return;
    }
    /* Received RSSI value is in fixed precision format Q14.1 */
    LOG_INF("Received data (RSSI: %d.%d): %s",
        (status->rssi_2 / 2), (status->rssi_2 & 0b1) * 5, (char *)data);
    LOG_HEXDUMP_DBG(data, len, "pdc:");
    if (status->snr >= 127) {
        LOG_ERR("SNR Not known or not detectable.");
        return;
    }
    //psdu_len = *(uint16_t *)data;
    psdu_len = *(uint8_t *)data;
    power = (status->rssi_2) / 2;
    lqi = status->snr;
    for (uint32_t i = 0; i < ARRAY_SIZE(dectnr_l2_ctx.rx_frames); i++) {
        //if (dectnr_l2_ctx.rx_frames[i].msdu != NULL) {
        if (dectnr_l2_ctx.rx_frames[i].rx_psdu[0] != 0) {
        //if (*(uint16_t *)dectnr_l2_ctx.rx_frames[i].rx_psdu != 0 ) {
            continue;
        }

        //dectnr_l2_ctx.rx_frames[i].msdu = data;
        memcpy(dectnr_l2_ctx.rx_frames[i].rx_psdu, (uint8_t *)data, psdu_len + OT_PHR_LENGTH);
        dectnr_l2_ctx.rx_frames[i].rssi = power;
        last_rssi = power;
        dectnr_l2_ctx.rx_frames[i].lqi = lqi;

#if defined(CONFIG_NET_PKT_TIMESTAMP)
#if 0
        dectnr_l2_ctx.rx_frames[i].time =
            nrf_802154_timestamp_end_to_phr_convert(time, data[0]);
#else
        dectnr_l2_ctx.rx_frames[i].time = 0;
#endif
#endif

        dectnr_l2_ctx.rx_frames[i].ack_fpb = dectnr_l2_ctx.last_frame_ack_fpb;
        dectnr_l2_ctx.rx_frames[i].ack_seb = dectnr_l2_ctx.last_frame_ack_seb;
        dectnr_l2_ctx.last_frame_ack_fpb = false;
        dectnr_l2_ctx.last_frame_ack_seb = false;

        k_fifo_put(&dectnr_l2_ctx.rx_fifo, &dectnr_l2_ctx.rx_frames[i]);

        return;
    }
    LOG_ERR("Not enough rx frames allocated for 15.4 driver!");
}

/* Physical Data Channel CRC error notification. */
static void pdc_crc_err(
    const uint64_t *time, const struct nrf_modem_dect_phy_rx_pdc_crc_failure *crc_failure)
{
    LOG_DBG("pdc_crc_err cb time %"PRIu64"", *time);
    ot_rx_result = OT_ERROR_FCS;
    set_pending_event(PENDING_EVENT_RX_FAILED);
}

/* RSSI measurement result notification. */
static void rssi(const uint64_t *time, const struct nrf_modem_dect_phy_rssi_meas *status)
{
    LOG_DBG("rssi cb time %"PRIu64" carrier %d", *time, status->carrier);
}

/* Callback after link configuration operation. */
static void link_config(const uint64_t *time, enum nrf_modem_dect_phy_err err)
{
    LOG_DBG("link_config cb time %"PRIu64" status %d", *time, err);
}

/* Callback after time query operation. */
static void time_get(const uint64_t *time, enum nrf_modem_dect_phy_err err)
{
    LOG_DBG("time_get cb time %"PRIu64" status %d", *time, err);
}

/* Callback after capability get operation. */
static void capability_get(const uint64_t *time, enum nrf_modem_dect_phy_err err,
            const struct nrf_modem_dect_phy_capability *capability)
{
    LOG_DBG("capability_get cb time %"PRIu64" status %d", *time, err);
}

static void stf_cover_seq_control(const uint64_t *time, enum nrf_modem_dect_phy_err err)
{
    LOG_WRN("Unexpectedly in %s\n", (__func__));
}

/* Dect PHY callbacks. */
static struct nrf_modem_dect_phy_callbacks dect_phy_callbacks = {
    .init = init,
    .deinit = deinit,
    .op_complete = op_complete,
    .rx_stop = rx_stop,
    .pcc = pcc,
    .pcc_crc_err = pcc_crc_err,
    .pdc = pdc,
    .pdc_crc_err = pdc_crc_err,
    .rssi = rssi,
    .link_config = link_config,
    .time_get = time_get,
    .capability_get = capability_get,
    .stf_cover_seq_control = stf_cover_seq_control,
};

/* Dect PHY init parameters. */
static struct nrf_modem_dect_phy_init_params dect_phy_init_params = {
    .harq_rx_expiry_time_us = 5000000,
    .harq_rx_process_count = 4,
};

static void dataInit(void)
{
    net_tx_pkt = net_pkt_alloc(K_NO_WAIT);
    __ASSERT_NO_MSG(net_tx_pkt != NULL);

    net_tx_buf = net_pkt_get_reserve_tx_data(OT_RADIO_FRAME_MAX_SIZE,
                         K_NO_WAIT);
    __ASSERT_NO_MSG(net_tx_buf != NULL);

    net_pkt_append_buffer(net_tx_pkt, net_tx_buf);

    ot_transmit_frame.mPsdu = net_tx_buf->data;
}

/* Receive operation. */
int dect_receive(void)
{
    int err;

    struct nrf_modem_dect_phy_rx_params rx_op_params = {
        .start_time = 0,
        .handle = DECT_RX_HANDLE,
        .network_id = TEST_NETWORK_ID,
        .mode = NRF_MODEM_DECT_PHY_RX_MODE_SEMICONTINUOUS,
        .rssi_interval = NRF_MODEM_DECT_PHY_RSSI_INTERVAL_OFF,
        .link_id = NRF_MODEM_DECT_PHY_LINK_UNSPECIFIED,
        .rssi_level = DEFAULT_SENSITIVITY,
        .carrier = DECT_CARRIER,
        .duration = UINT32_MAX,
        .filter.short_network_id = (uint8_t)(TEST_NETWORK_ID & 0xff),
        .filter.is_short_network_id_used = 1,
        /* listen for everything (broadcast mode used) */
        .filter.receiver_identity = 0,
    };
    err = nrf_modem_dect_phy_rx(&rx_op_params);
    if (err != 0) {
        return err;
    }

    return 0;
}

void platformRadioInit(void)
{
    dataInit();
    k_work_queue_start(&ot_work_q, ot_task_stack,
               K_KERNEL_STACK_SIZEOF(ot_task_stack),
               OT_WORKER_PRIORITY, NULL);
    k_thread_name_set(&ot_work_q.thread, "ot_radio_workq");

    k_fifo_init(&dectnr_l2_ctx.rx_fifo);

    k_thread_create(&dectnr_l2_ctx.rx_thread, dectnr_l2_ctx.rx_stack,
            DECT_RX_STACK_SIZE,
            dect_rx_thread, NULL, NULL, NULL,
            K_PRIO_COOP(2), 0, K_NO_WAIT);
}

static const int16_t byte_per_mcs_and_length[5][16] = {
    { 0,  17,  33,  50,  67,  83,  99, 115, 133, 149, 165, 181, 197, 213, 233, 249},
    { 4,  37,  69, 103, 137, 169, 201, 233, 263, 295, 327, 359, 391, 423, 463, 495},
    { 7,  57, 107, 157, 205, 253, 295, 343, 399, 447, 495, 540, 596, 644, 692,  -1},
    {11,  77, 141, 209, 271, 335, 399, 463, 532, 596, 660,  -1,  -1,  -1,  -1,  -1},
    {18, 117, 217, 311, 407, 503, 604, 700,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1}};

void dect_mac_utils_get_packet_length(size_t *data_size, uint32_t *mcs, uint32_t *packet_length)
{
    for (*packet_length = 0; *packet_length < 16; (*packet_length)++)
    {
        if (byte_per_mcs_and_length[*mcs][*packet_length] == -1) {
            packet_length--;
            break;
        }
        if (byte_per_mcs_and_length[*mcs][*packet_length] >= (int16_t)*data_size)
        {
            break;
        }
    }
}

/** Frame Control Field, see section 7.2.2 */
struct ieee802154_fcf_seq {
	struct {
#ifdef CONFIG_LITTLE_ENDIAN
		uint16_t frame_type : 3;
		uint16_t security_enabled : 1;
		uint16_t frame_pending : 1;
		uint16_t ar : 1;
		uint16_t pan_id_comp : 1;
		uint16_t reserved : 1;
		uint16_t seq_num_suppr : 1;
		uint16_t ie_list : 1;
		uint16_t dst_addr_mode : 2;
		uint16_t frame_version : 2;
		uint16_t src_addr_mode : 2;
#else
		uint16_t reserved : 1;
		uint16_t pan_id_comp : 1;
		uint16_t ar : 1;
		uint16_t frame_pending : 1;
		uint16_t security_enabled : 1;
		uint16_t frame_type : 3;
		uint16_t src_addr_mode : 2;
		uint16_t frame_version : 2;
		uint16_t dst_addr_mode : 2;
		uint16_t ie_list : 1;
		uint16_t seq_num_suppr : 1;
#endif
	} fc __packed;

	uint8_t sequence;
} __packed;

/*
 * convert_ot_addr_to_dect_txrx_id - Convert OpenThread address to DECT TX/RX ID 
 */
bool convert_ot_addr_to_dect_txrx_id(const uint8_t *msdu, size_t psduLength, bool *ackRequest, uint8_t *txIdHigh, uint8_t *txIdLow, uint8_t *rxIdHigh, uint8_t *rxIdLow)
{
    uint8_t offset = 0;
    uint64_t srcExtAddress = 0, dstExtAddress = 0;
    uint16_t srcShortAddress = 0, dstShortAddress = 0;

    // Validate the PSDU length and structure
    if (psduLength < 3) // A valid MAC frame should have at least 3 bytes (Frame Control, Sequence Number, and Address)
    {
        LOG_ERR("Invalid PSDU length");
        return false;
    }
    // Offset 2 bytes frame control field and 1 byte sequence field
    offset += 3;
    // Step 1: Decode the MAC frame header
    struct ieee802154_fcf_seq *fs = (struct ieee802154_fcf_seq *)msdu;

    LOG_INF("Frame type: %u AR: %d dst/src addr mode: %u %u", fs->fc.frame_type, fs->fc.ar, fs->fc.dst_addr_mode, fs->fc.src_addr_mode);
    if (fs->fc.frame_type != 0x1 && fs->fc.frame_type != 0x3) {
        LOG_INF("Unsupported frame type");
        return false;
    }
    *ackRequest = fs->fc.ar;

    // Step 2: Extract the source address from the frame header
    if ((fs->fc.dst_addr_mode == 0x03) || (fs->fc.dst_addr_mode == 0x02)) {
        // Offset 2 bytes DST PAN ID field
        offset += 2;
    }

    if (fs->fc.dst_addr_mode == 0x03) // Extended Addressing Mode
    {
        if (offset + 8 > psduLength) {
            LOG_INF("Extended dst address parse fail");
            return false;
        }
        // Extended address is 8 bytes long
        for (int i = 0; i < 8; i++)
        {
            dstExtAddress |= ((uint64_t)msdu[offset + i]) << (i * 8);
        }
        *rxIdHigh = msdu[offset + 1];
        *rxIdLow = msdu[offset];
        offset += 8;
        LOG_INF("Destination Address (Extended): 0x%016llX", dstExtAddress);
    }
    else if (fs->fc.dst_addr_mode == 0x02) // Short Addressing Mode
    {
        if (offset + 2 > psduLength) {
            LOG_INF("Short dst address parse fail");
            return false;
        }
        // Short address is 2 bytes long
        dstShortAddress = (msdu[offset + 1] << 8) | msdu[offset];
        *rxIdHigh = msdu[offset + 1];
        *rxIdLow = msdu[offset];
        offset += 2;
        LOG_INF("Destination Address (Short): 0x%04X", dstShortAddress);
    }
    else
    {
        LOG_INF("Unsupported dst addressing mode: %d", fs->fc.dst_addr_mode);
        return false;
    }

    if (!fs->fc.pan_id_comp) {
        offset += 2;
    }

    if (fs->fc.src_addr_mode == 0x03) // Extended Addressing Mode
    {
        if (offset + 8 > psduLength) {
            LOG_INF("Extended dst address parse fail");
            return false;
        }
        // Extended address is 8 bytes long
        for (int i = 0; i < 8; i++)
        {
            srcExtAddress |= ((uint64_t)msdu[offset + i]) << (i * 8);
        }
        *txIdHigh = msdu[offset + 1];
        *txIdLow = msdu[offset];
        LOG_INF("Source Address (Extended): 0x%016llX", srcExtAddress);
    }
    else if (fs->fc.src_addr_mode == 0x02) // Short Addressing Mode
    {
        if (offset + 2 > psduLength) {
            LOG_INF("Short dst address parse fail");
            return false;
        }
        // Short address is 2 bytes long
        srcShortAddress = (msdu[offset + 1] << 8) | msdu[offset];
        *txIdHigh = msdu[offset + 1];
        *txIdLow = msdu[offset];
        LOG_INF("Source Address (Short): 0x%04X", srcShortAddress);
    }
    else
    {
        LOG_INF("Unsupported src addressing mode: %d", fs->fc.src_addr_mode);
        return false;
    }
    return true;
}

/* Send operation. */
int dect_transmit(uint32_t lbt_period, void *data, size_t data_size, uint64_t start_time)
{
    int err;
    uint32_t df_mcs = 4;
    uint32_t packet_length = 0;
    struct dect_phy_header_type1_format0_t header_type1;
    struct dect_phy_header_type2_format0_t header_type2;
    struct nrf_modem_dect_phy_tx_params tx_op_params;
    bool ack_required = 0;

    dect_mac_utils_get_packet_length(&data_size, &df_mcs, &packet_length);

    if (!convert_ot_addr_to_dect_txrx_id((uint8_t *)data + OT_PHR_LENGTH,
                            data_size - OT_PHR_LENGTH,
                            &ack_required,
                            &header_type2.transmitter_identity_hi,
                            &header_type2.transmitter_identity_lo,
                            &header_type2.receiver_identity_hi,
                            &header_type2.receiver_identity_lo)) {
        LOG_ERR("Fail to get TX RX ID from OT MAC frame");
        return -EINVAL;
    }
    LOG_INF("ack_required: %d Tx hi: %d Tx lo: %d Rx hi: %d Rx lo: %d",
            ack_required,
            header_type2.transmitter_identity_hi,
            header_type2.transmitter_identity_lo,
            header_type2.receiver_identity_hi,
            header_type2.receiver_identity_lo);

    tx_op_params.bs_cqi = NRF_MODEM_DECT_PHY_BS_CQI_NOT_USED;
    tx_op_params.start_time = start_time;
    tx_op_params.network_id = TEST_NETWORK_ID;
	tx_op_params.lbt_rssi_threshold_max = DECT_RSSI_TARGET;
    tx_op_params.carrier = DECT_CARRIER;
	//tx_op_params.lbt_period = lbt_period;
    tx_op_params.data = data;
    tx_op_params.data_size = data_size;


    if (!ack_required) {
        /* Ack is not required. Set the type 1 header fields */
        header_type1.packet_length = packet_length;
        header_type1.header_format = 0x0;
        header_type1.packet_length_type = 0x0;
        header_type1.short_network_id = (uint8_t)(TEST_NETWORK_ID & 0xff);
        header_type1.transmitter_id_hi = (dect_device_id >> 8);
        header_type1.transmitter_id_lo = (dect_device_id & 0xff);
        header_type1.df_mcs = df_mcs;
        header_type1.transmit_power = tx_power;
        header_type1.reserved = 0;

        tx_op_params.phy_type = DECT_PHY_HEADER_TYPE1;
        tx_op_params.phy_header = (union nrf_modem_dect_phy_hdr *)&header_type1;
        tx_op_params.handle = DECT_TX_HANDLE;
        tx_op_params.lbt_period = 0;
        waiting_for_ack = false;

        err = nrf_modem_dect_phy_tx(&tx_op_params);
        if (err != 0)
        {
            LOG_ERR("nrf_modem_dect_phy_tx() returned %d", err);
            return err;
        }
        //set_pending_event(PENDING_EVENT_TX_STARTED);
    } else {
        /* Ack is required. Set the type 2 header fields with format 000 */
        header_type2.packet_length = packet_length;
        header_type2.packet_length_type = DECT_PHY_HEADER_PKT_LENGTH_TYPE_SUBSLOTS;
        header_type2.format = DECT_PHY_HEADER_FORMAT_000;
        header_type2.short_network_id = (uint8_t)(TEST_NETWORK_ID & 0xff);
        header_type2.df_mcs = df_mcs;
        header_type2.transmit_power = tx_power;
        header_type2.df_new_data_indication_toggle = 1; /* to be toggled later */
        header_type2.df_redundancy_version = 0;  /* 1st tx */
        header_type2.spatial_streams = 2;
        header_type2.feedback.format1.format = 1;
        header_type2.feedback.format1.CQI = 1;
        header_type2.feedback.format1.harq_process_number0 = 0;
        header_type2.feedback.format1.transmission_feedback0 = 1;
        header_type2.feedback.format1.buffer_status = 0;

        tx_op_params.phy_type = DECT_PHY_HEADER_TYPE2;
        tx_op_params.phy_header = (union nrf_modem_dect_phy_hdr *)&header_type2;
        tx_op_params.handle = DECT_TXRX_TX_HANDLE;

        struct nrf_modem_dect_phy_rx_params rx_op_params;

        rx_op_params.start_time = 0;
        rx_op_params.handle = DECT_TXRX_RX_HANDLE;
        rx_op_params.network_id = TEST_NETWORK_ID;
        rx_op_params.mode = NRF_MODEM_DECT_PHY_RX_MODE_SINGLE_SHOT;
        rx_op_params.rssi_interval = NRF_MODEM_DECT_PHY_RSSI_INTERVAL_OFF;
        rx_op_params.link_id = NRF_MODEM_DECT_PHY_LINK_UNSPECIFIED;
        rx_op_params.rssi_level = DEFAULT_SENSITIVITY;
        rx_op_params.carrier = DECT_CARRIER;
        rx_op_params.duration = 24 * DECT_RADIO_SUBSLOT_DURATION_IN_MODEM_TICKS;
        rx_op_params.filter.short_network_id = (uint8_t)(TEST_NETWORK_ID & 0xff);
        rx_op_params.filter.is_short_network_id_used = 1;
        /* listen for the HARQ */
        rx_op_params.filter.receiver_identity = (header_type2.transmitter_identity_hi << 8) | (header_type2.transmitter_identity_lo);
        tx_op_params.lbt_period = lbt_period;
        waiting_for_ack = true;
        struct nrf_modem_dect_phy_tx_rx_params tx_rx_params = {
            .tx = tx_op_params,
            .rx = rx_op_params,
        };
        err = nrf_modem_dect_phy_tx_rx(&tx_rx_params);
        if (err != 0)
        {
            LOG_ERR("nrf_modem_dect_phy_tx() returned %d", err);
            return err;
        }
    }
    LOG_HEXDUMP_INF(data, data_size, "tx frame:");
	set_pending_event(PENDING_EVENT_TX_STARTED);

    return err;
}

static void transmit_message(struct k_work *tx_job)
{
    int err;

    ARG_UNUSED(tx_job);

    __ASSERT_NO_MSG(ot_transmit_frame.mLength <= OT_RADIO_FRAME_MAX_SIZE);
    net_tx_buf->len = ot_transmit_frame.mLength;
    ot_channel = ot_transmit_frame.mChannel;

    net_pkt_set_ieee802154_frame_secured(net_tx_pkt,
                         ot_transmit_frame.mInfo.mTxInfo.mIsSecurityProcessed);
    net_pkt_set_ieee802154_mac_hdr_rdy(net_tx_pkt, ot_transmit_frame.mInfo.mTxInfo.mIsHeaderUpdated);

    /* Set PSDU length field and data field */
    if (OT_PHR_LENGTH == 1) {
        *(uint8_t *)dect_tx_psdu = (uint8_t)net_tx_buf->len;
    } else if (OT_PHR_LENGTH == 2) {
        *(uint16_t *)dect_tx_psdu = net_tx_buf->len;
    } else {
        __ASSERT_NO_MSG(0);
    }
    memcpy(dect_tx_psdu + OT_PHR_LENGTH, net_tx_buf->data, net_tx_buf->len);

    if ((ot_radio_caps & OT_RADIO_CAPS_TRANSMIT_TIMING) &&
        (ot_transmit_frame.mInfo.mTxInfo.mTxDelay != 0)) {
        /* Transmit packet in the future, perform CCA before transmission. */
#if defined(CONFIG_NET_PKT_TXTIME)
        uint32_t tx_at = ot_transmit_frame.mInfo.mTxInfo.mTxDelayBaseTime +
                 ot_transmit_frame.mInfo.mTxInfo.mTxDelay;
        net_pkt_set_timestamp_ns(net_tx_pkt, convert_32bit_us_wrapped_to_64bit_ns(tx_at));
#endif
		err = dect_transmit(NRF_MODEM_DECT_LBT_PERIOD_MIN, dect_tx_psdu, net_tx_buf->len + OT_PHR_LENGTH, 0);
    } else if (ot_transmit_frame.mInfo.mTxInfo.mCsmaCaEnabled) {
        /* Perform full CSMA/CA procedure before packet transmission */
		err = dect_transmit(NRF_MODEM_DECT_LBT_PERIOD_MIN, dect_tx_psdu, net_tx_buf->len + OT_PHR_LENGTH, 0);
    } else {
        /* Transmit packet immediately, no CCA */
		err = dect_transmit(0, dect_tx_psdu, net_tx_buf->len + OT_PHR_LENGTH, 0);
    }
}

static inline void handle_tx_done(otInstance *aInstance)
{
    ot_transmit_frame.mInfo.mTxInfo.mIsSecurityProcessed =
        net_pkt_ieee802154_frame_secured(net_tx_pkt);
    ot_transmit_frame.mInfo.mTxInfo.mIsHeaderUpdated = net_pkt_ieee802154_mac_hdr_rdy(net_tx_pkt);
LOG_INF("handle_tx_done: %d", ot_tx_result);
    if (IS_ENABLED(CONFIG_OPENTHREAD_DIAG) && otPlatDiagModeGet()) {
        otPlatDiagRadioTransmitDone(aInstance, &ot_transmit_frame, ot_tx_result);
    } else {
        otPlatRadioTxDone(aInstance, &ot_transmit_frame, ot_ack_frame.mLength ? &ot_ack_frame : NULL,
                  ot_tx_result);
        ot_ack_frame.mLength = 0;
    }
}

static void openthread_handle_received_frame(otInstance *instance,
                         struct net_pkt *pkt)
{
    otRadioFrame recv_frame;
    memset(&recv_frame, 0, sizeof(otRadioFrame));

    recv_frame.mPsdu = net_buf_frag_last(pkt->buffer)->data;
    /* Length inc. CRC. */
    recv_frame.mLength = net_buf_frags_len(pkt->buffer);
    recv_frame.mChannel = ot_channel;
    recv_frame.mInfo.mRxInfo.mLqi = net_pkt_ieee802154_lqi(pkt);
    recv_frame.mInfo.mRxInfo.mRssi = net_pkt_ieee802154_rssi_dbm(pkt);
    recv_frame.mInfo.mRxInfo.mAckedWithFramePending = net_pkt_ieee802154_ack_fpb(pkt);

#if defined(CONFIG_NET_PKT_TIMESTAMP)
    recv_frame.mInfo.mRxInfo.mTimestamp = net_pkt_timestamp_ns(pkt) / NSEC_PER_USEC;
#endif

    recv_frame.mInfo.mRxInfo.mAckedWithSecEnhAck = net_pkt_ieee802154_ack_seb(pkt);
    recv_frame.mInfo.mRxInfo.mAckFrameCounter = net_pkt_ieee802154_ack_fc(pkt);
    recv_frame.mInfo.mRxInfo.mAckKeyId = net_pkt_ieee802154_ack_keyid(pkt);

    if (IS_ENABLED(CONFIG_OPENTHREAD_DIAG) && otPlatDiagModeGet()) {
        otPlatDiagRadioReceiveDone(instance, &recv_frame, OT_ERROR_NONE);
    } else {
        otPlatRadioReceiveDone(instance, &recv_frame, OT_ERROR_NONE);
    }

    net_pkt_unref(pkt);
}

#if defined(CONFIG_OPENTHREAD_NAT64_TRANSLATOR)

static otMessage *openthread_ip4_new_msg(otInstance *instance, otMessageSettings *settings)
{
    return otIp4NewMessage(instance, settings);
}

static otError openthread_nat64_send(otInstance *instance, otMessage *message)
{
    return otNat64Send(instance, message);
}

#else /* CONFIG_OPENTHREAD_NAT64_TRANSLATOR */

static otMessage *openthread_ip4_new_msg(otInstance *instance, otMessageSettings *settings)
{
    return NULL;
}

static otError openthread_nat64_send(otInstance *instance, otMessage *message)
{
    return OT_ERROR_DROP;
}

#endif /* CONFIG_OPENTHREAD_NAT64_TRANSLATOR */

static void openthread_handle_frame_to_send(otInstance *instance,
                        struct net_pkt *pkt)
{
    otError error;
    struct net_buf *buf;
    otMessage *message;
    otMessageSettings settings;
    bool is_ip6 = PKT_IS_IPv6(pkt);

    NET_DBG("Sending %s packet to ot stack", is_ip6 ? "IPv6" : "IPv4");

    settings.mPriority = OT_MESSAGE_PRIORITY_NORMAL;
    settings.mLinkSecurityEnabled = true;

    message = is_ip6 ? otIp6NewMessage(instance, &settings)
             : openthread_ip4_new_msg(instance, &settings);
    if (!message) {
        NET_ERR("Cannot allocate new message buffer");
        goto exit;
    }

    if (IS_ENABLED(CONFIG_OPENTHREAD)) {
        /* Set multicast loop so the stack can process multicast packets for
         * subscribed addresses.
         */
        otMessageSetMulticastLoopEnabled(message, true);
    }

    for (buf = pkt->buffer; buf; buf = buf->frags) {
        if (otMessageAppend(message, buf->data, buf->len) != OT_ERROR_NONE) {
            NET_ERR("Error while appending to otMessage");
            otMessageFree(message);
            goto exit;
        }
    }

    error = is_ip6 ? otIp6Send(instance, message) : openthread_nat64_send(instance, message);

    if (error != OT_ERROR_NONE) {
        NET_ERR("Error while calling %s [error: %d]",
            is_ip6 ? "otIp6Send" : "openthread_nat64_send", error);
    }

exit:
    net_pkt_unref(pkt);
}

int notify_new_rx_frame(struct net_pkt *pkt)
{
    LOG_DBG("notify_new_rx_frame");
    k_fifo_put(&rx_pkt_fifo, pkt);
    set_pending_event(PENDING_EVENT_FRAME_RECEIVED);

    return 0;
}

int notify_new_tx_frame(struct net_pkt *pkt)
{
    LOG_DBG("notify_new_tx_frame");
    k_fifo_put(&tx_pkt_fifo, pkt);
    set_pending_event(PENDING_EVENT_FRAME_TO_SEND);

    return 0;
}

void platformRadioProcess(otInstance *aInstance)
{
    bool event_pending = false;

    if (is_pending_event_set(PENDING_EVENT_FRAME_TO_SEND)) {
        struct net_pkt *evt_pkt;

        reset_pending_event(PENDING_EVENT_FRAME_TO_SEND);
        while ((evt_pkt = (struct net_pkt *) k_fifo_get(&tx_pkt_fifo, K_NO_WAIT)) != NULL) {
            if (IS_ENABLED(CONFIG_OPENTHREAD_COPROCESSOR_RCP)) {
                net_pkt_unref(evt_pkt);
            } else {
                openthread_handle_frame_to_send(aInstance, evt_pkt);
            }
        }
    }
    if (is_pending_event_set(PENDING_EVENT_FRAME_RECEIVED)) {
        struct net_pkt *rx_pkt;

        reset_pending_event(PENDING_EVENT_FRAME_RECEIVED);
        while ((rx_pkt = (struct net_pkt *) k_fifo_get(&rx_pkt_fifo, K_NO_WAIT)) != NULL) {
            openthread_handle_received_frame(aInstance, rx_pkt);
        }
    }

    if (is_pending_event_set(PENDING_EVENT_RX_FAILED)) {
        reset_pending_event(PENDING_EVENT_RX_FAILED);
        if (IS_ENABLED(CONFIG_OPENTHREAD_DIAG) && otPlatDiagModeGet()) {
            otPlatDiagRadioReceiveDone(aInstance, NULL, ot_rx_result);
        } else {
            otPlatRadioReceiveDone(aInstance, NULL, ot_rx_result);
        }
    }

    if (is_pending_event_set(PENDING_EVENT_TX_STARTED)) {
        reset_pending_event(PENDING_EVENT_TX_STARTED);
        otPlatRadioTxStarted(aInstance, &ot_transmit_frame);
    }

    if (is_pending_event_set(PENDING_EVENT_TX_DONE)) {
        LOG_INF("PENDING_EVENT_TX_DONE");
        reset_pending_event(PENDING_EVENT_TX_DONE);
        handle_tx_done(aInstance);
    }

    if (is_pending_event_set(PENDING_EVENT_SLEEP)) {
        reset_pending_event(PENDING_EVENT_SLEEP);
        ARG_UNUSED(otPlatRadioSleep(aInstance));
    }

    /* handle events that can't run during transmission */
    if (ot_state != OT_RADIO_STATE_TRANSMIT) {
        /* TODO: Handle energy detection result */
    }

    if (event_pending) {
        otSysEventSignalPending();
    }
}

uint16_t platformRadioChannelGet(otInstance *aInstance)
{
    ARG_UNUSED(aInstance);

    return ot_channel;
}

void otPlatRadioSetPanId(otInstance *aInstance, otPanId aPanId)
{
    ARG_UNUSED(aInstance);

    LOG_INF("otPlatRadioSetPanId: %x", aPanId);
    ot_pan_id = aPanId;
}

static void ReverseExtAddress(otExtAddress *aReversed, const otExtAddress *aOrigin)
{
    for (size_t i = 0; i < sizeof(*aReversed); i++)
    {
        aReversed->m8[i] = aOrigin->m8[sizeof(*aOrigin) - 1 - i];
        printk("%x ", aReversed->m8[i]);
    }
    printk("\n");
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance,
                   const otExtAddress *aExtAddress)
{
    ARG_UNUSED(aInstance);
    LOG_DBG("otPlatRadioSetExtendedAddress");

    ReverseExtAddress(&ot_ext_addr, aExtAddress);
}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t aShortAddress)
{
    ARG_UNUSED(aInstance);
    LOG_DBG("otPlatRadioSetShortAddress: %x", aShortAddress);
    ot_short_addr = aShortAddress;
}

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
    ARG_UNUSED(aInstance);

    return (ot_state != OT_RADIO_STATE_DISABLED) ? true : false;
}

otError otPlatRadioEnable(otInstance *aInstance)
{
    if (!otPlatRadioIsEnabled(aInstance)) {
        ot_state = OT_RADIO_STATE_SLEEP;
    }

    return OT_ERROR_NONE;
}

otError otPlatRadioDisable(otInstance *aInstance)
{
    if (otPlatRadioIsEnabled(aInstance)) {
        ot_state = OT_RADIO_STATE_DISABLED;
    }

    return OT_ERROR_NONE;
}

otError otPlatRadioSleep(otInstance *aInstance)
{
    ARG_UNUSED(aInstance);

    otError error = OT_ERROR_INVALID_STATE;

    if (ot_state == OT_RADIO_STATE_SLEEP ||
        ot_state == OT_RADIO_STATE_RECEIVE ||
        ot_state == OT_RADIO_STATE_TRANSMIT) {
        error = OT_ERROR_NONE;
        /* Put the radio to sleep mode. Currently sleep mode of DECT NR+ radio is not supported */
        ot_state = OT_RADIO_STATE_SLEEP;
    }

    return error;
}

otError otPlatRadioReceive(otInstance *aInstance, uint8_t aChannel)
{
    ARG_UNUSED(aInstance);

    ot_channel = aChannel;
    LOG_INF("otPlatRadioReceive: %u state: %d waiting_for_ack: %d", aChannel, ot_state, waiting_for_ack);

    if ((ot_state != OT_RADIO_STATE_RECEIVE) && (waiting_for_ack == false)) {
        int err;

        err = dect_receive();
        if (err) {
            LOG_ERR("DECT Reception failed, err %d", err);
            return OT_ERROR_FAILED;
        }
		ot_state = OT_RADIO_STATE_RECEIVE;
    }

    return OT_ERROR_NONE;
}

otError platformRadioTransmitCarrier(otInstance *aInstance, bool aEnable)
{
    return OT_ERROR_NOT_IMPLEMENTED;
}

otRadioState otPlatRadioGetState(otInstance *aInstance)
{
    ARG_UNUSED(aInstance);

    return ot_state;
}

otError otPlatRadioTransmit(otInstance *aInstance, otRadioFrame *aPacket)
{
    otError error = OT_ERROR_INVALID_STATE;

    ARG_UNUSED(aInstance);
    ARG_UNUSED(aPacket);

    __ASSERT_NO_MSG(aPacket == &ot_transmit_frame);

    if ((ot_state == OT_RADIO_STATE_RECEIVE) || (ot_radio_caps & OT_RADIO_CAPS_SLEEP_TO_TX)) {
        ARG_UNUSED(aInstance);

        if (!k_work_is_pending(&tx_job)) {
            if (ot_state == OT_RADIO_STATE_RECEIVE) {
                int err;
                /* Wait for radio operation to complete. */
                err = nrf_modem_dect_phy_rx_stop(DECT_RX_HANDLE);
                if (err != 0) {
                    LOG_ERR("fail to stop dect phy rx: %d", err);
                    return err;
                }
                //k_sem_take(&radio_sem, K_FOREVER);
            } else {
                k_work_submit_to_queue(&ot_work_q, &tx_job);  
            }
            ot_state = OT_RADIO_STATE_TRANSMIT;
            //k_work_submit_to_queue(&ot_work_q, &tx_job);
            error = OT_ERROR_NONE;
        } else {
            error = -EBUSY;
        }
    }

    return error;
}

otRadioFrame *otPlatRadioGetTransmitBuffer(otInstance *aInstance)
{
    ARG_UNUSED(aInstance);

    return &ot_transmit_frame;
}

int8_t otPlatRadioGetRssi(otInstance *aInstance)
{
    LOG_INF("otPlatRadioGetRssi: %d", last_rssi);

    return last_rssi;
}

otRadioCaps otPlatRadioGetCaps(otInstance *aInstance)
{
    ARG_UNUSED(aInstance);

    LOG_INF("otPlatRadioGetCaps: %x", ot_radio_caps);

    return ot_radio_caps;
}

void otPlatRadioSetRxOnWhenIdle(otInstance *aInstance, bool aRxOnWhenIdle)
{
    ARG_UNUSED(aInstance);

    LOG_DBG("RxOnWhenIdle=%d", aRxOnWhenIdle ? 1 : 0);
    if (aRxOnWhenIdle) {
        ot_radio_caps |= OT_RADIO_CAPS_RX_ON_WHEN_IDLE;
    } else {
        ot_radio_caps &= ~OT_RADIO_CAPS_RX_ON_WHEN_IDLE;
    }
}

bool otPlatRadioGetPromiscuous(otInstance *aInstance)
{
    LOG_ERR("otPlatRadioSetPromiscuous is not supported");

    return false;
}

void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable)
{
    LOG_ERR("otPlatRadioSetPromiscuous is not supported");
}

otError otPlatRadioEnergyScan(otInstance *aInstance, uint8_t aScanChannel,
                  uint16_t aScanDuration)
{
    LOG_ERR("otPlatRadioEnergyScan\n");

    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioGetCcaEnergyDetectThreshold(otInstance *aInstance,
                           int8_t *aThreshold)
{
    printk("otPlatRadioGetCcaEnergyDetectThreshold\n");
    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioSetCcaEnergyDetectThreshold(otInstance *aInstance,
                           int8_t aThreshold)
{
    printk("otPlatRadioSetCcaEnergyDetectThreshold\n");
    return OT_ERROR_NOT_IMPLEMENTED;
}

void otPlatRadioEnableSrcMatch(otInstance *aInstance, bool aEnable)
{
    return;
}

otError otPlatRadioAddSrcMatchShortEntry(otInstance *aInstance,
                     const uint16_t aShortAddress)
{
    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioAddSrcMatchExtEntry(otInstance *aInstance,
                       const otExtAddress *aExtAddress)
{
    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioClearSrcMatchShortEntry(otInstance *aInstance,
                       const uint16_t aShortAddress)
{
    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioClearSrcMatchExtEntry(otInstance *aInstance,
                     const otExtAddress *aExtAddress)
{
    return OT_ERROR_NOT_IMPLEMENTED;
}

void otPlatRadioClearSrcMatchShortEntries(otInstance *aInstance)
{
    ARG_UNUSED(aInstance);
}

void otPlatRadioClearSrcMatchExtEntries(otInstance *aInstance)
{
    ARG_UNUSED(aInstance);
}

int8_t otPlatRadioGetReceiveSensitivity(otInstance *aInstance)
{
    ARG_UNUSED(aInstance);

    return DEFAULT_SENSITIVITY;
}

otError otPlatRadioGetTransmitPower(otInstance *aInstance, int8_t *aPower)
{
    ARG_UNUSED(aInstance);

    if (aPower == NULL) {
        return OT_ERROR_INVALID_ARGS;
    }

    *aPower = tx_power;

    return OT_ERROR_NONE;
}

otError otPlatRadioSetTransmitPower(otInstance *aInstance, int8_t aPower)
{
    ARG_UNUSED(aInstance);

    tx_power = aPower;

    return OT_ERROR_NONE;
}

uint64_t otPlatTimeGet(void)
{
        return k_ticks_to_us_floor64(k_uptime_ticks());
}

#if defined(CONFIG_NET_PKT_TXTIME)
uint64_t otPlatRadioGetNow(otInstance *aInstance)
{
    ARG_UNUSED(aInstance);

    return otPlatTimeGet();
}
#endif

#if !defined(CONFIG_OPENTHREAD_THREAD_VERSION_1_1)
void otPlatRadioSetMacKey(otInstance *aInstance, uint8_t aKeyIdMode, uint8_t aKeyId,
              const otMacKeyMaterial *aPrevKey, const otMacKeyMaterial *aCurrKey,
              const otMacKeyMaterial *aNextKey, otRadioKeyType aKeyType)
{
    ARG_UNUSED(aInstance);
    LOG_INF("otPlatRadioSetMacKey not implemented. Use software TX security instead");
}

void otPlatRadioSetMacFrameCounter(otInstance *aInstance,
                   uint32_t aMacFrameCounter)
{
    ARG_UNUSED(aInstance);
    LOG_INF("otPlatRadioSetMacFrameCounter not implemented. Use software TX security instead");
}

void otPlatRadioSetMacFrameCounterIfLarger(otInstance *aInstance, uint32_t aMacFrameCounter)
{
    ARG_UNUSED(aInstance);
    LOG_INF("otPlatRadioSetMacFrameCounterIfLarger not implemented. Use software TX security instead");
}
#endif

static void dect_phy_prefill_harq_feedback_data(void)
{
#if 0
	struct dect_phy_settings *current_settings = dect_common_settings_ref_get();
	struct dect_phy_perf_params *cmd_params = &(perf_data.cmd_params);
#endif
	//struct nrf_modem_dect_phy_tx_params harq_tx_params = {
    harq_tx_params.start_time = 0;
    harq_tx_params.handle = DECT_HARQ_FEEDBACK_TX_HANDLE;
    harq_tx_params.carrier = DECT_CARRIER;
    harq_tx_params.network_id = TEST_NETWORK_ID;
    harq_tx_params.phy_type = DECT_PHY_HEADER_TYPE2;
    harq_tx_params.lbt_period = 0;
    harq_tx_params.data_size = 4;
    harq_tx_params.bs_cqi = 1; /* MCS-0, no meaning in our case */
	//};

	//struct dect_phy_header_type2_format1_t header = {
    harp_header.packet_length = 0; /* = 1 slot */
    harp_header.packet_length_type = 0; /* SLOT */
    harp_header.format = DECT_PHY_HEADER_FORMAT_001;
    harp_header.short_network_id = (uint8_t)(TEST_NETWORK_ID & 0xff);
    //harp_header.transmitter_identity_hi = (uint8_t)(current_settings->common.transmitter_id >> 8);
    //harp_header.transmitter_identity_lo = (uint8_t)(current_settings->common.transmitter_id & 0xFF);
	//};
#if 0
	dect_phy_perf_pdu_t tx_data_pdu = {
		.header.message_type = DECT_MAC_MESSAGE_TYPE_PERF_HARQ_FEEDBACK,
		.header.transmitter_id = current_settings->common.transmitter_id,
	};

	dect_phy_perf_pdu_encode(perf_data.server_data.harq_feedback_data.encoded_data_to_send,
				 &tx_data_pdu);

	harq_tx_params.data = perf_data.server_data.harq_feedback_data.encoded_data_to_send;
	perf_data.server_data.harq_feedback_data.params = harq_tx_params;
	perf_data.server_data.harq_feedback_data.header = header;
#endif
}

static void dectnr_ot_l2_init(struct net_if *iface)
{
    struct dectnr_ot_l2_ctx *ctx = net_if_get_device(iface)->data;

    ctx->iface = iface;
    /* Copy 8 bytes device id */
    hwinfo_get_device_id((void *)ctx->eui64, sizeof(ctx->eui64));
    /* Company ID f4-ce-36  */
    ctx->eui64[0] = 0xF4;
    ctx->eui64[1] = 0xCE;
    ctx->eui64[2] = 0x36;
    net_if_set_link_addr(iface, ctx->eui64, sizeof(ctx->eui64), NET_LINK_IEEE802154);

    ieee802154_init(iface);
    dect_phy_prefill_harq_feedback_data();
}

static int dectnr_dev_init(const struct device *dev)
{
    ARG_UNUSED(dev);
    int err;

    err = nrf_modem_lib_init();
    if (err) {
        LOG_ERR("modem init failed, err %d", err);
        return -ENODEV;
    }
    err = nrf_modem_dect_phy_callback_set(&dect_phy_callbacks);
    if (err) {
        LOG_ERR("nrf_modem_dect_phy_callback_set failed, err %d", err);
        k_panic();
    }
    err = nrf_modem_dect_phy_init(&dect_phy_init_params);
    if (err) {
        LOG_ERR("nrf_modem_dect_phy_init failed, err %d", err);
        k_panic();
    }
    k_sem_take(&operation_sem, K_FOREVER);
    if (dect_op_result != NRF_MODEM_DECT_PHY_SUCCESS) {
        return -ENODEV;;
    }
    hwinfo_get_device_id((void *)&dect_device_id, sizeof(dect_device_id));

    LOG_INF("Dect NR+ PHY initialized, device ID: %d", dect_device_id);
    k_sem_give(&operation_sem);
    return 0;
}

static const struct ieee802154_radio_api dectnr_radio_api = {
    .iface_api.init = dectnr_ot_l2_init,
};

/* OPENTHREAD L2 */
#define L2_LAYER OPENTHREAD_L2
#define L2_CTX_TYPE NET_L2_GET_CTX_TYPE(OPENTHREAD_L2)

NET_DEVICE_INIT(dectnr_openthread_l2, "dectnr_openthread_l2",
        dectnr_dev_init, NULL,
        &dectnr_l2_ctx, NULL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
        &dectnr_radio_api, L2_LAYER, L2_CTX_TYPE, OPENTHREAD_MTU);