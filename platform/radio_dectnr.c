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

#include "dectnr.h"
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

#define DECT_TX_MCS 4
#define DEFAULT_SENSITIVITY		-105
#define DECT_RSSI_TARGET		-80
#define DECT_802154_RX_BUFFERS	16
#define DECT_RX_STACK_SIZE		800
#define OPENTHREAD_MTU			1280

/* 802.15.4 Physical Layer Header Length */
#define IEEE802154_MAC_FRAME_TYPE_DATA        0x01
#define IEEE802154_MAC_FRAME_SHORT_ADDR_MODE  0x02
#define IEEE802154_MAC_FRAME_TYPE_COMMAND     0x03
#define IEEE802154_MAC_BROADCAST_ADDR         0xffff
#define IEEE802154_PHR_LENGTH			1
#define IEEE802154_PSDU_LENGTH			OT_RADIO_FRAME_MAX_SIZE
#define IEEE802154_MAC_ADDRESS_MODE_SHORT 0x2
#define IEEE802154_MAC_ADDRESS_MODE_LONG 0x3
#define IEEE802154_MAC_DST_ADDR_OFFSET 5

enum pending_events {
    PENDING_EVENT_FRAME_TO_SEND, /* There is a tx frame to send  */
    PENDING_EVENT_FRAME_RECEIVED, /* Radio has received new frame */
    PENDING_EVENT_RX_FAILED, /* The RX failed */
    PENDING_EVENT_DECT_IDLE, /* DECT Radio is ready for next operation */
    PENDING_EVENT_TX_DONE, /* Radio transmission finished */
    PENDING_EVENT_COUNT /* Keep last */
};

/* Semaphore to synchronize modem calls. */
K_SEM_DEFINE(modem_operation_sem, 0, 1);

ATOMIC_DEFINE(pending_events, PENDING_EVENT_COUNT);

K_FIFO_DEFINE(rx_pkt_fifo);
K_FIFO_DEFINE(tx_pkt_fifo);
K_FIFO_DEFINE(dect_tx_fifo);

static struct net_pkt *net_tx_pkt;
static struct net_buf *net_tx_buf;

static struct nrf_modem_dect_phy_tx_params harq_feedback_tx_params;
static struct dect_phy_header_type2_format1_t harq_feedback_header;

static otRadioCaps ot_radio_caps = OT_RADIO_CAPS_RX_ON_WHEN_IDLE | OT_RADIO_CAPS_ACK_TIMEOUT | OT_RADIO_CAPS_TRANSMIT_RETRIES | OT_RADIO_CAPS_SLEEP_TO_TX | OT_RADIO_CAPS_ENERGY_SCAN;
static otRadioState ot_state = OT_RADIO_STATE_DISABLED;
static otRadioFrame ot_transmit_frame;
static otRadioFrame ot_ack_frame;
static otPanId        ot_pan_id;
static otError ot_rx_result;
static uint8_t ot_channel;

static int8_t last_rssi = INT8_MAX;
static int8_t tx_power = CONFIG_OPENTHREAD_DEFAULT_TX_POWER;
static uint32_t df_mcs = DECT_TX_MCS;

static enum nrf_modem_dect_phy_err dect_op_result;

#define MAX_RX_FRAME_LEN IEEE802154_PHR_LENGTH + OT_RADIO_FRAME_MAX_SIZE

enum buffered_frame_type {
    FRAME_UNUSED,
    FRAME_DECT_OT_ADDR_MAPPING,
    FRAME_OT_BROADCAST,
    FRAME_OT_UNICAST
};

// Common header (shared by all frames)
typedef struct {
    void *fifo_reserved; /* 1st word reserved for use by fifo. */
    enum buffered_frame_type frame_type;
    uint8_t length;
    uint8_t data[MAX_RX_FRAME_LEN];
    uint8_t lqi; /* Last received frame LQI value. */
    int8_t rssi; /* Last received frame RSSI value. */
    uint64_t time; /* RX timestamp. */
} FrameHeader;

// DECT OT Address Mapping Frame
typedef struct {
    //void *fifo_reserved; /* 1st word reserved for use by fifo. */
    FrameHeader header;
    //uint8_t data[MAX_RX_FRAME_LEN];
} DectOtAddrMappingFrame;

// OT Broadcast Frame
typedef struct {
    //void *fifo_reserved; /* 1st word reserved for use by fifo. */
    FrameHeader header;
    //uint8_t data[MAX_RX_FRAME_LEN];
} DectOtBroadcastFrame;

// OT Unicast Frame
typedef struct {
    //void *fifo_reserved; /* 1st word reserved for use by fifo. */
    FrameHeader header;
    //uint8_t data[MAX_RX_FRAME_LEN];
    uint16_t dect_peer_device_id; /* transmiter ID the received frame. */
    uint8_t sequence_number; /* Sequence number of the received frame. */
    bool pending_frame; /* true if the frame is pending. */
} DectOtUnicastFrame;

typedef union {
    FrameHeader header;
    DectOtAddrMappingFrame addr_mapping_frame;
    DectOtBroadcastFrame broadcast_frame;
    DectOtUnicastFrame unicast_frame;
} dect_802154_rx_frame;

#define MAX_HARQ_PROCESSES      4
#define MAX_BROADCAST_PROCESSES 2
#define DECT_MAX_PEER (32 + CONFIG_OPENTHREAD_MAX_CHILDREN)
#define DECT_PENDING_RX_FRAME_TIMEOUT 500 /* 500 ms timeout for pending RX frame */
#define DECT_PEER_DEVICE_TIMEOUT 600000000 /* 10 minutes timeout for peer device */

struct dectnr_ot_l2_ctx {
    struct net_if *iface;
    dect_radio_state_t radio_state;
    uint8_t eui64[8];

    /* Variable to mark if the last received frame is a beacon */
    bool pcc_is_beacon;
    bool pcc_is_ack;
    uint16_t last_dect_peer_device_id;
    struct dect_ot_address_mapping_t ot_addr_map;

    /* DECT PEER DEVICE TABLE */
    struct dect_peer_device_table dect_peer_devices[DECT_MAX_PEER];

    /* Work to send address mapping beacon */
    struct k_work_delayable address_mapping_beacon_work;

    /* RX thread stack. */
    K_KERNEL_STACK_MEMBER(rx_stack, DECT_RX_STACK_SIZE);

    /* RX thread control block. */
    struct k_thread rx_thread;

    /* RX fifo queue. */
    struct k_fifo rx_fifo;

    /* Buffers for passing received frame pointers and data to the
     * RX thread via rx_fifo object.
     */
    dect_802154_rx_frame rx_frames[DECT_802154_RX_BUFFERS];
};

static struct dectnr_ot_l2_ctx dectnr_l2_ctx;

/*
 * DECT PHY TX process information
 */
struct dect_tx_process_info {
    /* 1st word reserved for use by fifo. */
	void *fifo_reserved;
	bool tx_in_progress;
    uint16_t dect_receiver_device_id; /* DECT receiver device ID */
	uint8_t process_nbr;
    uint8_t dect_tx_psdu[DECT_DATA_MAX_LEN]; /* TX Buffer for DECT PHY transmition */
    uint16_t dect_data_size;
    bool ack_required;
    bool ack_received;
    uint8_t retransmit_count;
    uint8_t last_redundancy_version;
    /* Delayable work for waiting frame aggregation or random access process */
    struct k_work_delayable tx_process_work;
    struct k_work random_backoff_work;
};

static struct dect_tx_process_info tx_processes[MAX_HARQ_PROCESSES + MAX_BROADCAST_PROCESSES];

static bool process_radio_rx_frame(dect_802154_rx_frame *rx_frame);
static void process_dect_beacon_ot_addr_mapping(const uint8_t *data);

static void reset_tx_process(uint8_t process_nbr)
{
    LOG_DBG("reset tx_process %hhu", process_nbr);
    if (process_nbr >= MAX_HARQ_PROCESSES + MAX_BROADCAST_PROCESSES) {
        LOG_ERR("Invalid process number %hhu", process_nbr);
        return;
    }
    tx_processes[process_nbr].tx_in_progress = false;
    tx_processes[process_nbr].dect_data_size = 0;
    tx_processes[process_nbr].retransmit_count = 0;
    tx_processes[process_nbr].ack_required = false;
    tx_processes[process_nbr].ack_received = false;
    memset(tx_processes[process_nbr].dect_tx_psdu, 0, sizeof(tx_processes[process_nbr].dect_tx_psdu));
}

static int dect_set_radio_state(dect_radio_state_t radio_state) {
    LOG_DBG("DECT radio state %d -> %d", dectnr_l2_ctx.radio_state, radio_state);
    dectnr_l2_ctx.radio_state = radio_state;
    return 0;
}

void random_backoff_work_handler(struct k_work *work)
{
    uint16_t random_backoff_ms, max_backoff_ms;
    uint32_t random_value = sys_rand32_get();
    struct dect_tx_process_info *tx_process = CONTAINER_OF((struct k_work *)work, struct dect_tx_process_info, random_backoff_work);

    if (tx_process->retransmit_count > DECT_MAX_BACKOFF_COUNT) {
        LOG_WRN("Max backoff count reached");
        reset_tx_process(tx_process->process_nbr);
        return;
    }
    max_backoff_ms = 1 << (tx_process->retransmit_count + DECT_MIN_BACKOFF_EXPONENTIAL);
    random_backoff_ms = random_value % max_backoff_ms;

    random_backoff_ms = random_value % max_backoff_ms;
    k_work_reschedule(&tx_process->tx_process_work, K_MSEC(random_backoff_ms));
    tx_process->retransmit_count++;
    LOG_DBG("Retransmit %hhu time, delay %hu ms", tx_process->retransmit_count, random_backoff_ms);
}

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

static const int16_t byte_per_mcs_and_length[5][16] = {
    { 0,  17,  33,  50,  67,  83,  99, 115, 133, 149, 165, 181, 197, 213, 233, 249},
    { 4,  37,  69, 103, 137, 169, 201, 233, 263, 295, 327, 359, 391, 423, 463, 495},
    { 7,  57, 107, 157, 205, 253, 295, 343, 399, 447, 495, 540, 596, 644, 692,  -1},
    {11,  77, 141, 209, 271, 335, 399, 463, 532, 596, 660,  -1,  -1,  -1,  -1,  -1},
    {18, 117, 217, 311, 407, 503, 604, 700,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1}};

void dect_mac_utils_get_packet_length(int16_t *data_size, uint32_t *mcs, uint32_t *packet_length)
{
    for (*packet_length = 0; *packet_length < 16; (*packet_length)++)
    {
        if (byte_per_mcs_and_length[*mcs][*packet_length] == -1) {
            packet_length--;
            break;
        }
        if (byte_per_mcs_and_length[*mcs][*packet_length] >= *data_size)
        {
            break;
        }
    }
}

static int8_t harq_tx_next_redundancy_version(uint8_t current_redundancy_version)
{
	/* MAC spec ch. 5.5.1:
	 * Hybrid ARQ redundancies shall be sent in the order {0, 2, 3, 1, 0, …}.
	 * Initial transmission shall use redundancy version 0 for the selected HARQ process number.
	 * Number of retransmissions is application dependent.
	 */
	if (current_redundancy_version == 0) {
		return 2;
	} else if (current_redundancy_version == 2) {
		return 3;
	} else if (current_redundancy_version == 3) {
		return 1;
	} else if (current_redundancy_version == 1) {
		return 0;
	} else {
        LOG_ERR("Invalid redundancy version");
        return -1;
    }
}

/* DECT receive operation */
static int dect_receive(uint64_t start_time)
{
    int err;

    struct nrf_modem_dect_phy_rx_params rx_op_params = {
        .start_time = start_time,
        .handle = DECT_RECEIVE_HANDLE,
        .network_id = (uint32_t)ot_pan_id,
        .mode = NRF_MODEM_DECT_PHY_RX_MODE_SEMICONTINUOUS,
        .rssi_interval = NRF_MODEM_DECT_PHY_RSSI_INTERVAL_OFF,
        .link_id = NRF_MODEM_DECT_PHY_LINK_UNSPECIFIED,
        .rssi_level = DEFAULT_SENSITIVITY,
        .carrier = CONFIG_OPENTHREAD_RADIO_LINK_DECT_NR_PHY_FREQUENCY,
        .duration = UINT32_MAX,
        .filter.short_network_id = (uint8_t)(ot_pan_id & 0xff),
        .filter.is_short_network_id_used = 1,
        /* listen for beacon and unicast to me */
        .filter.receiver_identity = dectnr_l2_ctx.ot_addr_map.dev_id,
    };
    LOG_DBG("dect_receive. start_time: %"PRIu64"", start_time);
    err = nrf_modem_dect_phy_rx(&rx_op_params);
    if (err == 0) {
        dect_set_radio_state(DECT_RADIO_STATE_RX);
        LOG_DBG("DECT Reception started");
    }

    return err;
}

/* DECT transmit operation. */
static int dect_transmit(struct dect_tx_process_info *tx_process)
{
    int err;
    uint32_t packet_length = 0;
    struct dect_phy_header_type1_format0_t header_type1;
    struct dect_phy_header_type2_format0_t header_type2;
    struct nrf_modem_dect_phy_tx_params tx_op_params;

    dect_mac_utils_get_packet_length(&tx_process->dect_data_size, &df_mcs, &packet_length);

    tx_op_params.bs_cqi = NRF_MODEM_DECT_PHY_BS_CQI_NOT_USED;
    tx_op_params.start_time = 0;
    tx_op_params.network_id = (uint32_t)ot_pan_id;
	tx_op_params.lbt_rssi_threshold_max = DECT_RSSI_TARGET;
    tx_op_params.carrier = CONFIG_OPENTHREAD_RADIO_LINK_DECT_NR_PHY_FREQUENCY;
    tx_op_params.data = tx_process->dect_tx_psdu;
    tx_op_params.data_size = tx_process->dect_data_size;

    if (tx_process->process_nbr >= MAX_HARQ_PROCESSES) {
        /* Ack is not required. Set the type 1 header fields */
        header_type1.transmitter_id_hi = (dectnr_l2_ctx.ot_addr_map.dev_id >> 8);
        header_type1.transmitter_id_lo = (dectnr_l2_ctx.ot_addr_map.dev_id & 0xff);
        header_type1.packet_length = packet_length;
        header_type1.header_format = DECT_PHY_HEADER_FORMAT_000;
        header_type1.packet_length_type = DECT_PHY_HEADER_PKT_LENGTH_TYPE_SUBSLOTS;
        header_type1.short_network_id = (uint8_t)(ot_pan_id & 0xff);
        header_type1.df_mcs = df_mcs;
        header_type1.transmit_power = tx_power;
        header_type1.reserved = 0;

        tx_op_params.phy_type = DECT_PHY_HEADER_TYPE1;
        tx_op_params.phy_header = (union nrf_modem_dect_phy_hdr *)&header_type1;
        tx_op_params.handle = DECT_TX_PROCESS_TX_HANDLE_START + tx_process->process_nbr;
        tx_op_params.lbt_period = 0;

        err = nrf_modem_dect_phy_tx(&tx_op_params);
        if (err != 0)
        {
            LOG_ERR("nrf_modem_dect_phy_tx() returned %d", err);
            return err;
        }
    } else {
        header_type2.transmitter_id_hi = (dectnr_l2_ctx.ot_addr_map.dev_id >> 8);
        header_type2.transmitter_id_lo = (dectnr_l2_ctx.ot_addr_map.dev_id & 0xff);
        header_type2.receiver_identity_hi = tx_process->dect_receiver_device_id >> 8;
        header_type2.receiver_identity_lo = tx_process->dect_receiver_device_id & 0xff;
        /* Ack is required. Set the type 2 header fields with format 000 */
        header_type2.packet_length = packet_length;
        header_type2.packet_length_type = DECT_PHY_HEADER_PKT_LENGTH_TYPE_SUBSLOTS;
        header_type2.format = DECT_PHY_HEADER_FORMAT_000;
        header_type2.short_network_id = (uint8_t)(ot_pan_id & 0xff);
        header_type2.df_mcs = df_mcs;
        header_type2.transmit_power = tx_power;
        if (tx_process->retransmit_count > 0) {
            header_type2.df_new_data_indication_toggle = 0; /* no new data */
            header_type2.df_redundancy_version =
                harq_tx_next_redundancy_version(tx_process->last_redundancy_version);
        } else {
            header_type2.df_new_data_indication_toggle = 1; /* to be toggled later */
            header_type2.df_redundancy_version = 0;  /* 1st tx */
        }
        tx_process->last_redundancy_version = header_type2.df_redundancy_version;
        header_type2.df_harq_process_number = tx_process->process_nbr;
        header_type2.spatial_streams = 2;
        header_type2.feedback.format1.format = 1;
        header_type2.feedback.format1.CQI = 1;
        header_type2.feedback.format1.harq_process_number0 = tx_process->process_nbr;
        LOG_INF("process_nbr: %hhu", header_type2.feedback.format1.harq_process_number0);
        header_type2.feedback.format1.transmission_feedback0 = 1;
        header_type2.feedback.format1.buffer_status = 0;

        tx_op_params.phy_type = DECT_PHY_HEADER_TYPE2;
        tx_op_params.phy_header = (union nrf_modem_dect_phy_hdr *)&header_type2;
        tx_op_params.handle = DECT_TX_PROCESS_TX_HANDLE_START + tx_process->process_nbr;

        struct nrf_modem_dect_phy_rx_params rx_op_params;

        rx_op_params.start_time = 0;
        rx_op_params.handle = DECT_TX_PROCESS_RX_HANDLE_START + tx_process->process_nbr;
        rx_op_params.network_id = (uint32_t)ot_pan_id;
        rx_op_params.mode = NRF_MODEM_DECT_PHY_RX_MODE_SINGLE_SHOT;
        rx_op_params.rssi_interval = NRF_MODEM_DECT_PHY_RSSI_INTERVAL_OFF;
        rx_op_params.link_id = NRF_MODEM_DECT_PHY_LINK_UNSPECIFIED;
        rx_op_params.rssi_level = DEFAULT_SENSITIVITY;
        rx_op_params.carrier = CONFIG_OPENTHREAD_RADIO_LINK_DECT_NR_PHY_FREQUENCY;
        rx_op_params.duration =
            (DECT_HARQ_FEEDBACK_RX_DELAY_SUBSLOTS + DECT_HARQ_FEEDBACK_RX_SUBSLOTS) *
            DECT_RADIO_SUBSLOT_DURATION_IN_MODEM_TICKS;
        rx_op_params.filter.short_network_id = (uint8_t)(ot_pan_id & 0xff);
        rx_op_params.filter.is_short_network_id_used = 1;
        /* listen for the HARQ */
        rx_op_params.filter.receiver_identity = (header_type2.transmitter_id_hi << 8) | (header_type2.transmitter_id_lo);
        tx_op_params.lbt_period = NRF_MODEM_DECT_LBT_PERIOD_MIN;
        tx_process->ack_required = true;
        struct nrf_modem_dect_phy_tx_rx_params tx_rx_params = {
            .tx = tx_op_params,
            .rx = rx_op_params,
        };
        LOG_INF("dect_transmit process_nbr: %hhu sequence number: %u", tx_process->process_nbr, tx_process->dect_tx_psdu[0]);
        err = nrf_modem_dect_phy_tx_rx(&tx_rx_params);
        if (err != 0)
        {
            LOG_ERR("nrf_modem_dect_phy_tx_rx() returned %d", err);
            return err;
        }
    }
    LOG_HEXDUMP_DBG(tx_process->dect_tx_psdu, tx_process->dect_data_size, "");
    dect_set_radio_state(DECT_RADIO_STATE_TX);
    set_pending_event(PENDING_EVENT_TX_DONE);

    return err;
}

/* DECT HARQ feedback operation */
static int dect_harq_feedback(const struct nrf_modem_dect_phy_rx_pcc_status *status,
                              const struct dect_phy_header_type2_format0_t *header)
{
    uint16_t receiver_dev_id = (header->receiver_identity_hi << 8) | header->receiver_identity_lo;

    if (receiver_dev_id == dectnr_l2_ctx.ot_addr_map.dev_id) {
        LOG_DBG("RxID 0x%02X%02X Device ID 0x%02X%02X",
            header->receiver_identity_hi, header->receiver_identity_lo,
            dectnr_l2_ctx.ot_addr_map.dev_id >> 8, dectnr_l2_ctx.ot_addr_map.dev_id & 0xff);
    } else {
        LOG_ERR("Not for me. RxID 0x%02X%02X Device ID 0x%02X%02X",
            header->receiver_identity_hi, header->receiver_identity_lo,
            dectnr_l2_ctx.ot_addr_map.dev_id >> 8, dectnr_l2_ctx.ot_addr_map.dev_id & 0xff);
        return -EINVAL;
    }
    uint16_t len_slots = header->packet_length + 1;
    /* HARQ feedback requested */
    union nrf_modem_dect_phy_hdr phy_header;

    harq_feedback_header.format = DECT_PHY_HEADER_FORMAT_001;
    harq_feedback_header.df_mcs = df_mcs;
    harq_feedback_header.transmit_power = tx_power;
    harq_feedback_header.receiver_identity_hi = header->transmitter_id_hi;
    harq_feedback_header.receiver_identity_lo = header->transmitter_id_lo;
    harq_feedback_header.transmitter_id_hi = header->receiver_identity_hi;
    harq_feedback_header.transmitter_id_lo = header->receiver_identity_lo;
    harq_feedback_header.spatial_streams = header->spatial_streams;
    harq_feedback_header.feedback.format1.format = 1;
    harq_feedback_header.feedback.format1.CQI = 1;
    harq_feedback_header.feedback.format1.harq_process_number0 =
        header->df_harq_process_number;
    harq_feedback_header.short_network_id = (uint8_t)(ot_pan_id & 0xff);

    /* ACK/NACK: According to CRC: */
    harq_feedback_header.feedback.format1.transmission_feedback0 = 0;
    harq_feedback_header.feedback.format1.buffer_status = 0; //TODO: confirm buffer status is not required
    memcpy(&phy_header.type_2, &harq_feedback_header, sizeof(phy_header.type_2));
    harq_feedback_tx_params.network_id = (uint32_t)ot_pan_id;
    harq_feedback_tx_params.phy_header = &phy_header;
    harq_feedback_tx_params.start_time = status->stf_start_time +
        (len_slots * DECT_RADIO_SUBSLOT_DURATION_IN_MODEM_TICKS) + 
        DECT_HARQ_FEEDBACK_TX_DELAY_SUBSLOTS * DECT_RADIO_SUBSLOT_DURATION_IN_MODEM_TICKS;
    int err = nrf_modem_dect_phy_tx_harq(&harq_feedback_tx_params);
    if (err) {
        printk("nrf_modem_dect_phy_tx_harq() failed: %d\n", err);
        return err;
    }
    dect_set_radio_state(DECT_RADIO_STATE_TX);
    return 0;
}

/* prefill data for DECT PHY HARQ feedback operation */
static void dect_phy_prefill_harq_feedback_data(void)
{
    harq_feedback_tx_params.start_time = 0;
    harq_feedback_tx_params.handle = DECT_HARQ_FEEDBACK_HANDLE;
    harq_feedback_tx_params.carrier = CONFIG_OPENTHREAD_RADIO_LINK_DECT_NR_PHY_FREQUENCY;
    harq_feedback_tx_params.phy_type = DECT_PHY_HEADER_TYPE2;
    harq_feedback_tx_params.lbt_period = 0;
    harq_feedback_tx_params.data_size = 4;
    harq_feedback_tx_params.bs_cqi = 1; /* MCS-0, no meaning in our case */

    harq_feedback_header.packet_length = 0; /* = 1 slot */
    harq_feedback_header.packet_length_type = 0; /* SLOT */
    harq_feedback_header.format = DECT_PHY_HEADER_FORMAT_001;
}

/* Callback after init operation. */
static void init(const uint64_t *time, int16_t temp, enum nrf_modem_dect_phy_err err,
      const struct nrf_modem_dect_phy_modem_cfg *cfg)
{
    if (err) {
        LOG_ERR("DECT init operation failed, err %d", err);
    }
    dect_op_result = err;
    k_sem_give(&modem_operation_sem);
}

/* Callback after deinit operation. */
static void deinit(const uint64_t *time, enum nrf_modem_dect_phy_err err)
{
    if (err) {
        LOG_ERR("Deinit failed, err %d", err);
        return;
    }
    k_sem_give(&modem_operation_sem);
}

/* Operation complete notification. */
static void op_complete(const uint64_t *time, int16_t temperature,
         enum nrf_modem_dect_phy_err err, uint32_t handle)
{
	if (err != 0) {
		LOG_ERR("op_complete cb time %"PRIu64" handle: %d err %X", *time, handle, err);
	}
    if (handle == DECT_RECEIVE_HANDLE) {
        if (err == NRF_MODEM_DECT_PHY_SUCCESS) {
            LOG_DBG("DECT RX success in op_complete");
        } else {
            LOG_ERR("DECT RX failed in op_complete, err %X", err);
        }
        if (dectnr_l2_ctx.radio_state == DECT_RADIO_STATE_RX) {
            set_pending_event(PENDING_EVENT_DECT_IDLE);
        }
    }
    if (handle == DECT_HARQ_FEEDBACK_HANDLE) {
        if (err == NRF_MODEM_DECT_PHY_SUCCESS) {
            LOG_DBG("DECT HARQ Feedback TX success in op_complete");
        } else {
            LOG_ERR("DECT HARQ Feedback TX failed in op_complete, err %X", err);
        }
        err = dect_receive(*time + 2 * DECT_RADIO_SUBSLOT_DURATION_IN_MODEM_TICKS);
        if (err != 0) {
            LOG_ERR("DECT RX failed in op_complete, err %X", err);
        }
    }
    if (handle >= DECT_TX_PROCESS_TX_HANDLE_START && handle < DECT_TX_PROCESS_TX_HANDLE_START + MAX_HARQ_PROCESSES + MAX_BROADCAST_PROCESSES) {
        if (err == 0) {
            LOG_DBG("DECT TX process %d completed", handle - DECT_TX_PROCESS_TX_HANDLE_START);
        } else {
            LOG_ERR("DECT TX process %d failed, err %X", handle - DECT_TX_PROCESS_TX_HANDLE_START, err);
        }
        if (tx_processes[handle - DECT_TX_PROCESS_TX_HANDLE_START].ack_required) {
            LOG_DBG("Tx process %d wait for ack!", handle - DECT_TX_PROCESS_TX_HANDLE_START);
        } else {
            reset_tx_process(handle - DECT_TX_PROCESS_TX_HANDLE_START);
            set_pending_event(PENDING_EVENT_DECT_IDLE);
        }
    }
    if (handle >= DECT_TX_PROCESS_RX_HANDLE_START && handle < DECT_TX_PROCESS_RX_HANDLE_START + MAX_HARQ_PROCESSES) {
        if (err == 0) {
            LOG_DBG("DECT TX process %d RX completed", handle - DECT_TX_PROCESS_RX_HANDLE_START);
            if (!tx_processes[handle - DECT_TX_PROCESS_RX_HANDLE_START].ack_received && tx_processes[handle - DECT_TX_PROCESS_RX_HANDLE_START].retransmit_count < DECT_MAX_BACKOFF_COUNT) { 
                LOG_WRN("Ack not received. Tx process %d retransmit", handle - DECT_TX_PROCESS_RX_HANDLE_START);
                k_work_submit(&tx_processes[handle - DECT_TX_PROCESS_RX_HANDLE_START].random_backoff_work);
                set_pending_event(PENDING_EVENT_DECT_IDLE);
            } else {
                reset_tx_process(handle - DECT_TX_PROCESS_RX_HANDLE_START);
                set_pending_event(PENDING_EVENT_DECT_IDLE);
            }
        } else {
            LOG_ERR("DECT TX process %d RX failed, err %X", handle - DECT_TX_PROCESS_RX_HANDLE_START, err);
            if (err == NRF_MODEM_DECT_PHY_ERR_COMBINED_OP_FAILED) {
                LOG_WRN("Ack not received. Tx process %d retransmit", handle - DECT_TX_PROCESS_RX_HANDLE_START);
                k_work_submit(&tx_processes[handle - DECT_TX_PROCESS_RX_HANDLE_START].random_backoff_work);
                set_pending_event(PENDING_EVENT_DECT_IDLE);
            } else {
                reset_tx_process(handle - DECT_TX_PROCESS_RX_HANDLE_START);
                set_pending_event(PENDING_EVENT_DECT_IDLE);
            }
        }
    }
}

/* Callback after receive stop operation. */
static void rx_stop(const uint64_t *time, enum nrf_modem_dect_phy_err err, uint32_t handle)
{
    LOG_DBG("rx_stop cb time %"PRIu64" status %d handle %d", *time, err, handle);
}

/* Physical Control Channel reception notification. */
static void pcc(const uint64_t *time,
                const struct nrf_modem_dect_phy_rx_pcc_status *status,
                const union nrf_modem_dect_phy_hdr *hdr)
{
	/* Provide HARQ feedback if requested */
	if (status->header_status == NRF_MODEM_DECT_PHY_HDR_STATUS_VALID &&
        status->phy_type == DECT_PHY_HEADER_TYPE2) {
        struct dect_phy_header_type2_format0_t *header_fmt0 = (void *)hdr;
        struct dect_phy_header_type2_format1_t *header_fmt1 = (void *)hdr;
        /* If FORMAT 0 PCC header received, provide HARQ feedback */
        if (header_fmt0->format == DECT_PHY_HEADER_FORMAT_000) {
            int err = dect_harq_feedback(status, header_fmt0);
            if (err) {
                LOG_ERR("dect_harq_feedback failed: %d", err);
                return;
            }
            dectnr_l2_ctx.last_dect_peer_device_id = (header_fmt0->transmitter_id_hi << 8) | header_fmt0->transmitter_id_lo;
            dectnr_l2_ctx.pcc_is_ack = false;
        }
        /* If FORMAT 1 PCC header received, process ACK/NACK */
        else if (header_fmt1->format == DECT_PHY_HEADER_FORMAT_001) {
            /* Find TX process by TX ID */
            if (header_fmt1->feedback.format1.format == 1) {
                if (header_fmt1->feedback.format1.transmission_feedback0) {
                    /* Handle ACK: clear the HARQ process resources */
                    LOG_INF("ACK received for process %hhu", header_fmt1->feedback.format1.harq_process_number0);
                    tx_processes[header_fmt1->feedback.format1.harq_process_number0].ack_received = true;
                    dectnr_l2_ctx.pcc_is_ack = true;
                } else {
                    /* Handle NACK: retransmit */
                    LOG_INF("NACK received for process %hhu", header_fmt1->feedback.format1.harq_process_number0);
                    dectnr_l2_ctx.pcc_is_ack = true;
                }
            }
        }
        dectnr_l2_ctx.pcc_is_beacon = false;
    } else if (status->header_status == NRF_MODEM_DECT_PHY_HDR_STATUS_VALID &&
               status->phy_type == DECT_PHY_HEADER_TYPE1) {
        struct dect_phy_header_type1_format0_t *header_fmt0 = (void *)hdr;
        dectnr_l2_ctx.last_dect_peer_device_id  = (header_fmt0->transmitter_id_hi << 8) | header_fmt0->transmitter_id_lo;
        dectnr_l2_ctx.pcc_is_beacon = true;
        dectnr_l2_ctx.pcc_is_ack = false;
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
    dect_802154_rx_frame *rx_frame;
    uint8_t pkt_len = 0;

    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    while (1) {
        pkt = NULL;
        rx_frame = k_fifo_get(&dectnr_l2_ctx.rx_fifo, K_FOREVER);
        if (rx_frame == NULL) {
            LOG_ERR("Failed to get rx_frame from fifo");
            continue;
        }
        if (rx_frame->header.frame_type == FRAME_UNUSED) {
            LOG_ERR("Frame type is unused, skipping");
            continue;
        }
        if (rx_frame->header.frame_type == FRAME_DECT_OT_ADDR_MAPPING) {
            process_dect_beacon_ot_addr_mapping(rx_frame->addr_mapping_frame.header.data);
            rx_frame->header.frame_type = FRAME_UNUSED;
            continue;
        } else if (rx_frame->header.frame_type == FRAME_OT_BROADCAST) {
            LOG_DBG("Broadcast frame received");
        } else if (rx_frame->header.frame_type == FRAME_OT_UNICAST) {
            if (!process_radio_rx_frame(rx_frame)) {
                if (!rx_frame->unicast_frame.pending_frame) {
                    LOG_ERR("Failed to process received frame");
                    rx_frame->header.frame_type = FRAME_UNUSED;
                    continue;
                } else {
                    LOG_DBG("Pending frame received, not putting to FIFO");
                    continue;
                }
            } else {
                LOG_DBG("Unicast frame received");
            }
        }
        pkt_len = rx_frame->header.length;
        if (pkt_len > IEEE802154_PSDU_LENGTH || pkt_len == 0) {
            LOG_ERR("Invalid PSDU length: %hu", pkt_len);
            rx_frame->header.frame_type = FRAME_UNUSED;
            continue;
        }
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

        if (net_pkt_write(pkt, rx_frame->header.data, pkt_len)) {
            goto drop;
        }

        net_pkt_set_ieee802154_lqi(pkt, rx_frame->header.lqi);
        net_pkt_set_ieee802154_rssi_dbm(pkt, rx_frame->header.rssi);

#if defined(CONFIG_NET_PKT_TIMESTAMP)
        net_pkt_set_timestamp_ns(pkt, rx_frame->header.time * NSEC_PER_USEC);
#endif

        LOG_INF("Caught a packet (%u) (LQI: %u)", pkt_len, rx_frame->header.lqi);
        //LOG_HEXDUMP_DBG(rx_frame->rx_psdu + IEEE802154_PHR_LENGTH, pkt_len, "rx frame:");

        if (net_recv_data(dectnr_l2_ctx.iface, pkt) < 0) {
            LOG_ERR("Packet dropped by NET stack");
            goto drop;
        }

        rx_frame->header.frame_type = FRAME_UNUSED;
        continue;

drop:
        rx_frame->header.frame_type = FRAME_UNUSED;
        net_pkt_unref(pkt);
    }
}

static void process_dect_beacon_ot_addr_mapping(const uint8_t *data)
{
    if (data == NULL) {
        LOG_ERR("Invalid data pointer");
        return;
    }

    struct dect_ot_address_mapping_t *addr_mapping = (struct dect_ot_address_mapping_t *)data;
    LOG_DBG("process_dect_beacon_ot_addr_mapping: %hu %hu", addr_mapping->dev_id, addr_mapping->rloc);
    LOG_DBG("ext_addr: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
            addr_mapping->ext_addr.m8[0], addr_mapping->ext_addr.m8[1],
            addr_mapping->ext_addr.m8[2], addr_mapping->ext_addr.m8[3],
            addr_mapping->ext_addr.m8[4], addr_mapping->ext_addr.m8[5],
            addr_mapping->ext_addr.m8[6], addr_mapping->ext_addr.m8[7]);
    if (addr_mapping->dev_id == 0) {
        LOG_ERR("Invalid DECT device ID: %hu", addr_mapping->dev_id);
        return;
    }
    if (addr_mapping->dev_id == dectnr_l2_ctx.ot_addr_map.dev_id) {
        LOG_DBG("Received own DECT device ID: %hu", addr_mapping->dev_id);
        return;
    }
    if (addr_mapping->rloc == 0xffff) {
        LOG_ERR("Invalid DECT RLOC: %hu", addr_mapping->rloc);
        return;
    }
    /* Update peer address mapping table */
    for (int i = 0; i < DECT_MAX_PEER; i++) {
        if (dectnr_l2_ctx.dect_peer_devices[i].peer_device_id == addr_mapping->dev_id) {
            // The device is in the table, update rloc and ext addr
            dectnr_l2_ctx.dect_peer_devices[i].rloc = addr_mapping->rloc;
            dectnr_l2_ctx.dect_peer_devices[i].last_activity_time = otPlatTimeGet();
            memcpy(&dectnr_l2_ctx.dect_peer_devices[i].ext_addr, &addr_mapping->ext_addr, sizeof(dectnr_l2_ctx.dect_peer_devices[i].ext_addr));
            LOG_DBG("Updated DECT device ID: %hu RLOC: %hu", addr_mapping->dev_id, addr_mapping->rloc);
            return;
        }
    }
    // The device is not in the table, check if there is space for it
    // Check if the slot is empty or if the device has timed out
    // If yes, add the new device to the table
    for (int i = 0; i < DECT_MAX_PEER; i++) {
        if (dectnr_l2_ctx.dect_peer_devices[i].peer_device_id == 0 ||
            (otPlatTimeGet() - dectnr_l2_ctx.dect_peer_devices[i].last_activity_time >
                DECT_PEER_DEVICE_TIMEOUT)) {
            dectnr_l2_ctx.dect_peer_devices[i].peer_device_id = addr_mapping->dev_id;
            //dectnr_l2_ctx.dect_peer_devices[i].next_seq_to_peer++;
            memcpy(&dectnr_l2_ctx.dect_peer_devices[i].ext_addr, &addr_mapping->ext_addr, sizeof(dectnr_l2_ctx.dect_peer_devices[i].ext_addr));
            dectnr_l2_ctx.dect_peer_devices[i].rloc = addr_mapping->rloc;
            dectnr_l2_ctx.dect_peer_devices[i].last_activity_time = otPlatTimeGet();
            LOG_INF("Added receiver device ID: %u", dectnr_l2_ctx.dect_peer_devices[i].peer_device_id);
            return;
        }
    }
}

/* Process received frame to check the frame sequence */
static bool process_radio_rx_frame(dect_802154_rx_frame *rx_frame)
{
    for (int i = 0; i < DECT_MAX_PEER; i++) {
        if (dectnr_l2_ctx.dect_peer_devices[i].peer_device_id == rx_frame->unicast_frame.dect_peer_device_id) {
            // The peer and destination pair is in the table, check if the sequence number is expected
            // If yes, put the frame to the FIFO. Then update the expected sequence number and time of the device.
            // If not, keep it in the buffer for later processing
            dectnr_l2_ctx.dect_peer_devices[i].last_activity_time = rx_frame->header.time;
            if (rx_frame->unicast_frame.sequence_number == dectnr_l2_ctx.dect_peer_devices[i].next_seq_from_peer) {
                LOG_INF("Frame from device:%hu Expected: %u",
                    rx_frame->unicast_frame.dect_peer_device_id, dectnr_l2_ctx.dect_peer_devices[i].next_seq_from_peer);
                dectnr_l2_ctx.dect_peer_devices[i].next_seq_from_peer++;
                k_work_reschedule(&dectnr_l2_ctx.dect_peer_devices[i].pending_rx_frame_work,
                    K_NO_WAIT);
                return true;
            } else {
                LOG_INF("Out of order frame. Expected: %u Received: %u",
                        dectnr_l2_ctx.dect_peer_devices[i].next_seq_from_peer, rx_frame->unicast_frame.sequence_number);
                if (rx_frame->unicast_frame.sequence_number < 
                    dectnr_l2_ctx.dect_peer_devices[i].next_seq_from_peer) {
                    // The frame is old, drop it
                    LOG_ERR("Old frame from device:%hu Expected: %u",
                        rx_frame->unicast_frame.dect_peer_device_id, dectnr_l2_ctx.dect_peer_devices[i].next_seq_from_peer);
                    goto drop;
                }
                rx_frame->unicast_frame.pending_frame = true;
                k_work_reschedule(&dectnr_l2_ctx.dect_peer_devices[i].pending_rx_frame_work,
                        K_MSEC(DECT_PENDING_RX_FRAME_TIMEOUT));
                return false;
            }
        }
    }

    // The device is not in the table, check if there is space for it
    // Check if the slot is empty or if the device has timed out
    // If yes, add the new device to the table
    for (int i = 0; i < DECT_MAX_PEER; i++) {
        if (dectnr_l2_ctx.dect_peer_devices[i].peer_device_id == 0 ||
            (rx_frame->header.time - dectnr_l2_ctx.dect_peer_devices[i].last_activity_time >
                DECT_PEER_DEVICE_TIMEOUT)) {
            dectnr_l2_ctx.dect_peer_devices[i].peer_device_id = rx_frame->unicast_frame.dect_peer_device_id;
            dectnr_l2_ctx.dect_peer_devices[i].next_seq_from_peer = rx_frame->unicast_frame.sequence_number + 1;  
            dectnr_l2_ctx.dect_peer_devices[i].last_activity_time = rx_frame->header.time;
            LOG_DBG("Add new peer device: %u to index %d", rx_frame->unicast_frame.dect_peer_device_id, i);
            return true;
        }
    }
drop:
    // No space for new device, drop the frame
    LOG_ERR("No space for new peer device ID: %u", rx_frame->unicast_frame.dect_peer_device_id);
    return false;
}

/* Physical Data Channel reception notification. */
static void pdc(const uint64_t *time,
        const struct nrf_modem_dect_phy_rx_pdc_status *status,
        const void *data, uint32_t len)
{
    int8_t power, lqi;

    if (dectnr_l2_ctx.pcc_is_ack) {
        LOG_DBG("ack received");
        return;
    }
    if (data == NULL) {
        LOG_ERR("Invalid data pointer");
        return;
    }
    if (status->snr >= 127) {
        LOG_ERR("SNR Not known or not detectable.");
        return;
    }
    /* Received RSSI value is in fixed precision format Q14.1 */
    LOG_DBG("Received %d bytes data (RSSI: %d.%d) from %hu", len,
        (status->rssi_2 / 2), (status->rssi_2 & 0b1) * 5, dectnr_l2_ctx.last_dect_peer_device_id );
    LOG_HEXDUMP_DBG(data, len, "pdc:");
    power = (status->rssi_2) / 2;
    lqi = status->snr;

    for (int i = 0; i < DECT_802154_RX_BUFFERS; i++) {
        LOG_DBG("rx_frames[%d].header.frame_type: %u", i, dectnr_l2_ctx.rx_frames[i].header.frame_type);
        if (dectnr_l2_ctx.rx_frames[i].header.frame_type != FRAME_UNUSED) {
            /* Find next free slot */
            continue;
        }
        if (dectnr_l2_ctx.pcc_is_beacon) {
            if (*(enum dect_beacon_type *)data == DECT_BEACON_TYPE_DECT_OT_ADDR_MAPPING) {
                dectnr_l2_ctx.rx_frames[i].header.frame_type = FRAME_DECT_OT_ADDR_MAPPING;
                dectnr_l2_ctx.rx_frames[i].header.length = len - DECT_FRAME_INFO_SIZE;
                memcpy(dectnr_l2_ctx.rx_frames[i].header.data, (uint8_t *)data + DECT_FRAME_INFO_SIZE, dectnr_l2_ctx.rx_frames[i].header.length);
            } else if (*(enum dect_beacon_type *)data == DECT_BEACON_TYPE_OT_MAC_BROADCAST_FRAME) {
                dectnr_l2_ctx.rx_frames[i].header.frame_type = FRAME_OT_BROADCAST;
                dectnr_l2_ctx.rx_frames[i].header.length = *(uint8_t *)((uint8_t *)data + DECT_FRAME_INFO_SIZE);
                memcpy(dectnr_l2_ctx.rx_frames[i].header.data, (uint8_t *)data + DECT_FRAME_INFO_SIZE + IEEE802154_PHR_LENGTH, dectnr_l2_ctx.rx_frames[i].header.length);
            } else {
                LOG_ERR("Unknown beacon type: %u", *(enum dect_beacon_type *)data);
                return;
            }
        } else {
            dectnr_l2_ctx.rx_frames[i].header.frame_type = FRAME_OT_UNICAST;
            dectnr_l2_ctx.rx_frames[i].header.length = *(uint8_t *)((uint8_t *)data + DECT_FRAME_INFO_SIZE);
            dectnr_l2_ctx.rx_frames[i].unicast_frame.dect_peer_device_id = dectnr_l2_ctx.last_dect_peer_device_id ;
            dectnr_l2_ctx.rx_frames[i].unicast_frame.sequence_number = *(uint8_t *)data;
            memcpy(dectnr_l2_ctx.rx_frames[i].header.data, (uint8_t *)data + DECT_FRAME_INFO_SIZE + IEEE802154_PHR_LENGTH, dectnr_l2_ctx.rx_frames[i].header.length);
        }
        dectnr_l2_ctx.rx_frames[i].header.rssi = power;
        last_rssi = power;
        dectnr_l2_ctx.rx_frames[i].header.lqi = lqi;
        dectnr_l2_ctx.rx_frames[i].header.time = otPlatTimeGet();
        k_fifo_put(&dectnr_l2_ctx.rx_fifo, &dectnr_l2_ctx.rx_frames[i]);
        return;
    }
    LOG_ERR("Not enough rx frames allocated for 15.4 driver!");
}

/* Physical Data Channel CRC error notification. */
static void pdc_crc_err(const uint64_t *time,
                        const struct nrf_modem_dect_phy_rx_pdc_crc_failure *crc_failure)
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
    LOG_WRN("Unexpectedly in %s", (__func__));
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
    .harq_rx_process_count = MAX_HARQ_PROCESSES,
};

/* Allocate net buffer for transmit */
static void packet_buffer_init(void)
{
    net_tx_pkt = net_pkt_alloc(K_NO_WAIT);
    __ASSERT_NO_MSG(net_tx_pkt != NULL);

    net_tx_buf = net_pkt_get_reserve_tx_data(OT_RADIO_FRAME_MAX_SIZE,
                         K_NO_WAIT);
    __ASSERT_NO_MSG(net_tx_buf != NULL);

    net_pkt_append_buffer(net_tx_pkt, net_tx_buf);

    ot_transmit_frame.mPsdu = net_tx_buf->data;
}

void platformRadioInit(void)
{
    packet_buffer_init();
    k_fifo_init(&dectnr_l2_ctx.rx_fifo);

    k_thread_create(&dectnr_l2_ctx.rx_thread, dectnr_l2_ctx.rx_stack,
            DECT_RX_STACK_SIZE,
            dect_rx_thread, NULL, NULL, NULL,
            K_PRIO_COOP(2), 0, K_NO_WAIT);
}

static inline void handle_tx_done(otInstance *aInstance)
{
    ot_transmit_frame.mInfo.mTxInfo.mIsSecurityProcessed =
        net_pkt_ieee802154_frame_secured(net_tx_pkt);
    ot_transmit_frame.mInfo.mTxInfo.mIsHeaderUpdated = net_pkt_ieee802154_mac_hdr_rdy(net_tx_pkt);

    if (IS_ENABLED(CONFIG_OPENTHREAD_DIAG) && otPlatDiagModeGet()) {
        otPlatDiagRadioTransmitDone(aInstance, &ot_transmit_frame, OT_ERROR_NONE);
    } else {
        otPlatRadioTxDone(aInstance, &ot_transmit_frame, ot_ack_frame.mLength ? &ot_ack_frame : NULL,
            OT_ERROR_NONE);
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

#if defined(CONFIG_NET_PKT_TIMESTAMP)
    recv_frame.mInfo.mRxInfo.mTimestamp = net_pkt_timestamp_ns(pkt) / NSEC_PER_USEC;
#endif

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

/**
 * Notify OpenThread task about new rx message.
 */
int notify_new_rx_frame(struct net_pkt *pkt)
{
    LOG_DBG("notify_new_rx_frame");
    k_fifo_put(&rx_pkt_fifo, pkt);
    set_pending_event(PENDING_EVENT_FRAME_RECEIVED);

    return 0;
}

/**
 * Notify OpenThread task about new tx message.
 */
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
    if (is_pending_event_set(PENDING_EVENT_DECT_IDLE)) {
        /* Get pending tx process from fifo and transmit */
        struct dect_tx_process_info *tx_process;
        tx_process = (struct dect_tx_process_info *) k_fifo_get(&dect_tx_fifo, K_NO_WAIT);
        if (tx_process != NULL) {
            int err;
            err = dect_transmit(tx_process);
            if (err != 0) {
                LOG_ERR("dect_transmit() returned %d", err);
            }
        } else {
            LOG_DBG("No pending tx process. Start DECT reception");
            if (dect_receive(0) != 0) {
                LOG_ERR("DECT Reception failed.");
            }
        }
        reset_pending_event(PENDING_EVENT_DECT_IDLE);
    }
    if (is_pending_event_set(PENDING_EVENT_TX_DONE)) {
        reset_pending_event(PENDING_EVENT_TX_DONE);
        handle_tx_done(aInstance);
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

#define WINDOW_SIZE 128

static bool is_sequence_before(unsigned char seq1, unsigned char seq2) {
    int diff = (int)seq2 - (int)seq1;

    if (diff > 0 && diff < WINDOW_SIZE) {
        return true;
    }

    if (diff < -WINDOW_SIZE) {
        return true;
    }

    return false;
}

void pending_rx_frame_work_handler(struct k_work *work)
{
    struct dect_peer_device_table *peer_seq_tracker = CONTAINER_OF((struct k_work_delayable *)work, struct dect_peer_device_table, pending_rx_frame_work);

    dect_802154_rx_frame *closest_frame = NULL;
    LOG_INF("pending_rx_frame_work dev id: %hu", peer_seq_tracker->peer_device_id);
    for (int i = 0; i < DECT_802154_RX_BUFFERS; i++) {
        if (dectnr_l2_ctx.rx_frames[i].header.frame_type != FRAME_OT_UNICAST) {
            continue;
        } else if (!dectnr_l2_ctx.rx_frames[i].unicast_frame.pending_frame) {
            /* Skip frames that are not pending */
            continue;
        }

        LOG_INF("[%d] dev id:%hu", i, dectnr_l2_ctx.rx_frames[i].unicast_frame.dect_peer_device_id);
        if (dectnr_l2_ctx.rx_frames[i].unicast_frame.dect_peer_device_id != peer_seq_tracker->peer_device_id) {
            /* Skip frames from other devices */
            continue;
        }
        if (closest_frame == NULL) {
            closest_frame = &dectnr_l2_ctx.rx_frames[i];
        } else if (is_sequence_before(dectnr_l2_ctx.rx_frames[i].unicast_frame.sequence_number, closest_frame->unicast_frame.sequence_number)) {
            closest_frame = &dectnr_l2_ctx.rx_frames[i];
        }
    }

    if (closest_frame) {
        LOG_INF("Closest frame with sequence: %u", closest_frame->unicast_frame.sequence_number);
        peer_seq_tracker->next_seq_from_peer = closest_frame->unicast_frame.sequence_number;
        closest_frame->unicast_frame.pending_frame = false;
        k_fifo_put(&dectnr_l2_ctx.rx_fifo, closest_frame);
        k_work_reschedule(&peer_seq_tracker->pending_rx_frame_work, K_NO_WAIT);
    } else {
        LOG_DBG("No matching pending frame found");
    }
}

void tx_process_work_handler(struct k_work *work)
{
    struct dect_tx_process_info *tx_process = CONTAINER_OF((struct k_work_delayable *)work, struct dect_tx_process_info, tx_process_work);

    LOG_DBG("tx_process:%hhu dect_data_size:%hu DECT radio state:%hu",
            tx_process->process_nbr, tx_process->dect_data_size, dectnr_l2_ctx.radio_state);
    /* If DECT Radio is in receiving state, stop the receiving task */
    if (dectnr_l2_ctx.radio_state == DECT_RADIO_STATE_RX) {
        int err;
        /* Wait for radio operation to complete. */
        err = nrf_modem_dect_phy_rx_stop(DECT_RECEIVE_HANDLE);
        if (err == 0) {
            k_fifo_put(&dect_tx_fifo, tx_process);
        } else {
            LOG_ERR("Failed to stop dect phy rx");
        }
    } else if (dectnr_l2_ctx.radio_state == DECT_RADIO_STATE_TX) {
        /* DECT Radio is in transmitting state. Reschedule the task */
        k_work_submit(&tx_process->random_backoff_work);
    } else {
        /* DECT Radio is disabled */
        LOG_ERR("DECT Radio is in disabled state.");
    }
}

static int broadcast_dect_ot_address_mapping(struct dect_ot_address_mapping_t ot_addr_map)
{
    /* Allocate free broadcast tx process and put to tx fifo */
    for (int i = MAX_HARQ_PROCESSES; i < MAX_HARQ_PROCESSES + MAX_BROADCAST_PROCESSES; i++) {
        if (!tx_processes[i].tx_in_progress) {
            tx_processes[i].dect_data_size = DECT_FRAME_INFO_SIZE + sizeof(ot_addr_map);
            tx_processes[i].dect_tx_psdu[0] = DECT_BEACON_TYPE_DECT_OT_ADDR_MAPPING;
            memcpy(tx_processes[i].dect_tx_psdu + DECT_FRAME_INFO_SIZE, &ot_addr_map, sizeof(ot_addr_map));
            tx_processes[i].tx_in_progress = true;
            k_work_reschedule(&tx_processes[i].tx_process_work, K_NO_WAIT);
            return 0;
        }
    }
    LOG_ERR("No available broadcast process");
    return -ENOMEM;
}

void address_mapping_beacon_work_handler(struct k_work *work)
{
    if(broadcast_dect_ot_address_mapping(dectnr_l2_ctx.ot_addr_map) != 0) {
        LOG_ERR("Failed to send DECT OT address mapping beacon");
    }
    k_work_reschedule(&dectnr_l2_ctx.address_mapping_beacon_work, K_MSEC(DECT_OT_ADDR_BEACON_INTERVAL_MS));
}

static int send_mac_broadcast_frame(struct otRadioFrame *ot_transmit_frame)
{
    /* Allocate free broadcast tx process and put to tx fifo */
    for (int i = MAX_HARQ_PROCESSES; i < MAX_HARQ_PROCESSES + MAX_BROADCAST_PROCESSES; i++) {
        if (!tx_processes[i].tx_in_progress) {
            tx_processes[i].dect_data_size = DECT_FRAME_INFO_SIZE + IEEE802154_PHR_LENGTH + ot_transmit_frame->mLength;
            tx_processes[i].dect_tx_psdu[0] = DECT_BEACON_TYPE_OT_MAC_BROADCAST_FRAME;
            tx_processes[i].dect_tx_psdu[1] = (uint8_t)ot_transmit_frame->mLength;
            memcpy(tx_processes[i].dect_tx_psdu + DECT_FRAME_INFO_SIZE + IEEE802154_PHR_LENGTH, ot_transmit_frame->mPsdu, ot_transmit_frame->mLength);
            tx_processes[i].tx_in_progress = true;
            k_work_reschedule(&tx_processes[i].tx_process_work, K_NO_WAIT);
            return 0;
        }
    }
    LOG_ERR("No available broadcast process");
    return -ENOMEM;
}

static uint16_t ot_addr_to_dect_dev_id(struct otRadioFrame *ot_transmit_frame)
{
    uint8_t offset = IEEE802154_MAC_DST_ADDR_OFFSET;
    uint16_t dev_id = 0;

    /* Validate the PSDU length and structure */
    if (ot_transmit_frame->mLength < OT_RADIO_FRAME_MIN_SIZE || ot_transmit_frame->mLength > OT_RADIO_FRAME_MAX_SIZE)
    {
        LOG_ERR("Invalid PSDU length: %zu", ot_transmit_frame->mLength);
        return 0;
    }
    if (ot_transmit_frame->mPsdu == NULL) {
        LOG_ERR("Invalid msdu pointer");
        return 0;
    }

    struct ieee802154_fcf *fs = (struct ieee802154_fcf *)ot_transmit_frame->mPsdu;

    if (fs->fc.dst_addr_mode == IEEE802154_MAC_ADDRESS_MODE_LONG) // Extended Addressing Mode
    {
        if (offset + 8 > ot_transmit_frame->mLength) {
            LOG_INF("Extended dst address parse fail");
            return 0;
        }
        for (int i = 0; i < DECT_MAX_PEER; i++) {
            if (memcmp((const void *)(ot_transmit_frame->mPsdu + offset), &dectnr_l2_ctx.dect_peer_devices[i].ext_addr, sizeof(dectnr_l2_ctx.dect_peer_devices[i].ext_addr)) == 0) {
                // The device is in the table, return device id
                dev_id = dectnr_l2_ctx.dect_peer_devices[i].peer_device_id;
                LOG_INF("Found device ID: %hu", dev_id);
                return dev_id;
            }
        }
    }
    else if (fs->fc.dst_addr_mode == IEEE802154_MAC_ADDRESS_MODE_SHORT) // Short Addressing Mode
    {
        if (offset + 2 > ot_transmit_frame->mLength) {
            LOG_INF("Short dst address parse fail");
            return 0;
        }
        for (int i = 0; i < DECT_MAX_PEER; i++) {
            if (memcmp((const void *)(ot_transmit_frame->mPsdu + offset), &dectnr_l2_ctx.dect_peer_devices[i].rloc, sizeof(dectnr_l2_ctx.dect_peer_devices[i].rloc)) == 0) {
                // The device is in the table, return device id
                dev_id = dectnr_l2_ctx.dect_peer_devices[i].peer_device_id;
                LOG_INF("Found device ID: %hu", dev_id);
                return dev_id;
            }
        }
    }
    else
    {
        LOG_INF("Unsupported dst addressing mode: %d", fs->fc.dst_addr_mode);
        return 0;
    }
    return 0;
}

static int process_mac_unicast_tx_frame(struct otRadioFrame *ot_transmit_frame, struct dect_tx_process_info *tx_process)
{
    uint8_t sequence_number;

    tx_process->dect_receiver_device_id = ot_addr_to_dect_dev_id(ot_transmit_frame);
    if (tx_process->dect_receiver_device_id == 0) {
        LOG_ERR("Fail to get RX ID from OT MAC frame");
        return -EINVAL;
    }
    LOG_INF("dect_receiver_id: %hu", tx_process->dect_receiver_device_id);
    for (int i = 0; i < DECT_MAX_PEER; i++) {
        if (dectnr_l2_ctx.dect_peer_devices[i].peer_device_id == tx_process->dect_receiver_device_id) {
            // The device is in the table, increase the sequence number
            //dectnr_l2_ctx.dect_peer_devices[i].next_seq_to_peer++;
            sequence_number = dectnr_l2_ctx.dect_peer_devices[i].next_seq_to_peer;
            dectnr_l2_ctx.dect_peer_devices[i].next_seq_to_peer++;
            goto processed;
        }
    }
    LOG_ERR("Cannot find device ID: %hu in the table", tx_process->dect_receiver_device_id);
    return -EINVAL;
processed:
    tx_process->dect_data_size = sizeof(sequence_number) + ot_transmit_frame->mLength + IEEE802154_PHR_LENGTH;
    tx_process->dect_tx_psdu[0] = sequence_number;
    tx_process->dect_tx_psdu[1] = (uint8_t)ot_transmit_frame->mLength;
    memcpy(tx_process->dect_tx_psdu + IEEE802154_PHR_LENGTH + sizeof(sequence_number), ot_transmit_frame->mPsdu, ot_transmit_frame->mLength);
    return 0;
}

static int send_mac_unicast_frame(struct otRadioFrame *ot_transmit_frame)
{
    int ret;

    /* Allocate free unicast tx process and put to tx fifo */
    for (int i = 0; i < MAX_HARQ_PROCESSES; i++) {
        if (!tx_processes[i].tx_in_progress) {
            /* Parse received ID and increase sequence number */
            ret = process_mac_unicast_tx_frame(ot_transmit_frame, &tx_processes[i]);
            if (ret != 0) {
                LOG_ERR("Failed to get unicast sequence number");
                return ret;
            }
            tx_processes[i].tx_in_progress = true;
            k_work_reschedule(&tx_processes[i].tx_process_work, K_NO_WAIT);
            return 0;
        }
    }
    LOG_ERR("No available unicast tx process");
    return -ENOMEM;
}

static int process_radio_tx_frame(void)
{
    struct ieee802154_fcf *fs = (struct ieee802154_fcf *)ot_transmit_frame.mPsdu;
    LOG_DBG("Frame type: %u AR: %d dst/src addr mode: %u %u", fs->fc.frame_type, fs->fc.ar, fs->fc.dst_addr_mode, fs->fc.src_addr_mode);

    if (fs->fc.frame_type == 0x00) {
        LOG_DBG("802.15.4 beacon frame");
        return send_mac_broadcast_frame(&ot_transmit_frame);
    } else if (fs->fc.frame_type == 0x02) {
        LOG_DBG("802.15.4 ack frame, not supported");
        return OT_ERROR_FAILED;
    }

    if ((fs->fc.dst_addr_mode == IEEE802154_MAC_FRAME_SHORT_ADDR_MODE) &&
        (*(uint16_t *)(ot_transmit_frame.mPsdu + IEEE802154_MAC_DST_ADDR_OFFSET) == IEEE802154_MAC_BROADCAST_ADDR)) {
        LOG_DBG("Send 802.15.4 broadcast frame");
        return send_mac_broadcast_frame(&ot_transmit_frame);
    } else {
        LOG_DBG("Send 802.15.4 unicast frame");
        return send_mac_unicast_frame(&ot_transmit_frame);
    }
}

void otPlatRadioSetPanId(otInstance *aInstance, otPanId aPanId)
{
    ARG_UNUSED(aInstance);

    LOG_INF("otPlatRadioSetPanId: %x", aPanId);
    if (aPanId == 0xFFFF) {
        LOG_ERR("Invalid PAN ID: %x", aPanId);
        return;
    }
    ot_pan_id = aPanId;
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance,
                                   const otExtAddress *aExtAddress)
{
    ARG_UNUSED(aInstance);
    memcpy(&dectnr_l2_ctx.ot_addr_map.ext_addr, aExtAddress, sizeof(struct otExtAddress));
}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t aShortAddress)
{
    ARG_UNUSED(aInstance);
}

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
    ARG_UNUSED(aInstance);

    return (ot_state != OT_RADIO_STATE_DISABLED) ? true : false;
}

otError otPlatRadioEnable(otInstance *aInstance)
{
    LOG_DBG("otPlatRadioEnable");
    if (!otPlatRadioIsEnabled(aInstance)) {
        ot_state = OT_RADIO_STATE_SLEEP;
    }

    return OT_ERROR_NONE;
}

otError otPlatRadioDisable(otInstance *aInstance)
{
    LOG_DBG("otPlatRadioDisable");
    if (otPlatRadioIsEnabled(aInstance)) {
        ot_state = OT_RADIO_STATE_DISABLED;
    }

    return OT_ERROR_NONE;
}

otError otPlatRadioSleep(otInstance *aInstance)
{
    ARG_UNUSED(aInstance);

    otError error = OT_ERROR_NOT_IMPLEMENTED;

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

    LOG_DBG("otPlatRadioReceive: %u current state: %d", ot_channel, ot_state);
    if (ot_state == OT_RADIO_STATE_SLEEP) {
        if (dect_receive(0) != 0) {
            LOG_ERR("DECT Reception failed");
            return OT_ERROR_FAILED;
        }
		ot_state = OT_RADIO_STATE_RECEIVE;
    } else if (ot_state == OT_RADIO_STATE_TRANSMIT) {
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
    ARG_UNUSED(aInstance);
    ARG_UNUSED(aPacket);

    __ASSERT_NO_MSG(aPacket == &ot_transmit_frame);
    __ASSERT_NO_MSG(ot_transmit_frame.mLength <= OT_RADIO_FRAME_MAX_SIZE);

    if ((ot_state != OT_RADIO_STATE_RECEIVE) && !(ot_radio_caps & OT_RADIO_CAPS_SLEEP_TO_TX)) {
        LOG_ERR("otPlatRadioTransmit: invalid state %d", ot_state);
        return OT_ERROR_INVALID_STATE;
    }

    if (process_radio_tx_frame() == 0) {
        ot_state = OT_RADIO_STATE_TRANSMIT;
        return OT_ERROR_NONE;
    }
    return OT_ERROR_FAILED;
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

static void ot_state_changed_handler(uint32_t flags, void *context)
{
    struct otInstance *aInstance = (struct otInstance *)context;

	if (flags & OT_CHANGED_THREAD_ROLE) {
		switch (otThreadGetDeviceRole(aInstance)) {
		case OT_DEVICE_ROLE_CHILD:
		case OT_DEVICE_ROLE_ROUTER:
		case OT_DEVICE_ROLE_LEADER:
		case OT_DEVICE_ROLE_DISABLED:
		case OT_DEVICE_ROLE_DETACHED:
		default:
            LOG_DBG("Thread role changed: %d", otThreadGetDeviceRole(aInstance));
			break;
		}
        goto send_beacon;
	}

	if (flags & OT_CHANGED_IP6_ADDRESS_REMOVED) {
		LOG_DBG("Ipv6 address removed");
        goto send_beacon;
	}

	if (flags & OT_CHANGED_IP6_ADDRESS_ADDED) {
		LOG_DBG("Ipv6 address added");
        goto send_beacon;
	}
    return;
send_beacon:
    const otIp6Address *ext_addr = otThreadGetLinkLocalIp6Address(aInstance);
    const otIp6Address *rloc = otThreadGetRloc(aInstance);
    if (ext_addr != NULL) {
        LOG_DBG("Ext Address: %x:%x:%x:%x:%x:%x:%x:%x:%x:%x:%x:%x:%x:%x:%x:%x",
            ext_addr->mFields.m8[0], ext_addr->mFields.m8[1],
            ext_addr->mFields.m8[2], ext_addr->mFields.m8[3],
            ext_addr->mFields.m8[4], ext_addr->mFields.m8[5],
            ext_addr->mFields.m8[6], ext_addr->mFields.m8[7],
            ext_addr->mFields.m8[8], ext_addr->mFields.m8[9],
            ext_addr->mFields.m8[10], ext_addr->mFields.m8[11],
            ext_addr->mFields.m8[12], ext_addr->mFields.m8[13],
            ext_addr->mFields.m8[14], ext_addr->mFields.m8[15]);
    }
    if (rloc == NULL) {
        LOG_DBG("RLOC Address: %x:%x:%x:%x:%x:%x:%x:%x:%x:%x:%x:%x:%x:%x:%x:%x",
            rloc->mFields.m8[0], rloc->mFields.m8[1],
            rloc->mFields.m8[2], rloc->mFields.m8[3],
            rloc->mFields.m8[4], rloc->mFields.m8[5],
            rloc->mFields.m8[6], rloc->mFields.m8[7],
            rloc->mFields.m8[8], rloc->mFields.m8[9],
            rloc->mFields.m8[10], rloc->mFields.m8[11],
            rloc->mFields.m8[12], rloc->mFields.m8[13],
            rloc->mFields.m8[14], rloc->mFields.m8[15]);
    }
    dectnr_l2_ctx.ot_addr_map.rloc = rloc->mFields.m8[14] << 8 | rloc->mFields.m8[15];
    k_work_reschedule(&dectnr_l2_ctx.address_mapping_beacon_work, K_NO_WAIT);
}

otRadioCaps otPlatRadioGetCaps(otInstance *aInstance)
{
    ARG_UNUSED(aInstance);

    LOG_INF("otPlatRadioGetCaps: %x", ot_radio_caps);
    otSetStateChangedCallback(aInstance, ot_state_changed_handler, aInstance);
    return ot_radio_caps;
}

void otPlatRadioSetRxOnWhenIdle(otInstance *aInstance, bool aRxOnWhenIdle)
{
    ARG_UNUSED(aInstance);

    LOG_INF("RxOnWhenIdle=%d", aRxOnWhenIdle ? 1 : 0);
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
    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioGetCcaEnergyDetectThreshold(otInstance *aInstance,
                           int8_t *aThreshold)
{
    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioSetCcaEnergyDetectThreshold(otInstance *aInstance,
                           int8_t aThreshold)
{
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
}

static int dectnr_dev_init(const struct device *dev)
{
    ARG_UNUSED(dev);
    int err;

    for (int i = 0; i < DECT_802154_RX_BUFFERS; i++) {
        dectnr_l2_ctx.rx_frames[i].header.frame_type = FRAME_UNUSED;
    }

    /* Init resource for HARQ process */
    for (int i = 0; i < MAX_HARQ_PROCESSES + MAX_BROADCAST_PROCESSES; i++) {
        tx_processes[i].process_nbr = i;
        reset_tx_process(i);
        k_work_init_delayable(&tx_processes[i].tx_process_work, tx_process_work_handler);
        k_work_init(&tx_processes[i].random_backoff_work, random_backoff_work_handler);
    }
    for (int i = 0; i < DECT_MAX_PEER; i++) {
        dectnr_l2_ctx.dect_peer_devices[i].peer_device_id = 0;
        dectnr_l2_ctx.dect_peer_devices[i].next_seq_from_peer = 0;
        dectnr_l2_ctx.dect_peer_devices[i].last_activity_time = 0;
        dectnr_l2_ctx.dect_peer_devices[i].next_seq_to_peer = 0;
        dectnr_l2_ctx.dect_peer_devices[i].last_activity_time = 0;
        k_work_init_delayable(&dectnr_l2_ctx.dect_peer_devices[i].pending_rx_frame_work, pending_rx_frame_work_handler);
    }
    dect_phy_prefill_harq_feedback_data();
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
    k_sem_take(&modem_operation_sem, K_FOREVER);
    if (dect_op_result != NRF_MODEM_DECT_PHY_SUCCESS) {
        return -ENODEV;;
    }
    dect_set_radio_state(DECT_RADIO_STATE_DISABLED);
    hwinfo_get_device_id((void *)&dectnr_l2_ctx.ot_addr_map.dev_id, sizeof(dectnr_l2_ctx.ot_addr_map.dev_id));
    LOG_INF("Dect NR+ PHY initialized, device ID: %d", dectnr_l2_ctx.ot_addr_map.dev_id);
    k_work_init_delayable(&dectnr_l2_ctx.address_mapping_beacon_work, address_mapping_beacon_work_handler);

    k_sem_give(&modem_operation_sem);

    return 0;
}

static const struct ieee802154_radio_api dectnr_radio_api = {
    .iface_api.init = dectnr_ot_l2_init,
};

/* OPENTHREAD L2 */
#define L2_LAYER OPENTHREAD_L2
#define L2_CTX_TYPE NET_L2_GET_CTX_TYPE(OPENTHREAD_L2)

NET_DEVICE_INIT(dectnr_openthread_l2, "dectnr_openthread_l2",
                dectnr_dev_init, NULL, &dectnr_l2_ctx,
                NULL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                &dectnr_radio_api, L2_LAYER, L2_CTX_TYPE, OPENTHREAD_MTU);