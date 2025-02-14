#define DECT_HARQ_FEEDBACK_TX_DELAY_SUBSLOTS 2
#define DECT_HARQ_FEEDBACK_RX_DELAY_SUBSLOTS 2
#define DECT_HARQ_FEEDBACK_RX_SUBSLOTS       3

#define DECT_RADIO_FRAME_DURATION_US		(10000)
#define DECT_RADIO_SLOT_DURATION_US		((double)DECT_RADIO_FRAME_DURATION_US / 24)

#define US_TO_MODEM_TICKS(x) ((uint64_t)(((x) * NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ) / 1000))
#define SECONDS_TO_MODEM_TICKS(s) (((uint64_t)(s)) * 1000 * NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ)
#define DECT_RADIO_SLOT_DURATION_IN_MODEM_TICKS (US_TO_MODEM_TICKS(DECT_RADIO_SLOT_DURATION_US))
#define DECT_RADIO_SUBSLOT_DURATION_IN_MODEM_TICKS  ((DECT_RADIO_SLOT_DURATION_IN_MODEM_TICKS) / 2) /* Note: assumes that mu = 1 */

#define DECT_MAX_TBS	  5600
#define DECT_DATA_MAX_LEN (DECT_MAX_TBS / 8)
#define DECT_MIN_BACKOFF_EXPONENTIAL 3
#define DECT_MAX_BACKOFF_COUNT 5

/* Definition of the DECT opertaion handles */
#define DECT_RECEIVE_HANDLE			      0
#define DECT_HARQ_FEEDBACK_HANDLE         1
#define DECT_TX_PROCESS_TX_HANDLE_START   10
#define DECT_TX_PROCESS_RX_HANDLE_START   20

#define DECT_OT_ADDR_BEACON_INTERVAL_MS 3000
#define DECT_FRAME_INFO_SIZE	1

typedef enum {
    DECT_RADIO_STATE_DISABLED,
    DECT_RADIO_STATE_RX,
    DECT_RADIO_STATE_TX
} dect_radio_state_t;

enum dect_phy_header_type {
	DECT_PHY_HEADER_TYPE1 = 0,
	DECT_PHY_HEADER_TYPE2,
};

enum dect_phy_header_format {
	DECT_PHY_HEADER_FORMAT_000 = 0,
	DECT_PHY_HEADER_FORMAT_001 = 1,
};

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

enum dect_beacon_type {
	DECT_BEACON_TYPE_DECT_OT_ADDR_MAPPING = 0,
	DECT_BEACON_TYPE_OT_MAC_BROADCAST_FRAME,
};

/* Address mapping from DECT device id to OT IPv6 address */
struct dect_ot_address_mapping_t {
	uint16_t dev_id;
	uint16_t rloc;
	otExtAddress ext_addr;
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

	uint8_t transmitter_id_hi;
	uint8_t transmitter_id_lo;

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

	uint8_t transmitter_id_hi;
	uint8_t transmitter_id_lo;

	uint8_t df_mcs: 4;
	uint8_t transmit_power: 4;

	uint8_t receiver_identity_hi;
	uint8_t receiver_identity_lo;

	uint8_t reserved: 6;
	uint8_t spatial_streams: 2;

	dect_phy_feedback_t feedback;
};

struct dect_peer_device_table {
    uint16_t peer_device_id; /* Unique identifier for the peer device */
    uint16_t rloc; /* RLOC of peer device */
    otExtAddress ext_addr; /* Extended address of the receiver device */
    uint64_t last_activity_time; /* Last time a frame was sent or received from the peer */
    uint8_t next_seq_from_peer; /* Next expected sequence number from the peer */
    uint8_t next_seq_to_peer; /* Next sequence number to the peer */
    struct k_work_delayable pending_rx_frame_work; /* Work to process pending frame */
};

/** IEEE 802.15.4 Frame Control Field, see section 7.2.2 */
struct ieee802154_fcf {
	struct {
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
	} fc __packed;

	uint8_t sequence;
} __packed;
