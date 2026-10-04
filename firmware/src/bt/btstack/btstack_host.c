// btstack_host.c - BTstack HID Host (Classic Bluetooth)
//
// Transport-agnostic BTstack integration for HID devices.
// Uses BTstack's HID Host for Classic BT HID devices (DualShock 4).

#include "btstack_host.h"

#include "btstack_config.h"
#include "bt_device_db.h"
// Include specific BTstack headers instead of umbrella btstack.h
// (btstack.h pulls in audio codecs which need sbc_encoder.h)
#include "btstack_defines.h"
#include "btstack_event.h"
#include "btstack_run_loop.h"

// Run loop depends on transport: embedded for USB dongle, async_context for CYW43,
// FreeRTOS for ESP32
#if !defined(BTSTACK_USE_CYW43) && !defined(BTSTACK_USE_ESP32) && !defined(BTSTACK_USE_NRF)
#include "btstack_run_loop_embedded.h"
#endif

// Declare btstack_memory_init - can't include btstack_memory.h due to HID conflicts
extern void btstack_memory_init(void);

#include "bluetooth_data_types.h"
#include "bluetooth_company_id.h"
#include "bluetooth_sdp.h"
#include "gap.h"
#include "hci.h"
#include "l2cap.h"
#include "classic/hid_host.h"
#ifndef HID_PROTOCOL_MODE_REPORT_WITH_FALLBACK_TO_BOOT
#define HID_PROTOCOL_MODE_REPORT_WITH_FALLBACK_TO_BOOT HID_PROTOCOL_MODE_REPORT
#endif
#include "classic/sdp_client.h"
#include "classic/sdp_server.h"
#include "classic/sdp_util.h"
#include "classic/device_id_server.h"

// Link key storage: TLV (flash) based for all builds
// USB dongle uses pico_flash_bank_instance(), CYW43/ESP32 use their own TLV setup
#if !defined(BTSTACK_USE_CYW43) && !defined(BTSTACK_USE_ESP32) && !defined(BTSTACK_USE_NRF)
#include "classic/btstack_link_key_db_tlv.h"
#include "btstack_tlv_flash_bank.h"
#include "pico/btstack_flash_bank.h"
#include "pico/flash.h"
#include "hardware/flash.h"
#endif

#include "btstack_tlv.h"
#include "hci_dump.h"
#include "hci_dump_embedded_stdout.h"

// BTHID callbacks - for classic BT HID devices
extern void bt_on_hid_ready(uint8_t conn_index);
extern void bt_on_disconnect(uint8_t conn_index);
extern void bt_on_hid_report(uint8_t conn_index, const uint8_t* data, uint16_t len);
extern void bt_on_get_report(uint8_t conn_index, const uint8_t* data, uint16_t len);
extern bool bthid_has_device(uint8_t conn_index);
extern bool bthid_device_has_driver(uint8_t conn_index);
extern void bthid_update_device_info(uint8_t conn_index, const char* name,
                                      uint16_t vendor_id, uint16_t product_id);
extern void bthid_set_battery_level(uint8_t conn_index, uint8_t level);
extern void bthid_set_battery(uint8_t conn_index, uint8_t level, bool charging);
extern void bthid_set_hid_descriptor(uint8_t conn_index, const uint8_t* desc, uint16_t desc_len);

// Platform HAL
extern void platform_reboot(void);

#include <stdio.h>
#include <string.h>

// For rumble feedback passthrough
// Note: manager.h includes tusb.h which conflicts with BTstack, so forward declare
extern int find_player_index(int dev_addr, int instance);
#include "core/services/players/feedback.h"

// ============================================================================
// FLASH HELPERS (for TLV storage)
// ============================================================================
#if !defined(BTSTACK_USE_CYW43) && !defined(BTSTACK_USE_ESP32) && !defined(BTSTACK_USE_NRF)
// Erase both BTstack flash banks (8KB total at end of flash)
static void __no_inline_not_in_flash_func(flash_erase_banks_func)(void* p) {
    (void)p;
    uint32_t flash_offset = PICO_FLASH_SIZE_BYTES - (FLASH_SECTOR_SIZE * 2);
    // Erase both 4KB sectors
    flash_range_erase(flash_offset, FLASH_SECTOR_SIZE * 2);
}

// Erase BTstack flash banks using flash_safe_execute
static void btstack_erase_flash_banks(void) {
    printf("[BTSTACK_HOST] Erasing BTstack flash banks at 0x%lX...\n",
           (unsigned long)(PICO_FLASH_SIZE_BYTES - (FLASH_SECTOR_SIZE * 2)));
    int result = flash_safe_execute(flash_erase_banks_func, NULL, UINT32_MAX);
    if (result == PICO_OK) {
        printf("[BTSTACK_HOST] Flash banks erased successfully\n");
    } else {
        printf("[BTSTACK_HOST] Flash erase failed: %d\n", result);
    }
}
#endif

// ============================================================================
// GLOBAL HOST STATE
// ============================================================================

static struct {
    bool initialized;
    bool powered_on;

    // HCI transport (provided by caller)
    const hci_transport_t* hci_transport;
} hid_state;

static btstack_packet_callback_registration_t hci_event_callback_registration;

// ============================================================================
// CLASSIC BT HID HOST STATE
// ============================================================================

#define MAX_CLASSIC_CONNECTIONS 4
#define INQUIRY_DURATION 5  // Inquiry duration in 1.28s units
#define CLASSIC_CONNECT_TIMEOUT_MS 15000  // Max time to establish HID connection

typedef struct {
    bool active;
    uint16_t hid_cid;           // BTstack HID connection ID
    bd_addr_t addr;
    char name[48];
    uint8_t class_of_device[3];
    uint16_t vendor_id;
    uint16_t product_id;
    bool hid_ready;
    const bt_device_profile_t* profile;
    uint32_t connect_time;      // When connection was initiated (for timeout detection)

    // Setup watchdog state: links that come up unidentified (name and VID/PID
    // still unknown) must be identified so a driver is selected. The DS4 only
    // sends its full report after the driver sends an output report, so an
    // unidentified link would otherwise sit in basic mode forever (white
    // lightbar, no input). See classic_setup_watchdog_task().
    uint8_t vidpid_attempts;    // PNP SDP query attempts for this link
    uint32_t last_vidpid_ms;    // when the last attempt was issued
    bool setup_stall_recovery;  // link already dropped once to force a retry
} classic_connection_t;

#if defined(CONFIG_DS5_DROP_SCREAM) || defined(CONFIG_DS4_SPEAKER_AUDIO)
#include "pico.h"             // __not_in_flash_func for the RAM-resident tap
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"

// Hard-fault black box: stash the faulting PC/LR in watchdog scratch
// registers (survive reboot) and reset. Reported ~5s after next boot so a
// reattached log listener catches it. scratch[4] = magic, [5] = PC, [6] = LR.
#define DS5_CRASH_MAGIC 0xDEADFA11u

void __not_in_flash_func(isr_hardfault)(void)
{
    uint32_t* sp;
    __asm volatile ("mrs %0, msp" : "=r"(sp));
    watchdog_hw->scratch[4] = DS5_CRASH_MAGIC;
    watchdog_hw->scratch[5] = sp[6];   // stacked PC
    watchdog_hw->scratch[6] = sp[5];   // stacked LR
    watchdog_reboot(0, 0, 10);
    while (1) { __asm volatile ("nop"); }
}

static uint32_t crash_report_pc, crash_report_lr;
static bool crash_report_pending;

// Queryable anytime via BT.STATUS ("crash_pc"): a one-shot boot print gets
// missed when no log listener is attached at the time.
void btstack_host_get_crash_info(uint32_t* pc, uint32_t* lr)
{
    *pc = crash_report_pc;
    *lr = crash_report_lr;
}
// HID interrupt-channel CIDs for direct l2cap_send() (DS5 audio streaming).
// BTstack delivers L2CAP_EVENT_CHANNEL_OPENED only to the owning service
// (hid_host), so we observe it via the public hci_dump interface instead:
// l2cap_emit_channel_opened() feeds every event through hci_dump (with
// ENABLE_LOG_BTSTACK_EVENTS) before dispatching. No BTstack modification.
#include "hci_dump.h"

static struct {
    bd_addr_t addr;
    uint16_t cid;
} hid_intr_cids[4];  // MAX_CLASSIC_CONNECTIONS

static void __not_in_flash_func(ds5_cid_tap_reset)(void) {}
static void __not_in_flash_func(ds5_cid_tap_log_message)(int log_level, const char* format,
                                                         va_list argptr)
{
    (void)log_level; (void)format; (void)argptr;
}
// RAM-resident and self-contained: this callback fires for every HCI event,
// including during BTstack link-key FLASH writes (pairing/auth). A
// flash-resident function executing in that window can hard-fault the chip
// (same class of bug as the Core1 flash-contention issues elsewhere in this
// repo) — the adapter rebooted mid-handshake. No printf, no libc, no
// flash-resident callees in here.
static void __not_in_flash_func(ds5_cid_tap_log_packet)(uint8_t packet_type, uint8_t in,
                                                        uint8_t* packet, uint16_t len)
{
    (void)in;
    if (packet_type != HCI_EVENT_PACKET || len < 4) return;

    if (packet[0] == L2CAP_EVENT_CHANNEL_OPENED && len >= 24) {
        // [2]=status [3..8]=addr(reversed) [11..12]=psm [13..14]=local_cid
        if (packet[2] != 0) return;
        uint16_t psm = (uint16_t)(packet[11] | (packet[12] << 8));
        if (psm != PSM_HID_INTERRUPT) return;
        uint16_t cid = (uint16_t)(packet[13] | (packet[14] << 8));
        uint8_t addr[6];
        for (int b = 0; b < 6; b++) addr[b] = packet[3 + 5 - b];
        int free_slot = -1;
        for (int ci = 0; ci < 4; ci++) {
            bool same = true;
            for (int b = 0; b < 6; b++) {
                if (hid_intr_cids[ci].addr[b] != addr[b]) { same = false; break; }
            }
            if (same) {
                hid_intr_cids[ci].cid = cid;
                return;
            }
            if (free_slot < 0 && hid_intr_cids[ci].cid == 0) free_slot = ci;
        }
        if (free_slot >= 0) {
            for (int b = 0; b < 6; b++) hid_intr_cids[free_slot].addr[b] = addr[b];
            hid_intr_cids[free_slot].cid = cid;
        }
    } else if (packet[0] == L2CAP_EVENT_CHANNEL_CLOSED) {
        // [2..3]=local_cid
        uint16_t cid = (uint16_t)(packet[2] | (packet[3] << 8));
        for (int ci = 0; ci < 4; ci++) {
            if (hid_intr_cids[ci].cid == cid) {
                hid_intr_cids[ci].cid = 0;
                for (int b = 0; b < 6; b++) hid_intr_cids[ci].addr[b] = 0;
            }
        }
    }
}

static const hci_dump_t ds5_cid_tap = {
    .reset = ds5_cid_tap_reset,
    .log_packet = ds5_cid_tap_log_packet,
    .log_message = ds5_cid_tap_log_message,
};

#endif

static struct {
    bool inquiry_active;
    bool use_liac;  // Alternate between GIAC and LIAC discovery
    classic_connection_t connections[MAX_CLASSIC_CONNECTIONS];
    // Pending incoming connection info (from HCI_EVENT_CONNECTION_REQUEST)
    bd_addr_t pending_addr;
    uint32_t pending_cod;
    char pending_name[48];
    uint16_t pending_vid;
    uint16_t pending_pid;
    bool pending_valid;
    uint32_t pending_start_time;
    bool pending_outgoing;  // True if we initiated the connection (hid_host_connect)
    hci_con_handle_t pending_acl_handle;  // ACL handle for pending incoming connection
    const bt_device_profile_t* pending_profile;
    // Pending HID connect (deferred until encryption completes)
    bd_addr_t pending_hid_addr;
    hci_con_handle_t pending_hid_handle;
    bool pending_hid_connect;
    // Set after outgoing HID fails — wait for device to reconnect incoming
    uint32_t waiting_for_incoming_time;  // 0 = not waiting
    // Connection timeout recovery
    uint32_t recovery_start_time;        // When recovery started (0 = no recovery pending)
} classic_state;

// ============================================================================
// DIRECT L2CAP STATE (Sony DS4 on Pico W)
// ============================================================================
// Used to bypass BTstack's hid_host layer for the initial Sony pairing:
// SDP responses from Sony controllers crash the CYW43 SPI bus, so on
// BTSTACK_USE_CYW43 we create the HID L2CAP channels directly instead.

#ifndef PSM_HID_CONTROL
#define PSM_HID_CONTROL   0x0011
#endif
#ifndef PSM_HID_INTERRUPT
#define PSM_HID_INTERRUPT 0x0013
#endif

typedef enum {
    DIRECT_L2CAP_STATE_IDLE,
    DIRECT_L2CAP_STATE_W4_CONTROL_CONNECTED,
    DIRECT_L2CAP_STATE_W4_INTERRUPT_CONNECTED,
    DIRECT_L2CAP_STATE_CONNECTED
} direct_l2cap_state_t;

typedef struct {
    bool active;
    direct_l2cap_state_t state;
    bd_addr_t addr;
    hci_con_handle_t acl_handle;
    uint16_t control_cid;
    uint16_t interrupt_cid;
    char name[48];
    uint8_t class_of_device[3];
    uint16_t vendor_id;
    uint16_t product_id;
    int conn_index;  // Index in classic_state.connections for bthid routing
    uint32_t last_rx_ms;  // Last packet received from the pad (link liveness)
} direct_l2cap_connection_t;

static direct_l2cap_connection_t direct_l2cap_conns[MAX_CLASSIC_CONNECTIONS];

static direct_l2cap_connection_t* find_direct_l2cap_by_handle(hci_con_handle_t handle) {
    if (handle == HCI_CON_HANDLE_INVALID) return NULL;
    for (int i = 0; i < MAX_CLASSIC_CONNECTIONS; i++) {
        if (direct_l2cap_conns[i].active && direct_l2cap_conns[i].acl_handle == handle) {
            return &direct_l2cap_conns[i];
        }
    }
    return NULL;
}

static direct_l2cap_connection_t* find_direct_l2cap_by_cid(uint16_t cid) {
    if (cid == 0) return NULL;
    for (int i = 0; i < MAX_CLASSIC_CONNECTIONS; i++) {
        if (direct_l2cap_conns[i].active &&
            (direct_l2cap_conns[i].control_cid == cid || direct_l2cap_conns[i].interrupt_cid == cid)) {
            return &direct_l2cap_conns[i];
        }
    }
    return NULL;
}

static direct_l2cap_connection_t* find_direct_l2cap_by_addr(const bd_addr_t addr) {
    for (int i = 0; i < MAX_CLASSIC_CONNECTIONS; i++) {
        if (direct_l2cap_conns[i].active && memcmp(direct_l2cap_conns[i].addr, addr, 6) == 0) {
            return &direct_l2cap_conns[i];
        }
    }
    return NULL;
}

static direct_l2cap_connection_t* find_direct_l2cap_by_conn_index(int conn_index) {
    if (conn_index < 0 || conn_index >= MAX_CLASSIC_CONNECTIONS) return NULL;
    for (int i = 0; i < MAX_CLASSIC_CONNECTIONS; i++) {
        if (direct_l2cap_conns[i].active && direct_l2cap_conns[i].conn_index == conn_index) {
            return &direct_l2cap_conns[i];
        }
    }
    return NULL;
}

static direct_l2cap_connection_t* find_free_direct_l2cap(void) {
    for (int i = 0; i < MAX_CLASSIC_CONNECTIONS; i++) {
        if (!direct_l2cap_conns[i].active) {
            return &direct_l2cap_conns[i];
        }
    }
    return NULL;
}

// A pad that streams normally reports every ~4ms. Silence past this means the
// link is gone even though no HCI/L2CAP event ever reached us (CYW43 hiccup),
// and without this the connection slot would stay allocated forever.
#define DIRECT_L2CAP_RX_TIMEOUT_MS 10000

// Tear a specific direct-L2CAP link down completely:
//   - tell the HID driver the pad is gone, so the router stops publishing the
//     last input state (the game would keep running with a frozen controller);
//   - release the Classic connection slot, so the connection count drops and
//     page scan comes back when slots are free.
static void direct_l2cap_teardown_conn(direct_l2cap_connection_t* dl, const char* reason)
{
    if (!dl || !dl->active) {
        return;
    }

    int ci = dl->conn_index;
    printf("[BTSTACK_HOST] Direct L2CAP teardown (%s), conn_index=%d\n", reason, ci);

    memset(dl, 0, sizeof(*dl));
    dl->acl_handle = HCI_CON_HANDLE_INVALID;
    dl->conn_index = -1;  /* memset() would leave it at slot 0 */

    if (ci >= 0 && ci < MAX_CLASSIC_CONNECTIONS) {
        bt_on_disconnect((uint8_t)ci);
        memset(&classic_state.connections[ci], 0, sizeof(classic_connection_t));
    }

    classic_state.pending_valid = false;
    classic_state.pending_hid_connect = false;

    if (btstack_classic_get_connection_count() == 0) {
        gap_discoverable_control(1);
        gap_connectable_control(1);
    }
}

static void direct_l2cap_teardown(const char* reason)
{
    for (int i = 0; i < MAX_CLASSIC_CONNECTIONS; i++) {
        if (direct_l2cap_conns[i].active) {
            direct_l2cap_teardown_conn(&direct_l2cap_conns[i], reason);
        }
    }
}

// Deferred Classic ACL connect: the HCI command channel can be busy when we
// want to dial (e.g. the Inquiry Cancel command was just sent), so remember the
// address and let btstack_host_process() retry.
static struct {
    bool pending;
    bd_addr_t addr;
} classic_connect_deferred;

// Classic ACL connect helper. BTstack's public gap_connect() is not part of a
// Classic-only build, so request the ACL link the same way l2cap.c does for an
// outgoing Classic channel: hci.c intercepts this command and creates/tracks
// the connection record. Used only by the Sony direct-L2CAP path below.
static uint8_t btstack_classic_connect_acl(const bd_addr_t addr)
{
    if (!hci_can_send_command_packet_now()) {
        memcpy(classic_connect_deferred.addr, addr, 6);
        classic_connect_deferred.pending = true;
        return ERROR_CODE_SUCCESS;  // in progress: retried from btstack_host_process()
    }
    uint8_t status = hci_send_cmd(&hci_create_connection, addr,
                                  hci_usable_acl_packet_types(), 0, 0, 0,
                                  hci_get_allow_role_switch());
    // Callers tolerate COMMAND_DISALLOWED (link already being set up), so
    // normalize the "already exists" answer to the same code.
    if (status == ERROR_CODE_ACL_CONNECTION_ALREADY_EXISTS) {
        return ERROR_CODE_COMMAND_DISALLOWED;
    }
    return status;
}

// Forward declaration
static void direct_l2cap_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

// SDP query state
static uint8_t sdp_attribute_value[32];
static const uint16_t sdp_attribute_value_buffer_size = sizeof(sdp_attribute_value);

// Classic HID descriptor storage
static uint8_t classic_hid_descriptor_storage[512];

// SDP Device ID record buffer (needed for DS4/DS5 reconnection)
static uint8_t device_id_sdp_service_buffer[100];

// Find classic connection by hid_cid
static classic_connection_t* find_classic_connection_by_cid(uint16_t hid_cid) {
    for (int i = 0; i < MAX_CLASSIC_CONNECTIONS; i++) {
        if (classic_state.connections[i].active && classic_state.connections[i].hid_cid == hid_cid) {
            return &classic_state.connections[i];
        }
    }
    return NULL;
}

// Get conn_index for classic connection
static int get_classic_conn_index(uint16_t hid_cid) {
    for (int i = 0; i < MAX_CLASSIC_CONNECTIONS; i++) {
        if (classic_state.connections[i].active && classic_state.connections[i].hid_cid == hid_cid) {
            return i;  // conn_index matches array index
        }
    }
    return -1;
}

// Find free classic connection slot
static classic_connection_t* find_free_classic_connection(void) {
    for (int i = 0; i < MAX_CLASSIC_CONNECTIONS; i++) {
        if (!classic_state.connections[i].active) {
            return &classic_state.connections[i];
        }
    }
    return NULL;
}

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void hid_host_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void classic_setup_watchdog_task(void);
static void sdp_query_vid_pid_callback(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

// ============================================================================
// INITIALIZATION
// ============================================================================

// Internal function to set up HID handlers (used by both init paths)
static void setup_hid_handlers(void)
{
    printf("[BTSTACK_HOST] Init L2CAP...\n");
    l2cap_init();

    // Initialize classic BT HID Host
    printf("[BTSTACK_HOST] Init Classic HID Host...\n");
    memset(&classic_state, 0, sizeof(classic_state));
    memset(direct_l2cap_conns, 0, sizeof(direct_l2cap_conns));
    for (int i = 0; i < MAX_CLASSIC_CONNECTIONS; i++) {
        direct_l2cap_conns[i].acl_handle = HCI_CON_HANDLE_INVALID;
        direct_l2cap_conns[i].conn_index = -1;
    }
    // Set security level BEFORE hid_host_init (it registers L2CAP services with this level)
    gap_set_security_level(LEVEL_0);  // DS3 doesn't support SSP
    hid_host_init(classic_hid_descriptor_storage, sizeof(classic_hid_descriptor_storage));
    hid_host_register_packet_handler(hid_host_packet_handler);

    // SDP server - needed for DS4/DS5 reconnection (they query Device ID)
    sdp_init();
    device_id_create_sdp_record(device_id_sdp_service_buffer, 0x10003,
                                DEVICE_ID_VENDOR_ID_SOURCE_BLUETOOTH,
                                BLUETOOTH_COMPANY_ID_BLUEKITCHEN_GMBH, 1, 1);
    sdp_register_service(device_id_sdp_service_buffer);
    printf("[BTSTACK_HOST] SDP server initialized\n");

    // Classic BT link policy: only enable role switch.
    // Disabling sniff mode prevents periodic link stalls / latency spikes (~700-800ms) on Classic BT gamepads.
    gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_ROLE_SWITCH);

    // Register for HCI events
    printf("[BTSTACK_HOST] Register event handlers...\n");
    hci_event_callback_registration.callback = packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    hid_state.initialized = true;
    printf("[BTSTACK_HOST] HID handlers initialized (Classic)\n");
}

// btstack_host_init is only used for USB dongle transport
// For CYW43/ESP32, use btstack_host_init_hid_handlers() after external BTstack init
#if !defined(BTSTACK_USE_CYW43) && !defined(BTSTACK_USE_ESP32) && !defined(BTSTACK_USE_NRF)

// TLV context for flash-based link key storage (must be static/persistent)
static btstack_tlv_flash_bank_t btstack_tlv_flash_bank_context;

// Set up TLV (flash) storage for persistent link keys
static void setup_tlv_storage(void) {
    printf("[BTSTACK_HOST] Setting up flash-based TLV storage...\n");

    // Check for corrupted flash banks and erase if needed
    // Flash bank 0 starts at end of flash - 8KB
    uint32_t bank0_offset = PICO_FLASH_SIZE_BYTES - (FLASH_SECTOR_SIZE * 2);
    const uint8_t* bank0_ptr = (const uint8_t*)(XIP_BASE + bank0_offset);

    // BTstack TLV expects clean flash (0xFF) or valid header
    // If we see our debug pattern (0xDEADBEEF) or other garbage, erase
    bool needs_erase = false;
    if (bank0_ptr[0] == 0xDE && bank0_ptr[1] == 0xAD &&
        bank0_ptr[2] == 0xBE && bank0_ptr[3] == 0xEF) {
        printf("[BTSTACK_HOST] Detected corrupted flash bank (debug pattern)\n");
        needs_erase = true;
    }

    if (needs_erase) {
        btstack_erase_flash_banks();
    }

    // Get the Pico SDK flash bank HAL instance
    const hal_flash_bank_t *hal_flash_bank_impl = pico_flash_bank_instance();
    printf("[BTSTACK_HOST] Flash bank instance: %p\n", hal_flash_bank_impl);

    // Initialize BTstack TLV with flash bank
    const btstack_tlv_t *btstack_tlv_impl = btstack_tlv_flash_bank_init_instance(
            &btstack_tlv_flash_bank_context,
            hal_flash_bank_impl,
            NULL);
    printf("[BTSTACK_HOST] TLV instance: %p\n", btstack_tlv_impl);

    if (!btstack_tlv_impl) {
        printf("[BTSTACK_HOST] ERROR: TLV init failed!\n");
        return;
    }

    // Set global TLV instance
    btstack_tlv_set_instance(btstack_tlv_impl, &btstack_tlv_flash_bank_context);

    // Set up Classic BT link key storage using TLV
    const btstack_link_key_db_t *btstack_link_key_db = btstack_link_key_db_tlv_get_instance(
            btstack_tlv_impl, &btstack_tlv_flash_bank_context);
    printf("[BTSTACK_HOST] Link key DB instance: %p\n", btstack_link_key_db);

    if (!btstack_link_key_db) {
        printf("[BTSTACK_HOST] ERROR: Link key DB init failed!\n");
        return;
    }

    hci_set_link_key_db(btstack_link_key_db);
    printf("[BTSTACK_HOST] Classic BT link key DB configured (flash)\n");

    // Debug: check bank state
    printf("[BTSTACK_HOST] TLV context: current_bank=%d write_offset=0x%lX\n",
           btstack_tlv_flash_bank_context.current_bank,
           (unsigned long)btstack_tlv_flash_bank_context.write_offset);
}

void btstack_host_init(const void* transport)
{
    if (hid_state.initialized) {
        printf("[BTSTACK_HOST] Already initialized\n");
        return;
    }

    if (!transport) {
        printf("[BTSTACK_HOST] ERROR: No HCI transport provided\n");
        return;
    }

    printf("[BTSTACK_HOST] Initializing BTstack...\n");

    memset(&hid_state, 0, sizeof(hid_state));
    hid_state.hci_transport = (const hci_transport_t*)transport;

    // HCI dump disabled - too verbose (logs every ACL packet)
    // printf("[BTSTACK_HOST] Init HCI dump (for logging)...\n");
    // hci_dump_init(hci_dump_embedded_stdout_get_instance());

#if defined(CONFIG_DS5_DROP_SCREAM) || defined(CONFIG_DS4_SPEAKER_AUDIO)
    // Silent hci_dump tap: observes L2CAP_EVENT_CHANNEL_OPENED/CLOSED to learn
    // HID interrupt CIDs for direct audio sends (no BTstack modification)
    hci_dump_init(&ds5_cid_tap);
#endif

    printf("[BTSTACK_HOST] Init memory pools...\n");
    btstack_memory_init();

    printf("[BTSTACK_HOST] Init run loop...\n");
    btstack_run_loop_init(btstack_run_loop_embedded_get_instance());

    printf("[BTSTACK_HOST] Init HCI with provided transport...\n");
    hci_init(transport, NULL);

    // Set up flash-based TLV storage for persistent link keys
    setup_tlv_storage();

    // Set up HID handlers
    setup_hid_handlers();
    printf("[BTSTACK_HOST] Initialized OK\n");
}
#endif

void btstack_host_init_hid_handlers(void)
{
    if (hid_state.initialized) {
        printf("[BTSTACK_HOST] HID handlers already initialized\n");
        return;
    }

    printf("[BTSTACK_HOST] Initializing HID handlers (BTstack already initialized externally)...\n");

    memset(&hid_state, 0, sizeof(hid_state));
    // Note: hci_transport is not set here since BTstack was initialized externally

    // Set up HID handlers (BTstack core already initialized by btstack_cyw43_init or similar)
    setup_hid_handlers();

#if defined(CONFIG_DS5_DROP_SCREAM) || defined(CONFIG_DS4_SPEAKER_AUDIO)
    // Silent hci_dump tap: observes L2CAP_EVENT_CHANNEL_OPENED/CLOSED to learn
    // HID interrupt CIDs for direct audio sends (no BTstack modification).
    // NOTE: must be here — btstack_host_init() is USB-dongle-transport only.
    hci_dump_init(&ds5_cid_tap);
    printf("[BTSTACK_HOST] DS audio CID tap registered\n");

    if (watchdog_hw->scratch[4] == DS5_CRASH_MAGIC) {
        crash_report_pc = watchdog_hw->scratch[5];
        crash_report_lr = watchdog_hw->scratch[6];
        crash_report_pending = true;
        watchdog_hw->scratch[4] = 0;
        printf("[CRASH] Previous boot HardFault PC=0x%08lx LR=0x%08lx\n",
               (unsigned long)crash_report_pc, (unsigned long)crash_report_lr);
    }
#endif

    printf("[BTSTACK_HOST] HID handlers initialized OK\n");
}

void btstack_host_power_on(void)
{
    printf("[BTSTACK_HOST] power_on called, initialized=%d\n", hid_state.initialized);

    if (!hid_state.initialized) {
        printf("[BTSTACK_HOST] ERROR: Not initialized\n");
        return;
    }

    printf("[BTSTACK_HOST] HCI state before power_on: %d\n", hci_get_state());
    printf("[BTSTACK_HOST] Calling hci_power_control(HCI_POWER_ON)...\n");
    int err = hci_power_control(HCI_POWER_ON);
    printf("[BTSTACK_HOST] hci_power_control returned %d, state now: %d\n", err, hci_get_state());
}

// ============================================================================
// SCANNING
// ============================================================================

static uint32_t scan_timeout_end = 0;  // 0 = no timeout (indefinite scan)
static bool scan_suppressed = false;   // App can suppress auto-restart (e.g. USB device connected)

void btstack_host_start_scan(void)
{
    if (scan_suppressed) {
        return;  // App suppressed scanning (e.g. BT host disabled)
    }

    if (!hid_state.powered_on) {
        printf("[BTSTACK_HOST] Not powered on yet\n");
        return;
    }

    if (classic_state.inquiry_active) {
        return;  // Already scanning
    }

    // Cap scan duration to 15 seconds if called without explicit timeout.
    // Prevents indefinite background inquiry from starving controller audio bandwidth.
    if (scan_timeout_end == 0) {
        scan_timeout_end = btstack_run_loop_get_time_ms() + 15000;
    }

    // Alternate between GIAC (general) and LIAC (limited): some controllers only
    // are discoverable in limited inquiry mode while their pairing button is pressed
    uint32_t lap = classic_state.use_liac ? GAP_IAC_LIMITED_INQUIRY : GAP_IAC_GENERAL_INQUIRY;
    printf("[BTSTACK_HOST] Starting Classic inquiry (LAP=%s)...\n",
           classic_state.use_liac ? "LIAC" : "GIAC");
    gap_inquiry_set_lap(lap);
    gap_inquiry_start(INQUIRY_DURATION);
    classic_state.inquiry_active = true;
}

void btstack_host_stop_scan(void)
{
    scan_timeout_end = 0;

    if (classic_state.inquiry_active) {
        printf("[BTSTACK_HOST] Stopping Classic inquiry\n");
        gap_inquiry_stop();
        classic_state.inquiry_active = false;
    }
}

void btstack_host_start_timed_scan(uint32_t timeout_ms)
{
    // If pending connection has been in flight for > 3 seconds, consider it stale
    if (classic_state.pending_valid &&
        (btstack_run_loop_get_time_ms() - classic_state.pending_start_time) > 3000) {
        printf("[BTSTACK_HOST] Clearing stale pending connection (>3s)\n");
        classic_state.pending_valid = false;
        classic_state.pending_hid_connect = false;
    }
    // Explicit user scan clears waiting-for-incoming block
    classic_state.waiting_for_incoming_time = 0;

    // Never start inquiry over an in-flight Classic connection setup (e.g.
    // button pressed while a controller is actively mid-handshake within 3s):
    // inquiry starves the LMP encryption exchange.
    if (classic_state.pending_valid) {
        printf("[BTSTACK_HOST] Timed scan ignored: Classic connection setup in progress\n");
        return;
    }

    scan_suppressed = false;  // Explicit scan request clears suppression
    scan_timeout_end = btstack_run_loop_get_time_ms() + timeout_ms;
    printf("[BTSTACK_HOST] Starting timed scan (%lums)\n", (unsigned long)timeout_ms);
    btstack_host_start_scan();
}

void btstack_host_suppress_scan(bool suppress)
{
    scan_suppressed = suppress;
    if (suppress && btstack_host_is_scanning()) {
        btstack_host_stop_scan();
    }
}

// ============================================================================
// MAIN LOOP
// ============================================================================


// Transport-specific process function (weak, overridden by transport)
__attribute__((weak)) void btstack_host_transport_process(void) {
    // Default: no-op, transport should override
}

// ============================================================================
// CLASSIC SETUP WATCHDOG
// ============================================================================

// How often the unidentified-link recovery may retry the VID/PID SDP query,
// how many attempts per link, and how long a link may stay without a selected
// driver before it is dropped for a clean retry.
#define CLASSIC_VIDPID_RETRY_MS      1500
#define CLASSIC_VIDPID_MAX_ATTEMPTS  6
#define CLASSIC_SETUP_STALL_MS       6000

// Recover classic links that came up unidentified. On a fresh boot the DS4's
// name is unknown and no inquiry has run, so the driver can only be selected
// with the VID/PID from the PNP SDP query. That query races with the HID
// descriptor query of another controller (BTstack's SDP client serves one
// query at a time) and can be lost. Without identification the driver never
// sends the output report that switches the DS4 to its full report mode, so
// the controller sits in basic mode forever: white lightbar, no input.
static void classic_setup_watchdog_task(void)
{
    uint32_t now = btstack_run_loop_get_time_ms();
    static uint8_t stall_drops = 0;

    for (int i = 0; i < MAX_CLASSIC_CONNECTIONS; i++) {
        classic_connection_t* conn = &classic_state.connections[i];
        if (!conn->active || !conn->hid_ready) continue;
        if (conn->hid_cid == 0xFFFF) continue;   // direct L2CAP (Sony): own flow

        bool has_device = bthid_has_device((uint8_t)i);
        bool has_driver = bthid_device_has_driver((uint8_t)i);
        bool unidentified = (conn->vendor_id == 0 && conn->product_id == 0 &&
                             conn->name[0] == '\0');

        // 1) No driver was ever selected: the HID setup stalled, or the link
        // is unidentified and the PNP query cannot succeed. Drop the ACL once
        // so the controller reconnects and starts over. Globally capped so a
        // device that can never be identified cannot loop forever.
        if (!has_driver && (!has_device || unidentified) &&
            !conn->setup_stall_recovery && stall_drops < 4 &&
            (now - conn->connect_time) > CLASSIC_SETUP_STALL_MS) {
            conn->setup_stall_recovery = true;
            stall_drops++;
            printf("[BTSTACK_HOST] Classic link %d stuck unidentified - dropping for clean retry\n", i);
            hci_connection_t* hci_conn = hci_connection_for_bd_addr_and_type(conn->addr, BD_ADDR_TYPE_ACL);
            if (hci_conn) {
                gap_disconnect(hci_conn->con_handle);
            }
            continue;
        }

        // 2) Device waiting for identification: retry the PNP SDP query, but
        // only while the SDP client is free so we never steal it from
        // BTstack's own HID descriptor query for another controller.
        if (has_device && !has_driver &&
            conn->vendor_id == 0 && conn->product_id == 0 &&
            conn->vidpid_attempts < CLASSIC_VIDPID_MAX_ATTEMPTS &&
            (now - conn->last_vidpid_ms) >= CLASSIC_VIDPID_RETRY_MS &&
            sdp_client_ready()) {
            conn->vidpid_attempts++;
            conn->last_vidpid_ms = now;
            memcpy(classic_state.pending_addr, conn->addr, 6);
            classic_state.pending_vid = 0;
            classic_state.pending_pid = 0;
            printf("[BTSTACK_HOST] Retrying VID/PID SDP query for link %d (attempt %d)\n",
                   i, conn->vidpid_attempts);
            sdp_client_query_uuid16(&sdp_query_vid_pid_callback, conn->addr,
                                    BLUETOOTH_SERVICE_CLASS_PNP_INFORMATION);
        }
    }
}

void btstack_host_process(void)
{
    if (!hid_state.initialized) return;

    // Process transport-specific tasks (e.g., USB polling, CYW43 async context)
    btstack_host_transport_process();

#ifdef CONFIG_DS5_DROP_SCREAM
    // Re-announce last crash after log listeners have had time to reattach
    if (crash_report_pending &&
        btstack_run_loop_get_time_ms() > 6000) {
        crash_report_pending = false;
        printf("[CRASH] !!! Previous boot HardFault PC=0x%08lx LR=0x%08lx — addr2line these !!!\n",
               (unsigned long)crash_report_pc, (unsigned long)crash_report_lr);
    }
#endif


#if !defined(BTSTACK_USE_CYW43) && !defined(BTSTACK_USE_ESP32) && !defined(BTSTACK_USE_NRF)
    // Process BTstack run loop multiple times to let packets flow through HCI->L2CAP
    // Note: CYW43 uses async_context, ESP32 uses FreeRTOS run loop - both process automatically
    for (int i = 0; i < 5; i++) {
        btstack_run_loop_embedded_execute_once();
    }
#endif

    // Recover classic links stuck without an identified driver (DS4 white light)
    classic_setup_watchdog_task();

    // Direct-L2CAP link watchdog. The DS4 streams every ~4ms, so prolonged
    // silence means the link is dead even when no disconnect event ever reached
    // us (a CYW43 hiccup loses it silently). Without this the connection slot
    // stayed allocated, the connection count never reached zero, page scan was
    // never re-enabled and the pad could not reconnect until the Pico was
    // unplugged.
    for (int i = 0; i < MAX_CLASSIC_CONNECTIONS; i++) {
        if (direct_l2cap_conns[i].active &&
            direct_l2cap_conns[i].state == DIRECT_L2CAP_STATE_CONNECTED &&
            (btstack_run_loop_get_time_ms() - direct_l2cap_conns[i].last_rx_ms) >= DIRECT_L2CAP_RX_TIMEOUT_MS) {
            printf("[BTSTACK_HOST] Direct L2CAP [%d]: no uplink for %dms\n", i, DIRECT_L2CAP_RX_TIMEOUT_MS);
            direct_l2cap_teardown_conn(&direct_l2cap_conns[i], "uplink timeout");
        }
    }


    // Check scan timeout
    if (scan_timeout_end > 0 && btstack_host_is_scanning()) {
        if (btstack_run_loop_get_time_ms() >= scan_timeout_end) {
            printf("[BTSTACK_HOST] Timed scan expired\n");
            scan_timeout_end = 0;
            btstack_host_stop_scan();
        }
    }

    // Timeout for "waiting for incoming reconnection" after outgoing Classic HID failure.
    // If the device doesn't reconnect within 30s, give up and resume scanning.
    if (classic_state.waiting_for_incoming_time != 0 &&
        (btstack_run_loop_get_time_ms() - classic_state.waiting_for_incoming_time) >= 30000) {
        printf("[BTSTACK_HOST] Incoming reconnection timeout, resuming scan\n");
        classic_state.waiting_for_incoming_time = 0;
    }

    // Classic connection establishment timeout.
    // If a connection doesn't reach hid_ready within CLASSIC_CONNECT_TIMEOUT_MS,
    // something went wrong (e.g., CYW43 SPI bus failure during SSP pairing,
    // incompatible device, or stuck SDP query). Clean up and try to recover.
    for (int i = 0; i < MAX_CLASSIC_CONNECTIONS; i++) {
        classic_connection_t* conn = &classic_state.connections[i];
        if (conn->active && !conn->hid_ready && conn->connect_time != 0 &&
            (btstack_run_loop_get_time_ms() - conn->connect_time) >= CLASSIC_CONNECT_TIMEOUT_MS) {
            printf("[BTSTACK_HOST] Classic connection timeout after %lums (slot %d '%s'), cleaning up\n",
                   (unsigned long)(btstack_run_loop_get_time_ms() - conn->connect_time), i, conn->name);

            // Try to disconnect (may fail if BT transport is dead)
            if (conn->hid_cid != 0 && conn->hid_cid != 0xFFFF) {
                hid_host_disconnect(conn->hid_cid);
            }

            // Clean up direct-L2CAP state if this was a direct-L2CAP connection
            direct_l2cap_connection_t* dl = find_direct_l2cap_by_addr(conn->addr);
            if (dl) {
                direct_l2cap_teardown_conn(dl, "setup timeout");
            }

            // Clean up connection slot
            memset(conn, 0, sizeof(*conn));
            classic_state.pending_valid = false;
            classic_state.pending_hid_connect = false;

            // Start recovery timer and keep host connectable
            classic_state.recovery_start_time = btstack_run_loop_get_time_ms();
            gap_discoverable_control(1);
            gap_connectable_control(1);
            break;  // Only handle one timeout per tick
        }
    }

    // Recovery watchdog: if we cleaned up a stuck connection but BT transport
    // appears dead, force a reboot. Two guards keep this from nuking live
    // sessions (it caused mid-session "stealth reboots", crash_pc=0):
    //   - any active Classic link proves the transport works — a
    //     stalled single connection setup is not a dead radio;
    //   - a GIAC inquiry takes ~10.24s, so a 10.0s deadline rebooted before
    //     the all-clear (GAP_EVENT_INQUIRY_COMPLETE) could ever land. 20s
    //     gives the inquiry room to finish.
    if (classic_state.recovery_start_time != 0 &&
        (btstack_run_loop_get_time_ms() - classic_state.recovery_start_time) >= 20000) {
        bool any_link = btstack_classic_get_connection_count() > 0;
        if (any_link) {
            classic_state.recovery_start_time = 0;   // transport demonstrably alive
        } else {
            printf("[BTSTACK_HOST] No BT activity after connection timeout recovery, rebooting\n");
            platform_reboot();
        }
    }

    // Safety net: if idle with no active connections and not scanning, keep host connectable
    // for incoming reconnection via Page Scan (e.g. user pressing PS button on DS4).
    // Do NOT start active inquiry/scanning unprompted, as inquiry consumes radio bandwidth
    // and causes audio stuttering on the controller speaker. Pairing scan is triggered via BOOTSEL.
    if (hid_state.powered_on &&
        !scan_suppressed &&
        !classic_state.inquiry_active &&
        classic_state.waiting_for_incoming_time == 0 &&
        !classic_state.pending_valid &&
        btstack_classic_get_connection_count() == 0) {
        gap_discoverable_control(1);
        gap_connectable_control(1);
    }
}

// ============================================================================
// SDP QUERY CALLBACK (for VID/PID detection)
// ============================================================================

static void sdp_query_vid_pid_callback(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t event_type = hci_event_packet_get_type(packet);

    switch (event_type) {
        case SDP_EVENT_QUERY_ATTRIBUTE_VALUE: {
            uint16_t attr_len = sdp_event_query_attribute_byte_get_attribute_length(packet);
            if (attr_len <= sdp_attribute_value_buffer_size) {
                uint16_t offset = sdp_event_query_attribute_byte_get_data_offset(packet);
                sdp_attribute_value[offset] = sdp_event_query_attribute_byte_get_data(packet);

                // Check if we got all bytes for this attribute
                if (offset + 1 == attr_len) {
                    uint16_t attr_id = sdp_event_query_attribute_byte_get_attribute_id(packet);
                    uint16_t value;
                    if (de_element_get_uint16(sdp_attribute_value, &value)) {
                        if (attr_id == BLUETOOTH_ATTRIBUTE_VENDOR_ID) {
                            classic_state.pending_vid = value;
                            printf("[BTSTACK_HOST] SDP VID: 0x%04X\n", value);
                        } else if (attr_id == BLUETOOTH_ATTRIBUTE_PRODUCT_ID) {
                            classic_state.pending_pid = value;
                            printf("[BTSTACK_HOST] SDP PID: 0x%04X\n", value);
                        }
                    }
                }
            }
            break;
        }
        case SDP_EVENT_QUERY_COMPLETE:
            printf("[BTSTACK_HOST] SDP query complete: VID=0x%04X PID=0x%04X\n",
                   classic_state.pending_vid, classic_state.pending_pid);

            // Update the connection struct with VID/PID
            if (classic_state.pending_vid || classic_state.pending_pid) {
                for (int i = 0; i < MAX_CLASSIC_CONNECTIONS; i++) {
                    classic_connection_t* conn = &classic_state.connections[i];
                    if (conn->active && memcmp(conn->addr, classic_state.pending_addr, 6) == 0) {
                        conn->vendor_id = classic_state.pending_vid;
                        conn->product_id = classic_state.pending_pid;
                        printf("[BTSTACK_HOST] Updated conn[%d] VID/PID: 0x%04X/0x%04X\n",
                               i, conn->vendor_id, conn->product_id);

                        // Notify bthid to re-evaluate driver selection with new VID/PID
                        bthid_update_device_info(i, conn->name,
                                                  classic_state.pending_vid,
                                                  classic_state.pending_pid);

                        // Re-send HID descriptor in case driver was re-evaluated to generic
                        // (descriptor was delivered earlier but ignored by the previous driver)
                        const uint8_t* hid_desc = hid_descriptor_storage_get_descriptor_data(conn->hid_cid);
                        uint16_t hid_desc_len = hid_descriptor_storage_get_descriptor_len(conn->hid_cid);
                        if (hid_desc && hid_desc_len > 0) {
                            bthid_set_hid_descriptor(i, hid_desc, hid_desc_len);
                        }
                        break;
                    }
                }

                // Also update direct_l2cap_conn if active and address matches
                direct_l2cap_connection_t* dl_sdp = find_direct_l2cap_by_addr(classic_state.pending_addr);
                if (dl_sdp && dl_sdp->active) {
                    dl_sdp->vendor_id = classic_state.pending_vid;
                    dl_sdp->product_id = classic_state.pending_pid;
                    printf("[BTSTACK_HOST] Updated direct-L2CAP VID/PID: 0x%04X/0x%04X\n",
                           dl_sdp->vendor_id, dl_sdp->product_id);
                }
            }
            break;
    }
}

// ============================================================================
// HCI EVENT HANDLER
// ============================================================================

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t event_type = hci_event_packet_get_type(packet);

    // Debug: log key HCI events (pairing/encryption)
    // 0x04=CONNECTION_COMPLETE, 0x05=DISCONNECTION_COMPLETE, 0x06=AUTH_COMPLETE
    // 0x08=ENCRYPTION_CHANGE, 0x17=LINK_KEY_REQUEST, 0x18=LINK_KEY_NOTIFICATION
    // 0x16=PIN_CODE_REQUEST, 0x04=CONNECTION_REQUEST (offset differs)
    if (event_type == 0x17 || event_type == 0x18 || event_type == 0x06 ||
        event_type == 0x08 || event_type == 0x16) {
        printf("[BTSTACK_HOST] >>> HCI Event 0x%02X (size=%d)\n", event_type, size);
    }

    switch (event_type) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                printf("[BTSTACK_HOST] HCI working\n");
                hid_state.powered_on = true;

                // Reset scan state (in case of reconnect)
                classic_state.inquiry_active = false;

#if !defined(BTSTACK_USE_ESP32) && !defined(BTSTACK_USE_NRF)
                // Set master role policy for incoming Classic connections
                // Some Classic controllers require the host to be master
                hci_set_master_slave_policy(0);  // 0 = always try to become master
                printf("[BTSTACK_HOST] Set master role policy\n");
#endif

                // Print our local BD_ADDR
                bd_addr_t local_addr;
                gap_local_bd_addr(local_addr);
                printf("[BTSTACK_HOST] Local BD_ADDR: %02X:%02X:%02X:%02X:%02X:%02X\n",
                       local_addr[0], local_addr[1], local_addr[2],
                       local_addr[3], local_addr[4], local_addr[5]);

                // Print chip info (see hci_transport_h2_tinyusb.h for dongle compatibility guide)
                uint16_t manufacturer = hci_get_manufacturer();
                printf("[BTSTACK_HOST] Chip Manufacturer: 0x%04X", manufacturer);
                switch (manufacturer) {
                    case 0x000A: printf(" (CSR) - OK\n"); break;
                    case 0x000D: printf(" (TI)\n"); break;
                    case 0x000F: printf(" (Broadcom) - OK\n"); break;
                    case 0x001D: printf(" (Qualcomm)\n"); break;
                    case 0x0046: printf(" (MediaTek)\n"); break;
                    case 0x005D: printf(" (Realtek) - NEEDS FIRMWARE!\n"); break;
                    case 0x0002: printf(" (Intel)\n"); break;
                    default: printf("\n"); break;
                }

                // Set local name (for devices that want to see us)
                gap_set_local_name("REVOLUTIONDS4");

                // Enable bonding (needed for Classic)
                gap_set_bondable_mode(1);
                // Set IO capability for "just works" pairing (no PIN required)
                gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);

#if !defined(BTSTACK_USE_ESP32) && !defined(BTSTACK_USE_NRF)
                // Classic BT setup (not available on ESP32-S3/nRF)
                // Set class of device to Computer (Desktop Workstation)
                gap_set_class_of_device(0x000104);  // Major: Computer, Minor: Desktop

                // Enable SSP (Secure Simple Pairing) on the controller
                extern const hci_cmd_t hci_write_simple_pairing_mode;
                hci_send_cmd(&hci_write_simple_pairing_mode, 1);

                // Request bonding during SSP (required for BTstack to store link keys!)
                gap_ssp_set_authentication_requirement(SSP_IO_AUTHREQ_MITM_PROTECTION_NOT_REQUIRED_DEDICATED_BONDING);
                // Auto-accept incoming SSP pairing requests
                gap_ssp_set_auto_accept(1);

                // Make host discoverable and connectable for incoming connections
                // Required for Sony controllers (DS3, DS4, DS5) which initiate connections
                gap_discoverable_control(1);
                gap_connectable_control(1);
                gap_set_page_scan_type(PAGE_SCAN_MODE_STANDARD);
                // Standard Page Scan: 640ms interval, 11.25ms window (R1 mode).
                // Compliant with CYW43439 coexistence and protects audio streaming bandwidth.
                gap_set_page_scan_activity(0x0400, 0x0012);
#endif

                // Do not start continuous scan on boot. Page Scan is already enabled above
                // so previously paired controllers (like DS4) connect instantly on pressing PS.
                // Pairing scan (Inquiry) is started on-demand via the BOOTSEL button.
            }
            break;

        // Classic BT inquiry result
        case GAP_EVENT_INQUIRY_RESULT: {
            bd_addr_t addr;
            gap_event_inquiry_result_get_bd_addr(packet, addr);
            uint32_t cod = gap_event_inquiry_result_get_class_of_device(packet);

            // Parse name from extended inquiry response if available
            char name[240] = {0};
            if (gap_event_inquiry_result_get_name_available(packet)) {
                int name_len = gap_event_inquiry_result_get_name_len(packet);
                if (name_len > 0 && name_len < (int)sizeof(name)) {
                    memcpy(name, gap_event_inquiry_result_get_name(packet), name_len);
                    name[name_len] = 0;
                }
            }

            // Class of Device: Major=0x05 (Peripheral), Minor bits indicate type
            uint8_t major_class = (cod >> 8) & 0x1F;
            uint8_t minor_class = (cod >> 2) & 0x3F;
            bool is_gamepad = (major_class == 0x05) && ((minor_class & 0x0F) == 0x02);  // Gamepad
            bool is_joystick = (major_class == 0x05) && ((minor_class & 0x0F) == 0x01); // Joystick

            // Identify device by name
            const bt_device_profile_t* profile = bt_device_lookup_by_name(name);
            // Log all inquiry results for debugging (gamepads highlighted)
            const char* type_str = "";
            if (is_gamepad || is_joystick) type_str = " [GAMEPAD]";
            printf("[BTSTACK_HOST] Inquiry: %02X:%02X:%02X:%02X:%02X:%02X COD=0x%06X%s %s\n",
                   addr[5], addr[4], addr[3], addr[2], addr[1], addr[0],
                   (unsigned)cod, type_str, name);

            // Auto-connect to gamepads
            if ((is_gamepad || is_joystick) && classic_state.inquiry_active) {
                // Skip if we already have an active incoming connection to this device
                // (the device connected to us before we found it in inquiry)
                if (classic_state.pending_valid && !classic_state.pending_outgoing &&
                    memcmp(classic_state.pending_addr, addr, 6) == 0) {
                    printf("[BTSTACK_HOST] Already have incoming connection from this device, skipping outgoing\n");
                    break;
                }

                printf("[BTSTACK_HOST] Classic gamepad found, connecting...\n");
                btstack_host_stop_scan();  // Stop inquiry

                // Save pending info for PIN code handler and deferred connection
                memcpy(classic_state.pending_addr, addr, 6);
                classic_state.pending_cod = cod;
                strncpy(classic_state.pending_name, name, sizeof(classic_state.pending_name) - 1);
                classic_state.pending_name[sizeof(classic_state.pending_name) - 1] = '\0';
                classic_state.pending_profile = profile;
                classic_state.pending_valid = true;
                classic_state.pending_start_time = btstack_run_loop_get_time_ms();
                classic_state.pending_outgoing = true;  // We initiated this connection

                // If name is unavailable, request it and defer connection to
                // REMOTE_NAME_REQUEST_COMPLETE. Some devices need the name to
                // route through the correct connection path (direct L2CAP vs HID
                // Host), and their name is not always
                // included in the Extended Inquiry Response.
                if (!name[0]) {
                    printf("[BTSTACK_HOST] Name unavailable at inquiry, requesting before connect...\n");
                    classic_state.pending_hid_connect = true;
                    gap_remote_name_request(addr, 0, 0);
                    break;
                }

                // On the CYW43 the DualShock 4 uses direct L2CAP to skip SDP:
                // SDP responses from Sony controllers crash the CYW43 SPI bus.
                bool use_direct_l2cap = false;
#ifdef BTSTACK_USE_CYW43
                if (profile->default_vid == 0x054C) {
                    use_direct_l2cap = true;
                    printf("[BTSTACK_HOST] CYW43: using direct L2CAP for Sony (skip SDP)\n");
                }
#endif
                if (use_direct_l2cap) {
                    // Direct L2CAP: skip SDP, create HID channels after encryption
                    printf("[BTSTACK_HOST] %s detected, using direct L2CAP approach\n", profile->name);
                    classic_state.pending_hid_connect = true;

                    // Initialize direct L2CAP connection state
                    direct_l2cap_connection_t* dl = find_free_direct_l2cap();
                    if (dl) {
                        memset(dl, 0, sizeof(*dl));
                        dl->active = true;
                        dl->state = DIRECT_L2CAP_STATE_IDLE;
                        dl->acl_handle = HCI_CON_HANDLE_INVALID;
                        memcpy(dl->addr, addr, 6);
                        strncpy(dl->name, name, sizeof(dl->name) - 1);
                        dl->class_of_device[0] = cod & 0xFF;
                        dl->class_of_device[1] = (cod >> 8) & 0xFF;
                        dl->class_of_device[2] = (cod >> 16) & 0xFF;
                        dl->vendor_id = profile->default_vid;
                        dl->product_id = profile->default_pid;

                        // Allocate classic connection slot for bthid routing
                        classic_connection_t* conn = find_free_classic_connection();
                        if (conn) {
                            int conn_index = conn - classic_state.connections;
                            memset(conn, 0, sizeof(*conn));
                            conn->active = true;
                            conn->hid_cid = 0xFFFF;  // Special marker for direct L2CAP
                            memcpy(conn->addr, addr, 6);
                            strncpy(conn->name, name, sizeof(conn->name) - 1);
                            conn->class_of_device[0] = cod & 0xFF;
                            conn->class_of_device[1] = (cod >> 8) & 0xFF;
                            conn->class_of_device[2] = (cod >> 16) & 0xFF;
                            conn->profile = profile;
                            conn->connect_time = btstack_run_loop_get_time_ms();
                            dl->conn_index = conn_index;
                            printf("[BTSTACK_HOST] %s conn_index=%d\n", profile->name, conn_index);
                        } else {
                            dl->conn_index = -1;
                        }

                        // Create ACL connection directly (the HCI command will trigger the HCI connection)
                        // We'll create L2CAP channels after encryption completes
                        printf("[BTSTACK_HOST] Creating ACL connection to %s...\n", profile->name);
                        uint8_t status = btstack_classic_connect_acl(addr);
                        if (status != ERROR_CODE_SUCCESS && status != ERROR_CODE_COMMAND_DISALLOWED) {
                            printf("[BTSTACK_HOST] classic ACL connect failed: 0x%02X\n", status);
                            dl->active = false;
                            if (conn) memset(conn, 0, sizeof(*conn));
                            classic_state.pending_hid_connect = false;
                        }
                    }
                } else {
                    // Standard path: use normal hid_host_connect
                    // Use profile's hid_mode to determine SDP bypass
                    hid_protocol_mode_t mode = HID_PROTOCOL_MODE_REPORT;
                    uint16_t hid_cid;
                    uint8_t status = hid_host_connect(addr, mode, &hid_cid);
                    if (status == ERROR_CODE_SUCCESS) {
                        printf("[BTSTACK_HOST] hid_host_connect started, cid=0x%04X\n", hid_cid);

                        // Allocate connection slot
                        classic_connection_t* conn = find_free_classic_connection();
                        if (conn) {
                            memset(conn, 0, sizeof(*conn));
                            conn->active = true;
                            conn->hid_cid = hid_cid;
                            memcpy(conn->addr, addr, 6);
                            strncpy(conn->name, name, sizeof(conn->name) - 1);
                            conn->class_of_device[0] = cod & 0xFF;
                            conn->class_of_device[1] = (cod >> 8) & 0xFF;
                            conn->class_of_device[2] = (cod >> 16) & 0xFF;
                            conn->profile = profile;
                            conn->connect_time = btstack_run_loop_get_time_ms();
                        }
                    } else {
                        printf("[BTSTACK_HOST] hid_host_connect failed: %d\n", status);
                    }
                }
            }
            break;
        }

        case GAP_EVENT_INQUIRY_COMPLETE:
            classic_state.inquiry_active = false;
            classic_state.recovery_start_time = 0;  // BT transport is working
            // Restart inquiry ONLY while a timed scan is active (e.g. BOOTSEL pairing window)
            if (scan_timeout_end > 0 && btstack_run_loop_get_time_ms() < scan_timeout_end) {
                classic_state.use_liac = !classic_state.use_liac;
                uint32_t lap = classic_state.use_liac ? GAP_IAC_LIMITED_INQUIRY : GAP_IAC_GENERAL_INQUIRY;
                printf("[BTSTACK_HOST] Restarting inquiry (LAP=%s)...\n",
                       classic_state.use_liac ? "LIAC" : "GIAC");
                gap_inquiry_set_lap(lap);
                gap_inquiry_start(INQUIRY_DURATION);
                classic_state.inquiry_active = true;
            } else if (scan_timeout_end > 0) {
                printf("[BTSTACK_HOST] Inquiry finished, scan window closed\n");
                btstack_host_stop_scan();
            }
            break;

        // Classic BT incoming connection request (DS3 connects this way)
        case HCI_EVENT_CONNECTION_REQUEST: {
            bd_addr_t addr;
            hci_event_connection_request_get_bd_addr(packet, addr);
            uint32_t cod = hci_event_connection_request_get_class_of_device(packet);
            uint8_t link_type = hci_event_connection_request_get_link_type(packet);
            printf("[BTSTACK_HOST] Incoming connection: %02X:%02X:%02X:%02X:%02X:%02X COD=0x%06X link=%d\n",
                   addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], (unsigned)cod, link_type);

            // Save pending connection info for use when HID connection is established
            // Note: device name is not available yet at CONNECTION_REQUEST time.
            // Device detection is deferred to CONNECTION_COMPLETE or later, when
            // name resolution completes. Global master role policy (set at startup)
            // already ensures we become master for all connections.
            memcpy(classic_state.pending_addr, addr, 6);
            classic_state.pending_cod = cod;
            classic_state.pending_name[0] = '\0';  // Clear, will be filled by remote name request
            classic_state.pending_vid = 0;
            classic_state.pending_pid = 0;
            classic_state.pending_valid = true;
            classic_state.pending_start_time = btstack_run_loop_get_time_ms();
            classic_state.pending_outgoing = false;  // Device initiated this connection
            classic_state.waiting_for_incoming_time = 0;  // Device reconnected

            // Silence the radio NOW, before the handshake starts. Bonded
            // reconnects begin the LMP auth/encryption exchange immediately
            // after the ACL — racing the old stop-scan (which waited for the
            // remote name). If inquiry is still running when encryption
            // negotiates, the exchange starves and dies ~30s later with
            // reason 0x22. Scanning resumes via the normal paths if this
            // connection fails or ends.
            if (link_type == 1 /* ACL */) {
                printf("[BTSTACK_HOST] Incoming ACL: pausing scan for handshake\n");
                btstack_host_stop_scan();
            }
            // BTstack will auto-accept with the current master_slave_policy
            break;
        }

        case HCI_EVENT_CONNECTION_COMPLETE: {
            uint8_t status = hci_event_connection_complete_get_status(packet);
            hci_con_handle_t handle = hci_event_connection_complete_get_connection_handle(packet);
            bd_addr_t addr;
            hci_event_connection_complete_get_bd_addr(packet, addr);
            printf("[BTSTACK_HOST] Connection complete: status=%d handle=0x%04X addr=%02X:%02X:%02X:%02X:%02X:%02X\n",
                   status, handle, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

            // Handle connection complete for both incoming and outgoing connections
            if (status == 0) {
                if (classic_state.pending_valid &&
                    bd_addr_cmp(addr, classic_state.pending_addr) == 0) {
                    uint32_t cod = classic_state.pending_cod;

                    if (classic_state.pending_outgoing) {
                        // Outgoing connection (we initiated)
                        printf("[BTSTACK_HOST] Outgoing ACL complete, COD=0x%06X\n", cod);

                        // Direct L2CAP: store ACL handle and do L2CAP-specific setup
                        direct_l2cap_connection_t* dl = find_direct_l2cap_by_addr(addr);
                        if (classic_state.pending_hid_connect && dl && dl->active) {
                            dl->acl_handle = handle;
                            printf("[BTSTACK_HOST] Direct L2CAP: stored ACL handle=0x%04X\n", handle);

                            // Request remote name if we don't have it from inquiry
                            if (dl->name[0] == '\0') {
                                gap_remote_name_request(addr, 0, 0);
                            }

                            // Query VID/PID via SDP
                            sdp_client_query_uuid16(&sdp_query_vid_pid_callback, addr,
                                                    BLUETOOTH_SERVICE_CLASS_PNP_INFORMATION);

                            gap_request_security_level(handle, LEVEL_2);
                        }
                    } else {
                        // Incoming connection (device connected to us)
                        printf("[BTSTACK_HOST] Incoming ACL complete, COD=0x%06X\n", cod);
                        classic_state.pending_acl_handle = handle;

                        // Standard incoming connection flow (DS3, DS4, DS5, or unknown device).

                        // Request remote name for driver matching (we don't have it from inquiry)
                        gap_remote_name_request(addr, 0, 0);

                        // Don't query VID/PID via SDP here — BTstack HID Host runs its
                        // own SDP query after accepting the incoming connection, and the
                        // SDP client only handles one query at a time. Our VID/PID query
                        // would delay HID Host's descriptor query. Instead, query VID/PID
                        // at HID_SUBEVENT_CONNECTION_OPENED after HID channels are established.

                        // Request authentication only if we have a stored key (reconnection).
                        // For new pairings (no key), defer auth to after name resolution
                        // to avoid concurrent SDP+auth on CYW43 and to let device type
                        // detection (Switch vs Sony) determine the connection path.
                        link_key_t incoming_link_key;
                        link_key_type_t incoming_key_type;
                        if (gap_get_link_key_for_bd_addr(addr, incoming_link_key, &incoming_key_type)) {
                            gap_request_security_level(handle, LEVEL_2);
                        }
                    }
                }
            } else {
                printf("[BTSTACK_HOST] Connection failed with status 0x%02X\n", status);
                if (classic_state.pending_valid && bd_addr_cmp(addr, classic_state.pending_addr) == 0) {
                    classic_state.pending_valid = false;
                    classic_state.pending_hid_connect = false;
                }
            }
            break;
        }

        case L2CAP_EVENT_INCOMING_CONNECTION: {
            uint16_t psm = l2cap_event_incoming_connection_get_psm(packet);
            uint16_t cid = l2cap_event_incoming_connection_get_local_cid(packet);
            hci_con_handle_t handle = l2cap_event_incoming_connection_get_handle(packet);
            bd_addr_t addr;
            l2cap_event_incoming_connection_get_address(packet, addr);
            printf("[BTSTACK_HOST] L2CAP incoming: PSM=0x%04X cid=0x%04X handle=0x%04X\n", psm, cid, handle);
            break;
        }

        case L2CAP_EVENT_CHANNEL_OPENED: {
            uint8_t status = l2cap_event_channel_opened_get_status(packet);
            uint16_t psm = l2cap_event_channel_opened_get_psm(packet);
            uint16_t cid = l2cap_event_channel_opened_get_local_cid(packet);
            bd_addr_t l2cap_addr;
            l2cap_event_channel_opened_get_address(packet, l2cap_addr);
            printf("[BTSTACK_HOST] L2CAP opened: status=%d PSM=0x%04X cid=0x%04X addr=%s\n",
                   status, psm, cid, bd_addr_to_str(l2cap_addr));

            // Capture L2CAP CIDs for direct-L2CAP connections (for direct sending)
            // HID Host handles receiving, but we need direct L2CAP CIDs for sending
            // Note: bt_on_hid_ready is called from HID_SUBEVENT_CONNECTION_OPENED
            if (status == 0) {
                direct_l2cap_connection_t* dl = find_direct_l2cap_by_addr(l2cap_addr);
                if (dl && dl->active) {
                    if (psm == PSM_HID_CONTROL) {
                        dl->control_cid = cid;
                        printf("[BTSTACK_HOST] Direct L2CAP: captured control CID=0x%04X for direct sending\n", cid);
                    } else if (psm == PSM_HID_INTERRUPT) {
                        dl->interrupt_cid = cid;
                        printf("[BTSTACK_HOST] Direct L2CAP: captured interrupt CID=0x%04X for direct sending\n", cid);
                    }
                }
            }
            break;
        }

        case HCI_EVENT_REMOTE_NAME_REQUEST_COMPLETE: {
            bd_addr_t name_addr;
            hci_event_remote_name_request_complete_get_bd_addr(packet, name_addr);
            uint8_t name_status = hci_event_remote_name_request_complete_get_status(packet);

            if (name_status != 0) {
                printf("[BTSTACK_HOST] Remote name request failed: status=%d\n", name_status);

                // If we deferred a connection waiting for the name, fall back to
                // standard HID Host connect. This handles DS4, DS3, and other
                // controllers that may not respond to name requests.
                if (classic_state.pending_valid &&
                    classic_state.pending_outgoing &&
                    classic_state.pending_hid_connect &&
                    memcmp(name_addr, classic_state.pending_addr, 6) == 0) {
                    printf("[BTSTACK_HOST] Deferred connect: name failed, falling back\n");

#ifdef BTSTACK_USE_CYW43
                    // CYW43: if pending profile is Sony, use direct L2CAP to skip SDP
                    if (classic_state.pending_profile && classic_state.pending_profile->default_vid == 0x054C) {
                        printf("[BTSTACK_HOST] CYW43: forcing direct L2CAP for Sony (skip SDP)\n");
                        direct_l2cap_connection_t* dl = find_free_direct_l2cap();
                        if (dl) {
                            memset(dl, 0, sizeof(*dl));
                            dl->active = true;
                            dl->state = DIRECT_L2CAP_STATE_IDLE;
                            dl->acl_handle = HCI_CON_HANDLE_INVALID;
                            memcpy(dl->addr, name_addr, 6);
                            dl->class_of_device[0] = classic_state.pending_cod & 0xFF;
                            dl->class_of_device[1] = (classic_state.pending_cod >> 8) & 0xFF;
                            dl->class_of_device[2] = (classic_state.pending_cod >> 16) & 0xFF;
                            dl->vendor_id = classic_state.pending_profile->default_vid;
                            dl->product_id = classic_state.pending_profile->default_pid;

                            classic_connection_t* conn = find_free_classic_connection();
                            if (conn) {
                                int conn_index = conn - classic_state.connections;
                                memset(conn, 0, sizeof(*conn));
                                conn->active = true;
                                conn->hid_cid = 0xFFFF;
                                memcpy(conn->addr, name_addr, 6);
                                conn->class_of_device[0] = classic_state.pending_cod & 0xFF;
                                conn->class_of_device[1] = (classic_state.pending_cod >> 8) & 0xFF;
                                conn->class_of_device[2] = (classic_state.pending_cod >> 16) & 0xFF;
                                conn->profile = classic_state.pending_profile;
                                conn->connect_time = btstack_run_loop_get_time_ms();
                                dl->conn_index = conn_index;
                            } else {
                                dl->conn_index = -1;
                            }

                            uint8_t status = btstack_classic_connect_acl(name_addr);
                            if (status != ERROR_CODE_SUCCESS && status != ERROR_CODE_COMMAND_DISALLOWED) {
                                printf("[BTSTACK_HOST] classic ACL connect failed: 0x%02X\n", status);
                                dl->active = false;
                                if (conn) memset(conn, 0, sizeof(*conn));
                            }
                        }
                        classic_state.pending_hid_connect = false;
                        break;
                    }
#endif
                    classic_state.pending_hid_connect = false;

                    uint16_t hid_cid;
                    uint8_t status = hid_host_connect(name_addr, HID_PROTOCOL_MODE_REPORT, &hid_cid);
                    if (status == ERROR_CODE_SUCCESS) {
                        printf("[BTSTACK_HOST] hid_host_connect started, cid=0x%04X\n", hid_cid);
                        classic_connection_t* conn = find_free_classic_connection();
                        if (conn) {
                            memset(conn, 0, sizeof(*conn));
                            conn->active = true;
                            conn->hid_cid = hid_cid;
                            memcpy(conn->addr, name_addr, 6);
                            conn->class_of_device[0] = classic_state.pending_cod & 0xFF;
                            conn->class_of_device[1] = (classic_state.pending_cod >> 8) & 0xFF;
                            conn->class_of_device[2] = (classic_state.pending_cod >> 16) & 0xFF;
                            conn->connect_time = btstack_run_loop_get_time_ms();
                        }
                    } else {
                        printf("[BTSTACK_HOST] hid_host_connect failed: %d\n", status);
                    }
                }
                break;
            }

            {
                const char* name = hci_event_remote_name_request_complete_get_remote_name(packet);
                printf("[BTSTACK_HOST] Remote name: %s\n", name);

                // Store name if this is our pending incoming connection
                if (classic_state.pending_valid &&
                    memcmp(name_addr, classic_state.pending_addr, 6) == 0) {
                    strncpy(classic_state.pending_name, name, sizeof(classic_state.pending_name) - 1);
                    classic_state.pending_name[sizeof(classic_state.pending_name) - 1] = '\0';
                }

                // Also update any active connection with this address
                for (int i = 0; i < MAX_CLASSIC_CONNECTIONS; i++) {
                    classic_connection_t* conn = &classic_state.connections[i];
                    if (conn->active && memcmp(conn->addr, name_addr, 6) == 0) {
                        if (conn->name[0] == '\0') {
                            strncpy(conn->name, name, sizeof(conn->name) - 1);
                            conn->name[sizeof(conn->name) - 1] = '\0';
                            printf("[BTSTACK_HOST] Updated conn[%d] name: %s\n", i, conn->name);

                            // Update profile from name if not already set
                            if (!conn->profile || conn->profile == &BT_PROFILE_DEFAULT) {
                                conn->profile = bt_device_lookup_by_name(name);
                            }

                            if (conn->hid_ready) {
                                // Set VID/PID from profile defaults
                                const bt_device_profile_t* name_profile = bt_device_lookup_by_name(name);
                                if (name_profile->default_vid) {
                                    conn->vendor_id = name_profile->default_vid;
                                }
                                // Notify BTHID of late name arrival — allows driver
                                // re-evaluation for devices matched as generic because
                                // name wasn't available at connection time
                                bthid_update_device_info(i, conn->name,
                                                         conn->vendor_id, conn->product_id);
                            }
                        }
                        break;
                    }
                }

                // Also update direct_l2cap_conn if active
                direct_l2cap_connection_t* dl_name = find_direct_l2cap_by_addr(name_addr);
                if (dl_name && dl_name->active) {
                    if (dl_name->name[0] == '\0') {
                        strncpy(dl_name->name, name, sizeof(dl_name->name) - 1);
                        dl_name->name[sizeof(dl_name->name) - 1] = '\0';
                        printf("[BTSTACK_HOST] Updated direct-L2CAP name: %s\n", dl_name->name);
                    }
                }

#ifdef BTSTACK_USE_CYW43
                const bt_device_profile_t* late_profile = bt_device_lookup_by_name(name);
                // On CYW43, Sony incoming reconnections use HID Host (not direct L2CAP).
                // The controller initiates its own L2CAP channels; we just need to stop
                // scanning and set default VID so the connection slot gets Sony VID.
                // (Direct L2CAP is only used for outgoing initial pairing to skip SDP.)
                if (late_profile->default_vid == 0x054C &&
                    classic_state.pending_valid &&
                    !classic_state.pending_outgoing &&
                    memcmp(name_addr, classic_state.pending_addr, 6) == 0) {
                    printf("[BTSTACK_HOST] Late Sony detection (incoming) - using HID Host path\n");
                    if (classic_state.pending_vid == 0) {
                        classic_state.pending_vid = late_profile->default_vid;
                    }
                    btstack_host_stop_scan();
                }
#endif
                // Deferred outgoing connection: name was unavailable at inquiry time,
                // so we requested it before connecting. Now that the name has resolved,
                // connect using the appropriate path (direct L2CAP vs HID Host).
                if (classic_state.pending_valid &&
                    classic_state.pending_outgoing &&
                    classic_state.pending_hid_connect &&
                    memcmp(name_addr, classic_state.pending_addr, 6) == 0) {
                    // Update pending name and re-lookup profile
                    strncpy(classic_state.pending_name, name, sizeof(classic_state.pending_name) - 1);
                    classic_state.pending_name[sizeof(classic_state.pending_name) - 1] = '\0';
                    classic_state.pending_hid_connect = false;

                    const bt_device_profile_t* deferred_profile = bt_device_lookup_by_name(name);
                    classic_state.pending_profile = deferred_profile;

                    bool deferred_direct_l2cap = false;
#ifdef BTSTACK_USE_CYW43
                    if (deferred_profile->default_vid == 0x054C) {
                        deferred_direct_l2cap = true;
                        printf("[BTSTACK_HOST] CYW43: forcing direct L2CAP for Sony (skip SDP)\n");
                    }
#endif
                    if (deferred_direct_l2cap) {
                        printf("[BTSTACK_HOST] Deferred connect: %s detected, using direct L2CAP\n",
                               deferred_profile->name);
                        classic_state.pending_hid_connect = true;

                        direct_l2cap_connection_t* dl = find_free_direct_l2cap();
                        if (dl) {
                            memset(dl, 0, sizeof(*dl));
                            dl->active = true;
                            dl->state = DIRECT_L2CAP_STATE_IDLE;
                            dl->acl_handle = HCI_CON_HANDLE_INVALID;
                            memcpy(dl->addr, name_addr, 6);
                            strncpy(dl->name, name, sizeof(dl->name) - 1);
                            dl->class_of_device[0] = classic_state.pending_cod & 0xFF;
                            dl->class_of_device[1] = (classic_state.pending_cod >> 8) & 0xFF;
                            dl->class_of_device[2] = (classic_state.pending_cod >> 16) & 0xFF;
                            dl->vendor_id = deferred_profile->default_vid;
                            dl->product_id = deferred_profile->default_pid;

                            classic_connection_t* conn = find_free_classic_connection();
                            if (conn) {
                                int conn_index = conn - classic_state.connections;
                                memset(conn, 0, sizeof(*conn));
                                conn->active = true;
                                conn->hid_cid = 0xFFFF;
                                memcpy(conn->addr, name_addr, 6);
                                strncpy(conn->name, name, sizeof(conn->name) - 1);
                                conn->class_of_device[0] = classic_state.pending_cod & 0xFF;
                                conn->class_of_device[1] = (classic_state.pending_cod >> 8) & 0xFF;
                                conn->class_of_device[2] = (classic_state.pending_cod >> 16) & 0xFF;
                                conn->profile = deferred_profile;
                                conn->connect_time = btstack_run_loop_get_time_ms();
                                dl->conn_index = conn_index;
                            } else {
                                dl->conn_index = -1;
                            }

                            uint8_t status = btstack_classic_connect_acl(name_addr);
                            if (status != ERROR_CODE_SUCCESS && status != ERROR_CODE_COMMAND_DISALLOWED) {
                                printf("[BTSTACK_HOST] classic ACL connect failed: 0x%02X\n", status);
                                dl->active = false;
                                if (conn) memset(conn, 0, sizeof(*conn));
                                classic_state.pending_hid_connect = false;
                            }
                        }
                    } else {
                        printf("[BTSTACK_HOST] Deferred connect: %s, using HID Host\n",
                               deferred_profile->name);
                        hid_protocol_mode_t mode = HID_PROTOCOL_MODE_REPORT;
                        uint16_t hid_cid;
                        uint8_t status = hid_host_connect(name_addr, mode, &hid_cid);
                        if (status == ERROR_CODE_SUCCESS) {
                            printf("[BTSTACK_HOST] hid_host_connect started, cid=0x%04X\n", hid_cid);
                            classic_connection_t* conn = find_free_classic_connection();
                            if (conn) {
                                memset(conn, 0, sizeof(*conn));
                                conn->active = true;
                                conn->hid_cid = hid_cid;
                                memcpy(conn->addr, name_addr, 6);
                                strncpy(conn->name, name, sizeof(conn->name) - 1);
                                conn->class_of_device[0] = classic_state.pending_cod & 0xFF;
                                conn->class_of_device[1] = (classic_state.pending_cod >> 8) & 0xFF;
                                conn->class_of_device[2] = (classic_state.pending_cod >> 16) & 0xFF;
                                conn->profile = deferred_profile;
                                conn->connect_time = btstack_run_loop_get_time_ms();
                            }
                        } else {
                            printf("[BTSTACK_HOST] hid_host_connect failed: %d\n", status);
                        }
                    }
                }
            }
            break;
        }

        case HCI_EVENT_DISCONNECTION_COMPLETE: {
            hci_con_handle_t handle = hci_event_disconnection_complete_get_connection_handle(packet);
            uint8_t reason = hci_event_disconnection_complete_get_reason(packet);

            printf("[BTSTACK_HOST] Disconnected: handle=0x%04X reason=0x%02X\n", handle, reason);

            // Classic BT disconnect. Classic reconnection/scanning is handled by
            // HID_SUBEVENT_CONNECTION_CLOSED or the outgoing HID failure handler.
            // If we're waiting for an incoming reconnection, don't restart scanning here.

            // Clear pending connection state if this was the pending device.
            // Handles cases where ACL drops before HID opens (e.g., auth failure).
            if (classic_state.pending_valid) {
                classic_state.pending_valid = false;
                classic_state.pending_hid_connect = false;
            }

            // Fully reset direct-L2CAP state for this ACL and tell the driver
            // the pad is gone. The teardown used to be a bare memset here: no
            // bt_on_disconnect(), no release of the Classic slot. The router
            // then kept the frozen last input state and the connection count
            // stayed above zero, so page scan was never re-enabled and the pad
            // could not reconnect at all.
            direct_l2cap_connection_t* dl_disc = find_direct_l2cap_by_handle(handle);
            if (dl_disc) {
                direct_l2cap_teardown_conn(dl_disc, "acl disconnect");
            }

            // Ensure host resumes discoverable/connectable only if no devices remain
            if (btstack_classic_get_connection_count() == 0) {
                gap_discoverable_control(1);
                gap_connectable_control(1);
            }
            break;
        }

        case HCI_EVENT_MODE_CHANGE: {
            uint8_t status = hci_event_mode_change_get_status(packet);
            hci_con_handle_t con_handle = hci_event_mode_change_get_handle(packet);
            uint8_t mode = hci_event_mode_change_get_mode(packet);
            uint16_t interval = hci_event_mode_change_get_interval(packet);
            printf("[BTSTACK_HOST] Mode change: handle 0x%04X mode=%u interval=%u status=%u\n",
                   con_handle, mode, interval, status);
            if (status == ERROR_CODE_SUCCESS && mode != 0) {
                // If any controller enters sniff mode (mode 2) or hold/park, force back to active mode (mode 0)
                printf("[BTSTACK_HOST] Exiting power-save mode %u for handle 0x%04X -> active mode\n", mode, con_handle);
                gap_sniff_mode_exit(con_handle);
            }
            break;
        }

        case HCI_EVENT_LINK_KEY_REQUEST: {
            bd_addr_t req_addr;
            reverse_bytes(&packet[2], req_addr, 6);

            // Check if we have a stored link key
            link_key_t link_key;
            link_key_type_t key_type;
            bool have_key = gap_get_link_key_for_bd_addr(req_addr, link_key, &key_type);

            hci_connection_t *conn = hci_connection_for_bd_addr_and_type(req_addr, BD_ADDR_TYPE_ACL);
            printf("[BTSTACK_HOST] Link key request: %02X:%02X:%02X:%02X:%02X:%02X conn=%s have_key=%d type=%d\n",
                   req_addr[0], req_addr[1], req_addr[2], req_addr[3], req_addr[4], req_addr[5],
                   conn ? "YES" : "NO", have_key, have_key ? key_type : -1);

            // BTstack's hci.c handles this automatically - it will look up the key and respond
            // If no key is found, it sends negative reply which triggers PIN request for legacy pairing
            break;
        }

        // Legacy PIN code request: the DS4 pairs with Secure Simple Pairing,
        // so a legacy PIN request is never expected and is rejected below.
        case HCI_EVENT_PIN_CODE_REQUEST: {
            bd_addr_t pin_addr;
            hci_event_pin_code_request_get_bd_addr(packet, pin_addr);
            printf("[BTSTACK_HOST] PIN code request: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   pin_addr[0], pin_addr[1], pin_addr[2], pin_addr[3], pin_addr[4], pin_addr[5]);

            // The DualShock 4 pairs with Secure Simple Pairing, so a legacy PIN
            // request is never expected: reject it.
            printf("[BTSTACK_HOST] PIN request rejected (no legacy PIN profile)\n");
            gap_pin_code_negative(pin_addr);
            break;
        }

        case HCI_EVENT_LINK_KEY_NOTIFICATION: {
            bd_addr_t notif_addr;
            reverse_bytes(&packet[2], notif_addr, 6);
            link_key_t link_key;
            memcpy(link_key, &packet[8], 16);
            link_key_type_t key_type = (link_key_type_t)packet[24];

            printf("[BTSTACK_HOST] Link key notification: %02X:%02X:%02X:%02X:%02X:%02X type=%d\n",
                   notif_addr[0], notif_addr[1], notif_addr[2], notif_addr[3], notif_addr[4], notif_addr[5], key_type);

            // Explicitly store the link key (BTstack's auto-storage may not work for legacy pairing)
            gap_store_link_key_for_bd_addr(notif_addr, link_key, key_type);
            break;
        }

        case HCI_EVENT_AUTHENTICATION_COMPLETE: {
            uint8_t status = packet[2];
            hci_con_handle_t handle = little_endian_read_16(packet, 3);
            printf("[BTSTACK_HOST] Authentication complete: handle=0x%04X status=0x%02X\n", handle, status);

            // Handle PIN_OR_KEY_MISSING (0x06): controller cleared its link key
            // (e.g., put in pairing mode) but we still have a stale stored key.
            // Delete the stale key and disconnect so next attempt triggers fresh pairing.
            if (status == 0x06 && classic_state.pending_valid) {
                printf("[BTSTACK_HOST] Auth failed (key rejected), deleting stale link key\n");
                gap_drop_link_key_for_bd_addr(classic_state.pending_addr);

                // Clean up direct-L2CAP state if auth failed before channels were created
                direct_l2cap_connection_t* dl_auth = find_direct_l2cap_by_handle(handle);
                if (dl_auth) {
                    memset(dl_auth, 0, sizeof(*dl_auth));
                    dl_auth->acl_handle = HCI_CON_HANDLE_INVALID;
                    dl_auth->conn_index = -1;
                }
                classic_state.pending_hid_connect = false;

                gap_disconnect(handle);
            }
            break;
        }

        case HCI_EVENT_ENCRYPTION_CHANGE: {
            hci_con_handle_t handle = hci_event_encryption_change_get_connection_handle(packet);
            uint8_t status = hci_event_encryption_change_get_status(packet);
            uint8_t enabled = hci_event_encryption_change_get_encryption_enabled(packet);

            printf("[BTSTACK_HOST] Encryption change: handle=0x%04X status=0x%02X enabled=%d\n",
                   handle, status, enabled);

            // Direct L2CAP: create the HID control channel once encryption is enabled
            // (state=IDLE for a fresh link; W4_CONTROL_CONNECTED while retrying)
            direct_l2cap_connection_t* dl_enc = find_direct_l2cap_by_handle(handle);
            if (status == 0 && enabled && dl_enc && dl_enc->active &&
                (dl_enc->state == DIRECT_L2CAP_STATE_IDLE ||
                 dl_enc->state == DIRECT_L2CAP_STATE_W4_CONTROL_CONNECTED) &&
                dl_enc->control_cid == 0) {

                // For incoming reconnections, don't create outgoing L2CAP channels.
                // The controller will initiate its own channels via HID Host.
                // Creating outgoing channels conflicts with the incoming ones.
                if (classic_state.pending_valid && !classic_state.pending_outgoing) {
                    printf("[BTSTACK_HOST] Direct L2CAP: incoming reconnection, waiting for HID Host channels\n");
                    break;
                }

                printf("[BTSTACK_HOST] Direct L2CAP: encryption enabled, creating HID Control channel (PSM 0x11)...\n");

                uint16_t control_cid;
                uint8_t l2cap_status = l2cap_create_channel(direct_l2cap_packet_handler,
                                                            dl_enc->addr,
                                                            PSM_HID_CONTROL,
                                                            0xFFFF,  // MTU
                                                            &control_cid);
                if (l2cap_status == ERROR_CODE_SUCCESS) {
                    dl_enc->control_cid = control_cid;
                    dl_enc->state = DIRECT_L2CAP_STATE_W4_CONTROL_CONNECTED;
                    printf("[BTSTACK_HOST] Direct L2CAP: control channel request sent, cid=0x%04X\n", control_cid);
                } else {
                    printf("[BTSTACK_HOST] Direct L2CAP: l2cap_create_channel failed: 0x%02X\n", l2cap_status);
                    dl_enc->active = false;
                    classic_state.pending_hid_connect = false;
                }
            }
            break;
        }

        case GAP_EVENT_SECURITY_LEVEL: {
            hci_con_handle_t handle = gap_event_security_level_get_handle(packet);
            gap_security_level_t level = gap_event_security_level_get_security_level(packet);
            printf("[BTSTACK_HOST] Security level update: handle=0x%04X level=%d\n", handle, level);
            break;
        }

        case HCI_EVENT_ROLE_CHANGE: {
            uint8_t status = hci_event_role_change_get_status(packet);
            bd_addr_t addr;
            hci_event_role_change_get_bd_addr(packet, addr);
            uint8_t role = hci_event_role_change_get_role(packet);
            printf("[BTSTACK_HOST] Role change: %02X:%02X:%02X:%02X:%02X:%02X status=%d role=%s\n",
                   addr[0], addr[1], addr[2], addr[3], addr[4], addr[5],
                   status, role == 0 ? "MASTER" : "SLAVE");
            break;
        }
    }
}

// ============================================================================
// STATUS
// ============================================================================

bool btstack_host_is_initialized(void)
{
    return hid_state.initialized;
}

bool btstack_host_is_powered_on(void)
{
    return hid_state.powered_on;
}

bool btstack_host_is_scanning(void)
{
    return classic_state.inquiry_active;
}

bool btstack_host_has_ready_connection(void)
{
    for (int i = 0; i < MAX_CLASSIC_CONNECTIONS; i++) {
        if (classic_state.connections[i].active && classic_state.connections[i].hid_ready) {
            return true;
        }
    }
    return false;
}

bool btstack_host_is_connecting(void)
{
    if (classic_state.pending_valid) return true;
    for (int i = 0; i < MAX_CLASSIC_CONNECTIONS; i++) {
        if (classic_state.connections[i].active && !classic_state.connections[i].hid_ready) {
            return true;
        }
    }
    return false;
}

// ============================================================================
// CLASSIC BT HID HOST PACKET HANDLER
// ============================================================================

static bool btstack_report_debug_done = false;

static void hid_host_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t event_type = hci_event_packet_get_type(packet);
    if (event_type != HCI_EVENT_HID_META) return;

    uint8_t subevent = hci_event_hid_meta_get_subevent_code(packet);

    switch (subevent) {
        case HID_SUBEVENT_INCOMING_CONNECTION: {
            // Accept incoming HID connections from devices
            uint16_t hid_cid = hid_subevent_incoming_connection_get_hid_cid(packet);

            // In RevolutionDS4, all incoming connections are DualShock 4 controllers.
            // Accepting with HID_PROTOCOL_MODE_BOOT instructs BTstack hid_host to skip
            // the SDP descriptor query, preventing CYW43 baseband stalls and eliminating
            // the white lightbar hang on cold boot / fresh flash.
            hid_protocol_mode_t accept_mode = HID_PROTOCOL_MODE_BOOT;
            printf("[BTSTACK_HOST] HID incoming connection, cid=0x%04X - accepting (mode=BOOT/skip SDP)\n", hid_cid);
            hid_host_accept_connection(hid_cid, accept_mode);

            // Allocate connection slot if needed
            if (!find_classic_connection_by_cid(hid_cid)) {
                classic_connection_t* conn = find_free_classic_connection();
                if (conn) {
                    memset(conn, 0, sizeof(*conn));
                    conn->active = true;
                    conn->hid_cid = hid_cid;
                    hid_subevent_incoming_connection_get_address(packet, conn->addr);

                    // Use pending COD and name if address matches (from HCI_EVENT_CONNECTION_REQUEST)
                    if (classic_state.pending_valid &&
                        memcmp(conn->addr, classic_state.pending_addr, 6) == 0) {
                        conn->class_of_device[0] = classic_state.pending_cod & 0xFF;
                        conn->class_of_device[1] = (classic_state.pending_cod >> 8) & 0xFF;
                        conn->class_of_device[2] = (classic_state.pending_cod >> 16) & 0xFF;
                        // Copy name if we got it from remote name request
                        if (classic_state.pending_name[0]) {
                            strncpy(conn->name, classic_state.pending_name, sizeof(conn->name) - 1);
                            conn->name[sizeof(conn->name) - 1] = '\0';
                            printf("[BTSTACK_HOST] Using pending name: %s\n", conn->name);
                        }
                        // Copy VID/PID if we got them from SDP query
                        if (classic_state.pending_vid || classic_state.pending_pid) {
                            conn->vendor_id = classic_state.pending_vid;
                            conn->product_id = classic_state.pending_pid;
                            printf("[BTSTACK_HOST] Using pending VID/PID: 0x%04X/0x%04X\n",
                                   conn->vendor_id, conn->product_id);
                        }
                        // DON'T clear pending_valid here - PIN code request may come after this
                        // It will be cleared in HID_SUBEVENT_CONNECTION_OPENED
                        printf("[BTSTACK_HOST] Using pending COD: 0x%06X\n", (unsigned)classic_state.pending_cod);
                    }
                    conn->connect_time = btstack_run_loop_get_time_ms();
                }
            }
            break;
        }

        case HID_SUBEVENT_CONNECTION_OPENED: {
            uint16_t hid_cid = hid_subevent_connection_opened_get_hid_cid(packet);
            uint8_t status = hid_subevent_connection_opened_get_status(packet);

            // Reset security level if we elevated it for direct L2CAP
            if (classic_state.pending_hid_connect) {
                printf("[BTSTACK_HOST] Resetting security level to 0\n");
                gap_set_security_level(LEVEL_0);
                classic_state.pending_hid_connect = false;
            }

            // Clear pending connection info now that HID is established
            classic_state.pending_valid = false;

            if (status != ERROR_CODE_SUCCESS) {
                printf("[BTSTACK_HOST] HID connection failed, cid=0x%04X status=0x%02X\n", hid_cid, status);
                // Remove connection slot
                classic_connection_t* conn = find_classic_connection_by_cid(hid_cid);
                if (conn) {
                    memset(conn, 0, sizeof(*conn));
                }

                // If this was an outgoing connection, disconnect ACL and wait for
                // the device to reconnect via the incoming path. Some controllers
                // (certain DS4 HW revisions) don't accept HID L2CAP channels from
                // the host but work when they initiate the connection themselves.
                // A link key was exchanged during the failed attempt, so when the
                // device reconnects (incoming), authentication will use the stored key.
                // Don't resume scanning — otherwise we'll rediscover the device
                // still in pairing mode and loop endlessly.
                //
                // Incoming failures drop the ACL too: after BTstack finalizes a
                // half-open HID connection the controller can keep the ACL alive
                // (default white lightbar, nothing happening) and never retry by
                // itself. Dropping it lets the controller re-page immediately.
                hci_con_handle_t con_handle = hid_subevent_connection_opened_get_con_handle(packet);
                if (!hid_subevent_connection_opened_get_incoming(packet)) {
                    printf("[BTSTACK_HOST] Outgoing HID failed, disconnecting to allow incoming reconnect\n");
                    classic_state.pending_outgoing = false;
                    classic_state.pending_valid = false;
                    // Don't scan — stay connectable, wait for incoming reconnection
                    classic_state.waiting_for_incoming_time = btstack_run_loop_get_time_ms();
                } else {
                    printf("[BTSTACK_HOST] Incoming HID failed, dropping link for a clean retry\n");
                }
                gap_disconnect(con_handle);
                return;
            }

            printf("[BTSTACK_HOST] HID connection opened, cid=0x%04X\n", hid_cid);

            hci_con_handle_t con_handle = hid_subevent_connection_opened_get_con_handle(packet);
            hci_send_cmd(&hci_write_link_policy_settings, con_handle, LM_LINK_POLICY_ENABLE_ROLE_SWITCH);

            // Mark connection as ready (HID channels established)
            classic_connection_t* conn = find_classic_connection_by_cid(hid_cid);
            if (conn) {
                conn->hid_ready = true;

                if (conn->name[0] && !conn->profile) {
                    conn->profile = bt_device_lookup_by_name(conn->name);
                }
                if (!conn->profile) {
                    conn->profile = &BT_PROFILE_SONY;
                }

                // In RevolutionDS4, ensure Sony VID/PID and name are always set
                if (conn->vendor_id == 0) {
                    conn->vendor_id = conn->profile->default_vid ? conn->profile->default_vid : 0x054C;
                    conn->product_id = conn->profile->default_pid ? conn->profile->default_pid : 0x05C4;
                    printf("[BTSTACK_HOST] Set VID=0x%04X from %s profile\n",
                           conn->vendor_id, conn->profile->name);
                }
                if (conn->name[0] == '\0') {
                    strncpy(conn->name, "Wireless Controller", sizeof(conn->name) - 1);
                }

                // Wait for HID_SUBEVENT_DESCRIPTOR_AVAILABLE
                // NOTE: Do NOT issue SDP queries here — BTstack HID Host starts its
                // own SDP query (for HID descriptor) immediately after CONNECTION_OPENED.
                // sdp_client only handles one query at a time, so issuing ours here
                // would block BTstack's, preventing DESCRIPTOR_AVAILABLE from firing.
                // VID/PID SDP query is deferred to DESCRIPTOR_AVAILABLE instead.
            }
            break;
        }

        case HID_SUBEVENT_DESCRIPTOR_AVAILABLE: {
            uint16_t hid_cid = hid_subevent_descriptor_available_get_hid_cid(packet);
            uint8_t status = hid_subevent_descriptor_available_get_status(packet);

            printf("[BTSTACK_HOST] HID descriptor available, cid=0x%04X status=0x%02X\n", hid_cid, status);

            // Notify bthid layer that device is ready
            // This fires after SDP + SET_PROTOCOL complete, so BTstack's state
            // is CONNECTION_ESTABLISHED and hid_host_send_report() will succeed.
            int conn_index = get_classic_conn_index(hid_cid);
            if (conn_index >= 0) {
                // Pass HID descriptor to bthid for generic gamepad parsing
                const uint8_t* hid_desc = hid_descriptor_storage_get_descriptor_data(hid_cid);
                uint16_t hid_desc_len = hid_descriptor_storage_get_descriptor_len(hid_cid);
                if (hid_desc && hid_desc_len > 0) {
                    printf("[BTSTACK_HOST] Classic HID descriptor: %d bytes\n", hid_desc_len);
                    bthid_set_hid_descriptor(conn_index, hid_desc, hid_desc_len);
                }

                btstack_host_stop_scan();
                scan_timeout_end = 0;
                // Controller connected and ready: disable inquiry scan and page scan so the radio
                // devotes 100% of airtime and baseband bandwidth to the active gamepad link.
                // This completely eliminates periodic link stalls (~640ms) on touchpad and audio!
                gap_discoverable_control(0);
                gap_connectable_control(0);
                printf("[BTSTACK_HOST] Controller ready: page/inquiry scan disabled for link performance\n");
                printf("[BTSTACK_HOST] Calling bt_on_hid_ready(%d)\n", conn_index);
                bt_on_hid_ready(conn_index);

                // Query VID/PID via SDP if not yet known (deferred from CONNECTION_OPENED
                // to avoid conflicting with BTstack's internal HID descriptor SDP query).
                // Only issue it when the SDP client is actually free: a second
                // controller connecting right now owns it for its descriptor query,
                // and stealing it would leave that link unidentified (white DS4).
                // The setup watchdog retries once the SDP client is free again.
                classic_connection_t* desc_conn = find_classic_connection_by_cid(hid_cid);
                if (desc_conn && desc_conn->vendor_id == 0 && desc_conn->product_id == 0) {
                    if (sdp_client_ready()) {
                        desc_conn->vidpid_attempts = 1;
                        desc_conn->last_vidpid_ms = btstack_run_loop_get_time_ms();
                        memcpy(classic_state.pending_addr, desc_conn->addr, 6);
                        classic_state.pending_vid = 0;
                        classic_state.pending_pid = 0;
                        printf("[BTSTACK_HOST] Querying VID/PID via SDP (deferred)\n");
                        sdp_client_query_uuid16(&sdp_query_vid_pid_callback, desc_conn->addr,
                                                BLUETOOTH_SERVICE_CLASS_PNP_INFORMATION);
                    } else {
                        printf("[BTSTACK_HOST] VID/PID query deferred again (SDP busy)\n");
                    }
                }
            }
            break;
        }

        case HID_SUBEVENT_REPORT: {
            uint16_t hid_cid = hid_subevent_report_get_hid_cid(packet);
            const uint8_t* report = hid_subevent_report_get_report(packet);
            uint16_t report_len = hid_subevent_report_get_report_len(packet);

            // Debug: show raw BTstack report
            if (!btstack_report_debug_done && report_len >= 4) {
                printf("[BTSTACK_HOST] Raw report len=%d: %02X %02X %02X %02X\n",
                       report_len, report[0], report[1], report[2], report[3]);
                btstack_report_debug_done = true;
            }

            // Route to bthid layer
            // BTstack report already includes 0xA1 header (DATA|INPUT)
            int conn_index = get_classic_conn_index(hid_cid);
            if (conn_index >= 0 && report_len > 0) {
                bt_on_hid_report(conn_index, report, report_len);
            }
            break;
        }

        case HID_SUBEVENT_GET_REPORT_RESPONSE: {
            // Response to a driver-issued GET_REPORT (e.g. DS4 factory
            // calibration). The report data includes the report ID as its
            // first byte. Without a response the HID Host stays in its
            // waiting state, so drivers must time out gracefully.
            uint16_t hid_cid = hid_subevent_get_report_response_get_hid_cid(packet);
            uint8_t status = hid_subevent_get_report_response_get_handshake_status(packet);
            const uint8_t* report = hid_subevent_get_report_response_get_report(packet);
            uint16_t report_len = hid_subevent_get_report_response_get_report_len(packet);

            int conn_index = get_classic_conn_index(hid_cid);
            printf("[BTSTACK_HOST] GET_REPORT response: cid=0x%04X status=%d len=%d id=0x%02X\n",
                   hid_cid, status, report_len, report_len > 0 ? report[0] : 0);

            if (conn_index >= 0 &&
                status == HID_HANDSHAKE_PARAM_TYPE_SUCCESSFUL &&
                report_len > 0) {
                bt_on_get_report(conn_index, report, report_len);
            }
            break;
        }

        case HID_SUBEVENT_CONNECTION_CLOSED: {
            uint16_t hid_cid = hid_subevent_connection_closed_get_hid_cid(packet);
            printf("[BTSTACK_HOST] HID connection closed, cid=0x%04X\n", hid_cid);

            // Reset debug flag so reconnections produce debug output
            btstack_report_debug_done = false;

            // Notify bthid layer
            int conn_index = get_classic_conn_index(hid_cid);
            if (conn_index >= 0) {
                bt_on_disconnect(conn_index);
            }

            // Free connection slot
            classic_connection_t* conn = find_classic_connection_by_cid(hid_cid);
            if (conn) {
                memset(conn, 0, sizeof(*conn));
            }

            // Keep host connectable for incoming connection via Page Scan (e.g. user pressing PS button on DS4).
            // Do NOT start active inquiry — inquiry starves audio and radio bandwidth.
            if (btstack_classic_get_connection_count() == 0) {
                printf("[BTSTACK_HOST] No devices connected, waiting for incoming connection (Page Scan)\n");
                gap_discoverable_control(1);
                gap_connectable_control(1);
            }
            break;
        }

        case HID_SUBEVENT_SET_PROTOCOL_RESPONSE: {
            uint16_t hid_cid = hid_subevent_set_protocol_response_get_hid_cid(packet);
            uint8_t handshake = hid_subevent_set_protocol_response_get_handshake_status(packet);
            hid_protocol_mode_t mode = hid_subevent_set_protocol_response_get_protocol_mode(packet);
            printf("[BTSTACK_HOST] HID set protocol response: cid=0x%04X handshake=%d mode=%d\n",
                   hid_cid, handshake, mode);
            break;
        }

        default:
            printf("[BTSTACK_HOST] HID subevent: 0x%02X\n", subevent);
            break;
    }
}

// ============================================================================
// DIRECT L2CAP PACKET HANDLER (Sony DS4)
// ============================================================================

static void direct_l2cap_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    switch (packet_type) {
        case HCI_EVENT_PACKET: {
            uint8_t event_type = hci_event_packet_get_type(packet);

            if (event_type == L2CAP_EVENT_CHANNEL_OPENED) {
                uint8_t status = l2cap_event_channel_opened_get_status(packet);
                uint16_t local_cid = l2cap_event_channel_opened_get_local_cid(packet);
                uint16_t psm = l2cap_event_channel_opened_get_psm(packet);

                printf("[BTSTACK_HOST] Direct L2CAP opened: status=%d PSM=0x%04X cid=0x%04X\n",
                       status, psm, local_cid);

                direct_l2cap_connection_t* dl = find_direct_l2cap_by_cid(local_cid);
                if (!dl) {
                    bd_addr_t event_addr;
                    l2cap_event_channel_opened_get_address(packet, event_addr);
                    dl = find_direct_l2cap_by_addr(event_addr);
                }
                if (!dl) {
                    hci_con_handle_t handle = l2cap_event_channel_opened_get_handle(packet);
                    dl = find_direct_l2cap_by_handle(handle);
                }

                if (status != 0) {
                    // A channel that never opened still owns the state and the
                    // Classic slot (the setup watchdog deliberately skips
                    // hid_cid == 0xFFFF), so clean up here instead of waiting
                    // for a timeout that never comes.
                    printf("[BTSTACK_HOST] Direct L2CAP: channel failed: 0x%02X\n", status);
                    if (dl) {
                        direct_l2cap_teardown_conn(dl, "channel open failed");
                    }
                    return;
                }

                if (!dl) {
                    printf("[BTSTACK_HOST] Direct L2CAP: channel opened for unknown connection!\n");
                    return;
                }

                if (psm == PSM_HID_CONTROL && dl->state == DIRECT_L2CAP_STATE_W4_CONTROL_CONNECTED) {
                    // Control channel opened, now create interrupt channel
                    dl->control_cid = local_cid;
                    printf("[BTSTACK_HOST] Direct L2CAP: control channel connected, creating interrupt channel (PSM 0x13)...\n");

                    uint16_t interrupt_cid;
                    uint8_t l2cap_status = l2cap_create_channel(direct_l2cap_packet_handler,
                                                                dl->addr,
                                                                PSM_HID_INTERRUPT,
                                                                0xFFFF,
                                                                &interrupt_cid);
                    if (l2cap_status == ERROR_CODE_SUCCESS) {
                        dl->interrupt_cid = interrupt_cid;
                        dl->state = DIRECT_L2CAP_STATE_W4_INTERRUPT_CONNECTED;
                        printf("[BTSTACK_HOST] Direct L2CAP: interrupt channel request sent, cid=0x%04X\n", interrupt_cid);
                    } else {
                        printf("[BTSTACK_HOST] Direct L2CAP: l2cap_create_channel (interrupt) failed: 0x%02X\n", l2cap_status);
                        dl->active = false;
                        classic_state.pending_hid_connect = false;
                    }

                } else if (psm == PSM_HID_INTERRUPT && dl->state == DIRECT_L2CAP_STATE_W4_INTERRUPT_CONNECTED) {
                    // Interrupt channel opened - connection complete!
                    dl->interrupt_cid = local_cid;
                    printf("[BTSTACK_HOST] Direct L2CAP: interrupt channel connected - HID READY!\n");
                    dl->state = DIRECT_L2CAP_STATE_CONNECTED;
                    dl->last_rx_ms = btstack_run_loop_get_time_ms();
                    classic_state.pending_hid_connect = false;

                    // Stop scanning if all slots are filled
                    if (btstack_classic_get_connection_count() >= MAX_CLASSIC_CONNECTIONS) {
                        btstack_host_stop_scan();
                        scan_timeout_end = 0;
                    }

                    // Allocate classic connection slot if not already allocated
                    if (dl->conn_index < 0) {
                        classic_connection_t* conn = find_free_classic_connection();
                        if (conn) {
                            memset(conn, 0, sizeof(*conn));
                            conn->active = true;
                            conn->hid_cid = 0xFFFF;  // Marker: direct L2CAP (no HID Host CID)
                            memcpy(conn->addr, dl->addr, 6);
                            strncpy(conn->name, dl->name, sizeof(conn->name) - 1);
                            conn->vendor_id = dl->vendor_id;
                            conn->product_id = dl->product_id;
                            conn->hid_ready = true;

                            // Get index
                            for (int i = 0; i < MAX_CLASSIC_CONNECTIONS; i++) {
                                if (&classic_state.connections[i] == conn) {
                                    dl->conn_index = i;
                                    dl->vendor_id = conn->vendor_id;
                                    dl->product_id = conn->product_id;
                                    printf("[BTSTACK_HOST] Direct L2CAP: allocated conn_index=%d\n", i);
                                    break;
                                }
                            }
                        }
                    }

                    // Update the classic connection slot
                    if (dl->conn_index >= 0 && dl->conn_index < MAX_CLASSIC_CONNECTIONS) {
                        classic_connection_t* conn = &classic_state.connections[dl->conn_index];
                        conn->hid_ready = true;

                        // Update bthid with device info
                        uint16_t vid = dl->vendor_id;
                        uint16_t pid = dl->product_id;
                        printf("[BTSTACK_HOST] Direct L2CAP: updating bthid with name='%s' VID=0x%04X PID=0x%04X\n",
                               dl->name, vid, pid);
                        bthid_update_device_info(dl->conn_index, dl->name, vid, pid);

                        // Notify bthid layer
                        printf("[BTSTACK_HOST] Direct L2CAP: calling bt_on_hid_ready(%d)\n", dl->conn_index);
                        bt_on_hid_ready(dl->conn_index);
                    }
                }

            } else if (event_type == L2CAP_EVENT_CHANNEL_CLOSED) {
                uint16_t local_cid = l2cap_event_channel_closed_get_local_cid(packet);
                printf("[BTSTACK_HOST] Direct L2CAP closed: cid=0x%04X\n", local_cid);

                direct_l2cap_connection_t* dl = find_direct_l2cap_by_cid(local_cid);
                if (dl) {
                    direct_l2cap_teardown_conn(dl, "channel closed");
                }
            }
            break;
        }

        case L2CAP_DATA_PACKET: {
            // HID data from the direct-L2CAP interrupt channel
            // Data already includes HID header (0xA1 for DATA|INPUT)
            direct_l2cap_connection_t* dl = find_direct_l2cap_by_cid(channel);
            if (dl && dl->active) {
                // Any uplink packet proves the link is alive
                dl->last_rx_ms = btstack_run_loop_get_time_ms();
                if (dl->state == DIRECT_L2CAP_STATE_CONNECTED) {
                    // Route to bthid layer
                    if (dl->conn_index >= 0 && size > 0) {
                        bt_on_hid_report(dl->conn_index, packet, size);
                    }
                } else {
                    printf("[BTSTACK_HOST] Direct L2CAP data dropped: active=%d state=%d\n",
                           dl->active, dl->state);
                }
            }
            break;
        }

        default:
            break;
    }
}

// ============================================================================
// CLASSIC BT OUTPUT REPORTS
// ============================================================================

// Send SET_REPORT on control channel with specified report type
// report_type: 1=Input, 2=Output, 3=Feature
bool btstack_classic_send_set_report_type(uint8_t conn_index, uint8_t report_type,
                                           uint8_t report_id, const uint8_t* data, uint16_t len)
{
    if (conn_index >= MAX_CLASSIC_CONNECTIONS) return false;

    classic_connection_t* conn = &classic_state.connections[conn_index];
    if (!conn->active || !conn->hid_ready) return false;

    // Check if this is a direct-L2CAP connection (marked with hid_cid = 0xFFFF)
    if (conn->hid_cid == 0xFFFF) {
        direct_l2cap_connection_t* dl = find_direct_l2cap_by_conn_index(conn_index);
        if (dl && dl->active && dl->state == DIRECT_L2CAP_STATE_CONNECTED) {
            // Send SET_REPORT on control channel via raw L2CAP
            // HID transaction format: [SET_REPORT | report_type] [report_id] [data...]
            static uint8_t direct_l2cap_setreport_buf[80];
            uint16_t total = len + 2;
            if (total > sizeof(direct_l2cap_setreport_buf)) return false;
            direct_l2cap_setreport_buf[0] = 0x50 | (report_type & 0x03);  // SET_REPORT | type
            direct_l2cap_setreport_buf[1] = report_id;
            if (len > 0) memcpy(direct_l2cap_setreport_buf + 2, data, len);
            uint8_t status = l2cap_send(dl->control_cid, direct_l2cap_setreport_buf, total);
            if (status != ERROR_CODE_SUCCESS) {
                printf("[BTSTACK_HOST] Direct L2CAP send_set_report failed: type=%d id=0x%02X status=%d\n",
                       report_type, report_id, status);
            }
            return status == ERROR_CODE_SUCCESS;
        }
        return false;
    }

    // Map report type to BTstack enum
    hid_report_type_t hid_type;
    switch (report_type) {
        case 1: hid_type = HID_REPORT_TYPE_INPUT; break;
        case 2: hid_type = HID_REPORT_TYPE_OUTPUT; break;
        case 3: hid_type = HID_REPORT_TYPE_FEATURE; break;
        default: hid_type = HID_REPORT_TYPE_OUTPUT; break;
    }

    // hid_host_send_set_report stores a pointer to the data and sends asynchronously.
    // Copy into static buffer so the data persists until the actual L2CAP send completes.
    static uint8_t hid_host_set_report_buf[80];
    if (len > sizeof(hid_host_set_report_buf)) return false;
    if (len > 0) memcpy(hid_host_set_report_buf, data, len);

    uint8_t status = hid_host_send_set_report(conn->hid_cid, hid_type, report_id, hid_host_set_report_buf, len);
    if (status != ERROR_CODE_SUCCESS && status != ERROR_CODE_COMMAND_DISALLOWED) {
        printf("[BTSTACK_HOST] send_set_report failed: type=%d id=0x%02X status=%d\n",
               report_type, report_id, status);
    }
    return status == ERROR_CODE_SUCCESS;
}

// Send SET_REPORT on control channel (default to OUTPUT type)
bool btstack_classic_send_set_report(uint8_t conn_index, uint8_t report_id,
                                      const uint8_t* data, uint16_t len)
{
    return btstack_classic_send_set_report_type(conn_index, 2, report_id, data, len);
}

// Request a report with GET_REPORT on the control channel. The response is
// delivered asynchronously to the device driver via bt_on_get_report().
// Returns false when the request could not be queued (link busy/not ready).
// report_type: 1=Input, 2=Output, 3=Feature
bool btstack_classic_send_get_report(uint8_t conn_index, uint8_t report_type,
                                      uint8_t report_id)
{
    if (conn_index >= MAX_CLASSIC_CONNECTIONS) return false;

    classic_connection_t* conn = &classic_state.connections[conn_index];
    if (!conn->active || !conn->hid_ready) return false;

    // Direct-L2CAP devices have no HID host: send the GET_REPORT transaction
    // straight down the control channel. This used to return false, which left
    // the DS4 factory calibration unrequested forever and the connect blink
    // code (green/yellow/red) never ran.
    if (conn->hid_cid == 0xFFFF) {
        direct_l2cap_connection_t* dl = find_direct_l2cap_by_conn_index(conn_index);
        if (!dl || !dl->active || dl->state != DIRECT_L2CAP_STATE_CONNECTED) {
            return false;
        }
        // HID transaction header: [GET_REPORT | report_type] [report_id]
        uint8_t req[2];
        req[0] = (uint8_t)(0x40 | (report_type & 0x03));
        req[1] = report_id;
        return l2cap_send(dl->control_cid, req, sizeof(req)) == ERROR_CODE_SUCCESS;
    }

    // Map report type to BTstack enum
    hid_report_type_t hid_type;
    switch (report_type) {
        case 1: hid_type = HID_REPORT_TYPE_INPUT; break;
        case 2: hid_type = HID_REPORT_TYPE_OUTPUT; break;
        case 3: hid_type = HID_REPORT_TYPE_FEATURE; break;
        default: hid_type = HID_REPORT_TYPE_FEATURE; break;
    }

    return hid_host_send_get_report(conn->hid_cid, hid_type, report_id) == ERROR_CODE_SUCCESS;
}

// Send DATA on interrupt channel (for regular output reports)
bool btstack_classic_send_report(uint8_t conn_index, uint8_t report_id,
                                  const uint8_t* data, uint16_t len)
{
    if (conn_index >= MAX_CLASSIC_CONNECTIONS) return false;

    classic_connection_t* conn = &classic_state.connections[conn_index];
    if (!conn->active || !conn->hid_ready) return false;

    // Check if this is a direct-L2CAP connection (marked with hid_cid = 0xFFFF)
    if (conn->hid_cid == 0xFFFF) {
        direct_l2cap_connection_t* dl = find_direct_l2cap_by_conn_index(conn_index);
        if (dl && dl->active && dl->state == DIRECT_L2CAP_STATE_CONNECTED) {
            // Build HID packet: 0xA2 (DATA|OUTPUT) + report_id + data
            // Buffer must fit DS5 BT output (79 bytes: 0xA2 + 78-byte report with CRC)
            static uint8_t direct_l2cap_send_buf[80];
            if (len + 2 > sizeof(direct_l2cap_send_buf)) return false;
            direct_l2cap_send_buf[0] = 0xA2;  // DATA | OUTPUT
            direct_l2cap_send_buf[1] = report_id;
            memcpy(direct_l2cap_send_buf + 2, data, len);
            return l2cap_send(dl->interrupt_cid, direct_l2cap_send_buf, len + 2) == ERROR_CODE_SUCCESS;
        }
        return false;
    }

    // hid_host_send_report stores a pointer to the data and sends asynchronously.
    // Copy into static buffer so the data persists until the actual L2CAP send completes.
    // (DS5 audio report 0x36 does NOT go through here — it uses
    // btstack_classic_send_interrupt_raw with a captured L2CAP CID.)
    static uint8_t hid_host_report_buf[80];
    if (len > sizeof(hid_host_report_buf)) return false;
    if (len > 0) memcpy(hid_host_report_buf, data, len);

    return hid_host_send_report(conn->hid_cid, report_id, hid_host_report_buf, len) == ERROR_CODE_SUCCESS;
}

// Check if the L2CAP interrupt channel can accept a packet right now.
// Use this to skip expensive SBC encoding when the CYW43 ACL buffers are full.
bool btstack_classic_can_send_interrupt(uint8_t conn_index)
{
#if defined(CONFIG_DS5_DROP_SCREAM) || defined(CONFIG_DS4_SPEAKER_AUDIO)
    if (conn_index >= MAX_CLASSIC_CONNECTIONS) return false;
    classic_connection_t* conn = &classic_state.connections[conn_index];
    if (!conn->active || !conn->hid_ready) return false;

    uint16_t interrupt_cid = 0;
    if (conn->hid_cid == 0xFFFF) {
        direct_l2cap_connection_t* dl = find_direct_l2cap_by_conn_index(conn_index);
        if (dl && dl->active) {
            interrupt_cid = dl->interrupt_cid;
        }
    } else {
        for (int ci = 0; ci < MAX_CLASSIC_CONNECTIONS; ci++) {
            if (hid_intr_cids[ci].cid != 0 &&
                memcmp(hid_intr_cids[ci].addr, conn->addr, 6) == 0) {
                interrupt_cid = hid_intr_cids[ci].cid;
                break;
            }
        }
    }
    if (interrupt_cid == 0) return false;
    return l2cap_can_send_packet_now(interrupt_cid);
#else
    (void)conn_index;
    return false;
#endif
}

// Send a prebuilt HID interrupt packet (0xA2 + report incl. CRC) directly on
// the L2CAP interrupt channel, bypassing the HID Host send state machine.
// Audio streaming needs per-packet can-send-now pacing that hid_host_send_report's
// single-pending-report design can't give.
// The CID comes from our L2CAP_EVENT_CHANNEL_OPENED capture — no BTstack mods.
bool btstack_classic_send_interrupt_raw(uint8_t conn_index, const uint8_t* data, uint16_t len)
{
#if defined(CONFIG_DS5_DROP_SCREAM) || defined(CONFIG_DS4_SPEAKER_AUDIO)
    if (conn_index >= MAX_CLASSIC_CONNECTIONS) return false;
    classic_connection_t* conn = &classic_state.connections[conn_index];
    if (!conn->active || !conn->hid_ready) return false;

    uint16_t interrupt_cid = 0;
    if (conn->hid_cid == 0xFFFF) {
        // Direct-L2CAP path (Sony-on-CYW43 outgoing connections):
        // the channels are our own — CIDs live in direct_l2cap_conns.
        direct_l2cap_connection_t* dl = find_direct_l2cap_by_conn_index(conn_index);
        if (dl && dl->active) {
            interrupt_cid = dl->interrupt_cid;
        }
    } else {
        for (int ci = 0; ci < MAX_CLASSIC_CONNECTIONS; ci++) {
            if (hid_intr_cids[ci].cid != 0 &&
                memcmp(hid_intr_cids[ci].addr, conn->addr, 6) == 0) {
                interrupt_cid = hid_intr_cids[ci].cid;
                break;
            }
        }
    }
    if (interrupt_cid == 0) return false;
    if (!l2cap_can_send_packet_now(interrupt_cid)) return false;

    // l2cap_send copies into the HCI outgoing buffer; caller's buffer need not persist
    return l2cap_send(interrupt_cid, (uint8_t*)data, len) == ERROR_CODE_SUCCESS;
#else
    (void)conn_index;
    (void)data;
    (void)len;
    return false;
#endif
}

// Get connection info for bthid driver matching (Classic)
bool btstack_classic_get_connection(uint8_t conn_index, btstack_classic_conn_info_t* info)
{
    if (!info) return false;

    if (conn_index >= MAX_CLASSIC_CONNECTIONS) return false;

    classic_connection_t* conn = &classic_state.connections[conn_index];
    if (!conn->active) return false;

    info->active = conn->active;
    memcpy(info->bd_addr, conn->addr, 6);
    strncpy(info->name, conn->name, sizeof(info->name) - 1);
    info->name[sizeof(info->name) - 1] = '\0';
    memcpy(info->class_of_device, conn->class_of_device, 3);
    info->vendor_id = conn->vendor_id;
    info->product_id = conn->product_id;
    info->hid_ready = conn->hid_ready;

    return true;
}

// Get number of active connections (Classic)
uint8_t btstack_classic_get_connection_count(void)
{
    uint8_t count = 0;
    for (int i = 0; i < MAX_CLASSIC_CONNECTIONS; i++) {
        if (classic_state.connections[i].active) {
            count++;
        }
    }
    return count;
}

// ============================================================================
// DISCONNECT ALL
// ============================================================================

void btstack_host_disconnect_all_devices(void)
{
    printf("[BTSTACK_HOST] Disconnecting all devices...\n");

    for (int i = 0; i < MAX_CLASSIC_CONNECTIONS; i++) {
        if (direct_l2cap_conns[i].active) {
            if (direct_l2cap_conns[i].acl_handle != HCI_CON_HANDLE_INVALID) {
                gap_disconnect(direct_l2cap_conns[i].acl_handle);
            }
            direct_l2cap_teardown_conn(&direct_l2cap_conns[i], "disconnect all requested");
        }
    }

    for (int i = 0; i < MAX_CLASSIC_CONNECTIONS; i++) {
        classic_connection_t* c = &classic_state.connections[i];
        if (!c->active) continue;
        hci_connection_t* hci_conn = hci_connection_for_bd_addr_and_type(
            c->addr, BD_ADDR_TYPE_ACL);
        if (hci_conn) {
            gap_disconnect(hci_conn->con_handle);
        } else if (c->hid_cid && c->hid_cid != 0xFFFF) {
            hid_host_disconnect(c->hid_cid);  // fallback
        }
        memset(c, 0, sizeof(classic_connection_t));
    }
}

void btstack_host_disconnect_device(uint8_t conn_index)
{
    if (conn_index >= MAX_CLASSIC_CONNECTIONS) return;
    direct_l2cap_connection_t* dl = find_direct_l2cap_by_conn_index(conn_index);
    if (dl && dl->active) {
        if (dl->acl_handle != HCI_CON_HANDLE_INVALID) {
            gap_disconnect(dl->acl_handle);
        }
        direct_l2cap_teardown_conn(dl, "disconnect device requested");
        return;
    }
    classic_connection_t* c = &classic_state.connections[conn_index];
    if (!c->active) return;
    printf("[BTSTACK_HOST] Disconnecting device index %d...\n", conn_index);
    hci_connection_t* hci_conn = hci_connection_for_bd_addr_and_type(
        c->addr, BD_ADDR_TYPE_ACL);
    if (hci_conn) {
        gap_disconnect(hci_conn->con_handle);
    } else if (c->hid_cid && c->hid_cid != 0xFFFF) {
        hid_host_disconnect(c->hid_cid);
    }
    memset(c, 0, sizeof(classic_connection_t));
}

// ============================================================================
// BOND MANAGEMENT
// ============================================================================

void btstack_host_delete_all_bonds(void)
{
    printf("[BTSTACK_HOST] Deleting all Bluetooth bonds...\n");

#if !defined(BTSTACK_USE_CYW43) && !defined(BTSTACK_USE_ESP32) && !defined(BTSTACK_USE_NRF)
    // Erase BTstack flash banks to force clean re-initialization
    // This is more reliable than using BTstack's delete APIs when flash was corrupted
    btstack_erase_flash_banks();

    // Re-initialize the TLV context to pick up the erased banks
    const hal_flash_bank_t *flash_bank = pico_flash_bank_instance();
    btstack_tlv_flash_bank_init_instance(&btstack_tlv_flash_bank_context,
                                          flash_bank, NULL);
    printf("[BTSTACK_HOST] TLV re-initialized with clean flash banks\n");
#else
    // For CYW43/ESP32, use BTstack's standard APIs
    gap_delete_all_link_keys();
    printf("[BTSTACK_HOST] Classic BT link keys deleted\n");
#endif

    // Resume scanning. delete_all_bonds() disconnected every link, leaving the
    // host idle with nothing driving discovery — so a freshly-unbonded controller
    // put into pairing mode was never seen (you had to reboot to pair again).
    // Kick scanning back on so a re-pair works immediately.
    btstack_host_start_timed_scan(15000);

    printf("[BTSTACK_HOST] All bonds cleared. Devices will need to re-pair.\n");
}

// Enumerate persisted Classic BT link keys.
//
// Classic bonds live in the link key DB: hci_set_link_key_db() is called on both
// transports (the USB dongle path in setup_tlv_storage() above, and the CYW43
// path in the SDK's btstack_cyw43.c), so gap_link_key_iterator_*() is the one
// accessor that works for both.
int btstack_host_list_classic_bonds(uint8_t addrs_out[][6], int max_count)
{
#ifdef ENABLE_CLASSIC
    if (!addrs_out || max_count <= 0) return 0;

    btstack_link_key_iterator_t it;
    if (!gap_link_key_iterator_init(&it)) {
        // iterator_init is optional in the btstack_link_key_db interface, and
        // gap_link_key_iterator_init() also returns 0 before HCI reaches the
        // working state. Report "no bonds" rather than failing the whole query.
        return 0;
    }

    int count = 0;
    bd_addr_t addr;
    link_key_t link_key;
    link_key_type_t type;
    while (count < max_count &&
           gap_link_key_iterator_get_next(&it, addr, link_key, &type)) {
        memcpy(addrs_out[count++], addr, 6);
    }
    gap_link_key_iterator_done(&it);

    // Don't leave key material on the stack — callers only ever want addresses.
    memset(link_key, 0, sizeof(link_key));
    return count;
#else
    (void)addrs_out;
    (void)max_count;
    return 0;
#endif
}

void btstack_host_forget_device(const uint8_t bd_addr[6])
{
    if (!hid_state.initialized) return;

    bd_addr_t addr;
    memcpy(addr, bd_addr, 6);

    printf("[BTSTACK_HOST] Forgetting device %02X:%02X:%02X:%02X:%02X:%02X\n",
           addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Remove Classic link key
#ifdef ENABLE_CLASSIC
    gap_drop_link_key_for_bd_addr(addr);
#endif
}
