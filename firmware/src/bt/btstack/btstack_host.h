// btstack_host.h - BTstack HID Host (Classic BT)
//
// Transport-agnostic BTstack integration for HID devices.
// Uses BTstack's HID Host for Classic BT HID devices (DualShock 4).
//
// Usage:
//   btstack_host_init(hci_transport);  // Pass HCI transport (USB dongle or CYW43)
//   btstack_host_process();            // Call from main loop

#ifndef BTSTACK_HOST_H
#define BTSTACK_HOST_H

#include <stdint.h>
#include <stdbool.h>
#include "bluetooth.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// INITIALIZATION
// ============================================================================

// Initialize BTstack with specified HCI transport
// For USB dongle: pass hci_transport_h2_tinyusb_instance()
// For Pico W: pass hci_transport_cyw43_instance()
// Note: transport is actually hci_transport_t* but we use void* to avoid header conflicts
void btstack_host_init(const void* transport);

// Initialize only the HID handlers (callbacks, state) without BTstack init
// Use this when BTstack was already initialized externally (e.g., btstack_cyw43_init)
void btstack_host_init_hid_handlers(void);

// Power on the Bluetooth controller
void btstack_host_power_on(void);

// ============================================================================
// SCANNING / PAIRING
// ============================================================================

// Start scanning (Classic inquiry) for BT devices
void btstack_host_start_scan(void);

// Stop scanning
void btstack_host_stop_scan(void);

// Start scanning with a timeout (auto-stops after timeout_ms)
void btstack_host_start_timed_scan(uint32_t timeout_ms);

// Suppress/unsuppress automatic scan restart (e.g. when USB device connected).
// Explicit start_timed_scan clears suppression.
void btstack_host_suppress_scan(bool suppress);

// Link policy / scan controls
void gap_discoverable_control(uint8_t enable);
void gap_connectable_control(uint8_t enable);

// ============================================================================
// MAIN LOOP
// ============================================================================

// Process BTstack events - call from main loop
void btstack_host_process(void);

// ============================================================================
// STATUS
// ============================================================================

bool btstack_host_is_initialized(void);
bool btstack_host_is_powered_on(void);
bool btstack_host_is_scanning(void);
bool btstack_host_has_ready_connection(void);
bool btstack_host_is_connecting(void);

// ============================================================================
// CLASSIC BT CONNECTION INFO (for bthid driver matching)
// ============================================================================

typedef struct {
    bool active;
    uint8_t bd_addr[6];
    char name[48];
    uint8_t class_of_device[3];
    uint16_t vendor_id;
    uint16_t product_id;
    bool hid_ready;
} btstack_classic_conn_info_t;

bool btstack_classic_get_connection(uint8_t conn_index, btstack_classic_conn_info_t* info);
uint8_t btstack_classic_get_connection_count(void);

// Enumerate persisted Classic BT link keys — the bonds a Classic controller
// (DS4/DS5, Switch Pro) reconnects with. Writes up to max_count
// addresses and returns how many were written. No name is stored alongside a
// link key, so callers get the BD_ADDR only: enough to show the bond exists and
// to pass to btstack_host_forget_device().
int btstack_host_list_classic_bonds(uint8_t addrs_out[][6], int max_count);

// Classic BT output (for bthid drivers)
bool btstack_classic_send_set_report_type(uint8_t conn_index, uint8_t report_type,
                                           uint8_t report_id, const uint8_t* data, uint16_t len);
bool btstack_classic_send_set_report(uint8_t conn_index, uint8_t report_id,
                                      const uint8_t* data, uint16_t len);
bool btstack_classic_send_get_report(uint8_t conn_index, uint8_t report_type,
                                      uint8_t report_id);
bool btstack_classic_send_report(uint8_t conn_index, uint8_t report_id,
                                  const uint8_t* data, uint16_t len);

// ============================================================================
// BOND MANAGEMENT
// ============================================================================

// Disconnect all active BT devices (Classic)
void btstack_host_disconnect_all_devices(void);

// Disconnect a specific BT device by connection index (Classic)
void btstack_host_disconnect_device(uint8_t conn_index);

// Delete all stored BT bonds (Classic)
// Devices will need to re-pair after this
void btstack_host_delete_all_bonds(void);

// Forget a specific device by address — disconnects if connected, removes bond
void btstack_host_forget_device(const uint8_t bd_addr[6]);

#ifdef __cplusplus
}
#endif

#endif // BTSTACK_HOST_H
