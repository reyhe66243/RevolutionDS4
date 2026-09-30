// bt_device_db.h - Bluetooth device profile database
//
// Centralizes Bluetooth device identification so event handlers route on a
// stored profile pointer instead of repeated name comparisons.

#ifndef BT_DEVICE_DB_H
#define BT_DEVICE_DB_H

#include <stdint.h>
#include <stdbool.h>

// Classic Bluetooth connection strategy.
typedef enum {
    BT_CLASSIC_HID_HOST,     // Standard HID host (SDP + L2CAP via BTstack)
} bt_classic_strategy_t;

// PIN code type for legacy pairing.
typedef enum {
    BT_PIN_NONE,   // No PIN (uses SSP)
} bt_pin_type_t;

// HID protocol mode for hid_host_connect().
typedef enum {
    BT_HID_MODE_REPORT,   // HID_PROTOCOL_MODE_REPORT
} bt_hid_mode_t;

// Device profile - describes how to connect to a Bluetooth device type.
typedef struct {
    const char* name;              // Human-readable profile name (for logging)
    bt_classic_strategy_t classic;
    bt_hid_mode_t hid_mode;
    bt_pin_type_t pin_type;
    uint16_t default_vid;          // Default VID (0 = use SDP)
    uint16_t default_pid;          // Default PID (0 = use SDP)
} bt_device_profile_t;

// Profile constants.
extern const bt_device_profile_t BT_PROFILE_DEFAULT;
extern const bt_device_profile_t BT_PROFILE_SONY;

// Lookup a device profile by device name and/or company identifier.
// Returns the matching profile, or &BT_PROFILE_DEFAULT if there is no match.const bt_device_profile_t* bt_device_lookup(const char* name, uint16_t company_id);

// Lookup a device profile by name only.
const bt_device_profile_t* bt_device_lookup_by_name(const char* name);

#endif // BT_DEVICE_DB_H
