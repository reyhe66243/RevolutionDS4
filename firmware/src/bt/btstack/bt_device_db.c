// bt_device_db.c - Bluetooth device profile database
//
// Identification table for the controller this project supports, a DualShock 4
// connected over Bluetooth Classic. The fallback profile keeps the connection
// path functional for any other HID device, but only the DualShock 4 is
// identified explicitly.

#include "bt_device_db.h"
#include <string.h>

const bt_device_profile_t BT_PROFILE_DEFAULT = {
    .name = "Generic",
    .classic = BT_CLASSIC_HID_HOST,
    .hid_mode = BT_HID_MODE_REPORT,
    .pin_type = BT_PIN_NONE,
};

const bt_device_profile_t BT_PROFILE_SONY = {
    .name = "DualShock 4",
    .classic = BT_CLASSIC_HID_HOST,
    .hid_mode = BT_HID_MODE_REPORT,
    .pin_type = BT_PIN_NONE,
    .default_vid = 0x054C,
};

typedef struct {
    const char* substring;             // Name substring to match (strstr)
    const bt_device_profile_t* profile;
} bt_device_name_entry_t;

static const bt_device_name_entry_t name_table[] = {
    { "Wireless Controller",    &BT_PROFILE_SONY },
};

#define NAME_TABLE_SIZE (sizeof(name_table) / sizeof(name_table[0]))

const bt_device_profile_t* bt_device_lookup(const char* name, uint16_t company_id) {
    (void)company_id;

    if (name && name[0]) {
        for (unsigned i = 0; i < NAME_TABLE_SIZE; i++) {
            if (strstr(name, name_table[i].substring) != NULL) {
                return name_table[i].profile;
            }
        }
    }

    return &BT_PROFILE_DEFAULT;
}

const bt_device_profile_t* bt_device_lookup_by_name(const char* name) {
    return bt_device_lookup(name, 0);
}
