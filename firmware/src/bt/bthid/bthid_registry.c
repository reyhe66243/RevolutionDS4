// bthid_registry.c - BTHID Driver Registration
// Registers all Bluetooth HID device drivers

#include "bthid_registry.h"
#include "bthid.h"

// Include only DS4 BT driver
#include "devices/vendors/sony/ds4_bt.h"

void bthid_registry_init(void)
{
    // Initialize BTHID layer
    bthid_init();

    // Register Sony DualShock 4 only
    ds4_bt_register();
}
