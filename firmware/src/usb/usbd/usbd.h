// usbd.h - USB device output
// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Robert Dale Smith
//
// USB device output for the RevolutionDS4 adapter: a single emulated
// DualShock 4 (PS4) HID gamepad (VID:PID 054C:09CC).

#ifndef USBD_H
#define USBD_H

#include <stdint.h>
#include <stdbool.h>
#include "core/output_interface.h"

// ============================================================================
// OUTPUT MODES
// ============================================================================

// Output modes (console types). RevolutionDS4 is a dedicated DualShock 4
// dongle, so the emulated PS4 device is the only USB output mode.
typedef enum {
    USB_OUTPUT_MODE_PS4 = 0,            // PlayStation 4 (DualShock 4)
} usb_output_mode_t;

// ============================================================================
// PUBLIC API
// ============================================================================

// Initialize USB device output
void usbd_init(void);

// Process USB device tasks (call from main loop)
void usbd_task(void);

// Send gamepad report for a player
bool usbd_send_report(uint8_t player_index);
bool ps4_mode_send_disconnected_report(uint8_t player_index);
bool ps4_mode_repeat_last_report(uint8_t player_index);

// App-overridable callback fired when the console issues a "turn off controller"
// command to the USB device. Default impl in usbd.c is a no-op; the bridge
// application overrides it to disconnect the bridged BT controller so a real
// DS4 etc. auto-sleeps after losing its host.
void app_on_console_shutdown(void);

// ============================================================================
// MODE API
// ============================================================================

// Get current output mode (always PS4)
usb_output_mode_t usbd_get_mode(void);

// Get mode name string
const char* usbd_get_mode_name(usb_output_mode_t mode);

// Output interface for app integration
extern const OutputInterface usbd_output_interface;

#endif // USBD_H
