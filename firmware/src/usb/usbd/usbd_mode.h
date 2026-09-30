// usbd_mode.h - USB Device output mode interface
// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Robert Dale Smith

#ifndef USBD_MODE_H
#define USBD_MODE_H

#include <stdint.h>
#include <stdbool.h>
#include "usbd.h"
#include "core/input_event.h"
#include "core/output_interface.h"
#include "core/services/profiles/profile.h"
#include "tusb.h"
#include "device/usbd_pvt.h"  // For usbd_class_driver_t

// Mode interface - the active USB output mode implements this
typedef struct {
    const char* name;                   // Display name (e.g., "PS4")
    usb_output_mode_t mode;             // Mode enum value

    // === Descriptors ===
    const uint8_t* (*get_device_descriptor)(void);
    const uint8_t* (*get_config_descriptor)(void);
    const uint8_t* (*get_report_descriptor)(void);  // NULL if not HID-based

    // === Lifecycle ===
    void (*init)(void);                 // Initialize state to neutral values

    // === Report Sending ===
    // Returns true if report was sent successfully
    bool (*send_report)(uint8_t player_index,
                        const input_event_t* event,
                        const profile_output_t* profile_out,
                        uint32_t buttons);

    // Ready check - returns true if USB is ready to send
    bool (*is_ready)(void);

    // === Feedback (optional - NULL if not supported) ===
    // Handle output report from host (rumble, LEDs)
    void (*handle_output)(uint8_t report_id, const uint8_t* data, uint16_t len);

    // Get simple rumble value (0-255), legacy interface
    uint8_t (*get_rumble)(void);

    // Get full feedback state (rumble L/R, LEDs)
    bool (*get_feedback)(output_feedback_t* fb);

    // === HID Feature Reports (optional - NULL if not needed) ===
    // Handle GET_REPORT requests
    uint16_t (*get_report)(uint8_t report_id, hid_report_type_t report_type,
                           uint8_t* buffer, uint16_t reqlen);

    // === Custom Class Driver (optional - NULL for built-in HID) ===
    const usbd_class_driver_t* (*get_class_driver)(void);

    // === Mode-specific task (optional - NULL if not needed) ===
    // Called periodically from usbd_task()
    void (*task)(void);

} usbd_mode_t;

// ============================================================================
// ANALOG -> DIGITAL TRIGGER DERIVATION
// ============================================================================
//
// Not every input driver reports a digital L2/R2 bit. The PS4 report carries
// analog trigger axes, so an analog-only press still reaches the console; the
// derived digital bit below keeps compatibility with inputs that only report
// the digital button. Threshold 5/255 (~2%) is the value ps4_mode has shipped
// with: well below a deliberate press and above the noise floor.
#define USBD_TRIGGER_DIGITAL_THRESHOLD 5

static inline bool usbd_l2_digital(const profile_output_t* profile_out, uint32_t buttons)
{
    return (buttons & PAD_BUTTON_L2) ||
           profile_out->l2_analog >= USBD_TRIGGER_DIGITAL_THRESHOLD;
}

static inline bool usbd_r2_digital(const profile_output_t* profile_out, uint32_t buttons)
{
    return (buttons & PAD_BUTTON_R2) ||
           profile_out->r2_analog >= USBD_TRIGGER_DIGITAL_THRESHOLD;
}

// ============================================================================
// MODE DECLARATIONS
// ============================================================================

extern const usbd_mode_t ps4_mode;
// PS4 auth feature report handler (called from tud_hid_set_report_cb)
void ps4_mode_set_feature_report(uint8_t report_id, const uint8_t* buffer, uint16_t bufsize);

#endif // USBD_MODE_H
