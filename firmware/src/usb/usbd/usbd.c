// usbd.c - USB device output
// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Robert Dale Smith
//
// Implements the emulated DualShock 4 / PS4 USB device mode for the
// RevolutionDS4 adapter, using the TinyUSB device stack. This is a dedicated
// DS4 dongle: PS4 is the only USB output mode.

#include "usbd.h"
#include "usbd_mode.h"
#ifdef JOYPAD_USB_FAST_CLOCK
#include "hardware/clocks.h"  // set_sys_clock_khz — see clock policy in usbd_init
#endif
#include "descriptors/ps4_descriptors.h"
#include "core/router/router.h"
#include "core/input_event.h"
#include "core/buttons.h"
#include "core/services/storage/flash.h"
#include "core/services/button/button.h"
#include "core/services/profiles/profile.h"
#ifndef DISABLE_USB_HOST
#include "usb/usbh/hid/devices/vendors/sony/sony_ds4.h"
#endif
#ifdef ENABLE_PS4_LOCAL_AUTH
#include "usb/usbd/modes/ps4_local_auth.h"
#endif
#include "core/services/players/manager.h"
#ifdef ENABLE_BTSTACK
#include "bt/btstack/btstack_host.h"
#endif
#include "tusb.h"
#include "device/usbd_pvt.h"
#include "device/dcd.h"
#include "platform/platform.h"
#include <string.h>
#include <stdio.h>

// ============================================================================
// STATE
// ============================================================================
// EVENT-DRIVEN OUTPUT STATE
// ============================================================================

// Pending input events (queued by tap callback, sent when USB ready)
#define USB_MAX_PLAYERS 4
static input_event_t pending_events[USB_MAX_PLAYERS];
static bool pending_flags[USB_MAX_PLAYERS] = {false};

// Serial number from board unique ID (12 hex chars + null)
#define USB_SERIAL_LEN 12
static char usb_serial_str[USB_SERIAL_LEN + 1];

// ============================================================================
// PROFILE PROCESSING
// ============================================================================

// Apply profile mapping (combos, button remaps) to input event
// Returns the processed buttons; analog values are updated in-place in profile_out
static uint32_t apply_usbd_profile_player(const input_event_t* event, profile_output_t* profile_out, uint8_t player_index)
{
    const profile_t* profile = profile_get_active(OUTPUT_TARGET_USB_DEVICE);

    profile_apply(profile,
                  event->buttons,
                  event->analog[ANALOG_LX], event->analog[ANALOG_LY],
                  event->analog[ANALOG_RX], event->analog[ANALOG_RY],
                  event->analog[ANALOG_L2], event->analog[ANALOG_R2],
                  event->analog[ANALOG_RZ],
                  profile_out);

    // Custom profile (button remap, stick sensitivity, SOCD, axis inversion,
    // thresholds) is applied in router_submit_input() so it works uniformly
    // across all output interfaces.

    // Copy motion data through (no remapping)
    profile_out->has_motion = event->has_motion;
    if (event->has_motion) {
        profile_out->accel[0] = event->accel[0];
        profile_out->accel[1] = event->accel[1];
        profile_out->accel[2] = event->accel[2];
        profile_out->gyro[0] = event->gyro[0];
        profile_out->gyro[1] = event->gyro[1];
        profile_out->gyro[2] = event->gyro[2];
    }

    // Copy pressure data through (no remapping)
    profile_out->has_pressure = event->has_pressure;
    if (event->has_pressure) {
        for (int i = 0; i < 12; i++) {
            profile_out->pressure[i] = event->pressure[i];
        }
    }

    return profile_out->buttons;
}

// ============================================================================
// MODE SELECTION API
// ============================================================================

usb_output_mode_t usbd_get_mode(void)
{
    return USB_OUTPUT_MODE_PS4;
}

const char* usbd_get_mode_name(usb_output_mode_t mode)
{
    (void)mode;
    return "PS4";
}

// ============================================================================
// EVENT-DRIVEN TAP CALLBACK
// ============================================================================

// Called by router immediately when input arrives (push-based notification)
static void usbd_on_input(output_target_t output, uint8_t player_index, const input_event_t* event)
{
    (void)output;  // Always USB_DEVICE

    if (player_index >= USB_MAX_PLAYERS || !event) {
        return;
    }

    // Queue the event for sending when USB is ready
    pending_events[player_index] = *event;
    pending_flags[player_index] = true;
}

// ============================================================================
// PUBLIC API
// ============================================================================

void usbd_init(void)
{
#ifdef DISABLE_USB_DEVICE
    printf("[usbd] USB device DISABLED\n");
    return;
#endif
#ifdef JOYPAD_USB_FAST_CLOCK
    // USB-output controller-emulation apps run at 200 MHz — the fastest
    // build-supported clock, same value GP2040-CE uses — so PS4 local-auth
    // RSA signing completes inside the console's ~challenge window (~1.7 s vs
    // ~3.4 s at 125 MHz). Set here, once, before the USB stack starts: this is
    // a board-level clock policy, not a per-driver runtime hack.
    bool clk_ok = set_sys_clock_khz(200000, true);
    printf("[usbd] sys_clock=200MHz set: %s\n", clk_ok ? "OK" : "FAIL");
#endif
    printf("[usbd] Initializing USB device output\n");

    // Initialize and load settings from flash
    flash_init();
#ifdef ENABLE_PS4_LOCAL_AUTH
    // Load PS4 auth key material from flash (requires flash_init to have run first)
    ps4_local_auth_init();
#endif

    printf("[usbd] Mode: PS4\n");


    // Get unique board ID for USB serial number (first 12 chars)
    char full_id[17];  // Up to 8 bytes * 2 hex chars + null
    platform_get_serial(full_id, sizeof(full_id));
    memcpy(usb_serial_str, full_id, USB_SERIAL_LEN);
    usb_serial_str[USB_SERIAL_LEN] = '\0';
    printf("[usbd] Serial: %s\n", usb_serial_str);

    // Initialize TinyUSB device stack
    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_AUTO
    };
    tusb_init(0, &dev_init);
    dcd_sof_enable(0, true);

    // Initialize the PS4 report state to neutral values
    if (ps4_mode.init) {
        ps4_mode.init();
    }

    // Register tap callback for event-driven input (push-based notification)
    router_set_tap(OUTPUT_TARGET_USB_DEVICE, usbd_on_input);

    printf("[usbd] Initialization complete\n");
}

// ============================================================================
// REMOTE WAKEUP
// ============================================================================

// Wake a suspended host on real user input.
//
// This has to live above the report dispatch in usbd_task(), because every
// gate below it bottoms out in tud_ready() — which TinyUSB defines as
// tud_mounted() && !tud_suspended(), i.e. false exactly when the host is
// asleep. Both the mode->is_ready() checks here and the TU_VERIFY(tud_*_ready())
// at the top of each driver's send_report() therefore return early while
// suspended, so a tud_remote_wakeup() placed anywhere inside those paths is
// unreachable. Three drivers used to carry exactly that dead branch.
//
// Firing on router activity rather than on every task tick matters twice over:
// router_ms_since_activity() is only stamped by a held button or a stick pushed
// past a +/-24 noise margin, so a controller resting on a desk cannot wake the
// host, and dcd_remote_wakeup() drives resume signalling on the bus, so it must
// not be issued on idle polls. Retried on an interval because a single resume
// can be missed by the host.
#define USBD_WAKE_ACTIVITY_WINDOW_MS 500  // how recent input must be to count
#define USBD_WAKE_RETRY_MS           250  // min gap between resume attempts

static uint32_t usbd_wake_last_attempt_ms = 0;
static bool     usbd_wake_armed = true;   // first attempt of a suspend is immediate

static void usbd_try_remote_wakeup(void)
{
    if (!tud_suspended()) {
        usbd_wake_armed = true;  // re-arm for the next suspend
        return;
    }

    // On Nintendo Wii, Starlet never drives K-state resume or enables remote wakeup
    // after an IOS reload (launching USB Loader GX or booting a game).
    // If the bus was suspended during the reload, we immediately clear TinyUSB's
    // suspended state so that tud_hid_ready() becomes true and reports can flow.
    dcd_event_bus_signal(0, DCD_EVENT_RESUME, false);
}

void usbd_task(void)
{
    // TinyUSB device task - runs from core0 main loop
    // On ESP32 (FreeRTOS), tud_task() blocks forever (UINT32_MAX timeout).
    // Use tud_task_ext with short timeout so the main loop keeps running.
#ifdef PLATFORM_ESP32
    tud_task_ext(1, false);
#else
    tud_task();
#endif

    // Must precede the report dispatch below — nothing past this point runs
    // while the host is suspended. See usbd_try_remote_wakeup().
    usbd_try_remote_wakeup();

    // PS4 mode: run the mode task (RSA signing), then send HID reports for up
    // to two players.
    if (ps4_mode.task) ps4_mode.task();
    if (tud_hid_ready()) {
        static uint8_t next_player = 0;
        static uint32_t last_report_ms[2] = {0, 0};
        uint32_t now_ms = platform_time_ms();
        bool sent = false;

        for (int i = 0; i < 2; i++) {
            uint8_t p = (next_player + i) % 2;
            if (pending_flags[p]) {
                if (usbd_send_report(p)) {
                    last_report_ms[p] = now_ms;
                    next_player = (p + 1) % 2;
                    sent = true;
                    break;
                }
            }
        }

        // If a player has an active BT controller, stream a report every 4ms so the
        // Starlet /dev/usb/hid IN endpoint is continuously fed (200-250 Hz) and never starves
        if (!sent) {
            for (int i = 0; i < 2; i++) {
                uint8_t p = (next_player + i) % 2;
                bool p_connected = (playersCount > p && players[p].dev_addr >= 0);
#ifdef ENABLE_BTSTACK
                if (!btstack_host_has_ready_connection()) p_connected = false;
#endif
                if (p_connected && (now_ms - last_report_ms[p] >= 4)) {
                    if (ps4_mode_repeat_last_report(p)) {
                        last_report_ms[p] = now_ms;
                        next_player = (p + 1) % 2;
                        sent = true;
                        break;
                    }
                }
            }
        }

        // If Player 0 has no active BT controller, stream a disconnected status report
        // every 50ms so Starlet /dev/usb/hid IN endpoint never hangs or exhausts queues
        if (!sent && !pending_flags[0]) {
            static uint32_t last_discon_ms = 0;
            bool p0_connected = (playersCount > 0 && players[0].dev_addr >= 0);
#ifdef ENABLE_BTSTACK
            if (!btstack_host_has_ready_connection()) p0_connected = false;
#endif
            if (!p0_connected && (now_ms - last_discon_ms >= 50)) {
                last_discon_ms = now_ms;
                ps4_mode_send_disconnected_report(0);
            }
        }
    }
}

// Send PS4 report - delegates to mode interface
static bool usbd_send_ps4_report(uint8_t player_index)
{
    const usbd_mode_t* mode = &ps4_mode;
    if (!mode || !mode->send_report) return false;
    if (mode->is_ready && !mode->is_ready()) return false;

    // Check for pending event
    if (player_index >= USB_MAX_PLAYERS || !pending_flags[player_index]) {
        return false;
    }

    const input_event_t* event = &pending_events[player_index];

    // Apply profile
    profile_output_t profile_out;
    uint32_t processed_buttons = apply_usbd_profile_player(event, &profile_out, player_index);

    // Only consume the pending event when the report actually went out.
    // tud_hid_report() returns false while the IN endpoint is still busy
    // (e.g. when the console is sending audio output transfers); clearing the
    // flag before the call used to silently drop that input frame.
    if (!mode->send_report(player_index, event, &profile_out, processed_buttons)) {
        return false;
    }
    pending_flags[player_index] = false;
    return true;
}

bool usbd_send_report(uint8_t player_index)
{
    return usbd_send_ps4_report(player_index);
}

// Get rumble value from USB host (for feedback to input controllers)
static uint8_t usbd_get_rumble(void)
{
    if (ps4_mode.get_rumble) {
        return ps4_mode.get_rumble();
    }
    return 0;
}

// ============================================================================
// OUTPUT INTERFACE
// ============================================================================

// Get feedback state with separate left/right rumble and LED data
static bool usbd_get_feedback(output_feedback_t* fb)
{
    if (!fb) return false;

    fb->rumble_left = 0;
    fb->rumble_right = 0;
    fb->led_player = 0;
    fb->led_r = fb->led_g = fb->led_b = 0;
    fb->dirty = false;

    if (ps4_mode.get_feedback) {
        return ps4_mode.get_feedback(fb);
    }
    return false;
}

const OutputInterface usbd_output_interface = {
    .name = "USB",
    .target = OUTPUT_TARGET_USB_DEVICE,
    .init = usbd_init,
    .task = usbd_task,
    .core1_task = NULL,  // Runs from core0 task - doesn't need dedicated core
    .get_feedback = usbd_get_feedback,
    .get_rumble = usbd_get_rumble,
    .get_player_led = NULL,
    .get_profile_count = NULL,
    .get_active_profile = NULL,
    .set_active_profile = NULL,
    .get_profile_name = NULL,
    .get_trigger_threshold = NULL,
};

// ============================================================================
// TINYUSB DEVICE CALLBACKS
// ============================================================================

// ============================================================================
// DEVICE DESCRIPTOR
// ============================================================================

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&ps4_device_descriptor;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    if (tud_suspended()) {
        dcd_event_bus_signal(0, DCD_EVENT_RESUME, false);
    }
    return ps4_config_descriptor;
}

uint8_t const *tud_descriptor_device_qualifier_cb(void)
{
    return NULL;
}

#ifdef PLATFORM_CH32
// After a valid qualifier the host requests the OTHER_SPEED_CONFIGURATION. The
// device behaves identically at FS, so report the active config descriptor.
uint8_t const *tud_descriptor_other_speed_configuration_cb(uint8_t index)
{
    return tud_descriptor_configuration_cb(index);
}
#endif

// ============================================================================
// STRING DESCRIPTORS
// ============================================================================

// String descriptor indices
enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
#if CFG_TUD_CDC >= 1
    STRID_CDC_DATA,
#endif
    STRID_COUNT
};

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;

    static uint16_t _desc_str[32];
    const char *str = NULL;
    uint8_t chr_count;

    switch (index) {
        case STRID_LANGID:
            _desc_str[1] = 0x0409;  // English
            chr_count = 1;
            break;
        case STRID_MANUFACTURER:
            str = PS4_MANUFACTURER;
            break;
        case STRID_PRODUCT:
            str = PS4_PRODUCT;
            break;
        case STRID_SERIAL:
            str = usb_serial_str;  // Dynamic from board unique ID
            break;
#if CFG_TUD_CDC >= 1
        case STRID_CDC_DATA:
            str = "REVOLUTIONDS4 Data";
            break;
#endif
        default:
            return NULL;
    }

    if (str) {
        chr_count = strlen(str);
        if (chr_count > 31) chr_count = 31;
        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }

    // First byte is length (in bytes), second byte is descriptor type
    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);
    return _desc_str;
}

// HID Callbacks
uint8_t const *tud_hid_descriptor_report_cb(uint8_t itf)
{
    (void)itf;
    return ps4_report_descriptor;
}

uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    (void)itf;
    if (ps4_mode.get_report) {
        return ps4_mode.get_report(report_id, report_type, buffer, reqlen);
    }
    return 0;
}

// Weak default for app_on_console_shutdown()
__attribute__((weak)) void app_on_console_shutdown(void)
{
}

void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
    (void)itf;
    if (tud_suspended()) {
        dcd_event_bus_signal(0, DCD_EVENT_RESUME, false);
    }
    if (ps4_mode.handle_output) {
        ps4_mode.handle_output(report_id, buffer, bufsize);
    }
    if (report_type == HID_REPORT_TYPE_FEATURE) {
        ps4_mode_set_feature_report(report_id, buffer, bufsize);
    }
}

// ============================================================================
// CUSTOM CLASS DRIVER REGISTRATION
// ============================================================================

// No custom class drivers: the PS4 mode uses the built-in HID class.
usbd_class_driver_t const* usbd_app_driver_get_cb(uint8_t* driver_count)
{
    *driver_count = 0;
    return NULL;
}

// Vendor control request callback
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                 tusb_control_request_t const* request)
{
    (void)rhport;
    (void)stage;
    (void)request;
    return true;
}
