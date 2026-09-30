// ps4_mode.c - PlayStation 4 DualShock 4 USB device mode
// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Robert Dale Smith

#include "platform/platform.h"
#include "tusb.h"
#include "../usbd_mode.h"
#include "../usbd.h"
#include "descriptors/ps4_descriptors.h"
#include "core/buttons.h"
#include <stddef.h>
#include <string.h>

#ifndef DISABLE_USB_HOST
#include "usb/usbh/hid/devices/vendors/sony/sony_ds4.h"

// ps4_report_buffer below is the wire report *including* the report ID at
// index 0, so every field sits one byte later than in sony_ds4_report_t --
// the same DS4 layout this firmware already parses on the host side. The two
// were agreed by hand and nothing enforced it, which is how the accelerometer
// rest value came to be written one byte past the axis it was meant for.
// These pin the offsets this file hardcodes to the struct, so the next edit to
// either one fails the build instead of shipping a silent byte shift.
_Static_assert(offsetof(sony_ds4_report_t, gyro) + 1 == 13,
               "DS4 gyro must start at report byte 13");
_Static_assert(offsetof(sony_ds4_report_t, accel) + 1 == 19,
               "DS4 accel must start at report byte 19");
_Static_assert(offsetof(sony_ds4_report_t, headset) + 1 == 30,
               "DS4 status/power byte must be report byte 30");
_Static_assert(offsetof(sony_ds4_report_t, tpad_counter) + 1 == 34,
               "DS4 touchpad packet counter must be report byte 34");
_Static_assert(offsetof(sony_ds4_report_t, tpad_f1_pos) + 1 == 36,
               "DS4 touchpad finger 1 position must start at report byte 36");
#endif

// Vendor detection for the IMU frame remap below.
#if (defined(CONFIG_USB_HOST) || defined(CONFIG_USB)) && !defined(DISABLE_USB_HOST)
#include "usb/usbh/hid/hid_registry.h"
extern int hid_get_ctrl_type(uint8_t dev_addr, uint8_t instance);
#endif
#ifdef ENABLE_BTSTACK
#include "bt/bthid/bthid.h"
#endif

// True when the source input is a an external tool Controller 2 (USB). The SC2 reports
// its IMU in the Triton device frame, which needs an SC2-specific remap to the DS4
// output's expected (SDL) frame; other inputs (DS4/DualSense) are already in-frame.
static bool ps4_input_is_sc2(uint8_t dev_addr, int8_t instance)
{
#if (defined(CONFIG_USB_HOST) || defined(CONFIG_USB)) && !defined(DISABLE_USB_HOST)
    if (hid_get_ctrl_type(dev_addr, (uint8_t)instance) == CONTROLLER_STEAM_2) return true;
#else
    (void)instance;
#endif
#ifdef ENABLE_BTSTACK
    bthid_device_t* d = bthid_get_device(dev_addr);
    if (d && d->vendor_id == 0x28DE) return true;
#endif
    (void)dev_addr;
    return false;
}

// ============================================================================
// STATE
// ============================================================================

// Using raw byte buffer approach to avoid struct bitfield packing issues
static uint8_t ps4_report_buffer[2][64];
static ps4_out_report_t ps4_outputs[2];
static bool ps4_output_availables[2] = {false, false};
static uint8_t ps4_feedback_poll_idx = 0;
static uint8_t ps4_report_counter[2] = {0, 0};
static uint16_t ps4_timestamp = 0;


// ============================================================================
// MODE INTERFACE IMPLEMENTATION
// ============================================================================

static void ps4_mode_init(void)
{
    // Initialize PS4 reports for up to 2 players to neutral state (raw buffer approach)
    for (int p = 0; p < 2; p++) {
        uint8_t* rpt = ps4_report_buffer[p];
        uint8_t r_id = (p == 1) ? PS4_REPORT_ID_INPUT_P2 : PS4_REPORT_ID_INPUT;
        memset(rpt, 0, 64);
        rpt[0] = r_id;  // Report ID
        rpt[1] = 0x80;  // LX center
        rpt[2] = 0x80;  // LY center
        rpt[3] = 0x80;  // RX center
        rpt[4] = 0x80;  // RY center
        rpt[5] = PS4_HAT_NOTHING;  // D-pad neutral (0x08), no buttons
        rpt[23] = 0x60; // Accel Z ~1G
        rpt[24] = 0x20;
        rpt[30] = 0x1b; // Power level
        rpt[33] = 0x00; // 0 touches
        rpt[34] = 0x00; // increment
        rpt[35] = 0x80; // touchpad p1 unpressed
        rpt[36] = 0xC0; // X LSB
        rpt[37] = 0x73; // X MSB / Y LSB
        rpt[38] = 0x1D; // Y MSB
        rpt[39] = 0x80; // touchpad p2 unpressed
        rpt[40] = 0xC0;
        rpt[41] = 0x73;
        rpt[42] = 0x1D;
    }
    memset(ps4_outputs, 0, sizeof(ps4_outputs));
    ps4_output_availables[0] = false;
    ps4_output_availables[1] = false;
    ps4_feedback_poll_idx = 0;
    ps4_report_counter[0] = 0;
    ps4_report_counter[1] = 0;
    ps4_timestamp = 0;
}

static bool ps4_mode_is_ready(void)
{
    return tud_hid_ready();
}

// Send PS4 report (PlayStation 4 DualShock 4 mode)
// Uses raw byte array approach to avoid struct bitfield packing issues
//
// PS4 Report Layout (64 bytes):
//   Byte 0:    Report ID (0x01)
//   Byte 1:    Left stick X (0x00-0xFF, 0x80 center)
//   Byte 2:    Left stick Y (0x00-0xFF, 0x80 center)
//   Byte 3:    Right stick X (0x00-0xFF, 0x80 center)
//   Byte 4:    Right stick Y (0x00-0xFF, 0x80 center)
//   Byte 5:    D-pad (bits 0-3) + Square/Cross/Circle/Triangle (bits 4-7)
//   Byte 6:    L1/R1/L2/R2/Share/Options/L3/R3 (bits 0-7)
//   Byte 7:    PS (bit 0) + Touchpad (bit 1) + Counter (bits 2-7)
//   Byte 8:    Left trigger analog (0x00-0xFF)
//   Byte 9:    Right trigger analog (0x00-0xFF)
//   Bytes 10-11: Axis timing (timestamp)
//   Bytes 12-63: Sensor data, touchpad data, padding
static bool ps4_mode_send_report(uint8_t player_index,
                                  const input_event_t* event,
                                  const profile_output_t* profile_out,
                                  uint32_t buttons)
{
    if (player_index > 1) player_index = 0;
    uint8_t* rpt = ps4_report_buffer[player_index];
    uint8_t r_id = (player_index == 1) ? PS4_REPORT_ID_INPUT_P2 : PS4_REPORT_ID_INPUT;

    // Byte 0: Report ID
    rpt[0] = r_id;

    // Bytes 1-4: Analog sticks (HID convention: 0=up, 255=down)
    rpt[1] = profile_out->left_x;
    rpt[2] = profile_out->left_y;
    rpt[3] = profile_out->right_x;
    rpt[4] = profile_out->right_y;

    // Byte 5: D-pad (bits 0-3) + face buttons (bits 4-7)
    uint8_t up = (buttons & PAD_BUTTON_DU) ? 1 : 0;
    uint8_t down = (buttons & PAD_BUTTON_DD) ? 1 : 0;
    uint8_t left = (buttons & PAD_BUTTON_DL) ? 1 : 0;
    uint8_t right = (buttons & PAD_BUTTON_DR) ? 1 : 0;

    // Detect Touchpad Click (A2 or S2+R1 combo)
    bool s2_r1_combo = (buttons & PAD_BUTTON_S2) && (buttons & PAD_BUTTON_R1);
    bool tp_clicked = (buttons & PAD_BUTTON_A2) || s2_r1_combo;

    // Use D-pad Left/Right as modifiers for Touchpad Click
    if (tp_clicked) {
        if (left) left = 0;
        if (right) right = 0;
    }

    uint8_t dpad;
    if (up && right)        dpad = PS4_HAT_UP_RIGHT;
    else if (up && left)    dpad = PS4_HAT_UP_LEFT;
    else if (down && right) dpad = PS4_HAT_DOWN_RIGHT;
    else if (down && left)  dpad = PS4_HAT_DOWN_LEFT;
    else if (up)            dpad = PS4_HAT_UP;
    else if (down)          dpad = PS4_HAT_DOWN;
    else if (left)          dpad = PS4_HAT_LEFT;
    else if (right)         dpad = PS4_HAT_RIGHT;
    else                    dpad = PS4_HAT_NOTHING;

    uint8_t face_buttons = 0;
    if (buttons & PAD_BUTTON_B3) face_buttons |= 0x10;  // Square
    if (buttons & PAD_BUTTON_B1) face_buttons |= 0x20;  // Cross
    if (buttons & PAD_BUTTON_B2) face_buttons |= 0x40;  // Circle
    if (buttons & PAD_BUTTON_B4) face_buttons |= 0x80;  // Triangle

    rpt[5] = dpad | face_buttons;

    // Byte 6: Shoulder buttons + other buttons
    uint8_t byte6 = 0;
    if (buttons & PAD_BUTTON_L1) byte6 |= 0x01;  // L1
    if (!(s2_r1_combo)) {
        if (buttons & PAD_BUTTON_R1) byte6 |= 0x02;  // R1
    }
    
    // Hybrid Trigger Logic (High Compatibility):
    // Digital bits are flipped at USBD_TRIGGER_DIGITAL_THRESHOLD to support
    // SF6 strokes, while keeping analog values raw for FC26 replay
    // sensitivity. This mode has had it right all along - the shared helper
    // exists so the four modes that were missing it cannot drift to some
    // other threshold. Same value, same behaviour. See usbd_mode.h.
    uint8_t l2_val = profile_out->l2_analog;   // raw, for bytes 8-9 below
    uint8_t r2_val = profile_out->r2_analog;

    if (usbd_l2_digital(profile_out, buttons)) byte6 |= 0x04; // L2 Digital
    if (usbd_r2_digital(profile_out, buttons)) byte6 |= 0x08; // R2 Digital

    if (buttons & PAD_BUTTON_S1) byte6 |= 0x10;  // Share
    if (!(s2_r1_combo)) {
        if (buttons & PAD_BUTTON_S2) byte6 |= 0x20;  // Options
    }
    if (buttons & PAD_BUTTON_L3) byte6 |= 0x40;  // L3
    if (buttons & PAD_BUTTON_R3) byte6 |= 0x80;  // R3
    rpt[6] = byte6;

    // Byte 7: PS + Touchpad + Counter (6-bit)
    uint8_t byte7 = 0;
    if (buttons & PAD_BUTTON_A1) byte7 |= 0x01;  // PS button
    if (tp_clicked) byte7 |= 0x02;              // Touchpad click
    byte7 |= ((ps4_report_counter[player_index]++ & 0x3F) << 2);       // Counter in bits 2-7
    rpt[7] = byte7;

    // Bytes 8-9: Analog triggers (L2, R2)
    rpt[8] = l2_val;
    rpt[9] = r2_val;

    // Bytes 10-11: Timestamp (milissegundos, como GP2040-CE)
    ps4_timestamp = platform_time_ms(); 
    rpt[10] = ps4_timestamp & 0xFF;
    rpt[11] = (ps4_timestamp >> 8) & 0xFF;

    // Real battery level passthrough from DS4 controller
    if (event->battery_level > 0) {
        uint8_t bat = event->battery_level / 10;
        if (bat == 0) bat = 1;
        if (bat > 10) bat = 10;
        rpt[30] = (event->battery_charging ? 0x10 : 0x00) | (bat & 0x0F);
    } else {
        rpt[30] = 0x1b;
    }

    // Motion: gyro (bytes 13-18) + accel (bytes 19-24), int16 LE — the standard DS4
    // USB layout (Linux hid-playstation / joypad-web's DS4 parser). The event carries
    // raw int16 + full-scale ranges; DS4 is ±2000 dps gyro / ±4 g accel over full int16,
    // so scale each axis by its declared range into that standard.
    //   NOTE (needs a real PS4 / faithful DS4 parser to verify): axis order and signs
    //   are passed straight, but the SC2/DS device frames differ, so the orientation
    //   axes may need a per-input remap. This at least streams live motion (was static).
    if (event->has_motion) {
        int32_t gr = event->gyro_range  ? event->gyro_range  : 2000;
        int32_t ar = event->accel_range ? event->accel_range : 4000;
        int32_t gyro[3]  = { event->gyro[0],  event->gyro[1],  event->gyro[2]  };
        int32_t accel[3] = { event->accel[0], event->accel[1], event->accel[2] };
        // SC2 (Triton) IMU frame -> DS4/SDL frame: x=rawX, y=rawZ, z=-rawY (both gyro
        // and accel, per SDL's steam_triton driver). Without this the SC2 reads pitch
        // ~-90 when flat — sitting on the gimbal-lock singularity, so roll bounces.
        // DS4/DualSense inputs are already in-frame, so gate this to the SC2.
        if (ps4_input_is_sc2(event->dev_addr, event->instance)) {
            int32_t t;
            t = gyro[1];  gyro[1]  = gyro[2];  gyro[2]  = -t;
            t = accel[1]; accel[1] = accel[2]; accel[2] = -t;
        }
        for (int i = 0; i < 3; i++) {
            int32_t g = gyro[i]  * gr / 2000;
            int32_t a = accel[i] * ar / 4000;
            if (g > 32767) g = 32767; else if (g < -32768) g = -32768;
            if (a > 32767) a = 32767; else if (a < -32768) a = -32768;
            rpt[13 + i * 2] = (uint8_t)(g & 0xFF);
            rpt[14 + i * 2] = (uint8_t)((g >> 8) & 0xFF);
            rpt[19 + i * 2] = (uint8_t)(a & 0xFF);
            rpt[20 + i * 2] = (uint8_t)((a >> 8) & 0xFF);
        }
    } else {
        // No motion on this input: emit neutral (zero gyro, +1 g on Z) so a DS4 parser
        // reads "level and still" rather than the mis-aligned static value it had before.
        memset(&rpt[13], 0, 10);           // gyro XYZ + accel XY = 0
        rpt[23] = (uint8_t)(8192 & 0xFF);  // accel Z ~+1 g (8192 LSB/g)
        rpt[24] = (uint8_t)((8192 >> 8) & 0xFF);
    }

    // Touchpad (offset 33=packet count, 34=counter, 35-42=two 4-byte fingers).
    // Each finger: [id|bit7=released][X lo][ (Y lo<<4)|X hi ][Y hi], 12-bit X 0..1919
    // / Y 0..942. Must be present for the Options button to work in some titles.
    rpt[34]++;  // touch packet counter

    if (event->has_touch) {
        // Real touchpad: emit both fingers from the normalized event, scaled from the
        // device-agnostic 0..65535 canonical back to DS4 native (1919x942). Each new
        // finger-down gets a fresh id so the host tracks touches correctly.
        static uint8_t tp_id[2][2] = {{0, 0}, {0, 0}};
        static bool tp_prev[2][2] = {{false, false}, {false, false}};
        uint8_t active = 0;
        for (int f = 0; f < 2; f++) {
            uint8_t* fb = &rpt[35 + f * 4];
            bool on = event->touch[f].active;
            if (on) {
                if (!tp_prev[player_index][f]) tp_id[player_index][f] = (uint8_t)((tp_id[player_index][f] + 1) & 0x7F);
                uint16_t tx = touch_norm_to_range(event->touch[f].x, 1919);
                uint16_t ty = touch_norm_to_range(event->touch[f].y, 942);
                fb[0] = tp_id[player_index][f] & 0x7F;  // touching (bit7=0) + id
                fb[1] = (uint8_t)(tx & 0xFF);
                fb[2] = (uint8_t)(((ty & 0x0F) << 4) | ((tx >> 8) & 0x0F));
                fb[3] = (uint8_t)((ty >> 4) & 0xFF);
                active++;
            } else {
                fb[0] = 0x80; fb[1] = 0; fb[2] = 0; fb[3] = 0;  // released
            }
            tp_prev[player_index][f] = on;
        }
        rpt[33] = active ? 0x01 : 0x00;
    } else {
        // No real touchpad on this input: simulate a finger from the touchpad-click +
        // D-pad region so non-touch pads can still trigger touchpad-gated actions.
        rpt[33] = 0x00;
        uint16_t tp_x = 960, tp_y = 471;  // center
        if (tp_clicked) {
            if (buttons & PAD_BUTTON_DL)      tp_x = 480;   // left region
            else if (buttons & PAD_BUTTON_DR) tp_x = 1440;  // right region
            rpt[33] = 0x01;
            rpt[35] = 0x00;   // finger 1 pressed
        } else {
            rpt[35] = 0x80;   // finger 1 released
        }
        rpt[39] = 0x80;       // finger 2 released
        rpt[36] = (uint8_t)(tp_x & 0xFF);
        rpt[37] = (uint8_t)(((tp_y & 0x0F) << 4) | ((tp_x >> 8) & 0x0F));
        rpt[38] = (uint8_t)((tp_y >> 4) & 0xFF);
        rpt[40] = rpt[36];
        rpt[41] = rpt[37];
        rpt[42] = rpt[38];
    }

    // Send with report_id=r_id, letting TinyUSB prepend it
    // Skip byte 0 of buffer (our report_id) and send 63 bytes of data
    return tud_hid_report(r_id, &rpt[1], 63);
}

bool ps4_mode_send_disconnected_report(uint8_t player_index)
{
    if (player_index > 1) player_index = 0;
    uint8_t* rpt = ps4_report_buffer[player_index];
    uint8_t r_id = (player_index == 1) ? PS4_REPORT_ID_INPUT_P2 : PS4_REPORT_ID_INPUT;

    memset(rpt, 0, 64);
    rpt[0] = r_id;
    rpt[1] = 0x80;  // LX center
    rpt[2] = 0x80;  // LY center
    rpt[3] = 0x80;  // RX center
    rpt[4] = 0x80;  // RY center
    rpt[5] = PS4_HAT_NOTHING;
    rpt[23] = (uint8_t)(8192 & 0xFF);  // Accel Z ~1G
    rpt[24] = (uint8_t)((8192 >> 8) & 0xFF);
    rpt[30] = 0xFF;  // Special marker: CONTROLLER DISCONNECTED
    rpt[35] = 0x80;  // Finger 1 unpressed
    rpt[39] = 0x80;  // Finger 2 unpressed

    return tud_hid_report(r_id, &rpt[1], 63);
}

bool ps4_mode_repeat_last_report(uint8_t player_index)
{
    if (player_index > 1) player_index = 0;
    if (!tud_hid_ready()) return false;

    uint8_t* rpt = ps4_report_buffer[player_index];
    if (rpt[30] == 0xFF) return false;

    uint8_t r_id = (player_index == 1) ? PS4_REPORT_ID_INPUT_P2 : PS4_REPORT_ID_INPUT;

    // Advance sequence counter (bits 2-7 of byte 7)
    uint8_t b7 = rpt[7] & 0x03;
    b7 |= ((ps4_report_counter[player_index]++ & 0x3F) << 2);
    rpt[7] = b7;

    // Refresh timestamp
    uint16_t ts = (uint16_t)platform_time_ms();
    rpt[10] = ts & 0xFF;
    rpt[11] = (ts >> 8) & 0xFF;

    return tud_hid_report(r_id, &rpt[1], 63);
}


__attribute__((weak)) volatile uint32_t ds4_audio_diag_rx_count = 0;

__attribute__((weak)) void app_disconnect_player(uint8_t player_index)
{
    (void)player_index;
}

__attribute__((weak)) void ds4_audio_feed_report(const uint8_t* data, uint16_t len)
{
    (void)data;
    (void)len;
}

static void ps4_mode_handle_output(uint8_t report_id, const uint8_t* data, uint16_t len)
{
    if (!data || len == 0) return;

    // The PS4 sends the rumble/LED output report (ID 5 for P1, ID 6 for P2) on the interrupt OUT
    // endpoint, where TinyUSB delivers report_id=0 with the real ID in data[0]
    // (see hid_device.c: tud_hid_set_report_cb(itf, 0, ...OUTPUT, epout, ...)).
    // Normalize that to the control SET_REPORT form (report_id set, body only).
    if (report_id == 0 && data[0] == PS4_REPORT_ID_OUTPUT) {
        report_id = data[0];
        data += 1;
        len  -= 1;
    } else if (report_id == PS4_REPORT_ID_OUTPUT && data[0] == PS4_REPORT_ID_OUTPUT) {
        data += 1;
        len  -= 1;
    } else if (report_id == 0 && data[0] == PS4_REPORT_ID_OUTPUT_P2) {
        report_id = data[0];
        data += 1;
        len  -= 1;
    } else if (report_id == PS4_REPORT_ID_OUTPUT_P2 && data[0] == PS4_REPORT_ID_OUTPUT_P2) {
        data += 1;
        len  -= 1;
    } else if (report_id == 0 && data[0] == PS4_REPORT_ID_SPEAKER) {
        report_id = data[0];
        data += 1;
        len  -= 1;
    } else if (report_id == PS4_REPORT_ID_SPEAKER && data[0] == PS4_REPORT_ID_SPEAKER) {
        data += 1;
        len  -= 1;
    }

    // PS4 output report (rumble/LED) - Report ID 5 (Player 1) or Report ID 6 (Player 2).
    // `data` is now the report body (no ID byte); copy it into the struct after its report_id field
    // so motor_right/motor_left, lightbar, and speaker volume land at the right offsets.
    if (report_id == PS4_REPORT_ID_OUTPUT && len >= 8) {
        ps4_outputs[0].report_id = PS4_REPORT_ID_OUTPUT;
        uint16_t copy_len = len;
        if (copy_len > sizeof(ps4_out_report_t) - 1) {
            copy_len = sizeof(ps4_out_report_t) - 1;
        }
        memcpy((uint8_t*)&ps4_outputs[0] + 1, data, copy_len);
        ps4_output_availables[0] = true;

        if (ps4_outputs[0].set_flags2 & 0x80) {
            ps4_outputs[0].set_flags2 &= ~0x80;
            printf("[ps4_mode] Console requested Player 1 sleep (preserving BT link)\n");
        }
    } else if (report_id == PS4_REPORT_ID_OUTPUT_P2 && len >= 8) {
        ps4_outputs[1].report_id = PS4_REPORT_ID_OUTPUT_P2;
        uint16_t copy_len = len;
        if (copy_len > sizeof(ps4_out_report_t) - 1) {
            copy_len = sizeof(ps4_out_report_t) - 1;
        }
        memcpy((uint8_t*)&ps4_outputs[1] + 1, data, copy_len);
        ps4_output_availables[1] = true;

        if (ps4_outputs[1].set_flags2 & 0x80) {
            ps4_outputs[1].set_flags2 &= ~0x80;
            printf("[ps4_mode] Console requested Player 2 sleep (preserving BT link)\n");
        }
    } else if (report_id == PS4_REPORT_ID_SPEAKER) {
        ds4_audio_diag_rx_count++;
        ds4_audio_feed_report(data, len);
    }
}

static uint8_t ps4_mode_get_rumble(void)
{
    // PS4 has motor_left (large) and motor_right (small) 8-bit values
    return (ps4_outputs[0].motor_left > ps4_outputs[0].motor_right)
           ? ps4_outputs[0].motor_left : ps4_outputs[0].motor_right;
}

static bool ps4_mode_get_feedback(output_feedback_t* fb)
{
    for (int i = 0; i < 2; i++) {
        uint8_t p = (ps4_feedback_poll_idx + i) % 2;
        if (ps4_output_availables[p]) {
            ps4_feedback_poll_idx = (p + 1) % 2;
            fb->target_player = p;
            fb->rumble_left = ps4_outputs[p].motor_left;
            fb->rumble_right = ps4_outputs[p].motor_right;
            fb->led_r = ps4_outputs[p].lightbar_red;
            fb->led_g = ps4_outputs[p].lightbar_green;
            fb->led_b = ps4_outputs[p].lightbar_blue;
            fb->has_led_rgb = (ps4_outputs[p].set_led != 0);
            fb->volume_speaker = ps4_outputs[p].volume_speaker;
            fb->has_speaker_volume = (ps4_outputs[p].set_speaker_volume != 0);
            fb->dirty = true;
            ps4_output_availables[p] = false;
            return true;
        }
    }
    return false;
}

static uint16_t ps4_mode_get_report(uint8_t report_id, hid_report_type_t report_type,
                                     uint8_t* buffer, uint16_t reqlen)
{
    if (report_type != HID_REPORT_TYPE_FEATURE) {
        return 0;
    }

    uint16_t len = 0;
    switch (report_id) {
        case PS4_REPORT_ID_FEATURE_03:
            // Controller definition report - return GP2040-CE compatible data
            len = sizeof(ps4_feature_03);
            if (reqlen < len) len = reqlen;
            memcpy(buffer, ps4_feature_03, len);
            return len;

        case PS4_REPORT_ID_CALIBRATION:     // 0x02 - Calibration data
            len = sizeof(ps4_feature_02);
            if (reqlen < len) len = reqlen;
            memcpy(buffer, ps4_feature_02, len);
            return len;

        case PS4_REPORT_ID_PAIRING_INFO:    // 0x12 - Pairing info
            len = sizeof(ps4_feature_12);
            if (reqlen < len) len = reqlen;
            memcpy(buffer, ps4_feature_12, len);
            return len;

        case PS4_REPORT_ID_VERSION_INFO:    // 0xA3 - Firmware / version info
            len = sizeof(ps4_feature_a3);
            if (reqlen < len) len = reqlen;
            memcpy(buffer, ps4_feature_a3, len);
            return len;

        case PS4_REPORT_ID_AUTH_RESPONSE:   // 0xF1 - Signature
            len = 64;
            if (reqlen < len) len = reqlen;
            memset(buffer, 0, len);
            return len;

        case PS4_REPORT_ID_AUTH_STATUS:     // 0xF2 - Signing status
            len = 16;
            if (reqlen < len) len = reqlen;
            // No auth available — return "still signing" indefinitely
            memset(buffer, 0, len);
            buffer[1] = 0x10;
            return len;

        case PS4_REPORT_ID_AUTH_PAYLOAD:    // 0xF0 - handled in set_report
            len = 64;
            if (reqlen < len) len = reqlen;
            memset(buffer, 0, len);
            return len;

        case PS4_REPORT_ID_AUTH_RESET:      // 0xF3 - Return page size info
#ifdef ENABLE_PS4_LOCAL_AUTH
            if (ps4_local_auth_is_available()) {
                ps4_local_auth_reset();
            }
#endif
#ifndef DISABLE_USB_HOST
            ds4_auth_reset();
#endif
            len = sizeof(ps4_feature_f3);
            if (reqlen < len) len = reqlen;
            memcpy(buffer, ps4_feature_f3, len);
            return len;

        default:
            return 0;
    }
}

// Handle PS4 auth SET_REPORT (nonce from console, etc.)
// This is called from usbd.c's tud_hid_set_report_cb for feature reports
void ps4_mode_set_feature_report(uint8_t report_id, const uint8_t* buffer, uint16_t bufsize)
{
    switch (report_id) {
        case PS4_REPORT_ID_AUTH_PAYLOAD:    // 0xF0 - Nonce from console
#ifdef ENABLE_PS4_LOCAL_AUTH
            if (ps4_local_auth_is_available()) {
                ps4_local_auth_send_nonce_page(buffer, bufsize);
                break;
            }
#endif
#ifndef DISABLE_USB_HOST
            if (ds4_auth_is_available()) {
                ds4_auth_send_nonce(buffer, bufsize);
            }
#endif
            break;

        case PS4_REPORT_ID_AUTH_RESET:      // 0xF3 - Reset auth
#ifdef ENABLE_PS4_LOCAL_AUTH
            if (ps4_local_auth_is_available()) {
                ps4_local_auth_reset();
            }
#endif
#ifndef DISABLE_USB_HOST
            ds4_auth_reset();
#endif
            break;

        default:
            (void)buffer;
            (void)bufsize;
            break;
    }
}

// Called from the main loop (via usbd_mode_t.task) when PS4 mode is active
static void ps4_mode_task(void)
{
#ifdef ENABLE_PS4_LOCAL_AUTH
    ps4_local_auth_task();
#endif
}

static const uint8_t* ps4_mode_get_device_descriptor(void)
{
    return (const uint8_t*)&ps4_device_descriptor;
}

static const uint8_t* ps4_mode_get_config_descriptor(void)
{
    return ps4_config_descriptor;
}

static const uint8_t* ps4_mode_get_report_descriptor(void)
{
    return ps4_report_descriptor;
}

// ============================================================================
// MODE EXPORT
// ============================================================================

const usbd_mode_t ps4_mode = {
    .name = "PS4",
    .mode = USB_OUTPUT_MODE_PS4,

    .get_device_descriptor = ps4_mode_get_device_descriptor,
    .get_config_descriptor = ps4_mode_get_config_descriptor,
    .get_report_descriptor = ps4_mode_get_report_descriptor,

    .init = ps4_mode_init,
    .send_report = ps4_mode_send_report,
    .is_ready = ps4_mode_is_ready,

    .handle_output = ps4_mode_handle_output,
    .get_rumble = ps4_mode_get_rumble,
    .get_feedback = ps4_mode_get_feedback,
    .get_report = ps4_mode_get_report,
    .get_class_driver = NULL,
    .task = ps4_mode_task,
};
