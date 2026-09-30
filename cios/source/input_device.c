// Modified for REVOLUTIONDS4, Copyright (c) 2026 Reyhe66243.
// Licensed under the GNU General Public License version 2; see ../LICENSE.
#include "input_device.h"
#include "button_map.h"
#include "egc.h"
#include "fake_wiimote.h"
#include "types.h"
#include "utils.h"
#include "wiimote.h"

#define MAX_INPUT_DEVS  2
#define RECONNECT_DELAY 200 /* 1s @ 200Hz */
#define COMBO_HOLD_THRESHOLD 100 /* 100 ticks @ 200Hz = 500ms */

static const struct {
    u16 wiimote_button_map[EGC_GAMEPAD_BUTTON_COUNT];
    u16 wiimote_wheel_button_map[EGC_GAMEPAD_BUTTON_COUNT];
    u16 classic_button_map[EGC_GAMEPAD_BUTTON_COUNT];
    u8 classic_analog_axis_map[EGC_GAMEPAD_AXIS_COUNT];
} input_mappings = {
	.wiimote_button_map = {
		[EGC_GAMEPAD_BUTTON_NORTH] = WIIMOTE_BUTTON_ONE,
		[EGC_GAMEPAD_BUTTON_EAST] = WIIMOTE_BUTTON_B,
		[EGC_GAMEPAD_BUTTON_SOUTH] = WIIMOTE_BUTTON_A,
		[EGC_GAMEPAD_BUTTON_WEST] = WIIMOTE_BUTTON_TWO,
		[EGC_GAMEPAD_BUTTON_DPAD_UP] = WIIMOTE_BUTTON_UP,
		[EGC_GAMEPAD_BUTTON_DPAD_DOWN] = WIIMOTE_BUTTON_DOWN,
		[EGC_GAMEPAD_BUTTON_DPAD_LEFT] = WIIMOTE_BUTTON_LEFT,
		[EGC_GAMEPAD_BUTTON_DPAD_RIGHT] = WIIMOTE_BUTTON_RIGHT,
		[EGC_GAMEPAD_BUTTON_START] = WIIMOTE_BUTTON_PLUS,
		[EGC_GAMEPAD_BUTTON_BACK] = WIIMOTE_BUTTON_MINUS,
		[EGC_GAMEPAD_BUTTON_GUIDE] = WIIMOTE_BUTTON_HOME,
		/* L1 unmapped in Wiimote modes so it acts cleanly as modifier for combos (L1+L3, L1+R3) */
		[EGC_GAMEPAD_BUTTON_LEFT_PADDLE1] = WIIMOTE_BUTTON_B,
		[EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER] = WIIMOTE_BUTTON_B,
		[EGC_GAMEPAD_BUTTON_RIGHT_PADDLE1] = WIIMOTE_BUTTON_B,
		[EGC_GAMEPAD_BUTTON_TOUCHPAD] = WIIMOTE_BUTTON_A,
	},
	.wiimote_wheel_button_map = {
		[EGC_GAMEPAD_BUTTON_NORTH] = WIIMOTE_BUTTON_A,        /* Triangle: 1 <-> A */
		[EGC_GAMEPAD_BUTTON_EAST] = WIIMOTE_BUTTON_TWO,      /* Circle:   B <-> 2 */
		[EGC_GAMEPAD_BUTTON_SOUTH] = WIIMOTE_BUTTON_ONE,     /* Cross:    A <-> 1 */
		[EGC_GAMEPAD_BUTTON_WEST] = WIIMOTE_BUTTON_B,        /* Square:   2 <-> B */
		[EGC_GAMEPAD_BUTTON_DPAD_UP] = WIIMOTE_BUTTON_RIGHT, /* D-pad rotated 90 deg CCW */
		[EGC_GAMEPAD_BUTTON_DPAD_DOWN] = WIIMOTE_BUTTON_LEFT,
		[EGC_GAMEPAD_BUTTON_DPAD_LEFT] = WIIMOTE_BUTTON_UP,
		[EGC_GAMEPAD_BUTTON_DPAD_RIGHT] = WIIMOTE_BUTTON_DOWN,
		[EGC_GAMEPAD_BUTTON_START] = WIIMOTE_BUTTON_PLUS,
		[EGC_GAMEPAD_BUTTON_BACK] = WIIMOTE_BUTTON_MINUS,
		[EGC_GAMEPAD_BUTTON_GUIDE] = WIIMOTE_BUTTON_HOME,
		/* L1 unmapped in Wheel mode so it acts cleanly as modifier for combos (L1+L3, L1+R3) */
		[EGC_GAMEPAD_BUTTON_LEFT_PADDLE1] = WIIMOTE_BUTTON_B,    /* L2 = B */
		[EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER] = WIIMOTE_BUTTON_B,  /* R1 = B */
		[EGC_GAMEPAD_BUTTON_RIGHT_PADDLE1] = WIIMOTE_BUTTON_B,   /* R2 = B */
		[EGC_GAMEPAD_BUTTON_TOUCHPAD] = WIIMOTE_BUTTON_A,        /* Touchpad = A */
	},
	.classic_button_map = {
		[EGC_GAMEPAD_BUTTON_NORTH] = CLASSIC_CTRL_BUTTON_X,
		[EGC_GAMEPAD_BUTTON_EAST] = CLASSIC_CTRL_BUTTON_A,
		[EGC_GAMEPAD_BUTTON_SOUTH] = CLASSIC_CTRL_BUTTON_B,
		[EGC_GAMEPAD_BUTTON_WEST] = CLASSIC_CTRL_BUTTON_Y,
		[EGC_GAMEPAD_BUTTON_DPAD_UP] = CLASSIC_CTRL_BUTTON_UP,
		[EGC_GAMEPAD_BUTTON_DPAD_DOWN] = CLASSIC_CTRL_BUTTON_DOWN,
		[EGC_GAMEPAD_BUTTON_DPAD_LEFT] = CLASSIC_CTRL_BUTTON_LEFT,
		[EGC_GAMEPAD_BUTTON_DPAD_RIGHT] = CLASSIC_CTRL_BUTTON_RIGHT,
		[EGC_GAMEPAD_BUTTON_START] = CLASSIC_CTRL_BUTTON_PLUS,
		[EGC_GAMEPAD_BUTTON_BACK] = CLASSIC_CTRL_BUTTON_MINUS,
		[EGC_GAMEPAD_BUTTON_RIGHT_PADDLE1] = CLASSIC_CTRL_BUTTON_ZR,
		[EGC_GAMEPAD_BUTTON_LEFT_PADDLE1] = CLASSIC_CTRL_BUTTON_ZL,
		[EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER] = CLASSIC_CTRL_BUTTON_FULL_R,
		[EGC_GAMEPAD_BUTTON_LEFT_SHOULDER] = CLASSIC_CTRL_BUTTON_FULL_L,
		[EGC_GAMEPAD_BUTTON_GUIDE] = CLASSIC_CTRL_BUTTON_HOME,
		[EGC_GAMEPAD_BUTTON_TOUCHPAD] = CLASSIC_CTRL_BUTTON_A,
	},
	.classic_analog_axis_map = {
		[EGC_GAMEPAD_AXIS_LEFTX] = BM_CLASSIC_ANALOG_AXIS_LEFT_X,
		[EGC_GAMEPAD_AXIS_LEFTY] = BM_CLASSIC_ANALOG_AXIS_LEFT_Y,
		[EGC_GAMEPAD_AXIS_RIGHTX] = BM_CLASSIC_ANALOG_AXIS_RIGHT_X,
		[EGC_GAMEPAD_AXIS_RIGHTY] = BM_CLASSIC_ANALOG_AXIS_RIGHT_Y,
	},
};

static const u8 ir_analog_axis_map[EGC_GAMEPAD_AXIS_COUNT] = {
    [EGC_GAMEPAD_AXIS_RIGHTX] = BM_IR_AXIS_X,
    [EGC_GAMEPAD_AXIS_RIGHTY] = BM_IR_AXIS_Y,
};

static const enum bm_ir_emulation_mode_e ir_emu_modes[] = {
    BM_IR_EMULATION_MODE_DIRECT,
    BM_IR_EMULATION_MODE_RELATIVE_ANALOG_AXIS,
    BM_IR_EMULATION_MODE_ABSOLUTE_ANALOG_AXIS,
};

static struct input_device_t {
    egc_input_device_t *device;
    /* NULL if no assigned fake Wiimote */
    fake_wiimote_t *assigned_wiimote;
    u32 reconnect_delay;
    u32 switch_mapping_combo;
    u32 switch_orientation_combo;
    enum bm_ir_emulation_mode_e ir_emu_mode;
    struct bm_ir_emulation_state_t ir_emu_state;
    u8 switch_mapping_hold_count;
    u8 switch_orientation_hold_count;
    u8 extension;
    u8 ir_emu_mode_idx;
    u8 start_hold_count;
    u8 guide_hold_count;
    u8 back_hold_count;
    u8 guide_cooldown;
    bool controller_connected;
} input_devices[MAX_INPUT_DEVS];

static inline bool has_button(egc_input_device_t *device, egc_gamepad_button_e button)
{
    return device->desc->available_buttons & BIT(button);
}

static input_device_t *input_device_from_egc(egc_input_device_t *device)
{
    for (int i = 0; i < ARRAY_SIZE(input_devices); i++)
        if (input_devices[i].device == device)
            return &input_devices[i];
    return NULL;
}

bool input_device_is_registered(egc_input_device_t *device)
{
    return input_device_from_egc(device) != NULL;
}

void input_devices_init(void)
{
    for (int i = 0; i < ARRAY_SIZE(input_devices); i++) {
        input_devices[i].device = NULL;
        input_devices[i].assigned_wiimote = NULL;
        input_devices[i].controller_connected = false;
    }
}

void input_devices_reset(void)
{
    for (int i = 0; i < ARRAY_SIZE(input_devices); i++) {
        input_devices[i].assigned_wiimote = NULL;
        input_devices[i].reconnect_delay = RECONNECT_DELAY;
    }
}

void input_device_handle_added(egc_input_device_t *device, void *userdata)
{
    /* Find a free input device slot */
    for (int i = 0; i < ARRAY_SIZE(input_devices); i++) {
        if (!input_devices[i].device) {
            input_devices[i].device = device;
            /* No assigned fake Wiimote yet */
            input_devices[i].assigned_wiimote = NULL;
            input_devices[i].extension = WIIMOTE_EXT_NONE;
            input_devices[i].switch_mapping_hold_count = 0;
            input_devices[i].switch_orientation_hold_count = 0;
            input_devices[i].device->state.gamepad.touch_points[0].x = -1;
            input_devices[i].device->state.gamepad.touch_points[1].x = -1;
            if (device->desc && device->desc->num_touch_points > 0) {
                input_devices[i].ir_emu_mode_idx = 0; /* Direct touchpad mode */
            } else {
                input_devices[i].ir_emu_mode_idx = 1; /* Relative stick mode */
            }
            bm_ir_emulation_state_reset(&input_devices[i].ir_emu_state);
            input_devices[i].start_hold_count = 0;
            input_devices[i].guide_hold_count = 0;
            input_devices[i].back_hold_count = 0;
            input_devices[i].guide_cooldown = 0;
            input_devices[i].controller_connected = false;

            if (has_button(device, EGC_GAMEPAD_BUTTON_LEFT_STICK) &&
                has_button(device, EGC_GAMEPAD_BUTTON_LEFT_SHOULDER)) {
                input_devices[i].switch_mapping_combo =
                    BIT(EGC_GAMEPAD_BUTTON_LEFT_STICK) | BIT(EGC_GAMEPAD_BUTTON_LEFT_SHOULDER);
            } else {
                input_devices[i].switch_mapping_combo = 0;
            }

            if (has_button(device, EGC_GAMEPAD_BUTTON_RIGHT_STICK) &&
                has_button(device, EGC_GAMEPAD_BUTTON_LEFT_SHOULDER)) {
                input_devices[i].switch_orientation_combo =
                    BIT(EGC_GAMEPAD_BUTTON_RIGHT_STICK) | BIT(EGC_GAMEPAD_BUTTON_LEFT_SHOULDER);
            } else {
                input_devices[i].switch_orientation_combo = 0;
            }
            break;
        }
    }
}

void input_device_handle_removed(egc_input_device_t *device, void *userdata)
{
    input_device_t *input_device = input_device_from_egc(device);
    if (!input_device)
        return;

    input_device->controller_connected = false;
    fake_wiimote_t *wiimote = input_device->assigned_wiimote;
    input_device->assigned_wiimote = NULL;
    input_device->reconnect_delay = RECONNECT_DELAY;
    if (wiimote) {
        fake_wiimote_release_input_device(wiimote);
        fake_wiimote_disconnect(wiimote);
    }
    input_device->device = NULL;
}

void input_devices_tick(void)
{
    for (int i = 0; i < ARRAY_SIZE(input_devices); i++) {
        if (input_devices[i].device && !input_devices[i].assigned_wiimote) {
            if (input_devices[i].reconnect_delay > 0)
                input_devices[i].reconnect_delay--;
        }
    }
}

input_device_t *input_device_get_unassigned(void)
{
    for (int i = 0; i < ARRAY_SIZE(input_devices); i++) {
        if (input_devices[i].device && input_devices[i].controller_connected &&
            !input_devices[i].assigned_wiimote && input_devices[i].reconnect_delay == 0) {
            return &input_devices[i];
        }
    }

    return NULL;
}

void input_device_assign_wiimote(input_device_t *input_device, fake_wiimote_t *wiimote)
{
    input_device->assigned_wiimote = wiimote;
    fake_wiimote_set_extension(wiimote, input_device->extension);
}

void input_device_release_wiimote(input_device_t *input_device)
{
    input_device->assigned_wiimote = NULL;
    input_device->reconnect_delay = RECONNECT_DELAY;
}

void input_device_set_connected(egc_input_device_t *device, bool connected)
{
    input_device_t *input_device = input_device_from_egc(device);
    if (!input_device)
        return;

    input_device->controller_connected = connected;

    if (connected) {
        input_device->reconnect_delay = 0;
    }
}

bool input_device_is_connected(egc_input_device_t *device)
{
    input_device_t *input_device = input_device_from_egc(device);
    return input_device && input_device->controller_connected;
}

bool input_device_is_controller_connected(const input_device_t *input_device)
{
    return input_device && input_device->device && input_device->controller_connected;
}

int input_device_resume(input_device_t *input_device)
{
    return egc_input_device_resume(input_device->device);
}

int input_device_suspend(input_device_t *input_device)
{
    return egc_input_device_suspend(input_device->device);
}

int input_device_set_leds(input_device_t *input_device, int leds)
{
    return egc_input_device_set_leds(input_device->device, leds);
}

int input_device_set_rumble(input_device_t *input_device, bool rumble_on)
{
    return egc_input_device_set_rumble(input_device->device,
                                       rumble_on ? EGC_RUMBLE_MAX : EGC_RUMBLE_OFF);
}

bool input_device_report_input(input_device_t *input_device)
{
    const egc_input_state_t *input = &input_device->device->state;
    fake_wiimote_t *wiimote = input_device->assigned_wiimote;
    u16 wiimote_buttons = 0;
    union wiimote_extension_data_t extension_data;
    struct ir_dot_t ir_dots[IR_MAX_DOTS];
    bm_ir_dots_set_out_of_screen(ir_dots);
    enum bm_ir_emulation_mode_e ir_emu_mode;

    /*
     * Combos:
     * - L1 + L3 (switch_mapping_combo): Toggle Classic Controller mode
     * - L1 + R3 (switch_orientation_combo): Toggle Horizontal / Wheel mode
     *
     * Rules:
     * - L1 + L3 can ALWAYS be toggled:
     *    - Entering Classic mode:
     *      * Force is_wheel_mode = false and orientation_mode = VERTICAL.
     *      * Clear LED indicator 4.
     *      * fake_wiimote_set_extension requests WIIMOTE_EXT_CLASSIC (clearing Motion Plus).
     *    - Exiting Classic mode:
     *      * fake_wiimote_set_extension requests WIIMOTE_EXT_NONE.
     *      * Restores standard Wiimote with Motion Plus.
     * - L1 + R3 can ONLY be toggled when NOT in Classic mode (extension == WIIMOTE_EXT_NONE):
     *    - Toggles between WIIMOTE_ORIENTATION_VERTICAL and WIIMOTE_ORIENTATION_WHEEL.
     *    - Updates LED indicator 4 immediately.
     */
    /*
     * Require holding combos for COMBO_HOLD_THRESHOLD ticks (500 ms @ 200 Hz)
     * to eliminate accidental triggering during rapid arm motions.
     */
    if (input_device->switch_mapping_combo &&
        (input->gamepad.buttons & input_device->switch_mapping_combo) == input_device->switch_mapping_combo) {
        if (input_device->switch_mapping_hold_count < 255)
            input_device->switch_mapping_hold_count++;
    } else {
        input_device->switch_mapping_hold_count = 0;
    }

    if (input_device->switch_mapping_hold_count == COMBO_HOLD_THRESHOLD) {
        if (input_device->extension == WIIMOTE_EXT_NONE) {
            input_device->extension = WIIMOTE_EXT_CLASSIC;
            if (wiimote) {
                wiimote->is_wheel_mode = false;
                wiimote->orientation_mode = WIIMOTE_ORIENTATION_VERTICAL;
                u32 led_val = wiimote->status.leds ? wiimote->status.leds : BIT(wiimote->index);
                input_device_set_leds(input_device, led_val);
            }
        } else {
            input_device->extension = WIIMOTE_EXT_NONE;
        }
        if (wiimote) {
            fake_wiimote_set_extension(wiimote, input_device->extension);
        }
        return false;
    }

    if (input_device->extension == WIIMOTE_EXT_NONE && input_device->switch_orientation_combo) {
        if ((input->gamepad.buttons & input_device->switch_orientation_combo) == input_device->switch_orientation_combo) {
            if (input_device->switch_orientation_hold_count < 255)
                input_device->switch_orientation_hold_count++;
        } else {
            input_device->switch_orientation_hold_count = 0;
        }

        if (input_device->switch_orientation_hold_count == COMBO_HOLD_THRESHOLD) {
            if (wiimote) {
                u8 next_mode = (wiimote->orientation_mode == WIIMOTE_ORIENTATION_WHEEL)
                                    ? WIIMOTE_ORIENTATION_VERTICAL
                                    : WIIMOTE_ORIENTATION_WHEEL;
                fake_wiimote_set_orientation_mode(wiimote, next_mode);
            }
            return false;
        }
    } else {
        input_device->switch_orientation_hold_count = 0;
    }

    u32 gamepad_buttons = input->gamepad.buttons;

    /* Debounce Start/Plus button (EGC_GAMEPAD_BUTTON_START): require 6 continuous ticks (~30ms) */
    if (gamepad_buttons & BIT(EGC_GAMEPAD_BUTTON_START)) {
        if (input_device->start_hold_count < 20)
            input_device->start_hold_count++;
    } else {
        input_device->start_hold_count = 0;
    }

    if (input_device->start_hold_count < 6) {
        gamepad_buttons &= ~BIT(EGC_GAMEPAD_BUTTON_START);
    }

    /* Debounce Share/Minus button (EGC_GAMEPAD_BUTTON_BACK): require 6 continuous ticks (~30ms) */
    if (gamepad_buttons & BIT(EGC_GAMEPAD_BUTTON_BACK)) {
        if (input_device->back_hold_count < 20)
            input_device->back_hold_count++;
    } else {
        input_device->back_hold_count = 0;
    }

    if (input_device->back_hold_count < 6) {
        gamepad_buttons &= ~BIT(EGC_GAMEPAD_BUTTON_BACK);
    }

    /* Debounce PS button (EGC_GAMEPAD_BUTTON_GUIDE): require 6 continuous ticks (~30ms) */
    if (gamepad_buttons & BIT(EGC_GAMEPAD_BUTTON_GUIDE)) {
        if (input_device->guide_hold_count < 20)
            input_device->guide_hold_count++;
    } else {
        input_device->guide_hold_count = 0;
    }

    if (input_device->guide_hold_count < 6) {
        gamepad_buttons &= ~BIT(EGC_GAMEPAD_BUTTON_GUIDE);
    }


    if (input_device->extension == WIIMOTE_EXT_CLASSIC) {
        /*
         * In Classic Controller mode:
         * 1) Force is_wheel_mode to false (never rotate D-pad or swap face buttons).
         * 2) Do NOT map Wiimote face buttons / D-pad to avoid dual-input conflicts.
         *    Only forward HOME (PS button) and Touchpad click (Wiimote A for pointer clicks).
         */
        if (wiimote) {
            wiimote->is_wheel_mode = false;
        }
        if (gamepad_buttons & BIT(EGC_GAMEPAD_BUTTON_GUIDE)) {
            wiimote_buttons |= WIIMOTE_BUTTON_HOME;
        }
        if (gamepad_buttons & BIT(EGC_GAMEPAD_BUTTON_TOUCHPAD)) {
            wiimote_buttons |= WIIMOTE_BUTTON_A;
        }
    } else {
        const u16 *map = (wiimote && wiimote->is_wheel_mode)
                             ? input_mappings.wiimote_wheel_button_map
                             : input_mappings.wiimote_button_map;
        bm_map_wiimote(EGC_GAMEPAD_BUTTON_COUNT, gamepad_buttons, map, &wiimote_buttons);
    }

    s16 ax = 0, ay = 0, az = 0;
    if (input_device->device->desc->num_accelerometers > 0) {
        ax = input->gamepad.accelerometer[0].x;
        ay = input->gamepad.accelerometer[0].y;
        az = input->gamepad.accelerometer[0].z;
    }

    /*
     * Stream genuine physical accelerometer (ax, ay, az) continuously.
     * Never override with artificial (0, 4096, 0), which caused orientation jumps
     * and broke tilt controls in both Motion Plus and standard accelerometer games.
     */
    fake_wiimote_report_accelerometer(wiimote, ax, ay, az);

    if (input_device->device->desc->num_gyroscopes > 0) {
        s16 gyro_x = input->gamepad.gyroscope[0].pitch;
        s16 gyro_y = input->gamepad.gyroscope[0].roll;
        s16 gyro_z = input->gamepad.gyroscope[0].yaw;
        fake_wiimote_report_gyroscope(wiimote, gyro_x, gyro_y, gyro_z);
    }

    ir_emu_mode = ir_emu_modes[input_device->ir_emu_mode_idx];
    if (ir_emu_mode == BM_IR_EMULATION_MODE_NONE) {
        bm_ir_dots_set_out_of_screen(ir_dots);
    } else {
        if (ir_emu_mode == BM_IR_EMULATION_MODE_DIRECT) {
            bm_map_ir_direct(input->gamepad.touch_points[0].x,
                             input->gamepad.touch_points[0].y,
                             ax, ay,
                             ir_dots);
        } else {
            bm_map_ir_analog_axis(ir_emu_mode, &input_device->ir_emu_state, EGC_GAMEPAD_AXIS_COUNT,
                                  input->gamepad.axes, ir_analog_axis_map, ir_dots);
        }
    }

    fake_wiimote_report_ir_dots(wiimote, ir_dots);
    fake_wiimote_report_battery(wiimote, input->gamepad.battery_level,
                                input->gamepad.battery_charging);

    if (input_device->extension == WIIMOTE_EXT_CLASSIC) {
        bm_map_classic(EGC_GAMEPAD_BUTTON_COUNT, gamepad_buttons, EGC_GAMEPAD_AXIS_COUNT,
                       input->gamepad.axes, input_mappings.classic_button_map,
                       input_mappings.classic_analog_axis_map, &extension_data.classic);
        fake_wiimote_report_input_ext(wiimote, wiimote_buttons, &extension_data,
                                      sizeof(extension_data.classic));
    } else {
        fake_wiimote_report_input(wiimote, wiimote_buttons);
    }
    return true;
}

int input_device_send_speaker_data(input_device_t *input_device, const void *data, u16 len)
{
    if (input_device && input_device->device) {
        return egc_input_device_send_speaker_data(input_device->device, (const u8 *)data, len);
    }
    return 0;
}

bool input_device_has_gyroscope(const input_device_t *input_device)
{
    if (input_device && input_device->device && input_device->device->desc) {
        return input_device->device->desc->num_gyroscopes > 0;
    }
    return false;
}
