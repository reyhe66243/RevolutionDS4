#include "button_map.h"
#include "egc.h"
#include "internals.h"

static bool s_sensor_bar_position_top = false;

#define STICK_DEADZONE 3200 /* ~9.8% deadzone to completely absorb analog stick drift */

static inline u8 s16_to_u8(s16 value)
{
    if (value > -STICK_DEADZONE && value < STICK_DEADZONE) {
        value = 0;
    } else if (value >= STICK_DEADZONE) {
        /* Smooth rescaling so input starts smoothly from 0 and reaches full 32767 */
        value = (s16)(((s32)(value - STICK_DEADZONE) * 32767) / (32767 - STICK_DEADZONE));
    } else {
        value = (s16)(((s32)(value + STICK_DEADZONE) * 32767) / (32767 - STICK_DEADZONE));
    }
    return 0x80 + (value >> 8);
}

void bm_map_wiimote(
    /* Inputs */
    int num_buttons, u32 buttons,
    /* Mapping tables */
    const u16 *wiimote_button_map,
    /* Outputs */
    u16 *wiimote_buttons)
{
    for (int i = 0; i < num_buttons; i++) {
        if (buttons & 1)
            *wiimote_buttons |= wiimote_button_map[i];
        buttons >>= 1;
    }
}


void bm_map_classic(
    /* Inputs */
    int num_buttons, u32 buttons, int num_analog_axis, const s16 *analog_axis,
    /* Mapping tables */
    const u16 *classic_button_map, const u8 *classic_analog_axis_map,
    /* Outputs */
    struct wiimote_extension_data_format_classic_t *classic)
{
    u16 classic_buttons = 0;
    u8 classic_analog_axis[BM_CLASSIC_ANALOG_AXIS__NUM] = { 0 };

    for (int i = 0; i < num_buttons; i++) {
        if (buttons & 1)
            classic_buttons |= classic_button_map[i];
        buttons >>= 1;
    }

    for (int i = 0; i < num_analog_axis; i++) {
        if (classic_analog_axis_map[i])
            classic_analog_axis[classic_analog_axis_map[i] - 1] = s16_to_u8(analog_axis[i]);
    }

    bm_classic_format(classic, classic_buttons, classic_analog_axis);
}

static inline u32 isqrt(u32 val)
{
    u32 res = 0;
    u32 bit = 1U << 30;
    while (bit > val)
        bit >>= 2;
    while (bit != 0) {
        if (val >= res + bit) {
            val -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

static inline void map_ir_dot(struct ir_dot_t ir_dots[static IR_MAX_DOTS],
                              const struct ir_dot_t *dot, s16 acc_x, s16 acc_y)
{
    s16 vert_offset = s_sensor_bar_position_top ? IR_VERTICAL_OFFSET : -IR_VERTICAL_OFFSET;

    s32 center_x = IR_DOT_CENTER_MIN_X + (IR_DOT_CENTER_MAX_X - dot->x);
    s32 center_y = dot->y + vert_offset;

    /*
     * Roll rotation compensation:
     * When the player tilts/rolls the controller, a physical Wiimote's camera
     * physically rotates with the body, so the two sensor bar dots appear tilted
     * in the camera frame. The Wii's WPAD tracking library requires that the angle
     * between the two IR dots matches the accelerometer gravity roll angle.
     *
     * We calculate the roll unit vector (dx, dy) directly from gravity (acc_x, acc_y):
     * - ZERO drift because the accelerometer directly measures gravity (1.0G).
     * - Flat face-up (rest): acc_x = 0, acc_y = 4096 -> dx = 64, dy = 0 (horizontal).
     * - Roll right: acc_x > 0 -> dy < 0 (counter-clockwise on sensor).
     * - Roll left:  acc_x < 0 -> dy > 0 (clockwise on sensor).
     */
    s32 rx = (s32)acc_x / 32;
    s32 rz = (s32)acc_y / 32;
    s32 len_sq = rx * rx + rz * rz;
    s32 len = isqrt((u32)len_sq);

    s32 dx = IR_HORIZONTAL_OFFSET;
    s32 dy = 0;

    if (len >= 10) {
        dx = (s32)(IR_HORIZONTAL_OFFSET * rz) / len;
        dy = -(s32)(IR_HORIZONTAL_OFFSET * rx) / len;
    }

    s32 x0 = center_x - dx;
    s32 y0 = center_y - dy;
    s32 x1 = center_x + dx;
    s32 y1 = center_y + dy;

    if (x0 < 0) x0 = 0;
    if (x0 > 1022) x0 = 1022;
    if (x1 < 0) x1 = 0;
    if (x1 > 1022) x1 = 1022;
    if (y0 < 0) y0 = 0;
    if (y0 > 1022) y0 = 1022;
    if (y1 < 0) y1 = 0;
    if (y1 > 1022) y1 = 1022;

    ir_dots[0].x = x0;
    ir_dots[0].y = y0;
    ir_dots[1].x = x1;
    ir_dots[1].y = y1;
    for (int i = 2; i < IR_MAX_DOTS; i++) {
        ir_dots[i].x = 1023;
        ir_dots[i].y = 1023;
    }
}

void bm_map_ir_direct(
    /* Inputs */
    s16 x, s16 y, s16 acc_x, s16 acc_y,
    /* Outputs */
    struct ir_dot_t ir_dots[static IR_MAX_DOTS])
{
    struct ir_dot_t dot;

    if (x < 0) {
        bm_ir_dots_set_out_of_screen(ir_dots);
        return;
    }

    if (x > EGC_GAMEPAD_TOUCH_RES)
        x = EGC_GAMEPAD_TOUCH_RES;
    if (y < 0)
        y = 0;
    if (y > EGC_GAMEPAD_TOUCH_RES)
        y = EGC_GAMEPAD_TOUCH_RES;

    dot.x = IR_DOT_CENTER_MIN_X +
            ((int)x * (IR_DOT_CENTER_MAX_X - IR_DOT_CENTER_MIN_X)) / EGC_GAMEPAD_TOUCH_RES;
    dot.y = IR_DOT_CENTER_MIN_Y +
            ((int)y * (IR_DOT_CENTER_MAX_Y - IR_DOT_CENTER_MIN_Y)) / EGC_GAMEPAD_TOUCH_RES;
    map_ir_dot(ir_dots, &dot, acc_x, acc_y);
}

void bm_map_ir_analog_axis(
    /* Inputs */
    enum bm_ir_emulation_mode_e mode, struct bm_ir_emulation_state_t *state, int num_analog_axis,
    const s16 *analog_axis, const u8 *ir_analog_axis_map,
    /* Outputs */
    struct ir_dot_t ir_dots[static IR_MAX_DOTS])
{
    struct ir_dot_t dot;

    for (int i = 0; i < num_analog_axis; i++) {
        if (ir_analog_axis_map[i]) {
            s16 val = analog_axis[i];

            if (mode == BM_IR_EMULATION_MODE_RELATIVE_ANALOG_AXIS) {
                state->position[ir_analog_axis_map[i] - 1] += val / 16;
            } else if (mode == BM_IR_EMULATION_MODE_ABSOLUTE_ANALOG_AXIS) {
                u16 center = (ir_analog_axis_map[i] == BM_IR_AXIS_X) ? IR_CENTER_X : IR_CENTER_Y;
                state->position[ir_analog_axis_map[i] - 1] = center + val;
            }
        }
    }

    if (state->position[BM_IR_AXIS_X - 1] < IR_DOT_CENTER_MIN_X)
        state->position[BM_IR_AXIS_X - 1] = IR_DOT_CENTER_MIN_X;
    else if (state->position[BM_IR_AXIS_X - 1] > IR_DOT_CENTER_MAX_X)
        state->position[BM_IR_AXIS_X - 1] = IR_DOT_CENTER_MAX_X;

    if (state->position[BM_IR_AXIS_Y - 1] < IR_DOT_CENTER_MIN_Y)
        state->position[BM_IR_AXIS_Y - 1] = IR_DOT_CENTER_MIN_Y;
    else if (state->position[BM_IR_AXIS_Y - 1] > IR_DOT_CENTER_MAX_Y)
        state->position[BM_IR_AXIS_Y - 1] = IR_DOT_CENTER_MAX_Y;

    dot.x = state->position[BM_IR_AXIS_X - 1];
    dot.y = IR_DOT_CENTER_MIN_Y + (IR_DOT_CENTER_MAX_Y - state->position[BM_IR_AXIS_Y - 1]);
    map_ir_dot(ir_dots, &dot, 0, 4096);
}

void bm_set_sensor_bar_position_top(bool on_top)
{
    s_sensor_bar_position_top = on_top;
}
