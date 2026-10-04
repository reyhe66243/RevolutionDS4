// Modified for REVOLUTIONDS4, Copyright (c) 2026 Reyhe66243.
// Licensed under the GNU General Public License version 2; see ../LICENSE.
#include <limits.h>

#include "driver_api.h"
#include "utils.h"


#define SONY_VID 0x054c

#define DS4_TOUCHPAD_W    1920
#define DS4_TOUCHPAD_H    940
#define DS4_ACC_RES_PER_G 8192

struct ds4_input_report {
    u8 report_id;
    u8 left_x;
    u8 left_y;
    u8 right_x;
    u8 right_y;

    u8 triangle : 1;
    u8 circle : 1;
    u8 cross : 1;
    u8 square : 1;
    u8 dpad : 4;

    u8 r3 : 1;
    u8 l3 : 1;
    u8 options : 1;
    u8 share : 1;
    u8 r2 : 1;
    u8 l2 : 1;
    u8 r1 : 1;
    u8 l1 : 1;

    u8 cnt1 : 6;
    u8 tpad : 1;
    u8 ps : 1;

    u8 l_trigger;
    u8 r_trigger;

    u8 cnt2;
    u8 cnt3;

    u8 battery;

    union {
        s16 pitch;
        s16 gyro_x;
    };

    union {
        s16 roll;
        s16 gyro_y;
    };

    union {
        s16 yaw;
        s16 gyro_z;
    };

    s16 accel_x;
    s16 accel_y;
    s16 accel_z;

    u8 unk1[5];

    u8 padding : 1;
    u8 microphone : 1;
    u8 headphones : 1;
    u8 usb_plugged : 1;
    u8 battery_level : 4;

    u8 unk2[2];
    u8 trackpadpackets;
    u8 packetcnt;

    u8 finger1[4];
    u8 finger2[4];
} ATTRIBUTE_PACKED;

enum ds4_buttons_e {
    DS4_BUTTON_TRIANGLE,
    DS4_BUTTON_CIRCLE,
    DS4_BUTTON_CROSS,
    DS4_BUTTON_SQUARE,
    DS4_BUTTON_UP,
    DS4_BUTTON_DOWN,
    DS4_BUTTON_LEFT,
    DS4_BUTTON_RIGHT,
    DS4_BUTTON_R3,
    DS4_BUTTON_L3,
    DS4_BUTTON_OPTIONS,
    DS4_BUTTON_SHARE,
    DS4_BUTTON_R2,
    DS4_BUTTON_L2,
    DS4_BUTTON_R1,
    DS4_BUTTON_L1,
    DS4_BUTTON_TOUCHPAD,
    DS4_BUTTON_PS,
    DS4_BUTTON_COUNT
};

enum ds4_analog_axis_e {
    DS4_ANALOG_AXIS_LEFT_X,
    DS4_ANALOG_AXIS_LEFT_Y,
    DS4_ANALOG_AXIS_RIGHT_X,
    DS4_ANALOG_AXIS_RIGHT_Y,
    DS4_ANALOG_AXIS_COUNT
};

struct ds4_private_data_t {
    u8 led_color[3]; /* 0 - 255 */
    bool rumble_on;
    bool input_pending;
    bool led_rumble_pending; /* a LED/rumble request is in flight */
    bool led_rumble_dirty;   /* state changed while one was in flight */
    u8 player_index;
    u8 input_stuck_ticks;
    u8 led_rumble_stuck_ticks;
    u8 assigned_color_idx;
};
static_assert(sizeof(struct ds4_private_data_t) <= EGC_INPUT_DEVICE_PRIVATE_DATA_SIZE);

extern void ogc_ds4_attach_player2(egc_input_device_t *p1_dev);
extern void ogc_ds4_detach_player2(egc_input_device_t *p2_dev);
extern void input_device_set_connected(egc_input_device_t *device, bool connected);
extern bool input_device_is_connected(egc_input_device_t *device);

static egc_input_device_t *s_ds4_p1_device = NULL;
static egc_input_device_t *s_ds4_p2_device = NULL;
static bool s_ds4_p2_attaching = false;
static uint32_t s_ds4_p1_last_seen_ticks = 0;
static uint32_t s_ds4_p2_last_seen_ticks = 0;
static uint32_t s_ds4_ticks = 0;

/* Map each button of the controller to an egc_gamepad_button_e */
static const egc_gamepad_button_e s_button_map[DS4_BUTTON_COUNT] = {
    [DS4_BUTTON_UP] = EGC_GAMEPAD_BUTTON_DPAD_UP,
    [DS4_BUTTON_DOWN] = EGC_GAMEPAD_BUTTON_DPAD_DOWN,
    [DS4_BUTTON_LEFT] = EGC_GAMEPAD_BUTTON_DPAD_LEFT,
    [DS4_BUTTON_RIGHT] = EGC_GAMEPAD_BUTTON_DPAD_RIGHT,
    [DS4_BUTTON_TRIANGLE] = EGC_GAMEPAD_BUTTON_NORTH,
    [DS4_BUTTON_CIRCLE] = EGC_GAMEPAD_BUTTON_EAST,
    [DS4_BUTTON_CROSS] = EGC_GAMEPAD_BUTTON_SOUTH,
    [DS4_BUTTON_SQUARE] = EGC_GAMEPAD_BUTTON_WEST,
    [DS4_BUTTON_L1] = EGC_GAMEPAD_BUTTON_LEFT_SHOULDER,
    [DS4_BUTTON_R1] = EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER,
    [DS4_BUTTON_L2] = EGC_GAMEPAD_BUTTON_LEFT_PADDLE1,
    [DS4_BUTTON_R2] = EGC_GAMEPAD_BUTTON_RIGHT_PADDLE1,
    [DS4_BUTTON_SHARE] = EGC_GAMEPAD_BUTTON_BACK,
    [DS4_BUTTON_OPTIONS] = EGC_GAMEPAD_BUTTON_START,
    [DS4_BUTTON_PS] = EGC_GAMEPAD_BUTTON_GUIDE,
    [DS4_BUTTON_L3] = EGC_GAMEPAD_BUTTON_LEFT_STICK,
    [DS4_BUTTON_R3] = EGC_GAMEPAD_BUTTON_RIGHT_STICK,
    [DS4_BUTTON_TOUCHPAD] = EGC_GAMEPAD_BUTTON_TOUCHPAD,
};

static const egc_device_description_t s_device_description = {
    .vendor_id = SONY_VID,
    // Product ID is set dynamically
    /* clang-format off */
    .available_buttons =
        BIT(EGC_GAMEPAD_BUTTON_DPAD_UP) |
        BIT(EGC_GAMEPAD_BUTTON_DPAD_DOWN) |
        BIT(EGC_GAMEPAD_BUTTON_DPAD_LEFT) |
        BIT(EGC_GAMEPAD_BUTTON_DPAD_RIGHT) |
        BIT(EGC_GAMEPAD_BUTTON_NORTH) |
        BIT(EGC_GAMEPAD_BUTTON_EAST) |
        BIT(EGC_GAMEPAD_BUTTON_SOUTH) |
        BIT(EGC_GAMEPAD_BUTTON_WEST) |
        BIT(EGC_GAMEPAD_BUTTON_LEFT_SHOULDER) |
        BIT(EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER) |
        BIT(EGC_GAMEPAD_BUTTON_LEFT_PADDLE1) |
        BIT(EGC_GAMEPAD_BUTTON_RIGHT_PADDLE1) |
        BIT(EGC_GAMEPAD_BUTTON_BACK) |
        BIT(EGC_GAMEPAD_BUTTON_START) |
        BIT(EGC_GAMEPAD_BUTTON_GUIDE) |
        BIT(EGC_GAMEPAD_BUTTON_LEFT_STICK) |
        BIT(EGC_GAMEPAD_BUTTON_RIGHT_STICK),
    .available_axes =
        BIT(EGC_GAMEPAD_AXIS_LEFTX) |
        BIT(EGC_GAMEPAD_AXIS_LEFTY) |
        BIT(EGC_GAMEPAD_AXIS_RIGHTX) |
        BIT(EGC_GAMEPAD_AXIS_RIGHTY),
    /* clang-format on */
    .type = EGC_DEVICE_TYPE_GAMEPAD,
    .num_touch_points = 2,
    .num_leds = 4,
    .num_accelerometers = 1,
    .num_gyroscopes = 1,
    .has_rumble = true,
};

static inline int ds4_request_data(egc_input_device_t *device);

static inline u32 ds4_get_buttons(const struct ds4_input_report *report)
{
    u32 mask = 0;

#define MAP(field, button)                                                                         \
    if (report->field)                                                                             \
        mask |= BIT(button);

    MAP(triangle, DS4_BUTTON_TRIANGLE)
    MAP(circle, DS4_BUTTON_CIRCLE)
    MAP(cross, DS4_BUTTON_CROSS)
    MAP(square, DS4_BUTTON_SQUARE)

    if (report->dpad <= 8) {
        if (report->dpad == 0 || report->dpad == 1 || report->dpad == 7)
            mask |= BIT(DS4_BUTTON_UP);
        else if (report->dpad == 3 || report->dpad == 4 || report->dpad == 5)
            mask |= BIT(DS4_BUTTON_DOWN);
        if (report->dpad == 5 || report->dpad == 6 || report->dpad == 7)
            mask |= BIT(DS4_BUTTON_LEFT);
        else if (report->dpad == 1 || report->dpad == 2 || report->dpad == 3)
            mask |= BIT(DS4_BUTTON_RIGHT);
    }

    MAP(r3, DS4_BUTTON_R3)
    MAP(l3, DS4_BUTTON_L3)
    MAP(options, DS4_BUTTON_OPTIONS)
    MAP(share, DS4_BUTTON_SHARE)
    MAP(r2, DS4_BUTTON_R2)
    MAP(l2, DS4_BUTTON_L2)
    MAP(r1, DS4_BUTTON_R1)
    MAP(l1, DS4_BUTTON_L1)
    MAP(tpad, DS4_BUTTON_TOUCHPAD)
    MAP(ps, DS4_BUTTON_PS)
#undef MAP

    return mask;
}

static inline void ds4_get_analog_axis(const struct ds4_input_report *report,
                                       u8 analog_axis[static DS4_ANALOG_AXIS_COUNT])
{
    analog_axis[DS4_ANALOG_AXIS_LEFT_X] = report->left_x;
    analog_axis[DS4_ANALOG_AXIS_LEFT_Y] = 255 - report->left_y;
    analog_axis[DS4_ANALOG_AXIS_RIGHT_X] = report->right_x;
    analog_axis[DS4_ANALOG_AXIS_RIGHT_Y] = 255 - report->right_y;
}


/* Aggregated speaker audio (defined below). Pending audio is flushed right
 * after an input report completes, so the audio OUT transfers never collide
 * with the controller input polling transfers. */
static void ds4_audio_flush(void);
static void ds4_audio_flush_if_full(void);

static void ds4_request_data_cb(egc_usb_transfer_t *transfer)
{
    egc_input_device_t *device = transfer->device;
    struct ds4_private_data_t *priv = (void *)device->private_data;
    struct ds4_input_report *report = (void *)transfer->data;
    struct egc_input_state_t state;
    priv->input_pending = false;
    priv->input_stuck_ticks = 0;

    if (transfer->status == EGC_USB_TRANSFER_STATUS_COMPLETED &&
        transfer->length >= 10 &&
        (report->report_id == 0x01 || report->report_id == 0x02)) {

        if (report->report_id == 0x01) {
            s_ds4_p1_last_seen_ticks = s_ds4_ticks;
        } else if (report->report_id == 0x02) {
            s_ds4_p2_last_seen_ticks = s_ds4_ticks;
            if (!s_ds4_p2_device && !s_ds4_p2_attaching && s_ds4_p1_device) {
                s_ds4_p2_attaching = true;
                ogc_ds4_attach_player2(s_ds4_p1_device);
            }
            if (!s_ds4_p2_device) {
                /* P2 device is still attaching, continue polling */
                ds4_request_data(s_ds4_p1_device ? s_ds4_p1_device : device);
                return;
            }
        }

        egc_input_device_t *target_dev = (report->report_id == 0x02)
                                             ? s_ds4_p2_device
                                             : (s_ds4_p1_device ? s_ds4_p1_device : device);

        const u8 *raw = (const u8 *)transfer->data;
        if (raw[30] == 0xFF) {
            /* Controller is disconnected / sleeping. */
            input_device_set_connected(target_dev, false);
            memset(&state, 0, sizeof(state));
            state.gamepad.accelerometer[0].z = EGC_ACCELEROMETER_RES_PER_G;
            state.gamepad.touch_points[0].x = -1;
            state.gamepad.touch_points[1].x = -1;
            state.gamepad.battery_level = 10;
            egc_device_driver_report_input(target_dev, &state);
            ds4_request_data(s_ds4_p1_device ? s_ds4_p1_device : device);
            return;
        }
        input_device_set_connected(target_dev, true);

        u32 buttons = ds4_get_buttons(report);
        state.gamepad.buttons =
            egc_device_driver_map_buttons(buttons, DS4_BUTTON_COUNT, s_button_map);

        u8 axes[DS4_ANALOG_AXIS_COUNT];
        ds4_get_analog_axis(report, axes);
        state.gamepad.axes[EGC_GAMEPAD_AXIS_LEFTX] = egc_u8_to_s16(axes[DS4_ANALOG_AXIS_LEFT_X]);
        state.gamepad.axes[EGC_GAMEPAD_AXIS_LEFTY] = egc_u8_to_s16(axes[DS4_ANALOG_AXIS_LEFT_Y]);
        state.gamepad.axes[EGC_GAMEPAD_AXIS_RIGHTX] = egc_u8_to_s16(axes[DS4_ANALOG_AXIS_RIGHT_X]);
        state.gamepad.axes[EGC_GAMEPAD_AXIS_RIGHTY] = egc_u8_to_s16(axes[DS4_ANALOG_AXIS_RIGHT_Y]);

#define MAP_ACCEL(v) ((s16)le16toh(v) * EGC_ACCELEROMETER_RES_PER_G / DS4_ACC_RES_PER_G)
        state.gamepad.accelerometer[0].x = MAP_ACCEL(report->accel_x);
        state.gamepad.accelerometer[0].y = MAP_ACCEL(report->accel_y);
        state.gamepad.accelerometer[0].z = MAP_ACCEL(report->accel_z);
#undef MAP_ACCEL

        state.gamepad.gyroscope[0].pitch = (s16)le16toh(report->pitch);
        state.gamepad.gyroscope[0].yaw   = (s16)le16toh(report->yaw);
        state.gamepad.gyroscope[0].roll  = (s16)le16toh(report->roll);

#define MAP_TOUCH_X(v) ((v) * EGC_GAMEPAD_TOUCH_RES / DS4_TOUCHPAD_W)
#define MAP_TOUCH_Y(v) ((v) * EGC_GAMEPAD_TOUCH_RES / DS4_TOUCHPAD_H)
        const u8 *f1 = report->finger1;
        if ((f1[0] & 0x80) == 0) {
            u16 raw_x = (u16)f1[1] | ((u16)(f1[2] & 0x0F) << 8);
            u16 raw_y = ((u16)(f1[2] >> 4) & 0x0F) | ((u16)f1[3] << 4);
            state.gamepad.touch_points[0].x = MAP_TOUCH_X(raw_x);
            state.gamepad.touch_points[0].y = MAP_TOUCH_Y(raw_y);
        } else {
            state.gamepad.touch_points[0].x = -1;
        }

        const u8 *f2 = report->finger2;
        if ((f2[0] & 0x80) == 0) {
            u16 raw_x = (u16)f2[1] | ((u16)(f2[2] & 0x0F) << 8);
            u16 raw_y = ((u16)(f2[2] >> 4) & 0x0F) | ((u16)f2[3] << 4);
            state.gamepad.touch_points[1].x = MAP_TOUCH_X(raw_x);
            state.gamepad.touch_points[1].y = MAP_TOUCH_Y(raw_y);
        } else {
            state.gamepad.touch_points[1].x = -1;
        }
#undef MAP_TOUCH_X
#undef MAP_TOUCH_Y

        /* Extract real DS4 battery level and cable/charging state from report offset 30 */
        const u8 *raw_report = (const u8 *)transfer->data;
        u8 bat_byte = raw_report[30];
        state.gamepad.battery_level = bat_byte & 0x0F;
        state.gamepad.battery_charging = (bat_byte & 0x10) != 0;

        egc_device_driver_report_input(target_dev, &state);
    }

    ds4_request_data(s_ds4_p1_device ? s_ds4_p1_device : device);

    /* Input transfer is armed again: now it is safe to push pending audio,
     * so the audio OUT never has to wait behind an input IN transfer. */
    ds4_audio_flush_if_full();
}

static inline int ds4_request_data(egc_input_device_t *device)
{
    struct ds4_private_data_t *priv = (void *)device->private_data;
    if (priv->input_pending)
        return 0;
    priv->input_pending = true;
    const egc_usb_transfer_t *transfer = egc_device_driver_issue_intr_transfer_async(
        device, EGC_USB_ENDPOINT_IN, NULL, 0, ds4_request_data_cb);
    if (!transfer) {
        priv->input_pending = false;
        return -1;
    }
    return 0;
}

/* A lost completion used to keep a transfer slot reserved forever; once the
 * pool was exhausted, input polling failed permanently (all inputs were lost
 * while the game kept running). The backend now reclaims stale transfers by
 * age; this callback re-enables LED/rumble submissions when one completes. */
static int ds4_driver_update_leds_rumble(egc_input_device_t *device);

static void ds4_led_rumble_cb(egc_usb_transfer_t *transfer)
{
    if (!transfer || !transfer->device)
        return;
    struct ds4_private_data_t *priv = (void *)transfer->device->private_data;
    if (!priv)
        return;

    priv->led_rumble_pending = false;
    priv->led_rumble_stuck_ticks = 0;
    if (priv->led_rumble_dirty) {
        priv->led_rumble_dirty = false;
        ds4_driver_update_leds_rumble(transfer->device);
    }
}

static inline int ds4_set_leds_rumble(egc_input_device_t *device, u8 r, u8 g, u8 b, u8 rumble_small,
                                      u8 rumble_large)
{
    struct ds4_private_data_t *priv = (void *)device->private_data;
    u8 report_id = (device == s_ds4_p2_device || (priv && priv->player_index == 1)) ? 0x06 : 0x05;

    u8 buf[32] = {
        report_id, // Report ID 5 (P1) or 6 (P2)
        0x03, // valid_flag0: rumble (0x01) | leds (0x02)
        0x00,
        0x00,
        rumble_small, // Fast motor
        rumble_large, // Slow motor
        r,
        g,
        b,    // RGB
        0x00, // LED on duration
        0x00  // LED off duration
    };

    const egc_usb_transfer_t *transfer = egc_device_driver_issue_ctrl_transfer_async(
        device, EGC_USB_REQTYPE_INTERFACE_SET, EGC_USB_REQ_SETREPORT,
        (EGC_USB_REPTYPE_OUTPUT << 8) | report_id, 0, buf, sizeof(buf), ds4_led_rumble_cb);
    return transfer != NULL ? 0 : -1;
}

static int ds4_driver_update_leds_rumble(egc_input_device_t *device)
{
    struct ds4_private_data_t *priv = (void *)device->private_data;
    if (!priv) return -1;

    if (priv->led_rumble_pending) {
        /* At most one LED/rumble request in flight: coalescing bursts keeps transfers
         * available and prevents USB EP0 protocol stalls. */
        priv->led_rumble_dirty = true;
        return 0;
    }

    u8 r = priv->led_color[0], g = priv->led_color[1], b = priv->led_color[2];
    u8 rumble = priv->rumble_on ? 255 : 0;

    priv->led_rumble_pending = true;
    priv->led_rumble_stuck_ticks = 0;
    int rc = ds4_set_leds_rumble(device, r, g, b, rumble, rumble);
    if (rc < 0) {
        priv->led_rumble_pending = false;
        priv->led_rumble_dirty = true;
    }
    return rc;
}

#ifdef __arm__
#include "backends/ogc_arm.h"
#endif

static inline u32 ds4_enter_critical(void)
{
#ifdef __arm__
    u32 cpsr = get_cpsr();
    set_cpsr_c(cpsr | PSR_I);
    return cpsr;
#else
    return 0;
#endif
}

static inline void ds4_leave_critical(u32 level)
{
#ifdef __arm__
    set_cpsr_c(level);
#else
    (void)level;
#endif
}

/* ---------------------------------------------------------------------------
 * Aggregated speaker audio (per controller)
 *
 * Every game speaker write used to become its own USB interrupt transfer
 * (~150-300/s, and far more if a game writes small chunks). Those transfers
 * compete with the controller input polling transfers and were measured to
 * disturb the console's motion input stream whenever audio played.
 *
 * Each controller's audio accumulates in its own FIFO and is drained as up to
 * 64-byte USB transfers, so the transfer rate stays bounded while every sample
 * is forwarded exactly once (no drops, no duplicated packets). One FIFO per
 * controller means two controllers streaming audio at the same time cannot
 * interleave their sample streams or fight over a single header.
 *
 * Pacing:
 *   - the driver timer (10ms) flushes a partial FIFO so latency stays bounded
 *   - the completion callback drains faster when a backlog builds up
 * ------------------------------------------------------------------------- */
#define DS4_AUDIO_HEADER_LEN  5     /* [0x18, fmt, rate, vol, count] */
#define DS4_AUDIO_MAX_SAMPLES 59    /* 5 + 59 = 64 = endpoint max packet */
#define DS4_AUDIO_ACC_MAX     256   /* FIFO depth per controller (~40ms @6kHz) */
#define DS4_AUDIO_STREAMS     2     /* player 1 + player 2 */

typedef struct {
    egc_input_device_t *device;
    bool active;                    /* header valid */
    u8 fmt;                         /* (player << 4) | format */
    u8 rate;
    u8 vol;
    u16 count;
    u8 acc[DS4_AUDIO_ACC_MAX];
} ds4_audio_stream_t;

static ds4_audio_stream_t s_audio_streams[DS4_AUDIO_STREAMS];
/*
 * Player 1 has priority over player 2: the adapter cannot stream two audio
 * channels at the same time, so the single USB transfer always carries
 * player 1's audio while it has any, and player 2 fills in otherwise.
 */
static u8 s_audio_tx[DS4_AUDIO_HEADER_LEN + DS4_AUDIO_MAX_SAMPLES] ATTRIBUTE_ALIGN(32);
static volatile bool s_ds4_audio_out_pending = false;
static volatile u8 s_ds4_audio_idle_ticks = 0;

static void ds4_audio_flush(void);

/* Caller holds the critical section */
static bool ds4_audio_has_backlog_locked(void)
{
    for (int i = 0; i < DS4_AUDIO_STREAMS; i++) {
        if (s_audio_streams[i].count > DS4_AUDIO_MAX_SAMPLES)
            return true;
    }
    return false;
}

/* Caller holds the critical section */
static bool ds4_audio_has_data_locked(void)
{
    for (int i = 0; i < DS4_AUDIO_STREAMS; i++) {
        if (s_audio_streams[i].count > 0)
            return true;
    }
    return false;
}

static void ds4_audio_out_cb(egc_usb_transfer_t *transfer)
{
    (void)transfer;
    u32 cs = ds4_enter_critical();
    s_ds4_audio_out_pending = false;
    bool backlog = ds4_audio_has_backlog_locked();
    ds4_leave_critical(cs);

    /* Keep up with sustained audio without waiting for the next timer tick. */
    if (backlog)
        ds4_audio_flush();
}

static void ds4_audio_flush(void)
{
    u16 n = 0;
    u8 sent_idx = 0xFF;
    egc_input_device_t *dev = NULL;

    u32 cs = ds4_enter_critical();
    if (!s_ds4_audio_out_pending) {
        for (int k = 0; k < DS4_AUDIO_STREAMS; k++) {
            u8 idx = (u8)k;   /* player 1 first: it has priority */
            ds4_audio_stream_t *st = &s_audio_streams[idx];
            if (st->count > 0 && st->device) {
                n = (st->count > DS4_AUDIO_MAX_SAMPLES) ? DS4_AUDIO_MAX_SAMPLES
                                                        : st->count;
                s_audio_tx[0] = 0x18;
                s_audio_tx[1] = st->fmt;
                s_audio_tx[2] = st->rate;
                s_audio_tx[3] = st->vol;
                s_audio_tx[4] = (u8)n;
                memcpy(&s_audio_tx[DS4_AUDIO_HEADER_LEN], st->acc, n);
                dev = st->device;
                sent_idx = idx;
                s_ds4_audio_out_pending = true;
                s_ds4_audio_idle_ticks = 0;
                break;
            }
        }
    }
    ds4_leave_critical(cs);

    if (n == 0 || !dev)
        return;

    const egc_usb_transfer_t *t = egc_device_driver_issue_intr_transfer_async(
        dev, 1, s_audio_tx, (u16)(DS4_AUDIO_HEADER_LEN + n), ds4_audio_out_cb);
    if (!t) {
        cs = ds4_enter_critical();
        s_ds4_audio_out_pending = false;
        ds4_leave_critical(cs);
        return;  /* samples stay in the FIFO and are retried */
    }

    /* Transfer accepted: drop the sent prefix from its FIFO (forward copy,
     * destination is below source, so this is overlap-safe). */
    cs = ds4_enter_critical();
    {
        ds4_audio_stream_t *st = &s_audio_streams[sent_idx];
        if (st->count >= n) {
            u16 remaining = (u16)(st->count - n);
            for (u16 i = 0; i < remaining; i++) {
                st->acc[i] = st->acc[n + i];
            }
            st->count = remaining;
        } else {
            st->count = 0;
        }
    }
    ds4_leave_critical(cs);
}

/* Flush only when a player's FIFO holds a full USB packet. Called right after
 * an input report is processed, so audio transfers are interleaved at moments
 * where the next input IN transfer is ~4ms away. */
static void ds4_audio_flush_if_full(void)
{
    u32 cs = ds4_enter_critical();
    bool ready = false;
    for (int i = 0; i < DS4_AUDIO_STREAMS; i++) {
        if (s_audio_streams[i].count >= DS4_AUDIO_MAX_SAMPLES) {
            ready = true;
            break;
        }
    }
    ds4_leave_critical(cs);

    if (ready)
        ds4_audio_flush();
}

int ds4_driver_ops_send_speaker_data(egc_input_device_t *device, const u8 *data, u16 len)
{
    if (!device || !data || len < DS4_AUDIO_HEADER_LEN)
        return -1;

    u8 samples = data[4];
    if (samples > len - DS4_AUDIO_HEADER_LEN)
        samples = len - DS4_AUDIO_HEADER_LEN;
    if (samples == 0)
        return 0;

    /* data[1] carries the player index in its high nibble; keep one FIFO per
     * player so two controllers never mix their audio streams. */
    u8 player = (u8)((data[1] >> 4) & 0x03);
    if (player >= DS4_AUDIO_STREAMS)
        player = (u8)(DS4_AUDIO_STREAMS - 1);

    ds4_audio_stream_t *st = &s_audio_streams[player];

    u32 cs = ds4_enter_critical();
    st->device = device;

    bool header_changed = st->active &&
        (st->fmt != data[1] || st->rate != data[2] || st->vol != data[3]);

    /* Format/rate/volume change: the FIFO content still belongs to the old
     * header, so try to flush it before switching headers. */
    bool flush_first = header_changed && st->count > 0 && !s_ds4_audio_out_pending;
    ds4_leave_critical(cs);

    if (flush_first)
        ds4_audio_flush();

    cs = ds4_enter_critical();
    if (header_changed || !st->active) {
        if (st->count > 0 && s_ds4_audio_out_pending) {
            /* Old samples stuck behind an in-flight transfer: drop them
             * (header changes mid-stream are extremely rare). */
            st->count = 0;
        }
        st->fmt = data[1];
        st->rate = data[2];
        st->vol = data[3];
        st->active = true;
    }

    if (st->count >= DS4_AUDIO_ACC_MAX) {
        ds4_leave_critical(cs);
        return 0;
    }
    u16 space = (u16)(DS4_AUDIO_ACC_MAX - st->count);
    u16 copy = (samples < space) ? samples : space;
    if (copy > 0) {
        memcpy(&st->acc[st->count], &data[DS4_AUDIO_HEADER_LEN], copy);
        st->count += copy;
    }

    /* Safety only: the regular drain happens from the input completion
     * callback (input-aligned). Flush here just if the FIFO is nearly full. */
    bool flush_now = (st->count >= 200);
    ds4_leave_critical(cs);

    if (flush_now)
        ds4_audio_flush();

    return 0;
}

bool ds4_driver_ops_probe(u16 vid, u16 pid)
{
    return vid == SONY_VID &&
           (pid == 0x05c4 /* DS4 rev 1 */ || pid == 0x09cc /* DS4 rev 2 */ ||
            pid == 0x0ba0 /* DS4 USB adapter */);
}

int ds4_driver_ops_init(egc_input_device_t *device, u16 vid, u16 pid)
{
    struct ds4_private_data_t *priv = (void *)device->private_data;
    egc_device_description_t *desc = egc_device_driver_alloc_desc(device);

    if (desc) {
        memcpy(desc, &s_device_description, sizeof(*desc));
        desc->product_id = pid;
    } else {
        device->desc = &s_device_description;
    }

    /* Init private state */
    memset(priv, 0, sizeof(*priv));

    if (!s_ds4_p1_device || s_ds4_p1_device == device) {
        s_ds4_p1_device = device;
        s_ds4_p1_last_seen_ticks = s_ds4_ticks;
        priv->player_index = 0;
        priv->assigned_color_idx = 0;
        priv->led_color[0] = 0;
        priv->led_color[1] = 0;
        priv->led_color[2] = 255; /* Default: Player 1 Blue */
    } else {
        s_ds4_p2_device = device;
        s_ds4_p2_attaching = false;
        priv->player_index = 1;
        priv->assigned_color_idx = 1;
        priv->led_color[0] = 255; /* Default: Player 2 Red */
        priv->led_color[1] = 0;
        priv->led_color[2] = 0;
    }
    priv->input_pending = false;

    device->state.gamepad.touch_points[0].x = -1;
    device->state.gamepad.touch_points[1].x = -1;

    /* Send initial LED state */
    ds4_driver_update_leds_rumble(device);

    if (priv->player_index == 0) {
        /* Start a 10ms periodic watchdog timer to ensure input polling never stalls */
        egc_device_driver_set_timer(device, 1000 * 10, 1000 * 10);
        return ds4_request_data(device);
    }

    return 0;
}

int ds4_driver_ops_disconnect(egc_input_device_t *device)
{
    struct ds4_private_data_t *priv = (void *)device->private_data;
    if (priv) {
        priv->input_pending = false;
        priv->input_stuck_ticks = 0;
        priv->led_rumble_pending = false;
        priv->led_rumble_stuck_ticks = 0;
    }
    if (device == s_ds4_p1_device) {
        s_ds4_p1_device = NULL;
    }
    if (device == s_ds4_p2_device) {
        s_ds4_p2_device = NULL;
        s_ds4_p2_attaching = false;
    }
    egc_device_driver_set_timer(device, 0, 0);
    for (int i = 0; i < DS4_AUDIO_STREAMS; i++) {
        if (s_audio_streams[i].device == device) {
            s_audio_streams[i].device = NULL;
            s_audio_streams[i].active = false;
            s_audio_streams[i].count = 0;
        }
    }
    s_ds4_audio_out_pending = false;
    return 0;
}

static const u8 s_led_colors[4][3] = {
    {0, 0, 1}, /* Player 1: Blue */
    {1, 0, 0}, /* Player 2: Red */
    {0, 1, 0}, /* Player 3: Green */
    {1, 1, 0}, /* Player 4: Yellow (Nintendo Wii standard) */
};

int ds4_driver_ops_set_leds(egc_input_device_t *device, u32 leds)
{
    struct ds4_private_data_t *priv = (void *)device->private_data;
    if (!priv) return -1;

    u8 intensity = (leds & BIT(4)) ? 35 : 255;
    /* Default to the assigned player color so an active DS4 never turns dark
     * when a game or loader temporarily clears leds during transitions */
    u8 c_idx = (priv->assigned_color_idx < 4) ? priv->assigned_color_idx : 0;
    priv->led_color[0] = s_led_colors[c_idx][0] * intensity;
    priv->led_color[1] = s_led_colors[c_idx][1] * intensity;
    priv->led_color[2] = s_led_colors[c_idx][2] * intensity;

    for (int i = 0; i < 4; i++) {
        if (leds & BIT(i)) {
            priv->assigned_color_idx = i;
            priv->led_color[0] = s_led_colors[i][0] * intensity;
            priv->led_color[1] = s_led_colors[i][1] * intensity;
            priv->led_color[2] = s_led_colors[i][2] * intensity;
            return ds4_driver_update_leds_rumble(device);
        }
    }

    return ds4_driver_update_leds_rumble(device);
}

int ds4_driver_ops_set_rumble(egc_input_device_t *device, bool rumble_on)
{
    struct ds4_private_data_t *priv = (void *)device->private_data;

    priv->rumble_on = rumble_on;
    return ds4_driver_update_leds_rumble(device);
}

/*
 * Periodic 10ms timer:
 * 1. Guarantees 200 Hz input polling never hangs if an IN transfer was ever dropped.
 * 2. Drains pending audio packets if callback completed while queue was busy.
 * 3. A lost completion can never stall the pipeline: the backend reclaims the
 *    transfer after 500 ms and this timer retries it.
 */
static bool ds4_driver_ops_timer(egc_input_device_t *device)
{
    struct ds4_private_data_t *priv = (void *)device->private_data;
    if (!priv) return false;

    if (priv->player_index == 0) {
        s_ds4_ticks++;

        /* Watchdog: If Player 2 is attached, but no Report 0x02 has been received for ~5s
         * (500 ticks of 10ms timer), Player 2 has disconnected or powered off */
        if (s_ds4_p2_device && (s_ds4_ticks - s_ds4_p2_last_seen_ticks > 500)) {
            egc_input_device_t *p2 = s_ds4_p2_device;
            s_ds4_p2_device = NULL;
            s_ds4_p2_attaching = false;
            ogc_ds4_detach_player2(p2);
        } else if (s_ds4_p2_attaching && (s_ds4_ticks - s_ds4_p2_last_seen_ticks > 500)) {
            s_ds4_p2_attaching = false;
        }

        /* Failsafe watchdog: if IN transfer has been in-flight for > 2.0s (200 ticks @ 10ms),
         * Starlet OHCI hardware or endpoint is wedged. Cancel the endpoint cleanly via IOS API.
         * Starlet safely completes the IOCTL, invoking ds4_request_data_cb to restart polling! */
        if (priv->input_pending) {
            if (++priv->input_stuck_ticks > 200) {
                priv->input_stuck_ticks = 0;
                egc_device_driver_cancel_endpoint(device, EGC_USB_ENDPOINT_IN);
            }
        } else {
            priv->input_stuck_ticks = 0;
            ds4_request_data(device);
        }

        /* Failsafe watchdog for control transfers (LEDs / rumble) */
        if (priv->led_rumble_pending) {
            if (++priv->led_rumble_stuck_ticks > 200) {
                priv->led_rumble_stuck_ticks = 0;
                egc_device_driver_cancel_endpoint(device, 0);
            }
        } else if (priv->led_rumble_dirty) {
            priv->led_rumble_dirty = false;
            ds4_driver_update_leds_rumble(device);
        }
    }

    /* Aggregated audio: flush partial batches at least once per timer tick
     * (10ms) so latency stays bounded */
    u32 cs = ds4_enter_critical();
    bool flush_now = false;

    if (!s_ds4_audio_out_pending && ds4_audio_has_data_locked()) {
        /* Input-aligned flushes (from ds4_request_data_cb) handle the normal
         * case. This guard only drains a leftover tail when no input report
         * has flushed audio for ~30ms. */
        if (s_ds4_audio_idle_ticks < 0xFF)
            s_ds4_audio_idle_ticks++;
        if (s_ds4_audio_idle_ticks >= 3) {
            s_ds4_audio_idle_ticks = 0;
            flush_now = true;
        }
    }
    ds4_leave_critical(cs);

    if (flush_now)
        ds4_audio_flush();

    return true;
}

const egc_device_driver_t ds4_usb_device_driver = {
    .probe = ds4_driver_ops_probe,
    .init = ds4_driver_ops_init,
    .disconnect = ds4_driver_ops_disconnect,
    .set_leds = ds4_driver_ops_set_leds,
    .set_rumble = ds4_driver_ops_set_rumble,
    .timer = ds4_driver_ops_timer,
    .send_speaker_data = ds4_driver_ops_send_speaker_data,
};
