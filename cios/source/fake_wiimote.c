// Modified for REVOLUTIONDS4, Copyright (c) 2026 Reyhe66243.
// Licensed under the GNU General Public License version 2; see ../LICENSE.
#include "button_map.h"
#include "fake_wiimote.h"
#include "hci.h"
#include "hci_state.h"
#include "injmessage.h"
#include "l2cap.h"
#include "syscalls.h"
#include "utils.h"
#include "wiimote.h"

/* Channel bookkeeping */

/* A fake Wiimote that asked for a connection and never got an Accept or a
 * Reject back (dropped event, or a console that is not accepting connections at
 * that moment) would otherwise stay unconnected forever. */
#define CON_REQ_RETRY_TICKS 400 /* 2s at 200Hz */

static inline u16 generate_l2cap_channel_id(void)
{
    /* "Identifiers from 0x0001 to 0x003F are reserved" */
    static u16 starting_id = 0x40;
    return starting_id++;
}

static inline bool l2cap_channel_is_accepted(const l2cap_channel_info_t *info)
{
    return info->valid && (info->remote_cid != L2CAP_NULL_CID);
}

static inline bool l2cap_channel_is_is_remote_configured(const l2cap_channel_info_t *info)
{
    return info->valid && (info->remote_mtu != 0);
}

static inline bool l2cap_channel_is_complete(const l2cap_channel_info_t *info)
{
    return info->valid && l2cap_channel_is_accepted(info) &&
           l2cap_channel_is_is_remote_configured(info) &&
           (info->state == L2CAP_CHANNEL_STATE_COMPLETE);
}

static l2cap_channel_info_t *get_channel_info(fake_wiimote_t *dev, u16 local_cid)
{
    if (dev->psm_sdp_chn.valid && (local_cid == dev->psm_sdp_chn.local_cid)) {
        return &dev->psm_sdp_chn;
    } else if (dev->psm_hid_cntl_chn.valid && (local_cid == dev->psm_hid_cntl_chn.local_cid)) {
        return &dev->psm_hid_cntl_chn;
    } else if (dev->psm_hid_intr_chn.valid && (local_cid == dev->psm_hid_intr_chn.local_cid)) {
        return &dev->psm_hid_intr_chn;
    }
    return NULL;
}

static void l2cap_channel_info_setup(l2cap_channel_info_t *info, u16 psm, u16 local_cid)
{
    info->psm = psm;
    info->state = L2CAP_CHANNEL_STATE_INACTIVE;
    info->local_cid = local_cid;
    info->remote_cid = L2CAP_NULL_CID;
    info->remote_mtu = 0;
    info->valid = true;
}

/* HID reports */

static int send_hid_data(u16 hci_con_handle, u16 dcid, u8 hid_type, const void *data, u32 size)
{
    u8 buf[WIIMOTE_MAX_PAYLOAD];
    if (size > (WIIMOTE_MAX_PAYLOAD - 1))
        size = WIIMOTE_MAX_PAYLOAD - 1;
    buf[0] = hid_type;
    memcpy(&buf[1], data, size);
    return inject_l2cap_packet(hci_con_handle, dcid, buf, size + 1);
}

static inline int send_hid_input_report(u16 hci_con_handle, u16 dcid, u8 report_id,
                                        const void *data, u32 size)
{
    u8 buf[WIIMOTE_MAX_PAYLOAD - 1];
    if (size > (WIIMOTE_MAX_PAYLOAD - 2))
        size = WIIMOTE_MAX_PAYLOAD - 2;
    buf[0] = report_id;
    memcpy(&buf[1], data, size);
    return send_hid_data(hci_con_handle, dcid, (HID_TYPE_DATA << 4) | HID_PARAM_INPUT, buf,
                         size + 1);
}

static int wiimote_send_ack(const fake_wiimote_t *wiimote, u8 rpt_id, u8 error_code)
{
    struct wiimote_input_report_ack_t ack;
    ack.buttons = wiimote->buttons;
    ack.rpt_id = rpt_id;
    ack.error_code = error_code;
    return send_hid_input_report(wiimote->hci_con_handle, wiimote->psm_hid_intr_chn.remote_cid,
                                 INPUT_REPORT_ID_ACK, &ack, sizeof(ack));
}

static inline bool fake_wiimote_has_motion_plus(const fake_wiimote_t *wiimote)
{
    if (wiimote && (wiimote->cur_extension == WIIMOTE_EXT_CLASSIC ||
                    wiimote->new_extension == WIIMOTE_EXT_CLASSIC)) {
        return false;
    }
    return true;
}

static int wiimote_send_input_report_status(const fake_wiimote_t *wiimote)
{
    struct wiimote_input_report_status_t status;
    memset(&status, 0, sizeof(status));
    status.buttons = wiimote->buttons;

    u8 flags = 0;
    if (wiimote->battery_low)
        flags |= 0x01;
    if ((wiimote->cur_extension != WIIMOTE_EXT_NONE) || wiimote->mp_active)
        flags |= 0x02;
    if (wiimote->speaker_enabled || wiimote->status.speaker)
        flags |= 0x04;
    if (wiimote->status.ir)
        flags |= 0x08;
    flags |= (wiimote->status.leds & 0x0F) << 4;
    status.flags = flags;

    status.battery = wiimote->battery ? wiimote->battery : 0xFF;
    return send_hid_input_report(wiimote->hci_con_handle, wiimote->psm_hid_intr_chn.remote_cid,
                                 INPUT_REPORT_ID_STATUS, &status, sizeof(status));
}

/* Disconnection helper functions */

static inline int disconnect_l2cap_channel(u16 hci_con_handle, l2cap_channel_info_t *info)
{
    int ret;
    ret = inject_l2cap_disconnect_req(hci_con_handle, info->remote_cid, info->local_cid);
    info->valid = false;
    return ret;
}

/* Init state */

static inline u8 calculate_calibration_data_checksum(const u8 *data, u8 size)
{
    u8 sum = 0x55;

    for (u8 i = 0; i < size; i++)
        sum += data[i];

    return sum;
}

static void eeprom_init(union wiimote_usable_eeprom_data_t *eeprom)
{
    u8 ir_checksum, accel_checksum;
    memset(eeprom->data, 0, sizeof(eeprom->data));

    static const u8 ir_calibration[10] = {
        /* Point 1 */
        IR_LOW_X & 0xFF,
        IR_LOW_Y & 0xFF,
        /* Mix */
        ((IR_LOW_Y & 0x300) >> 2) | ((IR_LOW_X & 0x300) >> 4) | ((IR_LOW_Y & 0x300) >> 6) |
            ((IR_HIGH_X & 0x300) >> 8),
        /* Point 2 */
        IR_HIGH_X & 0xFF,
        IR_LOW_Y & 0xFF,
        /* Point 3 */
        IR_HIGH_X & 0xFF,
        IR_HIGH_Y & 0xFF,
        /* Mix */
        ((IR_HIGH_Y & 0x300) >> 2) | ((IR_HIGH_X & 0x300) >> 4) | ((IR_HIGH_Y & 0x300) >> 6) |
            ((IR_LOW_X & 0x300) >> 8),
        /* Point 4 */
        IR_LOW_X & 0xFF,
        IR_HIGH_Y & 0xFF,
    };

    ir_checksum = calculate_calibration_data_checksum(ir_calibration, sizeof(ir_calibration));
    /* Copy to IR calibration data 1 */
    memcpy(eeprom->ir_calibration_1, ir_calibration, sizeof(ir_calibration));
    eeprom->ir_calibration_1[sizeof(eeprom->ir_calibration_1) - 1] = ir_checksum;
    /* Copy to IR calibration data 2 */
    memcpy(eeprom->ir_calibration_2, ir_calibration, sizeof(ir_calibration));
    eeprom->ir_calibration_2[sizeof(eeprom->ir_calibration_2) - 1] = ir_checksum;

    static const u8 accel_calibration[9] = {
        ACCEL_ZERO_G >> 2,
        ACCEL_ZERO_G >> 2,
        ACCEL_ZERO_G >> 2,
        ((ACCEL_ZERO_G & 3) << 4) | ((ACCEL_ZERO_G & 3) << 2) | (ACCEL_ZERO_G & 3),
        ACCEL_ONE_G >> 2,
        ACCEL_ONE_G >> 2,
        ACCEL_ONE_G >> 2,
        ((ACCEL_ONE_G & 3) << 4) | ((ACCEL_ONE_G & 3) << 2) | (ACCEL_ONE_G & 3),
        0, /* Motor + volume */
    };

    accel_checksum =
        calculate_calibration_data_checksum(accel_calibration, sizeof(accel_calibration));
    /* Copy to accelerometer calibration data 1 */
    memcpy(eeprom->accel_calibration_1, accel_calibration, sizeof(accel_calibration));
    eeprom->accel_calibration_1[sizeof(eeprom->accel_calibration_1) - 1] = accel_checksum;
    /* Copy to accelerometer calibration data 2 */
    memcpy(eeprom->accel_calibration_2, accel_calibration, sizeof(accel_calibration));
    eeprom->accel_calibration_2[sizeof(eeprom->accel_calibration_2) - 1] = accel_checksum;
}

void fake_wiimote_init(fake_wiimote_t *wiimote, u8 index, const bdaddr_t *bdaddr)
{
    wiimote->active = false;
    wiimote->index = index;
    /* We can set it now, since it's permanent */
    bacpy(&wiimote->bdaddr, bdaddr);
}

/* WMP calibration data (32 bytes at 0xA60020 / 0xA40020) */
static const u8 wmp_calibration_data[32] = {
    0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x3c, 0x00, 0xc4, 0x00, 0x3c, 0x00, 0xc8, 0x0b, 0x3f, 0x4f,
    0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x3c, 0x00, 0xc4, 0x00, 0x3c, 0x00, 0x2d, 0xe9, 0xc9, 0xc1
};

static inline void fake_wiimote_reset_extension_state(fake_wiimote_t *wiimote)
{
    union wiimote_extension_data_t ext;
    u8 *ext_controller_data = wiimote->extension_regs.controller_data;
    const u8 *id_code = NULL;

    memset(&wiimote->extension_regs, 0, sizeof(wiimote->extension_regs));
    memset(wiimote->extension_regs.controller_data, 0xFF, sizeof(wiimote->extension_regs.controller_data));
    memset(&wiimote->extension_key, 0, sizeof(wiimote->extension_key));
    wiimote->extension_key_dirty = true;

    wiimote->mp_active = false;
    wiimote->mp_mode = 0;

    switch (wiimote->cur_extension) {
    case WIIMOTE_EXT_CLASSIC:
        id_code = EXP_ID_CODE_CLASSIC_CONTROLLER;
        break;
    case WIIMOTE_EXT_CLASSIC_WIIU_PRO:
        id_code = EXP_ID_CODE_CLASSIC_WIIU_PRO;
        break;
    case WIIMOTE_EXT_GUITAR:
        id_code = EXP_ID_CODE_GUITAR;
        break;
    default:
        break;
    }

    if (id_code) {
        memcpy(wiimote->extension_regs.identifier, id_code,
               sizeof(wiimote->extension_regs.identifier));
    }

    /* Reset extension controller state to defaults */
    if (wiimote->cur_extension == WIIMOTE_EXT_CLASSIC) {
        u8 analog_axis[BM_CLASSIC_ANALOG_AXIS__NUM];

        for (int i = 0; i < ARRAY_SIZE(analog_axis); i++)
            analog_axis[i] = 0x80;

        bm_classic_format(&ext.classic, 0, analog_axis);
        memcpy(ext_controller_data, &ext.classic, sizeof(ext.classic));
    }
}

void fake_wiimote_init_state(fake_wiimote_t *wiimote, input_device_t *input_device)
{
    wiimote->baseband_state = BASEBAND_STATE_REQUEST_CONNECTION;
    wiimote->con_req_retry = 0;
    wiimote->discon_ticks = 0;
    wiimote->acl_state = L2CAP_CHANNEL_STATE_INACTIVE;
    wiimote->psm_sdp_chn.valid = false;
    wiimote->psm_hid_cntl_chn.valid = false;
    wiimote->psm_hid_intr_chn.valid = false;
    wiimote->hci_con_handle = 0;
    wiimote->num_completed_acl_data_packets = 0;
    wiimote->input_device = input_device;
    wiimote->status.leds = BIT(wiimote->index);
    wiimote->status.ir = 0;
    wiimote->status.speaker = 0;
    wiimote->buttons = 0;
    wiimote->input_dirty = false;
    wiimote->acc_x = ACCEL_ZERO_G;
    wiimote->acc_y = ACCEL_ZERO_G;
    wiimote->acc_z = ACCEL_ONE_G;
    wiimote->orientation_mode = WIIMOTE_ORIENTATION_VERTICAL;
    wiimote->is_wheel_mode = false;
    wiimote->rumble_on = false;
    wiimote->battery = 0xFF;
    wiimote->battery_low = false;
    memset(&wiimote->ir_regs, 0, sizeof(wiimote->ir_regs));
    memset(wiimote->ir_regs.camera_data, 0xFF, sizeof(wiimote->ir_regs.camera_data));
    wiimote->ir_valid_dots = 0;
    fake_wiimote_reset_extension_state(wiimote);
    wiimote->cur_extension = WIIMOTE_EXT_NONE;
    wiimote->new_extension = WIIMOTE_EXT_NONE;
    wiimote->mp_active = false;
    wiimote->mp_mode = 0;
    wiimote->mp_challenge_state = 0x02;
    wiimote->mp_challenge_type = 0;
    wiimote->mp_report_toggle = false;
    wiimote->mp_pending_status = false;
    wiimote->gyro_roll = 0;
    wiimote->gyro_yaw = 0;
    wiimote->gyro_pitch = 0;
    wiimote->mp_slow_pitch = true;
    wiimote->mp_slow_roll = true;
    wiimote->mp_slow_yaw = true;
    wiimote->speaker_enabled = false;
    wiimote->speaker_muted = true;
    wiimote->speaker_format = 0;
    wiimote->speaker_volume = 0x40;
    wiimote->speaker_rate_khz = 3;
    wiimote->speaker_active_ticks = 0;
    eeprom_init(&wiimote->eeprom);
    wiimote->read_request.size = 0;
    wiimote->reporting_mode = INPUT_REPORT_ID_BTN;
    wiimote->reporting_continuous = false;
}

void fake_wiimote_handle_hci_cmd_accept_con(fake_wiimote_t *wiimote, u8 role)
{
    int ret;

    /* Connection accepted to our fake wiimote */
    LOG_DEBUG("Connection accepted to a Fake Wiimote!\n");

    /* The Accept_Connection_Request command will cause the Command Status
       event to be sent from the Host Controller when the Host Controller
       begins setting up the connection */
    ret = inject_hci_event_command_status(HCI_CMD_ACCEPT_CON);
    UNUSED(ret);

    wiimote->baseband_state = BASEBAND_STATE_COMPLETE;
    wiimote->hci_con_handle = hci_con_handle_virt_alloc();
    LOG_DEBUG("Fake Wiimote got HCI con_handle: 0x%x\n", wiimote->hci_con_handle);

    /* We can start the ACL (L2CAP) linking now */
    wiimote->acl_state = ACL_STATE_LINKING;

    if (role == HCI_ROLE_MASTER) {
        ret = inject_hci_event_role_change(&wiimote->bdaddr, HCI_ROLE_MASTER);
        UNUSED(ret);
    }

    /* In addition, when the Link Manager determines the connection is established,
     * the Host Controllers on both Bluetooth devices that form the connection
     * will send a Connection Complete event to each Host */
    ret = inject_hci_event_con_compl(&wiimote->bdaddr, wiimote->hci_con_handle, 0);
    UNUSED(ret);

    LOG_DEBUG("Connection complete sent, starting ACL linking!\n");
}

void fake_wiimote_release_input_device(fake_wiimote_t *wiimote)
{
    wiimote->input_device = NULL;
}

int fake_wiimote_disconnect(fake_wiimote_t *wiimote)
{
    int ret = 0;

    wiimote->active = false;

    /* Unassign the currently assigned input device (if any). Clear the pointer
     * too: keeping it would let a later disconnect release a device that has
     * already been reassigned to the other fake Wiimote. */
    if (wiimote->input_device) {
        input_device_release_wiimote(wiimote->input_device);
        wiimote->input_device = NULL;
    }

    /* Does a real Wiimote gracefully disconnect l2cap channels first?
       Not doing that doesn't seem to break anything. */
    if (wiimote->baseband_state == BASEBAND_STATE_COMPLETE) {
        /* If this event is lost the console keeps a zombie connection and will
         * reject the next Connection Request. The retry in fake_wiimote_tick
         * then disconnects again, so a later attempt does get through. */
        ret = inject_hci_event_discon_compl(wiimote->hci_con_handle, 0,
                                            0x13 /* User Ended Connection */);
    }
    wiimote->baseband_state = BASEBAND_STATE_INACTIVE;
    wiimote->con_req_retry = 0;
    wiimote->discon_ticks = 0;
    wiimote->acl_state = ACL_STATE_INACTIVE;
    wiimote->psm_sdp_chn.valid = false;
    wiimote->psm_hid_cntl_chn.valid = false;
    wiimote->psm_hid_intr_chn.valid = false;
    if (wiimote->num_completed_acl_data_packets > 0 && wiimote->hci_con_handle != 0) {
        u16 handle = wiimote->hci_con_handle;
        u16 pkts = (u16)wiimote->num_completed_acl_data_packets;
        inject_hci_event_num_compl_pkts(1, &handle, &pkts);
    }
    wiimote->num_completed_acl_data_packets = 0;
    wiimote->hci_con_handle = 0;

    return ret;
}

void fake_wiimote_set_extension(fake_wiimote_t *wiimote, enum wiimote_ext_e ext)
{
    wiimote->new_extension = ext;
    if (wiimote->acl_state == ACL_STATE_INACTIVE) {
        wiimote->cur_extension = ext;
        fake_wiimote_reset_extension_state(wiimote);
    }
}

void fake_wiimote_set_orientation_mode(fake_wiimote_t *wiimote, u8 mode)
{
    wiimote->orientation_mode = mode;
    if (wiimote->cur_extension == WIIMOTE_EXT_CLASSIC ||
        wiimote->new_extension == WIIMOTE_EXT_CLASSIC) {
        wiimote->is_wheel_mode = false;
    } else {
        wiimote->is_wheel_mode = (mode == WIIMOTE_ORIENTATION_WHEEL);
    }

    if (wiimote->input_device) {
        u32 led_val = wiimote->status.leds ? wiimote->status.leds : BIT(wiimote->index);
        if (wiimote->is_wheel_mode)
            led_val |= BIT(4);
        input_device_set_leds(wiimote->input_device, led_val);
    }
}

void fake_wiimote_report_battery(fake_wiimote_t *wiimote, u8 bat_level, bool charging)
{
    if (!wiimote) return;
    if (bat_level >= 10 || bat_level == 0) {
        wiimote->battery = 0xFF; /* 100% full (4 bars) or default/unknown */
        wiimote->battery_low = false;
    } else {
        wiimote->battery = (u8)(bat_level * 22 + 40); /* 1..9 -> 62..238 (1 to 4 bars) */
        wiimote->battery_low = (bat_level <= 1 && !charging);
    }
}

void fake_wiimote_report_input(fake_wiimote_t *wiimote, u16 buttons)
{
    bool btn_changed = (wiimote->buttons ^ buttons) != 0;

    if (btn_changed) {
        wiimote->buttons = buttons;
        wiimote->input_dirty = true;
    }
}

void fake_wiimote_report_accelerometer(fake_wiimote_t *wiimote, s16 acc_x, s16 acc_y, s16 acc_z)
{
    /*
     * Convert from EGC signed accelerometer units (4096 = 1.0G) to Wiimote units (0..1023):
     * 0G = ACCEL_ZERO_G (512)
     * 1G = ACCEL_ONE_G (616), which is +104 units above 0G.
     */
    if (wiimote->is_wheel_mode) {
        /*
         * Horizontal / Wii Wheel mode (e.g. Mario Kart Wii, NSMB Wii, Mario Party):
         * Wiimote is held sideways inside the wheel (IR camera at left, buttons facing player).
         * Physical correspondence with DS4 held naturally:
         * - DS4 Z (upright / 12 o'clock axis) -> Wiimote X (-1.0G at rest)
         * - DS4 X (steering turn right/left)  -> Wiimote Y (<0 right, >0 left)
         * - DS4 Y (wheel tilt forward/back)   -> Wiimote Z (<0 forward, >0 back)
         * Lets player steer / drive naturally without having to hold the controller sideways.
         */
        s32 x_val = ACCEL_ZERO_G + ((s32)acc_z * 104 / 4096);
        if (x_val < 0) x_val = 0; else if (x_val > 1023) x_val = 1023;
        wiimote->acc_x = (u16)x_val;

        s32 y_val = ACCEL_ZERO_G + ((s32)acc_x * 104 / 4096);
        if (y_val < 0) y_val = 0; else if (y_val > 1023) y_val = 1023;
        wiimote->acc_y = (u16)y_val;

        s32 z_val = ACCEL_ZERO_G + ((s32)acc_y * 104 / 4096);
        if (z_val < 0) z_val = 0; else if (z_val > 1023) z_val = 1023;
        wiimote->acc_z = (u16)z_val;
    } else {
        /*
         * Vertical / Standard Mode:
         * - DS4 -X (roll wrist left/right)  -> Wiimote X (<0 roll right, >0 roll left)
         * - DS4 +Z (lightbar axis)          -> Wiimote Y (>0 tip up / forward thrust, <0 tip down / brake)
         * - DS4 Y (face-up towards ceiling) -> Wiimote Z (+1.0G when flat face up)
         */
        s32 x_target = ACCEL_ZERO_G + ((s32)(-acc_x) * 104 / 4096);
        if (x_target < 160) x_target = 160; else if (x_target > 864) x_target = 864;

        s32 y_target = ACCEL_ZERO_G + ((s32)acc_z * 104 / 4096);
        if (y_target < 160) y_target = 160; else if (y_target > 864) y_target = 864;

        s32 z_target = ACCEL_ZERO_G + ((s32)acc_y * 104 / 4096);
        if (z_target < 160) z_target = 160; else if (z_target > 864) z_target = 864;

        /*
         * Low-pass filter (75% new, 25% previous):
         * Replicates the ~50Hz analog RC filter of the real Wiimote's ADXL330.
         * Absorbs sharp mechanical shock spikes when arm stops abruptly,
         * preventing spurious barrel rolls while preserving instant gesture response.
         */
        wiimote->acc_x = (u16)((wiimote->acc_x * 1 + x_target * 3) / 4);
        wiimote->acc_y = (u16)((wiimote->acc_y * 1 + y_target * 3) / 4);
        wiimote->acc_z = (u16)((wiimote->acc_z * 1 + z_target * 3) / 4);
    }

    wiimote->input_dirty = true;
}

// Compensation of the rate reported to the game (all three axes). The in-game
// rotation tracks noticeably less than the physical motion while a real
// MotionPlus remote tracks 1:1; every stage of the chain was verified correct
// (gyro integration, USB transport, MotionPlus encoding and the game's
// expected scale), so the final gain is a deliberate end-to-end calibration
// against the real hardware, tuned in-game until a full 360 degree turn
// matched the physical gesture. Tuning history: 125% left 5-8 degrees short
// per turn, 127% still fell slightly short after two consecutive turns, 128%
// fell behind after three turns, 129% overshot slightly on the first turn
// and accumulated error, and 128.5% (1285/1000) is the final calibrated
// value. 1000 = no change (pure physical data).
#ifndef DS4_MP_GAIN_PERMIL
#define DS4_MP_GAIN_PERMIL 1285
#endif

void fake_wiimote_report_gyroscope(fake_wiimote_t *wiimote, s16 gyro_x, s16 gyro_y, s16 gyro_z)
{
    /* Clamp -32768 to -32767 to prevent 16-bit two's complement sign wrap on negation (-(-32768) == -32768) */
    if (gyro_x < -32767) gyro_x = -32767;
    if (gyro_y < -32767) gyro_y = -32767;
    if (gyro_z < -32767) gyro_z = -32767;

    s16 pitch, roll, yaw;

    if (wiimote->is_wheel_mode) {
        pitch = gyro_z;
        roll  = gyro_x;
        yaw   = gyro_y;
    } else {
        /*
         * Standard / Vertical Mode (1:1 with Accelerometer):
         * - DS4 gyro_x (Pitch): Wiimote X is -DS4 X, so Wiimote Pitch = -gyro_x.
         * - DS4 gyro_z (Roll):  Wiimote Y is +DS4 Z (lightbar forward), so Wiimote Roll = +gyro_z.
         * - DS4 gyro_y (Yaw):   Wiimote Z is +DS4 Y (face-up out of touchpad), so Wiimote Yaw = +gyro_y.
         */
        pitch = -gyro_x;
        roll  = gyro_z;
        yaw   = gyro_y;
    }

    if (DS4_MP_GAIN_PERMIL != 1000) {
        /* Compensation of the reported rate toward what the game expects
         * from a real MotionPlus, applied to all three axes. 32-bit with a
         * clamp so a boosted full-rate swing can never wrap. */
        int32_t p32 = ((int32_t)pitch * DS4_MP_GAIN_PERMIL) / 1000;
        int32_t r32 = ((int32_t)roll  * DS4_MP_GAIN_PERMIL) / 1000;
        int32_t y32 = ((int32_t)yaw   * DS4_MP_GAIN_PERMIL) / 1000;
        if (p32 > 32767) p32 = 32767; else if (p32 < -32767) p32 = -32767;
        if (r32 > 32767) r32 = 32767; else if (r32 < -32767) r32 = -32767;
        if (y32 > 32767) y32 = 32767; else if (y32 < -32767) y32 = -32767;
        pitch = (s16)p32;
        roll  = (s16)r32;
        yaw   = (s16)y32;
    }

    if (wiimote->gyro_roll != roll || wiimote->gyro_yaw != yaw || wiimote->gyro_pitch != pitch) {
        wiimote->gyro_roll = roll;
        wiimote->gyro_yaw = yaw;
        wiimote->gyro_pitch = pitch;
        if (wiimote->mp_active) {
            wiimote->input_dirty = true;
        }
    }
}

void fake_wiimote_report_ir_dots(fake_wiimote_t *wiimote,
                                 struct ir_dot_t ir_dots[static IR_MAX_DOTS])
{
    u8 *ir_data = wiimote->ir_regs.camera_data;
    u8 old_data[CAMERA_DATA_BYTES];
    memcpy(old_data, ir_data, sizeof(old_data));

    /* If dots are out-of-screen (no finger on touchpad), emit standard 0xFF inactive data */
    if (ir_dots[0].x >= 1023 || ir_dots[0].y >= 1023) {
        memset(ir_data, 0xFF, sizeof(wiimote->ir_regs.camera_data));
        if (memcmp(old_data, ir_data, sizeof(old_data)) != 0) {
            wiimote->input_dirty = true;
        }
        return;
    }

    switch (wiimote->ir_regs.mode) {
    case IR_MODE_BASIC:
        ir_data[0] = ir_dots[0].x & 0xFF;
        ir_data[1] = ir_dots[0].y & 0xFF;
        ir_data[2] = (((ir_dots[0].y >> 8) & 3) << 6) | (((ir_dots[0].x >> 8) & 3) << 4) |
                     (((ir_dots[1].y >> 8) & 3) << 2) | ((ir_dots[1].x >> 8) & 3);
        ir_data[3] = ir_dots[1].x & 0xFF;
        ir_data[4] = ir_dots[1].y & 0xFF;
        ir_data[5] = 0xFF;
        ir_data[6] = 0xFF;
        ir_data[7] = 0xFF;
        ir_data[8] = 0xFF;
        ir_data[9] = 0xFF;
        break;
    case IR_MODE_EXTENDED:
        ir_data[0] = ir_dots[0].x & 0xFF;
        ir_data[1] = ir_dots[0].y & 0xFF;
        ir_data[2] = ((ir_dots[0].y & 0x300) >> 2) | ((ir_dots[0].x & 0x300) >> 4) | IR_DOT_SIZE;
        ir_data[3] = ir_dots[1].x & 0xFF;
        ir_data[4] = ir_dots[1].y & 0xFF;
        ir_data[5] = ((ir_dots[1].y & 0x300) >> 2) | ((ir_dots[1].x & 0x300) >> 4) | IR_DOT_SIZE;
        ir_data[6] = 0xFF;
        ir_data[7] = 0xFF;
        ir_data[8] = 0xFF;
        ir_data[9] = 0xFF;
        ir_data[10] = 0xFF;
        ir_data[11] = 0xFF;
        break;
    case IR_MODE_FULL:
        ir_data[0] = ir_dots[0].x & 0xFF;
        ir_data[1] = ir_dots[0].y & 0xFF;
        ir_data[2] = ((ir_dots[0].y & 0x300) >> 2) | ((ir_dots[0].x & 0x300) >> 4) | IR_DOT_SIZE;
        ir_data[3] = 0;
        ir_data[4] = 0x7F;
        ir_data[5] = 0;
        ir_data[6] = 0x7F;
        ir_data[7] = 0;
        ir_data[8] = 0xFF;
        ir_data[9] = ir_dots[1].x & 0xFF;
        ir_data[10] = ir_dots[1].y & 0xFF;
        ir_data[11] = ((ir_dots[1].y & 0x300) >> 2) | ((ir_dots[1].x & 0x300) >> 4) | IR_DOT_SIZE;
        ir_data[12] = 0;
        ir_data[13] = 0x7F;
        ir_data[14] = 0;
        ir_data[15] = 0x7F;
        ir_data[16] = 0;
        ir_data[17] = 0xFF;
        memset(&ir_data[18], 0xFF, 2 * 9);
        break;
    default:
        /* This seems to be fairly common, 0xff data is sent in this case */
        memset(ir_data, 0xFF, sizeof(wiimote->ir_regs.camera_data));
        break;
    }

    if (memcmp(old_data, ir_data, sizeof(old_data)) != 0) {
        wiimote->input_dirty = true;
    }
}

void fake_wiimote_report_input_ext(fake_wiimote_t *wiimote, u16 buttons, const void *ext_data,
                                   u8 ext_size)
{
    u8 *ext_controller_data = wiimote->extension_regs.controller_data;
    bool btn_changed = (wiimote->buttons ^ buttons) != 0;
    int ext_cmp = memmismatch(ext_controller_data, ext_data, ext_size);

    if (btn_changed || (ext_cmp != ext_size)) {
        wiimote->buttons = buttons;
        /* If there are changes to the extension bytes, copy them */
        if (ext_cmp != ext_size)
            memcpy(ext_controller_data + ext_cmp, ext_data + ext_cmp, ext_size - ext_cmp);
        wiimote->input_dirty = true;
    }
}

/* WMP challenge Rabin signature responses */
static const u8 wmp_challenge_param_x[64] = {
    0x44, 0x38, 0xb2, 0xc9, 0xba, 0x61, 0xe2, 0x50, 0x9e, 0x53, 0x06, 0xbe, 0xa5, 0x35, 0x0a, 0x81,
    0xf0, 0x53, 0x7e, 0x70, 0x27, 0x62, 0xc5, 0xdf, 0xbe, 0x52, 0x13, 0x22, 0x72, 0xa3, 0x28, 0x25,
    0x1b, 0x7c, 0xb5, 0x48, 0xe4, 0xd5, 0x37, 0x45, 0x98, 0xc4, 0x5a, 0x16, 0x4f, 0x3d, 0xdc, 0x81,
    0x6a, 0x93, 0xd1, 0x24, 0xd8, 0x52, 0xb3, 0xd6, 0x1a, 0x39, 0x3a, 0x12, 0x00, 0x00, 0x00, 0x00
};

static const u8 wmp_challenge_param_y0[64] = {
    0x2e, 0x54, 0x27, 0x4e, 0x4f, 0x44, 0x4e, 0x45, 0x54, 0x4e, 0x49, 0x4e, 0x20, 0x54, 0x41, 0x48,
    0x57, 0x20, 0x53, 0x45, 0x4f, 0x44, 0x20, 0x4e, 0x49, 0x48, 0x50, 0x4c, 0x4f, 0x44, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const u8 wmp_challenge_param_y1[64] = {
    0xa6, 0x7d, 0x73, 0xa1, 0xca, 0xf3, 0xa9, 0x7b, 0x5a, 0x19, 0xc5, 0x84, 0x9e, 0xa2, 0xec, 0xda,
    0x5a, 0xac, 0xc4, 0x49, 0x4d, 0x78, 0xe2, 0x5f, 0xfd, 0x9d, 0x0f, 0x19, 0x80, 0xfe, 0x45, 0x1c,
    0x87, 0x78, 0x84, 0xd6, 0xfb, 0xfd, 0x4a, 0xcd, 0x77, 0x25, 0x57, 0x99, 0x9f, 0xcb, 0x54, 0xf6,
    0x91, 0x88, 0x5a, 0xb2, 0x4a, 0x29, 0xbb, 0x1a, 0x70, 0x52, 0xa2, 0xa1, 0xc8, 0x96, 0x3c, 0x7c
};

static inline s32 encode_mp_axis(s32 raw, bool *was_slow)
{
    /*
     * Nintendo Wii MotionPlus dual-mode scaling with hysteresis.
     *
     * The scaling MUST match the calibration block we expose at 0xA60020
     * (the same one Dolphin emulates, with valid CRCs). That calibration
     * tells the game that +/-4352 units (14-bit domain) correspond to
     * 270 deg/s in slow mode and 1200 deg/s in fast mode:
     *   slow: 4352 /  270 = 16.1185 units per deg/s
     *   fast: 4352 / 1200 =  3.6267 units per deg/s
     * The DS4 gyro reports 16.384 LSB per deg/s, so the factors are:
     *   slow: 16.1185 / 16.384 = 984 / 1000
     *   fast:  3.6267 / 16.384 = 221 / 1000
     *
     * The previous nominal factors (840/1000 and 185/1000) did not match the
     * calibration, so the game read only ~85% of the real rotation (verified
     * on hardware: a real 360 deg turn showed ~270 deg on screen, while the
     * same motion with a real Wiimote+MotionPlus shows 360 deg).
     *
     * Hysteresis: enter fast mode above ~500 deg/s (raw 8200), inside the
     * slow-mode sensor limit of ~508 deg/s (4352 units = 270 deg/s, full
     * 14-bit range = ~508 deg/s); return to slow below ~470 deg/s (raw 7700).
     * Those thresholds are also what keep the slow-mode value inside the
     * 14-bit range: 8192 + 8200*0.984 = 16261 < 16383.
     */
    s32 abs_raw = raw < 0 ? -raw : raw;
    bool is_slow = *was_slow;

    if (is_slow && abs_raw > 8200)
        is_slow = false;     /* enter fast mode */
    else if (!is_slow && abs_raw < 7700)
        is_slow = true;      /* return to slow mode */

    *was_slow = is_slow;

    s32 val;
    if (is_slow) {
        val = 8192 + (raw * 984 / 1000);
    } else {
        val = 8192 + (raw * 221 / 1000);
    }
    if (val < 0) return 0;
    if (val > 16383) return 16383;
    return val;
}

static void fake_wiimote_encode_motion_plus(fake_wiimote_t *wiimote, u8 out[6], bool ext_connected)
{
    s32 pitch_val = encode_mp_axis(wiimote->gyro_pitch, &wiimote->mp_slow_pitch);
    s32 roll_val  = encode_mp_axis(wiimote->gyro_roll,  &wiimote->mp_slow_roll);
    s32 yaw_val   = encode_mp_axis(wiimote->gyro_yaw,   &wiimote->mp_slow_yaw);

    out[0] = yaw_val & 0xFF;
    out[1] = roll_val & 0xFF;
    out[2] = pitch_val & 0xFF;
    /* Byte 3: yaw_high (bits 7-2) | yaw_slow (bit 1) | pitch_slow (bit 0) */
    out[3] = ((yaw_val >> 8) << 2) | ((wiimote->mp_slow_yaw ? 1 : 0) << 1) | (wiimote->mp_slow_pitch ? 1 : 0);
    /* Byte 4: roll_high (bits 7-2) | roll_slow (bit 1) | ext_connected (bit 0) */
    out[4] = ((roll_val >> 8) << 2) | ((wiimote->mp_slow_roll ? 1 : 0) << 1) | (ext_connected ? 1 : 0);
    /* Byte 5: pitch_high (bits 7-2) | is_mp_data=1 (bit 1) | reserved=0 (bit 0) */
    out[5] = ((pitch_val >> 8) << 2) | (1 << 1) | 0;
}

static void fake_wiimote_encode_mp_classic(const fake_wiimote_t *wiimote, u8 out[6])
{
    memcpy(out, wiimote->extension_regs.controller_data, 6);
    out[0] = (out[0] & ~1) | ((out[5] >> 0) & 1);
    out[1] = (out[1] & ~1) | ((out[5] >> 1) & 1);
    out[5] &= ~(1 << 1); // is_mp_data = 0
    out[4] |= (1 << 0);  // extension_connected = 1
    out[5] &= ~(1 << 0); // zero = 0
}

static void fake_wiimote_get_mp_ext_data(fake_wiimote_t *wiimote, u8 *dst, u8 size)
{
    u8 buf[6];
    memset(buf, 0, sizeof(buf));

    if (wiimote->cur_extension == WIIMOTE_EXT_NONE || wiimote->mp_mode == 0x04 || wiimote->mp_mode == 0x05 || wiimote->mp_mode == 0) {
        fake_wiimote_encode_motion_plus(wiimote, buf, false);
    } else if (wiimote->mp_mode == 0x07) {
        wiimote->mp_report_toggle = !wiimote->mp_report_toggle;
        if (wiimote->mp_report_toggle) {
            fake_wiimote_encode_motion_plus(wiimote, buf, true);
        } else {
            fake_wiimote_encode_mp_classic(wiimote, buf);
        }
    }

    u8 copy_len = MIN2(size, 6);
    memcpy(dst, buf, copy_len);
    if (size > 6) {
        memset(dst + 6, 0, size - 6);
    }
}

static bool motion_plus_read_data(fake_wiimote_t *wiimote, void *dst, u16 address, u16 size)
{
    u8 *out = dst;
    memset(out, 0xFF, size);

    for (u16 i = 0; i < size; i++) {
        u16 reg = address + i;
        if (reg >= 0xFA && reg <= 0xFF) {
            out[i] = EXP_ID_CODE_MOTION_PLUS[reg - 0xFA];
        } else if (reg == 0xF7) {
            out[i] = wiimote->mp_challenge_state;
            if (wiimote->mp_challenge_state == 0x02) wiimote->mp_challenge_state = 0x04;
            else if (wiimote->mp_challenge_state == 0x04) wiimote->mp_challenge_state = 0x08;
            else if (wiimote->mp_challenge_state == 0x08) wiimote->mp_challenge_state = 0x0C;
            else if (wiimote->mp_challenge_state == 0x0C) wiimote->mp_challenge_state = 0x0E;
            else if (wiimote->mp_challenge_state == 0x14) wiimote->mp_challenge_state = 0x18;
            else if (wiimote->mp_challenge_state == 0x18) wiimote->mp_challenge_state = 0x1A;
        } else if (reg == 0xF6 || reg == 0xF8 || reg == 0xF9) {
            out[i] = 0x00;
        } else if (reg == 0xF0) {
            out[i] = 0x55;
        } else if (reg >= 0x50 && reg < 0x90) {
            const u8 *param = wmp_challenge_param_x;
            if (wiimote->mp_challenge_state >= 0x14) {
                param = (wiimote->mp_challenge_type == 0) ? wmp_challenge_param_y0 : wmp_challenge_param_y1;
            }
            out[i] = param[reg - 0x50];
        } else if (reg >= 0x20 && reg < 0x40) {
            out[i] = wmp_calibration_data[reg - 0x20];
        } else if (reg < 0x20 || (reg >= 0x40 && reg < 0x50)) {
            out[i] = 0x00;
        }
    }
    return true;
}

static bool motion_plus_write_data(fake_wiimote_t *wiimote, const void *src, u16 address, u16 size)
{
    const u8 *data = src;
    for (u16 i = 0; i < size; i++) {
        u16 reg = address + i;
        if (reg == 0xF0) {
            if (data[i] == 0x55) {
                wiimote->mp_challenge_state = 0x02;
            }
        } else if (reg == 0xF1) {
            wiimote->mp_challenge_type = data[i];
            wiimote->mp_challenge_state = 0x14;
        } else if (reg == 0xFE) {
            u8 mode = data[i];
            if (mode == 0x04 || mode == 0x05 || mode == 0x07) {
                wiimote->cur_extension = WIIMOTE_EXT_NONE;
                wiimote->mp_active = true;
                wiimote->mp_mode = mode;
                wiimote->mp_challenge_state = 0x02;
                wiimote->mp_report_toggle = false;

                wiimote->extension_regs.identifier[0] = 0x00;
                wiimote->extension_regs.identifier[1] = 0x00;
                wiimote->extension_regs.identifier[2] = 0xA4;
                wiimote->extension_regs.identifier[3] = 0x20;
                wiimote->extension_regs.identifier[4] = mode;
                wiimote->extension_regs.identifier[5] = 0x05;

                wiimote->mp_pending_status = true;
            } else if (mode == 0x00) {
                wiimote->mp_active = false;
                wiimote->mp_mode = 0;
                fake_wiimote_reset_extension_state(wiimote);
                wiimote->mp_pending_status = true;
            }
        }
    }
    return true;
}

static bool speaker_read_data(fake_wiimote_t *wiimote, void *dst, u16 address, u16 size)
{
    memset(dst, 0xFF, size);
    for (u16 i = 0; i < size; i++) {
        u16 reg = address + i;
        if (reg == 0x08 || reg == 0x09) {
            ((u8 *)dst)[i] = 0x09;
        }
    }
    return true;
}

static bool speaker_write_data(fake_wiimote_t *wiimote, const void *src, u16 address, u16 size)
{
    const u8 *data = src;
    if (address == 0x00) {
        wiimote->speaker_active_ticks = 40;
        u8 pkt[25];
        pkt[0] = OUTPUT_REPORT_ID_SPEAKER_DATA;
        pkt[1] = (u8)((wiimote->index << 4) | (wiimote->speaker_format & 0x0F));
        pkt[2] = wiimote->speaker_rate_khz;
        pkt[3] = wiimote->speaker_volume;
        u8 len = MIN2(size, 20);
        pkt[4] = len;
        memcpy(&pkt[5], data, len);
        input_device_send_speaker_data(wiimote->input_device, pkt, 5 + len);
    } else {
        for (u16 i = 0; i < size; i++) {
            u16 reg = address + i;
            if (reg == 0x01 || reg == 0x02) {
                wiimote->speaker_format = (data[i] == 0x40) ? 1 : 0;
            } else if (reg == 0x03 || (reg == 0x04 && size > 1)) {
                wiimote->speaker_rate_khz = 3;
            } else if (reg == 0x05 || (reg == 0x04 && size == 1)) {
                wiimote->speaker_volume = data[i];
            }
        }
    }
    return true;
}

static inline bool ir_camera_read_data(fake_wiimote_t *wiimote, void *dst, u16 address, u16 size)
{
    if (address + size > sizeof(wiimote->ir_regs))
        return false;

    /* Copy the requested data from the IR camera registers */
    memcpy(dst, (u8 *)&wiimote->ir_regs + address, size);

    return true;
}

static inline bool ir_camera_write_data(fake_wiimote_t *wiimote, const void *src, u16 address,
                                        u16 size)
{
    if (address + size > sizeof(wiimote->ir_regs))
        return false;

    /* Copy the requested data to the IR camera registers */
    memcpy((u8 *)&wiimote->ir_regs + address, src, size);

    return true;
}

static bool extension_read_data(fake_wiimote_t *wiimote, void *dst, u16 address, u16 size)
{
    if (address + size > sizeof(wiimote->extension_regs))
        return false;

    if (wiimote->mp_active) {
        if (address < 0x20) {
            u8 full[0x20];
            memset(full, 0, sizeof(full));
            fake_wiimote_get_mp_ext_data(wiimote, full, 6);
            memcpy(full + 8, full, 6);
            if (address < sizeof(full)) {
                u16 copy_len = MIN2(size, sizeof(full) - address);
                memcpy(dst, full + address, copy_len);
            }
            return true;
        }
        u8 *out = dst;
        memset(out, 0xFF, size);
        for (u16 i = 0; i < size; i++) {
            u16 reg = address + i;
            if (reg >= 0xFA && reg <= 0xFF) {
                u8 id[6] = { 0x00, 0x00, 0xA4, 0x20, wiimote->mp_mode, 0x05 };
                out[i] = id[reg - 0xFA];
            } else if (reg == 0xF7) {
                out[i] = wiimote->mp_challenge_state;
                // Advance challenge state on poll
                if (wiimote->mp_challenge_state == 0x02) wiimote->mp_challenge_state = 0x04;
                else if (wiimote->mp_challenge_state == 0x04) wiimote->mp_challenge_state = 0x08;
                else if (wiimote->mp_challenge_state == 0x08) wiimote->mp_challenge_state = 0x0C;
                else if (wiimote->mp_challenge_state == 0x0C) wiimote->mp_challenge_state = 0x0E;
                else if (wiimote->mp_challenge_state == 0x14) wiimote->mp_challenge_state = 0x18;
                else if (wiimote->mp_challenge_state == 0x18) wiimote->mp_challenge_state = 0x1A;
            } else if (reg == 0xF6 || reg == 0xF8 || reg == 0xF9) {
                out[i] = 0x00;
            } else if (reg == 0xF0) {
                out[i] = 0x55;
            } else if (reg >= 0x50 && reg < 0x90) {
                const u8 *param = wmp_challenge_param_x;
                if (wiimote->mp_challenge_state >= 0x14) {
                    param = (wiimote->mp_challenge_type == 0) ? wmp_challenge_param_y0 : wmp_challenge_param_y1;
                }
                out[i] = param[reg - 0x50];
            } else if (reg >= 0x20 && reg < 0x40) {
                out[i] = wmp_calibration_data[reg - 0x20];
            } else if (reg >= 0x40 && reg < 0x50) {
                out[i] = 0x00;
            }
        }
        return true;
    }

    /* Copy the requested data from the extension registers */
    memcpy(dst, (u8 *)&wiimote->extension_regs + address, size);

    /* Encrypt data read from extension registers (if necessary) */
    if (wiimote->extension_regs.encryption == ENCRYPTION_ENABLED) {
        if (wiimote->extension_key_dirty) {
            wiimote_crypto_generate_key_from_extension_key_data(
                &wiimote->extension_key, wiimote->extension_regs.encryption_key_data);
            wiimote->extension_key_dirty = false;
        }
        wiimote_crypto_encrypt(dst, &wiimote->extension_key, address, size);
    }

    return true;
}

static bool extension_write_data(fake_wiimote_t *wiimote, const void *src, u16 address, u16 size)
{
    if (address + size > sizeof(wiimote->extension_regs))
        return false;

    if (wiimote->mp_active) {
        const u8 *data = src;
        for (u16 i = 0; i < size; i++) {
            u16 reg = address + i;
            if (reg == 0xF0) {
                if (data[i] == 0x55) {
                    wiimote->mp_challenge_state = 0x02;
                }
            } else if (reg == 0xF1) {
                wiimote->mp_challenge_type = data[i];
                wiimote->mp_challenge_state = 0x14;
            } else if (reg == 0xFE) {
                u8 mode = data[i];
                if (mode == 0x04 || mode == 0x05 || mode == 0x07) {
                    wiimote->cur_extension = WIIMOTE_EXT_NONE;
                    wiimote->mp_active = true;
                    wiimote->mp_mode = mode;
                    wiimote->extension_regs.identifier[0] = 0x00;
                    wiimote->extension_regs.identifier[1] = 0x00;
                    wiimote->extension_regs.identifier[2] = 0xA4;
                    wiimote->extension_regs.identifier[3] = 0x20;
                    wiimote->extension_regs.identifier[4] = mode;
                    wiimote->extension_regs.identifier[5] = 0x05;
                    wiimote->mp_pending_status = true;
                } else if (mode == 0x00) {
                    wiimote->mp_active = false;
                    wiimote->mp_mode = 0;
                    fake_wiimote_reset_extension_state(wiimote);
                    wiimote->mp_pending_status = true;
                }
            }
        }
        return true;
    }

    if ((address + size > ENCRYPTION_KEY_DATA_BEGIN) && (address < ENCRYPTION_KEY_DATA_END)) {
        /* We just run the key generation on all writes to the key area */
        wiimote->extension_key_dirty = true;
    }

    /* Copy the requested data to the extension registers */
    memcpy((u8 *)&wiimote->extension_regs + address, src, size);
    return true;
}

static bool fake_wiimote_process_read_request(fake_wiimote_t *wiimote)
{
    struct wiimote_input_report_read_data_t reply;
    u8 error = ERROR_CODE_SUCCESS;
    u16 address, read_size = MIN2(16, wiimote->read_request.size);

    if (read_size == 0) {
        wiimote->read_request.ticks = 0;
        return false;
    }

    if (++wiimote->read_request.ticks > 20) {
        wiimote->read_request.size = 0;
        wiimote->read_request.ticks = 0;
        return false;
    }

    address = wiimote->read_request.address;
    memset(&reply.data, 0, sizeof(reply.data));

    switch (wiimote->read_request.space) {
    case ADDRESS_SPACE_EEPROM:
        if (address + wiimote->read_request.size > EEPROM_FREE_SIZE)
            error = ERROR_CODE_INVALID_ADDRESS;
        else
            memcpy(reply.data, &wiimote->eeprom.data[address], read_size);
        break;
    case ADDRESS_SPACE_I2C_BUS:
    case ADDRESS_SPACE_I2C_BUS_ALT:
        /* Attempting to access the EEPROM directly over i2c results in error 8 */
        if (wiimote->read_request.slave_address == EEPROM_I2C_ADDR) {
            error = ERROR_CODE_INVALID_ADDRESS;
        } else if (wiimote->read_request.slave_address == EXTENSION_I2C_ADDR) {
            if (wiimote->cur_extension == WIIMOTE_EXT_NONE && !wiimote->mp_active) {
                error = ERROR_CODE_NACK;
            } else if (!extension_read_data(wiimote, reply.data, address, read_size)) {
                error = ERROR_CODE_NACK;
            }
        } else if (wiimote->read_request.slave_address == CAMERA_I2C_ADDR) {
            if (!ir_camera_read_data(wiimote, reply.data, address, read_size))
                error = ERROR_CODE_NACK;
        } else if (wiimote->read_request.slave_address == MOTION_PLUS_I2C_ADDR) {
            if (!fake_wiimote_has_motion_plus(wiimote) || wiimote->mp_active) {
                error = ERROR_CODE_NACK;
            } else if (!motion_plus_read_data(wiimote, reply.data, address, read_size)) {
                error = ERROR_CODE_NACK;
            }
        } else if (wiimote->read_request.slave_address == SPEAKER_I2C_ADDR) {
            if (!speaker_read_data(wiimote, reply.data, address, read_size))
                error = ERROR_CODE_NACK;
        } else {
            error = ERROR_CODE_NACK;
        }
        break;
    default:
        error = ERROR_CODE_INVALID_SPACE;
        break;
    }

    /* Stop processing request on read error */
    if (error != ERROR_CODE_SUCCESS) {
        wiimote->read_request.size = 0;
        /* Real wiimote seems to set size to max value on read errors */
        read_size = 16;
    } else {
        wiimote->read_request.address += read_size;
        wiimote->read_request.size -= read_size;
    }

    reply.buttons = wiimote->buttons;
    reply.size_minus_one = read_size - 1;
    reply.error = error;
    reply.address = address;
    send_hid_input_report(wiimote->hci_con_handle, wiimote->psm_hid_intr_chn.remote_cid,
                          INPUT_REPORT_ID_READ_DATA_REPLY, &reply, sizeof(reply));
    return true;
}

static void fake_wiimote_process_write_request(fake_wiimote_t *wiimote,
                                               struct wiimote_output_report_write_data_t *write)
{
    u8 error = ERROR_CODE_SUCCESS;

    if (write->size == 0 || write->size > 16) {
        /* A real wiimote silently ignores such a request */
        return;
    }

    switch (write->space) {
    case ADDRESS_SPACE_EEPROM:
        if (write->address + write->size > EEPROM_FREE_SIZE)
            error = ERROR_CODE_INVALID_ADDRESS;
        else
            memcpy(&wiimote->eeprom.data[write->address], write->data, write->size);
        break;
    case ADDRESS_SPACE_I2C_BUS:
    case ADDRESS_SPACE_I2C_BUS_ALT:
        /* Attempting to access the EEPROM directly over i2c results in error 8 */
        if (write->slave_address == EEPROM_I2C_ADDR) {
            error = ERROR_CODE_INVALID_ADDRESS;
        } else if (write->slave_address == EXTENSION_I2C_ADDR) {
            if (wiimote->cur_extension == WIIMOTE_EXT_NONE && !wiimote->mp_active) {
                error = ERROR_CODE_NACK;
            } else if (!extension_write_data(wiimote, write->data, write->address, write->size)) {
                error = ERROR_CODE_NACK;
            }
        } else if (write->slave_address == CAMERA_I2C_ADDR) {
            if (!ir_camera_write_data(wiimote, write->data, write->address, write->size))
                error = ERROR_CODE_NACK;
        } else if (write->slave_address == MOTION_PLUS_I2C_ADDR) {
            if (!fake_wiimote_has_motion_plus(wiimote) || wiimote->mp_active) {
                error = ERROR_CODE_NACK;
            } else if (!motion_plus_write_data(wiimote, write->data, write->address, write->size)) {
                error = ERROR_CODE_NACK;
            }
        } else if (write->slave_address == SPEAKER_I2C_ADDR) {
            if (!speaker_write_data(wiimote, write->data, write->address, write->size))
                error = ERROR_CODE_NACK;
        } else {
            error = ERROR_CODE_NACK;
        }
        break;
    default:
        error = ERROR_CODE_INVALID_SPACE;
        break;
    }

    /* Real wiimotes seem to always ACK data writes */
    wiimote_send_ack(wiimote, OUTPUT_REPORT_ID_WRITE_DATA, error);
}

static inline bool fake_wiimote_process_extension_change(fake_wiimote_t *wiimote)
{
    if (wiimote->new_extension == wiimote->cur_extension)
        return false;

    /* Following a connection or disconnection event on the Extension Port, data reporting
     * is disabled and the Data Reporting Mode must be reset before new data can arrive */
    wiimote->reporting_mode = INPUT_REPORT_ID_REPORT_DISABLED;

    if (wiimote->new_extension == WIIMOTE_EXT_CLASSIC && (wiimote->mp_active || wiimote->mp_mode != 0)) {
        /* If user switched to Classic Controller while Motion Plus was active,
         * disconnect Motion Plus first so the console detects the state change cleanly. */
        wiimote->mp_active = false;
        wiimote->mp_mode = 0;
        wiimote->cur_extension = WIIMOTE_EXT_NONE;
        fake_wiimote_reset_extension_state(wiimote);
        wiimote_send_input_report_status(wiimote);
        return true;
    }

    if (wiimote->cur_extension == WIIMOTE_EXT_NONE) {
        /* Extension connect */
        wiimote->cur_extension = wiimote->new_extension;
    } else {
        /* First we must detach the current extension.
           The next call will change to the new extension if needed. */
        wiimote->cur_extension = WIIMOTE_EXT_NONE;
    }

    fake_wiimote_reset_extension_state(wiimote);
    wiimote_send_input_report_status(wiimote);

    return true;
}

static void fake_wiimote_send_data_report(fake_wiimote_t *wiimote)
{
    u8 report_data[CONTROLLER_DATA_BYTES] ATTRIBUTE_ALIGN(4);
    u16 buttons;
    bool has_btn;
    u8 acc_size, acc_offset;
    u8 ext_size, ext_offset;
    u8 ir_size, ir_offset;
    u8 report_size;

    if (wiimote->reporting_mode == INPUT_REPORT_ID_REPORT_DISABLED) {
        if (wiimote->buttons != 0) {
            wiimote->reporting_mode = INPUT_REPORT_ID_BTN;
        } else {
            return;
        }
    }

    if (wiimote->reporting_continuous || wiimote->input_dirty) {
        buttons = wiimote->buttons;
        has_btn = input_report_has_btn(wiimote->reporting_mode);
        acc_size = input_report_acc_size(wiimote->reporting_mode);
        acc_offset = input_report_acc_offset(wiimote->reporting_mode);
        ext_size = input_report_ext_size(wiimote->reporting_mode);
        ext_offset = input_report_ext_offset(wiimote->reporting_mode);
        ir_size = input_report_ir_size(wiimote->reporting_mode);
        ir_offset = input_report_ir_offset(wiimote->reporting_mode);
        report_size = (has_btn ? 2 : 0) + acc_size + ext_size + ir_size;

        if (acc_size) {
            report_data[acc_offset + 0] = (wiimote->acc_x >> 2) & 0xFF;
            report_data[acc_offset + 1] = (wiimote->acc_y >> 2) & 0xFF;
            report_data[acc_offset + 2] = (wiimote->acc_z >> 2) & 0xFF;
            buttons |= ((wiimote->acc_x & 3) << 13) | ((wiimote->acc_y & 2) << 4) |
                       ((wiimote->acc_z & 2) << 5);
        }

        if (ir_size)
            memcpy(&report_data[ir_offset], wiimote->ir_regs.camera_data, ir_size);

        if (ext_size) {
            /* Takes care of encrypting the extension data if necessary */
            if (wiimote->cur_extension == WIIMOTE_EXT_CLASSIC) {
                extension_read_data(wiimote, report_data + ext_offset, 0, ext_size);
            } else if (wiimote->mp_active) {
                fake_wiimote_get_mp_ext_data(wiimote, report_data + ext_offset, ext_size);
            } else if (wiimote->cur_extension != WIIMOTE_EXT_NONE) {
                extension_read_data(wiimote, report_data + ext_offset, 0, ext_size);
            } else {
                memset(report_data + ext_offset, 0xFF, ext_size);
            }
        }

        if (has_btn)
            memcpy(report_data, &buttons, sizeof(buttons));

        send_hid_input_report(wiimote->hci_con_handle, wiimote->psm_hid_intr_chn.remote_cid,
                              wiimote->reporting_mode, report_data, report_size);

        wiimote->input_dirty = false;
    }
}

static inline void fake_wiimote_update_rumble(fake_wiimote_t *wiimote, bool rumble_on)
{
    if (rumble_on != wiimote->rumble_on) {
        wiimote->rumble_on = rumble_on;
        input_device_set_rumble(wiimote->input_device, rumble_on);
    }
}

static void check_send_config_for_new_channel(u16 hci_con_handle, l2cap_channel_info_t *info)
{
    int ret;

    if (l2cap_channel_is_accepted(info) && (info->state == L2CAP_CHANNEL_STATE_INACTIVE)) {
        ret = inject_l2cap_config_req(hci_con_handle, info->remote_cid, WII_REQUEST_MTU,
                                      L2CAP_FLUSH_TIMO_DEFAULT);
        if (ret == IOS_OK) {
            info->state = L2CAP_CHANNEL_STATE_CONFIG_PEND;
        }
    }
}

void fake_wiimote_tick(fake_wiimote_t *wiimote)
{
    int ret;

    if (!wiimote->active || !wiimote->input_device)
        return;

    if (wiimote->speaker_active_ticks > 0)
        wiimote->speaker_active_ticks--;

    if (wiimote->baseband_state == BASEBAND_STATE_REQUEST_CONNECTION) {
        if (!input_device_is_controller_connected(wiimote->input_device)) {
            /* Physical controller is disconnected or asleep: DO NOT page Broadway! */
            return;
        }
        if (hci_can_request_connection()) {
            ret = inject_hci_event_con_req(&wiimote->bdaddr, WIIMOTE_HCI_CLASS_0,
                                           WIIMOTE_HCI_CLASS_1, WIIMOTE_HCI_CLASS_2, HCI_LINK_ACL);
            /* After a connection request is visible to the controller switch to inactive */
            if (ret == IOS_OK) {
                wiimote->baseband_state = BASEBAND_STATE_INACTIVE;
                wiimote->con_req_retry = CON_REQ_RETRY_TICKS;
            }
        }
    } else if (wiimote->baseband_state == BASEBAND_STATE_INACTIVE) {
        /* Waiting for the console to Accept or Reject our Connection Request.
         * If neither arrives the controller would look dead and never recover,
         * so ask again. */
        if (!input_device_is_controller_connected(wiimote->input_device)) {
            wiimote->con_req_retry = 0;
            return;
        }
        if (wiimote->con_req_retry == 0 || --wiimote->con_req_retry == 0)
            wiimote->baseband_state = BASEBAND_STATE_REQUEST_CONNECTION;
    } else if (wiimote->baseband_state == BASEBAND_STATE_COMPLETE) {
        /* "If the connection originated from the device (Wiimote) it will create
         * HID control and interrupt channels (in that order)." */
        if (wiimote->acl_state == ACL_STATE_LINKING) {
            bool hid_cntl_chn_complete = l2cap_channel_is_complete(&wiimote->psm_hid_cntl_chn);

            /* If-else-if cascade to avoid sending too many packets on the same "tick" */
            if (!wiimote->psm_hid_cntl_chn.valid) {
                u16 local_cid = generate_l2cap_channel_id();
                ret = inject_l2cap_connect_req(wiimote->hci_con_handle, L2CAP_PSM_HID_CNTL,
                                               local_cid);
                if (ret != IOS_OK)
                    return;
                l2cap_channel_info_setup(&wiimote->psm_hid_cntl_chn, L2CAP_PSM_HID_CNTL, local_cid);
                LOG_DEBUG("Generated local CID for HID CNTL: 0x%x\n", local_cid);
            } else if (hid_cntl_chn_complete && !wiimote->psm_hid_intr_chn.valid) {
                u16 local_cid = generate_l2cap_channel_id();
                ret = inject_l2cap_connect_req(wiimote->hci_con_handle, L2CAP_PSM_HID_INTR,
                                               local_cid);
                if (ret != IOS_OK)
                    return;
                l2cap_channel_info_setup(&wiimote->psm_hid_intr_chn, L2CAP_PSM_HID_INTR, local_cid);
                LOG_DEBUG("Generated local CID for HID INTR: 0x%x\n", local_cid);
            } else if (hid_cntl_chn_complete &&
                       l2cap_channel_is_complete(&wiimote->psm_hid_intr_chn)) {
                wiimote->acl_state = ACL_STATE_INACTIVE;
                /* Call resume() input device callback */
                input_device_resume(wiimote->input_device);
                return;
            }
            /* Send configuration for any newly connected channels. */
            check_send_config_for_new_channel(wiimote->hci_con_handle, &wiimote->psm_hid_cntl_chn);
            check_send_config_for_new_channel(wiimote->hci_con_handle, &wiimote->psm_hid_intr_chn);
        } else {
            /* Both HID ctrl and intr channels are connected (we only need intr though) */
            if (!input_device_is_controller_connected(wiimote->input_device)) {
                return;
            }

            if (fake_wiimote_process_read_request(wiimote)) {
                /* Read requests suppress normal input reports.
                 * Don't send any other reports */
                return;
            }

            if (wiimote->mp_pending_status) {
                wiimote->mp_pending_status = false;
                wiimote_send_input_report_status(wiimote);
                return;
            }

            if (fake_wiimote_process_extension_change(wiimote)) {
                /* Extension port event occurred. Don't send any other reports. */
                return;
            }

            if (input_device_report_input(wiimote->input_device))
                fake_wiimote_send_data_report(wiimote);
        }
    }
}

static void handle_l2cap_config_req(fake_wiimote_t *wiimote, u8 ident, u16 dcid, u16 flags,
                                    const u8 *options, u16 options_size)
{
    l2cap_channel_info_t *info;
    l2cap_cfg_opt_t *opt;
    l2cap_cfg_opt_val_t *val;
    u32 offset = 0;
    /* If the option is not provided, configure the default. */
    u16 remote_mtu = L2CAP_MTU_DEFAULT;

    UNUSED(flags);

    info = get_channel_info(wiimote, dcid);
    if (!info) {
        LOG_DEBUG("handle_l2cap_config_req: unknown dcid 0x%x\n", dcid);
        return;
    }

    /* Read configuration options. */
    while (offset < options_size) {
        opt = (l2cap_cfg_opt_t *)&options[offset];
        offset += sizeof(l2cap_cfg_opt_t);
        val = (l2cap_cfg_opt_val_t *)&options[offset];

        switch (opt->type) {
        case L2CAP_OPT_MTU:
            if (opt->length >= L2CAP_OPT_MTU_SIZE) {
                remote_mtu = le16toh(val->mtu);
                LOG_DEBUG("      MTU configured to: 0x%x\n", remote_mtu);
            }
            break;
        /* We don't care what the flush timeout is. Our packets are not dropped. */
        case L2CAP_OPT_FLUSH_TIMO:
            if (opt->length >= L2CAP_OPT_FLUSH_TIMO_SIZE) {
                LOG_DEBUG("      Flush timeout configured to 0x%x\n", val->flush_timo);
            }
            break;
        default:
            LOG_DEBUG("      Unknown Option: 0x%02x", opt->type);
            break;
        }

        offset += opt->length;
    }

    /* Set the configured MTU */
    info->remote_mtu = remote_mtu;

    /* Send Respone (with the same options as received) */
    inject_l2cap_config_rsp(wiimote->hci_con_handle, info->remote_cid, ident, options,
                            options_size);
}

static void handle_l2cap_signal_channel(fake_wiimote_t *wiimote, u8 code, u8 ident,
                                        const void *payload, u16 size)
{
    l2cap_channel_info_t *info;

    LOG_DEBUG("  signal channel: code: 0x%x, ident: 0x%x\n", code, ident);

    switch (code) {
    case L2CAP_CONNECT_REQ: {
        const l2cap_con_req_cp *req = payload;
        u16 psm = le16toh(req->psm);
        u16 scid = le16toh(req->scid);
        UNUSED(psm);
        UNUSED(scid);
        LOG_DEBUG("  L2CAP_CONNECT_REQ: psm: 0x%x, scid: 0x%x\n", psm, scid);
        /* TODO */
        break;
    }
    case L2CAP_CONNECT_RSP: {
        const l2cap_con_rsp_cp *rsp = payload;
        u16 dcid = le16toh(rsp->dcid);
        u16 scid = le16toh(rsp->scid);
        u16 result = le16toh(rsp->result);
        u16 status = le16toh(rsp->status);
        LOG_DEBUG("  L2CAP_CONNECT_RSP: dcid: 0x%x, scid: 0x%x, result: 0x%x, status: 0x%x\n", dcid,
                  scid, result, status);

        /* libogc/master/lwbt/l2cap.c#L318 sets it to the "dcid" and not to
         * the "result" field (and scid to 0)... */
        if ((result != L2CAP_SUCCESS) || ((dcid == L2CAP_PSM_NOT_SUPPORTED) && (scid == 0))) {
            fake_wiimote_disconnect(wiimote);
            break;
        }

        UNUSED(status);
        info = get_channel_info(wiimote, scid);
        if (info) {
            /* Save endpoint's Destination CID  */
            info->remote_cid = dcid;
        }
        break;
    }
    case L2CAP_CONFIG_REQ: {
        const l2cap_cfg_req_cp *rsp = payload;
        u16 dcid = le16toh(rsp->dcid);
        u16 flags = le16toh(rsp->flags);
        const void *options = (const void *)((u8 *)rsp + sizeof(l2cap_cfg_req_cp));
        UNUSED(flags);

        LOG_DEBUG("  L2CAP_CONFIG_REQ: dcid: 0x%x, flags: 0x%x\n", dcid, flags);
        handle_l2cap_config_req(wiimote, ident, dcid, flags, options,
                                size - sizeof(l2cap_cfg_req_cp));
        break;
    }
    case L2CAP_CONFIG_RSP: {
        const l2cap_cfg_rsp_cp *rsp = payload;
        u16 scid = le16toh(rsp->scid);
        u16 flags = le16toh(rsp->flags);
        u16 result = le16toh(rsp->result);
        UNUSED(flags);
        LOG_DEBUG("  L2CAP_CONFIG_RSP: scid: 0x%x, flags: 0x%x, result: 0x%x\n", scid, flags,
                  result);

        UNUSED(result);
        info = get_channel_info(wiimote, scid);
        if (info) {
            /* Mark channel as complete!  */
            info->state = L2CAP_CHANNEL_STATE_COMPLETE;
        }
        break;
    }
    case L2CAP_DISCONNECT_REQ: {
        const l2cap_discon_req_cp *req = payload;
        u16 dcid = le16toh(req->dcid);
        u16 scid = le16toh(req->scid);
        LOG_DEBUG("  L2CAP_DISCONNECT_REQ: dcid: 0x%x, scid: 0x%x\n", dcid, scid);

        info = get_channel_info(wiimote, dcid);
        if (info) {
            info->valid = false;
        }

        /* Send disconnect response */
        inject_l2cap_disconnect_rsp(wiimote->hci_con_handle, ident, dcid, scid);

        /* If both HID channels are closed by the host, disconnect fake Wiimote */
        if (!wiimote->psm_hid_cntl_chn.valid && !wiimote->psm_hid_intr_chn.valid) {
            fake_wiimote_disconnect(wiimote);
        }
        break;
    }
    }
}

static void handle_l2cap_signal_channel_request(fake_wiimote_t *wiimote, const void *data,
                                                u16 length)
{
    const l2cap_cmd_hdr_t *cmd_hdr;
    const void *cmd_payload;
    u16 cmd_len;

    while (length >= sizeof(l2cap_cmd_hdr_t)) {
        cmd_hdr = (const void *)data;
        cmd_len = le16toh(cmd_hdr->length);
        cmd_payload = (const void *)((u8 *)data + sizeof(*cmd_hdr));

        handle_l2cap_signal_channel(wiimote, cmd_hdr->code, cmd_hdr->ident, cmd_payload, cmd_len);

        data += sizeof(l2cap_cmd_hdr_t) + cmd_len;
        length -= sizeof(l2cap_cmd_hdr_t) + cmd_len;
    }
}

static void handle_hid_intr_data_output(fake_wiimote_t *wiimote, const u8 *data, u16 size)
{
    LOG_DEBUG("handle_hid_intr_data_output: size: 0x%" PRIx16 "\n", size);

    if (size == 0)
        return;

    /* Setting the LSB (bit 0) of the first byte of any output report
     * will activate the rumble motor, and unsetting it will deactivate it */
    if (size >= 2)
        fake_wiimote_update_rumble(wiimote, data[1] & 1);

    switch (data[0]) {
    case OUTPUT_REPORT_ID_RUMBLE:
        /* No need to do anything, just contains the rumble bit */
        break;
    case OUTPUT_REPORT_ID_LED: {
        u8 raw_leds = (data[1] >> 4) & 0x0F;
        wiimote->status.leds = raw_leds;
        u32 led_val = raw_leds;
        if (wiimote->is_wheel_mode)
            led_val |= BIT(4);
        input_device_set_leds(wiimote->input_device, led_val);
        if (data[1] & 0x02)
            wiimote_send_ack(wiimote, OUTPUT_REPORT_ID_LED, ERROR_CODE_SUCCESS);
        break;
    }
    case OUTPUT_REPORT_ID_REPORT_MODE: {
        u8 mode = data[2];
        bool continuous = (data[1] & 0x04) != 0;
        LOG_DEBUG("  Report mode: 0x%02x, cont: %d\n", mode, continuous);
        wiimote->reporting_mode = mode;
        wiimote->reporting_continuous = continuous;
        if (data[1] & 0x02)
            wiimote_send_ack(wiimote, OUTPUT_REPORT_ID_REPORT_MODE, ERROR_CODE_SUCCESS);
        break;
    }
    case OUTPUT_REPORT_ID_IR_ENABLE: {
        bool enable = (data[1] & 0x04) != 0;
        wiimote->status.ir = enable;
        if (!enable) {
            memset(wiimote->ir_regs.camera_data, 0xFF, sizeof(wiimote->ir_regs.camera_data));
            wiimote->input_dirty = true;
        }
        if (data[1] & 0x02)
            wiimote_send_ack(wiimote, OUTPUT_REPORT_ID_IR_ENABLE, ERROR_CODE_SUCCESS);
        break;
    }
    case OUTPUT_REPORT_ID_SPEAKER_ENABLE: {
        bool enable = (data[1] & 0x04) != 0;
        wiimote->status.speaker = enable;
        wiimote->speaker_enabled = enable;
        if (data[1] & 0x02)
            wiimote_send_ack(wiimote, OUTPUT_REPORT_ID_SPEAKER_ENABLE, ERROR_CODE_SUCCESS);
        break;
    }
    case OUTPUT_REPORT_ID_STATUS:
        wiimote_send_input_report_status(wiimote);
        break;
    case OUTPUT_REPORT_ID_WRITE_DATA: {
        struct wiimote_output_report_write_data_t *write = (void *)&data[1];
        LOG_DEBUG("  Write data to slave 0x%02x, address: 0x%x, size: 0x%x 0x%x\n",
                  write->slave_address, write->address, write->size, write->data[0]);

        fake_wiimote_process_write_request(wiimote, write);
        break;
    }
    case OUTPUT_REPORT_ID_READ_DATA: {
        struct wiimote_output_report_read_data_t *read = (void *)&data[1];
        LOG_DEBUG("  Read data from slave 0x%02x, addrspace: %d, address: 0x%x, size: 0x%x\n",
                  read->slave_address, read->space, read->address, read->size);

        /* Replace any prior pending read request cleanly instead of rejecting with ERROR_CODE_BUSY
         * (which causes Broadway WPAD library to stall). */

        /* Save the request and process it on the next "tick()" call(s) */
        wiimote->read_request.space = read->space;
        wiimote->read_request.slave_address = read->slave_address;
        wiimote->read_request.address = read->address;
        /* A zero size request is just ignored, like on the real wiimote */
        wiimote->read_request.size = read->size;
        wiimote->read_request.ticks = 0;

        /* Send first "read-data reply". If more data needs to be sent,
         * it will happen on the next "tick()" */
        fake_wiimote_process_read_request(wiimote);
        break;
    }
    case OUTPUT_REPORT_ID_SPEAKER_DATA: {
        wiimote->speaker_active_ticks = 40;
        if (size < 2) break;
        u8 len = (data[1] >> 3);
        if (len > 20) len = 20;
        if (len > (u8)(size - 2)) len = (u8)(size - 2);
        u8 pkt[25];
        pkt[0] = OUTPUT_REPORT_ID_SPEAKER_DATA;
        pkt[1] = (u8)((wiimote->index << 4) | (wiimote->speaker_format & 0x0F));
        pkt[2] = wiimote->speaker_rate_khz;
        pkt[3] = wiimote->speaker_volume;
        pkt[4] = len;
        if (len > 0)
            memcpy(&pkt[5], &data[2], len);
        input_device_send_speaker_data(wiimote->input_device, pkt, 5 + len);
        break;
    }
    case OUTPUT_REPORT_ID_SPEAKER_MUTE: {
        bool mute = (data[1] & 0x04) != 0;
        wiimote->speaker_muted = mute;
        if (data[1] & 0x02)
            wiimote_send_ack(wiimote, OUTPUT_REPORT_ID_SPEAKER_MUTE, ERROR_CODE_SUCCESS);
        break;
    }
    case OUTPUT_REPORT_ID_IR_ENABLE2: {
        if (data[1] & 0x02)
            wiimote_send_ack(wiimote, OUTPUT_REPORT_ID_IR_ENABLE2, ERROR_CODE_SUCCESS);
        break;
    }
    default:
        LOG_DEBUG("Unhandled output report: 0x%x\n", data[0]);
        break;
    }
}

void fake_wiimote_handle_acl_data_out_request_from_host(fake_wiimote_t *wiimote,
                                                        const hci_acldata_hdr_t *acl)
{
    const l2cap_hdr_t *header;
    u16 dcid, length;
    const u8 *payload;

    /* Increase the number of completed HCI ACL Data packets */
    wiimote->num_completed_acl_data_packets++;

    /* L2CAP header */
    header = (const void *)((u8 *)acl + sizeof(hci_acldata_hdr_t));
    length = le16toh(header->length);
    dcid = le16toh(header->dcid);
    payload = (u8 *)header + sizeof(l2cap_hdr_t);

    LOG_DEBUG(" Fake Wiimote ACL OUT: dcid: 0x%x, len: 0x%x\n", dcid, length);

    if (dcid == L2CAP_SIGNAL_CID) {
        handle_l2cap_signal_channel_request(wiimote, payload, length);
    } else {
        l2cap_channel_info_t *info = get_channel_info(wiimote, dcid);
        if (info) {
            switch (info->psm) {
            case L2CAP_PSM_SDP:
                /* TODO */
                LOG_DEBUG("  PSM HID SDP\n");
                break;
            case L2CAP_PSM_HID_CNTL:
            case L2CAP_PSM_HID_INTR:
                if (length > 1 && (payload[0] == ((HID_TYPE_SET_REPORT << 4) | HID_PARAM_OUTPUT) ||
                                   payload[0] == ((HID_TYPE_DATA << 4) | HID_PARAM_OUTPUT))) {
                    handle_hid_intr_data_output(wiimote, &payload[1], length - 1);
                } else if (length > 0 && (payload[0] & 0xF0) == 0x10) {
                    handle_hid_intr_data_output(wiimote, payload, length);
                }
                break;
            }
        } else {
            LOG_DEBUG("Received L2CAP packet to unknown channel: 0x%x\n", dcid);
        }
    }
}
