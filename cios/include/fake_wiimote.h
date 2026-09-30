#ifndef FAKE_WIIMOTE_H
#define FAKE_WIIMOTE_H

#include "hci.h"
#include "input_device.h"
#include "types.h"
#include "wiimote.h"
#include "wiimote_crypto.h"

#define MAX_FAKE_WIIMOTES 2

typedef enum {
    BASEBAND_STATE_INACTIVE,
    BASEBAND_STATE_REQUEST_CONNECTION,
    BASEBAND_STATE_COMPLETE
} baseband_state_e;

typedef enum {
    ACL_STATE_INACTIVE,
    ACL_STATE_LINKING
} acl_state_e;

typedef enum {
    L2CAP_CHANNEL_STATE_INACTIVE,
    L2CAP_CHANNEL_STATE_CONFIG_PEND,
    L2CAP_CHANNEL_STATE_COMPLETE
} l2cap_channel_state_e;

typedef struct {
    bool valid;
    l2cap_channel_state_e state;
    u16 psm;
    u16 local_cid;
    u16 remote_cid;
    u16 remote_mtu;
} l2cap_channel_info_t;

typedef enum {
    WIIMOTE_ORIENTATION_VERTICAL = 0,
    WIIMOTE_ORIENTATION_AUTO = 1,
    WIIMOTE_ORIENTATION_WHEEL = 2,
} wiimote_orientation_mode_e;

typedef struct fake_wiimote_t {
    bool active;
    u8 index;
    bdaddr_t bdaddr;
    /* Bluetooth connection state */
    u16 hci_con_handle;
    /* Ticks left before re-sending a Connection Request the console never
     * answered. Lives in the padding after hci_con_handle (free). */
    u16 con_req_retry;
    u16 discon_ticks;
    baseband_state_e baseband_state;
    acl_state_e acl_state;
    l2cap_channel_info_t psm_sdp_chn;
    l2cap_channel_info_t psm_hid_cntl_chn;
    l2cap_channel_info_t psm_hid_intr_chn;
    u32 num_completed_acl_data_packets;
    /* Associated input device with this fake Wiimote */
    input_device_t *input_device;
    /* Reporting mode */
    u8 reporting_mode;
    bool reporting_continuous;
    /* Status */
    struct {
        u8 leds : 4;
        u8 ir : 1;
        u8 speaker : 1;
    } status;
    /* Buttons */
    u16 buttons;
    bool input_dirty;
    /* Accelerometer */
    u16 acc_x, acc_y, acc_z;
    u8 orientation_mode;
    bool is_wheel_mode;
    /* Rumble */
    bool rumble_on;
    /* IR camera */
    struct wiimote_ir_camera_registers_t ir_regs;
    struct ir_dot_t ir_dots[2];
    u8 ir_valid_dots;
    /* Extension */
    struct wiimote_extension_registers_t extension_regs;
    struct wiimote_encryption_key_t extension_key;
    bool extension_key_dirty;
    enum wiimote_ext_e cur_extension;
    enum wiimote_ext_e new_extension;
    /* Wii Motion Plus */
    bool mp_active;
    u8 mp_mode;
    u8 mp_challenge_state;
    u8 mp_challenge_type;
    bool mp_report_toggle;
    bool mp_pending_status;
    s16 gyro_roll, gyro_yaw, gyro_pitch;
    bool mp_slow_pitch, mp_slow_roll, mp_slow_yaw;
    /* Speaker */
    bool speaker_enabled;
    bool speaker_muted;
    u8 speaker_format;
    u8 speaker_volume;
    u8 speaker_rate_khz;
    u8 speaker_active_ticks;
    /* EEPROM */
    union wiimote_usable_eeprom_data_t eeprom;
    /* Battery */
    u8 battery;
    bool battery_low;
    /* Current in-progress "memory read request" */
    struct {
        u8 space;
        u8 slave_address;
        u16 address;
        u16 size;
        u8 ticks;
    } read_request;
} fake_wiimote_t;

/** Used by the Fake Wiimote manager **/
void fake_wiimote_init(fake_wiimote_t *wiimote, u8 index, const bdaddr_t *bdaddr);
void fake_wiimote_init_state(fake_wiimote_t *wiimote, input_device_t *input_device);
void fake_wiimote_handle_hci_cmd_accept_con(fake_wiimote_t *wiimote, u8 role);
void fake_wiimote_release_input_device(fake_wiimote_t *wiimote);
int fake_wiimote_disconnect(fake_wiimote_t *wiimote);
void fake_wiimote_tick(fake_wiimote_t *wiimote);
void fake_wiimote_handle_acl_data_out_request_from_host(fake_wiimote_t *wiimote,
                                                        const hci_acldata_hdr_t *acl);

/* API for input devices  to receive and report controller events */
void fake_wiimote_set_extension(fake_wiimote_t *wiimote, enum wiimote_ext_e ext);
void fake_wiimote_set_orientation_mode(fake_wiimote_t *wiimote, u8 mode);
void fake_wiimote_report_battery(fake_wiimote_t *wiimote, u8 bat_level, bool charging);
void fake_wiimote_report_input(fake_wiimote_t *wiimote, u16 buttons);
void fake_wiimote_report_accelerometer(fake_wiimote_t *wiimote, s16 acc_x, s16 acc_y, s16 acc_z);
void fake_wiimote_report_gyroscope(fake_wiimote_t *wiimote, s16 gyro_x, s16 gyro_y, s16 gyro_z);
void fake_wiimote_report_ir_dots(fake_wiimote_t *wiimote,
                                 struct ir_dot_t ir_dots[static IR_MAX_DOTS]);
void fake_wiimote_report_input_ext(fake_wiimote_t *wiimote, u16 buttons, const void *ext_data,
                                   u8 ext_size);

/* Helper functions */

static inline bool fake_wiimote_is_connected(const fake_wiimote_t *wiimote)
{
    return wiimote->active && (wiimote->baseband_state == BASEBAND_STATE_COMPLETE);
}

#endif
