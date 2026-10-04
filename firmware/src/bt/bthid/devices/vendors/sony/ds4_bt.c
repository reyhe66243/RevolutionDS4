// ds4_bt.c - Sony DualShock 4 Bluetooth Driver
// Handles DS4 controllers over Bluetooth
//
// Report format reference: https://www.psdevwiki.com/ps4/DS4-BT
// BT reports have 2-byte offset compared to USB (report ID 0x11 vs 0x01)

#include "ds4_bt.h"
#include "ds4_audio.h"
#include "bt/bthid/bthid.h"
#include "bt/transport/bt_transport.h"
#include "core/input_event.h"
#include "core/router/router.h"
#include "core/buttons.h"
#include "core/services/players/manager.h"
#include "core/services/players/feedback.h"
#include "platform/platform.h"
#include "pico/time.h"
#include <string.h>
#include <stdio.h>

// Gyro bias auto-zero tuning. The DS4 gyro reports ~16.38 LSB per deg/s.
// "Still" = every axis within ~1.5 deg/s of the tracked zero for at least a
// second; only then the zero offset is slowly adapted (~1s time constant) and
// subtracted from the reported rate. The bias is clamped to ~49 deg/s so a
// confused tracker can never remove real movement.
#define GYRO_STILL_THRESHOLD 24
#define GYRO_STILL_TIME_MS   1000
#define GYRO_BIAS_MAX_Q4     (800 << 4)

// Player LED colors (RGB values)
static const uint8_t PLAYER_COLORS[][3] = {
    {  0,   0,  64 },   // Player 1: Blue
    { 64,   0,   0 },   // Player 2: Red
    {  0,  64,   0 },   // Player 3: Green
    { 64,   0,  64 },   // Player 4: Pink/Fuchsia
};

// btstack_host.c: direct L2CAP interrupt-channel send
extern bool btstack_classic_send_interrupt_raw(uint8_t conn_index,
                                               const uint8_t* data, uint16_t len);
extern bool btstack_classic_can_send_interrupt(uint8_t conn_index);
// btstack_host.c: feature report request (response arrives via bt_on_get_report)
extern bool btstack_classic_send_get_report(uint8_t conn_index, uint8_t report_type,
                                            uint8_t report_id);

// Factory calibration report (Bluetooth). 41 bytes: report ID, 36 data bytes
// and a CRC-32 (seed 0xA3) in the last 4 bytes. Same fields as the USB report
// 0x02 but with the plus/minus values interleaved per axis.
#define DS4_CALIB_REPORT_ID   0x05
#define DS4_CALIB_REPORT_LEN  41

// Calibration fetch timing (feature report 0x05): first request shortly after
// activation, quick retries while the link settles, then quiet background
// retries. The outcome is shown as lightbar blinks: 2 green = factory
// calibration applied, 3 red = no response, 4 yellow = response rejected.
#define DS4_CALIB_FIRST_MS       800
#define DS4_CALIB_RETRY_MS       1200
#define DS4_CALIB_RETRY_ATTEMPTS 5
#define DS4_CALIB_QUIET_MS       10000
#define DS4_CALIB_MAX_ATTEMPTS   26

// ============================================================================
// DS4 REPORT STRUCTURE (same as USB, but BT has 2-byte header offset)
// ============================================================================

typedef struct __attribute__((packed)) {
    uint8_t x, y, z, rz;    // Joysticks (0-255, centered at 128)

    struct {
        uint8_t dpad     : 4;   // Hat: 0=N, 1=NE, 2=E, 3=SE, 4=S, 5=SW, 6=W, 7=NW, 8=released
        uint8_t square   : 1;
        uint8_t cross    : 1;
        uint8_t circle   : 1;
        uint8_t triangle : 1;
    };

    struct {
        uint8_t l1     : 1;
        uint8_t r1     : 1;
        uint8_t l2     : 1;
        uint8_t r2     : 1;
        uint8_t share  : 1;
        uint8_t option : 1;
        uint8_t l3     : 1;
        uint8_t r3     : 1;
    };

    struct {
        uint8_t ps      : 1;
        uint8_t tpad    : 1;
        uint8_t counter : 6;
    };

    uint8_t l2_trigger;
    uint8_t r2_trigger;

    // Extended data for motion
    uint16_t timestamp;
    uint8_t sensor_temperature;
    int16_t gyro[3];    // x, y, z
    int16_t accel[3];   // x, y, z
    int8_t   unknown_a[5];
    uint8_t  headset;
    int8_t   unknown_b[2];
    struct { uint8_t tpad_event : 4; uint8_t unknown_c : 4; };
    uint8_t  tpad_counter;
    struct { uint8_t tpad_f1_count : 7; uint8_t tpad_f1_down : 1; };
    uint8_t  tpad_f1_pos[3];
    struct { uint8_t tpad_f2_count : 7; uint8_t tpad_f2_down : 1; };
    uint8_t  tpad_f2_pos[3];
} ds4_input_report_t;

// DS4 BT output report (for rumble/LED)
typedef struct __attribute__((packed)) {
    uint8_t report_id;      // 0x11 for BT
    uint8_t flags1;         // 0x80
    uint8_t flags2;         // 0x00
    uint8_t flags3;         // 0xFF (enable rumble+LED)

    uint8_t reserved1[2];

    uint8_t rumble_right;   // High frequency
    uint8_t rumble_left;    // Low frequency

    uint8_t led_red;
    uint8_t led_green;
    uint8_t led_blue;

    uint8_t flash_on;       // LED flash on duration
    uint8_t flash_off;      // LED flash off duration

    uint8_t reserved2[8];

    // Total: 23 bytes for basic output
} ds4_bt_output_report_t;

// ============================================================================
// DRIVER DATA
// ============================================================================

typedef struct {
    input_event_t event;
    bool initialized;
    bool sixaxis_enabled;
    uint8_t activation_state;
    uint32_t activation_time;

    // Current feedback state (for change detection)
    uint8_t rumble_left;
    uint8_t rumble_right;
    uint8_t led_r, led_g, led_b;
    uint8_t speaker_volume;

    // Touchpad swipe tracking
    uint16_t tpad_last_pos;
    bool tpad_dragging;

    // Audio streaming pacing
    uint64_t next_audio_us;
    uint64_t last_input_report_us;

    // Debounce counters for Share (S1), Option (S2), PS (A1), and Touchpad Click (A2)
    uint8_t share_press_cnt;
    uint8_t option_press_cnt;
    uint8_t ps_press_cnt;
    uint8_t tpad_press_cnt;
    uint8_t tpad_touch_cnt;
    uint32_t prev_raw_buttons;

    // Output report retry timing
    uint32_t last_output_attempt_ms;

    // Audio send retry tracking
    uint8_t audio_retry_count;

    // Gyro bias auto-zero (MotionPlus-style). The DS4 reports raw gyro rates
    // with a small zero offset (bias) that drifts with temperature; integrating
    // it makes the aim drift until the game recenters. While the controller is
    // still we track that offset and subtract it, so the reported rate is
    // unbiased - the same thing the MotionPlus does internally.
    int32_t gyro_bias_q4[3];    // bias in 1/16 LSB units
    uint16_t gyro_still_ms;     // accumulated still time
    bool gyro_bias_init;        // bias seeded from first sample

    // Factory calibration (BT feature report 0x05), fetched once per
    // connection. Scales normalize each axis to the nominal sensitivity
    // (16.384 LSB per deg/s gyro, 8192 LSB per g accel) in Q16 fixed point
    // (65536 = 1.0). With calib_valid false the raw values pass through
    // untouched, exactly as before this feature existed.
    int32_t gyro_scale_q16[3];
    int32_t accel_scale_q16[3];
    bool calib_valid;
    uint8_t calib_attempts;      // GET_REPORT attempts (stops at DS4_CALIB_MAX_ATTEMPTS)
    uint32_t calib_next_ms;      // 0 = not armed; else when to (re)issue the request
    uint8_t calib_led_result;    // 0=none, 1=applied, 2=no response, 3=rejected
    uint32_t calib_led_start_ms; // blink pattern start

    // Per-device output report buffer (was global static).
    // BTstack stores a pointer for deferred L2CAP send, so must remain valid
    // until CAN_SEND_NOW callback fires. Per-device avoids multi-DS4 stomping.
    uint8_t output_report_buf[79];

    // Per-device audio report buffer (was stack-local 335 bytes).
    // Moved here to avoid stack pressure on the BT task.
    uint8_t audio_report_buf[335];
} ds4_bt_data_t;

static ds4_bt_data_t ds4_data[BTHID_MAX_DEVICES];

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================


static inline uint32_t ds4_crc32_byte(uint32_t crc, uint8_t byte)
{
    crc ^= byte;
    for (int j = 0; j < 8; j++) {
        crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
    }
    return crc;
}

// Read a little-endian int16 from raw bytes (alignment-safe, no struct dependency)
static inline int16_t ds4_read_le16(const uint8_t* p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

// Reset factory-calibration state to nominal scaling (connect/disconnect).
static void ds4_reset_calibration(ds4_bt_data_t* ds4)
{
    for (int i = 0; i < 3; i++) {
        ds4->gyro_scale_q16[i] = 65536;
        ds4->accel_scale_q16[i] = 65536;
    }
    ds4->calib_valid = false;
    ds4->calib_attempts = 0;
    ds4->calib_next_ms = 0;
    ds4->calib_led_result = 0;
    ds4->calib_led_start_ms = 0;
}

static bool ds4_send_output(bthid_device_t* device, uint8_t rumble_left, uint8_t rumble_right,
                            uint8_t r, uint8_t g, uint8_t b)
{
    ds4_bt_data_t* ds4 = (ds4_bt_data_t*)device->driver_data;
    if (!ds4) return false;

    // DS4 BT output report (Report ID 0x11, 78 bytes).
    // Transmit over L2CAP interrupt channel (0xA2) via btstack_classic_send_interrupt_raw.
    // Unlike control channel SET_REPORT (0x52) which waits for a HID HANDSHAKE that can
    // drop over RF and permanently deadlock BTstack in HID_HOST_W4_SET_REPORT_RESPONSE,
    // interrupt channel DATA|OUTPUT (0xA2) is unacknowledged and never blocks or fails
    // with COMMAND_DISALLOWED. This guarantees rumble-off and keepalive are NEVER blocked!
    uint8_t* buf = ds4->output_report_buf;
    memset(buf, 0, sizeof(ds4->output_report_buf));

    buf[0] = 0xA2;  // HID DATA | Output for interrupt channel
    buf[1] = 0x11;  // Report ID
    buf[2] = 0xC4;  // Flags (BT): DS4_OUTPUT_HWCTL_HID (0x80) | DS4_OUTPUT_HWCTL_CRC32 (0x40) | 4ms poll (0x04)
    buf[3] = 0x00;
    buf[4] = 0x07;  // Enable rumble right (0x01), rumble left (0x02), LED (0x04). Do NOT enable speaker vol (0x80).

    // Balance motors and lightbar to eliminate harsh saturation / overdrive and battery brownout
    buf[7]  = (uint8_t)(((uint16_t)rumble_right * 140) / 255);  // High freq motor
    buf[8]  = (uint8_t)(((uint16_t)rumble_left  * 110) / 255);  // Low freq heavy motor
    buf[9]  = (uint8_t)(((uint16_t)r * 180) / 255);
    buf[10] = (uint8_t)(((uint16_t)g * 180) / 255);
    buf[11] = (uint8_t)(((uint16_t)b * 180) / 255);

    // Calculate CRC-32 over report body (Report ID 0x11 + 73 data bytes = 74 bytes)
    // with Sony BT output report seed 0xA2 (PS_OUTPUT_CRC32_SEED).
    uint32_t crc = ds4_crc32_byte(0xFFFFFFFF, 0xA2);
    for (int i = 1; i <= 74; i++) {
        crc = ds4_crc32_byte(crc, buf[i]);
    }
    crc = ~crc;

    buf[75] = (uint8_t)(crc & 0xFF);
    buf[76] = (uint8_t)((crc >> 8) & 0xFF);
    buf[77] = (uint8_t)((crc >> 16) & 0xFF);
    buf[78] = (uint8_t)((crc >> 24) & 0xFF);

    bool ok = btstack_classic_send_interrupt_raw(device->conn_index, buf, 79);
    if (!ok) {
        return false;
    }

    ds4->rumble_left = rumble_left;
    ds4->rumble_right = rumble_right;
    ds4->led_r = r;
    ds4->led_g = g;
    ds4->led_b = b;

    return true;
}

// ============================================================================
// DRIVER IMPLEMENTATION
// ============================================================================

static bool ds4_match(const char* device_name, const uint8_t* class_of_device,
                      uint16_t vendor_id, uint16_t product_id)
{
    (void)class_of_device;

    // VID/PID match (highest priority) - Sony vendor ID = 0x054C
    // DS4 v1 = 0x05C4, DS4 v2 (Slim) = 0x09CC
    if (vendor_id == 0x054C && (product_id == 0x05C4 || product_id == 0x09CC)) {
        return true;
    }
    if (vendor_id == 0x054C && (product_id == 0x0CE6 || product_id == 0x0DF2)) {
        return false;
    }

    // Name-based match (fallback if SDP query didn't return VID/PID)
    if (device_name) {
        // Match known DS4 device names
        // Note: the DS4 reports its name as just "Wireless Controller" (no "Sony" prefix)
        if (strstr(device_name, "Wireless Controller") != NULL) {
            return true;
        }
        if (strstr(device_name, "DUALSHOCK 4") != NULL) {
            return true;
        }
    }

    // In dedicated RevolutionDS4 adapter: if connected via BT and VID/PID are not known yet,
    // match DS4 immediately so device initialization and report 0x11 activation are not delayed
    // or skipped (which leaves the DS4 stuck in unconfigured white light).
    if ((vendor_id == 0 && product_id == 0) || (vendor_id == 0x054C)) {
        return true;
    }

    return false;
}

static bool ds4_init(bthid_device_t* device)
{
    printf("[DS4_BT] Init for device: %s\n", device->name);

    // Find free data slot
    for (int i = 0; i < BTHID_MAX_DEVICES; i++) {
        if (!ds4_data[i].initialized) {
            init_input_event(&ds4_data[i].event);
            ds4_data[i].initialized = true;
            ds4_data[i].sixaxis_enabled = false;
            ds4_data[i].activation_state = 0;
            ds4_data[i].activation_time = 0;
            ds4_data[i].rumble_left = 0;
            ds4_data[i].rumble_right = 0;
            ds4_data[i].led_r = 0;
            ds4_data[i].led_g = 0;
            ds4_data[i].led_b = 64;  // Default blue
            ds4_data[i].tpad_last_pos = 0;
            ds4_data[i].tpad_dragging = false;
            ds4_data[i].share_press_cnt = 0;
            ds4_data[i].option_press_cnt = 0;
            ds4_data[i].ps_press_cnt = 0;
            ds4_data[i].tpad_press_cnt = 0;
            ds4_data[i].tpad_touch_cnt = 0;
            ds4_data[i].prev_raw_buttons = 0;
            ds4_data[i].next_audio_us = 0;
            ds4_data[i].last_input_report_us = 0;
            ds4_data[i].last_output_attempt_ms = 0;
            memset(ds4_data[i].gyro_bias_q4, 0, sizeof(ds4_data[i].gyro_bias_q4));
            ds4_data[i].gyro_still_ms = 0;
            ds4_data[i].gyro_bias_init = false;
            ds4_reset_calibration(&ds4_data[i]);

            ds4_data[i].event.type = INPUT_TYPE_GAMEPAD;
            ds4_data[i].event.transport = INPUT_TRANSPORT_BT_CLASSIC;
            ds4_data[i].event.dev_addr = device->conn_index;
            ds4_data[i].event.instance = 0;
            ds4_data[i].event.button_count = 14;
            ds4_data[i].event.has_motion = true;  // DS4 has motion

            device->driver_data = &ds4_data[i];

            ds4_audio_init();

            // Activation happens in task (state machine with delays)
            return true;
        }
    }

    return false;
}


static bool ds4_check_report_crc(ds4_bt_data_t* ds4, const uint8_t* data, uint16_t len)
{
    if (!data || len < 10) return false;

    // Report 0x01: Basic input report (USB or initial BT connection, no CRC).
    // Once enhanced mode (0x11) is active, a genuine DS4 on Bluetooth NEVER sends 0x01 again.
    // Reject 0x01 if sixaxis_enabled is true so non-gamepad uplink audio or corrupted
    // fragments cannot bypass CRC-32 validation.
    if (data[0] == 0x01) {
        if (ds4 && ds4->sixaxis_enabled) {
            return false;
        }
        return true;
    }

    // Only the standard enhanced report (0x11) carries a trustworthy gamepad
    // state. While the controller has microphone / uplink audio queued it
    // emits 0x12..0x19 packets whose payload is audio, NOT controller state at
    // these offsets - verified on hardware: parsing them produced phantom
    // button presses (Home menu) and spinning motion. They must be dropped for
    // input. The root fix for that uplink flood is on the output side:
    // audio_control must stay 0xA0 so the microphone input path is not selected.
    bool is_known_report = (data[0] == 0x11);
    if (!is_known_report) {
        return false;
    }

    // All valid Sony DS4 Bluetooth reports MUST have:
    // - Length >= 78 bytes (the full gamepad state size)
    // - Bit 6 (0x40) EnableCRC set
    // - Bit 7 (0x80) EnableHID set (contains gamepad state)
    if (len < 78 || !(data[1] & 0x40) || !(data[1] & 0x80)) {
        return false;
    }

    uint32_t crc = ds4_crc32_byte(0xFFFFFFFF, 0xA1); // Seed 0xA1 (HID DATA | INPUT)
    for (uint16_t i = 0; i < len - 4; i++) {
        crc = ds4_crc32_byte(crc, data[i]);
    }
    crc = ~crc;

    uint32_t expected = (uint32_t)data[len - 4] |
                        ((uint32_t)data[len - 3] << 8) |
                        ((uint32_t)data[len - 2] << 16) |
                        ((uint32_t)data[len - 1] << 24);

    return crc == expected;
}

// ============================================================================
// FACTORY CALIBRATION (Bluetooth feature report 0x05)
// ============================================================================

// Parse the factory calibration and install per-axis scale factors so every
// controller reports the nominal sensitivity (16.384 LSB per deg/s gyro, 8192
// LSB per g accel). The measured factory spread is only a few percent, but it
// is free accuracy once the report is available. Anything suspicious rejects
// the whole calibration and the nominal values stay in place, so a clone or a
// firmware quirk can never degrade motion.
static void ds4_parse_calibration(ds4_bt_data_t* ds4, const uint8_t* data, uint16_t len)
{
    // The BTstack event passes the report with its ID as the first byte.
    if (len < DS4_CALIB_REPORT_LEN || data[0] != DS4_CALIB_REPORT_ID) {
        printf("[DS4_BT] Calibration report unexpected (len=%d id=0x%02X)\n",
               len, len > 0 ? data[0] : 0);
        return;
    }

    // CRC-32 over the report (ID included), seed 0xA3 for feature reports.
    // Same algorithm that validates the 0x11 input reports on this link.
    uint32_t crc = ds4_crc32_byte(0xFFFFFFFF, 0xA3);
    for (uint16_t i = 0; i < len - 4; i++) {
        crc = ds4_crc32_byte(crc, data[i]);
    }
    crc = ~crc;
    uint32_t expected = (uint32_t)data[len - 4] |
                        ((uint32_t)data[len - 3] << 8) |
                        ((uint32_t)data[len - 2] << 16) |
                        ((uint32_t)data[len - 1] << 24);
    if (crc != expected) {
        printf("[DS4_BT] Calibration CRC mismatch, keeping nominal\n");
        return;
    }

    // Bluetooth field order differs from USB: plus/minus grouped by axis.
    int16_t gyro_bias[3]  = { ds4_read_le16(data + 1),  ds4_read_le16(data + 3),
                              ds4_read_le16(data + 5) };
    int16_t gyro_plus[3]  = { ds4_read_le16(data + 7),  ds4_read_le16(data + 9),
                              ds4_read_le16(data + 11) };
    int16_t gyro_minus[3] = { ds4_read_le16(data + 13), ds4_read_le16(data + 15),
                              ds4_read_le16(data + 17) };
    int16_t speed_plus  = ds4_read_le16(data + 19);
    int16_t speed_minus = ds4_read_le16(data + 21);
    int16_t acc_plus[3]  = { ds4_read_le16(data + 23), ds4_read_le16(data + 27),
                             ds4_read_le16(data + 31) };
    int16_t acc_minus[3] = { ds4_read_le16(data + 25), ds4_read_le16(data + 29),
                             ds4_read_le16(data + 33) };

    int32_t speed_2x = (int32_t)speed_plus + speed_minus;
    if (speed_2x < 200) {
        printf("[DS4_BT] Calibration: bad reference speed (%d/%d)\n",
               speed_plus, speed_minus);
        return;
    }

    int32_t gyro_scale[3];
    int32_t accel_scale[3];
    for (int i = 0; i < 3; i++) {
        // Gyro slope = span / speed_2x LSB per deg/s.
        // scale_q16 = (16384/1000) / slope * 65536, i.e. nominal / measured.
        int32_t span = (int32_t)gyro_plus[i] - gyro_minus[i];
        if (span <= 0 || gyro_bias[i] > 2000 || gyro_bias[i] < -2000) {
            printf("[DS4_BT] Calibration: insane gyro axis %d, keeping nominal\n", i);
            return;
        }
        int32_t s = (int32_t)(((int64_t)16384 * speed_2x * 65536) /
                              ((int64_t)1000 * span));
        if (s < 52429 || s > 78643) {  // outside +/-20%: reject
            printf("[DS4_BT] Calibration: gyro axis %d scale out of range\n", i);
            return;
        }
        gyro_scale[i] = s;

        // Accel slope = span/2 LSB per g. scale_q16 = 8192 / slope * 65536.
        span = (int32_t)acc_plus[i] - acc_minus[i];
        if (span <= 0) {
            printf("[DS4_BT] Calibration: insane accel axis %d, keeping nominal\n", i);
            return;
        }
        s = (int32_t)(((int64_t)16384 * 65536) / span);
        if (s < 52429 || s > 78643) {  // outside +/-20%: reject
            printf("[DS4_BT] Calibration: accel axis %d scale out of range\n", i);
            return;
        }
        accel_scale[i] = s;
    }

    for (int i = 0; i < 3; i++) {
        ds4->gyro_scale_q16[i] = gyro_scale[i];
        ds4->accel_scale_q16[i] = accel_scale[i];
        // Seed the bias tracker with the factory zero-rate offset; the live
        // tracker keeps refining it afterwards (temperature drift).
        ds4->gyro_bias_q4[i] = (int32_t)gyro_bias[i] << 4;
    }
    ds4->gyro_still_ms = 0;
    ds4->gyro_bias_init = true;
    ds4->calib_valid = true;

    printf("[DS4_BT] Factory calibration applied: gyro Q16 %d/%d/%d accel %d/%d/%d\n",
           (int)gyro_scale[0], (int)gyro_scale[1], (int)gyro_scale[2],
           (int)accel_scale[0], (int)accel_scale[1], (int)accel_scale[2]);
}

// Asynchronous GET_REPORT response (control channel). Only the factory
// calibration is requested; anything else is ignored.
static void ds4_get_report(bthid_device_t* device, const uint8_t* data, uint16_t len)
{
    ds4_bt_data_t* ds4 = (ds4_bt_data_t*)device->driver_data;
    if (!ds4) return;

    if (data[0] == DS4_CALIB_REPORT_ID) {
        ds4_parse_calibration(ds4, data, len);
    }

    // Show the outcome once: green when applied, yellow when a response
    // arrived but could not be used. Quiet retries keep running meanwhile.
    if (ds4->calib_led_result == 0) {
        ds4->calib_led_result = ds4->calib_valid ? 1 : 3;
        ds4->calib_led_start_ms = platform_time_ms();
    }
}

static bool ds4_process_debug_done = false;

static void ds4_process_report(bthid_device_t* device, const uint8_t* data, uint16_t len)
{
    ds4_bt_data_t* ds4 = (ds4_bt_data_t*)device->driver_data;
    if (!ds4) {
        return;
    }

    // Mark link activity before validating the report: the audio scheduler
    // uses this timestamp to transmit right after any controller uplink packet
    // (state or audio), so it must also count packets we end up discarding.
    ds4->last_input_report_us = time_us_64();

    // Validate CRC-32 and drop any corrupted or non-gamepad reports immediately
    if (!ds4_check_report_crc(ds4, data, len)) {
        return;
    }

    // Debug: print first report received
    if (!ds4_process_debug_done) {
        printf("[DS4_BT] Process report: len=%d, data[0]=0x%02X\n", len, len > 0 ? data[0] : 0);
        ds4_process_debug_done = true;
    }

    // BT reports:
    // 0x01 = Basic report (initial connection only, no motion/touchpad)
    // 0x11 = Full enhanced report (standard 78 bytes)
    // 0x12..0x19 = uplink audio packets (NOT gamepad state) - dropped in the
    //              CRC filter above

    const uint8_t* report_data = NULL;
    uint16_t report_len = 0;

    switch (data[0]) {
        case 0x11: {
            // Standard enhanced report (78 bytes): report ID, flags, one
            // reserved byte, then the full controller state.
            // data[1] bit 7 (0x80) = EnableHID: when clear, the packet is
            // audio-only and contains no gamepad state - drop it.
            if (len < 12 || !(data[1] & 0x80)) {
                return;
            }
            report_data = data + 3;
            report_len = len - 3;
            ds4->sixaxis_enabled = true;
            break;
        }
        case 0x01: {
            // Basic report - skip 1 byte (report ID)
            if (len < 10) {
                return;
            }
            report_data = data + 1;
            report_len = len - 1;
            ds4->sixaxis_enabled = false;
            break;
        }
        default:
            // Reject unknown / non-gamepad reports
            return;
    }

    if (data[0] != 0x01 && report_len < sizeof(ds4_input_report_t)) {
        return;
    }

    const ds4_input_report_t* rpt = (const ds4_input_report_t*)report_data;

    // Filter: D-pad hat switch validity.
    // In any valid DS4 report, dpad is 0..7 (directions) or 8 (released).
    // Values 9..15 are invalid and indicate corrupted data.
    if (rpt->dpad > 8) {
        return;
    }

    // Parse D-pad (hat format)
    bool dpad_up    = (rpt->dpad == 0 || rpt->dpad == 1 || rpt->dpad == 7);
    bool dpad_right = (rpt->dpad >= 1 && rpt->dpad <= 3);
    bool dpad_down  = (rpt->dpad >= 3 && rpt->dpad <= 5);
    bool dpad_left  = (rpt->dpad >= 5 && rpt->dpad <= 7);

    // Build raw button state (inverted: 0 = pressed in USBR convention)
    uint32_t raw_buttons = 0x00000000;  // All released (active-high)

    if (dpad_up)       raw_buttons |= PAD_BUTTON_DU;
    if (dpad_down)     raw_buttons |= PAD_BUTTON_DD;
    if (dpad_left)     raw_buttons |= PAD_BUTTON_DL;
    if (dpad_right)    raw_buttons |= PAD_BUTTON_DR;
    if (rpt->cross)    raw_buttons |= PAD_BUTTON_B1;
    if (rpt->circle)   raw_buttons |= PAD_BUTTON_B2;
    if (rpt->square)   raw_buttons |= PAD_BUTTON_B3;
    if (rpt->triangle) raw_buttons |= PAD_BUTTON_B4;
    if (rpt->l1)       raw_buttons |= PAD_BUTTON_L1;
    if (rpt->r1)       raw_buttons |= PAD_BUTTON_R1;
    if (rpt->l2)       raw_buttons |= PAD_BUTTON_L2;
    if (rpt->r2)       raw_buttons |= PAD_BUTTON_R2;
    if (rpt->l3)       raw_buttons |= PAD_BUTTON_L3;
    if (rpt->r3)       raw_buttons |= PAD_BUTTON_R3;

    int player_idx = find_player_index(ds4->event.dev_addr, ds4->event.instance);
    if (player_idx < 0) player_idx = 0;
    bool audio_active = ds4_audio_is_player_active((uint8_t)player_idx) ||
                        (platform_time_ms() - ds4_audio_get_player_last_play_time_ms((uint8_t)player_idx) < 600);
    bool rumble_active = (ds4->rumble_left > 0 || ds4->rumble_right > 0);

    // Debounce Touchpad click (A2 / Wii Remote A):
    // 2 consecutive Bluetooth frames (~8ms) to eliminate mechanical switch bounce
    // and acoustic resonance from speaker vibrations, without noticeable human delay.
    if (rpt->tpad) {
        if (ds4->tpad_press_cnt < 255) ds4->tpad_press_cnt++;
        if (ds4->tpad_press_cnt >= 2) {
            raw_buttons |= PAD_BUTTON_A2;
        }
    } else {
        ds4->tpad_press_cnt = 0;
    }

    // Debounce Option (S2 / Plus), Share (S1 / Minus), and PS (A1 / Home):
    // Under heavy speaker audio or rumble motor resonance, mechanical micro-switch contacts
    // can chatter. Require 5 consecutive frames (~20-25ms) during audio/rumble,
    // or 3 frames (~12-15ms) normally. This filters all vibration bounces while feeling instantaneous.
    uint8_t menu_min_frames = (audio_active || rumble_active) ? 5 : 3;

    if (rpt->option) {
        if (ds4->option_press_cnt < 255) ds4->option_press_cnt++;
        if (ds4->option_press_cnt >= menu_min_frames) {
            raw_buttons |= PAD_BUTTON_S2;
        }
    } else {
        ds4->option_press_cnt = 0;
    }

    if (rpt->share) {
        if (ds4->share_press_cnt < 255) ds4->share_press_cnt++;
        if (ds4->share_press_cnt >= menu_min_frames) {
            raw_buttons |= PAD_BUTTON_S1;
        }
    } else {
        ds4->share_press_cnt = 0;
    }

    if (rpt->ps) {
        if (ds4->ps_press_cnt < 255) ds4->ps_press_cnt++;
        if (ds4->ps_press_cnt >= menu_min_frames) {
            raw_buttons |= PAD_BUTTON_A1;
        }
    } else {
        ds4->ps_press_cnt = 0;
    }

    // Universal 2-frame glitch filter (~8ms) for all buttons:
    // A button must be continuously asserted across at least 2 consecutive frames
    // to eliminate single-packet spikes, accidental micro-brushes, or contact chatter.
    // ~8ms is half of a 60fps frame, feeling 100% instantaneous to human reflex.
    // Release is instantaneous (0ms delay) the exact moment the button is released.
    uint32_t buttons = raw_buttons & ds4->prev_raw_buttons;
    ds4->prev_raw_buttons = raw_buttons;

    // Update event
    ds4->event.buttons = buttons;

    // Analog sticks (HID convention: 0=up, 255=down)
    ds4->event.analog[ANALOG_LX] = rpt->x;
    ds4->event.analog[ANALOG_LY] = rpt->y;
    ds4->event.analog[ANALOG_RX] = rpt->z;
    ds4->event.analog[ANALOG_RY] = rpt->rz;

    // Triggers
    ds4->event.analog[ANALOG_L2] = rpt->l2_trigger;
    ds4->event.analog[ANALOG_R2] = rpt->r2_trigger;

    // Motion data (DS4 has full 3-axis gyro and accel), from the standard
    // enhanced report only. Direct 1:1 readings, no filtering: the audio-time
    // glitches were USB contention on the console side, not sensor corruption.
    bool is_state_report = (data[0] == 0x11);

    if (is_state_report && ds4->sixaxis_enabled && report_len >= sizeof(ds4_input_report_t)) {
        ds4->event.has_motion = true;

        // Read gyro/accel by explicit byte offsets (alignment-safe, struct-independent).
        // In report_data (= data+3 for BT enhanced), offsets are:
        //   gyro:  [12..17]  accel: [18..23]  (each axis = 2 bytes LE)
        int16_t g_in[3];
        g_in[0] = ds4_read_le16(report_data + 12);
        g_in[1] = ds4_read_le16(report_data + 14);
        g_in[2] = ds4_read_le16(report_data + 16);

        ds4->event.accel[0] = ds4_read_le16(report_data + 18);
        ds4->event.accel[1] = ds4_read_le16(report_data + 20);
        ds4->event.accel[2] = ds4_read_le16(report_data + 22);

        // Factory calibration: normalize each axis to the nominal sensitivity
        // (gyro 16.384 LSB/deg/s, accel 8192 LSB/g) so the console-side
        // MotionPlus conversion is exact. No-op until the calibration arrives.
        if (ds4->calib_valid) {
            for (int a = 0; a < 3; a++) {
                int32_t av = (int32_t)ds4->event.accel[a];
                av = (int32_t)(((int64_t)av * ds4->accel_scale_q16[a]) >> 16);
                if (av > 32767) av = 32767;
                else if (av < -32768) av = -32768;
                ds4->event.accel[a] = (int16_t)av;
            }
        }
        // Gyro bias auto-zero (same idea as the MotionPlus internal
        // calibration): track the zero-rate offset while the controller is
        // truly still and subtract it, so integrating the rate stops drifting.
        // The tracker freezes as soon as real motion appears, adaptation is
        // slow and the bias is clamped, so it can never eat fast movement.
        if (!ds4->gyro_bias_init) {
            for (int i = 0; i < 3; i++) {
                ds4->gyro_bias_q4[i] = (int32_t)g_in[i] << 4;
            }
            ds4->gyro_still_ms = 0;
            ds4->gyro_bias_init = true;
        }

        bool still = true;
        for (int i = 0; i < 3; i++) {
            int32_t d = (int32_t)g_in[i] - ((ds4->gyro_bias_q4[i] + 8) >> 4);
            if (d < 0) d = -d;
            if (d > GYRO_STILL_THRESHOLD) {
                still = false;
                break;
            }
        }

        if (still) {
            if (ds4->gyro_still_ms < 0xFFFF) ds4->gyro_still_ms += 4;  // ~4ms per report
        } else {
            ds4->gyro_still_ms = 0;
        }

        if (ds4->gyro_still_ms >= GYRO_STILL_TIME_MS) {
            for (int i = 0; i < 3; i++) {
                int32_t target = (int32_t)g_in[i] << 4;
                ds4->gyro_bias_q4[i] += (target - ds4->gyro_bias_q4[i]) >> 8;
                if (ds4->gyro_bias_q4[i] > GYRO_BIAS_MAX_Q4)
                    ds4->gyro_bias_q4[i] = GYRO_BIAS_MAX_Q4;
                if (ds4->gyro_bias_q4[i] < -GYRO_BIAS_MAX_Q4)
                    ds4->gyro_bias_q4[i] = -GYRO_BIAS_MAX_Q4;
            }
        }

        for (int i = 0; i < 3; i++) {
            int32_t v = (int32_t)g_in[i] - ((ds4->gyro_bias_q4[i] + 8) >> 4);
            if (ds4->calib_valid) {
                v = (int32_t)(((int64_t)v * ds4->gyro_scale_q16[i]) >> 16);
            }
            if (v > 32767) v = 32767;
            else if (v < -32768) v = -32768;
            ds4->event.gyro[i] = (int16_t)v;
        }
    } else if (!ds4->sixaxis_enabled) {
        ds4->event.has_motion = false;
    }

    // Battery: status[0] at report_data[29] — bits 0-3 = level, bit 4 = cable connected
    // Level interpretation differs based on cable state (per Linux kernel hid-playstation.c)
    if (report_len > 29) {
        uint8_t raw = report_data[29];
        uint8_t battery_data = (raw & 0x0F);
        bool cable_connected = (raw & 0x10) != 0;

        if (cable_connected) {
            if (battery_data < 10) {
                ds4->event.battery_level = battery_data * 10 + 5;
                ds4->event.battery_charging = true;
            } else if (battery_data == 10) {
                ds4->event.battery_level = 100;
                ds4->event.battery_charging = true;
            } else if (battery_data == 11) {
                ds4->event.battery_level = 100;
                ds4->event.battery_charging = false;  // Full
            } else {
                ds4->event.battery_level = 0;  // Error (14=voltage/temp, 15=charge)
                ds4->event.battery_charging = false;
            }
        } else {
            if (battery_data < 10)
                ds4->event.battery_level = battery_data * 10 + 5;
            else
                ds4->event.battery_level = 100;
            ds4->event.battery_charging = false;
        }
    }

    // Touchpad: only decoded from the standard Report 0x11 (the only report
    // that reaches this point - larger audio-uplink packets are filtered out
    // earlier). When speaker audio is playing the DS4 can interleave audio
    // uplink packets; the touch state simply holds its last valid value.
    if (data[0] == 0x11 && report_len >= sizeof(ds4_input_report_t)) {
        uint16_t tx = (((uint16_t)(rpt->tpad_f1_pos[1] & 0x0f)) << 8) | (uint16_t)rpt->tpad_f1_pos[0];
        uint16_t ty = (((uint16_t)rpt->tpad_f1_pos[2]) << 4) | ((uint16_t)(rpt->tpad_f1_pos[1] >> 4));
        uint16_t tx2 = (((uint16_t)(rpt->tpad_f2_pos[1] & 0x0f)) << 8) | (uint16_t)rpt->tpad_f2_pos[0];
        uint16_t ty2 = (((uint16_t)rpt->tpad_f2_pos[2]) << 4) | ((uint16_t)(rpt->tpad_f2_pos[1] >> 4));

        bool raw_f1_down = !rpt->tpad_f1_down;
        bool f1_down = raw_f1_down;

        if (raw_f1_down) {
            if (ds4->tpad_touch_cnt < 255) ds4->tpad_touch_cnt++;
        } else {
            ds4->tpad_touch_cnt = 0;
        }

        // Touchpad left/right click detection (touchpad is ~1920 wide, center at 960)
        if (rpt->tpad && f1_down && tx < 960)
            ds4->event.buttons |= PAD_BUTTON_L4;
        if (rpt->tpad && f1_down && tx >= 960)
            ds4->event.buttons |= PAD_BUTTON_R4;

        // Touchpad swipe delta (horizontal)
        int8_t touchpad_delta_x = 0;
        if (f1_down) {
            if (ds4->tpad_dragging) {
                int16_t delta = (int16_t)tx - (int16_t)ds4->tpad_last_pos;
                if (delta > 12) delta = 12;
                if (delta < -12) delta = -12;
                touchpad_delta_x = (int8_t)delta;
            }
            ds4->tpad_last_pos = tx;
            ds4->tpad_dragging = true;
        } else {
            ds4->tpad_dragging = false;
        }
        ds4->event.delta_x = touchpad_delta_x;

        // Touch coordinates — normalize DS4 native (1919x942) into 0..65535 canonical.
        ds4->event.touch[0].x = touch_norm_from_range(tx, 1919);
        ds4->event.touch[0].y = touch_norm_from_range(ty, 942);
        ds4->event.touch[0].active = f1_down;
        ds4->event.touch[1].x = touch_norm_from_range(tx2, 1919);
        ds4->event.touch[1].y = touch_norm_from_range(ty2, 942);
        ds4->event.touch[1].active = !rpt->tpad_f2_down;
        ds4->event.has_touch = true;
    }

    // Submit to router
    router_submit_input(&ds4->event);
}

static void ds4_task(bthid_device_t* device)
{
    ds4_bt_data_t* ds4 = (ds4_bt_data_t*)device->driver_data;
    if (!ds4) return;

    uint32_t now = platform_time_ms();

    // State machine for activation with delays
    switch (ds4->activation_state) {
        case 0:  // Wait 100ms after init before sending first output
            ds4->activation_time = now;
            ds4->activation_state = 1;
            break;

        case 1:  // Send initial LED output (also triggers enhanced report mode)
            if (now - ds4->activation_time >= 100) {
                int player_idx = find_player_index(ds4->event.dev_addr, ds4->event.instance);
                int color_idx = (player_idx >= 0) ? (player_idx % 4) : 0;
                uint8_t init_r = PLAYER_COLORS[color_idx][0];
                uint8_t init_g = PLAYER_COLORS[color_idx][1];
                uint8_t init_b = PLAYER_COLORS[color_idx][2];
                feedback_state_t* fb = (player_idx >= 0) ? feedback_get_state(player_idx) : NULL;
                if (fb && fb->led.has_rgb && (fb->led.r != 0 || fb->led.g != 0 || fb->led.b != 0)) {
                    init_r = fb->led.r;
                    init_g = fb->led.g;
                    init_b = fb->led.b;
                }
                // First SET_REPORT Output triggers DS4 to switch from basic (0x01)
                // to enhanced (0x11) report mode with motion/touchpad data.
                if (ds4_send_output(device, 0, 0, init_r, init_g, init_b)) {
                    ds4->activation_time = now;
                    ds4->activation_state = 2;
                } else {
                    // Retry in 50ms if control channel was busy
                    ds4->activation_time = now - 50;
                }
            }
            break;

        case 2:  // Activated - monitor audio stream and feedback system
            {
                int player_idx = find_player_index(ds4->event.dev_addr, ds4->event.instance);
                if (player_idx < 0) player_idx = 0;

                // If controller hasn't switched to enhanced report 0x11 yet (still on report 0x01),
                // periodically re-send activation output report every 200ms until sixaxis_enabled is true
                if (!ds4->sixaxis_enabled && (now - ds4->activation_time >= 200)) {
                    ds4->activation_time = now;
                    int color_idx = player_idx % 4;
                    uint8_t act_r = PLAYER_COLORS[color_idx][0];
                    uint8_t act_g = PLAYER_COLORS[color_idx][1];
                    uint8_t act_b = PLAYER_COLORS[color_idx][2];
                    feedback_state_t* fb_init = feedback_get_state(player_idx);
                    if (fb_init && fb_init->led.has_rgb && (fb_init->led.r != 0 || fb_init->led.g != 0 || fb_init->led.b != 0)) {
                        act_r = fb_init->led.r;
                        act_g = fb_init->led.g;
                        act_b = fb_init->led.b;
                    }
                    ds4_send_output(device, 0, 0, act_r, act_g, act_b);
                }

                // Factory calibration (feature report 0x05). First attempts go
                // out shortly after activation; if the control channel was busy
                // during connection setup, quiet retries keep going in the
                // background. Entirely optional: until it arrives the nominal
                // sensitivity is used, exactly as before this existed.
                if (!ds4->calib_valid && ds4->calib_attempts < DS4_CALIB_MAX_ATTEMPTS) {
                    if (ds4->calib_next_ms == 0) {
                        ds4->calib_next_ms = now + DS4_CALIB_FIRST_MS;
                    } else if ((int32_t)(now - ds4->calib_next_ms) >= 0) {
                        if (btstack_classic_send_get_report(device->conn_index, 3,
                                                            DS4_CALIB_REPORT_ID)) {
                            ds4->calib_attempts++;
                            if (ds4->calib_attempts >= DS4_CALIB_RETRY_ATTEMPTS) {
                                // Visible timeout (3 red blinks), then slow retries
                                if (ds4->calib_led_result == 0) {
                                    ds4->calib_led_result = 2;
                                    ds4->calib_led_start_ms = now;
                                    printf("[DS4_BT] Factory calibration: no response yet\n");
                                }
                                ds4->calib_next_ms = now + DS4_CALIB_QUIET_MS;
                            } else {
                                ds4->calib_next_ms = now + DS4_CALIB_RETRY_MS;
                            }
                        } else {
                            // Control channel busy (output report in flight): retry soon
                            ds4->calib_next_ms = now + 300;
                        }
                    }
                }

                feedback_state_t* fb = feedback_get_state(player_idx);

                bool need_update = false;
                uint8_t r = ds4->led_r;
                uint8_t g = ds4->led_g;
                uint8_t b = ds4->led_b;
                uint8_t rumble_left = fb ? fb->rumble.left : ds4->rumble_left;
                uint8_t rumble_right = fb ? fb->rumble.right : ds4->rumble_right;

                if (fb) {
                    // Check LED from feedback system
                    if (fb->led_dirty) {
                        if (fb->led.has_rgb) {
                            if (fb->led.r == 0 && fb->led.g == 0 && fb->led.b == 0) {
                                int color_idx = player_idx % 4;
                                r = PLAYER_COLORS[color_idx][0];
                                g = PLAYER_COLORS[color_idx][1];
                                b = PLAYER_COLORS[color_idx][2];
                            } else {
                                r = fb->led.r;
                                g = fb->led.g;
                                b = fb->led.b;
                            }
                        } else if (fb->led.pattern != 0) {
                            int player_num = 0;
                            if (fb->led.pattern & 0x01) player_num = 0;
                            else if (fb->led.pattern & 0x02) player_num = 1;
                            else if (fb->led.pattern & 0x04) player_num = 2;
                            else if (fb->led.pattern & 0x08) player_num = 3;

                            r = PLAYER_COLORS[player_num][0];
                            g = PLAYER_COLORS[player_num][1];
                            b = PLAYER_COLORS[player_num][2];
                        } else {
                            int color_idx = player_idx % 4;
                            r = PLAYER_COLORS[color_idx][0];
                            g = PLAYER_COLORS[color_idx][1];
                            b = PLAYER_COLORS[color_idx][2];
                        }
                        need_update = true;
                    }

                    // Check rumble
                    if (fb->rumble_dirty) {
                        need_update = true;
                    }
                }

                // Calibration feedback: 2 green blinks = factory calibration
                // applied, 3 red blinks = no response, 4 yellow blinks =
                // response rejected. Then the normal player colour returns.
                if (ds4->calib_led_result != 0) {
                    uint8_t blinks = (ds4->calib_led_result == 1) ? 2 :
                                     (ds4->calib_led_result == 2) ? 3 : 4;
                    uint32_t elapsed = now - ds4->calib_led_start_ms;
                    if (elapsed >= (uint32_t)blinks * 600u) {
                        ds4->calib_led_result = 0;
                        if (fb && fb->led.has_rgb && (fb->led.r != 0 || fb->led.g != 0 || fb->led.b != 0)) {
                            r = fb->led.r;
                            g = fb->led.g;
                            b = fb->led.b;
                        } else {
                            int color_idx = player_idx % 4;
                            r = PLAYER_COLORS[color_idx][0];
                            g = PLAYER_COLORS[color_idx][1];
                            b = PLAYER_COLORS[color_idx][2];
                        }
                    } else if (((elapsed / 300u) & 1u) == 0) {
                        if (ds4->calib_led_result == 1)      { r = 0;   g = 255; b = 0; }
                        else if (ds4->calib_led_result == 2) { r = 255; g = 0;   b = 0; }
                        else                                 { r = 255; g = 255; b = 0; }
                    } else {
                        r = 0; g = 0; b = 0;   // off between blinks
                    }
                }

                // Also check if values changed (even without dirty flag)
                if (rumble_left != ds4->rumble_left || rumble_right != ds4->rumble_right ||
                    r != ds4->led_r || g != ds4->led_g || b != ds4->led_b) {
                    need_update = true;
                }


                bool audio_active = ds4_audio_is_player_active((uint8_t)player_idx);

                // Airtime Governor: Allow maximum 1 concurrent audio stream across all DS4s
                // to prevent saturating CYW43439 SPI/ACL bandwidth and degrading 200 Hz input polling.
                // Strict priority: Player 0 > Player 1.
                bool audio_allowed = false;
                if (audio_active) {
                    int active_prior_streams = 0;
                    for (int p = 0; p < player_idx; p++) {
                        if (ds4_audio_is_player_active((uint8_t)p)) {
                            active_prior_streams++;
                        }
                    }
                    audio_allowed = (active_prior_streams < 1);
                }

                if (audio_active && audio_allowed) {
                    uint64_t now_us = time_us_64();

                    if (ds4->next_audio_us == 0 || (now_us > ds4->next_audio_us + 40000)) {
                        ds4->next_audio_us = now_us;
                    }

                    bool sent_audio = false;
                    // Input-Priority Interleaving:
                    // An audio report occupies ~2ms of 2.4GHz airtime and the
                    // DS4 emits its input report every ~4ms. Transmit audio ONLY
                    // in the 2.0ms window immediately following an incoming input report.
                    // This leaves a clean >=1.0ms guard band before the next input report,
                    // guaranteeing zero packet collisions on the RF link and rock-solid
                    // 250Hz input polling without packet drop cascade.
                    bool input_window_open =
                        (now_us - ds4->last_input_report_us <= 2000) ||
                        (ds4->last_input_report_us == 0) ||
                        (now_us - ds4->last_input_report_us >= 50000);
                    if (now_us >= ds4->next_audio_us && input_window_open) {
                        /*
                         * Pre-check L2CAP buffer BEFORE doing SBC encoding.
                         * If CYW43 L2CAP buffer is temporarily full, retry on the next task pass.
                         * DO NOT shift the schedule timeline into the future!
                         */
                        if (btstack_classic_can_send_interrupt(device->conn_index)) {
                            if (ds4_audio_get_player_report_15((uint8_t)player_idx, ds4->audio_report_buf,
                                                               sizeof(ds4->audio_report_buf),
                                                               rumble_left, rumble_right, r, g, b)) {
                                if (btstack_classic_send_interrupt_raw(device->conn_index, ds4->audio_report_buf, 335)) {
                                    sent_audio = true;
                                    // Two 16kHz SBC frames = 16ms of audio per report.
                                    ds4->next_audio_us += 16000;
                                    if (now_us > ds4->next_audio_us + 32000) {
                                        ds4->next_audio_us = now_us;
                                    }
                                    ds4->rumble_left = rumble_left;
                                    ds4->rumble_right = rumble_right;
                                    ds4->led_r = r;
                                    ds4->led_g = g;
                                    ds4->led_b = b;
                                    if (player_idx >= 0) {
                                        feedback_clear_dirty(player_idx);
                                    }
                                } else {
                                    /* Send failed despite pre-check (race): revert and back off
                                     * one full 16ms frame instead of re-encoding every loop pass. */
                                    ds4_audio_revert_player_report((uint8_t)player_idx);
                                    ds4->next_audio_us = now_us + 16000;
                                }
                            } else {
                                // Stream is not primed yet or waiting for buffer to fill:
                                // Keep next_audio_us aligned with now_us to prevent accumulating
                                // historical debt that causes a burst storm when priming completes.
                                ds4->next_audio_us = now_us;
                            }
                        }
                    }
                    /* When audio is active, Report 0x15 normally embeds rumble and LED every 16ms.
                     * However, if Report 0x15 could not be sent (e.g. buffer drained, stream unprimed)
                     * or rumble/LED changed or keepalive is due, transmit Report 0x11
                     * immediately so the motor never gets stuck vibrating and inactivity timer never expires! */
                    bool keepalive_due = (now - ds4->last_output_attempt_ms >= 2000);
                    if (keepalive_due && r == 0 && g == 0 && b == 0 && ds4->calib_led_result == 0) {
                        int color_idx = (player_idx >= 0) ? (player_idx % 4) : 0;
                        r = PLAYER_COLORS[color_idx][0];
                        g = PLAYER_COLORS[color_idx][1];
                        b = PLAYER_COLORS[color_idx][2];
                    }
                    bool is_rumble_stop = (rumble_left == 0 && rumble_right == 0 && (ds4->rumble_left != 0 || ds4->rumble_right != 0));
                    if (is_rumble_stop || (!sent_audio && ((need_update && (now - ds4->last_output_attempt_ms >= 10)) || keepalive_due))) {
                        if (ds4_send_output(device, rumble_left, rumble_right, r, g, b)) {
                            ds4->last_output_attempt_ms = now;
                            if (player_idx >= 0) {
                                feedback_clear_dirty(player_idx);
                            }
                        }
                    }
                } else {
                    ds4->next_audio_us = 0;
                    // Keepalive: send an output report every 2s if idle so the DS4 internal
                    // power-save timer is continuously refreshed and never shuts down unexpectedly.
                    bool keepalive_due = (now - ds4->last_output_attempt_ms >= 2000);
                    if (keepalive_due && r == 0 && g == 0 && b == 0 && ds4->calib_led_result == 0) {
                        int color_idx = (player_idx >= 0) ? (player_idx % 4) : 0;
                        r = PLAYER_COLORS[color_idx][0];
                        g = PLAYER_COLORS[color_idx][1];
                        b = PLAYER_COLORS[color_idx][2];
                    }
                    bool is_rumble_stop = (rumble_left == 0 && rumble_right == 0 && (ds4->rumble_left != 0 || ds4->rumble_right != 0));
                    if (is_rumble_stop || (need_update && (now - ds4->last_output_attempt_ms >= 10)) || keepalive_due) {
                        if (ds4_send_output(device, rumble_left, rumble_right, r, g, b)) {
                            ds4->last_output_attempt_ms = now;
                            if (player_idx >= 0) {
                                feedback_clear_dirty(player_idx);
                            }
                        }
                    }
                }
            }
            break;
    }
}

static void ds4_disconnect(bthid_device_t* device)
{
    printf("[DS4_BT] Disconnect: %s\n", device->name);

    ds4_bt_data_t* ds4 = (ds4_bt_data_t*)device->driver_data;
    if (ds4) {
        int player_idx = find_player_index(ds4->event.dev_addr, ds4->event.instance);
        // Clear router state first (sends zeroed input report)
        router_device_disconnected(ds4->event.dev_addr, ds4->event.instance);
        // Remove player assignment
        remove_players_by_address(ds4->event.dev_addr, ds4->event.instance);

        init_input_event(&ds4->event);
        ds4->initialized = false;
        ds4->next_audio_us = 0;
        ds4->last_input_report_us = 0;
        ds4->last_output_attempt_ms = 0;
        ds4->audio_retry_count = 0;
        ds4->share_press_cnt = 0;
        ds4->option_press_cnt = 0;
        ds4->ps_press_cnt = 0;
        ds4->tpad_press_cnt = 0;
        ds4->tpad_touch_cnt = 0;
        ds4->prev_raw_buttons = 0;
        memset(ds4->gyro_bias_q4, 0, sizeof(ds4->gyro_bias_q4));
        ds4->gyro_still_ms = 0;
        ds4->gyro_bias_init = false;
        ds4_reset_calibration(ds4);
        if (player_idx >= 0 && player_idx < DS4_AUDIO_MAX_PLAYERS) {
            ds4_audio_reset_player((uint8_t)player_idx);
        } else {
            ds4_audio_init();
        }
    }
}

// ============================================================================
// DRIVER STRUCT
// ============================================================================

const bthid_driver_t ds4_bt_driver = {
    .name = "Sony DualShock 4",
    .match = ds4_match,
    .init = ds4_init,
    .process_report = ds4_process_report,
    .get_report = ds4_get_report,
    .task = ds4_task,
    .disconnect = ds4_disconnect,
};

void ds4_bt_register(void)
{
    bthid_register_driver(&ds4_bt_driver);
}
