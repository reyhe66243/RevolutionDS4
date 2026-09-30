// app.c - BT2USB App Entry Point
// Bluetooth to USB HID gamepad adapter for Pico W
//
// Uses Pico W's built-in CYW43 Bluetooth to receive controllers,
// outputs as USB HID device.

#include "app.h"
#include "core/router/router.h"
#include "core/services/players/manager.h"
#include "core/services/players/feedback.h"
#include "core/services/button/button.h"
#include "core/input_interface.h"
#include "core/output_interface.h"
#include "usb/usbd/usbd.h"
#include "bt/transport/bt_transport.h"
#include "bt/btstack/btstack_host.h"
#include "core/services/leds/leds.h"
#include "bt/bthid/devices/vendors/sony/ds4_audio.h"

#include "tusb.h"
#include "platform/platform.h"
#include <stdio.h>

#ifdef BTSTACK_USE_ESP32
#include "driver/gpio.h"
extern const bt_transport_t bt_transport_esp32;
// Status LED GPIO — board-specific defaults
// Feather ESP32-S3: GPIO 13 (red LED, active high). GPIO 21 is NeoPixel power!
// Seeed XIAO ESP32-S3: GPIO 21 (active low)
#ifndef STATUS_LED_GPIO
  #ifdef BOARD_FEATHER_ESP32S3
    #define STATUS_LED_GPIO 13
  #else
    #define STATUS_LED_GPIO 21
  #endif
#endif
#ifndef STATUS_LED_ACTIVE_LOW
  #ifdef BOARD_FEATHER_ESP32S3
    #define STATUS_LED_ACTIVE_LOW 0
  #else
    #define STATUS_LED_ACTIVE_LOW 1
  #endif
#endif
#elif defined(BTSTACK_USE_NRF)
extern const bt_transport_t bt_transport_nrf;
// nRF: LED status handled by ws2812_nrf.c (RGB LEDs driven via neopixel API)
#ifdef OLED_I2C_DISPLAY
#include "core/services/display/display.h"
#include "core/input_event.h"
#include "core/buttons.h"
#endif
#else
#include "pico/cyw43_arch.h"
extern const bt_transport_t bt_transport_cyw43;
#endif

// ============================================================================
// USB BUS SUSPEND / RESUME
// ============================================================================
// When the USB host (the console) enters sleep, the bus suspends (no SOF >3 ms).
// We drop the active BT link so the bridged controller (e.g. DS4) auto-sleeps
// instead of staying powered forever — the console keeps VBUS hot during sleep
// so nothing else would tell the controller to power down. On resume, the
// existing scan loop reconnects when the controller connects again.

#include "device/dcd.h"

static bool usb_attached = false;
static bool usb_suspended = false;
static uint32_t usb_suspend_start_ms = 0;

void tud_mount_cb(void)
{
    usb_attached = true;
    usb_suspended = false;
    dcd_event_bus_signal(0, DCD_EVENT_RESUME, false);
}

void tud_umount_cb(void)
{
    usb_attached = false;
}

void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
    usb_suspended = true;
    usb_suspend_start_ms = platform_time_ms();
    dcd_event_bus_signal(0, DCD_EVENT_RESUME, false);
}

void tud_resume_cb(void)
{
    usb_suspended = false;
}

// "Turn off controller" override. The console fires this when the user selects
// the accessory "Turn off controller" option while we are in PS4 output mode.
// The wired DS4 surface can't actually disappear (we stay plugged in), so
// propagate the intent to the BT side — a real DS4 bridged over Bluetooth
// loses its host and auto-sleeps within a minute. User presses the controller's
// PS button to wake + re-pair.
void app_on_console_shutdown(void)
{
    printf("[app:revolutionds4] Console shutdown -> disconnecting bridged BT controller\n");
    btstack_host_disconnect_all_devices();
    for (int i = 0; i < DS4_AUDIO_MAX_PLAYERS; i++) {
        ds4_audio_reset_player(i);
    }
}

void app_disconnect_player(uint8_t player_index)
{
    printf("[app:revolutionds4] Console requested player %d disconnect\n", player_index);
    if (player_index < MAX_PLAYERS && players[player_index].dev_addr >= 0) {
        btstack_host_disconnect_device((uint8_t)players[player_index].dev_addr);
    } else {
        btstack_host_disconnect_all_devices();
    }
    if (player_index < DS4_AUDIO_MAX_PLAYERS) {
        ds4_audio_reset_player(player_index);
    }
    feedback_set_rumble(player_index, 0, 0);
    feedback_clear_dirty(player_index);
}

// ============================================================================
// LED STATUS
// ============================================================================

static uint32_t led_last_toggle = 0;
static bool led_state = false;
static bool sync_window_active = false;
static uint32_t sync_window_start_ms = 0;

// Update LED based on connection status
// - Blink (0.8s): No device connected (scanning, connecting, or idle)
// - Solid on: Device connected
static void platform_led_set(bool on)
{
#ifdef BTSTACK_USE_ESP32
    gpio_set_level(STATUS_LED_GPIO, (on ^ STATUS_LED_ACTIVE_LOW) ? 1 : 0);
#elif defined(BTSTACK_USE_NRF)
    // No-op: RGB LEDs driven by ws2812_nrf.c via neopixel_task()
    (void)on;
#else
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on ? 1 : 0);
#endif
}

static void led_status_update(void)
{
    uint32_t now = platform_time_ms();

    // Actively scanning (button press / pairing window): FAST blink (100ms)
    if (btstack_host_is_scanning() || sync_window_active) {
        if (now - led_last_toggle >= 100) {
            led_state = !led_state;
            platform_led_set(led_state);
            led_last_toggle = now;
        }
        return;
    }

    // Fully connected and operational: SOLID ON
    if (btstack_host_has_ready_connection()) {
        if (!led_state) {
            platform_led_set(true);
            led_state = true;
        }
        return;
    }

    // Connecting / Handshaking: MEDIUM blink (200ms)
    if (btstack_host_is_connecting()) {
        if (now - led_last_toggle >= 200) {
            led_state = !led_state;
            platform_led_set(led_state);
            led_last_toggle = now;
        }
        return;
    }

    // Idle / waiting for controller: SLOW blink (700ms)
    if (now - led_last_toggle >= 700) {
        led_state = !led_state;
        platform_led_set(led_state);
        led_last_toggle = now;
    }
}

// ============================================================================
// BUTTON EVENT HANDLER
// ============================================================================

static void on_button_event(button_event_t event)
{
    switch (event) {
        case BUTTON_EVENT_CLICK:
            // Start 20-second BT sync scan for DS4 controllers (like Wii Sync button).
            // Do NOT disconnect existing active controllers!
            printf("[app:revolutionds4] BOOTSEL clicked: opening 20s DS4 sync window...\n");
            sync_window_active = true;
            sync_window_start_ms = platform_time_ms();
            gap_discoverable_control(1);
            gap_connectable_control(1);
            btstack_host_start_timed_scan(20000);
            break;

        case BUTTON_EVENT_DOUBLE_CLICK:
        case BUTTON_EVENT_TRIPLE_CLICK:
            // Dedicated DS4 dongle: output mode is permanently locked to PS4
            printf("[app:revolutionds4] Dedicated DS4 Dongle: locked to PS4 USB mode\n");
            break;

        case BUTTON_EVENT_HOLD:
            // Long press to disconnect all devices, clear all bonds, and immediately open sync window
            printf("[app:revolutionds4] Disconnecting all devices, clearing bonds, and opening 20s DS4 sync window...\n");
            btstack_host_disconnect_all_devices();
            btstack_host_delete_all_bonds();
            for (int i = 0; i < DS4_AUDIO_MAX_PLAYERS; i++) {
                ds4_audio_reset_player(i);
                feedback_set_rumble(i, 0, 0);
                feedback_clear_dirty(i);
            }
            sync_window_active = true;
            sync_window_start_ms = platform_time_ms();
            gap_discoverable_control(1);
            gap_connectable_control(1);
            btstack_host_start_timed_scan(20000);
            break;

        default:
            break;
    }
}

// ============================================================================
// APP INPUT INTERFACES
// ============================================================================

// BT2USB has no InputInterface - BT transport handles input internally
// via bthid drivers that call router_submit_input()

const InputInterface** app_get_input_interfaces(uint8_t* count)
{
    *count = 0;
    return NULL;
}

// ============================================================================
// APP OUTPUT INTERFACES
// ============================================================================

static const OutputInterface* output_interfaces[] = {
    &usbd_output_interface,
};

const OutputInterface** app_get_output_interfaces(uint8_t* count)
{
    *count = sizeof(output_interfaces) / sizeof(output_interfaces[0]);
    return output_interfaces;
}

// ============================================================================
// OLED DISPLAY (XIAO Expansion Board - SSD1306 128x64 I2C)
// ============================================================================

#ifdef OLED_I2C_DISPLAY

// Arrow characters for display (1=up, 2=down, 3=left, 4=right)
#define ARROW_UP    "\x01"
#define ARROW_DOWN  "\x02"
#define ARROW_LEFT  "\x03"
#define ARROW_RIGHT "\x04"

typedef struct {
    uint32_t mask;
    const char* name;
} button_name_t;

static const button_name_t button_names[] = {
    { PAD_BUTTON_DU, ARROW_UP },
    { PAD_BUTTON_DR, ARROW_RIGHT },
    { PAD_BUTTON_DD, ARROW_DOWN },
    { PAD_BUTTON_DL, ARROW_LEFT },
    { PAD_BUTTON_B1, "B1" },
    { PAD_BUTTON_B2, "B2" },
    { PAD_BUTTON_B3, "B3" },
    { PAD_BUTTON_B4, "B4" },
    { PAD_BUTTON_L1, "L1" },
    { PAD_BUTTON_R1, "R1" },
    { PAD_BUTTON_L2, "L2" },
    { PAD_BUTTON_R2, "R2" },
    { PAD_BUTTON_S1, "S1" },
    { PAD_BUTTON_S2, "S2" },
    { PAD_BUTTON_L3, "L3" },
    { PAD_BUTTON_R3, "R3" },
    { PAD_BUTTON_A1, "A1" },
    { 0, NULL }
};

static const char* transport_str(input_transport_t t) {
    switch (t) {
        case INPUT_TRANSPORT_BT_CLASSIC: return "BT";
        default:                         return "?";
    }
}

static void oled_init(void) {
    display_i2c_config_t cfg = {
        .i2c_inst = 0,
        .pin_sda  = 0,     // Configured by devicetree on nRF
        .pin_scl  = 0,
        .addr     = 0x3C,
    };
#ifdef BOARD_FEATHER_NRF52840
    display_init_i2c(&cfg);  // SH1107 FeatherWing OLED
    printf("[app:revolutionds4] OLED display initialized (SH1107 I2C)\n");
#else
    display_init_ssd1306_i2c(&cfg);  // SSD1306 XIAO Expansion Board
    printf("[app:revolutionds4] OLED display initialized (SSD1306 I2C)\n");
#endif
}

static input_event_t oled_cached_event;
static bool oled_has_event = false;

static void oled_update_display(void) {
    static uint32_t last_update = 0;
    static uint32_t last_buttons = 0;
    uint32_t now = platform_time_ms();

    // Cache latest router output
    if (playersCount > 0 && players[0].dev_addr >= 0) {
        const input_event_t* ev = router_get_output(OUTPUT_TARGET_USB_DEVICE, 0);
        if (ev) {
            oled_cached_event = *ev;
            oled_has_event = true;
        }
    }

    // Feed button presses to marquee (edge detection)
    uint32_t buttons = oled_has_event ? oled_cached_event.buttons : 0;
    uint32_t newly_pressed = ~last_buttons & buttons;
    last_buttons = buttons;
    for (int i = 0; button_names[i].name != NULL; i++) {
        if (newly_pressed & button_names[i].mask) {
            display_marquee_add(button_names[i].name);
        }
    }

    if (now - last_update < 50) return;  // 20fps max
    last_update = now;

    display_clear();

    // Line 1 (large, y=0): USB output mode
    usb_output_mode_t mode = usbd_get_mode();
    display_text_large(0, 0, usbd_get_mode_name(mode));

    // Separator
    display_hline(0, 17, DISPLAY_WIDTH);

    // Lines 2-4: Controller info
    if (playersCount > 0 && players[0].dev_addr >= 0) {
        const char* name = get_player_name(0);
        if (name) {
            display_text(0, 20, name);
        }

        char info[22];
        snprintf(info, sizeof(info), "%s dev:%d P%d/%d",
                 transport_str(players[0].transport),
                 players[0].dev_addr,
                 players[0].player_number, playersCount);
        display_text(0, 30, info);

        if (oled_has_event) {
            char line[22];
            snprintf(line, sizeof(line), "L:%02X,%02X R:%02X,%02X T:%02X,%02X",
                     oled_cached_event.analog[ANALOG_LX], oled_cached_event.analog[ANALOG_LY],
                     oled_cached_event.analog[ANALOG_RX], oled_cached_event.analog[ANALOG_RY],
                     oled_cached_event.analog[ANALOG_L2], oled_cached_event.analog[ANALOG_R2]);
            display_text(0, 40, line);
        }
    } else {
        display_text(0, 28, "No controller");
    }

    // Bottom (y=52): Button marquee
    display_marquee_tick();
    display_marquee_render(52);

    display_update();
}

#endif // OLED_I2C_DISPLAY

// ============================================================================
// APP INITIALIZATION
// ============================================================================

void app_init(void)
{
    printf("[app:revolutionds4] Initializing REVOLUTIONDS4 v%s\n", FIRMWARE_VERSION);
#ifdef BTSTACK_USE_ESP32
    printf("[app:revolutionds4] ESP32-S3 BLE -> USB HID\n");
    // Init status LED GPIO
    gpio_config_t led_cfg = {
        .pin_bit_mask = (1ULL << STATUS_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&led_cfg);
    gpio_set_level(STATUS_LED_GPIO, STATUS_LED_ACTIVE_LOW ? 1 : 0);  // Start OFF
#elif defined(BTSTACK_USE_NRF)
#ifdef BOARD_FEATHER_NRF52840
    printf("[app:revolutionds4] Adafruit Feather nRF52840 Express BLE -> USB HID\n");
#else
    printf("[app:revolutionds4] Seeed XIAO nRF52840 BLE -> USB HID\n");
#endif
    // RGB LEDs initialized by ws2812_nrf.c via leds_init()
#ifdef OLED_I2C_DISPLAY
    oled_init();
#endif
#else
    printf("[app:revolutionds4] Pico W built-in Bluetooth -> USB HID\n");
#endif

    // Initialize button service (uses BOOTSEL button on Pico W)
    button_init();
    button_set_callback(on_button_event);

    // Configure router for BT2USB
    router_config_t router_cfg = {
        .mode = ROUTING_MODE,
        .merge_mode = MERGE_MODE,
        .max_players_per_output = {
            [OUTPUT_TARGET_USB_DEVICE] = USB_OUTPUT_PORTS,
        },
        .merge_all_inputs = false,  // Do not merge all to player 0: keep player 0 and player 1 separate
        .transform_flags = TRANSFORM_FLAGS,
    };
    router_init(&router_cfg);

    // Add default route: BT central → USB Device
    router_add_route(INPUT_SOURCE_BLE_CENTRAL, OUTPUT_TARGET_USB_DEVICE, 0);

    // Configure player management
    player_config_t player_cfg = {
        .slot_mode = PLAYER_SLOT_MODE,
        .max_slots = MAX_PLAYER_SLOTS,
        .auto_assign_on_press = AUTO_ASSIGN_ON_PRESS,
    };
    players_init_with_config(&player_cfg);

    // Initialize Bluetooth transport
    // Must use bt_init() to set global transport pointer and register drivers
    printf("[app:revolutionds4] Initializing Bluetooth...\n");
#ifdef BTSTACK_USE_ESP32
    bt_init(&bt_transport_esp32);
#elif defined(BTSTACK_USE_NRF)
    bt_init(&bt_transport_nrf);
#else
    bt_init(&bt_transport_cyw43);
#endif

    // Keep USB disconnected on boot until a controller connects via Bluetooth.
    // Output mode is fixed to PS4 (dedicated DS4 dongle).
    tud_disconnect();
    usb_attached = false;

    printf("[app:revolutionds4] Initialization complete\n");
    printf("[app:revolutionds4]   Dongle: Sony DualShock 4 -> Emulated PS4 v2 USB\n");
    printf("[app:revolutionds4]   Player slots: %d\n", MAX_PLAYER_SLOTS);
    printf("[app:revolutionds4]   Click BOOTSEL for 20s DS4 sync scan (Wii Sync style)\n");
    printf("[app:revolutionds4]   Hold BOOTSEL to disconnect all + clear bonds\n");
}

// ============================================================================
// APP TASK (Called from main loop)
// ============================================================================

void app_task(void)
{
    // Handle USB connection: connect once a BT controller is first detected.
    // Keep USB attached even if the controller temporarily goes idle or disconnects,
    // because tearing down the USB connection crashes/wedges Nintendo Wii's Starlet OHCI
    // host driver and permanently destroys the cIOS fake Wii Remote session.
    bool has_controller = (playersCount > 0 && players[0].dev_addr >= 0) ||
                          btstack_host_has_ready_connection();

    if (has_controller && !usb_attached) {
        printf("[app:revolutionds4] Controller connected via BT -> connecting USB\n");
        tud_connect();
        usb_attached = true;
    }

    // Process button input
    button_task();

    // Process Bluetooth transport
    bt_task();

    // Bluetooth RF airtime and scan management:
    // If a sync window was opened via BOOTSEL button:
    // Close it if:
    // 1. Audio is actively playing (audio takes absolute 100% RF airtime priority!)
    // 2. Both controller slots are connected (conn_count >= USB_OUTPUT_PORTS)
    // 3. 20 seconds expired
    uint32_t now = platform_time_ms();
    uint8_t conn_count = btstack_classic_get_connection_count();
    bool has_ready = btstack_host_has_ready_connection();
    bool audio_active = ds4_audio_is_player_active(0) || ds4_audio_is_player_active(1);

    static bool scans_disabled = true;

    if (sync_window_active) {
        if ((now - sync_window_start_ms >= 20000) ||
            (conn_count >= USB_OUTPUT_PORTS) ||
            audio_active) {
            printf("[app:revolutionds4] Sync window closed (conn=%d, audio=%d) -> disabling discovery scan\n",
                   conn_count, audio_active);
            sync_window_active = false;
            if (btstack_host_is_scanning()) {
                btstack_host_stop_scan();
            }
            gap_discoverable_control(0);
            if (conn_count >= USB_OUTPUT_PORTS || audio_active) {
                scans_disabled = true;
                gap_connectable_control(0);
            } else {
                scans_disabled = false;
                gap_connectable_control(1);
            }
        }
    } else {
        // Not in sync window:
        // When all slots are full OR audio is playing, disable page scan so RF airtime is 100% for gameplay/audio.
        // But if slots are free (< USB_OUTPUT_PORTS) and no audio is playing, keep page scan enabled
        // so that already-bonded controllers can reconnect simply by pressing the PS button!
        if (conn_count >= USB_OUTPUT_PORTS || audio_active) {
            if (btstack_host_is_scanning()) {
                btstack_host_stop_scan();
            }
            if (!scans_disabled) {
                scans_disabled = true;
                gap_discoverable_control(0);
                gap_connectable_control(0);
            }
        } else {
            // Free slots available and no audio: allow page scan so paired controllers can connect on PS press
            if (scans_disabled) {
                scans_disabled = false;
                gap_connectable_control(1);
            }
        }
    }

    if (usb_suspended && (now - usb_suspend_start_ms >= 30000)) {
        printf("[app:revolutionds4] USB suspend > 30s -> console sleeping or powered down\n");
        usb_suspended = false;
        app_on_console_shutdown();
    }

    // Update LED status
    leds_set_connected_devices(btstack_host_has_ready_connection() ? 1 : 0);
    led_status_update();

    // Route feedback from USB device output to targeted BT controllers
    if (usbd_output_interface.get_feedback) {
        output_feedback_t fb;
        while (usbd_output_interface.get_feedback(&fb)) {
            uint8_t target = fb.target_player;
            feedback_set_rumble(target, fb.rumble_left, fb.rumble_right);
            if (fb.led_player > 0) {
                feedback_set_led_player(target, fb.led_player);
            }
            if (fb.has_led_rgb) {
                feedback_set_led_rgb(target, fb.led_r, fb.led_g, fb.led_b);
            }
            if (fb.has_speaker_volume) {
                feedback_set_speaker_volume(target, fb.volume_speaker);
            }
        }
    }

#ifdef OLED_I2C_DISPLAY
    oled_update_display();
#endif
}
