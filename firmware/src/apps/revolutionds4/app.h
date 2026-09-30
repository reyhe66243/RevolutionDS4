// app.h - BT2USB App Manifest
// Bluetooth to USB adapter (HID Gamepad output) for Pico W
//
// Uses Pico W's built-in CYW43 Bluetooth to receive controllers,
// outputs as USB HID device.
//
// This manifest is a human-readable summary of what this app uses.
// It is NOT consumed by the build system. The authoritative per-target
// configuration lives in src/CMakeLists.txt. Only #ifndef-guarded flags are
// read by code (see REQUIRE_BT_INPUT / REQUIRE_BLE_OUTPUT in the adapter app);
// every other flag here is descriptive only and changing it has no effect.
// See issue #198.

#ifndef APP_BT2USB_H
#define APP_BT2USB_H

#include <stdint.h>

// ============================================================================
// APP METADATA
// ============================================================================
#define APP_NAME "BT2USB"
#define APP_DESCRIPTION "Bluetooth to USB HID gamepad adapter (Pico W)"
#define APP_AUTHOR "RobertDaleSmith"

// ============================================================================
// CORE DEPENDENCIES (What drivers to compile in)
// ============================================================================

// Input drivers - Pico W built-in Bluetooth
#define REQUIRE_BT_CYW43 1              // CYW43 Bluetooth (Pico W built-in)
#define REQUIRE_USB_HOST 0              // No USB host needed
#define MAX_USB_DEVICES 0

// Output drivers
#define REQUIRE_USB_DEVICE 1
#define USB_OUTPUT_PORTS 2              // Dual gamepads for multiplexed 2-player

// Services
#define REQUIRE_FLASH_SETTINGS 0        // Descriptive only. Profiles DO persist:
                                        // flash_init() at usbd.c:574, loaded by
                                        // profile_load_from_flash(), saved via storage_task().
#define REQUIRE_PROFILE_SYSTEM 0        // No profiles yet
#define REQUIRE_PLAYER_MANAGEMENT 1

// ============================================================================
// ROUTING CONFIGURATION
// ============================================================================
#define ROUTING_MODE ROUTING_MODE_SIMPLE
#define MERGE_MODE MERGE_ALL
#define APP_MAX_ROUTES 4

// Input transformations
#define TRANSFORM_FLAGS 0               // No transformations

// ============================================================================
// PLAYER MANAGEMENT
// ============================================================================
#define PLAYER_SLOT_MODE PLAYER_SLOT_FIXED
#define MAX_PLAYER_SLOTS 4
#define AUTO_ASSIGN_ON_PRESS 1

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================
#define BOARD "pico_w"                  // Raspberry Pi Pico W
#define CPU_OVERCLOCK_KHZ 0             // No overclock needed
#define UART_DEBUG 1

// ============================================================================
// BLUETOOTH CONFIGURATION
// ============================================================================
#define BT_MAX_CONNECTIONS 4            // Max BT controllers
#define BT_SCAN_ON_STARTUP 0            // Scan on BOOTSEL click (Wii Sync style)

// ============================================================================
// APP FEATURES
// ============================================================================
#define FEATURE_PROFILES 0              // No profiles yet
#define FEATURE_OUTPUT_MODE_SELECT 0    // Descriptive only. The dedicated dongle
                                        // build is locked to PS4 output mode
                                        // (USB_OUTPUT_MODE_PS4 in CMakeLists).

// ============================================================================
// APP INTERFACE (OS calls these)
// ============================================================================
void app_init(void);
void app_task(void);
void app_disconnect_player(uint8_t player_index);

#endif // APP_BT2USB_H
