// btstack_config.h - BTstack configuration for TinyUSB HCI transport on RP2040
//
// This is a minimal configuration for using BTstack with a USB Bluetooth
// dongle via TinyUSB on RP2040-based boards.

#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

// ============================================================================
// PORT FEATURES
// ============================================================================

// Use embedded run loop (no RTOS)
#define HAVE_EMBEDDED_TIME_MS

// We have Pico SDK time functions
#define HAVE_BTSTACK_STDIN

// Memory allocation strategy
// For USB dongle transport: no malloc, use static pools
// For Pico W CYW43: use malloc (SDK provides it)
#ifdef BTSTACK_USE_CYW43
#define HAVE_MALLOC
#endif

// Printf works
#define HAVE_PRINTF
#define ENABLE_PRINTF_HEXDUMP

// ============================================================================
// BTSTACK FEATURES
// ============================================================================

// Enable Classic Bluetooth
// Use #ifndef to avoid redefinition when pico_btstack_classic is linked
#ifndef ENABLE_CLASSIC
#define ENABLE_CLASSIC
#endif

// Enable logging
#define ENABLE_LOG_ERROR
#define ENABLE_LOG_INFO
#define ENABLE_LOG_DEBUG  // Enable for verbose logging

// ============================================================================
// BUFFER SIZES
// ============================================================================

#if defined(CONFIG_DS5_DROP_SCREAM) || defined(CONFIG_DS4_SPEAKER_AUDIO)
// Route BTstack-emitted events (incl. L2CAP_EVENT_CHANNEL_OPENED) through the
// public hci_dump interface — lets us observe HID channel CIDs for direct
// audio sends without modifying the BTstack library.
#define ENABLE_LOG_BTSTACK_EVENTS
#endif

// HCI ACL payload size (standard BT is 1021, but we use smaller for memory)
#if defined(CONFIG_DS5_DROP_SCREAM) || defined(CONFIG_DS4_SPEAKER_AUDIO)
// Extended output report (DS4: 334 bytes + HID header; DS5: 398 bytes + HID header)
// must fit a single outgoing ACL buffer: l2cap max payload = HCI_ACL_PAYLOAD_SIZE - 4
#define HCI_ACL_PAYLOAD_SIZE (512 + 4 + 3)
#else
#define HCI_ACL_PAYLOAD_SIZE 256
#endif

// Pre-buffer for L2CAP/BNEP headers
#define HCI_INCOMING_PRE_BUFFER_SIZE 14

// CYW43-specific buffer requirements
#ifdef BTSTACK_USE_CYW43
// CYW43 requires 4 bytes pre-buffer for packet header
#define HCI_OUTGOING_PRE_BUFFER_SIZE 4
// CYW43 requires 4-byte alignment
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT 4
#endif

// ============================================================================
// MEMORY POOLS (static allocation)
// ============================================================================

// Number of HCI connections (Classic)
#define MAX_NR_HCI_CONNECTIONS 4

// Number of L2CAP channels (Classic HID needs Control + Interrupt + SDP per device)
#define MAX_NR_L2CAP_CHANNELS 16

// Number of L2CAP services
#define MAX_NR_L2CAP_SERVICES 4

// Link keys storage (Classic BT)
#define NVM_NUM_LINK_KEYS 4
#define MAX_NR_BTSTACK_LINK_KEY_DB_MEMORY_ENTRIES 8

// ============================================================================
// HID SUPPORT
// ============================================================================

// Enable HID Host (for game controllers)
#define ENABLE_HID_HOST

// Number of HID Host connections (Classic BT HID devices)
#define MAX_NR_HID_HOST_CONNECTIONS 4

#endif // BTSTACK_CONFIG_H
