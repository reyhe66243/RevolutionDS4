#ifndef DS4_AUDIO_H
#define DS4_AUDIO_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DS4_AUDIO_MAX_PLAYERS 4

// Initialize DS4 audio subsystem and SBC encoder
void ds4_audio_init(void);

// Reset a single player's audio stream (e.g. on controller disconnect)
void ds4_audio_reset_player(uint8_t player_idx);

// Feed audio packet from USB (fakemote speaker report 0x18)
void ds4_audio_feed_report(const uint8_t* data, uint16_t len);

// Check if audio playback is currently active
bool ds4_audio_is_active(void);
bool ds4_audio_is_player_active(uint8_t player_idx);

// Get the latest speaker volume received from console (0..255)
uint8_t ds4_audio_get_volume(void);
uint8_t ds4_audio_get_player_volume(uint8_t player_idx);

// Generate DS4 Bluetooth Report 0x15 packet (335 bytes)
bool ds4_audio_get_report_15(uint8_t* out_buf, size_t max_len,
                             uint8_t rumble_left, uint8_t rumble_right,
                             uint8_t r, uint8_t g, uint8_t b);

bool ds4_audio_get_player_report_15(uint8_t player_idx, uint8_t* out_buf, size_t max_len,
                                    uint8_t rumble_left, uint8_t rumble_right,
                                    uint8_t r, uint8_t g, uint8_t b);

// Revert last popped samples and sequence counter if transmission failed
void ds4_audio_revert_report(void);
void ds4_audio_revert_player_report(uint8_t player_idx);

// Timestamp of last transmitted audio packet
uint32_t ds4_audio_get_last_play_time_ms(void);
uint32_t ds4_audio_get_player_last_play_time_ms(uint8_t player_idx);

// Diagnostic counters for visual feedback
extern volatile uint32_t ds4_audio_diag_rx_count;
extern volatile uint32_t ds4_audio_diag_samples_count;

#ifdef __cplusplus
}
#endif

#endif // DS4_AUDIO_H
