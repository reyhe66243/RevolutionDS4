#include "ds4_audio.h"
#include "platform/platform.h"

#if defined(CONFIG_DS4_SPEAKER_AUDIO)

#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "classic/btstack_sbc.h"
#include "classic/btstack_sbc_bluedroid.h"
#include <string.h>
#include <stdio.h>

#define AUDIO_RB_SIZE 4096
#define AUDIO_RB_MASK (AUDIO_RB_SIZE - 1)
#define TARGET_CUSHION 640
#define PRIMING_THRESHOLD 512

// SBC output sample rate. 16 kHz (instead of 32 kHz) lets one 128-sample SBC
// frame carry 8 ms of audio, so a two-frame report covers 16 ms and the audio
// report is transmitted half as often. The Wii Remote speaker source is ~6 kHz
// band-limited, so there is no audible quality loss, and the PS4 itself drops
// to 16 kHz when several controllers stream audio.
#define SBC_SAMPLE_RATE 16000

typedef struct {
    int32_t predictor;
    int32_t step;
} yamaha_adpcm_state_t;

typedef struct {
    int16_t audio_rb[AUDIO_RB_SIZE];
    volatile uint16_t audio_rb_head;
    volatile uint16_t audio_rb_tail;
    uint16_t revert_rb_tail;
    uint16_t revert_seq;
    uint32_t last_feed_time_ms;
    uint8_t current_volume;
    uint16_t ds4_seq_counter;
    yamaha_adpcm_state_t adpcm_decoder;
    int16_t last_input_sample;
    uint32_t resample_phase;
    bool stream_primed;
    int32_t filtered_cushion;
    uint8_t warmup_reports_remaining;
    uint8_t postroll_reports_remaining;
    uint8_t fade_in_remaining;
    uint8_t revert_warmup;
    uint8_t revert_postroll;
    uint32_t last_audio_play_ms;
} ds4_audio_stream_t;

static ds4_audio_stream_t streams[DS4_AUDIO_MAX_PLAYERS];

volatile uint32_t ds4_audio_diag_rx_count = 0;
volatile uint32_t ds4_audio_diag_samples_count = 0;
static btstack_sbc_encoder_state_t sbc_encoder_state;
static bool sbc_initialized = false;
static uint8_t sbc_silence_frame[112];
static bool sbc_silence_ready = false;

static const int32_t yamaha_difflookup[] = {
    1, 3, 5, 7, 9, 11, 13, 15,
    -1, -3, -5, -7, -9, -11, -13, -15
};

static const int32_t yamaha_indexscale[] = {
    230, 230, 230, 230, 307, 409, 512, 614,
    230, 230, 230, 230, 307, 409, 512, 614
};

static inline int16_t av_clip16(int32_t a)
{
    if ((a + 32768) & ~65535)
        return (a >> 31) ^ 32767;
    else
        return (int16_t)a;
}

static inline int32_t av_clip(int32_t a, int32_t amin, int32_t amax)
{
    if (a < amin) return amin;
    if (a > amax) return amax;
    return a;
}

static int16_t adpcm_decode_nibble(uint8_t p, uint8_t nibble)
{
    yamaha_adpcm_state_t* dec = &streams[p].adpcm_decoder;
    dec->predictor += (dec->step * yamaha_difflookup[nibble & 0xF]) / 8;
    dec->predictor = av_clip16(dec->predictor);
    dec->step = (dec->step * yamaha_indexscale[nibble & 0xF]) >> 8;
    dec->step = av_clip(dec->step, 127, 24576);
    return (int16_t)dec->predictor;
}

static uint32_t ds4_crc32_raw(uint32_t seed, const uint8_t* data, size_t len)
{
    uint32_t crc = seed;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }
    return crc;
}

static uint32_t ds4_audio_bt_crc32(const uint8_t* report_data, size_t len)
{
    const uint8_t seed = 0xA2;
    uint32_t crc = ds4_crc32_raw(0xFFFFFFFF, &seed, 1);
    crc = ds4_crc32_raw(crc, report_data, len);
    return ~crc;
}

static inline uint16_t rb_count(uint8_t p)
{
    if (p >= DS4_AUDIO_MAX_PLAYERS) return 0;
    return (streams[p].audio_rb_head - streams[p].audio_rb_tail) & AUDIO_RB_MASK;
}

// Ring buffer access. Both the producer (USB audio OUT callback) and the
// consumer (bt_task audio report generation) run from the Core 0 main loop,
// never from an ISR, so no interrupt masking is needed here. The old
// save_and_disable_interrupts() per sample fired tens of thousands of times
// per second and only added IRQ latency for the CYW43/USB without protecting
// anything.
static inline void rb_push(uint8_t p, int16_t sample)
{
    if (p >= DS4_AUDIO_MAX_PLAYERS) return;
    streams[p].audio_rb[streams[p].audio_rb_head & AUDIO_RB_MASK] = sample;
    streams[p].audio_rb_head = (streams[p].audio_rb_head + 1) & AUDIO_RB_MASK;
    if (streams[p].audio_rb_head == streams[p].audio_rb_tail) {
        streams[p].audio_rb_tail = (streams[p].audio_rb_tail + 1) & AUDIO_RB_MASK;
    }
}

static inline int16_t rb_pop(uint8_t p)
{
    if (p >= DS4_AUDIO_MAX_PLAYERS) return 0;
    int16_t s = 0;
    if (streams[p].audio_rb_head != streams[p].audio_rb_tail) {
        s = streams[p].audio_rb[streams[p].audio_rb_tail & AUDIO_RB_MASK];
        streams[p].audio_rb_tail = (streams[p].audio_rb_tail + 1) & AUDIO_RB_MASK;
    }
    return s;
}

void ds4_audio_init(void)
{
    for (int p = 0; p < DS4_AUDIO_MAX_PLAYERS; p++) {
        streams[p].audio_rb_head = 0;
        streams[p].audio_rb_tail = 0;
        streams[p].revert_rb_tail = 0;
        streams[p].revert_seq = 0;
        streams[p].last_feed_time_ms = 0;
        streams[p].current_volume = 0x40;
        streams[p].ds4_seq_counter = 0;
        streams[p].adpcm_decoder.predictor = 0;
        streams[p].adpcm_decoder.step = 127;
        streams[p].last_input_sample = 0;
        streams[p].resample_phase = 0;
        streams[p].stream_primed = false;
        streams[p].warmup_reports_remaining = 0;
        streams[p].postroll_reports_remaining = 0;
        streams[p].fade_in_remaining = 0;
        streams[p].revert_warmup = 0;
        streams[p].revert_postroll = 0;
        streams[p].last_audio_play_ms = 0;
    }

    if (!sbc_initialized) {
        // 16 kHz SBC: the Wii Remote speaker source is ~6 kHz bandwidth, so this
        // is transparent, and one 128-sample SBC frame now carries 8 ms of
        // audio. Two frames per report = 16 ms of audio, so the WiFi/BT radio
        // only has to transmit the audio report every 16 ms instead of 8 ms
        // (halves the airtime stolen from the DS4 input reports). The PS4
        // itself uses 16 kHz when 3+ controllers stream audio.
        btstack_sbc_encoder_init(&sbc_encoder_state, SBC_MODE_STANDARD,
                                 16, 8, SBC_ALLOCATION_METHOD_LOUDNESS,
                                 SBC_SAMPLE_RATE, 25, SBC_CHANNEL_MODE_DUAL_CHANNEL);
        sbc_initialized = true;
    }

    if (!sbc_silence_ready && sbc_initialized) {
        int16_t silent_pcm[256];
        memset(silent_pcm, 0, sizeof(silent_pcm));
        btstack_sbc_encoder_process_data(silent_pcm);
        uint8_t* sbc = btstack_sbc_encoder_sbc_buffer();
        uint16_t sbc_len = btstack_sbc_encoder_sbc_buffer_length();
        if (sbc_len > 112) sbc_len = 112;
        memcpy(sbc_silence_frame, sbc, sbc_len);
        sbc_silence_ready = true;
    }
}

void ds4_audio_reset_player(uint8_t player_idx)
{
    if (player_idx >= DS4_AUDIO_MAX_PLAYERS) return;
    ds4_audio_stream_t* s = &streams[player_idx];
    uint32_t flags = save_and_disable_interrupts();
    s->audio_rb_head = 0;
    s->audio_rb_tail = 0;
    s->revert_rb_tail = 0;
    s->revert_seq = 0;
    s->last_feed_time_ms = 0;
    s->current_volume = 0x40;
    s->ds4_seq_counter = 0;
    s->adpcm_decoder.predictor = 0;
    s->adpcm_decoder.step = 127;
    s->last_input_sample = 0;
    s->resample_phase = 0;
    s->stream_primed = false;
    s->filtered_cushion = TARGET_CUSHION;
    s->warmup_reports_remaining = 0;
    s->postroll_reports_remaining = 0;
    s->fade_in_remaining = 0;
    s->revert_warmup = 0;
    s->revert_postroll = 0;
    s->last_audio_play_ms = 0;
    restore_interrupts(flags);
}

static void resample_and_push_sample(uint8_t p, int16_t sample, uint32_t step)
{
    ds4_audio_stream_t* s = &streams[p];
    while (s->resample_phase < 0x10000) {
        int32_t interp = (int32_t)s->last_input_sample +
                         (((int32_t)(sample - s->last_input_sample) * (int32_t)s->resample_phase) >> 16);
        rb_push(p, av_clip16(interp));
        s->resample_phase += step;
    }
    s->resample_phase -= 0x10000;
    s->last_input_sample = sample;
}

void ds4_audio_feed_report(const uint8_t* data, uint16_t len)
{
    if (!data || len == 0) return;

    if (data[0] == 0x18) {
        data++;
        len--;
    }
    if (len < 4) return;

    uint8_t player_idx = 0;
    uint8_t format = 0;
    uint8_t rate_khz = 3;
    uint8_t vol = 0x40;
    uint16_t payload_len = 0;
    const uint8_t* samples = NULL;

    if (data[0] == 0x19 && len >= 5) {
        format = (data[1] == 0x40 || data[1] == 1) ? 1 : 0;
        rate_khz = data[2];
        vol = data[3];
        payload_len = data[4];
        samples = &data[5];
        if (len < 5 + payload_len) {
            payload_len = (len > 5) ? (len - 5) : 0;
        }
    } else if (len >= 4 && (data[1] >= 1 && data[1] <= 10)) {
        // Wii Fakemote format: [player_and_format, rate_khz, vol, payload_len, samples...]
        player_idx = (data[0] >> 4) & 0x03;
        format = ((data[0] & 0x0F) == 1 || (data[0] & 0x0F) == 0x40) ? 1 : 0;
        rate_khz = data[1];
        vol = data[2];
        payload_len = data[3];
        // The cIOS batches speaker writes into single USB transfers of up to
        // 64 bytes, so a packet can carry up to 59 samples. Keep accepting
        // smaller legacy packets too.
        if (payload_len > 59) payload_len = 59;
        if (len < 4 + payload_len) {
            payload_len = (len > 4) ? (len - 4) : 0;
        }
        samples = &data[4];
    } else {
        payload_len = len;
        samples = data;
    }

    if (payload_len == 0 || !samples) return;
    if (player_idx >= DS4_AUDIO_MAX_PLAYERS) player_idx = 0;

    ds4_audio_stream_t* s = &streams[player_idx];
    ds4_audio_diag_samples_count += payload_len;

    uint32_t now = platform_time_ms();
    if (now - s->last_feed_time_ms > 50) {
        // New sound burst starting: clear ring buffer residue and reset decoder to prevent pops/bops
        s->audio_rb_tail = s->audio_rb_head;
        s->adpcm_decoder.predictor = 0;
        s->adpcm_decoder.step = 127;
        s->last_input_sample = 0;
        s->resample_phase = 0;
        s->stream_primed = false;
        s->fade_in_remaining = 64;
        s->postroll_reports_remaining = 0;
    }
    s->last_feed_time_ms = now;

    // Keep the last non-zero volume: some packets carry 0 while the console
    // is not actually changing the speaker level, and applying them made the
    // DS4 volume dip/jump mid-sound. Same guard as the known-working build.
    if (vol > 0) {
        s->current_volume = vol;
    }

    uint32_t in_rate = (rate_khz >= 1 && rate_khz <= 10) ? (rate_khz * 2000) : 6000;
    uint32_t nominal_step = ((uint32_t)in_rate << 16) / SBC_SAMPLE_RATE;
    if (nominal_step == 0) nominal_step = (6000u << 16) / SBC_SAMPLE_RATE;

    // Adaptive Resampling / Dynamic Clock Drift Compensation:
    // Keeps the ring buffer anchored at TARGET_CUSHION (640 samples = 40ms at
    // 16kHz). The generous cushion absorbs the audio batching of the cIOS (up
    // to 10ms between transfers) plus the Wii/Pico clock mismatch without ever
    // running dry. A 64-sample EMA removes the packet-arrival sawtooth.
    // Deadband of ±192 samples: adj stays 0 (rock-stable pitch).
    // Outside it: gentle correction clamped to ±100 (±0.4% / ~7 cents), well
    // below the Just-Noticeable Difference (JND) for pitch.
    uint32_t step = nominal_step;
    if (s->stream_primed) {
        int32_t current_fill = (s->audio_rb_head - s->audio_rb_tail) & AUDIO_RB_MASK;
        s->filtered_cushion = (s->filtered_cushion * 63 + current_fill) / 64;
        int32_t diff = s->filtered_cushion - TARGET_CUSHION;

        int32_t adj = 0;
        if (diff > 192) {
            adj = (diff - 192) / 12;
            if (adj > 100) adj = 100;
        } else if (diff < -192) {
            adj = (diff + 192) / 12;
            if (adj < -100) adj = -100;
        }
        step = (uint32_t)((int32_t)nominal_step + adj);
    }

    if (format == 0) {
        // Yamaha ADPCM (4-bit per sample, 2 samples per byte)
        for (uint16_t i = 0; i < payload_len; i++) {
            uint8_t b = samples[i];
            int16_t s1 = adpcm_decode_nibble(player_idx, (b >> 4) & 0x0F);
            resample_and_push_sample(player_idx, s1, step);
            int16_t s2 = adpcm_decode_nibble(player_idx, b & 0x0F);
            resample_and_push_sample(player_idx, s2, step);
        }
    } else {
        // PCM 8-bit signed/unsigned
        for (uint16_t i = 0; i < payload_len; i++) {
            int16_t smp = ((int16_t)(int8_t)samples[i]) << 8;
            resample_and_push_sample(player_idx, smp, step);
        }
    }
}

bool ds4_audio_is_player_active(uint8_t player_idx)
{
    if (player_idx >= DS4_AUDIO_MAX_PLAYERS) return false;
    return streams[player_idx].stream_primed || (rb_count(player_idx) >= 128);
}

bool ds4_audio_is_active(void)
{
    return ds4_audio_is_player_active(0);
}

uint8_t ds4_audio_get_player_volume(uint8_t player_idx)
{
    if (player_idx >= DS4_AUDIO_MAX_PLAYERS) return 0x40;
    return streams[player_idx].current_volume;
}

uint8_t ds4_audio_get_volume(void)
{
    return ds4_audio_get_player_volume(0);
}

// Encode one SBC frame into dst.
static void ds4_encode_frame_into(int16_t* pcm, uint8_t* dst)
{
    btstack_sbc_encoder_process_data(pcm);
    uint8_t* sbc = btstack_sbc_encoder_sbc_buffer();
    uint16_t len = btstack_sbc_encoder_sbc_buffer_length();
    if (len > 112) len = 112;
    memcpy(dst, sbc, len);
}

bool ds4_audio_get_player_report_15(uint8_t player_idx, uint8_t* out_buf, size_t max_len,
                                    uint8_t rumble_left, uint8_t rumble_right,
                                    uint8_t r, uint8_t g, uint8_t b)
{
    if (player_idx >= DS4_AUDIO_MAX_PLAYERS) return false;
    ds4_audio_stream_t* s = &streams[player_idx];

    if (!out_buf || max_len < 335) return false;
    if (!sbc_initialized) return false;

    // Audio Priority Arbitration:
    // Player 0 has priority over secondary players. If Player 0 is actively playing audio,
    // suppress Player 1 audio stream and drain its ring buffer to prevent CYW43439 Bluetooth
    // airtime saturation and packet loss.
    if (player_idx > 0 && ds4_audio_is_player_active(0)) {
        s->audio_rb_tail = s->audio_rb_head; // Drain
        s->stream_primed = false;
        return false;
    }

    uint32_t now = platform_time_ms();
    bool host_active = ((now - s->last_feed_time_ms < 100) && s->last_feed_time_ms != 0);

    // Failsafe: If host has stopped feeding audio for > 600ms, force drain and end stream!
    // Prevents holding stream_primed / audio_active indefinitely if packet sends were failing.
    if (!host_active && s->last_feed_time_ms != 0 && (now - s->last_feed_time_ms > 600)) {
        s->stream_primed = false;
        s->postroll_reports_remaining = 0;
        s->audio_rb_tail = s->audio_rb_head;
        return false;
    }

    // Initial cushion: start streaming once cushion reaches PRIMING_THRESHOLD (512 samples = 32ms at 16kHz),
    // or if host stopped feeding and we have at least 256 samples to play out.
    if (!s->stream_primed) {
        if (host_active && rb_count(player_idx) < PRIMING_THRESHOLD) {
            return false;
        }
        if (rb_count(player_idx) < 256) {
            if (!host_active) {
                s->audio_rb_tail = s->audio_rb_head; // Drain
            }
            return false;
        }
        s->stream_primed = true;
        s->filtered_cushion = TARGET_CUSHION;
        s->warmup_reports_remaining = 0;
        s->postroll_reports_remaining = 38;
        s->fade_in_remaining = 64;
    }

    // Only terminate the stream if the host has stopped feeding AND
    // all postroll silence reports are exhausted AND the ring buffer is completely drained.
    if (!host_active && s->postroll_reports_remaining == 0 && rb_count(player_idx) == 0) {
        s->stream_primed = false;
        return false;
    }

    // Snapshot state so it can be restored if L2CAP send fails
    s->revert_rb_tail = s->audio_rb_tail;
    s->revert_seq = s->ds4_seq_counter;
    s->revert_warmup = s->warmup_reports_remaining;
    s->revert_postroll = s->postroll_reports_remaining;

    memset(out_buf, 0, 335);

    // Header for L2CAP interrupt channel
    out_buf[0] = 0xA2;  // HID DATA | OUTPUT

    // Report ID and configuration
    out_buf[1] = 0x15;  // Report ID
    out_buf[2] = 0xC4;  // hw_control: HID | CRC32 | 4ms poll interval
    // audio_control. Bits[1:0] are the microphone input select in Sony's
    // audio-control register (same layout as the DualSense) and MUST stay 0.
    // The old value 0xA2 selected the internal mic, so the DS4 started
    // streaming mic audio back as 0x12..0x19 input packets, saturating the
    // 2.4GHz link and corrupting/starving the 78-byte gamepad reports exactly
    // while the Wii speaker played. 0xA0 is the value used by the working DS4
    // audio scripts (speaker path, no mic).
    out_buf[3] = 0xA0;  // audio_control (mic select = 0)
    out_buf[4] = 0xF7;  // valid flags (rumble 0x01|0x02 + lightbar 0x04 + speaker 0xF0)
    out_buf[5] = 0x04;
    out_buf[6] = 0x00;

    // Balance motors and lightbar to eliminate harsh saturation / overdrive and battery brownout
    out_buf[7] = (uint8_t)(((uint16_t)rumble_right * 140) / 255);  // High freq motor
    out_buf[8] = (uint8_t)(((uint16_t)rumble_left  * 110) / 255);  // Low freq heavy motor
    out_buf[9]  = (uint8_t)(((uint16_t)r * 180) / 255);
    out_buf[10] = (uint8_t)(((uint16_t)g * 180) / 255);
    out_buf[11] = (uint8_t)(((uint16_t)b * 180) / 255);

    // Headphone/mic volumes
    out_buf[22] = 0x38;  // vol left
    out_buf[23] = 0x38;  // vol right
    out_buf[24] = 0x00;  // vol mic

    // Scale speaker volume monotonically to DS4 DAC hardware range (no volume cliff):
    uint8_t spk_vol = 0;
    if (s->current_volume > 0) {
        uint32_t scaled = ((uint32_t)s->current_volume * 100) / 0x40;
        if (scaled > 130) scaled = 130;
        if (scaled < 0x10) scaled = 0x10;
        spk_vol = (uint8_t)scaled;
    }
    out_buf[25] = spk_vol;
    out_buf[26] = 0x85;

    // lilEndianCounter (increments by 2 each report)
    out_buf[79] = (uint8_t)(s->ds4_seq_counter & 0xFF);
    out_buf[80] = (uint8_t)((s->ds4_seq_counter >> 8) & 0xFF);
    s->ds4_seq_counter += 2;

    out_buf[81] = 0x02;  // 0x02 = Built-in Speaker On

    // Encode 2 SBC frames (each frame = 128 samples dual-channel = 256 int16_t)
    static int16_t sbc_pcm[256];

    if (s->warmup_reports_remaining > 0) {
        s->warmup_reports_remaining--;
        // Direct copy of cached silence frame - zero CPU load
        memcpy(&out_buf[82], sbc_silence_frame, 112);
        memcpy(&out_buf[194], sbc_silence_frame, 112);
    } else if (rb_count(player_idx) >= 256) {
        s->postroll_reports_remaining = 38;
        // Frame 1: offset 82 (112 bytes)
        for (int i = 0; i < 128; i++) {
            int16_t smp = rb_pop(player_idx);
            if (s->fade_in_remaining > 0) {
                smp = (int16_t)(((int32_t)smp * (64 - s->fade_in_remaining)) / 64);
                s->fade_in_remaining--;
            }
            sbc_pcm[i * 2]     = smp;
            sbc_pcm[i * 2 + 1] = smp;
        }
        ds4_encode_frame_into(sbc_pcm, &out_buf[82]);

        // Frame 2: offset 82 + 112 = 194 (112 bytes)
        for (int i = 0; i < 128; i++) {
            int16_t smp = rb_pop(player_idx);
            sbc_pcm[i * 2]     = smp;
            sbc_pcm[i * 2 + 1] = smp;
        }
        ds4_encode_frame_into(sbc_pcm, &out_buf[194]);
    } else if (rb_count(player_idx) >= 128) {
        if (!host_active) s->postroll_reports_remaining = 4;
        else s->postroll_reports_remaining = 38;
        // Frame 1: 128 real samples
        int16_t last_s = 0;
        for (int i = 0; i < 128; i++) {
            last_s = rb_pop(player_idx);
            if (s->fade_in_remaining > 0) {
                last_s = (int16_t)(((int32_t)last_s * (64 - s->fade_in_remaining)) / 64);
                s->fade_in_remaining--;
            }
            sbc_pcm[i * 2]     = last_s;
            sbc_pcm[i * 2 + 1] = last_s;
        }
        ds4_encode_frame_into(sbc_pcm, &out_buf[82]);

        // Frame 2: remaining samples with smooth linear decay to avoid clicks
        int rem = rb_count(player_idx);
        if (rem > 128) rem = 128;
        for (int i = 0; i < rem; i++) {
            last_s = rb_pop(player_idx);
            sbc_pcm[i * 2]     = last_s;
            sbc_pcm[i * 2 + 1] = last_s;
        }
        int decay_len = 128 - rem;
        for (int i = 0; i < decay_len; i++) {
            int32_t decayed = ((int32_t)last_s * (decay_len - 1 - i)) / decay_len;
            sbc_pcm[(rem + i) * 2]     = (int16_t)decayed;
            sbc_pcm[(rem + i) * 2 + 1] = (int16_t)decayed;
        }
        ds4_encode_frame_into(sbc_pcm, &out_buf[194]);
    } else if (rb_count(player_idx) > 0) {
        if (!host_active) s->postroll_reports_remaining = 4;
        else s->postroll_reports_remaining = 38;
        // Less than 128 samples left:
        // Drain all remaining samples in Frame 1 with linear decay down to 0,
        // and send silence for Frame 2 so the waveform joins 0 smoothly without popping!
        int rem = rb_count(player_idx);
        int16_t last_s = 0;
        for (int i = 0; i < rem; i++) {
            last_s = rb_pop(player_idx);
            if (s->fade_in_remaining > 0) {
                last_s = (int16_t)(((int32_t)last_s * (64 - s->fade_in_remaining)) / 64);
                s->fade_in_remaining--;
            }
            sbc_pcm[i * 2]     = last_s;
            sbc_pcm[i * 2 + 1] = last_s;
        }
        int decay_len = 128 - rem;
        for (int i = 0; i < decay_len; i++) {
            int32_t decayed = ((int32_t)last_s * (decay_len - 1 - i)) / decay_len;
            sbc_pcm[(rem + i) * 2]     = (int16_t)decayed;
            sbc_pcm[(rem + i) * 2 + 1] = (int16_t)decayed;
        }
        ds4_encode_frame_into(sbc_pcm, &out_buf[82]);

        // Frame 2: direct silence frame
        memcpy(&out_buf[194], sbc_silence_frame, 112);
    } else {
        if (s->postroll_reports_remaining > 0) {
            s->postroll_reports_remaining--;
        }
        // Direct copy of cached silence frame - zero CPU load!
        memcpy(&out_buf[82], sbc_silence_frame, 112);
        memcpy(&out_buf[194], sbc_silence_frame, 112);
    }

    // Calculate CRC-32 over report body [1..330] with seed 0xA2
    uint32_t crc = ds4_audio_bt_crc32(&out_buf[1], 330);
    out_buf[331] = (uint8_t)(crc & 0xFF);
    out_buf[332] = (uint8_t)((crc >> 8) & 0xFF);
    out_buf[333] = (uint8_t)((crc >> 16) & 0xFF);
    out_buf[334] = (uint8_t)((crc >> 24) & 0xFF);

    s->last_audio_play_ms = now;
    return true;
}

bool ds4_audio_get_report_15(uint8_t* out_buf, size_t max_len,
                             uint8_t rumble_left, uint8_t rumble_right,
                             uint8_t r, uint8_t g, uint8_t b)
{
    return ds4_audio_get_player_report_15(0, out_buf, max_len, rumble_left, rumble_right, r, g, b);
}

void ds4_audio_revert_player_report(uint8_t player_idx)
{
    if (player_idx >= DS4_AUDIO_MAX_PLAYERS) return;
    ds4_audio_stream_t* s = &streams[player_idx];
    uint32_t flags = save_and_disable_interrupts();
    s->audio_rb_tail = s->revert_rb_tail;
    s->ds4_seq_counter = s->revert_seq;
    s->warmup_reports_remaining = s->revert_warmup;
    s->postroll_reports_remaining = s->revert_postroll;
    restore_interrupts(flags);
}

void ds4_audio_revert_report(void)
{
    ds4_audio_revert_player_report(0);
}

uint32_t ds4_audio_get_player_last_play_time_ms(uint8_t player_idx)
{
    if (player_idx >= DS4_AUDIO_MAX_PLAYERS) return 0;
    return streams[player_idx].last_audio_play_ms;
}

uint32_t ds4_audio_get_last_play_time_ms(void)
{
    return ds4_audio_get_player_last_play_time_ms(0);
}

#else

void ds4_audio_init(void) {}
void ds4_audio_reset_player(uint8_t player_idx) { (void)player_idx; }
void ds4_audio_feed_report(const uint8_t* data, uint16_t len) { (void)data; (void)len; }
bool ds4_audio_is_active(void) { return false; }
bool ds4_audio_is_player_active(uint8_t player_idx) { (void)player_idx; return false; }
uint8_t ds4_audio_get_volume(void) { return 0; }
uint8_t ds4_audio_get_player_volume(uint8_t player_idx) { (void)player_idx; return 0; }
bool ds4_audio_get_report_15(uint8_t* out_buf, size_t max_len,
                             uint8_t rumble_left, uint8_t rumble_right,
                             uint8_t r, uint8_t g, uint8_t b)
{
    (void)out_buf; (void)max_len; (void)rumble_left; (void)rumble_right;
    (void)r; (void)g; (void)b;
    return false;
}
bool ds4_audio_get_player_report_15(uint8_t player_idx, uint8_t* out_buf, size_t max_len,
                                    uint8_t rumble_left, uint8_t rumble_right,
                                    uint8_t r, uint8_t g, uint8_t b)
{
    (void)player_idx; (void)out_buf; (void)max_len; (void)rumble_left; (void)rumble_right;
    (void)r; (void)g; (void)b;
    return false;
}
void ds4_audio_revert_report(void) {}
void ds4_audio_revert_player_report(uint8_t player_idx) { (void)player_idx; }
uint32_t ds4_audio_get_last_play_time_ms(void) { return 0; }
uint32_t ds4_audio_get_player_last_play_time_ms(uint8_t player_idx) { (void)player_idx; return 0; }

#endif
