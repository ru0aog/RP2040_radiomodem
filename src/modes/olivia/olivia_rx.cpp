// src/modes/olivia/olivia_rx.cpp — OLIVIA 8/250 RX.  
// Порт rx_olivia.cpp из RP2040_receiver (verbatim, только имена API).  
//  
// Fs=15625 Гц, 8 тонов, шаг 31.25 Гц, символ 500 отсчётов,  
// блок 64 тона → 3 символа. Walsh-Hadamard SPB=64,  
// Scramble=0xE257E6D0291574EC, nShift=13, 7-бит ASCII, Gray-тоны.  
#include "olivia.h"  
#include "../../dsp/rx_goertzel.h"  
#include <math.h>  
#include <string.h>  
  
#define OLV_DEBUG 0  
  
#define OLV_NTONES          8  
#define OLV_FS_HZ           15625.0f  
#define OLV_STEP_HZ         31.25f  
#define OLV_SYMBOL_SAMPLES  500  
#define OLV_BLOCK_SYMBOLS   64  
#define OLV_BLOCK_CHARS     3  
#define OLV_CENTER_OFFSET   (3.5f * OLV_STEP_HZ)  
  
static const uint64_t OLV_SCRAMBLE = UINT64_C(0xE257E6D0291574EC);  
static const uint8_t  OLV_NSHIFT   = 13;  
  
// 40 совпадений из 64 — порог поиска преамбулы.  
#define OLV_SYNC_THRESHOLD 40  
  
static bool g_active = false;  
static bool g_synced = false;  
  
static uint8_t g_tone_history[OLV_BLOCK_SYMBOLS];  
static uint8_t g_history_pos   = 0;  
static uint8_t g_history_count = 0;  
  
static uint8_t g_block_tones[OLV_BLOCK_SYMBOLS];  
static uint8_t g_block_count = 0;  
  
static uint32_t g_symbols_seen    = 0;  
static uint32_t g_blocks_decoded  = 0;  
static uint32_t g_sync_count      = 0;  
  
static float g_power[OLV_NTONES];  
  
static inline uint8_t olv_to_gray(uint8_t v) {  
    return (uint8_t)(v ^ (v >> 1));  
}  
  
static inline uint8_t olv_from_gray(uint8_t g) {  
    uint8_t v = g;  
    v ^= v >> 1;  
    v ^= v >> 2;  
    v ^= v >> 4;  
    return (uint8_t)(v & 7u);  
}  
  
static inline uint8_t olv_parity(uint32_t x) {  
    x ^= x >> 16;  
    x ^= x >> 8;  
    x ^= x >> 4;  
    x &= 0x0Fu;  
    return (uint8_t)((0x6996u >> x) & 1u);  
}  
  
// Ожидаемый тон преамбулы (передатчик: 6 блоков × 3 NUL).  
static uint8_t olv_expected_preamble_tone(uint8_t t) {  
    uint8_t tone_bits = 0;  
    for (uint8_t freq_bit = 0; freq_bit < 3; ++freq_bit) {  
        uint8_t code_bit     = (uint8_t)((freq_bit * OLV_NSHIFT + t) & 63u);  
        uint8_t scramble_bit = (uint8_t)((OLV_SCRAMBLE >> code_bit) & 1u);  
        if (scramble_bit) {  
            uint8_t bit_position = (uint8_t)((freq_bit + (t % 3u)) % 3u);  
            tone_bits |= (uint8_t)(1u << bit_position);  
        }  
    }  
    return olv_to_gray(tone_bits);  
}  
  
static bool olv_try_sync(void) {  
    if (g_history_count < OLV_BLOCK_SYMBOLS) return false;  
    unsigned matches = 0;  
    for (uint8_t i = 0; i < OLV_BLOCK_SYMBOLS; ++i) {  
        uint8_t idx = (uint8_t)((g_history_pos + i) & 63u);  
        if (g_tone_history[idx] == olv_expected_preamble_tone(i)) ++matches;  
    }  
    return matches >= OLV_SYNC_THRESHOLD;  
}  
  
static bool olv_decode_block(const uint8_t tones[OLV_BLOCK_SYMBOLS],  
                             uint8_t chars[OLV_BLOCK_CHARS]) {  
    uint8_t row_bits[3][64];  
  
    for (uint8_t t = 0; t < 64; ++t) {  
        uint8_t tone_value = olv_from_gray(tones[t]);  
        for (uint8_t freq_bit = 0; freq_bit < 3; ++freq_bit) {  
            uint8_t bit_position = (uint8_t)((freq_bit + (t % 3u)) % 3u);  
            uint8_t bit = (uint8_t)((tone_value >> bit_position) & 1u);  
            uint8_t code_bit = (uint8_t)((freq_bit * OLV_NSHIFT + t) & 63u);  
            bit ^= (uint8_t)((OLV_SCRAMBLE >> code_bit) & 1u);  
            row_bits[freq_bit][t] = bit;  
        }  
    }  
  
    for (uint8_t freq_bit = 0; freq_bit < 3; ++freq_bit) {  
        int best_corr = -65;  
        uint8_t best_index = 0;  
        bool best_inverted = false;  
  
        for (uint8_t candidate = 0; candidate < 64; ++candidate) {  
            int corr = 0;  
            const uint8_t parity_candidate = olv_parity(candidate);  
            for (uint8_t t = 0; t < 64; ++t) {  
                uint8_t expected =  
                    (uint8_t)(olv_parity((uint32_t)candidate & t) ^ parity_candidate);  
                corr += (row_bits[freq_bit][t] == expected) ? 1 : -1;  
            }  
            if (corr > best_corr) {  
                best_corr = corr; best_index = candidate; best_inverted = false;  
            }  
            if (-corr > best_corr) {  
                best_corr = -corr; best_index = candidate; best_inverted = true;  
            }  
        }  
  
        if (best_corr < 24) return false;   // повреждённый блок — не выводим мусор  
  
        uint8_t value = best_index;  
        if (best_inverted) value |= 0x40u;  
        chars[freq_bit] = value & 0x7Fu;  
    }  
    return true;  
}  
  
static void olv_emit_chars(const uint8_t chars[OLV_BLOCK_CHARS]) {  
    for (uint8_t i = 0; i < OLV_BLOCK_CHARS; ++i) {  
        const uint8_t c = chars[i];  
        if (c == 0) continue;  
        if (c == '\r' || c == '\n' || c == '\t' || (c >= 32 && c <= 126)) {  
            Serial.write((char)c);  
        }  
    }  
}  
  
static void olv_process_block(void) {  
    uint8_t chars[OLV_BLOCK_CHARS] = {0, 0, 0};  
    ++g_blocks_decoded;  
    if (!olv_decode_block(g_block_tones, chars)) return;  
    olv_emit_chars(chars);  
}  
  
void olivia_rx_init(uint32_t center_hz) {  
    g_active = false;  
    g_synced = false;  
    g_history_pos = g_history_count = g_block_count = 0;  
    g_symbols_seen = g_blocks_decoded = g_sync_count = 0;  
    memset(g_tone_history, 0, sizeof(g_tone_history));  
    memset(g_block_tones, 0, sizeof(g_block_tones));  
    memset(g_power, 0, sizeof(g_power));  
  
    if (center_hz == 0) center_hz = 1500;  
    if (center_hz >= (uint32_t)(OLV_FS_HZ * 0.5f)) return;  
  
    // RSID указывает центр полосы; нижний тон на 3.5 шага ниже.  
    const float base_hz = (float)center_hz - OLV_CENTER_OFFSET;  
    const float last_hz = base_hz + (OLV_NTONES - 1) * OLV_STEP_HZ;  
    if (base_hz < 0.0f || last_hz >= OLV_FS_HZ * 0.5f) return;  
  
    const gb_cfg_t cfg = {  
        OLV_NTONES, base_hz, OLV_STEP_HZ, OLV_FS_HZ, OLV_SYMBOL_SAMPLES  
    };  
    gb_init(&cfg);  
  
    g_active = true;  
    Serial.printf("[OLV] init center=%lu base=%ld tones=%d step=%ld.%02ld\n",  
                  (unsigned long)center_hz,  
                  (long)(base_hz + 0.5f), (int)OLV_NTONES,  
                  (long)OLV_STEP_HZ,  
                  (long)(OLV_STEP_HZ * 100.0f + 0.5f) % 100);  
}  
  
void olivia_rx_stop(void) {  
    g_active = false;  
    g_synced = false;  
    g_history_pos = g_history_count = g_block_count = 0;  
    memset(g_tone_history, 0, sizeof(g_tone_history));  
    memset(g_block_tones, 0, sizeof(g_block_tones));  
    memset(g_power, 0, sizeof(g_power));  
    // gb_reset() не зовём — банк переинициализируется следующей модой.  
}  
  
void olivia_rx_push(int16_t sample) {  
    if (!g_active) return;  
  
    if (!gb_push(sample, g_power)) return;   // окно 500 отсчётов  
  
    ++g_symbols_seen;  
  
    uint8_t best_tone = 0;  
    float best_power = g_power[0];  
    for (uint8_t i = 1; i < OLV_NTONES; ++i) {  
        if (g_power[i] > best_power) { best_power = g_power[i]; best_tone = i; }  
    }  
  
    g_tone_history[g_history_pos] = best_tone;  
    g_history_pos = (uint8_t)((g_history_pos + 1u) & 63u);  
    if (g_history_count < OLV_BLOCK_SYMBOLS) ++g_history_count;  
  
    if (!g_synced) {  
        if (olv_try_sync()) {  
            g_synced = true;  
            ++g_sync_count;  
            g_block_count = 0;  
#if OLV_DEBUG  
            Serial.printf("[OLV] sync seen=%lu\n", (unsigned long)g_symbols_seen);  
#endif  
        }  
        return;  
    }  
  
    g_block_tones[g_block_count++] = best_tone;  
    if (g_block_count >= OLV_BLOCK_SYMBOLS) {  
        g_block_count = 0;  
        olv_process_block();  
    }  
}
