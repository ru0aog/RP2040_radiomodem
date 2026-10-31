// src/modes/contestia/contestia_rx.cpp — CONTESTIA 8/250.  
// Порт rx_contestia.cpp из RP2040_receiver (verbatim):  
//   Fs = 15625 Hz, 8 тонов, шаг 31.25 Гц, 500 отсчётов/символ,  
//   32 символа/блок, 3 символа/блок, скремблер 0xEDB88320, nShift=5,  
//   6-битный код Contestia.  
// Адаптации: init принимает uint32_t Гц, rx_decoders_emit -> модем,  
// goertzel из src/rx/.  
#include <Arduino.h>  
#include <math.h>  
#include <stdint.h>  
#include <stdbool.h>  
#include <string.h>  
  
#include "contestia.h"  
#include "../../rx/rx_goertzel.h"  
#include "../../rx/rx_decoders.h"        // rx_decoders_emit  
  
// ---------------------------------------------------------------------------  
// Параметры  
// ---------------------------------------------------------------------------  
  
#define CT_NTONES          8  
#define CT_FS_HZ           15625.0f  
#define CT_STEP_HZ         31.25f  
#define CT_SYMBOL_SAMPLES  500  
  
#define CT_BLOCK_SYMBOLS   32  
#define CT_BLOCK_CHARS     3  
  
#define CT_CENTER_OFFSET_HZ (3.5f * CT_STEP_HZ)  
  
static const uint32_t CT_SCRAMBLE = UINT32_C(0xEDB88320);  
static const uint8_t  CT_NSHIFT   = 5;  
  
// Минимальная корреляция для принятия кодового слова (макс. 32).  
#define CT_MIN_CORRELATION 14  
// Порог обнаружения преамбулы.  
#define CT_SYNC_THRESHOLD  24  
  
// ---------------------------------------------------------------------------  
// Состояние  
// ---------------------------------------------------------------------------  
  
static bool g_active = false;  
static bool g_synced = false;  
  
static float   g_power[CT_NTONES];  
  
static uint8_t g_history[CT_BLOCK_SYMBOLS];  
static uint8_t g_history_pos   = 0;  
static uint8_t g_history_count = 0;  
  
static uint8_t g_block[CT_BLOCK_SYMBOLS];  
static uint8_t g_block_count   = 0;  
  
static uint32_t g_symbols_seen    = 0;  
static uint32_t g_blocks_decoded  = 0;  
  
// ---------------------------------------------------------------------------  
// Gray-код / чётность  
// ---------------------------------------------------------------------------  
  
static inline uint8_t ct_from_gray(uint8_t g)  
{  
    uint8_t v = g;  
    v ^= v >> 1;  
    v ^= v >> 2;  
    v ^= v >> 4;  
    return (uint8_t)(v & 7u);  
}  
  
static inline uint8_t ct_parity(uint32_t x)  
{  
    x ^= x >> 16;  
    x ^= x >> 8;  
    x ^= x >> 4;  
    x &= 0x0Fu;  
    return (uint8_t)((0x6996u >> x) & 1u);  
}  
  
// ---------------------------------------------------------------------------  
// Ожидаемый тон преамбулы: 6 блоков NUL перед передачей.  
// ---------------------------------------------------------------------------  
  
static uint8_t ct_expected_preamble_tone(uint8_t t)  
{  
    uint8_t tone_value = 0;  
  
    for (uint8_t freq_bit = 0; freq_bit < 3; ++freq_bit) {  
        const uint8_t code_bit =  
            (uint8_t)((freq_bit * CT_NSHIFT + t) & 31u);  
  
        const uint8_t scrambled =  
            (uint8_t)((CT_SCRAMBLE >> code_bit) & 1u);  
  
        if (scrambled) {  
            const uint8_t bit_position =  
                (uint8_t)((freq_bit + (t % 3u)) % 3u);  
            tone_value |= (uint8_t)(1u << bit_position);  
        }  
    }  
  
    return (uint8_t)(tone_value ^ (tone_value >> 1));  
}  
  
static bool ct_try_sync(void)  
{  
    if (g_history_count < CT_BLOCK_SYMBOLS) return false;  
  
    unsigned matches = 0;  
    for (uint8_t i = 0; i < CT_BLOCK_SYMBOLS; ++i) {  
        const uint8_t idx = (uint8_t)((g_history_pos + i) & 31u);  
        if (g_history[idx] == ct_expected_preamble_tone(i)) ++matches;  
    }  
    return matches >= CT_SYNC_THRESHOLD;  
}  
  
// ---------------------------------------------------------------------------  
// Декод блока: 3 плоскости → 32 слова Уолша → 3 шестибитных символа.  
// ---------------------------------------------------------------------------  
  
static bool ct_decode_block(  
    const uint8_t tones[CT_BLOCK_SYMBOLS],  
    uint8_t chars[CT_BLOCK_CHARS])  
{  
    uint8_t plane[3][CT_BLOCK_SYMBOLS];  
  
    for (uint8_t t = 0; t < CT_BLOCK_SYMBOLS; ++t) {  
        const uint8_t tone_value = ct_from_gray(tones[t]);  
  
        for (uint8_t freq_bit = 0; freq_bit < 3; ++freq_bit) {  
            const uint8_t bit_position =  
                (uint8_t)((freq_bit + (t % 3u)) % 3u);  
  
            uint8_t bit = (uint8_t)((tone_value >> bit_position) & 1u);  
  
            const uint8_t code_bit =  
                (uint8_t)((freq_bit * CT_NSHIFT + t) & 31u);  
  
            bit ^= (uint8_t)((CT_SCRAMBLE >> code_bit) & 1u);  
            plane[freq_bit][t] = bit;  
        }  
    }  
  
    for (uint8_t freq_bit = 0; freq_bit < 3; ++freq_bit) {  
        int best_corr = -33;  
        uint8_t best_index = 0;  
        bool best_inverted = false;  
  
        for (uint8_t candidate = 0; candidate < 32; ++candidate) {  
            const uint8_t parity_candidate = ct_parity(candidate);  
            int corr = 0;  
  
            for (uint8_t t = 0; t < CT_BLOCK_SYMBOLS; ++t) {  
                const uint8_t expected =  
                    (uint8_t)(ct_parity((uint32_t)candidate & t)  
                              ^ parity_candidate);  
                corr += (plane[freq_bit][t] == expected) ? 1 : -1;  
            }  
  
            if (corr > best_corr) {  
                best_corr = corr;  
                best_index = candidate;  
                best_inverted = false;  
            }  
            if (-corr > best_corr) {  
                best_corr = -corr;  
                best_index = candidate;  
                best_inverted = true;  
            }  
        }  
  
        if (best_corr < CT_MIN_CORRELATION) return false;  
  
        // Старший бит — инверсия кодового слова, младшие 5 — индекс Уолша.  
        chars[freq_bit] =  
            (uint8_t)(best_index | (best_inverted ? 32u : 0u));  
    }  
    return true;  
}  
  
// ---------------------------------------------------------------------------  
// 6-битный код Contestia → символ:  
//   0 -> NUL, 1..58 -> ASCII 33..90, 59 -> ' ', 60 -> CR, 61 -> BS, 31 -> '?'  
// ---------------------------------------------------------------------------  
  
static char ct_decode_character(uint8_t value)  
{  
    value &= 0x3Fu;  
    if (value == 0)                 return '\0';  
    if (value >= 1 && value <= 58)  return (char)(value + 32);  
    if (value == 59)                return ' ';  
    if (value == 60)                return '\r';  
    if (value == 61)                return '\b';  
    if (value == 31)                return '?';  
    return '?';  
}  
  
static void ct_emit_chars(const uint8_t chars[CT_BLOCK_CHARS])  
{  
    if (chars[0] == 0 && chars[1] == 0 && chars[2] == 0) return;  
  
    for (uint8_t i = 0; i < CT_BLOCK_CHARS; ++i) {  
        const char c = ct_decode_character(chars[i]);  
        if (c == '\0' || c == '\b') continue;  
        if (c == '\r' || c == '\n' || c == '\t' ||  
            (c >= 32 && c <= 126)) {  
            rx_decoders_emit(c);  
        }  
    }  
}  
  
static void ct_process_block(void)  
{  
    uint8_t chars[CT_BLOCK_CHARS] = {0, 0, 0};  
    ++g_blocks_decoded;  
    if (!ct_decode_block(g_block, chars)) return;  
    ct_emit_chars(chars);  
}  
  
// ---------------------------------------------------------------------------  
// API  
// ---------------------------------------------------------------------------  
  
void contestia_rx_init(uint32_t center_hz)  
{  
    g_active = false;  
    g_synced = false;  
    g_history_pos = g_history_count = g_block_count = 0;  
    g_symbols_seen = g_blocks_decoded = 0;  
    memset(g_history, 0, sizeof(g_history));  
    memset(g_block, 0, sizeof(g_block));  
    memset(g_power, 0, sizeof(g_power));  
  
    if (center_hz == 0 || center_hz >= (uint32_t)(CT_FS_HZ * 0.5f)) {  
        Serial.println("[CTIA] init: bad center");  
        return;  
    }  
  
    const float base_hz = (float)center_hz - CT_CENTER_OFFSET_HZ;  
    const float last_hz = base_hz + (CT_NTONES - 1) * CT_STEP_HZ;  
  
    if (base_hz < 0.0f || last_hz >= CT_FS_HZ * 0.5f) {  
        Serial.println("[CTIA] init: tones out of band");  
        return;  
    }  
  
    const gb_cfg_t cfg = {  
        CT_NTONES, base_hz, CT_STEP_HZ, CT_FS_HZ, CT_SYMBOL_SAMPLES  
    };  
    gb_init(&cfg);  
  
    Serial.printf("[CTIA] init center=%lu base=%.0f tones=8 step=31.25\n",  
                  (unsigned long)center_hz, (double)base_hz);  
    g_active = true;  
}  
  
void contestia_rx_stop(void)  
{  
    g_active = false;  
    g_synced = false;  
    g_history_pos = g_history_count = g_block_count = 0;  
    memset(g_history, 0, sizeof(g_history));  
    memset(g_block, 0, sizeof(g_block));  
    memset(g_power, 0, sizeof(g_power));  
    // Банк Гёрцеля общий с OLIVIA — не сбрасываем, передаётся другому декодеру.  
}  
  
void contestia_rx_push(int16_t sample)  
{  
    if (!g_active) return;  
  
    // gb_push() возвращает true после накопления 500 отсчётов.  
    if (!gb_push(sample, g_power)) return;  
    ++g_symbols_seen;  
  
    // Жёсткое решение по максимальной мощности одного из восьми тонов.  
    uint8_t best_tone = 0;  
    float best_power = g_power[0];  
    for (uint8_t i = 1; i < CT_NTONES; ++i) {  
        if (g_power[i] > best_power) {  
            best_power = g_power[i];  
            best_tone = i;  
        }  
    }  
  
    g_history[g_history_pos] = best_tone;  
    g_history_pos = (uint8_t)((g_history_pos + 1u) & 31u);  
    if (g_history_count < CT_BLOCK_SYMBOLS) ++g_history_count;  
  
    if (!g_synced) {  
        if (ct_try_sync()) {  
            g_synced = true;  
            g_block_count = 0;  
        }  
        return;  
    }  
  
    g_block[g_block_count++] = best_tone;  
    if (g_block_count >= CT_BLOCK_SYMBOLS) {  
        g_block_count = 0;  
        ct_process_block();  
    }  
}
