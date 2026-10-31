// src/modes/contestia/contestia_rx.cpp — CONTESTIA 8/250.  
// Verbatim-порт RP2040_receiver/src/modes/rx_contestia.cpp:  
//   Fs=15625, 8 тонов шаг 31.25 Гц, символ=500 отсчётов,  
//   блок=32 символа=3 знака, scrambler=0xEDB88320, nShift=5,  
//   6-битный Contestia-код. Общий банк Гёрцеля с OLIVIA.  
// Адаптации: имена contestia_rx_*, init(uint32_t), пути include.  
#include "contestia.h"  
#include "../../rx/rx_goertzel.h"      // gb_init, gb_push, gb_cfg_t  
#include "../../rx/rx_decoders.h"      // rx_decoders_emit  
#include <math.h>  
#include <stdint.h>  
#include <stdbool.h>  
#include <string.h>  
  
#define CT_NTONES          8  
#define CT_FS_HZ           15625.0f  
#define CT_STEP_HZ         31.25f  
#define CT_SYMBOL_SAMPLES  500  
#define CT_BLOCK_SYMBOLS   32  
#define CT_BLOCK_CHARS     3  
#define CT_CENTER_OFFSET_HZ (3.5f * CT_STEP_HZ)  
  
static const uint32_t CT_SCRAMBLE = UINT32_C(0xEDB88320);  
static const uint8_t  CT_NSHIFT   = 5;  
  
#define CT_MIN_CORRELATION 14  
#define CT_SYNC_THRESHOLD  24  
  
// ---- Состояние --------------------------------------------------------------  
static bool g_active = false;  
static bool g_synced = false;  
  
static float   g_power[CT_NTONES];  
static uint8_t g_history[CT_BLOCK_SYMBOLS];  
static uint8_t g_history_pos   = 0;  
static uint8_t g_history_count = 0;  
static uint8_t g_block[CT_BLOCK_SYMBOLS];  
static uint8_t g_block_count   = 0;  
static uint32_t g_symbols_seen = 0;  
static uint32_t g_blocks_decoded = 0;  
  
// ---- Gray-код ---------------------------------------------------------------  
static inline uint8_t ct_from_gray(uint8_t g) {  
    uint8_t v = g;  
    v ^= v >> 1;  
    v ^= v >> 2;  
    v ^= v >> 4;  
    return (uint8_t)(v & 7u);  
}  
  
// ---- Чётность числа единичных битов -----------------------------------------  
static inline uint8_t ct_parity(uint32_t x) {  
    x ^= x >> 16;  
    x ^= x >> 8;  
    x ^= x >> 4;  
    x &= 0x0Fu;  
    return (uint8_t)((0x6996u >> x) & 1u);  
}  
  
// ---- Ожидаемый тон преамбулы -------------------------------------------------  
// В передатчике символ NUL кодируется значением 0; строка Уолша 0 — все знаки 0.  
// Считаем ожидаемые 32 тона с учётом scrambler и Gray-кодирования.  
static uint8_t ct_expected_preamble_tone(uint8_t t) {  
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
    return (uint8_t)(tone_value ^ (tone_value >> 1));   // to Gray  
}  
  
// ---- Поиск повторяющейся преамбулы (шесть NUL-блоков по 32 тона) ------------  
static bool ct_try_sync(void) {  
    if (g_history_count < CT_BLOCK_SYMBOLS) return false;  
  
    unsigned matches = 0;  
    for (uint8_t i = 0; i < CT_BLOCK_SYMBOLS; ++i) {  
        const uint8_t idx = (uint8_t)((g_history_pos + i) & 31u);  
        if (g_history[idx] == ct_expected_preamble_tone(i)) ++matches;  
    }  
    return matches >= CT_SYNC_THRESHOLD;  
}  
  
// ---- Обратное кодирование Contestia -----------------------------------------  
// Три битовых плоскости кодируют три 6-битных знака. Для каждой:  
//  1. Gray-decode тона, снятие скремблера;  
//  2. корреляция с 32 кодовыми словами Уолша (с учётом инверсии);  
//  3. знак = index | (inverted ? 32 : 0).  
static bool ct_decode_block(const uint8_t tones[CT_BLOCK_SYMBOLS],  
                            uint8_t chars[CT_BLOCK_CHARS]) {  
    uint8_t plane[3][CT_BLOCK_SYMBOLS];  
  
    for (uint8_t t = 0; t < CT_BLOCK_SYMBOLS; ++t) {  
        const uint8_t tone_value = ct_from_gray(tones[t]);  
  
        for (uint8_t freq_bit = 0; freq_bit < 3; ++freq_bit) {  
            const uint8_t bit_position =  
                (uint8_t)((freq_bit + (t % 3u)) % 3u);  
            uint8_t bit =  
                (uint8_t)((tone_value >> bit_position) & 1u);  
  
            const uint8_t code_bit =  
                (uint8_t)((freq_bit * CT_NSHIFT + t) & 31u);  
            bit ^= (uint8_t)((CT_SCRAMBLE >> code_bit) & 1u);  
  
            plane[freq_bit][t] = bit;  
        }  
    }  
  
    for (uint8_t freq_bit = 0; freq_bit < 3; ++freq_bit) {  
        int     best_corr     = -33;  
        uint8_t best_index    = 0;  
        bool    best_inverted = false;  
  
        for (uint8_t candidate = 0; candidate < 32; ++candidate) {  
            const uint8_t parity_candidate = ct_parity(candidate);  
            int corr = 0;  
  
            for (uint8_t t = 0; t < CT_BLOCK_SYMBOLS; ++t) {  
                // build_walsh() передатчика:  
                // row bit = parity(candidate & t) XOR parity(candidate)  
                const uint8_t expected =  
                    (uint8_t)(ct_parity((uint32_t)candidate & t)  
                              ^ parity_candidate);  
                corr += (plane[freq_bit][t] == expected) ? 1 : -1;  
            }  
  
            if (corr > best_corr) {  
                best_corr = corr; best_index = candidate; best_inverted = false;  
            }  
            if (-corr > best_corr) {  
                best_corr = -corr; best_index = candidate; best_inverted = true;  
            }  
        }  
  
        if (best_corr < CT_MIN_CORRELATION) return false;  
  
        chars[freq_bit] =  
            (uint8_t)(best_index | (best_inverted ? 32u : 0u));  
    }  
    return true;  
}  
  
// ---- 6-битный код → символ (соответствует contestia_encode передатчика) -----  
static char ct_decode_character(uint8_t value) {  
    value &= 0x3Fu;  
    if (value == 0)                return '\0';  
    if (value >= 1 && value <= 58) return (char)(value + 32);  
    if (value == 59)               return ' ';  
    if (value == 60)               return '\r';  
    if (value == 61)               return '\b';  
    return '?';  
}  
  
// ---- Вывод трёх знаков; блок из трёх NUL не печатаем -------------------------  
static void ct_emit_chars(const uint8_t chars[CT_BLOCK_CHARS]) {  
    if (chars[0] == 0 && chars[1] == 0 && chars[2] == 0) return;  
  
    for (uint8_t i = 0; i < CT_BLOCK_CHARS; ++i) {  
        const char c = ct_decode_character(chars[i]);  
        if (c == '\0' || c == '\b') continue;  
        if (c == '\r' || c == '\n' || c == '\t' ||  
            (c >= 32 && c <= 126)) {  
            rx_decoders_emit((uint8_t)c);  
        }  
    }  
}  
  
static void ct_process_block(void) {  
    uint8_t chars[CT_BLOCK_CHARS] = {0, 0, 0};  
    ++g_blocks_decoded;  
    if (!ct_decode_block(g_block, chars)) return;  
    ct_emit_chars(chars);  
}  
  
// ---- API --------------------------------------------------------------------  
  
void contestia_rx_init(uint32_t center_hz) {  
    g_active = false;  
    g_synced = false;  
    g_history_pos = 0;  
    g_history_count = 0;  
    g_block_count = 0;  
    g_symbols_seen = 0;  
    g_blocks_decoded = 0;  
    memset(g_history, 0, sizeof(g_history));  
    memset(g_block,    0, sizeof(g_block));  
    memset(g_power,    0, sizeof(g_power));  
  
    const float center = (float)center_hz;  
    if (!isfinite(center) || center <= 0.0f ||  
        center >= CT_FS_HZ * 0.5f) {  
        return;  
    }  
  
    const float base_hz = center - CT_CENTER_OFFSET_HZ;  
    const float last_hz = base_hz + (CT_NTONES - 1) * CT_STEP_HZ;  
    if (base_hz < 0.0f || last_hz >= CT_FS_HZ * 0.5f) return;  
  
    const gb_cfg_t cfg = {  
        CT_NTONES, base_hz, CT_STEP_HZ, CT_FS_HZ, CT_SYMBOL_SAMPLES  
    };  
    gb_init(&cfg);  
  
    g_active = true;
    Serial.printf("[CTIA] init center=%ld base=%ld tones=%u step=%ld\n",  
                (unsigned long)center_hz, (long)(base_hz + 0.5f), (unsigned)CT_NTONES,  
                (double)(CT_STEP_HZ * 100.0f));
}  
  
void contestia_rx_stop(void) {  
    g_active = false;  
    g_synced = false;  
    g_history_pos = 0;  
    g_history_count = 0;  
    g_block_count = 0;  
    memset(g_history, 0, sizeof(g_history));  
    memset(g_block,    0, sizeof(g_block));  
    memset(g_power,    0, sizeof(g_power));  
    // Банк Гёрцеля не сбрасываем — он разделяемый между декодерами.  
}  
  
bool contestia_rx_active(void) { return g_active; }  
  
void contestia_rx_push(int16_t sample) {  
    if (!g_active) return;  
  
    // gb_push возвращает true каждые 500 отсчётов.  
    if (!gb_push(sample, g_power)) return;  
  
    ++g_symbols_seen;  
  
    // Жёсткое решение по максимальной мощности одного из 8 тонов.  
    uint8_t best_tone = 0;  
    float   best_power = g_power[0];  
    for (uint8_t i = 1; i < CT_NTONES; ++i) {  
        if (g_power[i] > best_power) {  
            best_power = g_power[i];  
            best_tone = i;  
        }  
    }  
  
    // Кольцевая история для поиска преамбулы.  
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