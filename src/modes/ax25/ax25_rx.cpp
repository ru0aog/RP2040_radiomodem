// src/modes/ax25/ax25_rx.cpp — приём: корреляторы → дискриминатор →  
// слайсер → DPLL → NRZI → destuff → FCS, вход моно 15625 sps.  
// Порт rx_ax25.cpp из RP2040_receiver (verbatim, только имена API).  
#include "ax25.h"  
#include "../../rx/rx_decoders.h"          // rx_decoders_emit, rx_decoders_mode  
#include <Arduino.h>  
#include <math.h>  
  
// ---------------- Конфигурация тракта ----------------  
#define AX25_FS         15625.0f  
#define AX25_BAUD       300.0f  
#define AX25_SHIFT_HZ   200.0f  
#define AX25_HYST_FRAC  0.10f  
  
// DPLL (фазовый аккумулятор 32 бита)  
#define PLL_STEP        (uint32_t)(4294967296.0 * AX25_BAUD / AX25_FS)  
#define PLL_LOCKED      0.05f      // узкий полюс — синхронизирован  
#define PLL_SEARCHING   0.30f      // широкий полюс — поиск  
#define PLL_GOOD_BITS   240        // ~80 бит подряд «хороших» → locked  
  
#define AX25_MAX_FRAME  330        // байт в буфере кадра  
#define AX25_FCS_MAGIC  0xF0B8     // остаток CRC-CCITT при верном FCS  
  
// ---------------- Состояние ----------------  
static ax25_frame_cb_t s_cb = nullptr;   // внешний колбэк (если задан)  
  
static bool     g_active = false;  
static uint32_t g_center_hz = 0;  
  
// Корреляторы mark/space: exp(-j*2π*f/fs) рекуррентное умножение  
static float    g_cos_mark,  g_sin_mark;  
static float    g_cos_space, g_sin_space;  
static float    g_corr_mark_i = 0.0f, g_corr_mark_q = 0.0f;  
static float    g_corr_sp_i   = 0.0f, g_corr_sp_q   = 0.0f;  
static uint8_t  g_corr_phase  = 0;  
static float    g_avg_mark = 0.0f, g_avg_space = 0.0f;  
  
// Дискриминатор + слайсер  
static int8_t   g_sliced = 0;            // 0 = mark, 1 = space  
static float    g_hyst_hi, g_hyst_lo;  
static uint8_t  g_prev_tone = 0;  
  
// DPLL  
static uint32_t g_dpll      = 0;  
static uint32_t g_dpll_step = PLL_STEP;  
static uint32_t g_pll_quality = 0;       // сколько «хороших» переходов подряд  
  
// HDLC/AX.25  
static uint32_t g_bitbuf   = 0;  
static uint8_t  g_bitcnt   = 0;  
static uint8_t  g_ones_cnt = 0;  
static uint8_t  g_flag_ok  = 0;          // принят флаг 0x7E  
static uint8_t  g_frame[AX25_MAX_FRAME];  
static uint16_t g_frame_len = 0;  
static uint16_t g_fcs       = 0xFFFF;  
static bool     g_in_frame  = false;  
  
// ---------------- Внутренние функции ----------------  
  
// CRC-CCITT (0x8408 reversed), посчитан над одним битом  
static uint16_t ax25_fcs_update(uint16_t crc, uint8_t bit) {  
    crc ^= (bit & 1);  
    return (crc & 1) ? (crc >> 1) ^ 0x8408 : (crc >> 1);  
}  
  
// Выдать принятый байт в накопитель кадра  
static void ax25_frame_byte(uint8_t b) {  
    if (g_frame_len < AX25_MAX_FRAME) {  
        g_frame[g_frame_len++] = b;  
    }  
}  
  
// Печать/сброс готового кадра (FCS проверен)  
static void ax25_frame_done(bool ok) {  
    if (g_in_frame && g_frame_len >= 18 && ok) {  
        // Кадр: [addr(14+)][ctrl][pid][info...][FCS(2)]  
        if (s_cb) {  
            s_cb(g_frame, g_frame_len);      // внешний колбэк  
        } else {  
            // Дефолтный разбор: позывные + info через rx_decoders_emit  
            auto emit = [](char c){ rx_decoders_emit((uint8_t)c); };  
            // Адресаты: src callsign — байты 7..12 (сдвиг >>1, 6 символов)  
            char src[8]; char dst[8];  
            for (int i = 0; i < 6; i++) {  
                dst[i] = (char)(g_frame[i]   >> 1);  
                src[i] = (char)(g_frame[7+i] >> 1);  
            }  
            dst[6] = 0; src[6] = 0;  
            for (const char *p = dst; *p; p++) rx_decoders_emit((uint8_t)*p);  
            rx_decoders_emit('>');  
            for (const char *p = src; *p; p++) rx_decoders_emit((uint8_t)*p);
            emit(':');  
            // info: после адресного поля (14 байт) + ctrl + pid  
            uint16_t info_off = 16;  
            for (uint16_t i = info_off; i + 2 < g_frame_len; i++) {  
                char c = (char)g_frame[i];  
                if (c >= 32 && c < 127) emit(c);  
            }  
            rx_decoders_emit('\n');  
        }  
    }  
    g_in_frame  = false;  
    g_frame_len = 0;  
    g_fcs       = 0xFFFF;  
    g_bitcnt    = 0;  
    g_ones_cnt  = 0;  
    g_bitbuf    = 0;  
}  
  
// Обработчик одного бита HDLC (NRZI-дескремблированный)  
static void ax25_hdlc_bit(uint8_t bit) {  
    // Bit stuffing: после 5 единиц идёт stuffed-ноль — выкинуть  
    if (g_ones_cnt == 5) {  
        if (bit == 0) {              // stuffed bit  
            g_ones_cnt = 0;  
            return;  
        }  
        // 6 единиц = флаг 0x7E, 7+ = abort/idle  
        g_ones_cnt++;  
        if (g_ones_cnt == 6) {  
            // Флаг — конец/начало кадра  
            if (g_in_frame) {  
                // Проверить FCS по последним 16 битам  
                ax25_frame_done(g_fcs == AX25_FCS_MAGIC);  
            }  
            g_flag_ok  = 1;  
            g_in_frame = true;  
            g_frame_len = 0;  
            g_fcs = 0xFFFF;  
            g_bitbuf = 0; g_bitcnt = 0;  
            g_ones_cnt = 0;  
            return;  
        }  
        // 7+ единиц — abort  
        if (g_ones_cnt >= 7) {  
            g_in_frame = false;  
            g_flag_ok  = 0;  
            g_ones_cnt = 0;  
            return;  
        }  
    }  
  
    g_ones_cnt = (bit) ? (uint8_t)(g_ones_cnt + 1) : 0;  
  
    if (!g_in_frame) return;  
  
    g_bitbuf = (g_bitbuf >> 1) | ((uint32_t)(bit & 1) << 31);  
    if (++g_bitcnt == 8) {  
        uint8_t b = (uint8_t)(g_bitbuf >> 24);  
        // FCS считаем по всем байтам кадра (включая FCS-байты;  
        // при верной посылке итог равен AX25_FCS_MAGIC)  
        for (int i = 0; i < 8; i++)  
            g_fcs = ax25_fcs_update(g_fcs, (b >> i) & 1);  
        ax25_frame_byte(b);  
        g_bitbuf = 0; g_bitcnt = 0;  
    }  
}  
  
// ---------------- Публичный API ----------------  
  
void ax25_rx_init(uint32_t center_hz) {  
    if (center_hz == 0) center_hz = 1500;    // дефолт — центр RSID-полосы  
  
    g_center_hz = center_hz;  
    const float f_mark  = (float)center_hz - AX25_SHIFT_HZ * 0.5f;  
    const float f_space = (float)center_hz + AX25_SHIFT_HZ * 0.5f;  
  
    g_cos_mark  = cosf(-2.0f * (float)M_PI * f_mark  / AX25_FS);  
    g_sin_mark  = sinf(-2.0f * (float)M_PI * f_mark  / AX25_FS);  
    g_cos_space = cosf(-2.0f * (float)M_PI * f_space / AX25_FS);  
    g_sin_space = sinf(-2.0f * (float)M_PI * f_space / AX25_FS);  
  
    const float hyst = AX25_SHIFT_HZ * AX25_HYST_FRAC;  
    g_hyst_hi =  hyst;  
    g_hyst_lo = -hyst;  
  
    g_corr_mark_i = g_corr_mark_q = 0.0f;  
    g_corr_sp_i   = g_corr_sp_q   = 0.0f;  
    g_corr_phase  = 0;  
    g_avg_mark = g_avg_space = 0.0f;  
  
    g_sliced = 0; g_prev_tone = 0;  
    g_dpll = 0; g_dpll_step = PLL_STEP; g_pll_quality = 0;  
    g_in_frame = false; g_flag_ok = 0;  
    g_frame_len = 0; g_fcs = 0xFFFF;  
    g_bitbuf = 0; g_bitcnt = 0; g_ones_cnt = 0;  
  
    g_active = true;  
  
    Serial.printf("[AX25] init center=%lu mark=%lu space=%lu\n",  
                  (unsigned long)center_hz,  
                  (unsigned long)f_mark, (unsigned long)f_space);  
}  
  
void ax25_rx_stop(void) {  
    g_active   = false;  
    g_in_frame = false;  
    g_frame_len = 0;  
}  
  
void ax25_rx_push(int16_t m) {  
    if (!g_active) return;  
    const float x = (float)m;  
  
    // ---- Коррелятор mark (окно = 1/300 бод ≈ 52 сэмпла) ----  
    {  
        float i = g_corr_mark_i * g_cos_mark - g_corr_mark_q * g_sin_mark + x;  
        float q = g_corr_mark_i * g_sin_mark + g_corr_mark_q * g_cos_mark;  
        g_corr_mark_i = i; g_corr_mark_q = q;  
    }  
    // ---- Коррелятор space ----  
    {  
        float i = g_corr_sp_i * g_cos_space - g_corr_sp_q * g_sin_space + x;  
        float q = g_corr_sp_i * g_sin_space + g_corr_sp_q * g_cos_space;  
        g_corr_sp_i = i; g_corr_sp_q = q;  
    }  
  
    if (++g_corr_phase < (uint8_t)(AX25_FS / AX25_BAUD)) return;  
    g_corr_phase = 0;  
  
    // Мощности за окно символа  
    const float p_mark  = g_corr_mark_i * g_corr_mark_i  
                        + g_corr_mark_q * g_corr_mark_q;  
    const float p_space = g_corr_sp_i * g_corr_sp_i  
                        + g_corr_sp_q * g_corr_sp_q;  
    g_corr_mark_i = g_corr_mark_q = 0.0f;  
    g_corr_sp_i   = g_corr_sp_q   = 0.0f;  
  
    // Скользящее среднее для адаптивного порога  
    g_avg_mark  += (p_mark  - g_avg_mark)  * 0.05f;  
    g_avg_space += (p_space - g_avg_space) * 0.05f;  
  
    // ---- Дискриминатор с гистерезисом ----  
    const float d = p_space - p_mark;  
    uint8_t tone = g_sliced;  
    if (d > g_hyst_hi)       tone = 1;   // space  
    else if (d < g_hyst_lo)  tone = 0;   // mark  
  
    // NRZI: переход = 0, отсутствие = 1  
    const uint8_t bit = (tone == g_prev_tone) ? 1 : 0;  
    g_sliced = tone;  
  
    // Качество синхронизации: переходы «на месте» → качество растёт  
    if (tone != g_prev_tone) g_pll_quality = 0;  
    else if (g_pll_quality < 0xFFFFFFFFu) g_pll_quality++;  
  
    // ---- DPLL: строб бита на переносе фазы ----  
    const uint32_t prev_phase = g_dpll;  
    g_dpll += g_dpll_step;  
    if (g_dpll < prev_phase) ax25_hdlc_bit(bit);  
  
    // Коррекция фазы по переходу (переход должен быть на фронте символа)  
    if (tone != g_prev_tone) {  
        const int32_t phase_err = (int32_t)(g_dpll - 0x80000000u);  
        const float gain = (g_pll_quality >= PLL_GOOD_BITS) ? PLL_LOCKED  
                                                            : PLL_SEARCHING;  
        int64_t corr = (int64_t)((double)gain * (double)phase_err);  
        const int64_t max_corr = (int64_t)1 << 30;  
        if (corr >  max_corr) corr =  max_corr;  
        if (corr < -max_corr) corr = -max_corr;  
        g_dpll -= (uint32_t)corr;  
    }  
    g_prev_tone = tone;  
}  
  
void ax25_rx_set_frame_cb(ax25_frame_cb_t cb) { s_cb = cb; }