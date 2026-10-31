// src/modes/ax25/ax25_rx.cpp — приём: корреляторы → дискриминатор →  
// слайсер → DPLL → NRZI → destuff → FCS, вход моно 15625 sps.  
// Verbatim-порт rx_ax25.cpp из RP2040_receiver.  
// Адаптации: center_hz uint32_t Гц, вывод через Serial (rx_decoders_emit  
// в модеме нет), debug-printf без %f.  
#include "ax25.h"  
#include <Arduino.h>  
#include <stdint.h>  
#include <math.h>  
  
// -----------------------------------------------------------------------------  
// Configuration  
// -----------------------------------------------------------------------------  
  
static constexpr float AX25_FS       = 15625.0f;  
static constexpr float AX25_BAUD     = 300.0f;  
static constexpr float AX25_LP_ALPHA = 0.04f;  
  
static constexpr float AX25_SHIFT_HZ      = 200.0f;  
static constexpr float AX25_HYST_FRACTION = 0.10f;  
  
static constexpr int AX25_MAX_FRAME = 330;  
  
static constexpr uint16_t AX25_FCS_MAGIC = 0xF0B8;  
  
#define AX25_DEBUG 0  
#define AX25_FIXED_MIDPOINT 1  
  
static constexpr float PLL_SEARCHING = 0.25f;  
static constexpr float PLL_LOCKED    = 0.10f;  
static constexpr int PLL_GOOD_BITS   = 8;  
  
// -----------------------------------------------------------------------------  
// Diagnostics  
// -----------------------------------------------------------------------------  
  
#if AX25_DEBUG  
static uint32_t dbg_samples = 0;  
static uint32_t dbg_bits = 0;  
static uint32_t dbg_flags = 0;  
static uint32_t dbg_stuff = 0;  
static uint32_t dbg_abort = 0;  
static uint32_t dbg_fcs_bad = 0;  
static uint32_t dbg_fcs_ok = 0;  
static uint32_t dbg_trans = 0;  
  
static uint32_t dbg_run_sum = 0;  
static uint32_t dbg_run_n = 0;  
  
static float dbg_dmin = 1.0e30f;  
static float dbg_dmax = -1.0e30f;  
#endif  
  
// -----------------------------------------------------------------------------  
// Tone correlators  
// -----------------------------------------------------------------------------  
  
struct ToneCorrelator {  
    float osc_re;  
    float osc_im;  
  
    float rot_re;  
    float rot_im;  
  
    float lp_re;  
    float lp_im;  
};  
  
static ToneCorrelator g_mark;  
static ToneCorrelator g_space;  
  
static void tone_init(ToneCorrelator* t, float frequency_hz)  
{  
    t->osc_re = 1.0f;  
    t->osc_im = 0.0f;  
  
    const float w =  
        2.0f * (float)M_PI * frequency_hz / AX25_FS;  
  
    t->rot_re = cosf(w);  
    t->rot_im = sinf(w);  
  
    t->lp_re = 0.0f;  
    t->lp_im = 0.0f;  
}  
  
static float tone_push(ToneCorrelator* t, float x)  
{  
    const float yr = x * t->osc_re;  
    const float yi = x * t->osc_im;  
  
    const float nr =  
        t->osc_re * t->rot_re -  
        t->osc_im * t->rot_im;  
  
    const float ni =  
        t->osc_re * t->rot_im +  
        t->osc_im * t->rot_re;  
  
    t->osc_re = nr;  
    t->osc_im = ni;  
  
    t->lp_re += AX25_LP_ALPHA * (yr - t->lp_re);  
    t->lp_im += AX25_LP_ALPHA * (yi - t->lp_im);  
  
    return t->lp_re * t->lp_re +  
           t->lp_im * t->lp_im;  
}  
  
// -----------------------------------------------------------------------------  
// Receiver state  
// -----------------------------------------------------------------------------  
  
static bool g_active = false;  
  
static float g_peak = 100.0f;  
static float g_valley = -100.0f;  
  
static uint8_t g_tone_state = 0;  
  
static uint32_t g_dpll = 0;  
static uint32_t g_dpll_step = 0;  
  
static int g_pll_quality = 0;  
  
static uint8_t g_prev_tone = 0;  
static uint8_t g_prev_raw = 0;  
  
// -----------------------------------------------------------------------------  
// HDLC state  
//  
// Восьмибитное скользящее окно задерживает выдачу в декодер данных.  
// Кандидат на флаг распознаётся до того, как его биты попадут  
// в destuffer или сборщик байтов.  
//  
// g_in_abort:  
//   true после семи подряд единиц данных;  
//   данные игнорируются до следующего флага.  
// -----------------------------------------------------------------------------  
  
static uint8_t g_hdlc_window = 0;  
static uint8_t g_window_bits = 0;  
  
static uint8_t g_ones = 0;  
static bool g_in_abort = false;  
  
// Сборка байтов, LSB-first.  
static uint8_t g_oacc = 0;  
static uint8_t g_olen = 0;  
  
// Буфер кадра.  
static uint8_t g_frame_buf[AX25_MAX_FRAME];  
static int g_frame_len = 0;  
  
// -----------------------------------------------------------------------------  
// CRC-16/X.25  
// -----------------------------------------------------------------------------  
  
static uint16_t ax25_fcs_update(uint16_t crc, uint8_t b)  
{  
    crc ^= b;  
  
    for (int i = 0; i < 8; ++i) {  
        if (crc & 1u) {  
            crc = (uint16_t)((crc >> 1) ^ 0x8408u);  
        } else {  
            crc = (uint16_t)(crc >> 1);  
        }  
    }  
  
    return crc;  
}  
  
// -----------------------------------------------------------------------------  
// Вывод: в модеме нет rx_decoders_emit — печатаем в Serial.  
// -----------------------------------------------------------------------------  
  
static inline void ax25_emit(char c) { Serial.write((uint8_t)c); }  
  
static void ax25_emit_callsign(const uint8_t* p)  
{  
    for (int i = 0; i < 6; ++i) {  
        const char c = (char)(p[i] >> 1);  
  
        if (c != ' ') {  
            ax25_emit(c);  
        }  
    }  
  
    const int ssid = (p[6] >> 1) & 0x0F;  
  
    if (ssid != 0) {  
        ax25_emit('-');  
  
        if (ssid >= 10) {  
            ax25_emit((char)('0' + ssid / 10));  
        }  
  
        ax25_emit((char)('0' + ssid % 10));  
    }  
}  
  
// -----------------------------------------------------------------------------  
// Сброс текущей сборки кадра  
// -----------------------------------------------------------------------------  
  
static void ax25_frame_reset(void)  
{  
    g_frame_len = 0;  
  
    g_oacc = 0;  
    g_olen = 0;  
  
    g_ones = 0;  
}  
  
// -----------------------------------------------------------------------------  
// Обработка бита, который точно не часть флага  
//  
// HDLC bit stuffing: после пяти подряд единиц передатчик вставляет ноль;  
// приёмник его отбрасывает.  
// HDLC abort: семь подряд единиц — обрыв кадра, затем игнор до флага.  
// -----------------------------------------------------------------------------  
  
static void ax25_process_data_bit(uint8_t dbit)  
{  
    if (g_in_abort) {  
        return;  
    }  
  
    if (dbit != 0) {  
        if (g_ones < 7) {  
            ++g_ones;  
        }  
  
        if (g_ones >= 7) {  
#if AX25_DEBUG  
            ++dbg_abort;  
#endif  
            ax25_frame_reset();  
            g_in_abort = true;  
            return;  
        }  
    } else {  
        if (g_ones == 5) {  
            // Вставленный ноль — отбросить.  
#if AX25_DEBUG  
            ++dbg_stuff;  
#endif  
            g_ones = 0;  
            return;  
        }  
  
        g_ones = 0;  
    }  
  
    // Сборка de-stuffed потока, LSB-first.  
    g_oacc = (uint8_t)(g_oacc >> 1);  
  
    if (dbit != 0) {  
        g_oacc |= 0x80;  
    }  
  
    ++g_olen;  
  
    if (g_olen == 8) {  
        g_olen = 0;  
  
        if (g_frame_len >= 0 &&  
            g_frame_len < AX25_MAX_FRAME) {  
            g_frame_buf[g_frame_len++] = g_oacc;  
        } else {  
            g_frame_len = -1;  
        }  
  
        g_oacc = 0;  
    }  
}  
  
// -----------------------------------------------------------------------------  
// Завершение и проверка кадра AX.25  
// -----------------------------------------------------------------------------  
  
static void ax25_frame_finish(void)  
{  
    if (g_frame_len < 18) {  
        return;  
    }  
  
    if (g_frame_len < 0) {  
        return;  
    }  
  
    uint16_t crc = 0xFFFF;  
  
    for (int i = 0; i < g_frame_len; ++i) {  
        crc = ax25_fcs_update(crc, g_frame_buf[i]);  
    }  
  
    if (crc != AX25_FCS_MAGIC) {  
#if AX25_DEBUG  
        ++dbg_fcs_bad;  
  
        Serial.printf(  
            "# AX25 FCS bad len=%d res=%04X bytes: ",  
            g_frame_len,  
            crc  
        );  
  
        for (int i = 0; i < g_frame_len; ++i) {  
            Serial.printf("%02X", g_frame_buf[i]);  
        }  
  
        Serial.println();  
#endif  
        return;  
    }  
  
#if AX25_DEBUG  
    ++dbg_fcs_ok;  
#endif  
  
    if (g_pll_quality < PLL_GOOD_BITS * 2) {  
        ++g_pll_quality;  
    }  
  
    // Два байта FCS не входят в информационное поле.  
    const int data_len = g_frame_len - 2;  
  
    if (data_len < 16) {  
        return;  
    }  
  
    const uint8_t* destination = &g_frame_buf[0];  
    const uint8_t* source = &g_frame_buf[7];  
  
    // Адресное поле: destination + source, затем дигипитеры.  
    int addr_pos = 14;  
    const uint8_t* last_addr = source;  
  
    while ((last_addr[6] & 0x01u) == 0u) {  
        if (addr_pos + 7 > data_len) {  
            return;  
        }  
  
        last_addr = &g_frame_buf[addr_pos];  
        addr_pos += 7;  
    }  
  
    // После адресного поля нужны control и PID.  
    if (addr_pos + 2 > data_len) {  
        return;  
    }  
  
    const uint8_t control = g_frame_buf[addr_pos];  
  
    // Выводим только UI-кадры.  
    if (control != 0x03 && control != 0x13) {  
        return;  
    }  
  
    ax25_emit_callsign(source);  
    ax25_emit('>');  
    ax25_emit_callsign(destination);  
  
    // Дигипитеры, если есть.  
    int digi_pos = 14;  
  
    while (digi_pos < addr_pos) {  
        ax25_emit(',');  
        ax25_emit_callsign(&g_frame_buf[digi_pos]);  
        digi_pos += 7;  
    }  
  
    ax25_emit(':');  
  
    // Пропускаем control и PID.  
    const int payload_pos = addr_pos + 2;  
  
    for (int i = payload_pos; i < data_len; ++i) {  
        const uint8_t c = g_frame_buf[i];  
  
        ax25_emit(  
            (c >= 0x20 && c < 0x7F) ? (char)c : '.'  
        );  
    }  
  
    ax25_emit('\n');  
}  
  
// -----------------------------------------------------------------------------  
// NRZI -> HDLC  
//  
// NRZI: переход тона = 0, отсутствие перехода = 1.  
// Флаг на линии, LSB-first: 0 1 1 1 1 1 1 0.  
// Окно проверяется до выдачи старшего бита: если это флаг — все восемь  
// битов поглощаются флагом и в кадр не попадают.  
// -----------------------------------------------------------------------------  
  
static void ax25_hdlc_bit(uint8_t raw)  
{  
    const uint8_t dbit =  
        (raw == g_prev_raw) ? 1u : 0u;  
  
    g_prev_raw = raw;  
  
    g_hdlc_window =  
        (uint8_t)((g_hdlc_window << 1) | dbit);  
  
    if (g_window_bits < 8) {  
        ++g_window_bits;  
    }  
  
    if (g_window_bits < 8) {  
        return;  
    }  
  
    // Флаг имеет приоритет над данными и abort.  
    if (g_hdlc_window == 0x7E) {  
#if AX25_DEBUG  
        ++dbg_flags;  
#endif  
  
        // Повторные флаги — надёжные метки тайминга ещё до первого  
        // валидного FCS: позволяют DPLL выйти из захвата.  
        if (g_pll_quality < PLL_GOOD_BITS * 2) {  
            ++g_pll_quality;  
        }  
  
        // Флаг закрывает предыдущий кадр, если тот не был в abort.  
        if (!g_in_abort) {  
            ax25_frame_finish();  
        }  
  
        // Тот же флаг открывает следующий кадр.  
        ax25_frame_reset();  
        g_in_abort = false;  
  
        // Биты флага в кадр не попадают.  
        g_hdlc_window = 0;  
        g_window_bits = 0;  
  
        return;  
    }  
  
    // Не флаг: выдаём старший бит окна, оставляем семь.  
    const uint8_t oldest =  
        (uint8_t)((g_hdlc_window >> 7) & 1u);  
  
    ax25_process_data_bit(oldest);  
  
    g_hdlc_window &= 0x7F;  
    g_window_bits = 7;  
}  
  
// -----------------------------------------------------------------------------  
// Публичный init — интерфейс модема (center в Гц, 0 = дефолт 1500)  
// -----------------------------------------------------------------------------  
  
void ax25_rx_init(uint32_t center_hz)  
{  
    if (center_hz == 0) center_hz = 1500;  
  
    g_active = false;  
  
    const float mark_hz =  
        (float)center_hz - AX25_SHIFT_HZ * 0.5f;  
  
    const float space_hz =  
        (float)center_hz + AX25_SHIFT_HZ * 0.5f;  
  
    tone_init(&g_mark, mark_hz);  
    tone_init(&g_space, space_hz);  
  
    g_dpll_step =  
        (uint32_t)(4294967296.0 * AX25_BAUD / AX25_FS);  
  
    g_dpll = 0;  
    g_pll_quality = 0;  
  
    g_peak = 100.0f;  
    g_valley = -100.0f;  
  
    g_tone_state = 0;  
  
    g_prev_tone = 0;  
    g_prev_raw = 0;  
  
    g_hdlc_window = 0;  
    g_window_bits = 0;  
  
    g_in_abort = false;  
  
    ax25_frame_reset();  
  
    Serial.printf("[AX25] init center=%lu mark=%lu space=%lu\n",  
                  (unsigned long)center_hz,  
                  (unsigned long)mark_hz,  
                  (unsigned long)space_hz);  
  
    g_active = true;  
}  
  
// -----------------------------------------------------------------------------  
// Публичный stop  
// -----------------------------------------------------------------------------  
  
void ax25_rx_stop(void)  
{  
    g_active = false;  
  
    g_dpll = 0;  
    g_pll_quality = 0;  
  
    g_peak = 100.0f;  
    g_valley = -100.0f;  
  
    g_tone_state = 0;  
  
    g_prev_tone = 0;  
    g_prev_raw = 0;  
  
    g_hdlc_window = 0;  
    g_window_bits = 0;  
  
    g_in_abort = false;  
  
    ax25_frame_reset();  
}  
  
// -----------------------------------------------------------------------------  
// Публичный вход — один отсчёт 15625 sps  
// -----------------------------------------------------------------------------  
  
void ax25_rx_push(int16_t sample)  
{  
    if (!g_active) {  
        return;  
    }  
  
    const float pm = tone_push(&g_mark, (float)sample);  
    const float ps = tone_push(&g_space, (float)sample);  
  
    const float d = pm - ps;  
  
#if AX25_DEBUG  
    if (d < dbg_dmin) dbg_dmin = d;  
    if (d > dbg_dmax) dbg_dmax = d;  
#endif  
  
    // Трекинг пика и впадины.  
    if (d > g_peak) {  
        g_peak += 0.50f * (d - g_peak);  
    } else {  
        g_peak += 0.02f * (d - g_peak);  
    }  
  
    if (d < g_valley) {  
        g_valley += 0.50f * (d - g_valley);  
    } else {  
        g_valley += 0.02f * (d - g_valley);  
    }  
  
#if AX25_FIXED_MIDPOINT  
    const float mid = 0.0f;  
#else  
    const float mid = (g_peak + g_valley) * 0.5f;  
#endif  
  
    float range = g_peak - g_valley;  
  
    if (range < 0.0f) {  
        range = 0.0f;  
    }  
  
    const float hyst = range * AX25_HYST_FRACTION;  
  
    // Слайсер с гистерезисом.  
    if (d > mid + hyst) {  
        g_tone_state = 1;  
    } else if (d < mid - hyst) {  
        g_tone_state = 0;  
    }  
  
    const uint8_t tone = g_tone_state;  
  
    // DPLL-строб бита — на КАЖДОМ отсчёте.  
    const uint32_t prev_phase = g_dpll;  
    g_dpll += g_dpll_step;  
  
    if (g_dpll < prev_phase) {  
#if AX25_DEBUG  
        ++dbg_bits;  
#endif  
        ax25_hdlc_bit(tone);  
    }  
  
    // Коррекция фазы DPLL по переходам слайсера.  
    if (tone != g_prev_tone) {  
#if AX25_DEBUG  
        ++dbg_trans;  
        dbg_run_sum += dbg_run_n;  
        dbg_run_n = 0;  
#endif  
  
        // Переход тона = начало бита NRZI; строб должен лежать  
        // около середины бита, т.е. фаза ~0.5 оборота (0x80000000).  
        const int32_t phase_err =  
            (int32_t)(g_dpll - 0x80000000u);  
  
        const float gain =  
            (g_pll_quality >= PLL_GOOD_BITS)  
                ? PLL_LOCKED  
                : PLL_SEARCHING;  
  
        int64_t correction =  
            (int64_t)((double)gain * (double)phase_err);  
  
        // Не больше четверти оборота за переход.  
        const int64_t max_corr = (int64_t)1 << 30;  
  
        if (correction > max_corr) {  
            correction = max_corr;  
        }  
  
        if (correction < -max_corr) {  
            correction = -max_corr;  
        }  
  
        // Беззнаковое вычитание — намеренный wrap по модулю 2^32.  
        g_dpll -= (uint32_t)correction;  
  
        g_prev_tone = tone;  
    }  
  
#if AX25_DEBUG  
    ++dbg_run_n;  
  
    if (++dbg_samples >= (uint32_t)AX25_FS) {  
        const uint32_t sample_count = dbg_samples;  
  
        const float samples_per_bit =  
            dbg_bits  
                ? (float)sample_count / (float)dbg_bits  
                : 0.0f;  
  
        const float samples_per_transition =  
            dbg_trans  
                ? (float)dbg_run_sum / (float)dbg_trans  
                : 0.0f;  
  
        Serial.printf(  
            "# AX25 d[%ld..%ld] pk=%ld vl=%ld mid=%ld "  
            "bits=%lu smp/bit=%ld.%02ld smp/trans=%ld.%02ld "  
            "flags=%lu stuff=%lu abort=%lu "  
            "fcs_ok=%lu fcs_bad=%lu pll=%d\n",  
            (long)dbg_dmin, (long)dbg_dmax,  
            (long)g_peak, (long)g_valley, (long)mid,  
            (unsigned long)dbg_bits,  
            (long)samples_per_bit,  
            labs((long)(samples_per_bit * 100.0f) % 100),  
            (long)samples_per_transition,  
            labs((long)(samples_per_transition * 100.0f) % 100),  
            (unsigned long)dbg_flags,  
            (unsigned long)dbg_stuff,  
            (unsigned long)dbg_abort,  
            (unsigned long)dbg_fcs_ok,  
            (unsigned long)dbg_fcs_bad,  
            g_pll_quality  
        );  
  
        dbg_samples = 0;  
        dbg_bits = 0;  
        dbg_trans = 0;  
        dbg_run_sum = 0;  
  
        dbg_dmin = 1.0e30f;  
        dbg_dmax = -1.0e30f;  
    }  
#endif  
}