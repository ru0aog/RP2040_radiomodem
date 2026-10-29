// src/rx/rx_dsp.cpp  
// ============================================================================  
// Децимирующий DSP-тракт (Core 1, только RX-фаза), poll-модель для  
// RP2040_radiomodem: вызывается из loop1() при trx_state()==TRX_RX.  
// Порт из RP2040_receiver/src/dsp/rx_dsp.cpp v2.26 — вместо отдельного  
// потока core1 блоки выкачиваются из очереди rx_capture прямо в poll.  
//  
// Тракт: [I|Q] из rx_capture → центрирование → NCO-сдвиг → CIC /2 →  
// 2 каскада half-band FIR /4 → 15625 sps I/Q (очередь out_* для мод) +  
// демодулятор RAW/USB/LSB/AM/FM → AF-полосовой → notch → NR → aud-очередь  
// (моно, для usb_audio). RSID: ресемплер Фарроу 15625→11025.  
// ============================================================================  
#include <Arduino.h>  
#include <string.h>  
  
#include "rx_dsp.h"  
#include "rx_capture.h"  
#include "../vfo/soft_rx.h"   // vfo_quad_lo_hz()  
#include "../rsid/rsid.h"
  
// ---- Параметры тракта ----  
#define RX_FS_IN_HZ     (RX_ADC_SAMPLE_RATE_HZ / 2.0f)  // 125 ksps/канал  
#define CIC_R           2  
#define CIC_N           3  
#define RX_DECIM_STAGES 2                               // /8 итого → 15625  
#define RX_FS_OUT_HZ    (RX_FS_IN_HZ / 8.0f)  
  
// ---- NCO (32-бит фазовый аккумулятор) ----  
static volatile uint32_t nco_phase = 0;  
static volatile uint32_t nco_step  = 0;  
  
static volatile uint32_t clip_count = 0;  
  
// sin/cos LUT: 1024 точки, Q15  
static int16_t nco_lut[1024];  
static void nco_lut_init(void) {  
    for (int i = 0; i < 1024; i++)  
        nco_lut[i] = (int16_t)(32767.0f * sinf(2.0f * PI * i / 1024.0f));  
}  
static inline void __not_in_flash_func(nco_sincos)(  
    uint32_t ph, int16_t* s, int16_t* c) {  
    uint16_t idx = ph >> 22;  
    *s = nco_lut[idx];  
    *c = nco_lut[(idx + 256) & 1023];  
}  
  
// ---- Half-band FIR ----  
// Каскад 0 (62.5→31.25k): 15 отводов, DC-gain = 1.0 (сумма = 32768)  
#define HB0_NTAPS 15  
static const int16_t hb0_taps[HB0_NTAPS] = {  
    -120, 0, 532, 0, -2250, 0, 10030, 16384, 10030, 0, -2250, 0, 532, 0, -120  
};  
// Каскад 1 (31.25→15.625k): 31 отвод, Блэкман-Харрис, ~90 дБ  
#define HBX_NTAPS 31  
static const int16_t hbx_taps[HBX_NTAPS] = {  
       0,     0,     3,     0,   -25,     0,   119,     0,  
    -399,     0,  1086,     0, -2760,     0, 10168, 16384,  
   10168,     0, -2760,     0,  1086,     0,  -399,     0,  
     119,     0,   -25,     0,     3,     0,     0  
};  
  
typedef struct {  
    int32_t d[HBX_NTAPS];  
    uint8_t w;  
    uint8_t phase;  
    uint8_t ntaps;  
    const int16_t* taps;  
} hb_state_t;  
  
static hb_state_t hb_i[RX_DECIM_STAGES];  
static hb_state_t hb_q[RX_DECIM_STAGES];  
  
// ---- CIC R=2, N=3 ----  
static int32_t cic_int_i[CIC_N],  cic_int_q[CIC_N];  
static int32_t cic_comb_i[CIC_N], cic_comb_q[CIC_N];  
static uint8_t cic_cnt = 0;  
  
static inline bool __not_in_flash_func(cic_push_iq)(  
    int32_t xi, int32_t xq, int32_t* yi, int32_t* yq) {  
    cic_int_i[0] += xi; cic_int_i[1] += cic_int_i[0]; cic_int_i[2] += cic_int_i[1];  
    cic_int_q[0] += xq; cic_int_q[1] += cic_int_q[0]; cic_int_q[2] += cic_int_q[1];  
  
    if (++cic_cnt < CIC_R) return false;  
    cic_cnt = 0;  
  
    int32_t ti = cic_int_i[2], tq = cic_int_q[2];  
    for (int k = 0; k < CIC_N; ++k) {  
        int32_t di = ti - cic_comb_i[k]; cic_comb_i[k] = ti; ti = di;  
        int32_t dq = tq - cic_comb_q[k]; cic_comb_q[k] = tq; tq = dq;  
    }  
    *yi = (ti + 4) >> 3;   // DC-gain = R^N = 8  
    *yq = (tq + 4) >> 3;  
    return true;  
}  
  
// ---- HB-каскад: на чётных входах выдаёт отсчёт ----  
static inline bool __not_in_flash_func(hb_push)(  
    hb_state_t* st, int32_t x, int32_t* out) {  
    st->d[st->w] = x;  
    if (++st->w >= st->ntaps) st->w = 0;  
    st->phase ^= 1;  
    if (st->phase) return false;  
  
    const int N = (int)st->ntaps;  
    const int C = (N - 1) >> 1;  
    int32_t acc = 0;  
    for (int k = 0; k < C; ++k) {  
        const int32_t c = st->taps[k];  
        if (c == 0) continue;                  // half-band: нулевые отводы  
        int idx_lo = (int)st->w + k;  
        if (idx_lo >= N) idx_lo -= N;  
        int idx_hi = (int)st->w + (N - 1 - k);  
        if (idx_hi >= N) idx_hi -= N;  
        acc += c * (st->d[idx_lo] + st->d[idx_hi]);   // симметрия: одно умножение на пару  
    }  
    int idx_c = (int)st->w + C;  
    if (idx_c >= N) idx_c -= N;  
    acc += st->taps[C] * st->d[idx_c];  
  
    *out = (acc + (1 << 14)) >> 15;  
    return true;  
}  
  
static inline int16_t __not_in_flash_func(sat16)(int32_t v) {  
    if ((v >> 15) ^ (v >> 31)) {  
        clip_count++;  
        return (int16_t)((v >> 31) ^ 0x7FFF);  
    }  
    return (int16_t)v;  
}  
  
// ---- Выходная очередь базовой полосы I/Q (для мод, Core 0) ----  
#define RX_OUT_SAMPLES (RX_BLOCK_SAMPLES / 8)   // 64 пары на блок  
#define RX_OUT_BUFS    4  
static int16_t out_i[RX_OUT_BUFS][RX_OUT_SAMPLES];  
static int16_t out_q[RX_OUT_BUFS][RX_OUT_SAMPLES];  
static volatile uint8_t  out_head = 0;          // пишет Core 1  
static volatile uint8_t  out_tail = 0;          // читает Core 0  
static volatile uint32_t out_dropped = 0;  
  
// ---- Аудио-очередь (демод-моно → usb_audio, Core 0) ----  
#define RX_AUD_SAMPLES 64  
#define RX_AUD_BUFS    8  
static int16_t aud_i[RX_AUD_BUFS][RX_AUD_SAMPLES];  
static int16_t aud_q[RX_AUD_BUFS][RX_AUD_SAMPLES];  
static volatile uint8_t  aud_head = 0;  
static volatile uint8_t  aud_tail = 0;  
static volatile uint32_t aud_dropped = 0;  
static uint8_t aud_wr_n = 0;  
  
// DC-трекинг ADC (смещение ~2048)  
static int32_t dc_i = 2048 << 8;  
static int32_t dc_q = 2048 << 8;  
  
// ---- Демодулятор ----  
static volatile uint8_t rxd_demod_mode = RXD_DEMOD_RAW;  
  
// Hilbert FIR 31 tap Q15 (только 16 ненулевых отводов, антисимметрия)  
#define HILB_N 31  
#define HILB_M ((HILB_N - 1) / 2)  
static const int16_t hilb_hq15[16] = {  
    -111, -193, -439, -924, -1753, -3211, -6341, -20654,  
   20654,  6341,  3211,  1753,   924,   439,   193,    111  
};  
static const uint8_t hilb_k[16] = {  
    0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30  
};  
static int16_t hilb_buf[HILB_N] = {};  
static int     hilb_idx = 0;  
static int16_t i_del[HILB_M + 1] = {};  
static int     i_del_idx = 0;  
  
// Один демод-отсчёт SSB. Вызывать на КАЖДОМ emit — иначе рвётся фаза Гильберта.  
static inline int32_t __not_in_flash_func(rxd_demod_sample)(  
    int32_t i, int32_t q) {  
    i_del[i_del_idx] = (int16_t)i;  
    int del = i_del_idx - HILB_M;  
    if (del < 0) del += (HILB_M + 1);  
    const int32_t i_d = (int32_t)i_del[del];  
    if (++i_del_idx >= (HILB_M + 1)) i_del_idx = 0;  
  
    hilb_buf[hilb_idx] = (int16_t)q;  
    if (++hilb_idx >= HILB_N) hilb_idx = 0;  
  
    int64_t acc = 0;  
    for (int n = 0; n < 8; ++n) {  
        const int k = hilb_k[n];  
        int idx_lo = hilb_idx + HILB_N - 1 - k;  
        if (idx_lo >= HILB_N) idx_lo -= HILB_N;  
        int idx_hi = hilb_idx + k;  
        if (idx_hi >= HILB_N) idx_hi -= HILB_N;  
        const int32_t diff =  
            (int32_t)hilb_buf[idx_lo] - (int32_t)hilb_buf[idx_hi];  
        acc += (int32_t)hilb_hq15[n] * diff;  
    }  
    const int32_t hq = (int32_t)(acc >> 15);  
    return (rxd_demod_mode == RXD_DEMOD_LSB) ? (i_d + hq) : (i_d - hq);  
}  
  
// ---- AM: alpha-max-beta-min огибающая + DC-блокер ----  
static int32_t am_dc = 0;  
static inline int32_t __not_in_flash_func(rxd_am_sample)(int32_t i, int32_t q) {  
    int32_t ai = i < 0 ? -i : i;  
    int32_t aq = q < 0 ? -q : q;  
    int32_t mx = ai > aq ? ai : aq;  
    int32_t mn = ai > aq ? aq : ai;  
    int32_t env = mx + ((mn * 3) >> 3);  
    am_dc += (env - am_dc) >> 8;  
    return env - am_dc;  
}  
  
// ---- FM-дискриминатор ----  
#define FM_GAIN 24000  
static int32_t fm_i_prev = 0, fm_q_prev = 0;  
static inline int32_t __not_in_flash_func(rxd_fm_sample)(int32_t i, int32_t q) {  
    int64_t cross = (int64_t)fm_i_prev * q - (int64_t)fm_q_prev * i;  
    int64_t mag2  = (int64_t)i * i + (int64_t)q * q;  
    fm_i_prev = i;  
    fm_q_prev = q;  
    if (mag2 < 1) mag2 = 1;  
    return (int32_t)((cross * FM_GAIN) / mag2);  
}  
  
// ---- AF-полосовой фильтр (63 отвода, линейная фаза) ----  
#define AF_LP_N 63  
static volatile uint32_t af_lo_hz = 0;  
static volatile uint32_t af_hi_hz = 3100;  
static int16_t af_lp_taps[AF_LP_N];  
static int32_t af_lp_buf[AF_LP_N] = {};  
static int     af_lp_idx = 0;  
  
static void af_lp_init(void) {  
    const int   M     = (AF_LP_N - 1) / 2;  
    const float fc_hi = (float)af_hi_hz / RX_FS_OUT_HZ;  
    const float fc_lo = (float)af_lo_hz / RX_FS_OUT_HZ;  
    float h[AF_LP_N];  
    float sum_dc = 0.0f;  
    for (int n = 0; n < AF_LP_N; n++) {  
        const int m = n - M;  
        float s_hi = (m == 0) ? (2.0f * fc_hi)  
                     : sinf(2.0f * PI * fc_hi * m) / (PI * m);  
        float s_lo = (m == 0) ? (2.0f * fc_lo)  
                     : sinf(2.0f * PI * fc_lo * m) / (PI * m);  
        float w = 0.42f - 0.5f * cosf(2.0f * PI * n / (AF_LP_N - 1))  
                        + 0.08f * cosf(4.0f * PI * n / (AF_LP_N - 1));  
        h[n] = (s_hi - s_lo) * w;  
        sum_dc += fabsf(h[n]);  
    }  
    const float norm = (sum_dc > 0.0f) ? (2.0f / sum_dc) : 1.0f;  
    for (int n = 0; n < AF_LP_N; n++)  
        af_lp_taps[n] = (int16_t)lroundf(h[n] * norm * 32768.0f);  
}  
  
void rx_dsp_set_af_band(uint32_t f_lo_hz, uint32_t f_hi_hz) {  
    const uint32_t FS2 = (uint32_t)(RX_FS_OUT_HZ * 0.5f);  
    if (f_hi_hz > FS2 - 200) f_hi_hz = FS2 - 200;  
    if (f_lo_hz >= f_hi_hz)  f_lo_hz = (f_hi_hz > 100) ? (f_hi_hz - 100) : 0;  
    af_lo_hz = f_lo_hz;  
    af_hi_hz = f_hi_hz;  
    af_lp_init();  
}  
uint32_t rx_dsp_af_lo_hz(void) { return af_lo_hz; }  
uint32_t rx_dsp_af_hi_hz(void) { return af_hi_hz; }  
  
static inline int32_t __not_in_flash_func(af_lp_push)(int32_t x) {  
    af_lp_buf[af_lp_idx] = x;  
    if (++af_lp_idx >= AF_LP_N) af_lp_idx = 0;  
    int64_t acc = 0;  
    int lo = af_lp_idx;  
    int hi = af_lp_idx + AF_LP_N - 1;  
    if (hi >= AF_LP_N) hi -= AF_LP_N;  
    for (int k = 0; k < AF_LP_N / 2; k++) {  
        acc += (int32_t)af_lp_taps[k] * (af_lp_buf[lo] + af_lp_buf[hi]);  
        if (++lo >= AF_LP_N) lo = 0;  
        if (--hi < 0)        hi = AF_LP_N - 1;  
    }  
    acc += (int32_t)af_lp_taps[AF_LP_N / 2] * af_lp_buf[lo];  
    return (int32_t)((acc + (1 << 14)) >> 15);  
}  
  
// ---- Адаптивный автонотч (NLMS-предиктор, выход = ошибка) ----  
#define NOTCH_N          24  
#define NOTCH_DELAY      96  
#define NOTCH_MU_Q       2458  
#define NOTCH_LEAK_SHIFT 10  
#define NOTCH_EPS        4096  
#define NOTCH_G_MAX      (1 << 20)  
  
static volatile bool notch_enabled = false;  
static int16_t notch_w[NOTCH_N] = {};  
static int32_t notch_buf[NOTCH_N + NOTCH_DELAY] = {};  
static int     notch_idx = 0;  
static int64_t notch_pow = 0;  
  
static inline int32_t __not_in_flash_func(notch_lms_push)(int32_t x) {  
    notch_buf[notch_idx] = x;  
    int64_t acc = 0, energy = 0;  
    int p = notch_idx - NOTCH_DELAY;  
    if (p < 0) p += NOTCH_N + NOTCH_DELAY;  
    int32_t taps[NOTCH_N];  
    for (int k = 0; k < NOTCH_N; k++) {  
        int32_t v = notch_buf[p];  
        taps[k] = v;  
        acc    += (int64_t)notch_w[k] * v;  
        energy += (int64_t)v * v;  
        if (--p < 0) p += NOTCH_N + NOTCH_DELAY;  
    }  
    int32_t y = (int32_t)(acc >> 15);  
    int32_t e = x - y;  
    notch_pow += (energy - notch_pow) >> 5;  
    int64_t den = notch_pow + NOTCH_EPS;  
    int64_t g = ((int64_t)e * NOTCH_MU_Q << 15) / den;  
    if (g >  NOTCH_G_MAX) g =  NOTCH_G_MAX;  
    if (g < -NOTCH_G_MAX) g = -NOTCH_G_MAX;  
    for (int k = 0; k < NOTCH_N; k++) {  
        int32_t upd = (int32_t)(((int64_t)g * taps[k]) >> 15);  
        int32_t w = (int32_t)notch_w[k] + upd  
                  - ((int32_t)notch_w[k] >> NOTCH_LEAK_SHIFT);  
        if (w >  32767) w =  32767;  
        if (w < -32768) w = -32768;  
        notch_w[k] = (int16_t)w;  
    }  
    if (++notch_idx >= NOTCH_N + NOTCH_DELAY) notch_idx = 0;  
    return e;  
}  
  
void rx_dsp_set_notch(bool on) {  
    if (on && !notch_enabled) {  
        memset(notch_buf, 0, sizeof(notch_buf));  
        memset(notch_w,   0, sizeof(notch_w));  
        notch_idx = 0;  
        notch_pow = 0;  
    }  
    notch_enabled = on;  
}  
  
// ---- NR: LMS ALE, выход = предсказание ----  
#define NR_N          16  
#define NR_DELAY       5  
#define NR_MU_Q      1638  
#define NR_LEAK_SHIFT 10  
#define NR_EPS  ((int64_t)1 << 21)  
#define NR_G_MAX ((int64_t)1 << 20)  
#define NR_PWR_SHIFT  5  
#define NR_DEPTH_Q15   24576  
#define NR_MAKEUP_Q15  65536  
  
static int64_t nr_pow = 0;  
static volatile bool nr_enabled = false;  
static int32_t nr_buf[NR_N + NR_DELAY] = {0};  
static int     nr_idx = 0;  
static int16_t nr_w[NR_N] = {0};  
  
static inline int32_t __not_in_flash_func(nr_lms_push)(int32_t x) {  
    nr_idx = nr_idx + 1;  
    if (nr_idx >= NR_N + NR_DELAY) nr_idx = 0;  
    nr_buf[nr_idx] = x;  
  
    int64_t acc = 0, energy = 0;  
    int p = nr_idx - NR_DELAY;  
    if (p < 0) p += NR_N + NR_DELAY;  
    int taps[NR_N];  
    for (int k = 0; k < NR_N; k++) {  
        taps[k] = p;  
        int32_t xk = nr_buf[p];  
        acc    += (int64_t)nr_w[k] * xk;  
        energy += (int64_t)xk * xk;  
        if (--p < 0) p += NR_N + NR_DELAY;  
    }  
    int32_t y = (int32_t)(acc >> 15);  
    int32_t e = x - y;  
  
    nr_pow += (energy - nr_pow) >> NR_PWR_SHIFT;  
    int64_t g = (((int64_t)e * NR_MU_Q) << 15) / (nr_pow + NR_EPS);  
    if (g >  NR_G_MAX) g =  NR_G_MAX;  
    if (g < -NR_G_MAX) g = -NR_G_MAX;  
  
    for (int k = 0; k < NR_N; k++) {  
        int64_t upd = (g * nr_buf[taps[k]]) >> 15;  
        int32_t w = (int32_t)nr_w[k];  
        w += (int32_t)upd - (w >> NR_LEAK_SHIFT);  
        if (w >  32767) w =  32767;  
        if (w < -32768) w = -32768;  
        nr_w[k] = (int16_t)w;  
    }  
    int32_t out = x - (int32_t)(((int64_t)e * NR_DEPTH_Q15) >> 15);  
    out = (int32_t)(((int64_t)out * NR_MAKEUP_Q15) >> 15);  
    if (out >  32767) out =  32767;  
    if (out < -32768) out = -32768;  
    return out;  
}  
  
void rx_dsp_set_nr(bool on) {  
    if (on && !nr_enabled) {  
        memset(nr_buf, 0, sizeof(nr_buf));  
        memset(nr_w,   0, sizeof(nr_w));  
        nr_idx = 0;  
        nr_pow = 0;  
    }  
    nr_enabled = on;  
}  
bool rx_dsp_get_nr(void) { return nr_enabled; }  
  
// ---- Измерение загрузки Core 1 ----  
static uint32_t core1_dsp_busy_us  = 0;  
static uint32_t core1_rsid_busy_us = 0;  
static uint32_t core1_window_start_us = 0;  
static volatile uint32_t dsp_load_permille   = 0;  
static volatile uint32_t rsid_load_permille  = 0;  
static volatile uint32_t total_load_permille = 0;  
  
static inline void core1_update_load(uint32_t now_us) {  
    const uint32_t window_us = now_us - core1_window_start_us;  
    if (window_us < 1000000u) return;  
    uint32_t dsp_busy  = core1_dsp_busy_us;  
    uint32_t rsid_busy = core1_rsid_busy_us;  
    if (dsp_busy  > window_us) dsp_busy  = window_us;  
    if (rsid_busy > window_us) rsid_busy = window_us;  
    uint32_t total_busy = dsp_busy + rsid_busy;  
    if (total_busy > window_us) total_busy = window_us;  
    dsp_load_permille   = (uint32_t)(((uint64_t)dsp_busy   * 1000u) / window_us);  
    rsid_load_permille  = (uint32_t)(((uint64_t)rsid_busy  * 1000u) / window_us);  
    total_load_permille = (uint32_t)(((uint64_t)total_busy * 1000u) / window_us);  
    core1_dsp_busy_us = 0;  
    core1_rsid_busy_us = 0;  
    core1_window_start_us = now_us;  
}  
  
// ---- RSID-ресемплер Фарроу 15625 → 11025 ----  
static int32_t rsid_rr_xm1 = 0, rsid_rr_x0 = 0, rsid_rr_x1 = 0, rsid_rr_x2 = 0;  
static uint8_t rsid_rr_init = 0;  
#define RSID_RR_ONE_Q32 ((uint64_t)1 << 32)  
static const uint64_t rsid_rr_step_q32 =  
    (uint64_t)((15625.0 / 11025.0) * 4294967296.0);  
static uint64_t rsid_rr_frac_q = 0;  
  
static inline void __not_in_flash_func(rsid_farrow_calc_i)(  
    int32_t ym1, int32_t y0, int32_t y1, int32_t y2,  
    int32_t* c0, int32_t* c1, int32_t* c2, int32_t* c3) {  
    *c0 = 2 * y0;  
    *c1 = y1 - ym1;  
    *c2 = 2 * ym1 - 5 * y0 + 4 * y1 - y2;  
    *c3 = 3 * (y0 - y1) + y2 - ym1;  
}  
static inline int32_t __not_in_flash_func(rsid_farrow_eval_i)(  
    int32_t c0, int32_t c1, int32_t c2, int32_t c3, uint32_t mu_q15) {  
    int32_t t = c3;  
    t = (int32_t)(((int64_t)t * mu_q15) >> 15) + c2;  
    t = (int32_t)(((int64_t)t * mu_q15) >> 15) + c1;  
    t = (int32_t)(((int64_t)t * mu_q15) >> 15) + c0;  
    return t;  
}  
static inline void __not_in_flash_func(rsid_resampler_push)(int32_t x) {  
    if (!rsid_rx_push) return;                 // RSID ещё каркас  
    rsid_rr_xm1 = rsid_rr_x0;  
    rsid_rr_x0  = rsid_rr_x1;  
    rsid_rr_x1  = rsid_rr_x2;  
    rsid_rr_x2  = x;  
    if (rsid_rr_init < 4) {  
        ++rsid_rr_init;  
        if (rsid_rr_init < 4) return;  
    }  
    int32_t c0, c1, c2, c3;  
    rsid_farrow_calc_i(rsid_rr_xm1, rsid_rr_x0, rsid_rr_x1, rsid_rr_x2,  
                       &c0, &c1, &c2, &c3);  
    while (rsid_rr_frac_q < RSID_RR_ONE_Q32) {  
        const uint32_t mu_q15 = (uint32_t)(rsid_rr_frac_q >> 17);  
        const int32_t y2 = rsid_farrow_eval_i(c0, c1, c2, c3, mu_q15);  
        rsid_rx_push((int16_t)((y2 + 1) >> 1));  
        rsid_rr_frac_q += rsid_rr_step_q32;  
    }  
    rsid_rr_frac_q -= RSID_RR_ONE_Q32;  
}  
static void rsid_resampler_reset(void) {  
    rsid_rr_xm1 = rsid_rr_x0 = rsid_rr_x1 = rsid_rr_x2 = 0;  
    rsid_rr_init = 0;  
    rsid_rr_frac_q = 0;  
}  
  
// ============================================================================  
// Обработка одного DMA-блока (горячий путь Core 1)  
// ============================================================================  
static void __not_in_flash_func(rx_dsp_process_block)(  
    const volatile uint32_t* blk) {  
    uint8_t wr        = out_head;  
    uint8_t next_head = (uint8_t)((wr + 1) % RX_OUT_BUFS);  
    bool    queue_full = (next_head == out_tail);  
    uint8_t n_out = 0;  
  
    const volatile int16_t* s = (const volatile int16_t*)blk;  
  
    for (int n = 0; n < RX_BLOCK_SAMPLES; n++) {  
        // [15:0] = ADC0 (I), [31:16] = ADC1 (Q)  
        int32_t i_meas = s[2 * n];  
        int32_t q_meas = s[2 * n + 1];  
  
        // Центрирование: EMA смещения, k=1/256  
        dc_i += ((i_meas << 8) - dc_i) >> 8;  
        dc_q += ((q_meas << 8) - dc_q) >> 8;  
        int32_t i_raw = i_meas - (dc_i >> 8);  
        int32_t q_raw = q_meas - (dc_q >> 8);  
  
        // Комплексный перенос NCO  
        int16_t sn, cs;  
        nco_sincos(nco_phase, &sn, &cs);  
        nco_phase += nco_step;  
        int32_t i_rot = (i_raw * cs + q_raw * sn) >> 15;  
        int32_t q_rot = (q_raw * cs - i_raw * sn) >> 15;  
  
        // CIC /2 → 2×HB /2 → 15.625 ksps  
        int32_t ci, cq;  
        bool emit = cic_push_iq(i_rot, q_rot, &ci, &cq);  
        int32_t vi = ci, vq = cq;  
  
        if (emit) {  
            for (int k = 0; k < RX_DECIM_STAGES; ++k) {  
                int32_t oi, oq;  
                bool emit_i = hb_push(&hb_i[k], vi, &oi);  
                bool emit_q = hb_push(&hb_q[k], vq, &oq);  
                if (emit_i != emit_q) { emit = false; break; }  
                if (!emit_i)          { emit = false; break; }  
                vi = oi;  
                vq = oq;  
            }  
        }  
  
        if (emit) {  
            int16_t si, sq;  
            if (rxd_demod_mode == RXD_DEMOD_RAW) {  
                si = sat16(vi);  
                sq = sat16(vq);  
            } else {  
                int32_t d;  
                if      (rxd_demod_mode == RXD_DEMOD_AM) d = rxd_am_sample(vi, vq);  
                else if (rxd_demod_mode == RXD_DEMOD_FM) d = rxd_fm_sample(vi, vq);  
                else                                   d = rxd_demod_sample(vi, vq);  
  
                int32_t df = af_lp_push(d);  
                if (notch_enabled &&  
                    (rxd_demod_mode == RXD_DEMOD_USB ||  
                     rxd_demod_mode == RXD_DEMOD_LSB)) {  
                    df = notch_lms_push(df);  
                }  
                if (nr_enabled &&  
                    (rxd_demod_mode == RXD_DEMOD_USB ||  
                     rxd_demod_mode == RXD_DEMOD_LSB)) {  
                    df = nr_lms_push(df);  
                }  
                int16_t m = sat16(df);  
                rsid_resampler_push(m);   // моно → RSID 11025 Гц  
                si = m;  
                sq = m;  
            }  
  
            // Аудио-очередь (моно-демод для usb_audio/монитора)  
            uint8_t anext = (uint8_t)((aud_head + 1) % RX_AUD_BUFS);  
            if (anext != aud_tail) {  
                aud_i[aud_head][aud_wr_n] = si;  
                aud_q[aud_head][aud_wr_n] = sq;  
                if (++aud_wr_n >= RX_AUD_SAMPLES) {  
                    __dmb();  
                    aud_head = anext;  
                    aud_wr_n = 0;  
                }  
            } else {  
                aud_dropped++;  
            }  
  
            // Базовая полоса I/Q для мод (Core 0, через rx_decoders)  
            if (!queue_full && n_out < RX_OUT_SAMPLES) {  
                out_i[wr][n_out] = si;  
                out_q[wr][n_out] = sq;  
                ++n_out;  
            }  
        }  
    }  
  
    rx_capture_release();  
  
    if (queue_full) {  
        out_dropped++;  
    } else {  
        __dmb();                    // данные блока видимы до head  
        out_head = next_head;  
    }  
}  
  
// ============================================================================  
// Публичный API  
// ============================================================================  
  
// Одноразовая инициализация таблиц (вызывается из setup(), Core 0).  
void rx_dsp_init(void) {  
    nco_lut_init();  
    af_lp_init();  
    core1_window_start_us = time_us_32();  
    Serial.println("[RX] dsp init");  
}  
  
// Старт тракта под частоту приёма (Core 0, до rx_capture_start).  
void rx_dsp_start(uint32_t freq_hz) {  
    rsid_resampler_reset();  
    memset(hb_i, 0, sizeof(hb_i));  
    memset(hb_q, 0, sizeof(hb_q));  
    for (int k = 0; k < RX_DECIM_STAGES; ++k) {  
        const int16_t* t = (k == 0) ? hb0_taps : hbx_taps;  
        uint8_t        n = (k == 0) ? HB0_NTAPS : HBX_NTAPS;  
        hb_i[k].taps = t;  hb_i[k].ntaps = n;  
        hb_q[k].taps = t;  hb_q[k].ntaps = n;  
    }  
    memset(out_i, 0, sizeof(out_i));  
    memset(out_q, 0, sizeof(out_q));  
    out_head = out_tail = 0;  
    out_dropped = 0;  
    aud_head = aud_tail = aud_wr_n = 0;  
    aud_dropped = 0;  
    clip_count = 0;  
  
    memset(hilb_buf, 0, sizeof(hilb_buf));  
    memset(i_del,   0, sizeof(i_del));  
    hilb_idx = 0;  
    i_del_idx = 0;  
    am_dc = 0;  
    fm_i_prev = fm_q_prev = 0;  
  
    memset(cic_int_i, 0, sizeof(cic_int_i));  
    memset(cic_int_q, 0, sizeof(cic_int_q));  
    memset(cic_comb_i, 0, sizeof(cic_comb_i));  
    memset(cic_comb_q, 0, sizeof(cic_comb_q));  
    cic_cnt = 0;  
    dc_i = 2048 << 8;  
    dc_q = 2048 << 8;  
  
    memset(af_lp_buf, 0, sizeof(af_lp_buf));  
    af_lp_idx = 0;  
  
    // NCO = f_rx - фактический квадратурный LO (soft_rx ставит  
    // LO = f_rx − RX_IF_SHIFT; остаток доедаем здесь).  
    const uint32_t lo_hz = vfo_quad_lo_hz();  
    float f_nco = (float)freq_hz - (float)lo_hz;  
    double step = 4294967296.0 * (double)f_nco / (double)RX_FS_IN_HZ;  
    nco_step = (uint32_t)(int64_t)step;  
    nco_phase = 0;  
  
    Serial.printf("[RX] start f=%lu lo=%lu f_nco=%.1f step=0x%08lX\n",  
                  (unsigned long)freq_hz, (unsigned long)lo_hz,  
                  (double)f_nco, (unsigned long)nco_step);  
}  
  
// Останов тракта (Core 0). Должен предшествовать rx_capture_stop().  
void rx_dsp_stop(void) {  
    // DSP работает только в rx_dsp_poll() на Core 1 в RX-фазе:  
    // trx_switch гарантирует, что poll не вызывается после выхода из RX.  
    // Сброс очередей выполняет rx_capture_stop() + rx_dsp_start().  
}  
  
// Выкачка всех доступных блоков из rx_capture (Core 1, из loop1()).  
void rx_dsp_poll(void) {  
    const volatile uint32_t* blk;  
    bool got = false;  
    while ((blk = rx_capture_get_block()) != NULL) {  
        got = true;  
        const uint32_t t0 = time_us_32();  
        rx_dsp_process_block(blk);  
        core1_dsp_busy_us += (uint32_t)(time_us_32() - t0);  
    }  
    if (!got && rsid_rx_service_sliced) {  
        const uint32_t t0 = time_us_32();  
        rsid_rx_service_sliced();  
        core1_rsid_busy_us += (uint32_t)(time_us_32() - t0);  
    }  
    core1_update_load(time_us_32());  
}  
  
// AFC: подстройка NCO на лету (Гц относительно LO).  
void rx_dsp_set_nco_hz(int32_t f_hz) {  
    double step = 4294967296.0 * (double)f_hz / (double)RX_FS_IN_HZ;  
    nco_step = (uint32_t)(int64_t)step;  
}  
  
void rx_dsp_set_demod(uint8_t mode) { rxd_demod_mode = mode; }  
  
// ---- Очередь базовой полосы (Core 0 → rx_decoders/моды) ----  
bool rx_dsp_get_output(const int16_t** i, const int16_t** q, uint8_t* n) {  
    uint8_t tail = out_tail;  
    if (tail == out_head) return false;  
    __dmb();  
    *i = out_i[tail];  
    *q = out_q[tail];  
    *n = RX_OUT_SAMPLES;  
    return true;  
}  
void rx_dsp_release_output(void) {  
    __dmb();  
    out_tail = (uint8_t)((out_tail + 1) % RX_OUT_BUFS);  
}  
  
// ---- Аудио-очередь (Core 0 → usb_audio/монитор) ----  
bool rx_dsp_get_audio(const int16_t** i, const int16_t** q, uint8_t* n) {  
    uint8_t tail = aud_tail;  
    if (tail == aud_head) return false;  
    __dmb();  
    *i = aud_i[tail];  
    *q = aud_q[tail];  
    *n = RX_AUD_SAMPLES;  
    return true;  
}  
void rx_dsp_release_audio(void) {  
    __dmb();  
    aud_tail = (uint8_t)((aud_tail + 1) % RX_AUD_BUFS);  
}  
uint8_t rx_dsp_aud_fill(void) {  
    return (uint8_t)((aud_head + RX_AUD_BUFS - aud_tail) % RX_AUD_BUFS);  
}  
  
// ---- Диагностика ----  
uint32_t rx_dsp_out_dropped(void)          { return out_dropped; }  
uint32_t rx_dsp_aud_dropped(void)          { return aud_dropped; }  
uint32_t rx_dsp_clip_count(void)           { return clip_count; }  
uint32_t rx_dsp_load_permille(void)        { return total_load_permille; }  
uint32_t rx_dsp_compute_load_permille(void){ return dsp_load_permille; }  
uint32_t rx_dsp_rsid_load_permille(void)   { return rsid_load_permille; }  
  
void rx_dsp_debug(void) {  
    Serial.printf("[RX] dsp=%u‰ rsid=%u‰ total=%u‰ cap_drop=%lu out_drop=%lu "  
                  "aud_drop=%lu clip=%lu aud_fill=%u\n",  
                  (unsigned)dsp_load_permille, (unsigned)rsid_load_permille,  
                  (unsigned)total_load_permille,  
                  (unsigned long)rx_capture_dropped(),  
                  (unsigned long)out_dropped, (unsigned long)aud_dropped,  
                  (unsigned long)clip_count, (unsigned)rx_dsp_aud_fill());  
}