// src/rsid/rsid.cpp  
// ============================================================================  
// RSID-декодер — верbatim-порт RP2040_receiver/src/rsid/rx_rsid.cpp.  
//  
// Адаптации под rsid.h модема:  
//  - имена API: rsid_rx_init(float) -> rsid_init(void);  
//    rsid_rx_dispatch() -> rsid_poll() (Core 0);  
//  - результаты детекта дополнительно уходят в rx_decoders_on_rsid(code);  
//  - C++-линковка (rsid.h без extern "C");  
//  - rtty_center_* оставлены локальными (API пока не используется).  
// Вход: отсчёты 11025 Гц от rsid_resampler_push() в rx_dsp (Core 1).  
// ============================================================================  
#include "rsid.h"  
#include "../rx/rx_decoders.h"      // rx_decoders_on_rsid()  
#include <Arduino.h>  
#include <math.h>  
#include <string.h>  
  
// ============================================================================  
//  Константы RSID (refs/rsid.h). Вход уже 11025 Гц.  
// ============================================================================  
#define RSID_SAMPLE_RATE   11025.0  
#define RSID_FFT_SAMPLES   512  
#define RSID_FFT_SIZE      1024  
#define RSID_ARRAY_SIZE    (RSID_FFT_SIZE * 2)     // 2048  
#define RSID_BUFFER_SIZE   (RSID_ARRAY_SIZE * 2)   // 4096  
#define RSID_NSYMBOLS      15  
#define RSID_NTIMES        (RSID_NSYMBOLS * 2)      // 30  
#define RSID_ESCAPE        6  
#define RSID_NONE2         0  
  
#define RSID_FFT_N         RSID_ARRAY_SIZE          // 2048  
#define RSID_FFT_HALF      (RSID_FFT_N / 2)  
  
#define RSID_SECONDARY_WINDOWS   60  
  
// Полоса поиска: 300..3000 Гц (бин FFT-2048 @11025 = 5.383 Гц).  
#define RSID_BIN_LOW   ((int)(300.0  * 2048.0 / RSID_SAMPLE_RATE))                // 55  
#define RSID_BIN_HIGH  ((int)(3000.0 * 2048.0 / RSID_SAMPLE_RATE) + RSID_NTIMES)  // 587  
  
#define RSID_SAMPLE_Q_SIZE 1024u  
#define RSID_CHUNK         128  
#define RSID_WIN_SLOTS     4  
  
// Бюджет времени на один заход (мкс) — ключевое отличие от моей  
// реконструкции: нарезка по времени, а не "одна стадия за вызов".  
#ifndef RSID_SLICE_BUDGET_US  
#define RSID_SLICE_BUDGET_US 120u  
#endif  
  
// ============================================================================  
//  Таблицы мод (code -> имя)  
// ============================================================================  
struct RSIDmap { unsigned short rs; const char* name; };  
  
#define RSID_LIST \
    ELEM_(263, "EOT")            ELEM_(6,   "ESCAPE")        \
    ELEM_(26,  "CW")                                         \
    ELEM_(39,  "RTTY-45")        ELEM_(40, "RTTY-50")        \
    ELEM_(49,  "CONT-8/250")                                 \
    ELEM_(69,  "OLIVIA-8/250")   ELEM_(70, "AX25")           \
    ELEM_(86, "DOMEX8")          ELEM_(137,"THOR8")          \
    ELEM_(143,"THOR11")          ELEM_(57, "MFSK16")         \
    ELEM_(0,   "NONE")
  
#define ELEM_(code_, name_) { code_, name_ },  
static const RSIDmap rsid_ids_1[] = { RSID_LIST };  
#undef ELEM_  
static const int rsid_ids_size1 =  
    (int)(sizeof(rsid_ids_1) / sizeof(rsid_ids_1[0])) - 1;  
  
#define RSID_LIST2 \  
    ELEM2_(1171, "IFKP")  \  
    ELEM2_(1172, "JTTY")  \  
    ELEM2_(0,    "NONE2")  
  
#define ELEM2_(code_, name_) { code_, name_ },  
static const RSIDmap rsid_ids_2[] = { RSID_LIST2 };  
#undef ELEM2_  
static const int rsid_ids_size2 =  
    (int)(sizeof(rsid_ids_2) / sizeof(rsid_ids_2[0])) - 1;  
  
// ============================================================================  
//  Reed-Solomon кодер (refs/rsid.cxx)  
// ============================================================================  
static const int Squares[256] = {  
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,  
    0, 2, 4, 6, 8,10,12,14, 9,11,13,15, 1, 3, 5, 7,  
    0, 3, 6, 5,12,15,10, 9, 1, 2, 7, 4,13,14,11, 8,  
    0, 4, 8,12, 9,13, 1, 5,11,15, 3, 7, 2, 6,10,14,  
    0, 5,10,15,13, 8, 7, 2, 3, 6, 9,12,14,11, 4, 1,  
    0, 6,12,10, 1, 7,13,11, 2, 4,14, 8, 3, 5,15, 9,  
    0, 7,14, 9, 5, 2,11,12,10,13, 4, 3,15, 8, 1, 6,  
    0, 8, 9, 1,11, 3, 2,10,15, 7, 6,14, 4,12,13, 5,  
    0, 9,11, 2,15, 6, 4,13, 7,14,12, 5, 8, 1, 3,10,  
    0,10,13, 7, 3, 9,14, 4, 6,12,11, 1, 5,15, 8, 2,  
    0,11,15, 4, 7,12, 8, 3,14, 5, 1,10, 9, 2, 6,13,  
    0,12, 1,13, 2,14, 3,15, 4, 8, 5, 9, 6,10, 7,11,  
    0,13, 3,14, 6,11, 5, 8,12, 1,15, 2,10, 7, 9, 4,  
    0,14, 5,11,10, 4,15, 1,13, 3, 8, 6, 7, 9, 2,12,  
    0,15, 7, 8,14, 1, 9, 6, 5,10, 2,13,11, 4,12, 3  
};  
static const int indices[12] = { 2, 4, 8, 9, 11, 15, 7, 14, 5, 10, 13, 3 };  
  
static void rsid_encode(int code, unsigned char* rsid /*RSID_NSYMBOLS*/) {  
    rsid[0] = code >> 8;  
    rsid[1] = (code >> 4) & 0x0f;  
    rsid[2] = code & 0x0f;  
    for (int i = 3; i < RSID_NSYMBOLS; i++) rsid[i] = 0;  
    for (int i = 0; i < 12; i++) {  
        for (int j = RSID_NSYMBOLS - 1; j > 0; j--)  
            rsid[j] = rsid[j - 1] ^ Squares[(rsid[j] << 4) + indices[i]];  
        rsid[0] = Squares[(rsid[0] << 4) + indices[i]];  
    }  
}  
  
// ============================================================================  
//  Состояние и буферы  
// ============================================================================  
static int16_t rsid_sample_queue[RSID_SAMPLE_Q_SIZE];  
static volatile uint32_t rsid_sq_head = 0;  
static volatile uint32_t rsid_sq_tail = 0;  
  
static volatile uint32_t rsid_sample_dropped = 0;  
static volatile uint32_t rsid_window_dropped = 0;  
static volatile uint32_t rsid_service_max_us = 0;  
static volatile uint32_t rsid_service_calls_total = 0;  
  
static inline int16_t sat16(int32_t x) {  
    if (x > 32767) return 32767;  
    if (x < -32768) return -32768;  
    return (int16_t)x;  
}  
  
static inline void rsid_service_record_time(uint32_t start_us) {  
    const uint32_t elapsed = micros() - start_us;  
    rsid_service_calls_total++;  
    if (elapsed > rsid_service_max_us) rsid_service_max_us = elapsed;  
}  
  
// Диагностика (опционально — вызывать из rx_dsp_debug/modem).  
uint32_t rsid_rx_dropped(void)        { return rsid_sample_dropped + rsid_window_dropped; }  
uint32_t rsid_rx_sample_dropped(void) { return rsid_sample_dropped; }  
uint32_t rsid_rx_window_dropped(void) { return rsid_window_dropped; }  
uint32_t rsid_rx_queue_fill(void)     { return (rsid_sq_head - rsid_sq_tail) % RSID_SAMPLE_Q_SIZE; }  
uint32_t rsid_rx_service_max_us(void) { return rsid_service_max_us; }  
uint32_t rsid_rx_service_calls(void)  { return rsid_service_calls_total; }  
  
// --- Стейт-машина ---  
enum rsid_state_t {  
    RSID_STATE_IDLE = 0,  
    RSID_STATE_WINDOWING,  
    RSID_STATE_FFT_BITREVERSE,  
    RSID_STATE_FFT_STAGE,  
    RSID_STATE_CALC_AMPL,  
    RSID_STATE_SINK_AND_BUCKETS,  
    RSID_STATE_SEARCH  
};  
static rsid_state_t rsid_sm_state = RSID_STATE_IDLE;  
uint32_t rsid_rx_sm_state(void) { return (uint32_t)rsid_sm_state; }  
  
// Итераторы нарезанных стадий — живут между квантами.  
static uint16_t fft_len = 2;  
static uint16_t fft_base = 0;  
static uint16_t w_i      = 0;  
static uint16_t br_i     = 1;  
static uint16_t br_j     = 0;  
static uint16_t fft_j2   = 0;  
static int      ampl_i   = 0;  
static int      bkt_phase = 0;  
static int      bkt_row   = 0;  
static int      bkt_i     = 0;  
static int      bkt_imax  = 0;  
static uint64_t bkt_ampmax = 0;  
  
static bool srch_active    = false;  
static int  srch_i         = 0;  
static int  srch_best_dist = 1000;  
static int  srch_best_bin  = -1;  
static int  srch_best_sym  = -1;  
  
static bool           rsid_inited = false;  
  
static unsigned char  pCodes1[sizeof(rsid_ids_1) / sizeof(rsid_ids_1[0]) * RSID_NSYMBOLS];  
static unsigned char  pCodes2[sizeof(rsid_ids_2) / sizeof(rsid_ids_2[0]) * RSID_NSYMBOLS];  
  
static int16_t        aInputSamples[RSID_BUFFER_SIZE];           // 8K  
static int16_t        window_q15[RSID_FFT_N];                    // 4K  
static int16_t        tw_re[RSID_FFT_HALF];  
static int16_t        tw_im[RSID_FFT_HALF];  
static int16_t        fft_i[RSID_FFT_N];                         // 4K  
static int16_t        fft_q[RSID_FFT_N];                         // 4K  
static uint64_t       aFFTAmpl[RSID_FFT_SIZE];                   // 8K  
static int16_t        fft_buckets[RSID_NTIMES][RSID_FFT_SIZE];   // 60K!  
static int16_t        rsid_proc[RSID_WIN_SLOTS][RSID_ARRAY_SIZE];// 16K  
static uint8_t rsid_win_head = 0;  
static uint8_t rsid_win_tail = 0;  
static uint8_t rsid_cur_slot = 0;  
  
static int   inptr;  
static int   nBinLow, nBinHigh;  
static int   hamming_resolution = 1;    // RSID_HAMMING_RESOLUTION  
static int   rsid_secondary_windows = 0;  
  
// Почтовый ящик детекта: пишется на Core 1, читается на Core 0.  
typedef struct { int mode_code; const char* mode_name; float freq_hz; } rsid_result_t;  
static volatile bool rsid_det_pending = false;  
static rsid_result_t rsid_det_result;  
  
static float         rtty_center_hz = 0.0f;  
static volatile bool rtty_center_new = false;  
  
// ============================================================================  
//  Служебные  
// ============================================================================  
static inline double rsid_hamming(double x) {  
    return 0.54 - 0.46 * cos(2.0 * M_PI * x);  
}  
  
static void rsid_reset_detect(void) {  
    memset(aFFTAmpl,    0, sizeof(aFFTAmpl));  
    memset(fft_buckets, 0, sizeof(fft_buckets));  
    hamming_resolution = 1;  
    rsid_secondary_windows = 0;  
    srch_i = 0;  
    srch_active = false;  
    rsid_sm_state = RSID_STATE_IDLE;  
    // Индексы кольца не трогаем — см. комментарий в оригинале.  
}  
  
static void rsid_reset_state(void) {  
    inptr = RSID_FFT_SIZE;  
    memset(aInputSamples, 0, sizeof(aInputSamples));  
    rsid_reset_detect();  
}  
  
static inline void buckets_begin(int iBegin) {  
    bkt_i      = iBegin;  
    bkt_imax   = iBegin - 2;  
    bkt_ampmax = 0;  
}  
  
static bool buckets_step(const uint64_t* pSpectrum, int iEnd) {  
    int n = 0;  
    for (; bkt_i < iEnd && n < RSID_CHUNK; bkt_i += 2, n++) {  
        const int i = bkt_i;  
        if (bkt_imax == i - 2) {  
            bkt_ampmax = pSpectrum[i];  
            bkt_imax = i;  
            for (int j = i + 2; j < i + RSID_NTIMES + 2; j += 2) {  
                const uint64_t Amp = pSpectrum[j];  
                if (Amp > bkt_ampmax) { bkt_ampmax = Amp; bkt_imax = j; }  
            }  
        } else {  
            const int j = i + RSID_NTIMES;  
            const uint64_t Amp = pSpectrum[j];  
            if (Amp > bkt_ampmax) { bkt_ampmax = Amp; bkt_imax = j; }  
        }  
        fft_buckets[RSID_NTIMES - 1][i] = (int16_t)((bkt_imax - i) >> 1);  
    }  
    return (bkt_i >= iEnd);  
}  
  
static inline int HammingDistance(int iBucket, const unsigned char* p2) {  
    int dist = 0;  
    for (int i = 0, j = 1; i < RSID_NSYMBOLS; i++, j += 2) {  
        if (fft_buckets[j][iBucket] != p2[i]) {  
            if (++dist > hamming_resolution) break;  
        }  
    }  
    return dist;  
}  
  
static bool search_amp_step(const unsigned char* pCodes,  
                            const RSIDmap* ids, int ids_size) {  
    if (!srch_active) {  
        srch_i = 0;  
        srch_best_dist = 1000;  
        srch_best_bin  = -1;  
        srch_best_sym  = -1;  
        srch_active = true;  
    }  
    const unsigned char* pc = pCodes + srch_i * RSID_NSYMBOLS;  
    for (int j = nBinLow; j < nBinHigh - RSID_NTIMES; j++) {  
        int d = HammingDistance(j, pc);  
        if (d < srch_best_dist) {  
            srch_best_dist = d;  
            srch_best_sym  = ids[srch_i].rs;  
            srch_best_bin  = j;  
            if (srch_best_dist == 0) break;  
        }  
    }  
    srch_i++;  
    if (srch_i >= ids_size || srch_best_dist == 0) {  
        srch_active = false;  
        return true;  
    }  
    return false;  
}  
  
static void rsid_apply(int iBin, int iSymbol,  
                       const RSIDmap* ids, int ids_size) {  
    double rsidfreq = (iBin + RSID_NSYMBOLS - 0.5) * RSID_SAMPLE_RATE / 2048.0;  
    const char* name = nullptr;  
    for (int n = 0; n < ids_size; n++) {  
        if (ids[n].rs == iSymbol) { name = ids[n].name; break; }  
    }  
    if (!name) return;  
    rsid_det_result.mode_code = iSymbol;  
    rsid_det_result.mode_name = name;  
    rsid_det_result.freq_hz   = (float)rsidfreq;  
    __dmb();  
    rsid_det_pending = true;  
}  
  
// ============================================================================  
//  Публичный интерфейс (rsid.h модема)  
// ============================================================================  
void rsid_init(void) {  
    for (int i = 0; i < RSID_FFT_N; i++)  
        window_q15[i] = sat16((int32_t)(rsid_hamming(1.0 * i / RSID_FFT_N) * 32767.0 + 0.5));  
    for (int k = 0; k < RSID_FFT_HALF; k++) {  
        const double a = -6.2831853071795864769 * (double)k / (double)RSID_FFT_N;  
        tw_re[k] = sat16((int32_t)(cos(a) * 32767.0));  
        tw_im[k] = sat16((int32_t)(sin(a) * 32767.0));  
    }  
    memset(pCodes1, 0, sizeof(pCodes1));  
    for (int i = 0; i < rsid_ids_size1; i++)  
        rsid_encode(rsid_ids_1[i].rs, pCodes1 + i * RSID_NSYMBOLS);  
    memset(pCodes2, 0, sizeof(pCodes2));  
    for (int i = 0; i < rsid_ids_size2; i++)  
        rsid_encode(rsid_ids_2[i].rs, pCodes2 + i * RSID_NSYMBOLS);  
  
    rsid_sq_head = rsid_sq_tail = 0;  
    rsid_win_head = rsid_win_tail = rsid_cur_slot = 0;  
    rsid_sample_dropped = rsid_window_dropped = 0;  
    rsid_service_max_us = rsid_service_calls_total = 0;  
  
    rsid_reset_state();  
    rsid_inited = true;  
    Serial.println("[RSID] init (verbatim port)");  
}  
  
void __not_in_flash_func(rsid_rx_push)(int16_t s) {  
    if (!rsid_inited) return;  
    uint32_t head = rsid_sq_head;  
    uint32_t next = (head + 1u) % RSID_SAMPLE_Q_SIZE;  
    if (next != rsid_sq_tail) {  
        rsid_sample_queue[head] = s;  
        __dmb();  
        rsid_sq_head = next;  
    } else {  
        rsid_sample_dropped++;  
    }  
}  
  
// Core 0: печать метки + автодетект моды (было rsid_rx_dispatch).  
void rsid_poll(void) {  
    if (!rsid_det_pending) return;  
    rsid_det_pending = false;  
    __dmb();  
    Serial.printf("# RSID mode=%s code=%d freq=%ld\n",  
              rsid_det_result.mode_name,  
              rsid_det_result.mode_code,  
              (long)(rsid_det_result.freq_hz + 0.5f)); 
    rx_decoders_on_rsid((uint16_t)rsid_det_result.mode_code);  
}  
  
bool rsid_get_rtty_center(float* f_hz) {  
    if (!rtty_center_new) return false;  
    rtty_center_new = false;  
    if (f_hz) *f_hz = rtty_center_hz;  
    return true;  
}  
  
// ============================================================================  
//  Нарезанная стейт-машина — Core 1, бюджет RSID_SLICE_BUDGET_US за заход.  
// ============================================================================  
void __not_in_flash_func(rsid_rx_service_sliced)(void) {  
    const uint32_t t_service_start = micros();  
  
    if (!rsid_inited) {  
        rsid_service_record_time(t_service_start);  
        return;  
    }  
  
    // ФАЗА 0: сбор сэмплов из очереди — не более 256 за вызов.  
    for (int n = 0; n < 256 && rsid_sq_tail != rsid_sq_head; n++) {  
        uint32_t tail = rsid_sq_tail;  
        int16_t sample = rsid_sample_queue[tail];  
        rsid_sq_tail = (tail + 1u) % RSID_SAMPLE_Q_SIZE;  
        __dmb();  
  
        aInputSamples[inptr++] = sample;  
        if (inptr >= RSID_ARRAY_SIZE) {  
            uint8_t nh = (uint8_t)((rsid_win_head + 1) % RSID_WIN_SLOTS);  
            if (nh != rsid_win_tail) {  
                memcpy(rsid_proc[rsid_win_head], aInputSamples,  
                       RSID_ARRAY_SIZE * sizeof(int16_t));  
                rsid_win_head = nh;  
            } else {  
                rsid_window_dropped++;  
            }  
            memmove(&aInputSamples[0], &aInputSamples[RSID_FFT_SAMPLES],  
                    (RSID_BUFFER_SIZE - RSID_FFT_SAMPLES) * sizeof(int16_t));  
            inptr -= RSID_FFT_SAMPLES;  
        }  
    }  
  
    // ФАЗА 1: кванты стейт-машины в пределах таймбюджета.  
    const uint32_t t_slice_start = micros();  
    do {  
        if (rsid_sm_state == RSID_STATE_IDLE && rsid_win_tail == rsid_win_head) break;  
  
        switch (rsid_sm_state) {  
  
        case RSID_STATE_IDLE:  
            if (rsid_win_tail != rsid_win_head) {  
                rsid_cur_slot = rsid_win_tail;  
                w_i = 0; br_i = 1; br_j = 0;  
                fft_j2 = 0;  
                ampl_i = RSID_BIN_LOW - 2;  
                bkt_phase = 0; bkt_row = 0;  
                rsid_sm_state = RSID_STATE_WINDOWING;  
            }  
            break;  
  
        case RSID_STATE_WINDOWING:  
            if (w_i == 0) {  
                nBinLow  = RSID_BIN_LOW;  
                nBinHigh = RSID_BIN_HIGH;  
            }  
            for (int n = 0; n < RSID_CHUNK && w_i < RSID_FFT_N; n++, w_i++) {  
                int32_t xi = (int32_t)rsid_proc[rsid_cur_slot][w_i];  
                fft_i[w_i] = (int16_t)(((int32_t)xi * window_q15[w_i]) >> 15);  
                fft_q[w_i] = 0;  
            }  
            if (w_i >= RSID_FFT_N)  
                rsid_sm_state = RSID_STATE_FFT_BITREVERSE;  
            break;  
  
        case RSID_STATE_FFT_BITREVERSE: {  
            int n = 0;  
            for (; br_i < RSID_FFT_N && n < RSID_CHUNK; ++br_i, ++n) {  
                uint16_t bit = RSID_FFT_HALF;  
                while (br_j & bit) { br_j ^= bit; bit >>= 1; }  
                br_j ^= bit;  
                if (br_i < br_j) {  
                    int16_t t;  
                    t = fft_i[br_i]; fft_i[br_i] = fft_i[br_j]; fft_i[br_j] = t;  
                    t = fft_q[br_i]; fft_q[br_i] = fft_q[br_j]; fft_q[br_j] = t;  
                }  
            }  
            if (br_i >= RSID_FFT_N) {  
                fft_len = 2;  
                fft_base = 0;  
                rsid_sm_state = RSID_STATE_FFT_STAGE;  
            }  
            break;  
        }  
  
        case RSID_STATE_FFT_STAGE: {  
            const uint16_t half = fft_len >> 1;  
            const uint16_t tw_step = RSID_FFT_N / fft_len;  
  
            for (uint16_t n = 0; n < RSID_CHUNK && fft_j2 < half; ++fft_j2, ++n) {  
                const uint16_t tw = fft_j2 * tw_step;  
                const int32_t wr = tw_re[tw];  
                const int32_t wi = tw_im[tw];  
                const uint16_t a = fft_base + fft_j2;  
                const uint16_t b = a + half;  
                const int32_t br = fft_i[b], bi = fft_q[b];  
                const int32_t tr = (br * wr - bi * wi) >> 15;  
                const int32_t ti = (br * wi + bi * wr) >> 15;  
                const int32_t ar = fft_i[a], ai = fft_q[a];  
                fft_i[a] = sat16((ar + tr) >> 1);  
                fft_q[a] = sat16((ai + ti) >> 1);  
                fft_i[b] = sat16((ar - tr) >> 1);  
                fft_q[b] = sat16((ai - ti) >> 1);  
            }  
  
            if (fft_j2 >= half) {  
                fft_j2 = 0;  
                fft_base += fft_len;  
                if (fft_base >= RSID_FFT_N) {  
                    fft_base = 0;  
                    fft_len <<= 1;  
                    if (fft_len > RSID_FFT_N)  
                        rsid_sm_state = RSID_STATE_CALC_AMPL;  
                }  
            }  
            break;  
        }  
  
        case RSID_STATE_CALC_AMPL:  
            for (int n = 0; n < RSID_CHUNK && ampl_i < RSID_BIN_HIGH; n++, ampl_i++) {  
                const int32_t re = fft_i[ampl_i];  
                const int32_t im = fft_q[ampl_i];  
                aFFTAmpl[ampl_i] = (uint64_t)((int64_t)re * re + (int64_t)im * im);  
            }  
            if (ampl_i >= RSID_BIN_HIGH)  
                rsid_sm_state = RSID_STATE_SINK_AND_BUCKETS;  
            break;  
  
        case RSID_STATE_SINK_AND_BUCKETS: {  
            const int bw   = RSID_BIN_HIGH - RSID_BIN_LOW;  
            const int iEnd = RSID_BIN_HIGH - RSID_NTIMES;  
  
            switch (bkt_phase) {  
            case 0: {  
                int n = 0;  
                for (; bkt_row < RSID_NTIMES - 1 && n < 8; bkt_row++, n++)  
                    memmove(&fft_buckets[bkt_row][RSID_BIN_LOW],  
                            &fft_buckets[bkt_row + 1][RSID_BIN_LOW],  
                            bw * sizeof(fft_buckets[0][0]));  
                if (bkt_row >= RSID_NTIMES - 1) bkt_phase = 1;  
                break;  
            }  
            case 1:  
                memset(&fft_buckets[RSID_NTIMES - 1][RSID_BIN_LOW], 0,  
                       bw * sizeof(fft_buckets[0][0]));  
                buckets_begin(RSID_BIN_LOW);  
                bkt_phase = 2;  
                break;  
            case 2:  
                if (buckets_step(aFFTAmpl, iEnd)) {  
                    buckets_begin(RSID_BIN_LOW + 1);  
                    bkt_phase = 3;  
                }  
                break;  
            case 3:  
                if (buckets_step(aFFTAmpl, iEnd))  
                    rsid_sm_state = RSID_STATE_SEARCH;  
                break;  
            }  
            break;  
        }  
  
        case RSID_STATE_SEARCH: {  
            const bool            primary  = (rsid_secondary_windows <= 0);  
            const unsigned char*  pCodes   = primary ? pCodes1       : pCodes2;  
            const RSIDmap*        ids      = primary ? rsid_ids_1    : rsid_ids_2;  
            const int             ids_size = primary ? rsid_ids_size1 : rsid_ids_size2;  
  
            if (!search_amp_step(pCodes, ids, ids_size))  
                break;  
  
            const bool found = (srch_best_dist <= hamming_resolution);  
            const int  bin   = srch_best_bin;  
            const int  sym   = srch_best_sym;  
  
            if (primary) {  
                if (found) {  
                    if (sym != RSID_ESCAPE) {  
                        rsid_apply(bin, sym, rsid_ids_1, rsid_ids_size1);  
                        rtty_center_hz = (float)((bin + RSID_NSYMBOLS - 0.5) * RSID_SAMPLE_RATE / 2048.0);  
                        __dmb();  
                        rtty_center_new = true;  
                        rsid_reset_detect();  
                    } else {  
                        rsid_secondary_windows = RSID_SECONDARY_WINDOWS;  
                    }  
                }  
            } else {  
                rsid_secondary_windows--;  
                if (found) {  
                    if (sym != RSID_NONE2) {  
                        rsid_apply(bin, sym, rsid_ids_2, rsid_ids_size2);  
                        rsid_reset_detect();  
                    }  
                }  
                if (rsid_secondary_windows <= 0)  
                    rsid_secondary_windows = 0;  
            }  
  
            rsid_win_tail = (uint8_t)((rsid_win_tail + 1) % RSID_WIN_SLOTS);  
            rsid_sm_state = RSID_STATE_IDLE;  
            break;  
        }  
        }  
    } while ((uint32_t)(micros() - t_slice_start) < RSID_SLICE_BUDGET_US);  
  
    rsid_service_record_time(t_service_start);  
}

uint32_t rsid_last_freq_hz(void) {  
    return (uint32_t)(rsid_det_result.freq_hz + 0.5f);  
}
