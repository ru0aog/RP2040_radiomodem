// src/rsid/rsid.cpp  
// ============================================================================  
// RSID-декодер: обнаружение 15-символьной MFSK-посылки (шаг тона  
// 11025/1024 ≈ 10.77 Гц), определение моды и центральной частоты.  
//  
// Поток: rx_dsp (Farrow 15625→11025) -> rsid_rx_push() -> очередь ->  
// rsid_rx_service_sliced() на Core 1 -> rsid_poll() на Core 0 ->  
// rx_decoders_on_rsid(code).  
//  
// Примечание: rsid.h упоминает вход ~9775 Гц — см. RSID_FS_HZ ниже,  
// сверить с rsid_rr_step_q32 в rx_dsp.cpp (сейчас 15625/11025).  
// ============================================================================  
#include <Arduino.h>  
#include <string.h>  
#include <math.h>  
  
#include "rsid.h"  
  
// ---- Параметры ----  
#define RSID_FS_HZ        11025.0f        // TODO(verify): 9775 по rsid.h?  
#define RSID_FFT_N        512             // окно БПФ  
#define RSID_BIN_HZ       (RSID_FS_HZ / 1024.0f)   // 10.7666 Гц/бин  
#define RSID_NSYMBOLS     15              // символов в посылке  
#define RSID_INQ_DEPTH    512             // очередь отсчётов (SPSC)  
  
// ---- Таблица мод: RSID-код -> имя ----  
// TODO(verify): сверить с таблицей приёмника/FLDigi.  
typedef struct { uint16_t code; const char* name; } rsid_mode_t;  
static const rsid_mode_t rsid_modes[] = {  
    { 25,  "RTTY45"   },  
    { 28,  "RTTY50"   },  
    { 34,  "BPSK31"   },  
    { 36,  "QPSK31"   },  
    { 92,  "MFSK16"   },  
    { 94,  "MFSK8"    },  
    { 131, "JT65A"    },  
    { 173, "FT8"      },  
    // AX25/APRS не имеет официального RSID — локальный код для нашей сети:  
    { 999, "AX25-300" },  
};  
#define RSID_NMODES (sizeof(rsid_modes) / sizeof(rsid_modes[0]))  
  
// Gray-подстановка базиса 16 (по refs/rsid.cxx) — сверить с приёмником.  
static const uint8_t rsid_gray16[16] =  
    { 0, 1, 3, 2, 6, 4, 5, 7, 12, 8, 9, 10, 14, 15, 13, 11 };  
  
typedef struct { uint8_t sym[RSID_NSYMBOLS]; uint16_t code; } rsid_seq_t;  
static rsid_seq_t rsid_seqs[RSID_NMODES];  
static uint8_t    rsid_seqs_n = 0;  
  
// TODO(verify): кодирование последовательности по коду — порт cRsId::Encode  
// из приёмника. Реализация ниже — реконструкция (раскладка кода по тетрадам  
// базиса 16 с Gray-подстановкой), заменить верbatim-портом при переносе.  
static void rsid_encode(uint16_t code, uint8_t sym[RSID_NSYMBOLS]) {  
    uint32_t v = code;  
    for (int i = 0; i < RSID_NSYMBOLS; i++) {  
        sym[RSID_NSYMBOLS - 1 - i] = rsid_gray16[v & 0xF];  
        v >>= 4;  
        if (v == 0 && i < RSID_NSYMBOLS - 1) v = code >> (i % 9);  
    }  
}  
  
// ---- Входная очередь отсчётов (пишет Core 1 hot path из rx_dsp) ----  
static volatile int16_t  inq_buf[RSID_INQ_DEPTH];  
static volatile uint16_t inq_head = 0, inq_tail = 0;  
static volatile uint32_t rsid_sample_dropped = 0;  
  
void rsid_rx_push(int16_t sample) {  
    uint16_t next = (uint16_t)((inq_head + 1) % RSID_INQ_DEPTH);  
    if (next == inq_tail) { ++rsid_sample_dropped; return; }  
    inq_buf[inq_head] = sample;  
    __dmb();  
    inq_head = next;  
}  
  
// ---- Окно FFT и состояние ----  
static float    fft_re[RSID_FFT_N];  
static float    fft_im[RSID_FFT_N];  
static int16_t  win_buf[RSID_FFT_N];  
static uint16_t win_fill = 0;  
  
// Один найденный результат на диспетчеризацию (Core1 -> Core0)  
typedef struct {  
    uint16_t    code;  
    const char* name;  
    float       freq_hz;  
} rsid_res_t;  
  
static volatile bool res_pending = false;  
static rsid_res_t    res;  
  
// ---- Таблицы FFT (256 пар twiddle для N=512) ----  
static float    tw_re[RSID_FFT_N / 2];  
static float    tw_im[RSID_FFT_N / 2];  
static uint16_t bitrev_tab[RSID_FFT_N];  
  
static void rsid_tables_init(void) {  
    for (int k = 0; k < RSID_FFT_N / 2; k++) {  
        float a = -2.0f * PI * k / RSID_FFT_N;  
        tw_re[k] = cosf(a);  
        tw_im[k] = sinf(a);  
    }  
    for (int i = 0; i < RSID_FFT_N; i++) {  
        uint16_t x = (uint16_t)i, r = 0;  
        for (int b = 0; b < 9; b++) { r = (uint16_t)((r << 1) | (x & 1)); x >>= 1; }  
        bitrev_tab[i] = r;  
    }  
}  
  
// Radix-2 DIT БПФ на 512 точек, на месте.  
static void rsid_fft(void) {  
    for (int i = 0; i < RSID_FFT_N; i++) {  
        uint16_t j = bitrev_tab[i];  
        if (j > (uint16_t)i) {  
            float t = fft_re[i]; fft_re[i] = fft_re[j]; fft_re[j] = t;  
            t = fft_im[i]; fft_im[i] = fft_im[j]; fft_im[j] = t;  
        }  
    }  
    for (int len = 2; len <= RSID_FFT_N; len <<= 1) {  
        int half = len >> 1;  
        int step = RSID_FFT_N / len;  
        for (int i = 0; i < RSID_FFT_N; i += len) {  
            for (int j = 0; j < half; j++) {  
                float wr = tw_re[j * step], wi = tw_im[j * step];  
                int a = i + j, b = a + half;  
                float xr = fft_re[b] * wr - fft_im[b] * wi;  
                float xi = fft_re[b] * wi + fft_im[b] * wr;  
                fft_re[b] = fft_re[a] - xr; fft_im[b] = fft_im[a] - xi;  
                fft_re[a] += xr;            fft_im[a] += xi;  
            }  
        }  
    }  
}  
  
// Поиск RSID по спектру окна: корреляция с ожидаемыми 15 тонами каждой  
// моды по всем смещениям бина (центр сигнала неизвестен).  
static void rsid_search(void) {  
    static float mag[RSID_FFT_N / 2];  
    for (int i = 0; i < RSID_FFT_N / 2; i++)  
        mag[i] = fft_re[i] * fft_re[i] + fft_im[i] * fft_im[i];  
  
    float best = 0.0f;  
    int   best_mode = -1;  
    int   best_base = 0;  
  
    for (int m = 0; m < (int)rsid_seqs_n; m++) {  
        for (int base = 0; base < 200; base++) {    // до ~2.1 кГц  
            float acc = 0.0f;  
            int   ok  = 1;  
            for (int s = 0; s < RSID_NSYMBOLS && ok; s++) {  
                // TODO(verify): геометрия символа (шаг 34 бина, тон*2)  
                // — сверить с приёмником.  
                int off = base + s * 34 + rsid_seqs[m].sym[s] * 2;  
                if (off >= RSID_FFT_N / 2) { ok = 0; break; }  
                acc += mag[off] + mag[off + 1];  
            }  
            if (ok && acc > best) { best = acc; best_mode = m; best_base = base; }  
        }  
    }  
  
    if (best_mode >= 0 && best > 1e6f) {            // порог — калибровать!  
        if (!res_pending) {  
            res.code    = rsid_modes[best_mode].code;  
            res.name    = rsid_modes[best_mode].name;  
            res.freq_hz = (float)best_base * RSID_BIN_HZ;  
            __dmb();  
            res_pending = true;  
        }  
    }  
}  
  
// Срез работы RSID-детектора: вызывается из rx_dsp_poll() на Core 1  
// после каждого обработанного входного блока. Держать коротким.  
// ---- Конечный автомат RSID (мелкие срезы между DSP-блоками) ----  
enum RsidSm {  
    RSID_SM_COLLECT = 0,   // дренаж входной очереди в окно  
    RSID_SM_FFT     = 1,   // БПФ — по одной ступени (stage) за вызов, всего 9  
    RSID_SM_SEARCH  = 2,   // поиск — по одной моде за вызов  
    RSID_SM_PUBLISH = 3,   // запись результата  
};  
static volatile uint8_t rsid_sm = RSID_SM_COLLECT;  
static uint8_t  fft_stage   = 0;   // 0..8 (len = 2<<stage)  
static uint8_t  search_mode = 0;   // индекс моды в текущем проходе поиска  
  
// Результат частичного поиска — накапливается по мере перебора мод.  
static float search_best       = 0.0f;  
static int   search_best_mode  = -1;  
static int   search_best_base  = 0;  
static float search_mag[RSID_FFT_N / 2];  
  
// Одна ступень БПФ (len = 2..512). Делаем ровно один len за вызов.  
static void rsid_fft_stage(int stage) {  
    int len  = 2 << stage;          // 2,4,...,512  
    int half = len >> 1;  
    int step = RSID_FFT_N / len;  
    for (int i = 0; i < RSID_FFT_N; i += len) {  
        for (int j = 0; j < half; j++) {  
            float wr = tw_re[j * step], wi = tw_im[j * step];  
            int a = i + j, b = a + half;  
            float xr = fft_re[b] * wr - fft_im[b] * wi;  
            float xi = fft_re[b] * wi + fft_im[b] * wr;  
            fft_re[b] = fft_re[a] - xr; fft_im[b] = fft_im[a] - xi;  
            fft_re[a] += xr;            fft_im[a] += xi;  
        }  
    }  
}  
  
// Битреверсная перестановка — тоже отдельным срезом (stage == -1 условно;  
// проще: влить в первый вызов FFT до ступеней).  
static void rsid_bitrev_all(void) {  
    for (int i = 0; i < RSID_FFT_N; i++) {  
        uint16_t j = bitrev_tab[i];  
        if (j > (uint16_t)i) {  
            float t = fft_re[i]; fft_re[i] = fft_re[j]; fft_re[j] = t;  
            t = fft_im[i]; fft_im[i] = fft_im[j]; fft_im[j] = t;  
        }  
    }  
}  
  
// Один срез поиска: все базовые смещения для ОДНОЙ моды.  
static void rsid_search_slice(int m) {  
    for (int base = 0; base < 200; base++) {  
        float acc = 0.0f;  
        int   ok  = 1;  
        for (int s = 0; s < RSID_NSYMBOLS && ok; s++) {  
            int off = base + s * 34 + rsid_seqs[m].sym[s] * 2;  
            if (off >= RSID_FFT_N / 2) { ok = 0; break; }  
            acc += search_mag[off] + search_mag[off + 1];  
        }  
        if (ok && acc > search_best) {  
            search_best      = acc;  
            search_best_mode = m;  
            search_best_base = base;  
        }  
    }  
}  
  
void rsid_rx_service_sliced(void) {  
    switch (rsid_sm) {  
  
    case RSID_SM_COLLECT: {  
        // Дренаж очереди — ограниченный срез.  
        int budget = 64;  
        while (budget-- > 0 && inq_tail != inq_head &&  
               win_fill < RSID_FFT_N) {  
            win_buf[win_fill++] = inq_buf[inq_tail];  
            inq_tail = (uint16_t)((inq_tail + 1) % RSID_INQ_DEPTH);  
        }  
        if (win_fill >= RSID_FFT_N) {  
            // Загрузка окна в БПФ + скольжение на пол-окна.  
            for (int i = 0; i < RSID_FFT_N; i++) {  
                fft_re[i] = (float)win_buf[i];  
                fft_im[i] = 0.0f;  
            }  
            memmove(win_buf, win_buf + RSID_FFT_N / 2,  
                    (RSID_FFT_N / 2) * sizeof(int16_t));  
            win_fill  = RSID_FFT_N / 2;  
            fft_stage = -1;             // сначала битреверс  
            rsid_sm   = RSID_SM_FFT;  
        }  
        break;  
    }  
  
    case RSID_SM_FFT:  
        if (fft_stage < 0) {  
            rsid_bitrev_all();  
            fft_stage = 0;  
            break;                      // один срез — только перестановка  
        }  
        rsid_fft_stage(fft_stage);  
        if (++fft_stage >= 9) {         // ступеней 9 (len=2..512)  
            // Магнитуды — маленький срез, можно здесь же.  
            for (int i = 0; i < RSID_FFT_N / 2; i++)  
                search_mag[i] = fft_re[i] * fft_re[i] + fft_im[i] * fft_im[i];  
            search_best      = 0.0f;  
            search_best_mode = -1;  
            search_best_base = 0;  
            search_mode      = 0;  
            rsid_sm          = RSID_SM_SEARCH;  
        }  
        break;  
  
    case RSID_SM_SEARCH:  
        rsid_search_slice(search_mode);  
        if (++search_mode >= (int)rsid_seqs_n)  
            rsid_sm = RSID_SM_PUBLISH;  
        break;  
  
    case RSID_SM_PUBLISH:  
        if (search_best_mode >= 0 && search_best > 1e6f && !res_pending) {  
            res.code    = rsid_modes[search_best_mode].code;  
            res.name    = rsid_modes[search_best_mode].name;  
            res.freq_hz = (float)search_best_base * RSID_BIN_HZ;  
            __dmb();  
            res_pending = true;  
        }  
        rsid_sm = RSID_SM_COLLECT;      // следующее окно  
        break;  
    }  
}
  
// ---- Диспетчеризация результата на Core 0 ----  
extern void rx_decoders_on_rsid(uint16_t code);    // из rx_decoders.cpp  
  
static void rsid_dispatch(void) {  
    if (!res_pending) return;  
    rsid_res_t r = res;  
    res_pending = false;  
    Serial.printf("# RSID mode=%s code=%u freq=%.1f Hz\n",  
                  r.name ? r.name : "?", (unsigned)r.code,  
                  (double)r.freq_hz);  
    rx_decoders_on_rsid(r.code);  
}  
  
// ---- API по rsid.h ----  
  
void rsid_init(void) {  
    rsid_tables_init();  
    for (int i = 0; i < (int)RSID_NMODES; i++) {  
        rsid_seqs[i].code = rsid_modes[i].code;  
        rsid_encode(rsid_modes[i].code, rsid_seqs[i].sym);  
    }  
    rsid_seqs_n = RSID_NMODES;  
    Serial.println("[RSID] init");  
}  
  
void rsid_poll(void) {  
    rsid_dispatch();        // Core 0: печать метки + смена моды  
}