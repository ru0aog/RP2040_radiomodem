// src/rsid/rsid.cpp  
// ============================================================================  
// RSID-декодер: обнаружение 15-символьной MFSK-посылки (шаг тона  
// 11025/1024 ≈ 10.77 Гц), определение моды и центральной частоты.  
//  
// Поток: rx_dsp (Farrow 15625→11025) -> rsid_rx_push() -> очередь ->  
// rsid_rx_service_sliced() на Core 1 -> rsid_rx_dispatch() на Core 0 ->  
// rx_decoders_on_rsid(code, freq_hz).  
//  
// ВНИМАНИЕ: функции extern "C" — объявления в rx_dsp.cpp совпадают.  
// ============================================================================  
#include <Arduino.h>  
#include <string.h>  
#include <math.h>  
  
#include "rsid.h"  
  
// ---- Параметры (по rx_rsid.h приёмника) ----  
#define RSID_FS_HZ        11025.0f  
#define RSID_FFT_N        512            // окно БПФ  
#define RSID_BIN_HZ       (RSID_FS_HZ / 1024.0f)   // 10.7666 Гц/бин  
#define RSID_NSYMBOLS     15             // символов в посылке  
#define RSID_SYM_BINS     1024           // символ = 1024 бина  
#define RSID_HAMMING_MAX  1              // допустимое расстояние Хэмминга  
#define RSID_INQ_DEPTH    512            // очередь отсчётов (SPSC)  
  
// ---- Таблица мод: RSID-код → имя ----  
// TODO(verify): сверить с таблицей приёмника/FLDigi; коды ниже — по  
// стандартной таблице RSID (поле code — десятичный индекс в таблице fldigi).  
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
  
// RSID-последовательность: код моды (9 бит) раскладывается на 15 MFSK-  
// символов базиса 16 (как в refs/rsid.cxx). Ожидаемые тоны считаем по  
// формуле pgm — таблицу Gray-подстановки сверить с приёмником!  
static const uint8_t rsid_gray16[16] =  
    { 0, 1, 3, 2, 6, 4, 5, 7, 12, 8, 9, 10, 14, 15, 13, 11 };  
  
typedef struct { uint8_t sym[RSID_NSYMBOLS]; uint16_t code; } rsid_seq_t;  
static rsid_seq_t rsid_seqs[RSID_NMODES];  
static uint8_t    rsid_seqs_n = 0;  
  
// TODO(verify): кодирование последовательности по коду — порт cRsId::Encode.  
// Вариант: код разбивается на 5 тетрад (старшая→младшая), каждая тетрада  
// даёт 3 символа через базис-16 разложение. Точную формулу перенести  
// из приёмника при переносе таблицы.  
static void rsid_encode(uint16_t code, uint8_t sym[RSID_NSYMBOLS]) {  
    // Разложение кода на 15 символов базиса 16: code = sum sym[i]*16^k  
    // с поправкой кодирования fldigi (взвешенные степени).  
    // Заглушечная реализация: позиции тетрад + детерминированные тоны.  
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
  
extern "C" void rsid_rx_push(int16_t s) {  
    uint16_t next = (uint16_t)((inq_head + 1) % RSID_INQ_DEPTH);  
    if (next == inq_tail) { ++rsid_sample_dropped; return; }  
    inq_buf[inq_head] = s;  
    __dmb();  
    inq_head = next;  
}  
  
// ---- Окно FFT и состояние ----  
static float    fft_re[RSID_FFT_N];  
static float    fft_im[RSID_FFT_N];  
static int16_t  win_buf[RSID_FFT_N];  
static uint16_t win_fill = 0;  
static volatile uint8_t  rsid_sm = 0;        // 0=collect, 1=fft, 2=search  
static volatile uint32_t rsid_window_dropped = 0;  
  
// Один найденный результат на диспетчеризацию (Core1→Core0)  
static volatile bool     res_pending = false;  
static rsid_result_t     res;  
  
// ---- Таблицы FFT (256 пар twiddle для N=512) ----  
static float tw_re[RSID_FFT_N / 2];  
static float tw_im[RSID_FFT_N / 2];  
static uint16_t bitrev_tab[RSID_FFT_N];  
  
static void rsid_tables_init(void) {  
    for (int k = 0; k < RSID_FFT_N / 2; k++) {  
        float a = -2.0f * PI * k / RSID_FFT_N;  
        tw_re[k] = cosf(a);  
        tw_im[k] = sinf(a);  
    }  
    for (int i = 0; i < RSID_FFT_N; i++) {  
        uint16_t x = i, r = 0;  
        for (int b = 0; b < 9; b++) { r = (r << 1) | (x & 1); x >>= 1; }  
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
  
// Поиск RSID по спектру окна: ищем корреляцию с ожидаемыми 15 тонами  
// каждой моды по всем смещениям бина (центр сигнала неизвестен).  
static void rsid_search(void) {  
    // Спектр мощности  
    static float mag[RSID_FFT_N / 2];  
    for (int i = 0; i < RSID_FFT_N / 2; i++)  
        mag[i] = fft_re[i] * fft_re[i] + fft_im[i] * fft_im[i];  
  
    float best = 0.0f; int best_mode = -1; int best_base = 0;  
    for (int m = 0; m < (int)rsid_seqs_n; m++) {  
        // Базовый тон последовательности пробегает 0..(512-RSID span)  
        for (int base = 0; base < 200; base++) {   // до ~2.1 кГц  
            float acc = 0.0f;  
            int ok = 1;  
            for (int s = 0; s < RSID_NSYMBOLS && ok; s++) {  
                // Символ s занимает бины base+s*64 .. +64; ожидаемый тон  
                // — пик на rsid_seqs[m].sym[s] внутри подокна 64 бина  
                // (64 бина/символ: 11025/1024*64*15 ≈ посылка ~0.93с…  
                //  TODO(verify): геометрию символа сверить с приёмником)  
                int off = base + s * 34 + rsid_seqs[m].sym[s] * 2;  
                if (off >= RSID_FFT_N / 2) { ok = 0; break; }  
                acc += mag[off] + mag[off + 1];  
            }  
            if (ok && acc > best) { best = acc; best_mode = m; best_base = base; }  
        }  
    }  
    if (best_mode >= 0 && best > 1e6f) {           // порог — калибровать!  
        if (!res_pending) {  
            res.mode_code = rsid_modes[best_mode].code;  
            res.mode_name = rsid_modes[best_mode].name;  
            res.freq_hz   = (float)best_base * RSID_BIN_HZ;  
            __dmb();  
            res_pending = true;  
        }  
    }  
}  
  
extern "C" void rsid_rx_service_sliced(void) {  
    // Накапливаем окно из очереди  
    while (inq_tail != inq_head && win_fill < RSID_FFT_N) {  
        win_buf[win_fill++] = inq_buf[inq_tail];  
        inq_tail = (uint16_t)((inq_tail + 1) % RSID_INQ_DEPTH);  
    }  
    if (win_fill < RSID_FFT_N) return;  
  
    // Перекрытие 50%: вторая половина становится началом  
    for (int i = 0; i < RSID_FFT_N; i++) {  
        fft_re[i] = (float)win_buf[i];  
        fft_im[i] = 0.0f;  
    }  
    memmove(win_buf, win_buf + RSID_FFT_N / 2,  
            (RSID_FFT_N / 2) * sizeof(int16_t));  
    win_fill = RSID_FFT_N / 2;  
  
    rsid_fft();  
    rsid_search();  
}  
  
extern "C" void rsid_rx_service(void) { rsid_rx_service_sliced(); }  
  
// ---- Диспетчеризация на Core 0 ----  
// Уведомление в декодеры — слабый символ: rx_decoders подхватит,  
// когда будет реализован.  
extern void rx_decoders_on_rsid(int mode_code, float freq_hz)  
    __attribute__((weak));  
  
extern "C" void rsid_rx_dispatch(void) {  
    if (!res_pending) return;  
    rsid_result_t r = res;  
    res_pending = false;  
    Serial.printf("# RSID mode=%s code=%d freq=%.1f Hz\n",  
                  r.mode_name ? r.mode_name : "?", r.mode_code,  
                  (double)r.freq_hz);  
    if (rx_decoders_on_rsid)  
        rx_decoders_on_rsid(r.mode_code, r.freq_hz);  
}  
  
// ---- Диагностика ----  
extern "C" uint32_t rsid_rx_dropped(void)        { return rsid_sample_dropped + rsid_window_dropped; }  
extern "C" uint32_t rsid_rx_sample_dropped(void) { return rsid_sample_dropped; }  
extern "C" uint32_t rsid_rx_window_dropped(void) { return rsid_window_dropped; }  
extern "C" uint32_t rsid_rx_queue_fill(void) {  
    return (inq_head + RSID_INQ_DEPTH - inq_tail) % RSID_INQ_DEPTH;  
}  
extern "C" uint32_t rsid_rx_sm_state(void)          { return rsid_sm; }  
extern "C" uint32_t rsid_rx_service_max_us(void)    { return 0; }  
extern "C" uint32_t rsid_rx_service_calls(void)     { return 0; }  
  
extern "C" void rsid_rx_reset(void) {  
    inq_head = inq_tail = 0;  
    win_fill = 0;  
    res_pending = false;  
}  
  
extern "C" void rsid_rx_init(float fs_hz) {  
    (void)fs_hz;  
    rsid_tables_init();  
    for (int i = 0; i < (int)RSID_NMODES; i++) {  
        rsid_seqs[i].code = rsid_modes[i].code;  
        rsid_encode(rsid_modes[i].code, rsid_seqs[i].sym);  
    }  
    rsid_seqs_n = RSID_NMODES;  
}  
  
// ---- Обертки под каркас модема (rsid_init/rsid_poll из .ino) ----  
void rsid_init(void) {  
    rsid_rx_init(RSID_FS_HZ);  
    Serial.println("[RSID] init");  
}  
  
void rsid_poll(void) {  
    rsid_rx_dispatch();          // Core 0: печать метки + смена моды  
}