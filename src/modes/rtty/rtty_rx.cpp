// src/modes/rtty/rtty_rx.cpp — приём RTTY-45 (45.45 бод, Baudot/ITA2).  
// Порт RP2040_receiver/src/modes/rx_rtty.cpp (бодовый автомат verbatim)  
// + фронтенд уровней mark/space (аналог rx_mf.cpp): квадратурные  
// корреляторы на mark/space → IIR-фильтр мощности.  
// Вход: моно-AF 15625 sps от rx_decoders (тот же отвод, что AX25/CW).  
#include "rtty.h"  
#include "../../rx/rx_decoders.h"  
#include <Arduino.h>  
#include <math.h>  
  
// ---------------- параметры RTTY ----------------  
#define RTTY_BAUD        45.45f  
#define RTTY_SHIFT_HZ    170.0f  
#define RTTY_NBITS       5          // Baudot/ITA2  
#define MF_SQUELCH_DB    6.0f       // порог присутствия: сигнал ≥6 дБ над шумом  
#define RTTY_FS          15625.0f  
  
// ---------------- Baudot/ITA2 (из FLDigi rtty.cxx) ----------------  
static const char letters[32] = {  
    '\0','E','\n','A',' ','S','I','U',  
    '\r','D','R','J','N','F','C','K',  
    'T','Z','L','W','H','Y','P','Q',  
    'O','B','G',' ','M','X','V',' '  
};  
static const char figures[32] = {  
    '\0','3','\n','-',' ','\a','8','7',  
    '\r','$','4','\'',',','!',':','(',  
    '5','"',')','2','#','6','0','1',  
    '9','?','&',' ','.','/',';',' '  
};  
  
#define RTTY_LTRS  0x1F   // 11111 — сдвиг в регистр букв  
#define RTTY_FIGS  0x1B   // 11011 — сдвиг в регистр цифр  
  
// ---------------- тональные корреляторы (фронтенд) ----------------  
// mark = center + shift/2, space = center - shift/2.  
// Конвенция fldigi для HF-RTTY: mark — верхний тон.  
// Если декодируется «зеркально» (LTRS/FIGS сбиты, крякозябры) —  
// поменять местами g_mark_hz/g_space_hz в init.  
struct rtty_tone {  
    float    c, s;        // cos/sin шаг NCO  
    float    ci, si;      // фаза NCO  
    float    i, q;        // IIR-накопители  
    float    lpf;         // коэффициент фильтра  
    float    pwr;         // мощность тона  
};  
  
static rtty_tone g_mark, g_space;  
static float     g_mark_hz, g_space_hz;  
static bool      g_active = false;  
  
static inline void tone_init(rtty_tone* t, float freq_hz, float lpf) {  
    const float w = 2.0f * (float)M_PI * freq_hz / RTTY_FS;  
    t->c = cosf(w); t->s = sinf(w);  
    t->ci = 1.0f; t->si = 0.0f;  
    t->i = t->q = 0.0f;  
    t->lpf = lpf;  
    t->pwr = 0.0f;  
}  
  
// Один отсчёт через коррелятор: вращение NCO, перемножение,  
// IIR-фильтр I/Q, мощность. Полоса фильтра ~1.2×baud.  
static inline float tone_step(rtty_tone* t, int16_t smp) {  
    const float nci = t->ci * t->c - t->si * t->s;  
    const float nsi = t->si * t->c + t->ci * t->s;  
    t->ci = nci; t->si = nsi;  
  
    const float x = (float)smp;  
    t->i += t->lpf * (x * nci - t->i);  
    t->q += t->lpf * (x * nsi - t->q);  
    t->pwr = t->i * t->i + t->q * t->q;  
    return t->pwr;  
}  
  
// ---------------- состояние автомата ----------------  
enum { RX_IDLE = 0, RX_START = 1, RX_DATA = 2, RX_STOP = 3 };  
  
static float    fs_hz     = RTTY_FS;  
static float    symbollen = RTTY_FS / RTTY_BAUD;   // ~344 отсчёта/бит  
  
static int      rxstate   = RX_IDLE;  
static float    counter   = 0.0f;  
static uint8_t  bitcntr   = 0;  
static uint8_t  rxdata    = 0;  
static bool     figs_mode = false;                 // false=LTRS, true=FIGS  
  
// ---------------- SNR-сквелч ----------------  
static float sig_pwr   = 0.0f;  
static float noise_pwr = 1e-6f;  
static bool  present   = false;  
  
static inline float decayavg(float avg, float x, float w) {  
    if (w < 1.0f) return x;  
    return avg + (x - avg) / w;  
}  
  
// ---------------- вывод декодированного символа ----------------  
static void rtty_put(char c) { Serial.write(c); }  
  
static void rtty_emit(uint8_t c) {  
    c &= 0x1F;  
    if (c == RTTY_LTRS) { figs_mode = false; return; }  
    if (c == RTTY_FIGS) { figs_mode = true;  return; }  
    char out = figs_mode ? figures[c] : letters[c];  
    if (out) rtty_put(out);  
}  
  
// ---------------- инициализация ----------------  
void rtty_rx_init(uint32_t center_hz) {  
    if (center_hz == 0) center_hz = 1500;   // дефолт при ручном выборе  
  
    g_mark_hz  = (float)center_hz + RTTY_SHIFT_HZ * 0.5f;  
    g_space_hz = (float)center_hz - RTTY_SHIFT_HZ * 0.5f;  
  
    // Полоса фильтра: ~1.2 бода (как matched-фильтр в rx_mf.cpp)  
    const float bw  = RTTY_BAUD * 1.2f;  
    const float lpf = 1.0f - expf(-2.0f * (float)M_PI * bw / RTTY_FS);  
  
    tone_init(&g_mark,  g_mark_hz,  lpf);  
    tone_init(&g_space, g_space_hz, lpf);  
  
    fs_hz     = RTTY_FS;  
    symbollen = fs_hz / RTTY_BAUD;  
  
    rxstate   = RX_IDLE;  
    counter   = 0.0f;  
    bitcntr   = 0;  
    rxdata    = 0;  
    figs_mode = false;  
  
    sig_pwr   = 0.0f;  
    noise_pwr = 1e-6f;  
    present   = false;  
  
    g_active  = true;  
    Serial.printf("[RTTY] init center=%lu mark=%lu space=%lu baud=45.45\n",  
                  (unsigned long)center_hz,  
                  (unsigned long)g_mark_hz,  
                  (unsigned long)g_space_hz);  
}  
  
void rtty_rx_stop(void) { g_active = false; }  
  
// ---------------- пер-сэмпловая точка входа (15625 sps) ----------------  
void rtty_rx_push(int16_t smp) {  
    if (!g_active) return;  
  
    // уровни mark/space этого отсчёта  
    const float pm = tone_step(&g_mark,  smp);  
    const float ps = tone_step(&g_space, smp);  
  
    // ---- ниже verbatim из rx_rtty_frame_levels() приёмника ----  
  
    // SNR-сквелч: сильный тон = сигнал, слабый канал = шум  
    const float sp = (pm > ps) ? pm : ps;  
    const float np = (pm < ps) ? pm : ps;  
    sig_pwr   = decayavg(sig_pwr,   sp, sp > sig_pwr ? 2.0f : 8.0f);  
    noise_pwr = decayavg(noise_pwr, np, 16.0f);  
    const float snr_db = 10.0f * log10f(sig_pwr / (noise_pwr + 1e-6f));  
    present = (snr_db >= MF_SQUELCH_DB);  
  
    const bool bit = (pm > ps);   // true = mark(1), false = space(0)  
  
    // present гейтит ТОЛЬКО старт из IDLE. Уже начатый символ дотактовываем  
    // до стоп-бита, иначе кратковременные провалы SNR на переходах mark/space  
    // рвут каждый символ (present моргает 1->0->1 на каждом фронте).  
    switch (rxstate) {  
    case RX_IDLE:  
        // старт-бит = space (bit==0); входим только при наличии сигнала  
        if (present && !bit) {  
            rxstate = RX_START;  
            counter = symbollen * 0.5f;   // строб в середину старт-бита  
        }  
        break;  
  
    case RX_START:  
        if ((counter -= 1.0f) <= 0.0f) {  
            if (!bit) {                    // старт-бит подтверждён  
                rxstate = RX_DATA;  
                counter = symbollen;  
                bitcntr = 0;  
                rxdata  = 0;  
            } else {  
                rxstate = RX_IDLE;         // ложный старт  
            }  
        }  
        break;  
  
    case RX_DATA:  
        if ((counter -= 1.0f) <= 0.0f) {  
            rxdata |= (bit ? 1 : 0) << bitcntr;  // LSB первым  
            counter = symbollen;  
            if (++bitcntr >= RTTY_NBITS) rxstate = RX_STOP;  
        }  
        break;  
  
    case RX_STOP:  
        if ((counter -= 1.0f) <= 0.0f) {  
            if (bit) {                     // стоп-бит = mark → кадр валиден  
                rtty_emit(rxdata & 0x1F);  
            }  
            rxstate = RX_IDLE;  
        }  
        break;  
    }  
}
