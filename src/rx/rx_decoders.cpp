// src/rx/rx_decoders.cpp  
// ============================================================================  
// Маршрутизатор декодеров (Core 0): выкачивает базовую полосу I/Q из  
// rx_dsp_get_output() и гонит отсчёты в активную моду через ModeIface.  
// При RSID-детекте выбирает моду по rsid_code из реестра мод.  
// ============================================================================  
#include <Arduino.h>  
#include "rx_dsp.h"  
#include "../modes/modes.h"          // ModeIface, modes_registry  
#include "../rsid/rsid.h"  
  
static const ModeIface* active_mode = 0;  
static uint32_t dec_blocks = 0;  
static uint32_t dec_blocks_dropped = 0;  
  
// Ограничение на блоки за один poll — чтобы Core 0 успевал loop-задачи.  
#define DEC_MAX_BLOCKS_PER_POLL  2  
  
void rx_decoders_init(void) {  
    active_mode = 0;  
    Serial.println("[DEC] init");  
}  
  
// Выбор активной моды по имени/RSID-коду.  
bool rx_decoders_set_mode(const char* name) {  
    if (!name) return false;  
    for (int i = 0; i < modes_count(); i++) {  
        const ModeIface* m = modes_get(i);  
        if (m && m->name && strcmp(m->name, name) == 0) {  
            active_mode = m;  
            if (m->rx_init) m->rx_init();  
            Serial.printf("[DEC] mode=%s\n", m->name);  
            return true;  
        }  
    }  
    return false;  
}  
  
// RSID-детект → автовыбор моды по rsid_code.  
void rx_decoders_on_rsid(int mode_code, float freq_hz) {  
    for (int i = 0; i < modes_count(); i++) {  
        const ModeIface* m = modes_get(i);  
        if (m && m->rsid_code == mode_code) {  
            if (m != active_mode) {  
                active_mode = m;  
                if (m->rx_init) m->rx_init();  
            }  
            // Центр RSID → AFC: подстройка NCO под реальный сигнал.  
            // Знак/опорную частоту сверить с логикой приёмника.  
            Serial.printf("[DEC] rsid→%s f=%.1f\n", m->name, (double)freq_hz);  
            return;  
        }  
    }  
    Serial.printf("[DEC] rsid code=%d — моды нет в реестре\n", mode_code);  
}  
  
const ModeIface* rx_decoders_active(void) { return active_mode; }  
  
// Выкачка блоков базовой полосы → активная мода.  
void rx_decoders_poll(void) {  
    const int16_t* i_buf;  
    const int16_t* q_buf;  
    uint8_t n;  
    int done = 0;  
  
    while (done < DEC_MAX_BLOCKS_PER_POLL &&  
           rx_dsp_get_output(&i_buf, &q_buf, &n)) {  
        if (active_mode && active_mode->rx_push) {  
            for (int k = 0; k < n; k++)  
                active_mode->rx_push(i_buf[k], q_buf[k]);  
        }  
        rx_dsp_release_output();  
        ++dec_blocks;  
        ++done;  
    }  
    // Если нет активной моды — очередь надо всё равно освобождать,  
    // иначе она переполнится и rx_dsp начнёт ронять блоки.  
    while (rx_dsp_get_output(&i_buf, &q_buf, &n)) {  
        rx_dsp_release_output();  
        ++dec_blocks_dropped;  
    }  
}  
  
void rx_decoders_debug(void) {  
    Serial.printf("[DEC] blocks=%lu drained=%lu mode=%s\n",  
                  (unsigned long)dec_blocks,  
                  (unsigned long)dec_blocks_dropped,  
                  active_mode ? active_mode->name : "-");  
}