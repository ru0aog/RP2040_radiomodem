// src/rx/rx_decoders.cpp  
// ============================================================================  
// Маршрутизатор декодеров (Core 0): выкачивает отсчёты из  
// rx_dsp_get_output() и гонит их в активную моду через ModeIface.  
// RSID-автодетект переключает моду через modes_get_by_rsid().  
// ============================================================================  
#include <Arduino.h>  
#include "rx_dsp.h"  
#include "rx_decoders.h"  
#include "../modes/modes.h"  
  
static const ModeIface* active_mode = 0;  
static RxModeId         active_rx   = RX_MODE_NONE;  
static uint32_t         dec_blocks  = 0;  
static uint32_t         dec_drained = 0;  
  
#define DEC_MAX_BLOCKS_PER_POLL 2  
  
void rx_decoders_init(void) {  
    active_mode = 0;  
    active_rx   = RX_MODE_NONE;  
    Serial.println("[DEC] init");  
}  
  
// Ручной выбор моды.  
void rx_decoders_set_mode(RxModeId id) {  
    const ModeIface* m = 0;  
    if (id == RX_MODE_AX25) m = modes_get(MODE_AX25);  
    else if (id == RX_MODE_NONE) m = 0;  
    // при добавлении мод: дописать маппинг RxModeId -> ModeId  
  
    active_mode = m;  
    active_rx   = m ? id : RX_MODE_NONE;  
    if (m && m->rx_init) m->rx_init(0);   // center_hz — уточнить при порте мод  
    Serial.printf("[DEC] mode=%s\n", m ? m->name : "none");  
}  
  
RxModeId rx_decoders_mode(void) { return active_rx; }  
  
// Колбэк от RSID: автовыбор моды по коду.  
void rx_decoders_on_rsid(uint16_t code) {  
    const ModeIface* m = modes_get_by_rsid((uint8_t)code);  
    if (!m) {  
        Serial.printf("[DEC] rsid code=%u — моды нет в реестре\n",  
                      (unsigned)code);  
        return;  
    }  
    if (m != active_mode) {  
        active_mode = m;  
        active_rx   = (m->id == MODE_AX25) ? RX_MODE_AX25 : RX_MODE_NONE;  
        if (m->rx_init) m->rx_init(0);  
    }  
    Serial.printf("[DEC] rsid -> %s\n", m->name);  
}  
  
// Выкачка блоков -> активная мода. rx_push принимает моно-отсчёт:  
// из I/Q-пары берём I-канал (демод-моно в не-RAW режимах).  
void rx_decoders_poll(void) {  
    const int16_t* i_buf;  
    const int16_t* q_buf;  
    uint8_t n;  
    int done = 0;  
  
    while (done < DEC_MAX_BLOCKS_PER_POLL &&  
           rx_dsp_get_output(&i_buf, &q_buf, &n)) {  
        if (active_mode && active_mode->rx_push) {  
            for (int k = 0; k < n; k++)  
                active_mode->rx_push(i_buf[k]);  
        }  
        rx_dsp_release_output();  
        ++dec_blocks;  
        ++done;  
    }  
    // Без активной моды очередь всё равно дренируем — иначе переполнение.  
    while (rx_dsp_get_output(&i_buf, &q_buf, &n)) {  
        rx_dsp_release_output();  
        ++dec_drained;  
    }  
}