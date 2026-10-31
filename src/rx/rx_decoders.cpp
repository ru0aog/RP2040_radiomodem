// src/rx/rx_decoders.cpp  
// ============================================================================  
// Маршрутизатор декодеров (Core 0): выкачивает отсчёты из  
// rx_dsp_get_output() и гонит их в активную моду через ModeIface.  
// RSID-автодетект переключает моду через modes_get_by_rsid().  
// ============================================================================  
#include <Arduino.h>  
#include "rx_dsp.h"  
#include "rx_decoders.h"
#include "../rsid/rsid.h"          // rsid_last_freq_hz 
#include "../modes/modes.h"  
#include "../modes/ax25/ax25.h"     // ax25_rx_stop  
#include "../modes/cw/cw.h"         // cw_rx_stop  
#include "../modes/rtty/rtty.h"     // rtty_rx_stop  
#include "../modes/olivia/olivia.h" // olivia_rx_stop
#include "../modes/contestia/contestia.h" // contestia_rx_stop
  
static const ModeIface* active_mode = 0;  
static RxModeId         active_rx   = RX_MODE_NONE;  
static uint32_t         dec_blocks  = 0;  
static uint32_t         dec_drained = 0;  
  
#define DEC_MAX_BLOCKS_PER_POLL 2  
  
// ---------------------------------------------------------------------------  
// Маппинг ModeId -> RxModeId (для rx_decoders_mode()).  
// RxModeId — перечисление из rx_decoders.h; если RX_MODE_RTTY/RX_MODE_OLIVIA  
// в нём ещё нет, добавить туда по образцу RX_MODE_AX25.  
// ---------------------------------------------------------------------------  
static RxModeId rx_id_for(ModeId id) {  
    switch (id) {  
        case MODE_AX25:   return RX_MODE_AX25;  
        case MODE_CW:     return RX_MODE_CW;  
        case MODE_RTTY:   return RX_MODE_RTTY;  
        case MODE_OLIVIA:   return RX_MODE_OLIVIA;  
        case MODE_CONTESTIA: return RX_MODE_CONTESTIA;  
        default:            return RX_MODE_NONE; 
    }  
}  
  
// ---------------------------------------------------------------------------  
// Останов активного декодера: сброс внутреннего состояния моды  
// (стартовые биты, битовый буфер, DPLL и т.д.).  
// ---------------------------------------------------------------------------  
static void stop_active_decoder(void) {  
    if (!active_mode) return;  
    switch (active_mode->id) {  
        case MODE_AX25:   ax25_rx_stop();   break;  
        case MODE_CW:     cw_rx_stop();     break;  
        case MODE_RTTY:   rtty_rx_stop();   break;  
        case MODE_OLIVIA: olivia_rx_stop(); break;
        case MODE_CONTESTIA: contestia_rx_stop(); break;
        default: break;  
    }  
}  
  
// ---------------------------------------------------------------------------  
void rx_decoders_init(void) {  
    active_mode = 0;  
    active_rx   = RX_MODE_NONE;  
    dec_blocks  = 0;  
    dec_drained = 0;  
    Serial.println("[DEC] init");  
}  
  
// ---------------------------------------------------------------------------  
// Ручной выбор моды (команды/CLI). Центр неизвестен — передаём 0,  
// декодер подставляет дефолт (например, 1500 Гц).  
// ---------------------------------------------------------------------------  
void rx_decoders_set_mode(RxModeId id) {  
    ModeId mid;  
    switch (id) {  
        case RX_MODE_AX25:   mid = MODE_AX25;   break;  
        case RX_MODE_CW:     mid = MODE_CW;     break;  
        case RX_MODE_RTTY:   mid = MODE_RTTY;   break;  
        case RX_MODE_OLIVIA: mid = MODE_OLIVIA; break;
        case RX_MODE_CONTESTIA: mid = MODE_CONTESTIA; break;
        default:             mid = MODE_NONE;   break;  
    }  
    const ModeIface* m = modes_get(mid);  
    if (active_mode && m != active_mode)  
        stop_active_decoder();  
    active_mode = m;  
    active_rx   = (m ? id : RX_MODE_NONE);  
    if (m && m->rx_init) m->rx_init(0);  
    Serial.printf("[DEC] mode -> %s\n", m ? m->name : "none");  
}  
  
RxModeId rx_decoders_mode(void) { return active_rx; }  
  
// ---------------------------------------------------------------------------  
// Колбэк от RSID: автовыбор моды по коду. Центр берём из детекта —  
// декодер настраивает корреляторы/тон на фактическую частоту посылки.  
// ---------------------------------------------------------------------------  
void rx_decoders_on_rsid(uint16_t code) {  
    const ModeIface* m = modes_get_by_rsid((uint8_t)code);  
    if (!m) {  
        Serial.printf("[DEC] rsid code=%u — моды нет в реестре\n",  
                      (unsigned)code);  
        return;  
    }  
  
    uint32_t center = (uint32_t)(rsid_last_freq_hz() + 0.5f);  
  
    if (m != active_mode) {  
        stop_active_decoder();          // сброс предыдущей моды  
        active_mode = m;  
        active_rx   = rx_id_for(m->id);  
        if (m->rx_init) m->rx_init(center);  
    } else if (m->rx_init) {  
        m->rx_init(center);             // та же мода — перецентровка тона  
    }  
    Serial.printf("[DEC] rsid -> %s center=%lu\n",  
                  m->name, (unsigned long)center);  
}  
  
// ---------------------------------------------------------------------------  
// Выкачиваем блоки из rx_dsp и скармливаем активной моде:  
// в i_buf лежит моно-AF 15625 sps (AF-отвод в rx_dsp).  
// ---------------------------------------------------------------------------  
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
    // Перелимит и простаивающая очередь — дренируем всегда,  
    // иначе out_drop.  
    while (rx_dsp_get_output(&i_buf, &q_buf, &n)) {  
        rx_dsp_release_output();  
        ++dec_drained;  
    }  
}  
  
// ---------------------------------------------------------------------------  
// Общий канал вывода декодированного текста (Core 0 → USB-CDC).  
// ---------------------------------------------------------------------------  
void rx_decoders_emit(uint8_t c) { Serial.write(c); }  
  
uint32_t rx_decoders_blocks(void)  { return dec_blocks;  }  
uint32_t rx_decoders_drained(void) { return dec_drained; }