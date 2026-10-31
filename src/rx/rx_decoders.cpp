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
#include "../modes/ax25/ax25.h"     // ax25_rx_stop  
#include "../rsid/rsid.h"          // rsid_last_freq_hz  
  
static const ModeIface* active_mode = 0;  
static RxModeId         active_rx   = RX_MODE_NONE;  
static uint32_t         dec_blocks  = 0;  
static uint32_t         dec_drained = 0;  
  
#define DEC_MAX_BLOCKS_PER_POLL 2  
  
void rx_decoders_init(void) {  
    active_mode = 0;  
    active_rx   = RX_MODE_NONE;  
    dec_blocks  = 0;  
    dec_drained = 0;  
    Serial.println("[DEC] init");  
}  
  
// Ручной выбор моды (без RSID): центр неизвестен — моды  
// подставляют дефолт (ax25: 1500 Гц при center_hz==0).  
void rx_decoders_set_mode(RxModeId id) {  
    const ModeIface* m = modes_get_by_id(id);  
    if (!m) {  
        Serial.printf("[DEC] mode id=%u не найдена\n", (unsigned)id);  
        return;  
    }  
    if (active_mode && active_mode->id == MODE_AX25) ax25_rx_stop();  
    active_mode = m;  
    active_rx   = id;  
    if (m->rx_init) m->rx_init(0);  
    Serial.printf("[DEC] mode -> %s\n", m->name);  
}  
  
// Колбэк от RSID: автовыбор моды по коду. Центр берём из  
// частоты RSID-посылки — она совпадает с центром сигнала.  
void rx_decoders_on_rsid(uint16_t code) {  
    const ModeIface* m = modes_get_by_rsid((uint8_t)code);  
    if (!m) {  
        Serial.printf("[DEC] rsid code=%u — моды нет в реестре\n",  
                      (unsigned)code);  
        return;  
    }  
  
    const uint32_t center = (uint32_t)(rsid_last_freq_hz() + 0.5f);  
  
    if (m != active_mode) {  
        // Смена моды: остановить декодер предыдущей.  
        if (active_mode && active_mode->id == MODE_AX25) ax25_rx_stop();  
        active_mode = m;  
        active_rx   = (m->id == MODE_AX25) ? RX_MODE_AX25 : RX_MODE_NONE;  
        if (m->rx_init) m->rx_init(center);  
        Serial.printf("[DEC] rsid -> %s center=%lu\n",  
                      m->name, (unsigned long)center);  
    } else {  
        // Та же мода на новом центре — переинициализировать корреляторы.  
        if (m->rx_init) m->rx_init(center);  
        Serial.printf("[DEC] rsid -> %s reinit center=%lu\n",  
                      m->name, (unsigned long)center);  
    }  
}  
  
RxModeId rx_decoders_mode(void) { return active_rx; }  
  
uint32_t rx_decoders_blocks(void)  { return dec_blocks; }  
uint32_t rx_decoders_drained(void) { return dec_drained; }  
  
// Выкачиваем блоки из rx_dsp и скармливаем активной моде:  
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
    // Перелимит и простаивающая очередь — дренируем всегда,  
    // иначе out_drop.  
    while (rx_dsp_get_output(&i_buf, &q_buf, &n)) {  
        rx_dsp_release_output();  
        ++dec_drained;  
    }  
}
