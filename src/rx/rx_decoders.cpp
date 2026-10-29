// src/rx/rx_decoders.cpp  
// ============================================================================  
// Маршрутизатор декодеров: активная мода получает блоки I/Q от rx_dsp,  
// RSID-код может переключать активную моду. Работает на Core 0 (loop()).  
// ============================================================================  
#include "rx_decoders.h"  
#include "rx_dsp.h"  
#include "../modes/modes.h"  
  
// ---------- Состояние ----------  
static const ModeIface *s_active   = nullptr;   // активная мода  
static ModeId           s_active_id = MODE_NONE;  
  
// ---------- API ----------  
  
// Активировать моду; вызывает её rx_init с центральной частотой.  
void rx_decoders_set_mode(ModeId id, uint32_t center_hz) {  
  s_active    = modes_get(id);  
  s_active_id = s_active ? id : MODE_NONE;  
  if (s_active && s_active->rx_init) s_active->rx_init(center_hz);  
  Serial.printf("[DEC] mode=%s center=%lu Hz\n",  
                s_active ? s_active->name : "none",  
                (unsigned long)center_hz);  
}  
  
ModeId rx_decoders_active(void) { return s_active_id; }  
  
// Колбэк от RSID: код → мода → rx_decoders_set_mode.  
void rx_decoders_on_rsid(uint8_t rsid_code, uint32_t center_hz) {  
  const ModeIface *m = modes_get_by_rsid(rsid_code);  
  if (!m) return;  
  Serial.printf("[DEC] RSID=%u -> mode=%s\n", (unsigned)rsid_code, m->name);  
  rx_decoders_set_mode(m->id, center_hz);  
}  
  
// Вычитка очереди аудио/IQ и передача сэмплов активной моде.  
// Вызывать из loop() — частота вызовов определяет задержку декода.  
void rx_decoders_poll(void) {  
  const int16_t *i_ch, *q_ch;  
  uint8_t n;  
  
  while (rx_dsp_get_audio(&i_ch, &q_ch, &n)) {  
    for (uint8_t k = 0; k < n; k++) {  
      // TODO: моно-микс по выбранному демодулятору; пока — I-канал.  
      int16_t m = i_ch[k];  
      if (s_active && s_active->rx_push) s_active->rx_push(m);  
    }  
    rx_dsp_release_audio();        // вернуть блок в очередь  
  }  
}


void rx_decoders_init(void) {  
  s_active    = NULL;  
  s_active_id = MODE_NONE;  
  Serial.println("[STUB] rx_decoders init");  
}