// src/rx/rx_decoders.cpp — маршрутизатор декодеров. КАРКАС.  
#include "rx_decoders.h"  
#include "rx_dsp.h"  
#include "../modes/ax25/ax25.h"  
  
static RxModeId s_mode = RX_MODE_NONE;  
  
void rx_decoders_init(void) {  
  ax25_rx_init(0);  
  s_mode = RX_MODE_AX25;   // пока единственная мода — выбираем сразу 
  Serial.println("[STUB] rx_decoders_init: only AX.25 registered");  
}  
  
void rx_decoders_poll(void) {
  // Скелет: вычитываем аудио из rx_dsp и кормим активную моду. 
  int16_t buf[RX_DSP_BLOCK_SAMPLES];  
  size_t n = rx_dsp_get_audio(buf, RX_DSP_BLOCK_SAMPLES);  
  if (n == 0) return;  
  for (size_t i = 0; i < n; i++) {  
    switch (s_mode) {  
      case RX_MODE_AX25: ax25_rx_push(buf[i]); break;  
      default: break;  
    }  
  }  
}  
  
void rx_decoders_set_mode(RxModeId id) { s_mode = id; }  
RxModeId rx_decoders_mode(void)        { return s_mode; }  
void rx_decoders_on_rsid(uint16_t code) {  
  Serial.printf("[STUB] rsid code %u — mode mapping not implemented\n", code);  
}
