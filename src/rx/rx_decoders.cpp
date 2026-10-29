// src/rx/rx_decoders.cpp — маршрутизатор декодеров. КАРКАС.  
#include "rx_decoders.h"  
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
  // (include "rx_dsp.h" в .cpp)  
}
