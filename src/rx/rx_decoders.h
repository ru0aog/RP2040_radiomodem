// src/rx/rx_decoders.h  
// ============================================================================  
// Маршрутизатор декодеров: активная мода получает сэмплы из rx_dsp,  
// RSID-автодетект переключает моду. Моды живут в src/modes/<name>/.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
  
typedef enum {  
  RX_MODE_NONE = 0,  
  RX_MODE_AX25,  
  // RX_MODE_RTTY, RX_MODE_FT8, ... — расширяется при порте мод  
} RxModeId;  
  
void     rx_decoders_init(void);  
void     rx_decoders_poll(void);              // вызов из loop() на Core 0  
void     rx_decoders_set_mode(RxModeId id);   // ручной выбор  
RxModeId rx_decoders_mode(void);  
void     rx_decoders_on_rsid(uint16_t code);  // колбэк от RSID → автодетект
