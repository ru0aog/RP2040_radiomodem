// src/rx/rx_capture.h  
// ============================================================================  
// Захват ADC+DMA: GP26=I (ADC0), GP27=Q (ADC1), интерлив [31:16]=Q, [15:0]=I.  
// Цель ~250k слов/с (125 ksps на канал). Порт из RP2040_receiver.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
#include "rx_dsp.h"  
  
// Старт захвата. Каждый полный блок уходит в rx_dsp_push_raw().  
// Возврат false — не удалось занять DMA-канал/ADC.  
bool     rx_capture_start(void);  
void     rx_capture_stop(void);  
uint32_t rx_capture_rate_hz(void);      // фактическая скорость слов/с  
uint32_t rx_capture_overruns(void);     // счётчик переполнений DMA  
void     rx_capture_debug(void);
