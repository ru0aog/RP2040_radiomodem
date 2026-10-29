// src/rx/rx_capture.h  
// ============================================================================  
// Захват ADC+DMA: ADC0=I (GP26), ADC1=Q (GP27), round-robin, 16-бит  
// переносы в 32-бит слова [15:0]=I, [31:16]=Q. Пинг-понг два канала DMA  
// через chain_to; IRQ0 на Core 0 публикует блоки в SPSC-очередь для Core 1.  
// Порт из RP2040_receiver/src/dsp/rx_capture.cpp.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
#include "rx_dsp.h"  
  
void     rx_capture_start(void);          // запуск конвейера (после rx_dsp_start)  
void     rx_capture_stop(void);           // останов (после rx_dsp_stop!)  
  
// API очереди — вызывается ТОЛЬКО из rx_dsp_poll() на Core 1.  
const volatile uint32_t* rx_capture_get_block(void);  
void     rx_capture_release(void);  
  
uint32_t rx_capture_dropped(void);        // суммарный счётчик потерь

extern volatile uint32_t rx_cap_isr_count;   // счётчик входов DMA IRQ, диагностика
extern volatile uint32_t rx_drop_scratch, rx_drop_queue_full, rx_drop_busy;