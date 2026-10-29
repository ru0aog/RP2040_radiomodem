// src/vfo/soft_tx.h  
#pragma once  
#include <Arduino.h>  
// Порт дизеринг-движка из RP2040_beacon/vfo_hardware.cpp:  
// PIO-меандр (2-тактовая программа) + запись делителя clkdiv  
// из кооперативного poll-цикла на Core 1 (MASH-1-1 + FRAC-дизер).  
//  
// ОТЛИЧИЕ от исходника: движок НЕ захватывает ядро — горячий цикл  
// разбит на итерации soft_tx_poll(), вызываемые из loop1() в фазе TX.  
  
struct SoftTxParams {          // = VfoParameters исходника  
  uint32_t pio_int;  
  uint32_t pio_frac;  
  uint32_t dds_step;  
  uint32_t target_freq_chz;  
};  
  
void     soft_tx_init();  
void     soft_tx_start(uint32_t freq_hz);  
void     soft_tx_stop();  
void     soft_tx_tone(uint8_t index);  
void     soft_tx_poll();               // дергать каждую итерацию loop1() в TX  
uint32_t soft_tx_actual_hz();  
  
void     soft_tx_boost_enter(uint32_t target_hz);  
void     soft_tx_boost_exit();  
uint32_t soft_tx_clk_sys_hz();  
float    soft_tx_core_temp_c();  
  
void     soft_tx_set_narrowband(bool on);  // Olivia: чистые тоны без FRAC-дизера
