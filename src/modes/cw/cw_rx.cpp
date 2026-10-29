// src/modes/cw/cw_rx.cpp — КАРКАС.  
// ============================================================================  
// CW RX: узкополосный тональный детектор (Goertzel или квадратурный  
// перенос на CW_TONE_OFFSET_HZ) → огибающая → FSM: тон/пауза с таймингами  
// → классификация точка/тире → таблица Morse → символы.  
// Порт из RP2040_receiver/src/modes/rx_cw.cpp.  
// ============================================================================  
#include "cw.h"  
  
static uint32_t s_center_hz = 0;  
static uint8_t  s_wpm = CW_DEFAULT_WPM;  
  
void cw_rx_init(uint32_t center_hz) {  
  s_center_hz = center_hz;  
  // TODO: настроить Goertzel-окно на CW_TONE_OFFSET_HZ при fs=15625,  
  //       сбросить FSM декодера.  
  Serial.printf("[STUB] cw_rx_init center=%lu Hz tone=%u\n",  
                (unsigned long)center_hz, (unsigned)CW_TONE_OFFSET_HZ);  
}  
  
void cw_rx_push(int16_t sample) {  
  (void)sample;  
  // TODO: Goertzel-накопление |ton| → порог с гистерезисом →  
  //       edges → длительности тона/паузы → морзянка → Serial/emit.  
}  
  
void cw_rx_set_wpm(uint8_t wpm) {  
  if (wpm >= 5 && wpm <= 60) s_wpm = wpm;  
}
