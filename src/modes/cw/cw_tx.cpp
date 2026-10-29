// src/modes/cw/cw_tx.cpp — КАРКАС.  
// ============================================================================  
// CW TX: одна частота — несущая на freq_hz. Ключ = start/stop VFO.  
// Реализация: парсер текста → таблица морзянки → очередь элементов  
// (тон/пауза/длительность) → по-тиковый ключ.  
// ============================================================================  
#include "cw.h"  
#include "../../vfo/vfo.h"  
#include "../../trx/trx_control.h"  
  
static uint8_t s_wpm = CW_DEFAULT_WPM;  
static uint32_t s_freq_hz = 0;  
  
// Длительность точки: 1200 мс / WPM (стандарт PARIS).  
static uint32_t dit_ms(void) { return 1200u / s_wpm; }  
  
void cw_tx_set_wpm(uint8_t wpm) {  
  if (wpm >= 5 && wpm <= 60) s_wpm = wpm;  
}  
  
void cw_tx_key(bool down) {  
  // TODO: down → vfo start на s_freq_hz (нужен trx_switch(TRX_TX,...)),  
  //       up   → vfo stop. Или тихий тон через тоновый банк.  
  (void)down;  
  Serial.printf("[STUB] cw_tx_key %s\n", down ? "DOWN" : "UP");  
}  
  
uint32_t cw_tx_frame(const uint8_t *payload, uint16_t len) {  
  (void)payload; (void)len;  
  // TODO: текст → элементы .- через таблицу Morse → trx_switch(TRX_TX,  
  //       s_freq_hz) → очередь тон/пауза по dit_ms() → trx_switch(TRX_RX).  
  Serial.printf("[STUB] cw_tx_frame len=%u wpm=%u dit=%lu ms\n",  
                (unsigned)len, (unsigned)s_wpm,  
                (unsigned long)dit_ms());  
  return 0;  
}
