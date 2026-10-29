// src/modes/ax25/ax25_tx.cpp — передача: 300 bd AFSK, ±100 Гц, NRZI.  
// КАРКАС. Алгоритм — порт ax25_modem.cpp из RP2040_beacon.  
#include "ax25.h"  
#include "../../trx/trx_control.h"  
  
static bool s_busy = false;  
  
bool ax25_tx_frame(const uint8_t *payload, size_t len) {  
  Serial.printf("[STUB] ax25_tx_frame: %u bytes — TX chain not ported\n",  
                (unsigned)len);  
  (void)trx_switch;  // реальная версия: trx_switch(TRX_TX, center) → биты → RX  
  return false;  
}  
bool ax25_tx_busy(void) { return s_busy; }
