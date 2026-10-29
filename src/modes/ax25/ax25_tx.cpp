// src/modes/ax25/ax25_tx.cpp — передача: 300 bd AFSK, ±100 Гц, NRZI.  
// КАРКАС. Алгоритм — порт ax25_modem.cpp из RP2040_beacon.  
#include "ax25.h"  
#include "../../trx/trx_control.h"  
  
static bool s_busy = false;  
  
uint32_t ax25_tx_frame(const uint8_t *payload, uint16_t len) {  
  (void)payload; (void)len;  
  // TODO: HDLC-кадр → битовый поток → vfo_bank mark/space → trx_switch  
  return 0;   // заглушка: 0 = TX не реализован  
}
bool ax25_tx_busy(void) { return s_busy; }
