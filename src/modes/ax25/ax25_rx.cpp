// src/modes/ax25/ax25_rx.cpp — приём: корреляторы → дискриминатор →  
// слайсер → DPLL → NRZI → destuff → FCS, вход моно 15625 sps.  
// КАРКАС. Алгоритм — порт rx_ax25.cpp из RP2040_receiver.  
#include "ax25.h"  
  
static ax25_frame_cb_t s_cb = nullptr;  
  
void ax25_rx_init(uint32_t center_hz) {  
  Serial.printf("[STUB] ax25_rx_init: center=%lu Hz — decoder not ported\n",  
                (unsigned long)center_hz);  
}  
void ax25_rx_push(int16_t sample) { (void)sample; }  
void ax25_rx_set_frame_cb(ax25_frame_cb_t cb) { s_cb = cb; }
