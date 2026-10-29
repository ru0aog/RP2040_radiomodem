// src/modes/ax25/ax25_mode.cpp — регистрация моды в общем интерфейсе.  
#include "ax25.h"  
  
static const ModeIface s_mode = {  
  .id          = MODE_AX25,  
  .name        = "ax25",  
  .rsid_code   = 0,  
  .rx_init     = ax25_rx_init,  
  .rx_push     = ax25_rx_push,  
  .tx_frame    = ax25_tx_frame,  
};  
const ModeIface *ax25_mode_iface(void) { return &s_mode; }