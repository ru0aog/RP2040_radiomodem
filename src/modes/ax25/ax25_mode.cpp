// src/modes/ax25/ax25_mode.cpp — регистрация моды в общем интерфейсе.  
#include "ax25.h"  
  
static const ModeIface s_ax25 = {  
  "ax25",  
  ax25_tx_frame,  
  ax25_rx_init,  
  ax25_rx_push,  
};  
  
const ModeIface *mode_ax25(void) { return &s_ax25; }
