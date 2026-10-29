// src/modes/cw/cw_mode.cpp — регистрация моды в общем интерфейсе.  
#include "cw.h"  
  
static const ModeIface s_mode = {  
  .id          = MODE_CW,  
  .name        = "cw",  
  .rsid_code   = 0,              // TODO: RSID-код CW (7?) при интеграции RSID  
  .rx_init     = cw_rx_init,  
  .rx_push     = cw_rx_push,  
  .tx_frame    = cw_tx_frame,  
};  
const ModeIface *cw_mode_iface(void) { return &s_mode; }
