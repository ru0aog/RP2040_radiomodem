// src/modes/jtty/jtty_mode.cpp — регистрация моды JTTY в реестре.  
#include "jtty.h"  
  
static const ModeIface s_iface = {  
    .id        = MODE_JTTY,  
    .name      = "jtty",  
    .rsid_code = 1172,      // ELEM2_(1172, "JTTY") из RSID_LIST2  
    .rx_init   = jtty_rx_init,  
    .rx_push   = jtty_rx_push,  
    .tx_frame  = 0,  
};  
  
const ModeIface* jtty_mode_iface(void) { return &s_iface; }