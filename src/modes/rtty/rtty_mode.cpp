#include "rtty.h"  
  
static const ModeIface s_iface = {  
    .id        = MODE_RTTY,  
    .name      = "rtty",  
    .rsid_code = 39,   // fldigi RSID: RTTY-45 (45.45 bd, shift 170 Hz)
    .rx_init   = rtty_rx_init,  
    .rx_push   = rtty_rx_push,  
    .tx_frame  = 0,                    // или rtty_tx_frame, если есть заглушка  
};  
  
const ModeIface* rtty_mode_iface(void) { return &s_iface; }