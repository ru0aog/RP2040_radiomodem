#include "contestia.h"  
  
static const ModeIface s_iface = {  
    .id        = MODE_CONTESTIA,  
    .name      = "contestia",  
    .rsid_code = 49,   // fldigi RSID: CONTESTIA-8/250
    .rx_init   = contestia_rx_init,  
    .rx_push   = contestia_rx_push,  
    .tx_frame  = 0,  
};  
  
const ModeIface* contestia_mode_iface(void) { return &s_iface; }
