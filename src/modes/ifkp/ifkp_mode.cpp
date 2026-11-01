#include "ifkp.h"  
  
static const ModeIface s_iface = {  
    .id        = MODE_IFKP,  
    .name      = "ifkp",  
    .rsid_code = 1171,      // ELEM2_(1171, "IFKP") из RSID_LIST2  
    .rx_init   = ifkp_rx_init,  
    .rx_push   = ifkp_rx_push,  
    .tx_frame  = 0,  
};  
  
const ModeIface* ifkp_mode_iface(void) { return &s_iface; }