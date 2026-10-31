#include "olivia.h"  
  
static const ModeIface s_iface = {  
    .id        = MODE_OLIVIA,  
    .name      = "olivia",  
    .rsid_code = 38,          // проверить по RSID_LIST в rsid.cpp!  
    .rx_init   = olivia_rx_init,  
    .rx_push   = olivia_rx_push,  
    .tx_frame  = 0,  
};  
  
const ModeIface* olivia_mode_iface(void) { return &s_iface; }
