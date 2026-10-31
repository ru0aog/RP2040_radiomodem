// src/modes/modes_registry.cpp  
#include "modes.h"  
#include "ax25/ax25.h"      // ax25_mode_iface()  
#include "cw/cw.h"          // cw_mode_iface()  
#include "rtty/rtty.h"      // rtty_mode_iface()
#include "olivia/olivia.h"
  
static const ModeIface* const modes_table[] = {  
    ax25_mode_iface(),  
    cw_mode_iface(),  
    rtty_mode_iface(),  
    olivia_mode_iface(),  
};
  
#define MODES_N (sizeof(modes_table) / sizeof(modes_table[0]))  
  
const ModeIface* modes_get(ModeId id) {  
    for (unsigned i = 0; i < MODES_N; i++) {  
        if (modes_table[i] && modes_table[i]->id == id)  
            return modes_table[i];  
    }  
    return 0;  
}  
  
const ModeIface* modes_get_by_rsid(uint8_t rsid_code) {  
    if (rsid_code == 0) return 0;  
    for (unsigned i = 0; i < MODES_N; i++) {  
        if (modes_table[i] && modes_table[i]->rsid_code == rsid_code)  
            return modes_table[i];  
    }  
    return 0;  
}