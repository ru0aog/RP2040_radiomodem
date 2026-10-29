// src/modes/modes_registry.cpp  
// ============================================================================  
// Реестр мод: таблица указателей на ModeIface + доступ по id / rsid_code.  
// Каждая мода экспортирует аксессор <mode>_mode_iface() (ModeIface у неё  
// static — прямого символа нет, см. ax25_mode.cpp).  
// ============================================================================  
#include "modes.h"  
#include "ax25/ax25.h"      // const ModeIface *ax25_mode_iface(void);  
#include "cw/cw.h"          // const ModeIface *cw_mode_iface(void);  
  
static const ModeIface* const modes_table[] = {  
    ax25_mode_iface(),  
    cw_mode_iface(),  
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
    if (rsid_code == 0) return 0;          // 0 = нет авто-детекта  
    for (unsigned i = 0; i < MODES_N; i++) {  
        if (modes_table[i] && modes_table[i]->rsid_code == rsid_code)  
            return modes_table[i];  
    }  
    return 0;  
}