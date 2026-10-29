// src/modes/modes_registry.cpp  
// ============================================================================  
// Реестр мод: таблица указателей на ModeIface + доступ по id / rsid_code.  
// Моды экспортируют аксессоры: mode_ax25() (ax25.h), cw_mode_iface() (cw.h).  
// ============================================================================  
#include "modes.h"  
#include "ax25/ax25.h"      // const ModeIface *mode_ax25(void);  
#include "cw/cw.h"          // const ModeIface *cw_mode_iface(void);  
  
static const ModeIface* const modes_table[] = {  
    mode_ax25(),  
    cw_mode_iface(),  
    // При добавлении мод — аксессор сюда и include выше.  
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