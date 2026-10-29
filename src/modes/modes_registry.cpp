// src/modes/modes_registry.cpp  
// ============================================================================  
// Реестр мод: таблица ModeIface + доступ по id / rsid_code.  
// Каждая мода экспортирует const ModeIface из своей папки  
// (см. ax25_mode.cpp, cw/cw_mode.cpp).  
// ============================================================================  
#include "modes.h"  
  
// Экспорты мод — имена сверить с определениями в ax25_mode.cpp / cw/.  
extern const ModeIface ax25_mode;    // src/modes/ax25/ax25_mode.cpp  
extern const ModeIface cw_mode;      // src/modes/cw/ (если есть)  
  
static const ModeIface* const modes_table[] = {  
    &ax25_mode,  
    &cw_mode,  
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
