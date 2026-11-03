// src/filesys/fmgr_compat.h — маяковые типы/прототипы до их портирования.  
// УДАЛИТЬ после подключения scheduler.h и vfo_hardware.h.  
#ifndef FMGR_COMPAT_H  
#define FMGR_COMPAT_H  
  
#include <Arduino.h>  
  
// --- из scheduler.h (RP2040_beacon) ---  
#define MAX_SCHEDULE_TASKS 32  
enum BeaconMode : uint8_t {  
  MODE_CW, MODE_RTTY, MODE_IFKP, MODE_OLIVIA, MODE_CONTESTIA,  
  MODE_THOR, MODE_MFSK16, MODE_DOMINOEX, MODE_FSQ, MODE_JTTY,  
  MODE_AX25, MODE_SEQ  
};  
struct TaskItem {  
  bool     active = false;  
  uint8_t  days = 0;           // биты 1..7 (0 = каждый день)  
  uint8_t  start_hour = 0, start_min = 0;  
  uint8_t  end_hour = 0, end_min = 0;  
  uint16_t interval_min = 0;  
  uint32_t freq_hz = 0;  
  BeaconMode mode = MODE_CW;  
};  
extern TaskItem beacon_schedule[MAX_SCHEDULE_TASKS];  
int     get_next_start_minute(BeaconMode mode);   // weak-заглушка в .cpp  
String  fmt_next_start(int minute);               // weak-заглушка в .cpp  
  
// --- из RTC-модуля (RP2040_beacon) ---  
extern uint16_t rtc_year;  
extern uint8_t  rtc_month, rtc_day, rtc_hour, rtc_min, rtc_sec;  
String get_current_date(void);                    // weak-заглушка  
String get_current_time(void);                    // weak-заглушка  
void   watchdog_update(void);                     // weak-заглушка  
void   remount_usb_disk(void);                    // weak-заглушка  
  
// --- из vfo_hardware.h (RP2040_beacon) ---  
#define PASSPORT_CLK_SYS_MAX_HZ 133000000UL  
#define PASSPORT_PLL_VCO_MAX_HZ 1600000000UL  
#define VFO_XOSC_NOMINAL_HZ     12000000UL  
#define VFO_XOSC_DEFAULT_PPB    0  
#ifndef BCN_VER  
#define BCN_VER "2.19-port"  
#define BCN_DAT __DATE__  
#endif  
// volatile обязателен — должно совпадать с объявлением в soft_tx.h,  
// иначе conflicting declaration. Определение живёт в soft_tx.cpp.  
extern volatile bool clk_boosted;  
uint64_t vfo_calibrated_xosc_hz(void);            // weak-заглушка  
  
#endif