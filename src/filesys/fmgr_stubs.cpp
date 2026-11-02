// src/filesys/fmgr_stubs.cpp — определения weak-заглушек из fmgr_compat.h.  
// Радиомодем не имеет RTC и планировщика маяка — здесь нейтральные  
// реализации, чтобы file_manager.cpp собирался и работал.  
// УДАЛИТЬ соответствующие функции после портирования реальных модулей  
// (weak позволяет реальному определению молча перекрыть заглушку).  
  
#include "fmgr_compat.h"  
#include "file_manager.h"  
  
// --- RTC-переменные (маяк держит их в .ino, здесь — фиксированные) ---  
__attribute__((weak)) uint16_t rtc_year  = 2024;  
__attribute__((weak)) uint8_t  rtc_month = 1;  
__attribute__((weak)) uint8_t  rtc_day   = 1;  
__attribute__((weak)) uint8_t  rtc_hour  = 0;  
__attribute__((weak)) uint8_t  rtc_min   = 0;  
__attribute__((weak)) uint8_t  rtc_sec   = 0;  
  
// --- Строки даты/времени для LOG.TXT ---  
// Если в проекте есть свой источник времени (GPS/NTP/RTC) —  
// замени тело на него, или удали эти weak-определения.  
__attribute__((weak)) String get_current_date(void) {  
  char buf[11];  
  snprintf(buf, sizeof(buf), "%04u-%02u-%02u",  
           (unsigned)rtc_year, (unsigned)rtc_month, (unsigned)rtc_day);  
  return String(buf);  
}  
  
__attribute__((weak)) String get_current_time(void) {  
  char buf[9];  
  snprintf(buf, sizeof(buf), "%02u:%02u:%02u",  
           (unsigned)rtc_hour, (unsigned)rtc_min, (unsigned)rtc_sec);  
  return String(buf);  
}  
  
// --- Сторожевой таймер (в маяке — rp2040.wdt_reset) ---  
__attribute__((weak)) void watchdog_update(void) {  
#if defined(ARDUINO_ARCH_RP2040)  
  // раскомментировать, если WDT включён:  
  // rp2040.wdt_reset();  
#endif  
}  
  
// --- Перемонтирование USB-MSC после изменения диска ---  
__attribute__((weak)) void remount_usb_disk(void) {  
  //usb_msc.setUnitReady(false);  
  delay(50);  
  //usb_msc.setUnitReady(true);  
}  
  
// --- Планировщик маяка (в радиомодеме не портирован) ---  
__attribute__((weak)) int get_next_start_minute(BeaconMode /*mode*/) {  
  return -1;   // задач нет  
}  
  
__attribute__((weak)) String fmt_next_start(int minute) {  
  if (minute < 0) return F("--:--");  
  char buf[6];  
  snprintf(buf, sizeof(buf), "%02d:%02d", minute / 60, minute % 60);  
  return String(buf);  
}  
  
// --- Калиброванная частота кварца (маяк считает по XOSC_PPB) ---  
__attribute__((weak)) uint64_t vfo_calibrated_xosc_hz(void) {  
  return (uint64_t)VFO_XOSC_NOMINAL_HZ;  
}