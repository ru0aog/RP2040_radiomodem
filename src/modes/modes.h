// src/modes/modes.h  
// ============================================================================  
// Общий интерфейс мода (приём + передача). Каждая мода живёт в своей  
// подпапке src/modes/<name>/ и экспортирует ModeIface — маршрутизатор  
// rx_decoders и Hayes-слой работают только через эту таблицу.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
  
// Уникальный ID моды; совпадает с RSID-кодом там, где он стандартизован.  
typedef enum {  
  MODE_NONE = 0,  
  MODE_AX25,  
  MODE_CW,  
  MODE_RTTY,  
  MODE_COUNT  
} ModeId;
  
struct ModeIface {  
  ModeId id;  
  const char *name;  
  uint8_t rsid_code;   // NEW: RSID-код моды; 0 = нет авто-детекта  
  void (*rx_init)(uint32_t center_hz);  
  void (*rx_push)(int16_t sample);  
  uint32_t (*tx_frame)(const uint8_t *payload, uint16_t len);  
};
  
// Реестр мод — заполняется в modes_registry.cpp.  
const ModeIface *modes_get(ModeId id);  
const ModeIface *modes_get_by_rsid(uint8_t rsid_code);
