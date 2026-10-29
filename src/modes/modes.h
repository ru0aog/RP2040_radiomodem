// src/modes/modes.h  
// ============================================================================  
// Общий интерфейс моды. Каждая мода — самодостаточная папка src/modes/<name>/  
// с кодером, декодером и общим протокольным слоем. Наружу торчит только  
// этот контракт; прямой доступ к VFO/PIO у мод нет (только через vfo.h  
// и trx_control).  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
  
struct ModeIface {  
  const char *name;                            // "ax25", "rtty", ...  
  
  // TX: true — посылка пошла (мода сама делает trx_switch TX→RX).  
  bool (*tx_frame)(const uint8_t *payload, size_t len);  
  
  // RX: установка центра демодулятора, Гц; пер-сэмпловая точка входа.  
  void (*rx_init)(uint32_t center_hz);  
  void (*rx_push)(int16_t sample);             // fs = 15625 Гц  
};

struct ModeRx {  
  void (*init)(uint32_t center_hz);   // подстройка под центр ПЧ  
  void (*push)(int16_t mono);         // 15625 sps от rx_dsp  
  void (*stop)(void);  
};  
struct ModeTx {  
  bool (*send)(const uint8_t* data, uint16_t len); // кадр → TRX_TX  
  bool (*busy)(void);  
};  
struct ModeIface {  
  const char* name;        // "ax25"  
  uint8_t     rsid_code;   // код RSID для автодетекта (0 = нет)  
  ModeRx      rx;  
  ModeTx      tx;  
};  
const ModeIface* mode_find(const char* name);  
const ModeIface* mode_find_rsid(uint8_t code);
