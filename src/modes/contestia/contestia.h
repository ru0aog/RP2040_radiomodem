// src/modes/contestia/contestia.h — CONTESTIA 8/250: банк Гёрцеля →  
// Walsh-декод 3×6 бит/блок, вход моно-AF 15625 sps.  
// Порт rx_contestia.cpp из RP2040_receiver.  
#pragma once  
#include <Arduino.h>  
#include "../modes.h"  
  
void contestia_rx_init(uint32_t center_hz);   // центр RSID → base = center - 3.5×31.25  
void contestia_rx_push(int16_t sample);       // моно-AF 15625 sps  
void contestia_rx_stop(void);                 // сброс состояния  
  
const ModeIface* contestia_mode_iface(void);  // реализация в contestia_mode.cpp
