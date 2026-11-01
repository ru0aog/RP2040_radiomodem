// src/modes/ifkp/ifkp.h — IFKP: incremental FSK, 33 тона шаг 11.697 Гц,  
// символ ~500 мс, дельта-декодинг тонов. Вход моно-AF 15625 sps.  
// Порт rx_ifkp.cpp из RP2040_receiver.  
#pragma once  
#include <Arduino.h>  
#include "../modes.h"  
  
void ifkp_rx_init(uint32_t center_hz);   // центр RSID → base = center - 187.15  
void ifkp_rx_push(int16_t sample);  
void ifkp_rx_stop(void);  
  
const ModeIface* ifkp_mode_iface(void);  // реализация в ifkp_mode.cpp