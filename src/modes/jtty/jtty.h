// src/modes/jtty/jtty.h — JTTY: 4-FSK + KВ-код + Витерби + CRC16.  
// Порт rx_jtty.cpp из RP2040_receiver. Вход: моно-AF 15625 sps.  
#pragma once  
#include <Arduino.h>  
#include "../modes.h"  
  
void jtty_rx_init(uint32_t center_hz);   // центр RSID → 4 тона ±step  
void jtty_rx_push(int16_t sample);       // моно-AF 15625 sps  
void jtty_rx_stop(void);                 // сброс состояния декодера  
  
const ModeIface* jtty_mode_iface(void);  // реализация в jtty_mode.cpp