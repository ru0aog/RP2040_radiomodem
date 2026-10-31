// src/modes/rtty/rtty.h — RTTY-45: matched filters → FMDEM → UART-slicer,  
// вход моно-AF 15625 sps. Порт rx_rtty.cpp из RP2040_receiver.  
#pragma once  
#include <Arduino.h>                 // uint32_t/int16_t  
#include "../modes.h"                // ModeIface  
  
void rtty_rx_init(uint32_t center_hz);   // центр RSID → mark/space ±85 Гц  
void rtty_rx_push(int16_t sample);       // моно-AF 15625 sps  
void rtty_rx_stop(void);  
  
const ModeIface* rtty_mode_iface(void);  // реализация в rtty_mode.cpp