// src/modes/olivia/olivia.h — OLIVIA 8/250 RX: Goertzel-банк → преамбула →  
// Walsh-Hadamard 64→3 символа. Вход моно-AF 15625 sps.  
#pragma once  
#include <Arduino.h>  
#include "../modes.h"  
  
void olivia_rx_init(uint32_t center_hz);  // центр RSID; 0 → дефолт 1500 Гц  
void olivia_rx_push(int16_t sample);  
void olivia_rx_stop(void);  
  
const ModeIface* olivia_mode_iface(void);
