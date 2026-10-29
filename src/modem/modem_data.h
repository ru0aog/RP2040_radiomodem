// src/modem/modem_data.h — поток данных DATA-режима. КАРКАС.  
#pragma once  
#include <Arduino.h>  
  
void modem_data_push(uint8_t b);       // байт от хоста → в очередь TX  
void modem_data_out(uint8_t b);        // байт от моды → хосту  
bool modem_data_busy(void);
