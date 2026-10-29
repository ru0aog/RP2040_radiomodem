// src/modem/modem_data.cpp — DATA-режим. КАРКАС.  
#include "modem_data.h"  
  
void modem_data_push(uint8_t b) { (void)b; }  
void modem_data_out(uint8_t b)  { (void)b; }  
bool modem_data_busy(void)      { return false; }
