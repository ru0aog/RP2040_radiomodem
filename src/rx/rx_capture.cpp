// src/rx/rx_capture.cpp — ADC+DMA захват. КАРКАС.  
#include "rx_capture.h"  
#include "../pins.h"  
  
bool rx_capture_start(void) {  
  Serial.printf("[STUB] rx_capture_start: ADC on GP%u/GP%u not ported yet\n",  
                PIN_ADC_I, PIN_ADC_Q);  
  return true;  
}  
void     rx_capture_stop(void)          {}  
uint32_t rx_capture_rate_hz(void)       { return 0; }  
uint32_t rx_capture_overruns(void)      { return 0; }  
void     rx_capture_debug(void)         { Serial.println("[STUB] rx_capture_debug"); }
