// src/rx/rx_pipeline.cpp  
#include <Arduino.h>  
#include "rx_dsp.h"  
#include "rx_capture.h"  
  
static uint32_t s_freq_hz = 0;        // последняя рабочая частота RX  
  
extern "C" void rx_pipeline_set_freq(uint32_t hz) { s_freq_hz = hz; }  
  
extern "C" void rx_pipeline_stop(void) {  
    rx_capture_stop();  
    rx_dsp_stop();  
}  
  
extern "C" void rx_pipeline_start(void) {  
    // частота берётся из VFO: LO уже стоит после hook'а,  
    // а рабочая — последняя запрошенная (trx_switch её знает).  
    rx_dsp_start(s_freq_hz);  
    rx_capture_start();  
}
