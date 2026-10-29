// src/rx/rx_dsp.cpp — DDC + децимация /8. КАРКАС.  
#include "rx_dsp.h"  
  
static uint32_t s_nco_hz = 0;  
  
void rx_dsp_init(void) {  
  Serial.println("[STUB] rx_dsp_init: NCO/FIR tables not ported yet");  
}  
void rx_dsp_push_raw(const uint32_t *words, size_t count) {  
  (void)words; (void)count;  
}  
void rx_dsp_poll(void) {}  
void rx_dsp_set_nco_hz(uint32_t hz) { s_nco_hz = hz; }  
void rx_dsp_set_demod(RxDemod d)    { (void)d; }  
size_t rx_dsp_get_audio(int16_t *out, size_t n) { (void)out; (void)n; return 0; }  
uint32_t rx_dsp_dropped_blocks(void) { return 0; }  
uint32_t rx_dsp_clip_count(void)     { return 0; }  
void rx_dsp_debug(void) {  
  Serial.printf("[STUB] rx_dsp: nco=%lu Hz\n", (unsigned long)s_nco_hz);  
}
