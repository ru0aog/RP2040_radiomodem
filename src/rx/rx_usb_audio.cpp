// src/rx/rx_usb_audio.cpp — UAC-микрофон. КАРКАС (дескриптор не подключён).  
#include "rx_usb_audio.h"  
#include "../usb/usb_audio.h"   // usb_audio_write_block — нижний слой  
#include "rx_dsp.h"             // RX_DSP_OUT_RATE_HZ  
  
static uint32_t s_rate = RX_DSP_OUT_RATE_HZ;  
  
void rx_usb_audio_set_rate(uint32_t hz) { s_rate = hz; }  
uint32_t rx_usb_audio_rate_hz(void) { return s_rate; }  
  
void rx_usb_audio_push_block(const int16_t *pcm, size_t n) {  
  // TODO: кольцевой буфер -> usb_audio_write_block()  
  (void)pcm; (void)n;  
}