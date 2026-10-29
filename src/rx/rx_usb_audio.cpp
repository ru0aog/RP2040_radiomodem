// src/rx/rx_usb_audio.cpp — UAC-микрофон. КАРКАС (дескриптор не подключён).  
#include "rx_usb_audio.h"  
  
static uint32_t s_rate = RX_DSP_OUT_RATE_HZ;  
  
void usb_audio_init(void) {  
  Serial.printf("[STUB] usb_audio_init: UAC mic descriptor not enabled, rate=%lu\n",  
                (unsigned long)s_rate);  
}  
void usb_audio_poll(void) {}  
void usb_audio_push_block(const int16_t *pcm, size_t n) { (void)pcm; (void)n; }  
bool usb_audio_mounted(void) { return false; }  
void usb_audio_set_rate(uint32_t hz) { s_rate = hz; }
