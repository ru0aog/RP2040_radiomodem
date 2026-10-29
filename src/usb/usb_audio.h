// src/usb/usb_audio.h — дескриптор UAC-микрофона. КАРКАС.  
// Подключение: правка tusb_config.h + регистрация ITF в usb_descriptors.  
#pragma once  
#include <Arduino.h>  
  
void usb_audio_desc_init(void);   // регистрация дескриптора (пока no-op)
