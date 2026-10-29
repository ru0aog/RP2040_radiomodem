// src/usb/usb_audio.h  
#pragma once  
#include <Arduino.h>  
  
// КАРКАС UAC-микрофона. Реальный дескриптор/стрим — после DSP.  
void usb_audio_init(void);   // регистрация дескриптора (пока no-op)  
void usb_audio_poll(void);   // сервисный вызов из loop()