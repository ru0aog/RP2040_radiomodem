// src/rx/rx_usb_audio.h  
// ============================================================================  
// Вывод демодулированного аудио на виртуальный USB-микрофон (UAC1, моно  
// PCM16). Обёртка над TinyUSB audio_device; интерфейс стабилен до включения  
// дескриптора в tusb_config.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
  
void     rx_usb_audio_set_rate(uint32_t hz);              // частота PCM для хоста  
void     rx_usb_audio_push_block(const int16_t *pcm, size_t n);  
uint32_t rx_usb_audio_rate_hz(void);