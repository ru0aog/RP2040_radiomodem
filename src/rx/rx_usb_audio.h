// src/rx/rx_usb_audio.h  
// ============================================================================  
// Вывод демодулированного аудио на виртуальный USB-микрофон (UAC1, моно  
// PCM16). Обёртка над TinyUSB audio_device; интерфейс стабилен до включения  
// дескриптора в tusb_config.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
  
void   usb_audio_init(void);  
void   usb_audio_poll(void);                       // вызов из loop(), Core 0  
void   usb_audio_push_block(const int16_t *pcm, size_t samples);  
bool   usb_audio_mounted(void);                    // хост открыл микрофон  
void   usb_audio_set_rate(uint32_t hz);            // до init; потом read-only
