// src/rx/rx_usb_audio.h  
// ============================================================================  
// Вывод демодулированного аудио на виртуальный USB-микрофон (UAC1, PCM16).  
// Источник: аудио-очередь rx_dsp (rx_dsp_get_audio/release_audio/aud_fill).  
// Транспорт: AudioTools USBAudioStream поверх TinyUSB (44100 Гц, стерео,  
// 16 бит). Ресемплер Фарроу 15625 -> 44100 + ФНЧ + АРУ — в rx_usb_audio.cpp.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
  
// ---- Точки входа из .ino (Core 0) ----  
void     usb_audio_init(void);    // инициализация UAC (из setup())  
void     usb_audio_poll(void);    // обслуживание UAC + выкачка очереди (из loop())  
  
// ---- Runtime-режимы вывода ----  
enum UsbAudioMode {  
    UAC_MODE_RAW_IQ = 0,   // сырые I/Q в стерео (I=L, Q=R)  
    UAC_MODE_SSB    = 1,   // демодулированное моно в оба канала (default)  
    UAC_MODE_TEST   = 2,   // тестовый синус 1000 Гц  
};  
void     rx_usb_audio_set_mode(uint8_t mode);   // UAC_MODE_*  
uint8_t  rx_usb_audio_get_mode(void);  
  
// ---- Выбор демодулятора для UAC_MODE_SSB ----  
// Значения: DEMOD_USB/DEMOD_LSB/DEMOD_AM/DEMOD_FM (rx_usb_audio.cpp),  
// транслируются в RXD_DEMOD_* из rx_dsp.h.  
void     rx_usb_audio_set_demod(uint8_t demod);  
uint8_t  rx_usb_audio_get_demod(void);  
  
// ---- Совместимость со старым контрактом ----  
void     rx_usb_audio_set_rate(uint32_t hz);    // частота PCM для хоста  
void     rx_usb_audio_push_block(const int16_t *pcm, size_t n);  
uint32_t rx_usb_audio_rate_hz(void);