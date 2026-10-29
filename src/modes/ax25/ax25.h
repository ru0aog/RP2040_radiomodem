// src/modes/ax25/ax25.h  
// ============================================================================  
// AX.25 300 bd: AFSK + NRZI + HDLC UI-фреймы. Общий API обеих сторон.  
// Разнос тонов ±100 Гц вокруг центра (адаптация RU0AOG для HF, НЕ Bell 202).  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
#include "../modes.h"  
  
// ---------- Общий протокольный слой (ax25_hdlc.cpp) ----------  
uint16_t ax25_fcs(const uint8_t *data, size_t len);   // CRC-16/X.25  
  
// ---------- TX ----------  
bool ax25_tx_frame(const uint8_t *payload, size_t len);  
bool ax25_tx_busy(void);        // идёт посылка  
  
// ---------- RX ----------  
void ax25_rx_init(uint32_t center_hz);  
void ax25_rx_push(int16_t sample);                  // fs = 15625 Гц  
  
// Колбэк декодированного UI-фрейма (FCS проверен). Установить до init;  
// не задан — кадры печатаются в Serial.  
typedef void (*ax25_frame_cb_t)(const uint8_t *frame, size_t len);  
void ax25_rx_set_frame_cb(ax25_frame_cb_t cb);  
  
// ---------- Регистрация ----------  
const ModeIface *mode_ax25(void);
