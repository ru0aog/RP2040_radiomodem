// src/rx/rx_dsp.h  
// ============================================================================  
// DSP-тракт RX: 32-бит слова [Q|I] от rx_capture → центрирование →  
// NCO-сдвиг → 3 каскада half-band FIR (/8) → базовая полоса int16 I/Q  
// блоками по 64 отсчёта, 15625 sps. Плюс демодуляция (USB/LSB/AM/FM),  
// AF-полосовой, notch, NR — порт из RP2040_receiver/src/dsp/rx_dsp.cpp.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
  
#define RX_DSP_BLOCK_SAMPLES   64  
#define RX_DSP_OUT_RATE_HZ     15625u   // целевой выходной сэмплрейт на канал  
  
// Инициализация таблиц NCO/FIR. Вызвать до rx_capture_start().  
void rx_dsp_init(void);  
  
// Вход от rx_capture (Core 1): блок 32-бит слов [Q<<16 | I].  
void rx_dsp_push_raw(const uint32_t *words, size_t count);  
  
// Сервисный вызов из loop1() — перекладка блоков в очередь выхода.  
void rx_dsp_poll(void);  
  
// Частота NCO, Гц (смещение LO относительно центра — RX_IF_SHIFT_HZ + остаток  
// квантования квадрата). Выставляет менеджер при старте квада.  
void rx_dsp_set_nco_hz(uint32_t hz);  
  
// Демодулятор: выбор моды детектора аудиовыхода.  
typedef enum { RX_DEMOD_USB, RX_DEMOD_LSB, RX_DEMOD_AM, RX_DEMOD_FM } RxDemod;  
void rx_dsp_set_demod(RxDemod d);  
  
// Выход аудио (моно int16, RX_DSP_OUT_RATE_HZ) — для декодеров и USB-мика.  
// Возвращает число выданных сэмплов (0 — данных нет).  
size_t rx_dsp_get_audio(int16_t *out, size_t max_samples);  

// src/rx/rx_dsp.h  
#define RX_DSP_OUT_RATE_HZ  15625u   // выход декимирующего тракта, сэмплов/с

// Диагностика.  
uint32_t rx_dsp_dropped_blocks(void);  
uint32_t rx_dsp_clip_count(void);  
void     rx_dsp_debug(void);
