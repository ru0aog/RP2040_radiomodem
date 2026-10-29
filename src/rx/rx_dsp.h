// src/rx/rx_dsp.h  
// ============================================================================  
// Децимирующий DSP-тракт (Core 1, только RX-фаза).  
// Порт из RP2040_receiver/src/dsp/rx_dsp.cpp v2.26.  
//  
// Тракт: 32-бит слова [I|Q] из rx_capture → центрирование → NCO-сдвиг →  
// CIC /2 → 2 каскада half-band FIR (/4) → 15625 sps I/Q + демодулятор  
// (RAW/USB/LSB/AM/FM) → полосовой ФНЧ → notch → NR → аудио-очередь.  
// RSID: ресемплер Фарроу 15625→11025 встроен, работает постоянно.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
  
// ---- Параметры захвата (общие с rx_capture) ----  
#define RX_ADC_SAMPLE_RATE_HZ  250000UL  // суммарно ADC0+ADC1, ~125 ksps/канал  
#define RX_BLOCK_SAMPLES       512       // пар I/Q в DMA-блоке  
#define RX_DSP_OUT_RATE_HZ     15625u    // выход после децимации /8  
  
// ---- Управление (Core 0) ----  
void     rx_dsp_init(void);                    // одноразовая инициализация таблиц  
void     rx_dsp_start(uint32_t freq_hz);       // старт тракта под частоту, Гц  
void     rx_dsp_stop(void);                    // останов (перед rx_capture_stop)  
void     rx_dsp_poll(void);                    // вызов из loop1() в RX-фазе  
  
// ---- NCO / демодулятор ----  
void     rx_dsp_set_nco_hz(int32_t f_hz);      // AFC, подстройка на лету  
void     rx_dsp_set_demod(uint8_t mode);       // RXD_DEMOD_*  
#define RXD_DEMOD_RAW 0   // сырые I/Q стерео  
#define RXD_DEMOD_USB 1  
#define RXD_DEMOD_LSB 2  
#define RXD_DEMOD_AM  3  
#define RXD_DEMOD_FM  4  
  
// ---- Аудио-фильтры ----  
void     rx_dsp_set_af_band(uint32_t f_lo_hz, uint32_t f_hi_hz);  
uint32_t rx_dsp_af_lo_hz(void);  
uint32_t rx_dsp_af_hi_hz(void);  
void     rx_dsp_set_notch(bool on);  
void     rx_dsp_set_nr(bool on);  
bool     rx_dsp_get_nr(void);  
  
// ---- Очереди для Core 0 ----  
// Базовая полоса I/Q (для цифровых декодеров-мод через rx_decoders)  
bool     rx_dsp_get_output(const int16_t** i, const int16_t** q, uint8_t* n);  
void     rx_dsp_release_output(void);  
// Моно-аудио/демод-выход (usb_audio, монитор)  
bool     rx_dsp_get_audio(const int16_t** i, const int16_t** q, uint8_t* n);  
void     rx_dsp_release_audio(void);  
uint8_t  rx_dsp_aud_fill(void);  
  
// ---- Диагностика ----  
uint32_t rx_dsp_out_dropped(void);  
uint32_t rx_dsp_aud_dropped(void);  
uint32_t rx_dsp_clip_count(void);  
uint32_t rx_dsp_load_permille(void);         // суммарная загрузка Core 1  
uint32_t rx_dsp_compute_load_permille(void); // только DSP  
uint32_t rx_dsp_rsid_load_permille(void);    // только RSID  
void     rx_dsp_debug(void);