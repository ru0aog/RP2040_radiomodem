// src/vfo/vfo.h  
// ============================================================================  
// Единый менеджер частот радиомодема RP2040_radiomodem.  
// Единственная точка входа: CAT, планировщик, trx_control, декодеры.  
// Движки (soft_tx/soft_rx/si5351) наружу не торчат.  
// Модель: полудуплекс, активен ровно один генератор.  
// ВСЕ ЧАСТОТЫ — uint32_t в ГЕРЦАХ. Дробных/МГц-полей нет.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
#include "../trx/trx_control.h"  
  
// ---------- Типы ----------  
typedef enum {  
  VFO_BACKEND_NONE    = 0,  
  VFO_BACKEND_SOFT_TX = 1,   // программный меандр + MASH-дизеринг (pio0)  
  VFO_BACKEND_SOFT_RX = 2,   // квадратурный LO (pio1)  
  VFO_BACKEND_SI5351  = 3,   // внешний генератор по I2C  
} VfoBackendId;  
  
#define VFO_BANK_MAX_TONES 4  
#define VFO_TONE_NONE      0xFFu  
  
// Банк тонов — ТОЛЬКО целые герцы, вычислений на лету нет.  
typedef struct {  
  uint32_t base_hz;                     // частота тона 0  
  uint32_t step_hz;                     // шаг сетки  
  uint32_t freqs_hz[VFO_BANK_MAX_TONES];// частоты тонов, Гц  
  uint8_t  count;  
} VfoBank;  
  
// Таблица виртуальных функций движка. Движок обязан быть идемпотентным  
// по init() и не трогать чужие ядра — координация в vfo_on_trx_switch().  
typedef struct VfoEngine {  
  void     (*init)(void);              // резерв PIO/DMA, статические ресурсы  
  void     (*start)(uint32_t freq_hz); // запуск генерации (LO или тон 0)  
  void     (*stop)(void);              // тишина на выходе  
  void     (*tone)(uint8_t idx);       // мгновенная смена тона из банка  
  void     (*poll)(void);              // сервисный тик из loop1()  
  uint32_t (*actual_hz)(void);         // фактическая выходная частота, Гц  
} VfoEngine;  
  
const VfoEngine *vfo_engine_soft_tx(void);  
const VfoEngine *vfo_engine_soft_rx(void);  
const VfoEngine *vfo_engine_si5351(void);  
  
// ---------- Инициализация ----------  
void vfo_init(void);  
  
// ---------- Банк тонов ----------  
void     vfo_bank_set_grid(uint32_t base_hz, uint32_t step_hz, uint8_t count);  
void     vfo_tone(uint8_t index);  
uint8_t  vfo_tone_active(void);  
const VfoBank *vfo_bank(void);  
uint32_t vfo_bank_tone_hz(uint8_t index);  
uint32_t vfo_bank_step_hz(void);  
  
// ---------- Выбор бэкенда ----------  
void vfo_set_backend(TrxState dir, VfoBackendId id);  
  
// ---------- Hook trx_control ----------  
void vfo_on_trx_switch(TrxState next, uint32_t freq_hz);  
  
// ---------- Сервис из loop1() ----------  
void vfo_poll(void);  
  
// ---------- Разгон / термогвард ----------  
void     vfo_boost_enter(uint32_t target_hz);  
void     vfo_thermal_guard(void);  
uint64_t vfo_xosc_hz(void);  
  
// ---------- Диагностика ----------  
uint32_t vfo_clk_sys_hz(void);  
uint32_t vfo_actual_hz(void);  
void     vfo_debug_dump(void);  
  
// Квадратурный LO приёмника (низкоуровневый API для калибровки/теста).  
void     vfo_quad_start(uint32_t rx_freq_hz);  
void     vfo_quad_stop(void);  
uint32_t vfo_quad_lo_hz(void);  
  
// ---------- Пределы разгона (паспорт стабильности, из OCTEST/SET.TXT) ----------  
extern uint32_t vfo_max_clk_sys_hz;  
extern uint32_t vfo_max_pll_vco_hz;

// ПЧ приёмника: квад-LO ставится на f_rx - RX_IF_SHIFT_HZ,  
// остаток добирает цифровой NCO в rx_dsp. Общий параметр системы —  
// его же использует будущий DDC.  
#define RX_IF_SHIFT_HZ  11000UL