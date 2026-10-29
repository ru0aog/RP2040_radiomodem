// src/vfo/vfo.h  
// ============================================================================  
// Единый менеджер частот радиомодема RP2040_radiomodem.  
//  
// Единственная точка входа для всех потребителей: CAT, планировщик,  
// trx_control, декодеры. Внутренние движки (soft_tx/soft_rx/si5351)  
// наружу не торчат — менеджер диспетчеризует через VfoEngine.  
//  
// Модель: полудуплекс. Одновременно активен ровно один генератор:  
//  - RX: квадратурный LO (soft_rx, pio1) для SDR-смесителя;  
//  - TX: дизеринг-генератор (soft_tx, pio0) либо Si5351 (перспектива).  
// Переключение идёт через vfo_on_trx_switch() — hook trx_control.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
#include "../trx/trx_control.h"   // TrxState  
  
// ============================================================================  
// БЭКЕНДЫ ГЕНЕРАЦИИ  
// ============================================================================  
enum VfoBackend : uint8_t {  
  VFO_BACKEND_NONE = 0,  
  VFO_BACKEND_SOFT_TX,   // PIO-меандр + MASH-дизеринг (передача)  
  VFO_BACKEND_SOFT_RX,   // PIO-квадратура I/Q (гетеродин приёмника)  
  VFO_BACKEND_SI5351,    // внешний синтезатор (заглушка, на перспективу)  
};  
  
// ============================================================================  
// БАНК ТОНОВ  
// ============================================================================  
#define VFO_BANK_MAX_TONES  64    // = SOFT_TX_TONES / VFO_TONES_COUNT исходника  
#define VFO_TONE_NONE       255   // тишина  
  
// Частоты — в миллигерцах (мГц = Гц×1000): сохраняет точность  
// DDS-расчёта исходного движка без float.  
struct VfoBank {  
  uint64_t freqs_mhz[VFO_BANK_MAX_TONES];  
  uint8_t  count;  
};  
  
// ============================================================================  
// ВНЕШНИЙ API МЕНЕДЖЕРА  
// ============================================================================  
  
// Инициализация: пины, PIO, регистрация движков.  
// Вызывать из setup() ДО trx_init() — hook может сработать сразу.  
void     vfo_init(void);  
  
// Выбор бэкенда направления. По умолчанию: RX→SOFT_RX, TX→SOFT_TX.  
// Например vfo_set_backend(TRX_TX, VFO_BACKEND_SI5351) переводит  
// передачу на внешний синтезатор.  
void     vfo_set_backend(TrxState dir, VfoBackend b);  
  
// --- Банк частот ---  
// Равномерная сетка: base + i·step (RTTY/MFSK/Olivia FSK-сетки).  
void     vfo_bank_set_grid(uint32_t base_hz, double step_hz, uint8_t count);  
// Произвольный список частот (JTTY/IFKP-подобные сетки).  
void     vfo_bank_set_list(const uint32_t *freqs_hz, uint8_t count);  
// Read-only доступ к банку (движки пересчитывают по нему таблицы).  
const VfoBank *vfo_bank(void);  
  
// --- Управление тоном ---  
// Мгновенная смена тона из банка (фаза генератора не прерывается).  
// VFO_TONE_NONE — тишина на выходе.  
void     vfo_tone(uint8_t index);  
uint8_t  vfo_tone_active(void);  
  
// --- Приёмный гетеродин (обёртки над RX-бэкендом) ---  
// Запуск квадратурного LO на freq_hz−RX_IF_SHIFT_HZ.  
// Возвращает фактическую частоту LO после квантования делителя —  
// передавать в NCO приёмника.  
uint32_t vfo_quad_start(uint32_t rx_freq_hz);  
void     vfo_quad_stop(void);  
uint32_t vfo_quad_lo_hz(void);  
  
// --- Разгон (PLL boost) ---  
// Поднять clk_sys под целевую частоту (скан метрики спуров + vreg).  
// Только в фазе TX, когда RX-конвейер остановлен.  
void     vfo_boost_enter(uint32_t target_hz);  
void     vfo_boost_exit(void);           // возврат на номинал 133 МГц  
bool     vfo_boosted(void);  
  
// Термогвард: >80°C → откат на номинал, <60°C → возврат в overclock.  
// Дёргать периодически из loop() на Core 0.  
void     vfo_thermal_guard(void);  
  
// --- Сервисный вызов активного бэкенда ---  
// Дёргать каждую итерацию loop1(). Для TX это MASH-дизеринг  
// (soft_tx_poll); у квадратуры и Si5351 poll пустой.  
void     vfo_poll(void);  
  
// --- Hook для trx_control ---  
// Останавливает старый генератор, перенастраивает, запускает бэкенд  
// нового направления на freq_hz. Передать в trx_init().  
void     vfo_on_trx_switch(TrxState next, uint32_t freq_hz);  
  
// --- Калибровка опоры ---  
// Коррекция XOSC в ppb; заполняется из Flash ([XOSC_PPB] в SET.TXT)  
// файловым менеджером до vfo_init(). До того — дефолт.  
extern int32_t  vfo_xosc_corr_ppb;  
uint64_t vfo_xosc_hz(void);              // скорректированная опора, Гц  
  
// --- Диагностика ---  
uint32_t vfo_clk_sys_hz(void);           // текущая частота шины  
void     vfo_debug_dump(void);           // дамп состояния в Serial  
  
// ============================================================================  
// ИНТЕРФЕЙС ВНУТРЕННИХ ДВИЖКОВ  
// ============================================================================  
// Реализуется каждым бэкендом; vfo.cpp только диспетчеризует.  
// Все вызовы исполняются на Core 1 внутри trx-последовательности  
// (кроме init — из setup(), и poll — из loop1()).  
struct VfoEngine {  
  void     (*init)(void);               // резерв ресурсов, загрузка PIO-программ  
  void     (*start)(uint32_t freq_hz);  // запуск генерации  
  void     (*stop)(void);               // тишина на выходе  
  void     (*tone)(uint8_t index);      // мгновенная смена тона из банка  
  void     (*poll)(void);               // сервисный вызов из loop1()  
  uint32_t (*actual_hz)(void);          // фактическая частота, Гц  
};  
  
const VfoEngine *vfo_engine_soft_tx(void);  
const VfoEngine *vfo_engine_soft_rx(void);  
const VfoEngine *vfo_engine_si5351(void);
