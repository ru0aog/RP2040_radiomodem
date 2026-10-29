// src/vfo/vfo.h  
// Единый менеджер частот радиомодема.  
// Единственная точка входа для всех потребителей: CAT, планировщик,  
// trx_control, декодеры. Внутренние движки (soft_tx/soft_rx/si5351)  
// наружу не торчат.  
#pragma once  
#include <Arduino.h>  
#include "../trx/trx_control.h"   // TrxState  
  
// ---- Бэкенды генерации ----  
enum VfoBackend : uint8_t {  
  VFO_BACKEND_NONE = 0,  
  VFO_BACKEND_SOFT_TX,   // PIO меандр + MASH-дизеринг (передача)  
  VFO_BACKEND_SOFT_RX,   // PIO квадратура I/Q (гетеродин приёмника)  
  VFO_BACKEND_SI5351,    // внешний синтезатор (заглушка, на перспективу)  
};  
  
// ---- Банк тонов ----  
#define VFO_BANK_MAX_TONES  64     // совместимо с VFO_TONES_COUNT исходников  
#define VFO_TONE_NONE       255  
  
// Частоты храним в миллигерцах (мГц = Гц·1000): точность DDS-расчёта  
// исходного движка сохраняется без float.  
struct VfoBank {  
  uint64_t freqs_mhz[VFO_BANK_MAX_TONES];  // частота каждого тона, мГц  
  uint8_t  count;                           // сколько ячеек занято  
};  
  
// ======================== API ========================  
  
// Инициализация: пины, PIO, чтение калибровок.  
// Вызывать из setup() до trx_init().  
void     vfo_init();  
  
// Выбор аппаратного бэкенда для направления. По умолчанию:  
// RX → SOFT_RX (квадро-LO), TX → SOFT_TX (дизеринг).  
// Вызов si5351_set_backend() перебьёт TX на внешний синтезатор.  
void     vfo_set_backend(TrxState dir, VfoBackend b);  
  
// --- Банк частот ---  
// Загрузить банк равномерной сеткой: base + i*step (FSK-сетки  
// RTTY/MFSK/Olivia). Пересчитывает таблицу делителей PIO.  
void     vfo_bank_set_grid(uint32_t base_hz, double step_hz, uint8_t count);  
// Загрузить произвольный список частот (JTTY/IFKP-подобные сетки).  
void     vfo_bank_set_list(const uint32_t *freqs_hz, uint8_t count);  
const VfoBank *vfo_bank();                  // read-only доступ к банку  
  
// --- Управление тоном ---  
// Включить тон №index из банка (мгновенная смена делителя PIO,  
// фаза MASH не сбрасывается). VFO_TONE_NONE — тишина на выходе.  
void     vfo_tone(uint8_t index);  
uint8_t  vfo_tone_active();  
  
// --- Приёмный гетеродин ---  
// Запустить квадратурный LO: на freq_hz−RX_IF_SHIFT_HZ.  
// Возвращает фактическую частоту LO после квантования clkdiv (для NCO).  
uint32_t vfo_quad_start(uint32_t rx_freq_hz);  
void     vfo_quad_stop();  
uint32_t vfo_quad_lo_hz();                  // фактический LO, Гц  
  
// --- Разгон (PLL boost) ---  
// Поднять clk_sys под целевую частоту (пересчёт банка + vreg).  
// Вызов только в фазе TX, когда RX-конвейер остановлен.  
void     vfo_boost_enter(uint32_t target_hz);  
void     vfo_boost_exit();                  // возврат на номинал 133 МГц  
bool     vfo_boosted();  
// Термогвард: понижение при >VFO_THROTTLE_HI_C. Дёргать из loop().  
void     vfo_thermal_guard();  
  
// --- Сервис trx_control ---  
// Hook для trx_init(): останавливает старый генератор, запускает  
// бэкенд нового направления на freq_hz.  
void     vfo_on_trx_switch(TrxState next, uint32_t freq_hz);  
  
// --- Калибровка опоры ---  
// Коррекция XOSC в ppb, загружается из Flash ([XOSC_PPB] в SET.TXT).  
extern int32_t vfo_xosc_corr_ppb;  
uint64_t vfo_xosc_hz();                     // скорректированная опора  
  
// --- Диагностика ---  
uint32_t vfo_clk_sys_hz();                  // текущая частота шины  
void     vfo_debug_dump();                  // дамп состояния в Serial  
  
// ============ Интерфейс внутренних движков ============  
// Реализуется каждым бэкендом; vfo.cpp только диспетчеризует.  
struct VfoEngine {  
  void     (*init)();  
  void     (*start)(uint32_t freq_hz);   // запуск генерации  
  void     (*stop)();                     // тишина на выходе  
  void     (*tone)(uint8_t index);        // мгновенная смена тона  
  void     (*poll)();                     // сервисный вызов из loop1()  
  uint32_t (*actual_hz)();                // фактическая частота  
};  
const VfoEngine *vfo_engine_soft_tx();  
const VfoEngine *vfo_engine_soft_rx();  
const VfoEngine *vfo_engine_si5351();
