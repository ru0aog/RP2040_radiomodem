// src/vfo/soft_tx.h  
// ============================================================================  
// Дизеринг-движок программного DDS VFO (передача): PIO-меандр на pio0 +  
// кооперативный MASH-1-1 дизеринг clkdiv + автотюн PLL.  
// Все частоты — Гц.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
#include "vfo.h"  
#include "hardware/vreg.h"  
  
// ---------- Конфигурация PLL ----------  
// vco = xosc/refdiv * fbdiv;  clk_sys = vco/(p1*p2).  
// Имена/порядок полей — единый контракт с soft_rx.cpp и с инициализаторами  
// pll_nominal/pll_overclock/pll_ceiling в soft_tx.cpp:  
//   { fbdiv, p1, p2, clk_sys_hz, vsel, is_oc, refdiv }  
typedef struct {  
  uint32_t fbdiv;       // множитель VCO  
  uint32_t p1, p2;      // post-делители  
  uint32_t clk_sys_hz;  // итоговая частота шины, Гц  
  uint32_t vsel;        // VREG_VOLTAGE_*  
  bool     is_oc;       // true — конфигурация разгона (>133 МГц)  
  uint32_t refdiv;      // делитель опоры PLL (обычно 1)  
} PllConfig;  
  
extern PllConfig pll_nominal;    // 133 МГц, дефолт  
extern PllConfig pll_overclock;  // выбранный boost-конфиг  
extern PllConfig pll_ceiling;    // верхняя граница для термогварда  
  
// Пределы разгона (weak-дефолты в vfo.cpp, сильные здесь нет — только extern)  
extern uint32_t vfo_max_clk_sys_hz;  
extern uint32_t vfo_max_pll_vco_hz;  
  
// ---------- Параметры тона дизеринга ----------  
// Число тонов в таблице — совпадает с банком менеджера.  
#define SOFT_TX_TONES   16  
  
// Таблица лежит в .time_critical — Core 1 читает каждый цикл без Flash.  
struct SoftTxParams {  
  uint32_t pio_int;         // целая часть делителя PIO (clkdiv int)  
  uint32_t pio_frac;        // дробная часть 0..255  
  uint32_t target_freq_chz; // целевая частота, сантигерцы (Hz*100)  
  uint32_t dds_step;        // шаг DDS-дизеринга  
  uint8_t  frac_mask;       // маска бит дизеринга (0..3)  
};  
  
// ---------- PLL-хелперы (их же зовёт soft_rx) ----------  
uint32_t soft_tx_vsel_for(uint64_t vco_hz);  
bool     soft_tx_set_clk_sys(const PllConfig &cfg, uint32_t vsel);  
uint32_t soft_tx_clk_sys_hz(void);  
  
// ---------- Движок ----------  
const VfoEngine *vfo_engine_soft_tx(void);  
uint32_t soft_tx_actual_hz(void);  
  
// ---------- Разгон/термогвард ----------  
void soft_tx_boost_enter(uint32_t target_hz);  
void soft_tx_boost_exit(void);  
void soft_tx_thermal_guard(void);  
void soft_tx_set_narrowband(bool en);  
  
extern volatile bool clk_boosted;  
  
// ---------- Диагностика ----------  
void soft_tx_debug(void);