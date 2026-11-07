// quad_vfo_pll.cpp  
// Порт PLL-подсистемы из src/vfo/soft_tx.cpp (RU0AOG) + правки аудита.  
//  
// PLL настраивается своим кодом, без pll_init() SDK (тот ждёт LOCK бесконечно).  
// clk_sys переключается напрямую через регистры, без clock_configure.  
//  
// Парковка на время смены clk_sys обязательна — через core_sync  
// (единственный владелец протокола): с Core 1 — core_sync_park_core0(),  
// с Core 0 — core_sync_park_core1(). Путь idleOtherCore() удалён.  
// Serial печатает только с Core 0 (QP_LOG) и только вне окон.
#include "quad_vfo_pll.h"  
#include "../../trx/core_sync.h"
#include "hardware/clocks.h"  
#include "hardware/pll.h"  
#include "hardware/sync.h"  
#include "hardware/timer.h"  
#include "hardware/vreg.h"  
#include "hardware/structs/clocks.h"  
#include "hardware/structs/ssi.h"  
#include "hardware/structs/pll.h"  
#include "hardware/regs/pll.h"
#include <stdio.h>  

#define QUAD_LOCK_TIMEOUT   20000u
#define FLASH_SAFE_MAX_HZ   48000000u
volatile uint32_t quad_vfo_pll_dbg_stage = 0;
static uint32_t s_clk_sys_hz = 133000000;

static bool s_pll_quiet = false;

// USB-стек живёт на Core 0 — с Core 1 не печатаем вообще.  
#define QP_LOG(...)  do { if (!s_pll_quiet && get_core_num() == 0) \
                            Serial.printf(__VA_ARGS__); } while (0)
void quad_vfo_pll_set_quiet(bool on) { s_pll_quiet = on; }  
  
uint32_t quad_vfo_clk_sys_hz(void) { return s_clk_sys_hz; }  
  
static uint32_t meas_clk_sys_hz(void) {  
  return frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) * 1000u;  
}  
uint32_t quad_vfo_clk_sys_meas_hz(void) { return meas_clk_sys_hz(); }  
  
uint32_t quad_vfo_vsel_for(uint64_t clk_hz) {    
  if (clk_hz >= 250000000ULL)      return VREG_VOLTAGE_1_30;    
  else if (clk_hz >= 200000000ULL) return VREG_VOLTAGE_1_25;    
  else if (clk_hz > 133000000ULL)  return VREG_VOLTAGE_1_15;    
  return VREG_VOLTAGE_DEFAULT;    
}

// ---------------------------------------------------------------------------  
// Регистровые хелперы — только RAM, без SDK  
// ---------------------------------------------------------------------------  
static void detach_peri(void) {            // вызывается ВНЕ окна  
  clock_configure(clk_peri, 0,  
                  CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,  
                  48u * 1000000u, 48u * 1000000u);  
}  
  
// Счётный таймаут, НЕ time_us_32(): при перестройке часов таймер  
// может не тикать, и «ограниченное» ожидание станет вечным.  
#define MUX_SEL_TIMEOUT 200000u  
  
static bool __not_in_flash_func(clk_sys_mux_aux)(uint32_t auxsrc) {  
  clock_hw_t *c = &clocks_hw->clk[clk_sys];  
  hw_clear_bits(&c->ctrl, CLOCKS_CLK_SYS_CTRL_SRC_BITS);  
  uint32_t t = MUX_SEL_TIMEOUT;  
  while (!(c->selected & 1u) && --t) {}  
  if (!t) return false;  
  hw_write_masked(&c->ctrl,  
                  auxsrc << CLOCKS_CLK_SYS_CTRL_AUXSRC_LSB,  
                  CLOCKS_CLK_SYS_CTRL_AUXSRC_BITS);  
  c->div = 1u << CLOCKS_CLK_SYS_DIV_INT_LSB;  
  hw_set_bits(&c->ctrl, CLOCKS_CLK_SYS_CTRL_SRC_BITS);  
  t = MUX_SEL_TIMEOUT;  
  while (!(c->selected & 2u) && --t) {}  
  return t != 0;  
}
  
static bool __not_in_flash_func(clk_sys_to_xosc)(void) {  
  return clk_sys_mux_aux(CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC);  
}  
static bool __not_in_flash_func(clk_sys_to_pll)(void) {  
  return clk_sys_mux_aux(CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS);  
}
  
// BAUDR можно менять только при SSIENR=0. Ставить ДО подъёма частоты.  
static void __not_in_flash_func(ssi_set_baud_for)(uint32_t clk_hz) {  
  uint32_t b = (clk_hz + FLASH_SAFE_MAX_HZ - 1) / FLASH_SAFE_MAX_HZ;  
  b = (b + 1u) & ~1u;  
  if (b < 2u)  b = 2u;  
  if (b > 34u) b = 34u;  
  ssi_hw->ssienr = 0;  
  ssi_hw->baudr  = b;  
  ssi_hw->ssienr = 1;  
}  
  
static bool __not_in_flash_func(wait_pll_lock)(void) {  
  uint32_t t = QUAD_LOCK_TIMEOUT;  
  while (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS) && --t) { }  
  return (pll_sys_hw->cs & PLL_CS_LOCK_BITS) != 0;  
}  
  
static bool __not_in_flash_func(pll_setup_bounded)(uint32_t refdiv, uint32_t fbdiv,  
                                                   uint32_t p1, uint32_t p2) {  
  hw_set_bits(&pll_sys_hw->pwr,  
              PLL_PWR_PD_BITS | PLL_PWR_VCOPD_BITS | PLL_PWR_POSTDIVPD_BITS);  
  hw_write_masked(&pll_sys_hw->cs, refdiv << PLL_CS_REFDIV_LSB, PLL_CS_REFDIV_BITS);  
  pll_sys_hw->fbdiv_int = fbdiv;  
  hw_clear_bits(&pll_sys_hw->pwr, PLL_PWR_PD_BITS | PLL_PWR_VCOPD_BITS);  
  if (!wait_pll_lock()) {  
    hw_set_bits(&pll_sys_hw->pwr,  
                PLL_PWR_PD_BITS | PLL_PWR_VCOPD_BITS | PLL_PWR_POSTDIVPD_BITS);  
    return false;  
  }  
  pll_sys_hw->prim = (p1 << PLL_PRIM_POSTDIV1_LSB) | (p2 << PLL_PRIM_POSTDIV2_LSB);  
  hw_clear_bits(&pll_sys_hw->pwr, PLL_PWR_POSTDIVPD_BITS);  
  return true;  
}  
  
// Одно окно: паркуем второе ядро и глушим IRQ на ВСЮ последовательность  
// XOSC -> PLL setup -> baudr -> mux на PLL. Как в маяке (380 МГц, стабильно).  
// 0 = ок, 1 = отказ парковки, 10 = нет LOCK, 11/13 = mux не подтвердил.  
static int __not_in_flash_func(switch_sequence)(const quad_vfo_pll_t *c,  
                                                uint32_t hz) {  
  const bool c1 = (get_core_num() == 1);  
  
  if (c1 ? !core_sync_park_core0(0) : !core_sync_park_core1(0)) return 1;  
  quad_vfo_pll_dbg_stage = 1;  
  uint32_t ints = save_and_disable_interrupts();  
  
  if (!clk_sys_to_xosc())                          { restore_interrupts(ints); goto fail11; }  
  quad_vfo_pll_dbg_stage = 2;  
  if (!pll_setup_bounded(c->refdiv, c->fbdiv, c->p1, c->p2)) {  
                                                   restore_interrupts(ints); goto fail10; }  
  quad_vfo_pll_dbg_stage = 3;  
  ssi_set_baud_for(hz);              // ДО подъёма частоты  
  quad_vfo_pll_dbg_stage = 4;  
  if (!clk_sys_to_pll())                           { restore_interrupts(ints); goto fail13; }  
  
  restore_interrupts(ints);  
  if (c1) core_sync_unpark_core0(); else core_sync_unpark_core1();  
  quad_vfo_pll_dbg_stage = 5;  
  return 0;  
  
fail11: if (c1) core_sync_unpark_core0(); else core_sync_unpark_core1(); return 11;  
fail10: if (c1) core_sync_unpark_core0(); else core_sync_unpark_core1(); return 10;  
fail13: if (c1) core_sync_unpark_core0(); else core_sync_unpark_core1(); return 13;  
}

  
static bool rollback_nominal(const quad_vfo_pll_t *nominal, uint64_t xosc_hz) {    
  int r = switch_sequence(nominal, nominal->clk_sys_hz);
  bool ok = (r == 0);  
  s_clk_sys_hz = ok ? nominal->clk_sys_hz : (uint32_t)xosc_hz;  
  if (!ok) QP_LOG("[QUAD] rollback: r=%d — шина может быть на XOSC!\n", r);  
  return ok;  
}  
  
bool quad_vfo_set_clk_sys(const quad_vfo_pll_t *cfg_in, uint32_t vsel,  
                          const quad_vfo_pll_t *nominal, uint64_t xosc_hz,  
                          bool *out_nominal_ok) {  
  if (out_nominal_ok) *out_nominal_ok = false;    
  if (!cfg_in || !nominal || xosc_hz == 0) return false;    
  // Неблокирующий захват: параллельный вход — контролируемый отказ.  
  if (!core_sync_pll_try_lock()) return false;
  
  quad_vfo_pll_t cfg = *cfg_in;  
  if (cfg.refdiv == 0) cfg.refdiv = 1;  
  if (cfg.p1 < 2) cfg = *nominal;  
  if (cfg.p1 < 2 || cfg.p2 < 1 || cfg.refdiv < 1 ||  
      cfg.fbdiv < 16 || cfg.fbdiv > 320 || cfg.clk_sys_hz == 0) {  
    bool rb = rollback_nominal(nominal, xosc_hz);    
    if (out_nominal_ok) *out_nominal_ok = rb;    
    core_sync_pll_unlock();
    return false;    
  }
  
  uint32_t target_clk_hz = (uint32_t)(((uint64_t)cfg.fbdiv * xosc_hz) /  
                           ((uint64_t)cfg.refdiv * cfg.p1 * cfg.p2));  
  // Реальную частоту меряем — кэш s_clk_sys_hz рассинхронизирован  
  // с soft_tx_set_clk_sys() (у того свой кэш current_clk_sys_hz).  
  uint32_t cur_hz = meas_clk_sys_hz();    
  bool is_oc = (target_clk_hz > cur_hz);   
  
  QP_LOG("[PLL] dbg: entry target=%lu oc=%d\n",    
         (unsigned long)target_clk_hz, (int)is_oc);
  
  // Поднимаем VREG, если переход вверх ИЛИ цель выше номинала 133 МГц —  
  // тогда запас по напряжению нужен в любом случае.  
  if (is_oc || target_clk_hz > 133000000u) {  
    vreg_set_voltage((vreg_voltage)vsel);  
    busy_wait_us(500);  
  }  
  detach_peri();
  quad_vfo_pll_dbg_stage = 0;  
  
  int r = switch_sequence(&cfg, target_clk_hz);    
  QP_LOG("[PLL] dbg: seq r=%d stage=%lu\n", r,    
         (unsigned long)quad_vfo_pll_dbg_stage);
  
  bool     locked_ok = false;  
  uint32_t meas_hz   = 0;  
  const uint64_t expect_hz = cfg.clk_sys_hz;  
  
  if (r == 0) {  
    for (int attempt = 0; attempt < 3; attempt++) {  
      busy_wait_us(300);  
      meas_hz = meas_clk_sys_hz();  
      QP_LOG("[PLL] attempt %d: meas=%lu Hz expect=%llu Hz\n",  
             attempt, (unsigned long)meas_hz, (unsigned long long)expect_hz);  
      if ((uint64_t)meas_hz * 1000ull > expect_hz * 997ull &&  
          (uint64_t)meas_hz * 1000ull < expect_hz * 1003ull) {  
        locked_ok = true;  
        break;  
      }  
      r = switch_sequence(&cfg, target_clk_hz);
      if (r != 0) break;  
    }  
  }  
  
  if (!locked_ok) {    
    QP_LOG("[PLL] dbg: no lock -> rollback\n");    
    bool rb = rollback_nominal(nominal, xosc_hz);    
    if (out_nominal_ok) *out_nominal_ok = rb;    
    if (!is_oc) { busy_wait_us(100); vreg_set_voltage((vreg_voltage)vsel); }    
    core_sync_pll_unlock();
    return false;    
  }    
  
  if (!is_oc) { busy_wait_us(100); vreg_set_voltage((vreg_voltage)vsel); }    
  QP_LOG("[PLL] dbg: done meas=%lu\n", (unsigned long)meas_hz);    
  s_clk_sys_hz = meas_hz;    
  core_sync_pll_unlock();
  return true;    
}

// Откат clk_sys на номинал и обратно — ТОЛЬКО из flash-окна:  
// второе ядро уже припарковано, IRQ выключены, XIP мёртв.  
// Парковку НЕ делаем — она уже выполнена вызывающим.  
static const quad_vfo_pll_t *s_flash_saved_cfg = nullptr;  
static uint32_t s_flash_saved_hz = 0;  
  
bool __not_in_flash_func(quad_vfo_clk_to_nominal_locked)(const quad_vfo_pll_t *nominal) {  
  if (!nominal) return false;  
  s_flash_saved_hz = s_clk_sys_hz;  
  if (!clk_sys_to_xosc()) return false;  
  if (!pll_setup_bounded(nominal->refdiv, nominal->fbdiv,  
                         nominal->p1, nominal->p2)) return false;  
  ssi_set_baud_for(nominal->clk_sys_hz);  
  if (!clk_sys_to_pll()) return false;  
  s_clk_sys_hz = nominal->clk_sys_hz;  
  return true;  
}  
  
bool __not_in_flash_func(quad_vfo_clk_restore_locked)(const quad_vfo_pll_t *oc_cfg,  
                                                      uint32_t oc_hz) {  
  if (!oc_cfg || oc_hz == 0) return false;  
  if (!clk_sys_to_xosc()) return false;  
  if (!pll_setup_bounded(oc_cfg->refdiv, oc_cfg->fbdiv,  
                         oc_cfg->p1, oc_cfg->p2)) return false;  
  ssi_set_baud_for(oc_hz);  
  if (!clk_sys_to_pll()) return false;  
  s_clk_sys_hz = oc_hz;  
  return true;  
}
