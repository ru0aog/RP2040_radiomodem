// quad_vfo_pll.cpp  
// Порт PLL-подсистемы из src/vfo/soft_tx.cpp (RU0AOG) + правки аудита.  
//  
// Настройка PLL — своя, без pll_init() SDK (тот ждёт LOCK бесконечно).  
// Последовательность повторяет pll_deinit()+pll_init() из clocks.c:  
//   PWR: PD|VCOPD|POSTDIVPD = 1  
//   CS: refdiv (маскированно)  
//   FBDIV_INT  
//   PWR: PD|VCOPD = 0  
//   ждём LOCK с таймаутом QUAD_LOCK_TIMEOUT  
//   PRIM: POSTDIV1|POSTDIV2  
//   PWR: POSTDIVPD = 0  
//  
// Core 0 ВСЕГДА паркуется (rp2040.idleOtherCore) на время переключения  
// clk_sys: диагностикой подтверждено — без парковки Core 0 не переживает  
// смену шины под ногами (flash-код + запросы к XIP).  
// ВНУТРИ парковочных окон Serial/printf ЗАПРЕЩЁН: CDC обслуживается Core 0,  
// printf при припаркованном ядре — дедлок. Диагностика — только  
// SRAM-флаги (quad_vfo_pll_dbg_stage).  
#include "quad_vfo_pll.h"  
#include "hardware/clocks.h"  
#include "hardware/pll.h"  
#include "hardware/sync.h"  
#include "hardware/timer.h"  
#include "hardware/vreg.h"  
#include "hardware/structs/ssi.h"  
#include "hardware/structs/pll.h"  
#include "hardware/regs/pll.h"  
#include <stdio.h>  
  
#define QUAD_LOCK_TIMEOUT 20000u  
  
static uint32_t s_clk_sys_hz = 133000000;  
  
// Тихий режим — см. quad_vfo_set_quiet() в quad_vfo.cpp.  
static bool s_pll_quiet = false;  
#define QP_LOG(...)  do { if (!s_pll_quiet) Serial.printf(__VA_ARGS__); } while (0)  
void quad_vfo_pll_set_quiet(bool on) { s_pll_quiet = on; }  
  
// Диагностический счётчик стадий внутри парковочных окон.  
// Живёт в SRAM, безопасен при припаркованном Core 0 и выключенных IRQ.  
// Читать после зависания через отладчик/дамп:  
//   0 — функция не входила в окно  
//   1 — после idleOtherCore (парковка прошла)  
//   2 — после clk_sys_to_xosc  
//   3 — после pll_setup_bounded  
//   4 — после clk_sys_to_pll (перед resume)  
//   5 — после resumeOtherCore  
volatile uint32_t quad_vfo_pll_dbg_stage = 0;  
  
uint32_t quad_vfo_clk_sys_hz(void) { return s_clk_sys_hz; }  
  
// Измерение clk_sys через SDK-функцию: разрешение 1 кГц.  
static uint32_t __not_in_flash_func(meas_clk_sys_hz)(void) {  
  return frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) * 1000u;  
}  
  
// Публичный вариант для debug-вывода (клапан обычный, без flash-атрибута).  
uint32_t quad_vfo_clk_sys_meas_hz(void) { return meas_clk_sys_hz(); }  
  
uint32_t quad_vfo_vsel_for(uint64_t clk_hz) {  
  if (clk_hz >= 250000000ULL)      return VREG_VOLTAGE_1_30;  
  else if (clk_hz >= 200000000ULL) return VREG_VOLTAGE_1_25;  
  else if (clk_hz > 133000000ULL)  return VREG_VOLTAGE_1_15;  
  return VREG_VOLTAGE_DEFAULT;  
}  
  
static void __not_in_flash_func(detach_peri)(void) {  
  clock_configure(clk_peri, 0,  
                  CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,  
                  48u * 1000000u, 48u * 1000000u);  
}  
  
static void __not_in_flash_func(clk_sys_to_xosc)(uint64_t xosc_hz) {  
  clock_configure(clk_sys,  
                  CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                  CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,  
                  xosc_hz, xosc_hz);  
}  
  
static void __not_in_flash_func(clk_sys_to_pll)(uint64_t hz) {  
  clock_configure(clk_sys,  
                  CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                  CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,  
                  hz, hz);  
}  
  
static bool __not_in_flash_func(wait_pll_lock)(void) {  
  uint32_t timeout = QUAD_LOCK_TIMEOUT;  
  while (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS) && --timeout) {}  
  return (pll_sys_hw->cs & PLL_CS_LOCK_BITS) != 0;  
}  
  
static bool __not_in_flash_func(pll_setup_bounded)(uint32_t refdiv,  
                                                   uint32_t fbdiv,  
                                                   uint32_t p1,  
                                                   uint32_t p2) {  
  hw_set_bits(&pll_sys_hw->pwr,  
              PLL_PWR_PD_BITS | PLL_PWR_VCOPD_BITS | PLL_PWR_POSTDIVPD_BITS);  
  hw_write_masked(&pll_sys_hw->cs,  
                  refdiv << PLL_CS_REFDIV_LSB,  
                  PLL_CS_REFDIV_BITS);  
  pll_sys_hw->fbdiv_int = fbdiv;  
  hw_clear_bits(&pll_sys_hw->pwr, PLL_PWR_PD_BITS | PLL_PWR_VCOPD_BITS);  
  
  if (!wait_pll_lock()) {  
    hw_set_bits(&pll_sys_hw->pwr,  
                PLL_PWR_PD_BITS | PLL_PWR_VCOPD_BITS | PLL_PWR_POSTDIVPD_BITS);  
    return false;  
  }  
  
  pll_sys_hw->prim = (p1 << PLL_PRIM_POSTDIV1_LSB) |  
                     (p2 << PLL_PRIM_POSTDIV2_LSB);  
  hw_clear_bits(&pll_sys_hw->pwr, PLL_PWR_POSTDIVPD_BITS);  
  return true;  
}  
  
// true: nominal profile acquired and clk_sys switched to PLL.  
// false: LOCK was not acquired; clk_sys stays on XOSC.  
// Парковка безусловная; внутри окна — только SRAM-стадии, без Serial.  
static bool __not_in_flash_func(rollback_nominal)(  
    const quad_vfo_pll_t *nominal, uint64_t xosc_hz) {  
  quad_vfo_pll_dbg_stage = 0;  
  rp2040.idleOtherCore();  
  quad_vfo_pll_dbg_stage = 1;  
  
  uint32_t ints = save_and_disable_interrupts();  
  clk_sys_to_xosc(xosc_hz);  
  quad_vfo_pll_dbg_stage = 2;  
  
  bool ok = pll_setup_bounded(nominal->refdiv, nominal->fbdiv,  
                              nominal->p1, nominal->p2);  
  quad_vfo_pll_dbg_stage = 3;  
  if (ok) {  
    clk_sys_to_pll(nominal->clk_sys_hz);  
    s_clk_sys_hz = nominal->clk_sys_hz;  
  } else {  
    s_clk_sys_hz = (uint32_t)xosc_hz;  
  }  
  quad_vfo_pll_dbg_stage = 4;  
  restore_interrupts(ints);  
  rp2040.resumeOtherCore();  
  quad_vfo_pll_dbg_stage = 5;  
  
  if (!ok) {  
    QP_LOG("[QUAD] rollback: номинальный PLL без LOCK — шина на XOSC!");  
  }  
  return ok;  
}  
  
bool __not_in_flash_func(quad_vfo_set_clk_sys)(  
    const quad_vfo_pll_t *cfg_in, uint32_t vsel,  
    const quad_vfo_pll_t *nominal, uint64_t xosc_hz,  
    bool *out_nominal_ok) {  
  
  if (out_nominal_ok) *out_nominal_ok = false;  
  if (!cfg_in || !nominal || xosc_hz == 0) return false;  
  
  quad_vfo_pll_t cfg = *cfg_in;  
  if (cfg.refdiv == 0) cfg.refdiv = 1;  
  // p1=1 недокументированно делит пополам на VCO < ~2.8 ГГц — отклоняем.  
  if (cfg.p1 < 2) cfg = *nominal;  
  
  if (cfg.p1 < 2 || cfg.p2 < 1 || cfg.refdiv < 1 ||  
      cfg.fbdiv < 16 || cfg.fbdiv > 320 || cfg.clk_sys_hz == 0) {  
    bool rb = rollback_nominal(nominal, xosc_hz);  
    if (out_nominal_ok) *out_nominal_ok = rb;  
    return false;  
  }  
  
  uint32_t target_clk_hz =  
      (uint32_t)(((uint64_t)cfg.fbdiv * xosc_hz) /  
                 ((uint64_t)cfg.refdiv * cfg.p1 * cfg.p2));  
  bool is_oc = (target_clk_hz > s_clk_sys_hz);  
  
  Serial.printf("[PLL] dbg: entry target=%lu oc=%d\n",  
                (unsigned long)target_clk_hz, (int)is_oc);  
  Serial.flush();  
  
  if (is_oc) {  
    vreg_set_voltage((vreg_voltage)vsel);  
    busy_wait_us(500);  
  }  
  
  // --- Парковочное окно: Serial запрещён до resumeOtherCore ---  
  Serial.println("[PLL] dbg: before park");  Serial.flush();  
  quad_vfo_pll_dbg_stage = 0;  
  
  rp2040.idleOtherCore();  
  quad_vfo_pll_dbg_stage = 1;  
  
  detach_peri();  
  uint32_t ints_status = save_and_disable_interrupts();  
  clk_sys_to_xosc(xosc_hz);  
  quad_vfo_pll_dbg_stage = 2;  
  
  if (!pll_setup_bounded(cfg.refdiv, cfg.fbdiv, cfg.p1, cfg.p2)) {  
    quad_vfo_pll_dbg_stage = 3;  
    restore_interrupts(ints_status);  
    rp2040.resumeOtherCore();  
    quad_vfo_pll_dbg_stage = 5;  
  
    Serial.println("[PLL] dbg: setup fail"); Serial.flush();  
    bool rb = rollback_nominal(nominal, xosc_hz);  
    if (out_nominal_ok) *out_nominal_ok = rb;  
    if (!is_oc) {  
      busy_wait_us(100);  
      vreg_set_voltage((vreg_voltage)vsel);  
    }  
    return false;  
  }  
  quad_vfo_pll_dbg_stage = 3;  
  
  const uint32_t FLASH_SAFE_MAX_HZ = 48000000u;  
  uint32_t ssi_baud =  
      (uint32_t)((target_clk_hz + FLASH_SAFE_MAX_HZ - 1) /  
                 FLASH_SAFE_MAX_HZ);  
  ssi_baud = (ssi_baud + 1u) & ~1u;  
  if (ssi_baud < 2u)  ssi_baud = 2u;  
  if (ssi_baud > 34u) ssi_baud = 34u;  
  ssi_hw->ssienr = 0;  
  ssi_hw->baudr  = ssi_baud;  
  ssi_hw->ssienr = 1;  
  
  clk_sys_to_pll(target_clk_hz);  
  quad_vfo_pll_dbg_stage = 4;  
  restore_interrupts(ints_status);  
  rp2040.resumeOtherCore();  
  quad_vfo_pll_dbg_stage = 5;  
  // --- Конец парковочного окна ---  
  
  Serial.println("[PLL] dbg: switched, verifying");  Serial.flush();  
  
  bool     locked_ok = false;  
  uint32_t meas_hz   = 0;  
  const uint64_t expect_hz = cfg.clk_sys_hz;  
  
  for (int attempt = 0; attempt < 3; attempt++) {  
    busy_wait_us(300);  
    meas_hz = meas_clk_sys_hz();  
    QP_LOG("[PLL] attempt %d: meas=%lu Hz expect=%llu Hz\n",  
                  attempt, (unsigned long)meas_hz,  
                  (unsigned long long)expect_hz);  
    if ((uint64_t)meas_hz * 1000ull > expect_hz * 997ull &&  
        (uint64_t)meas_hz * 1000ull < expect_hz * 1003ull) {  
      locked_ok = true;  
      break;  
    }  
  
    // Ретрай — отдельное парковочное окно, Serial тоже запрещён внутри.  
    Serial.printf("[PLL] dbg: retry %d\n", attempt);  Serial.flush();  
    quad_vfo_pll_dbg_stage = 0;  
  
    rp2040.idleOtherCore();  
    quad_vfo_pll_dbg_stage = 1;  
  
    uint32_t ints2 = save_and_disable_interrupts();  
    clk_sys_to_xosc(xosc_hz);  
    quad_vfo_pll_dbg_stage = 2;  
  
    if (!pll_setup_bounded(cfg.refdiv, cfg.fbdiv, cfg.p1, cfg.p2)) {  
      quad_vfo_pll_dbg_stage = 3;  
      restore_interrupts(ints2);  
      rp2040.resumeOtherCore();  
      quad_vfo_pll_dbg_stage = 5;  
      break;  
    }  
    quad_vfo_pll_dbg_stage = 3;  
  
    clk_sys_to_pll(target_clk_hz);  
    quad_vfo_pll_dbg_stage = 4;  
    restore_interrupts(ints2);  
    rp2040.resumeOtherCore();  
    quad_vfo_pll_dbg_stage = 5;  
  }  
  
  if (!locked_ok) {  
    Serial.println("[PLL] dbg: no lock -> rollback");  Serial.flush();  
    bool rb = rollback_nominal(nominal, xosc_hz);  
    if (out_nominal_ok) *out_nominal_ok = rb;  
    if (!is_oc) {  
      busy_wait_us(100);  
      vreg_set_voltage((vreg_voltage)vsel);  
    }  
    return false;  
  }  
  
  if (!is_oc) {  
    busy_wait_us(100);  
    vreg_set_voltage((vreg_voltage)vsel);  
  }  
  Serial.printf("[PLL] dbg: done meas=%lu\n", (unsigned long)meas_hz);  
  Serial.flush();  
  s_clk_sys_hz = meas_hz;  
  return true;  
}