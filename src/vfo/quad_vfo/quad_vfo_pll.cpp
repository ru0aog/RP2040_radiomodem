// quad_vfo_pll.cpp  
// Порт PLL-подсистемы из src/vfo/soft_tx.cpp (RU0AOG) + правки аудита.  
//  
// PLL настраивается своим кодом, без pll_init() SDK (тот ждёт LOCK бесконечно).  
// clk_sys переключается напрямую через регистры, без clock_configure.  
//  
// Парковка Core 0 на время смены clk_sys обязательна.  
//   Core 1: флаг-протокол с номером поколения (core0_park_*, trx_control.cpp).  
//   Core 0: rp2040.idleOtherCore() — запасной путь.  
// Внутри окон и МЕЖДУ ними нет Serial: печатаем только после всех окон.  
#include "quad_vfo_pll.h"  
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
#define PARK_TIMEOUT_US     1000000u  
#define FLASH_SAFE_MAX_HZ   48000000u  
// Запасной счётный таймаут парковки — time_us_32() может встать  
// при перестройке часов, и ограниченное ожидание станет вечным.  
#define PARK_SPIN_LIMIT     4000000u
  
extern volatile uint32_t core0_park_req;  
extern volatile uint32_t core0_park_ack;  
extern volatile uint32_t core0_park_inside;  
extern volatile uint32_t core0_park_enter;  
extern volatile uint32_t core0_park_exit;  
extern volatile uint32_t core0_loop_seen;  
  
volatile uint32_t quad_vfo_pll_dbg_stage = 0;  
  
static uint32_t s_clk_sys_hz = 133000000;  
static uint32_t s_park_gen   = 0;  
static bool     s_park_timeout = false;   // печатаем после окон  
  
static bool s_pll_quiet = false;  
#define QP_LOG(...)  do { if (!s_pll_quiet) Serial.printf(__VA_ARGS__); } while (0)  
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
// Парковка  
// ---------------------------------------------------------------------------  
static inline bool use_flag_park(void) { return get_core_num() == 1; }  
  
static bool __not_in_flash_func(park_core0)(bool flag_mode) {  
  if (!flag_mode) { rp2040.idleOtherCore(); return true; }  
  
  uint32_t t0 = time_us_32();  
  uint32_t spins = PARK_SPIN_LIMIT;        // запасной потолок без таймера  
  // 1. Дождаться выхода Core 0 из прошлого окна.  
  while (core0_park_inside) {  
    if ((time_us_32() - t0 > PARK_TIMEOUT_US) || --spins == 0) {  
      s_park_timeout = true; return false;  
    }  
  }
  // 2. Новое поколение (0 зарезервирован).  
  if (++s_park_gen == 0) s_park_gen = 1;  
  uint32_t gen = s_park_gen;  
  __dmb();  
  core0_park_req = gen;  
  __dmb();  
  __sev();  
  // 3. Ждать подтверждения именно этого поколения.  
  spins = PARK_SPIN_LIMIT;  
  while (core0_park_ack != gen) {  
    if ((time_us_32() - t0 > PARK_TIMEOUT_US) || --spins == 0) {  
      core0_park_req = 0;  
      __dmb();  
      __sev();  
      s_park_timeout = true;  
      return false;  
    }  
  }
  return true;  
}  
  
static void __not_in_flash_func(unpark_core0)(bool flag_mode) {  
  if (!flag_mode) { rp2040.resumeOtherCore(); return; }  
  core0_park_req = 0;  
  __dmb();  
  __sev();  
}  
  
static void dump_park_timeout(void) {  
  if (!s_park_timeout) return;  
  s_park_timeout = false;  
  Serial.printf("[PLL] PARK TIMEOUT gen=%lu req=%lu ack=%lu inside=%lu "  
                "enter=%lu exit=%lu loop=%lu stage=%lu\n",  
                (unsigned long)s_park_gen, (unsigned long)core0_park_req,  
                (unsigned long)core0_park_ack, (unsigned long)core0_park_inside,  
                (unsigned long)core0_park_enter, (unsigned long)core0_park_exit,  
                (unsigned long)core0_loop_seen,  
                (unsigned long)quad_vfo_pll_dbg_stage);  
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
  
// Три окна: XOSC -> PLL setup -> mux на PLL. 0 = ок, 1/2/3 = отказ парковки  
// в этом сегменте, 10 = нет LOCK (шина на XOSC).  
static int __not_in_flash_func(switch_sequence)(const quad_vfo_pll_t *c,  
                                                uint32_t hz, bool fm) {  
  uint32_t ints;  
  
  if (!park_core0(fm)) return 1;  
  quad_vfo_pll_dbg_stage = 1;  
  ints = save_and_disable_interrupts();  
  bool mux_ok = clk_sys_to_xosc();  
  restore_interrupts(ints);  
  unpark_core0(fm);  
  if (!mux_ok) return 11;            // clk_ref не подтвердил selected
  
  if (!park_core0(fm)) return 2;  
  quad_vfo_pll_dbg_stage = 2;  
  ints = save_and_disable_interrupts();  
  bool ok = pll_setup_bounded(c->refdiv, c->fbdiv, c->p1, c->p2);  
  restore_interrupts(ints);  
  unpark_core0(fm);  
  if (!ok) return 10;  
  
if (!park_core0(fm)) return 3;  
  quad_vfo_pll_dbg_stage = 3;  
  ints = save_and_disable_interrupts();  
  ssi_set_baud_for(hz);              // ДО подъёма частоты  
  quad_vfo_pll_dbg_stage = 4;  
  bool pll_mux_ok = clk_sys_to_pll();  
  restore_interrupts(ints);  
  unpark_core0(fm);  
  if (!pll_mux_ok) return 13;        // aux/PLL не подтвердил selected  
  quad_vfo_pll_dbg_stage = 5;  
  return 0;
}  
  
static bool rollback_nominal(const quad_vfo_pll_t *nominal, uint64_t xosc_hz) {  
  int r = switch_sequence(nominal, nominal->clk_sys_hz, use_flag_park());  
  dump_park_timeout();  
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
  
  quad_vfo_pll_t cfg = *cfg_in;  
  if (cfg.refdiv == 0) cfg.refdiv = 1;  
  if (cfg.p1 < 2) cfg = *nominal;  
  if (cfg.p1 < 2 || cfg.p2 < 1 || cfg.refdiv < 1 ||  
      cfg.fbdiv < 16 || cfg.fbdiv > 320 || cfg.clk_sys_hz == 0) {  
    bool rb = rollback_nominal(nominal, xosc_hz);  
    if (out_nominal_ok) *out_nominal_ok = rb;  
    return false;  
  }  
  
  uint32_t target_clk_hz = (uint32_t)(((uint64_t)cfg.fbdiv * xosc_hz) /  
                           ((uint64_t)cfg.refdiv * cfg.p1 * cfg.p2));  
  // Реальную частоту меряем — кэш s_clk_sys_hz рассинхронизирован  
  // с soft_tx_set_clk_sys() (у того свой кэш current_clk_sys_hz).  
  uint32_t cur_hz = meas_clk_sys_hz();  
  bool is_oc = (target_clk_hz > cur_hz); 
  bool fm    = use_flag_park();  
  
  Serial.printf("[PLL] dbg: entry target=%lu oc=%d flag=%d\n",  
                (unsigned long)target_clk_hz, (int)is_oc, (int)fm);  
  Serial.flush();                    // последний flush ДО окон  
  
  // Поднимаем VREG, если переход вверх ИЛИ цель выше номинала 133 МГц —  
  // тогда запас по напряжению нужен в любом случае.  
  if (is_oc || target_clk_hz > 133000000u) {  
    vreg_set_voltage((vreg_voltage)vsel);  
    busy_wait_us(500);  
  }  
  detach_peri();
  quad_vfo_pll_dbg_stage = 0;  
  
  int r = switch_sequence(&cfg, target_clk_hz, fm);  
  dump_park_timeout();  
  Serial.printf("[PLL] dbg: seq r=%d stage=%lu\n", r,  
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
      r = switch_sequence(&cfg, target_clk_hz, fm);  
      dump_park_timeout();  
      if (r != 0) break;  
    }  
  }  
  
  if (!locked_ok) {  
    Serial.println("[PLL] dbg: no lock -> rollback");  
    bool rb = rollback_nominal(nominal, xosc_hz);  
    if (out_nominal_ok) *out_nominal_ok = rb;  
    if (!is_oc) { busy_wait_us(100); vreg_set_voltage((vreg_voltage)vsel); }  
    return false;  
  }  
  
  if (!is_oc) { busy_wait_us(100); vreg_set_voltage((vreg_voltage)vsel); }  
  Serial.printf("[PLL] dbg: done meas=%lu\n", (unsigned long)meas_hz);  
  s_clk_sys_hz = meas_hz;  
  return true;  
}