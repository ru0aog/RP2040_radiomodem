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
// Парковка Core 0 на время переключения clk_sys — ОБЯЗАТЕЛЬНА:  
// без неё Core 0 не переживает смену шины (flash-код + XIP).  
//  
// Два механизма парковки:  
//   1. Флаг-протокол (вызов с Core 1): Core 0 находится в одном из  
//      wait-циклов (while(!core1_pll_done) в setup или trx_switch) и  
//      по core0_park_req уходит в core0_wait_in_ram(), отвечая ack.  
//   2. idleOtherCore — только для вызова с Core 0.  
//  
// ВНУТРИ парковочных окон Serial/printf ЗАПРЕЩЁН: CDC обслуживается  
// Core 0, printf при припаркованном ядре — дедлок. Диагностика —  
// только SRAM-флаги (quad_vfo_pll_dbg_stage). Исключение — timeout  
// в park_core0 (Core 0 ещё не припаркован).  
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
#define PARK_ACK_TIMEOUT  10000000u   // ~секунды итераций ожидания ack  
  
static uint32_t s_clk_sys_hz = 133000000;  
  
// Тихий режим — см. quad_vfo_set_quiet() в quad_vfo.cpp.  
static bool s_pll_quiet = false;  
#define QP_LOG(...)  do { if (!s_pll_quiet) Serial.printf(__VA_ARGS__); } while (0)  
void quad_vfo_pll_set_quiet(bool on) { s_pll_quiet = on; }  
  
// --- Протокол парковки Core 0 (объекты в src/trx/trx_control.cpp) ---  
extern volatile bool     core0_park_req;  
extern volatile bool     core0_park_ack;  
extern volatile uint32_t core0_park_enter;  
extern volatile uint32_t core0_park_exit;  
extern volatile uint32_t core0_loop_seen;  
  
// Диагностический счётчик стадий внутри парковочных окон (SRAM).  
//   0 — вне окна; 1 — после park; 2 — после clk_sys_to_xosc;  
//   3 — после pll_setup_bounded; 4 — после mux на PLL; 5 — после unpark  
volatile uint32_t quad_vfo_pll_dbg_stage = 0;  
  
// true → флаг-протокол; false → rp2040.idleOtherCore().  
static bool __not_in_flash_func(use_flag_park)(void) {  
  return (get_core_num() == 1);  
}  
  
// Парковка Core 0. true — ack получен, Core 0 в RAM-цикле.  
static bool __not_in_flash_func(park_core0)(bool flag_mode) {  
  if (flag_mode) {  
    core0_park_ack = false;             // гасим stale-ack прошлого сегмента  
    __dmb();  
    core0_park_req = true;  
    __dmb();  
    uint32_t spins = 0;  
    while (!core0_park_ack) {  
      if (++spins > PARK_ACK_TIMEOUT) {  
        // Core 0 ещё НЕ припаркован — Serial здесь безопасен.  
        Serial.printf("[PLL] PARK TIMEOUT req=%d ack=%d "  
                      "enter=%lu exit=%lu loop=%lu stage=%lu\n",  
                      (int)core0_park_req, (int)core0_park_ack,  
                      (unsigned long)core0_park_enter,  
                      (unsigned long)core0_park_exit,  
                      (unsigned long)core0_loop_seen,  
                      (unsigned long)quad_vfo_pll_dbg_stage);  
        Serial.flush();  
        core0_park_req = false;  
        __dmb();  
        __sev();  
        return false;  
      }  
    }  
    return true;  
  }  
  rp2040.idleOtherCore();  
  return true;  
}  
  
// Unpark. __sev() обязателен: core0_wait_in_ram() спит на __wfe().  
static void __not_in_flash_func(unpark_core0)(bool flag_mode) {  
  if (flag_mode) {  
    core0_park_req = false;  
    __dmb();  
    __sev();  
  } else {  
    rp2040.resumeOtherCore();  
  }  
}  
  
uint32_t quad_vfo_clk_sys_hz(void) { return s_clk_sys_hz; }  
  
// Измерение clk_sys через SDK-функцию: разрешение 1 кГц.  
static uint32_t __not_in_flash_func(meas_clk_sys_hz)(void) {  
  return frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) * 1000u;  
}  
  
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
  
// Голый mux clk_sys -> PLL_SYS, только регистры, без clock_configure.  
// AUXSRC нельзя менять, пока шина сидит на aux (не glitchless) —  
// поэтому: src -> clk_ref, ждём, меняем AUXSRC, src -> aux, ждём.  
static void __not_in_flash_func(clk_sys_src_to_pll)(void) {  
  // 1. src = clk_ref (glitchless)  
  hw_clear_bits(&clocks_hw->clk[clk_sys].ctrl, CLOCKS_CLK_SYS_CTRL_SRC_BITS);  
  while (!(clocks_hw->clk[clk_sys].selected & 1u)) {  
    tight_loop_contents();  
  }  
  // 2. делитель = 1  
  clocks_hw->clk[clk_sys].div = (1u << CLOCKS_CLK_SYS_DIV_INT_LSB) &  
                                CLOCKS_CLK_SYS_DIV_BITS;  
  __dmb();  
  // 3. auxsrc = PLL_SYS (шина сейчас не на aux — безопасно)  
  hw_write_masked(&clocks_hw->clk[clk_sys].ctrl,  
      (uint32_t)CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS  
          << CLOCKS_CLK_SYS_CTRL_AUXSRC_LSB,  
      CLOCKS_CLK_SYS_CTRL_AUXSRC_BITS);  
  // 4. src = aux  
  hw_set_bits(&clocks_hw->clk[clk_sys].ctrl, CLOCKS_CLK_SYS_CTRL_SRC_BITS);  
  while (!(clocks_hw->clk[clk_sys].selected & (1u << 1))) {  
    tight_loop_contents();  
  }  
}  

// Ждём LOCK PLL_SYS с ограничением по числу итераций (без SDK, без flash).  
// Таймаут считается итерациями, а не через time_us: шина в этот момент  
// на XOSC, и лишние обращения не нужны. QUAD_LOCK_TIMEOUT = 20000 итераций  
// на 12 МГц — это единицы мс, а реальный захват занимает десятки мкс.  
static bool __not_in_flash_func(wait_pll_lock)(void) {  
  for (uint32_t i = 0; i < QUAD_LOCK_TIMEOUT; ++i) {  
    if (pll_sys_hw->cs & PLL_CS_LOCK_BITS) return true;  
  }  
  return false;  
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
  
// true: номинальный профиль получен, clk_sys на PLL.  
// false: LOCK не получен; clk_sys остаётся на XOSC.  
static bool __not_in_flash_func(rollback_nominal)(  
    const quad_vfo_pll_t *nominal, uint64_t xosc_hz) {  
  quad_vfo_pll_dbg_stage = 0;  
  bool flag_mode = use_flag_park();  
  if (!park_core0(flag_mode)) {  
    QP_LOG("[QUAD] rollback: park timeout — откат отменён\n");  
    return false;  
  }  
  quad_vfo_pll_dbg_stage = 1;  
  
  uint32_t ints = save_and_disable_interrupts();  
  clk_sys_to_xosc(xosc_hz);  
  quad_vfo_pll_dbg_stage = 2;  
  
  bool ok = pll_setup_bounded(nominal->refdiv, nominal->fbdiv,  
                              nominal->p1, nominal->p2);  
  quad_vfo_pll_dbg_stage = 3;  
  if (ok) {  
    ssi_hw->baudr = 4;  
    __dmb();  
    clk_sys_src_to_pll();  
    s_clk_sys_hz = nominal->clk_sys_hz;  
  } else {  
    s_clk_sys_hz = (uint32_t)xosc_hz;  
  }  
  quad_vfo_pll_dbg_stage = 4;  
  restore_interrupts(ints);  
  unpark_core0(flag_mode);  
  quad_vfo_pll_dbg_stage = 5;  
  
  if (!ok) {  
    QP_LOG("[QUAD] rollback: номинальный PLL без LOCK — шина на XOSC!\n");  
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
  
  Serial.printf("[PLL] dbg: entry target=%lu oc=%d flag=%d\n",  
                (unsigned long)target_clk_hz, (int)is_oc,  
                (int)use_flag_park());  
  Serial.flush();  
  
  if (is_oc) {  
    vreg_set_voltage((vreg_voltage)vsel);  
    busy_wait_us(500);  
  }  
  
  Serial.println("[PLL] dbg: before park");  Serial.flush();  
  quad_vfo_pll_dbg_stage = 0;  
  
  bool flag_mode = use_flag_park();  
  
  // Сегмент 1: detach_peri + xosc  
  if (!park_core0(flag_mode)) return false;  
  quad_vfo_pll_dbg_stage = 1;  
  detach_peri();  
  uint32_t ints_status = save_and_disable_interrupts();  
  clk_sys_to_xosc(xosc_hz);  
  restore_interrupts(ints_status);  
  unpark_core0(flag_mode);  
  quad_vfo_pll_dbg_stage = 2;  
  Serial.println("[PLL] dbg: seg1 xosc ok");  Serial.flush();  
  
  // Сегмент 2: PLL setup  
  if (!park_core0(flag_mode)) return false;  
  quad_vfo_pll_dbg_stage = 2;  
  ints_status = save_and_disable_interrupts();  
  bool pll_ok = pll_setup_bounded(cfg.refdiv, cfg.fbdiv, cfg.p1, cfg.p2);  
  restore_interrupts(ints_status);  
  unpark_core0(flag_mode);  
  quad_vfo_pll_dbg_stage = 3;  
  Serial.printf("[PLL] dbg: seg2 pll ok=%d\n", (int)pll_ok);  Serial.flush();  
  if (!pll_ok) {  
    bool rb = rollback_nominal(nominal, xosc_hz);  
    if (out_nominal_ok) *out_nominal_ok = rb;  
    return false;  
  }  
  
  // Сегмент 3: clk_sys -> PLL (голый mux). Serial — только ДО park.  
  Serial.println("[PLL] dbg: before park3");  Serial.flush();  
  if (!park_core0(flag_mode)) return false;  
  quad_vfo_pll_dbg_stage = 3;  
  ints_status = save_and_disable_interrupts();  
  ssi_hw->baudr = 4;                    // XIP ≤ 84 МГц на 337.5 МГц  
  __dmb();  
  clk_sys_src_to_pll();  
  quad_vfo_pll_dbg_stage = 4;  
  restore_interrupts(ints_status);  
  unpark_core0(flag_mode);  
  quad_vfo_pll_dbg_stage = 5;  
  Serial.println("[PLL] dbg: switched");  Serial.flush();  
  
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
  
    // Ретрай — отдельное парковочное окно, Serial внутри запрещён.  
    Serial.printf("[PLL] dbg: retry %d\n", attempt);  Serial.flush();  
    quad_vfo_pll_dbg_stage = 0;  
  
    bool flag_mode2 = use_flag_park();  
    if (!park_core0(flag_mode2)) {  
      Serial.printf("[PLL] dbg: retry %d park timeout\n", attempt);  
      Serial.flush();  
      break;  
    }  
    quad_vfo_pll_dbg_stage = 1;  
  
    uint32_t ints2 = save_and_disable_interrupts();  
    clk_sys_to_xosc(xosc_hz);  
    quad_vfo_pll_dbg_stage = 2;  
  
    if (!pll_setup_bounded(cfg.refdiv, cfg.fbdiv, cfg.p1, cfg.p2)) {  
      quad_vfo_pll_dbg_stage = 3;  
      restore_interrupts(ints2);  
      unpark_core0(flag_mode2);  
      quad_vfo_pll_dbg_stage = 5;  
      break;  
    }  
    quad_vfo_pll_dbg_stage = 3;  
  
    ssi_hw->baudr = 4;  
    __dmb();  
    clk_sys_src_to_pll();  
    quad_vfo_pll_dbg_stage = 4;  
    restore_interrupts(ints2);  
    unpark_core0(flag_mode2);  
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