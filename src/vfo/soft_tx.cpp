// src/vfo/soft_tx.cpp  
// ============================================================================  
// Дизеринг-движок программного DDS VFO (передача): PIO-меандр + MASH-1-1 +  
// FRAC TPDF-дизер + многокритериальный автотюн PLL.  
// Портировано из RP2040_beacon/vfo_hardware.cpp v2.23 (GPLv3, RU0AOG).  
//  
// АДАПТАЦИЯ под архитектуру RP2040_radiomodem:  
//  - вместо выделенного ядра: кооперативный soft_tx_poll() из loop1()  
//    (одна итерация = одна запись clkdiv; флаги опрашиваются каждый 64-й);  
//  - таблица тонов заполняется из VfoBank менеджера (vfo_bank());  
//  - spinlock не нужен: применение параметров — внутри poll на Core 1,  
//    запись — из vfo_tone() на Core 0 через volatile-флаги;  
//  - multicore_reset_core1()/launch убраны — core1 теперь общий с RX.  
// ============================================================================  
#include "soft_tx.h"  
#include "vfo.h"  
#include "../pins.h"  
#include "../mem/mem_arena.h"  
#include "../trx/trx_control.h"  
#include "hardware/pio.h"  
#include "hardware/clocks.h"  
#include "hardware/pll.h"  
#include "hardware/sync.h"  
#include "hardware/timer.h"  
#include "hardware/vreg.h"  
#include "hardware/adc.h"  
#include "hardware/divider.h"  
#include "hardware/structs/sio.h"  
#include "hardware/structs/ssi.h"  
#include "hardware/structs/pll.h"  
#include "pico/multicore.h"  
#include <math.h>  
  
// ---- Диагностика ----  
#define BOOST_DEBUG       0  
#define VFO_PLL_DEBUG     0  
#define PLL_RUNTIME_DEBUG 0  
  
// ============================================================================  
// ЛИМИТЫ И ПРОФИЛИ ТАКТИРОВАНИЯ  
// ============================================================================  
#define SOFT_TX_CLK_SYS_MIN   48000000ULL  
#define SOFT_TX_CLK_SYS_MAX  420000000ULL   // абсолютный барьер clk_sys  
#define SOFT_TX_PLL_MAX     4300000000ULL   // абсолютный барьер VCO  
#define VFO_CLK_SYS_NOMINAL_HZ 133000000ULL  
#define VFO_CLK_SYS_PREF_MIN_HZ 250000000ULL  
#define VFO_PREFERRED_MIN_CLK   320000000ULL  
#define VFO_THROTTLE_HI_C 80.0f  
#define VFO_THROTTLE_LO_C 60.0f  
  
// Константы метрики (как в исходнике)  
#define VFO_INT_PENALTY        500000000ULL  
#define VFO_SPUR_MIN_OFFSET_HZ 1000000ULL  
#define VFO_DMIN_CEILING       280000000ULL  
#define VFO_K8_BELT            (1u << 25)  
#define VFO_SHELF_REF_CLK_HZ   400000000ULL  
#define VFO_SPUR_HORIZON_HZ    3000000ULL  
#define VFO_SIMPLE_FRAC_INT_MAX 50  
  
// Лимиты из Flash-паспорта (OCTEST). До интеграции file_manager — дефолты.  
uint32_t vfo_max_clk_sys_hz = 133000000;  
uint32_t vfo_max_pll_vco_hz = 1600000000;  
  
static inline uint64_t effective_clk_max() {  
  uint64_t l = vfo_max_clk_sys_hz;  
  return (l < SOFT_TX_CLK_SYS_MAX) ? l : SOFT_TX_CLK_SYS_MAX;  
}  
static inline uint64_t effective_pll_max() {  
  uint64_t l = vfo_max_pll_vco_hz;  
  return (l < SOFT_TX_PLL_MAX) ? l : SOFT_TX_PLL_MAX;  
}  
  
// PllConfig: {fbdiv, p1, p2, clk_sys_hz, vsel, is_oc, refdiv}  
PllConfig pll_nominal   = { 133, 6, 2, 133000000ULL, (uint32_t)VREG_VOLTAGE_DEFAULT, false, 1 };  
PllConfig pll_overclock = {  69, 3, 2, 138004025ULL, (uint32_t)VREG_VOLTAGE_DEFAULT, false, 1 };  
PllConfig pll_ceiling   = {  95, 3, 1, 380011083ULL, (uint32_t)VREG_VOLTAGE_1_30,    true,  1 };  
  
volatile bool clk_boosted    = false;  
volatile bool vfo_auto_boost = false;  
  
// ============================================================================  
// PIO И ТАБЛИЦА ТОНОВ  
// ============================================================================  
static const uint16_t pio_square_instructions[] = { 0xe001, 0xe000 };  
static const pio_program_t pio_square_program = {  
  .instructions = pio_square_instructions, .length = 2, .origin = -1, .pio_version = 0  
};  
  
// Таблица делителей в RAM (.time_critical — гарантированно в SRAM)  
SoftTxParams __attribute__((section(".time_critical.soft_tx_tones")))  
             s_tones[SOFT_TX_TONES];  
  
static PIO  lo_pio = pio0;  
static unsigned int lo_sm = 0;  
static unsigned int lo_offset = 0;  
static bool pio_program_loaded = false;  
static bool s_running = false;  
  
static uint8_t  current_active_tone = VFO_TONE_NONE;  
static volatile bool tone_changed = false;  
static volatile bool dither_reset_pending = false;  
  
// Параметры активного тона (volatile: писал Core 0 через tone-флаг)  
static volatile uint32_t dds_step = 0;  
static volatile uint32_t target_pio_int = 8;  
static volatile uint32_t target_pio_frac8 = 0;  
static volatile uint32_t frac_dither_mask = 0x03u;  
volatile uint8_t soft_tx_narrowband = 0;  
  
static uint32_t current_clk_sys_hz = 133000000;  
static bool thermal_throttled = false;  
  
// ============================================================================  
// ВСПОМОГАТЕЛЬНЫЕ  
// ============================================================================  
static vreg_voltage vsel_for(uint64_t clk_hz) {  
  if (clk_hz >= 250000000ULL)      return VREG_VOLTAGE_1_30;  
  else if (clk_hz >= 200000000ULL) return VREG_VOLTAGE_1_25;  
  else if (clk_hz > 133000000ULL)  return VREG_VOLTAGE_1_15;  
  return VREG_VOLTAGE_DEFAULT;  
}  
  
static uint16_t vsel_to_mv(uint32_t vsel) {  
  return (uint16_t)(900 + (vsel - VREG_VOLTAGE_0_90) * 50);  
}  
  
float soft_tx_core_temp_c(void) {  
  if (trx_state() == TRX_RX) return NAN;   // ADC занят приёмным трактом  
  clock_configure(clk_adc,  
                  CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,  
                  0, 48000000u, 48000000u);  
  adc_select_input(4);  
  uint32_t samples[3];  
  for (int i = 0; i < 3; i++) { samples[i] = adc_read(); busy_wait_us(10); }  
  uint32_t raw;  
  if ((samples[0] <= samples[1] && samples[1] <= samples[2]) ||  
      (samples[2] <= samples[1] && samples[1] <= samples[0])) raw = samples[1];  
  else if ((samples[1] <= samples[0] && samples[0] <= samples[2]) ||  
           (samples[2] <= samples[0] && samples[0] <= samples[1])) raw = samples[0];  
  else raw = samples[2];  
  float v_bead = raw * 3.3f / 4095.0f;  
  return 27.0f - (v_bead - 0.706f) / 0.001721f;  
}  
  
// ============================================================================  
// РАСЧЁТ ДЕЛИТЕЛЯ PIO Q8.8 + DDS-ОСТАТОК (calculate_raw_params_mhz исходника)  
// ============================================================================  
static SoftTxParams calc_params(uint64_t clk_sys_hz, uint64_t mhz_target) {  
  if (mhz_target < 100000000ULL)   mhz_target = 100000000ULL;    // >= 0.1 MHz  
  if (mhz_target > 40000000000ULL) mhz_target = 40000000000ULL;  // <= 40 MHz  
  
  uint64_t vfo_denom = mhz_target * 2ULL;             // 2 такта/период  
  uint64_t pio_div_fixed8 = ((clk_sys_hz * 256ULL) * 1000ULL) / vfo_denom;  
  uint8_t frac8 = (uint8_t)(pio_div_fixed8 & 0xFFu);  
  
  SoftTxParams p;  
  p.pio_int         = pio_div_fixed8 >> 8;  
  p.pio_frac        = frac8;  
  p.target_freq_chz = (uint32_t)(mhz_target / 10ULL);  
  if (p.pio_int < 2) { p.pio_int = 2; p.pio_frac = 0; }  
  
  uint64_t rem           = ((clk_sys_hz * 256ULL) * 1000ULL) % vfo_denom;  
  uint64_t intermediate  = (rem << 16) / vfo_denom;  
  uint64_t remainder_low = (rem << 16) % vfo_denom;  
  p.dds_step = (uint32_t)((intermediate << 16) + ((remainder_low << 16) / vfo_denom));  
  if (p.dds_step != 0) p.dds_step |= 1u;   // gcd(step,2^32)=1 → шумовая полка  
  return p;  
}  
  
static void rebuild_table() {  
  const VfoBank *b = vfo_bank();  
  for (uint8_t i = 0; i < b->count && i < SOFT_TX_TONES; i++)  
    s_tones[i] = calc_params(current_clk_sys_hz, b->freqs_mhz[i]);  
  current_active_tone = VFO_TONE_NONE;  
}  
  
// ============================================================================  
// МЕТРИКА СПУРОВ (vfo_pll_metric_verbose исходника)  
// ============================================================================  
struct MetricBreakdown {  
  uint32_t raw_step, frac, ctz_val;  
  uint64_t ctz_penalty, center_dist, mash_penalty, clk_penalty,  
           int_penalty, spur_power_pen, total_metric;  
};  
  
static uint32_t dds_step_min_dist_to_k8(uint32_t step) {  
  uint32_t best = (1u << 28);  
  for (uint32_t k = 0; k < 8; k++) {  
    uint32_t a = k << 29;  
    uint32_t d = step - a;  
    if (d > 0x80000000u) d = 0u - d;  
    if (d < best) best = d;  
  }  
  return best;  
}  
  
static uint64_t frac_spur_power_pen(uint16_t pio_int, uint8_t pio_frac) {  
  if (pio_frac == 0) return 0;  
  uint64_t D_x256 = (uint64_t)pio_int * 256ULL + pio_frac;  
  uint64_t A      = (uint64_t)pio_frac * (256u - pio_frac);  
  return ((uint64_t)A << 46) / (D_x256 * D_x256);  
}  
  
static uint64_t frac_spur_min_off_hz(uint16_t pio_int, uint8_t pio_frac,  
                                     uint64_t clk_sys_hz) {  
  if (pio_frac == 0) return UINT64_MAX;  
  uint64_t div_x256 = (uint64_t)pio_int * 256ULL + pio_frac;  
  if (div_x256 == 0) return UINT64_MAX;  
  uint64_t best = UINT64_MAX;  
  for (uint32_t k = 1; k <= 4; k++) {  
    uint32_t m  = ((uint32_t)pio_frac * k) & 0xFFu;  
    uint32_t fd = (m < 256u - m) ? m : 256u - m;  
    if (fd == 0) continue;  
    uint64_t off = clk_sys_hz * fd / div_x256;  
    if (off < best) best = off;  
  }  
  return best;  
}  
  
static float frac_spur_level_db(uint16_t pio_int, uint8_t pio_frac) {  
  if (pio_frac == 0 || pio_int == 0) return -999.0f;  
  float D  = (float)pio_int + (float)pio_frac / 256.0f;  
  float f  = (float)pio_frac;  
  float p2 = f * (256.0f - f);  
  float lvl = 10.0f * log10f(p2 / (32.0f * 9.8696f * D * D));  
  return lvl - 21.0f;   // откалиброванная поправка  
}  
  
static uint64_t pll_metric_verbose(uint64_t clk_sys_hz, uint64_t target_mhz,  
                                   MetricBreakdown *b, SoftTxParams *out) {  
  SoftTxParams test = calc_params(clk_sys_hz, target_mhz);  
  if (out) *out = test;  
  memset(b, 0, sizeof(*b));  
  b->frac = test.pio_frac;  
  b->raw_step = test.dds_step;  
  
  if (test.pio_frac == 0 && test.dds_step == 0) return 0;  
  
  uint64_t spur_off = frac_spur_min_off_hz(test.pio_int, test.pio_frac, clk_sys_hz);  
  
  b->spur_power_pen = frac_spur_power_pen(test.pio_int, test.pio_frac);  
  if (spur_off != UINT64_MAX && spur_off < VFO_SPUR_HORIZON_HZ) {  
    uint64_t prox   = VFO_SPUR_HORIZON_HZ - spur_off;  
    uint64_t prox_q = prox / 1024ULL;  
    b->spur_power_pen += (prox_q * prox_q * prox_q) << 4;  
  }  
  b->total_metric += b->spur_power_pen;  
  
  if (test.pio_frac == 0) {  
    uint64_t act = (test.dds_step <= (0xFFFFFFFFu - test.dds_step))  
                   ? (uint64_t)test.dds_step  
                   : (0xFFFFFFFFu - (uint64_t)test.dds_step);  
    b->total_metric += (act >> 4) + (1ULL << 36);  
  }  
  
  {  
    uint32_t f = test.pio_frac;  
    uint32_t ctz = (f == 0) ? 8 : (uint32_t)__builtin_ctz(f);  
    b->ctz_val = ctz;  
    if (ctz >= 5) b->ctz_penalty = (1ULL << (ctz + 26)) + (1ULL << 33);  
    b->total_metric += b->ctz_penalty;  
  }  
  
  {  
    const uint64_t DEN_REF = 400000000ULL * 55ULL;  
    uint64_t dens = clk_sys_hz * (uint64_t)test.pio_int;  
    b->clk_penalty = (dens < DEN_REF)  
                     ? ((DEN_REF - dens) * (1ULL << 30)) / DEN_REF : 0;  
    b->total_metric += b->clk_penalty;  
  }  
  
  if (test.dds_step != 0) {  
    uint32_t dist_k8 = dds_step_min_dist_to_k8(test.dds_step);  
    b->center_dist = dist_k8;  
    if (dist_k8 < VFO_K8_BELT) {  
      uint64_t deficit = (uint64_t)(VFO_K8_BELT - dist_k8);  
      b->mash_penalty = (deficit * deficit) >> 14;  
    }  
    b->total_metric += b->mash_penalty;  
  }  
  
  if (test.pio_int < 4) { b->int_penalty = VFO_INT_PENALTY; b->total_metric += b->int_penalty; }  
  
  if (test.pio_frac != 0 && test.pio_int != 0) {  
    float lvl_db = frac_spur_level_db(test.pio_int, test.pio_frac);  
    if (lvl_db > -30.0f) {  
      float over = lvl_db + 30.0f;  
      uint64_t base = (uint64_t)(over * over) << 22;  
      float kdist;  
      if      (spur_off == UINT64_MAX)          kdist = 1.0f;  
      else if (spur_off >= VFO_SPUR_HORIZON_HZ) kdist = 0.1f;  
      else kdist = 1.0f - 0.9f * ((float)spur_off / (float)VFO_SPUR_HORIZON_HZ);  
      uint64_t pen = (uint64_t)((double)base * kdist);  
      b->spur_power_pen += pen;  
      b->total_metric   += pen;  
    }  
  }  
  return b->total_metric;  
}  
  
static uint64_t pll_metric(uint64_t clk_sys_hz, uint64_t target_mhz, SoftTxParams *out) {  
  MetricBreakdown b;  
  return pll_metric_verbose(clk_sys_hz, target_mhz, &b, out);  
}  
  
// ============================================================================  
// МАТРИЧНЫЙ СКАН PLL (vfo_find_optimal_pll исходника, debug-печать убрана)  
// ============================================================================  
static PllConfig find_optimal_pll(uint32_t target_frequency_hz, uint64_t max_clk_limit) {  
  uint64_t crystal_hz = vfo_xosc_hz();  
  uint64_t target_mhz = (uint64_t)target_frequency_hz * 1000ULL;  
  uint64_t clk_limit  = (max_clk_limit == 0) ? effective_clk_max() : max_clk_limit;  
  
  PllConfig best_pll = { 133, 6, 2, 133000000ULL, (uint32_t)VREG_VOLTAGE_DEFAULT, false, 1 };  
  
  if (target_mhz < 1000000000ULL)  target_mhz = 1000000000ULL;   // 1.0 MHz  
  if (target_mhz > 30000000000ULL) target_mhz = 30000000000ULL;  // 30 MHz  
  
  const uint64_t min_allowed_clk = SOFT_TX_CLK_SYS_MIN;  
  const uint64_t max_allowed_clk = clk_limit;  
  
  uint64_t min_dds_metric = UINT64_MAX;  
  uint32_t n_arbit        = 0;  
  
  for (uint32_t refdiv = 1; refdiv <= 3; refdiv++) {  
    uint64_t ref_hz = crystal_hz / refdiv;  
    for (uint32_t p1 = 2; p1 <= 7; p1++) {  
      for (uint32_t p2 = 1; p2 <= 7; p2++) {  
        if (p1 < p2) continue;  
        uint32_t pdiv_total = p1 * p2;  
        for (uint32_t fbdiv = 16; fbdiv <= 320; fbdiv++) {  
          uint64_t vco_hz = ref_hz * fbdiv;  
          if (vco_hz < 750000000ULL) continue;  
          if (vco_hz > effective_pll_max()) continue;  
  
          uint64_t clk_sys_hz = vco_hz / (uint64_t)pdiv_total;  
          if (clk_sys_hz < min_allowed_clk || clk_sys_hz > clk_limit) continue;  
  
          SoftTxParams test;  
          MetricBreakdown brk;  
          uint64_t metric = pll_metric_verbose(clk_sys_hz, target_mhz, &brk, &test);  
          if (test.pio_int < 2) continue;  
  
          // Адаптивный гейт плотности коррекции clk·int  
          {  
            uint64_t f_out_hz = target_mhz / 1000ULL;  
            uint64_t prod_max = (max_allowed_clk * max_allowed_clk) / (2ULL * f_out_hz);  
            uint64_t gate = 4000000000ULL;  
            uint64_t cap  = (prod_max * 9ULL) / 10ULL;  
            if (gate > cap) gate = cap;  
            if (clk_sys_hz * (uint64_t)test.pio_int < gate) continue;  
          }  
  
          uint64_t spur_off_hz = frac_spur_min_off_hz(test.pio_int, test.pio_frac, clk_sys_hz);  
  
          bool simple_frac = false;  
          if (test.pio_frac != 0u && test.pio_int <= VFO_SIMPLE_FRAC_INT_MAX) {  
            uint32_t danger_zone = (test.pio_int <= 14) ? 16u : 8u;  
            for (uint32_t n = 2; n <= 4 && !simple_frac; n++) {  
              for (uint32_t k = 1; k < n; k++) {  
                uint32_t anchor = (k * 256u) / n;  
                uint32_t d = (test.pio_frac > anchor) ? test.pio_frac - anchor  
                                                      : anchor - test.pio_frac;  
                if (d < danger_zone) { simple_frac = true; break; }  
              }  
            }  
          }  
  
          bool frac_forbidden = (test.pio_frac != 0u) &&  
                                ((spur_off_hz < VFO_SPUR_MIN_OFFSET_HZ) || simple_frac);  
          n_arbit++;  
  
          uint64_t arb_metric = metric;  
          if (frac_forbidden) arb_metric += (1ULL << 40);  
          if (test.pio_frac == 0u && test.dds_step != 0u)  
            arb_metric += (1ULL << 42);  
  
          bool prefer = (arb_metric < min_dds_metric) ||  
                        (arb_metric == min_dds_metric && clk_sys_hz > best_pll.clk_sys_hz);  
          if (prefer) {  
            min_dds_metric      = arb_metric;  
            best_pll.fbdiv      = fbdiv;  
            best_pll.p1         = p1;  
            best_pll.p2         = p2;  
            best_pll.refdiv     = refdiv;  
            best_pll.clk_sys_hz = clk_sys_hz;  
            best_pll.vsel       = (uint32_t)vsel_for(clk_sys_hz);  
            best_pll.is_oc      = (clk_sys_hz > VFO_CLK_SYS_NOMINAL_HZ);  
          }  
        }  
      }  
    }  
  }  
  
  // Защита от пустого скана: встаём на max доказанный flash-clk  
  if (n_arbit == 0) {  
    uint64_t fb_clk_target = effective_clk_max();  
    uint64_t best_fb_clk   = 0;  
    PllConfig fb_pll       = best_pll;  
    for (uint32_t refdiv = 1; refdiv <= 3; refdiv++) {  
      uint64_t ref_hz = crystal_hz / refdiv;  
      for (uint32_t p1 = 2; p1 <= 7; p1++) {  
        for (uint32_t p2 = 1; p2 <= 7; p2++) {  
          if (p1 < p2) continue;  
          uint32_t pdiv_total = p1 * p2;  
          for (uint32_t fbdiv = 16; fbdiv <= 320; fbdiv++) {  
            uint64_t vco_hz = ref_hz * fbdiv;  
            if (vco_hz < 750000000ULL) continue;  
            if (vco_hz > effective_pll_max()) continue;  
            uint64_t clk_sys_hz = vco_hz / (uint64_t)pdiv_total;  
            if (clk_sys_hz < min_allowed_clk) continue;  
            if (clk_sys_hz > fb_clk_target)   continue;  
            if (clk_sys_hz > best_fb_clk) {  
              best_fb_clk        = clk_sys_hz;  
              fb_pll.fbdiv       = fbdiv;  
              fb_pll.p1          = p1;  
              fb_pll.p2          = p2;  
              fb_pll.refdiv      = refdiv;  
              fb_pll.clk_sys_hz  = clk_sys_hz;  
              fb_pll.vsel        = (uint32_t)vsel_for(clk_sys_hz);  
              fb_pll.is_oc       = (clk_sys_hz > VFO_CLK_SYS_NOMINAL_HZ);  
            }  
          }  
        }  
      }  
    }  
    return (best_fb_clk > 0) ? fb_pll : best_pll;  
  }  
  return best_pll;  
}  
  
// ============================================================================  
// ПЕРЕКЛЮЧЕНИЕ clk_sys (vfo_set_clk_sys исходника — в RAM, под маской IRQ)  
// ============================================================================  
static void __not_in_flash_func(detach_peripheral_clock)() {  
  clock_configure(clk_peri, 0,  
                  CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,  
                  48 * 1000000, 48 * 1000000);  
}  
  
static void __not_in_flash_func(set_clk_sys)(const PllConfig& cfg_in, uint32_t vsel) {  
  PllConfig cfg = cfg_in;  
  if (cfg.refdiv == 0) cfg.refdiv = 1;  
  if (cfg.p1 < 2) cfg = pll_nominal;   // p1=1 делит пополам на VCO < ~2.8 ГГц  
  
  uint32_t target_clk_hz = (uint32_t)(((uint64_t)cfg.fbdiv * vfo_xosc_hz()) /  
                                      (uint64_t)(cfg.refdiv * cfg.p1 * cfg.p2));  
  bool is_overclocking = (target_clk_hz > current_clk_sys_hz);  
  
  if (is_overclocking) {  
    vreg_set_voltage((vreg_voltage)vsel);  
    busy_wait_us(500);  
  }  
  
  detach_peripheral_clock();  
  uint32_t ints_status = save_and_disable_interrupts();  
  
  // clk_sys -> XOSC (SCK флэша безопасен)  
  clock_configure(clk_sys,  
                  CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                  CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,  
                  vfo_xosc_hz(), vfo_xosc_hz());  
  
  uint64_t ref_hz = ((uint64_t)vfo_xosc_hz() + cfg.refdiv - 1u) / cfg.refdiv;  
  uint32_t vco_nominal_hz = (uint32_t)(ref_hz * (uint64_t)cfg.fbdiv);  
  
  pll_init(pll_sys, cfg.refdiv, vco_nominal_hz, cfg.p1, cfg.p2);  
  
  uint32_t timeout = 20000;  
  while (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS) && --timeout);  
  
  // BAUDR: тактирование флэша строго <= 48 МГц  
  const uint32_t FLASH_SAFE_MAX_HZ = 48000000u;  
  uint32_t ssi_baud = (uint32_t)((target_clk_hz + FLASH_SAFE_MAX_HZ - 1) / FLASH_SAFE_MAX_HZ);  
  ssi_baud = (ssi_baud + 1u) & ~1u;  
  if (ssi_baud < 2u)  ssi_baud = 2u;  
  if (ssi_baud > 34u) ssi_baud = 34u;  
  ssi_hw->ssienr = 0;  
  ssi_hw->baudr  = ssi_baud;  
  ssi_hw->ssienr = 1;  
  
  clock_configure(clk_sys,  
                  CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                  CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,  
                  target_clk_hz, target_clk_hz);  
  restore_interrupts(ints_status);  
  
  // Самопроверка: при промахе повторяем весь цикл (до 3 раз)  
  bool locked_ok = false;  
  uint32_t meas_khz = 0;  
  uint32_t expect_khz = cfg.clk_sys_hz / 1000u;  
  
  for (int attempt = 0; attempt < 3; attempt++) {  
    busy_wait_us(300);  
    meas_khz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);  
    if (meas_khz > expect_khz * 997u / 1000u &&  
        meas_khz < expect_khz * 1003u / 1000u) { locked_ok = true; break; }  
  
    uint32_t ints2 = save_and_disable_interrupts();  
    clock_configure(clk_sys,  
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,  
                    vfo_xosc_hz(), vfo_xosc_hz());  
    pll_init(pll_sys, cfg.refdiv, vco_nominal_hz, cfg.p1, cfg.p2);  
    timeout = 20000;  
    while (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS) && --timeout);  
    clock_configure(clk_sys,  
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,  
                    target_clk_hz, target_clk_hz);  
    restore_interrupts(ints2);  
  }  
  
  if (!locked_ok) {  
    // 3 промаха — безопасный откат на номинал  
    clock_configure(clk_sys,  
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,  
                    vfo_xosc_hz(), vfo_xosc_hz());  
    uint64_t ref_hz_fb = ((uint64_t)vfo_xosc_hz() + pll_nominal.refdiv - 1u) / pll_nominal.refdiv;  
    pll_init(pll_sys, pll_nominal.refdiv,  
             (uint32_t)(ref_hz_fb * (uint64_t)pll_nominal.fbdiv),  
             pll_nominal.p1, pll_nominal.p2);  
    timeout = 20000;  
    while (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS) && --timeout);  
    clock_configure(clk_sys,  
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,  
                    pll_nominal.clk_sys_hz, pll_nominal.clk_sys_hz);  
    current_clk_sys_hz = pll_nominal.clk_sys_hz;  
    return;  
  }  
  
  if (!is_overclocking) {  
    busy_wait_us(100);  
    vreg_set_voltage((vreg_voltage)vsel);  
  }  
  current_clk_sys_hz = cfg.clk_sys_hz;  
}  
  
// ============================================================================  
// ПРИМЕНЕНИЕ ТОНА / СБРОС ДИЗЕРА  
// ============================================================================  
static void set_tone_instant(uint8_t index) {  
  if (index >= SOFT_TX_TONES) return;  
  SoftTxParams p = s_tones[index];  
  
  // Глубина FRAC-дизера по pio_int (как в исходнике):  
  // малый int → широкая маска (больше шум-размазывания),  
  // большой int (НЧ) → узкая/ноль (рядом гармоники не видны).  
  uint32_t mask;  
  if      (p.pio_int <= 16)  mask = 0x03u;  
  else if (p.pio_int <= 64)  mask = 0x01u;  
  else                       mask = 0x00u;  
  
  dds_step           = p.dds_step;  
  target_pio_int     = p.pio_int;  
  target_pio_frac8   = p.pio_frac;  
  frac_dither_mask   = soft_tx_narrowband ? 0u : mask;  
  current_active_tone = index;  
  tone_changed = true;  
}  
  
// ============================================================================  
// КООПЕРАТИВНЫЙ ДИЗЕРИНГ — одна итерация = одна запись clkdiv  
// (тело vfo_core1_entry исходника, переведено на poll-модель)  
// ============================================================================  
void __not_in_flash_func(soft_tx_poll)() {  
  static int32_t  l_step = 0, l_int = 8, l_frac = 0;  
  static uint32_t l_dmask = 0;  
  static uint32_t acc1 = 0, acc2 = 0, m2_carry_prev = 0;  
  static uint32_t rand_state = 0xACE1u;  
  static uint32_t poll_cnt = 0;  
  const uint32_t poll_mask = (1u << 6) - 1u;  
  
  if (!s_running) return;  
  
  if ((poll_cnt & poll_mask) == 0u) {  
    if (__builtin_expect(tone_changed, 0)) {  
      l_step  = (int32_t)dds_step;  
      l_int   = (int32_t)target_pio_int;  
      l_frac  = (int32_t)target_pio_frac8;  
      l_dmask = frac_dither_mask;  
      if (dither_reset_pending) {  
        acc1 = acc2 = m2_carry_prev = 0;  
        rand_state = 0xACE1u + time_us_32();  
        dither_reset_pending = false;  
      }  
      tone_changed = false;  
    }  
  }  
  poll_cnt++;  
  
  rand_state ^= rand_state << 13;  
  rand_state ^= rand_state >> 17;  
  rand_state ^= rand_state << 5;  
  
  int32_t r_bits = (int32_t)(rand_state & 0x0Fu);  
  int32_t step   = l_step + ((r_bits << 1) - 15);  
  
  int32_t total_correction;  
  if (l_dmask == 0u) {  
    uint32_t old_acc = acc1;  
    acc1 += (uint32_t)step;  
    total_correction = (acc1 < old_acc);  
  } else {  
    uint32_t c1 = __builtin_add_overflow(acc1, (uint32_t)step, &acc1) ? 1u : 0u;  
    uint32_t c2 = __builtin_add_overflow(acc2, acc1, &acc2) ? 1u : 0u;  
    total_correction = (int32_t)c1 + (int32_t)c2 - (int32_t)m2_carry_prev;  
    m2_carry_prev = c2;  
  }  
  
  int32_t current_frac = l_frac + total_correction;  
  int32_t current_int  = l_int;  
  
  if (l_dmask != 0u) {  
    uint32_t r1 = (rand_state >> 8)  & l_dmask;  
    uint32_t r2 = (rand_state >> 16) & l_dmask;  
    current_frac += (int32_t)(r1 + r2) - (int32_t)l_dmask;  
  }  
  
  current_int  += (current_frac >> 8);  
  current_frac &= 0xFF;  
  
  lo_pio->sm[lo_sm].clkdiv = ((uint32_t)current_int << 16) | ((uint32_t)current_frac << 8);  
}  
  
// ============================================================================  
// ЖИЗНЕННЫЙ ЦИКЛ  
// ============================================================================  
void soft_tx_init() {  
  int pin = PIN_VFO_TX;  
  if (!pio_program_loaded) {  
    lo_offset = pio_add_program(lo_pio, &pio_square_program);  
    pio_program_loaded = true;  
  }  
  lo_sm = pio_claim_unused_sm(lo_pio, true);  
  pio_sm_config c = pio_get_default_sm_config();  
  sm_config_set_wrap(&c, lo_offset + 0, lo_offset + 1);  
  sm_config_set_set_pins(&c, pin, 1);  
  pio_gpio_init(lo_pio, pin);  
  pio_sm_set_consecutive_pindirs(lo_pio, lo_sm, pin, 1, true);  
  pio_sm_init(lo_pio, lo_sm, lo_offset, &c);  
}  
  
void soft_tx_start(uint32_t freq_hz) {  
  // 1. Глушим SM перед возможной перестройкой PLL — иначе меандр  
  //    уедет на съезжающей частоте.  
  pio_sm_set_enabled(lo_pio, lo_sm, false);  
  
  // 2. Автотюн PLL под целевую частоту (в пределах boost-лимита,  
  //    если буст активен — иначе до номинала 133 МГц).  
  uint64_t limit = clk_boosted ? effective_clk_max() : VFO_CLK_SYS_NOMINAL_HZ;  
  PllConfig opt  = find_optimal_pll(freq_hz ? freq_hz : 14000000u, limit);  
  if (opt.clk_sys_hz != current_clk_sys_hz) {  
    uint32_t vsel = opt.vsel;  
    if (vsel == 0 || vsel < (uint32_t)VREG_VOLTAGE_1_10)  
      vsel = (uint32_t)vsel_for(opt.clk_sys_hz);  
    set_clk_sys(opt, vsel);  
    pll_overclock = opt;  
  }  
  
  // 3. Таблица делителей из банка менеджера → тон 0, сброс MASH.  
  rebuild_table();  
  set_tone_instant(0);  
  dither_reset_pending = true;  
  
  pio_sm_set_enabled(lo_pio, lo_sm, true);  
  s_running = true;  
}  
  
void soft_tx_stop() {  
  pio_sm_set_enabled(lo_pio, lo_sm, false);  
  s_running = false;  
  current_active_tone = VFO_TONE_NONE;  
}  
  
void soft_tx_tone(uint8_t index) {  
  if (index == VFO_TONE_NONE) return;  
  // Аккумуляторы НЕ сбрасываем — фаза не прыгает при смене тона.  
  set_tone_instant(index);  
}  
  
uint32_t soft_tx_actual_hz() {  
  if (!s_running || current_active_tone == VFO_TONE_NONE) return 0;  
  return s_tones[current_active_tone].target_freq_chz * 10u;  
}  
  
uint32_t soft_tx_clk_sys_hz() { return current_clk_sys_hz; }  
  
void soft_tx_set_narrowband(bool on) {  
  soft_tx_narrowband = on ? 1 : 0;  
  if (current_active_tone != VFO_TONE_NONE)  
    set_tone_instant(current_active_tone);  
}  
  
// ============================================================================  
// РАЗГОН / ТЕРМОГВАРД  
// ============================================================================  
void soft_tx_boost_enter(uint32_t target_hz) {  
  if (target_hz == 0) { clk_boosted = true; return; }  
  
  PllConfig opt = find_optimal_pll(target_hz, effective_clk_max());  
  
  // Софт-тюн: если метрика текущего clk_sys не хуже — только пересчёт  
  // таблицы без физического переключения PLL.  
  uint64_t target_mhz = (uint64_t)target_hz * 1000ULL;  
  SoftTxParams dummy;  
  uint64_t m_opt = pll_metric(opt.clk_sys_hz, target_mhz, &dummy);  
  uint64_t m_cur = pll_metric(current_clk_sys_hz, target_mhz, &dummy);  
  
  if (m_opt >= m_cur || opt.clk_sys_hz == current_clk_sys_hz) {  
    rebuild_table();  
    set_tone_instant(0);  
    busy_wait_us(10);  
    return;  
  }  
  
  // SM уже остановлен trx_switch() до вызова — рвём PLL.  
  pio_sm_set_enabled(lo_pio, lo_sm, false);  
  uint32_t vsel = opt.vsel;  
  if (vsel == 0 || vsel < (uint32_t)VREG_VOLTAGE_1_10)  
    vsel = (uint32_t)vsel_for(opt.clk_sys_hz);  
  set_clk_sys(opt, vsel);  
  
  rebuild_table();  
  set_tone_instant(0);  
  dither_reset_pending = true;  
  pll_overclock = opt;  
  clk_boosted = true;  
  if (s_running) pio_sm_set_enabled(lo_pio, lo_sm, true);  
}  
  
void soft_tx_boost_exit() {  
  if (!clk_boosted) return;  
  pio_sm_set_enabled(lo_pio, lo_sm, false);  
  set_clk_sys(pll_nominal, VREG_VOLTAGE_DEFAULT);  
  rebuild_table();  
  set_tone_instant(0);  
  dither_reset_pending = true;  
  clk_boosted = false;  
  if (s_running) pio_sm_set_enabled(lo_pio, lo_sm, true);  
}  
  
void soft_tx_thermal_guard() {  
  if (!clk_boosted && !thermal_throttled) return;  
  if (pll_ceiling.clk_sys_hz <= pll_nominal.clk_sys_hz) return;  
  
  float temp = soft_tx_core_temp_c();  
  if (isnan(temp)) return;              // RX-фаза — ADC занят  
  
  if (!thermal_throttled && temp >= VFO_THROTTLE_HI_C) {  
    pio_sm_set_enabled(lo_pio, lo_sm, false);  
    set_clk_sys(pll_nominal, VREG_VOLTAGE_DEFAULT);  
    rebuild_table();  
    set_tone_instant(0);  
    dither_reset_pending = true;  
    clk_boosted = false;  
    thermal_throttled = true;  
    if (s_running) pio_sm_set_enabled(lo_pio, lo_sm, true);  
  } else if (thermal_throttled && temp <= VFO_THROTTLE_LO_C) {  
    pio_sm_set_enabled(lo_pio, lo_sm, false);  
    set_clk_sys(pll_overclock, pll_overclock.vsel);  
    rebuild_table();  
    set_tone_instant(0);  
    dither_reset_pending = true;  
    clk_boosted = true;  
    thermal_throttled = false;  
    if (s_running) pio_sm_set_enabled(lo_pio, lo_sm, true);  
  }  
}  
  
// ============================================================================  
// РЕГИСТРАЦИЯ ДВИЖКА В МЕНЕДЖЕРЕ  
// ============================================================================  
static const VfoEngine s_engine = {  
  soft_tx_init, soft_tx_start, soft_tx_stop,  
  soft_tx_tone, soft_tx_poll, soft_tx_actual_hz  
};  
const VfoEngine *vfo_engine_soft_tx() { return &s_engine; }
