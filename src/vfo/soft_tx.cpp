// src/vfo/soft_tx.cpp  
// Перенесено из RP2040_beacon/vfo_hardware.cpp (GPLv3, RU0AOG):  
// расчёт делителя PIO Q8.8, dds_step с принудительной нечётностью,  
// MASH-1-1, TPDF FRAC-дизер, Xorshift32.  
#include "soft_tx.h"  
#include "vfo.h"  
#include "../mem/mem_arena.h"  
#include "hardware/pio.h"  
#include "hardware/clocks.h"  
#include "hardware/pll.h"  
#include "hardware/sync.h"  
#include "hardware/vreg.h"  
#include "hardware/adc.h"  
#include "hardware/structs/ssi.h"  
  
#define SOFT_TX_TONES   64  
#define XOSC_NOMINAL    12000000ULL  
  
// PIO-программа меандра: set pins,1 / set pins,0 (2 такта на период).  
static const uint16_t pio_sq_instr[] = { 0xe001, 0xe000 };  
static const pio_program_t pio_sq = { pio_sq_instr, 2, -1, 0 };  
  
static PIO  s_pio = pio0;  
static uint s_sm  = 0;  
static uint s_off = 0;  
static int  s_pin = -1;             // VFO_OUTPUT_PIN — назначить из pins.h  
static bool s_loaded = false;  
static bool s_running = false;  
  
// Банк делителей в RAM — заполняется из vfo_bank.  
static SoftTxParams s_tones[SOFT_TX_TONES];  
static volatile uint8_t  s_tone_idx = 255;  
static volatile bool     s_tone_pending = false;  
static volatile bool     s_reset_pending = false;   // сброс MASH при рестарте  
  
// Целевые параметры, применяемые в poll (спинлок не нужен: запись  
// и применение обе на Core 1-потоке опроса — см. trx-модель).  
static volatile uint32_t s_step = 0, s_int = 8, s_frac = 0;  
static volatile uint32_t s_dmask = 0x03;              // FRAC-дизер маска  
volatile uint8_t soft_tx_narrowband = 0;  
  
static uint32_t s_clk_sys = 133000000;  
  
uint32_t soft_tx_clk_sys_hz() { return s_clk_sys; }  
uint32_t soft_tx_actual_hz()  { return s_running ? (uint32_t)(s_tones[s_tone_idx < SOFT_TX_TONES ? s_tone_idx : 0].target_freq_chz) * 10u : 0; }  
  
// --- Прецизионный расчёт делителя (calculate_raw_params_mhz исходника) ---  
static SoftTxParams calc_params(uint64_t clk_sys_hz, uint64_t mhz_target) {  
  if (mhz_target < 100000000ULL)   mhz_target = 100000000ULL;    // >=0.1 МГц  
  if (mhz_target > 40000000000ULL) mhz_target = 40000000000ULL;  // <=40 МГц  
  uint64_t denom = mhz_target * 2ULL;                    // 2 такта/период  
  uint64_t div8  = ((clk_sys_hz * 256ULL) * 1000ULL) / denom;  
  
  SoftTxParams p;  
  p.pio_int  = div8 >> 8;  
  p.pio_frac = (uint8_t)(div8 & 0xFFu);  
  p.target_freq_chz = (uint32_t)(mhz_target / 10ULL);  
  if (p.pio_int < 2) { p.pio_int = 2; p.pio_frac = 0; }  
  
  uint64_t rem  = ((clk_sys_hz * 256ULL) * 1000ULL) % denom;  
  uint64_t mid  = (rem << 16) / denom;  
  uint64_t low  = (rem << 16) % denom;  
  p.dds_step = (uint32_t)((mid << 16) + ((low << 16) / denom));  
  if (p.dds_step) p.dds_step |= 1u;    // gcd(step,2^32)=1 → шумовая полка  
  return p;  
}  
  
// Пересчёт банка из менеджера в таблицу делителей.  
static void rebuild_table() {  
  const VfoBank *b = vfo_bank();  
  for (uint8_t i = 0; i < b->count && i < SOFT_TX_TONES; i++)  
    s_tones[i] = calc_params(s_clk_sys, b->freqs_mhz[i]);  
}  
  
void soft_tx_init() {  
  // TODO: s_pin = VFO_OUTPUT_PIN из pins.h  
  s_pin = 2;  
  s_off = pio_add_program(s_pio, &pio_sq);  
  s_sm  = pio_claim_unused_sm(s_pio, true);  
  s_loaded = true;  
  pio_sm_set_consecutive_pindirs(s_pio, s_sm, s_pin, 1, true);  
  pio_sm_config c = pio_get_default_sm_config();  
  sm_config_set_set_pins(&c, s_pin, 1);  
  sm_config_set_clkdiv(&c, 8.0f);  
  pio_sm_init(s_pio, s_sm, s_off, &c);  
}  
  
void soft_tx_start(uint32_t freq_hz) {  
  rebuild_table();  
  s_reset_pending = true;  
  s_tone_idx = 0;  
  s_tone_pending = true;  
  if (s_loaded) { pio_sm_set_enabled(s_pio, s_sm, true); s_running = true; }  
  (void)freq_hz;  
}  
  
void soft_tx_stop() {  
  if (s_loaded) pio_sm_set_enabled(s_pio, s_sm, false);  
  s_running = false;  
  s_tone_idx = 255;  
}  
  
void soft_tx_tone(uint8_t index) {  
  if (index == 255) { s_tone_idx = 255; return; }  
  MEM_ASSERT(index < SOFT_TX_TONES);  
  s_tone_idx = index;  
  s_tone_pending = true;   // фаза MASH сохраняется — без щелчка  
}  
  
// ---- Кооперативный дизеринг: одна итерация = один clkdiv-апдейт ----  
void __not_in_flash_func(soft_tx_poll)() {  
  static int32_t  l_step = 0, l_int = 8, l_frac = 0;  
  static uint32_t l_dmask = 0;  
  static uint32_t acc1 = 0, acc2 = 0, m2c = 0, rnd = 0xACE1u;  
  static uint32_t poll_cnt = 0;  
  if (!s_running) return;  
  
  if (((poll_cnt++ & 0x3F) == 0) && s_tone_pending) {  
    uint8_t i = s_tone_idx;  
    if (i < SOFT_TX_TONES) {  
      l_step = (int32_t)s_tones[i].dds_step;  
      l_int  = (int32_t)s_tones[i].pio_int;  
      l_frac = (int32_t)s_tones[i].pio_frac;  
      l_dmask = soft_tx_narrowband ? 0u : s_dmask;  
    }  
    if (s_reset_pending) { acc1 = acc2 = m2c = 0; rnd = 0xACE1u; s_reset_pending = false; }  
    s_tone_pending = false;  
  }  
  
  rnd ^= rnd << 13; rnd ^= rnd >> 17; rnd ^= rnd << 5;  
  int32_t step = l_step + (((int32_t)(rnd & 0x0Fu) << 1) - 15);  
  
  int32_t corr;  
  if (l_dmask == 0) {  
    uint32_t o = acc1; acc1 += (uint32_t)step;  
    corr = (acc1 < o);  
  } else {  
    uint32_t c1 = __builtin_add_overflow(acc1, (uint32_t)step, &acc1) ? 1 : 0;  
    uint32_t c2 = __builtin_add_overflow(acc2, acc1, &acc2) ? 1 : 0;  
    corr = (int32_t)c1 + (int32_t)c2 - (int32_t)m2c; m2c = c2;  
  }  
  
  int32_t fr = l_frac + corr;  
  int32_t it = l_int;  
  if (l_dmask) {  
    uint32_t r1 = (rnd >> 8) & l_dmask, r2 = (rnd >> 16) & l_dmask;  
    fr += (int32_t)(r1 + r2) - (int32_t)l_dmask;  
  }  
  it += (fr >> 8); fr &= 0xFF;  
  s_pio->sm[s_sm].clkdiv = ((uint32_t)it << 16) | ((uint32_t)fr << 8);  
}  
  
// ---- Разгон ----  
// Полный PLL-сканер исходника (vfo_find_optimal_pll + метрика спуров)  
// переносится сюда следующей итерацией вместе с vfo_set_clk_sys().  
// Пока: ограниченный boost фиксированными профилями.  
void soft_tx_boost_enter(uint32_t target_hz) {  
  // TODO: порт vfo_find_optimal_pll()/vfo_set_clk_sys() из vfo_hardware.cpp  
  // (переключение clk_sys на XOSC → pll_init → BAUDR флэша → clk_sys на  
  //  PLL_SYS → проверка frequency_count_khz с 3 ретраями).  
  rebuild_table();  
  s_reset_pending = true;  
  s_tone_pending  = true;  
  (void)target_hz;  
}  
void soft_tx_boost_exit() { rebuild_table(); s_reset_pending = true; s_tone_pending = true; }  
  
void soft_tx_set_narrowband(bool on) { soft_tx_narrowband = on ? 1 : 0; }  
  
float soft_tx_core_temp_c() {  
  adc_select_input(4);  
  const float conv = 3.3f / (1 << 12);  
  float v = (float)adc_read() * conv;  
  return 27.0f - (v - 0.706f) / 0.001721f;  
}  
  
// Регистрация в менеджере  
static const VfoEngine s_engine = {  
  soft_tx_init, soft_tx_start, soft_tx_stop,  
  soft_tx_tone, soft_tx_poll, soft_tx_actual_hz  
};  
const VfoEngine *vfo_engine_soft_tx() { return &s_engine; }
