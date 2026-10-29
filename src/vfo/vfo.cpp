// src/vfo/vfo.cpp  
#include "vfo.h"  
#include "../mem/mem_arena.h"  
#include "hardware/vreg.h"  
  
// Пределы тактирования (как в исходниках)  
#define VFO_CLK_SYS_NOMINAL_HZ  133000000ULL  
#define VFO_CLK_SYS_MAX_HZ      420000000ULL  
#define VFO_XOSC_NOMINAL_HZ     12000000ULL  
#define VFO_XOSC_DEFAULT_PPB    (+25000LL)  
#define VFO_THROTTLE_HI_C       80.0f  
#define VFO_THROTTLE_LO_C       60.0f  
  
// Загружается из Flash при старте файловым менеджером; до того — дефолт.  
int32_t vfo_xosc_corr_ppb = VFO_XOSC_DEFAULT_PPB;  
  
uint64_t vfo_xosc_hz() {  
  int64_t hz = (int64_t)VFO_XOSC_NOMINAL_HZ +  
               (int64_t)VFO_XOSC_NOMINAL_HZ * (int64_t)vfo_xosc_corr_ppb /  
               1000000000LL;  
  return (hz > 0) ? (uint64_t)hz : (uint64_t)VFO_XOSC_NOMINAL_HZ;  
}  
  
// ---- Состояние менеджера ----  
static VfoBank         s_bank;  
static const VfoEngine *s_eng_rx = nullptr;  
static const VfoEngine *s_eng_tx = nullptr;  
static const VfoEngine *s_active = nullptr;   // бэкенд, сейчас генерящий  
static volatile uint8_t s_tone = VFO_TONE_NONE;  
static bool             s_boost = false;  
  
void vfo_init() {  
  memset(&s_bank, 0, sizeof(s_bank));  
  s_eng_rx = vfo_engine_soft_rx();  
  s_eng_tx = vfo_engine_soft_tx();  
  if (s_eng_rx && s_eng_rx->init) s_eng_rx->init();  
  if (s_eng_tx && s_eng_tx->init) s_eng_tx->init();  
}  
  
void vfo_set_backend(TrxState dir, VfoBackend b) {  
  const VfoEngine *e = nullptr;  
  switch (b) {  
    case VFO_BACKEND_SOFT_TX: e = vfo_engine_soft_tx(); break;  
    case VFO_BACKEND_SOFT_RX: e = vfo_engine_soft_rx(); break;  
    case VFO_BACKEND_SI5351:  e = vfo_engine_si5351();  break;  
    default: break;  
  }  
  MEM_ASSERT(e != nullptr);  
  if (dir == TRX_RX) s_eng_rx = e; else s_eng_tx = e;  
}  
  
// ---- Банк ----  
void vfo_bank_set_grid(uint32_t base_hz, double step_hz, uint8_t count) {  
  MEM_ASSERT(count <= VFO_BANK_MAX_TONES);  
  uint64_t base_mhz = (uint64_t)base_hz * 1000ULL;  
  uint64_t step_mhz = (uint64_t)(step_hz * 1000.0);  
  for (uint8_t i = 0; i < count; i++)  
    s_bank.freqs_mhz[i] = base_mhz + (uint64_t)i * step_mhz;  
  s_bank.count = count;  
  s_tone = VFO_TONE_NONE;  
  // Активный бэкенд обязан пересчитать таблицу — сигнал через start/tone  
  if (s_active && s_active->start)  
    s_active->start((uint32_t)(s_bank.freqs_mhz[0] / 1000ULL));  
}  
  
void vfo_bank_set_list(const uint32_t *freqs_hz, uint8_t count) {  
  MEM_ASSERT(freqs_hz && count <= VFO_BANK_MAX_TONES);  
  for (uint8_t i = 0; i < count; i++)  
    s_bank.freqs_mhz[i] = (uint64_t)freqs_hz[i] * 1000ULL;  
  s_bank.count = count;  
  s_tone = VFO_TONE_NONE;  
}  
  
const VfoBank *vfo_bank() { return &s_bank; }  
  
// ---- Тон ----  
void vfo_tone(uint8_t index) {  
  if (index != VFO_TONE_NONE) MEM_ASSERT(index < s_bank.count);  
  s_tone = index;  
  if (s_active && s_active->tone) s_active->tone(index);  
}  
uint8_t vfo_tone_active() { return s_tone; }  
  
// ---- Квадратурный LO (обёртки над SOFT_RX-бэкендом) ----  
uint32_t vfo_quad_start(uint32_t rx_freq_hz) {  
  MEM_ASSERT(s_eng_rx);  
  s_active = s_eng_rx;  
  s_eng_rx->start(rx_freq_hz);  
  return s_eng_rx->actual_hz ? s_eng_rx->actual_hz() : 0;  
}  
void     vfo_quad_stop()  { if (s_eng_rx && s_eng_rx->stop) s_eng_rx->stop(); }  
uint32_t vfo_quad_lo_hz() {  
  return (s_eng_rx && s_eng_rx->actual_hz) ? s_eng_rx->actual_hz() : 0;  
}  
  
// ---- Разгон ----  
void vfo_boost_enter(uint32_t target_hz) {  
  // Реальная реализация живёт в soft_tx.cpp (PLL-скан + vreg + рестарт  
  // таблицы). Здесь только фиксируем намерение и проксируем.  
  extern void soft_tx_boost_enter(uint32_t);  
  extern void soft_tx_boost_exit();  
  soft_tx_boost_enter(target_hz);  
  s_boost = true;  
}  
void vfo_boost_exit() {  
  extern void soft_tx_boost_exit();  
  if (!s_boost) return;  
  soft_tx_boost_exit();  
  s_boost = false;  
}  
bool vfo_boosted() { return s_boost; }  
  
void vfo_thermal_guard() {  
  extern float soft_tx_core_temp_c();  
  static bool throttled = false;  
  float t = soft_tx_core_temp_c();  
  if (!throttled && t > VFO_THROTTLE_HI_C) {  
    throttled = true;  
    vfo_boost_exit();                    // безопасный номинал  
  } else if (throttled && t < VFO_THROTTLE_LO_C) {  
    throttled = false;  
  }  
}  
  
// ---- Hook для trx_control ----  
void vfo_on_trx_switch(TrxState next, uint32_t freq_hz) {  
  // 1. Погасить всё, что генерило (полудуплекс — один генератор).  
  if (s_active && s_active->stop) s_active->stop();  
  s_active = nullptr;  
  s_tone = VFO_TONE_NONE;  
  
  // 2. Разгон — только для передачи и только под TX-частоту.  
  if (next == TRX_RX && s_boost) vfo_boost_exit();  
  
  // 3. Запустить генератор нового направления.  
  const VfoEngine *e = (next == TRX_TX) ? s_eng_tx : s_eng_rx;  
  if (e && e->start) {  
    s_active = e;  
    e->start(freq_hz);  
  }  
}  
  
uint32_t vfo_clk_sys_hz() { extern uint32_t soft_tx_clk_sys_hz(); return soft_tx_clk_sys_hz(); }  
  
void vfo_debug_dump() {  
  Serial.printf("[VFO] clk_sys=%lu Hz  boost=%d  tone=%u  bank=%u\n",  
                (unsigned long)vfo_clk_sys_hz(), (int)s_boost,  
                (unsigned)s_tone, (unsigned)s_bank.count);  
}
