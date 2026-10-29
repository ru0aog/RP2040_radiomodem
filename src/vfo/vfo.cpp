// src/vfo/vfo.cpp  
// ============================================================================  
// Диспетчер VFO: банк тонов, выбор бэкенда по направлению, hook  
// trx_control, разгон и термогвард. Аппаратная работа — в движках  
// (soft_tx.cpp / soft_rx.cpp / si5351.cpp).  
// ============================================================================  
#include "vfo.h"  
#include "soft_tx.h"        // boost/thermal/clk_sys прокси  
#include "../mem/mem_arena.h"  
  
// ---- Константы ----  
#define VFO_XOSC_NOMINAL_HZ   12000000ULL  
#define VFO_XOSC_DEFAULT_PPB  (+25000LL)  
  
// Коррекция опоры, ppb. Заполняется из Flash до vfo_init().  
int32_t vfo_xosc_corr_ppb = VFO_XOSC_DEFAULT_PPB;  
  
uint64_t vfo_xosc_hz(void) {  
  int64_t hz = (int64_t)VFO_XOSC_NOMINAL_HZ +  
               (int64_t)VFO_XOSC_NOMINAL_HZ * (int64_t)vfo_xosc_corr_ppb /  
               1000000000LL;  
  return (hz > 0) ? (uint64_t)hz : (uint64_t)VFO_XOSC_NOMINAL_HZ;  
}  
  
// ---- Состояние менеджера ----  
static VfoBank          s_bank;  
static const VfoEngine *s_eng_rx = nullptr;   // бэкенд направления RX  
static const VfoEngine *s_eng_tx = nullptr;   // бэкенд направления TX  
static const VfoEngine *s_active = nullptr;   // сейчас генерит  
static volatile uint8_t s_tone   = VFO_TONE_NONE;  
  
// ============================================================================  
// ИНИЦИАЛИЗАЦИЯ  
// ============================================================================  
void vfo_init(void) {  
  memset(&s_bank, 0, sizeof(s_bank));  
  
  // Дефолтная привязка бэкендов.  
  s_eng_rx = vfo_engine_soft_rx();  
  s_eng_tx = vfo_engine_soft_tx();  
  
  // Движок внешнего синтезатора инициализируем тоже — он сам  
  // детектирует наличие микросхемы на I2C и молчит, если её нет.  
  const VfoEngine *si = vfo_engine_si5351();  
  if (si && si->init) si->init();  
  
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
  if (dir == TRX_RX) s_eng_rx = e;  
  else               s_eng_tx = e;  
}  
  
// ============================================================================  
// БАНК ТОНОВ  
// ============================================================================  
void vfo_bank_set_grid(uint32_t base_hz, double step_hz, uint8_t count) {  
  MEM_ASSERT(count > 0 && count <= VFO_BANK_MAX_TONES);  
  uint64_t base_mhz = (uint64_t)base_hz * 1000ULL;  
  uint64_t step_mhz = (uint64_t)(step_hz * 1000.0);  
  for (uint8_t i = 0; i < count; i++)  
    s_bank.freqs_mhz[i] = base_mhz + (uint64_t)i * step_mhz;  
  s_bank.count = count;  
  s_tone = VFO_TONE_NONE;  
  
  // Пересчёт таблицы делителей — через рестарт активного бэкенда  
  // на его текущей частоте (движок перечитывает vfo_bank()).  
  if (s_active && s_active->start)  
    s_active->start(s_active->actual_hz ? s_active->actual_hz() : base_hz);  
}  
  
void vfo_bank_set_list(const uint32_t *freqs_hz, uint8_t count) {  
  MEM_ASSERT(freqs_hz && count > 0 && count <= VFO_BANK_MAX_TONES);  
  for (uint8_t i = 0; i < count; i++)  
    s_bank.freqs_mhz[i] = (uint64_t)freqs_hz[i] * 1000ULL;  
  s_bank.count = count;  
  s_tone = VFO_TONE_NONE;  
}  
  
const VfoBank *vfo_bank(void) { return &s_bank; }  
  
// ============================================================================  
// УПРАВЛЕНИЕ ТОНОМ  
// ============================================================================  
void vfo_tone(uint8_t index) {  
  if (index != VFO_TONE_NONE) MEM_ASSERT(index < s_bank.count);  
  s_tone = index;  
  if (s_active && s_active->tone) s_active->tone(index);  
}  
  
uint8_t vfo_tone_active(void) { return s_tone; }  
  
// ============================================================================  
// КВАДРАТУРНЫЙ LO (обёртки над RX-бэкендом)  
// ============================================================================  
uint32_t vfo_quad_start(uint32_t rx_freq_hz) {  
  MEM_ASSERT(s_eng_rx != nullptr);  
  s_active = s_eng_rx;  
  if (s_eng_rx->start) s_eng_rx->start(rx_freq_hz);  
  return s_eng_rx->actual_hz ? s_eng_rx->actual_hz() : 0;  
}  
  
void vfo_quad_stop(void) {  
  if (s_eng_rx && s_eng_rx->stop) s_eng_rx->stop();  
  if (s_active == s_eng_rx) s_active = nullptr;  
}  
  
uint32_t vfo_quad_lo_hz(void) {  
  return (s_eng_rx && s_eng_rx->actual_hz) ? s_eng_rx->actual_hz() : 0;  
}  
  
// ============================================================================  
// РАЗГОН / ТЕРМОГВАРД — прокси в soft_tx  
// ============================================================================  
void vfo_boost_enter(uint32_t target_hz) {  
  soft_tx_boost_enter(target_hz);  
}  
  
void vfo_boost_exit(void) {  
  soft_tx_boost_exit();  
}  
  
bool vfo_boosted(void) {  
  return clk_boosted;   // из soft_tx.cpp  
}  
  
void vfo_thermal_guard(void) {  
  soft_tx_thermal_guard();  
}  
  
// ============================================================================  
// СЕРВИСНЫЙ ВЫЗОВ АКТИВНОГО БЭКЕНДА (из loop1())  
// ============================================================================  
void vfo_poll(void) {  
  if (s_active && s_active->poll) s_active->poll();  
}  
  
// ============================================================================  
// HOOK trx_control: смена направления  
// ============================================================================  
void vfo_on_trx_switch(TrxState next, uint32_t freq_hz) {  
  // 1. Погасить текущий генератор (полудуплекс — активен один).  
  if (s_active && s_active->stop) s_active->stop();  
  s_active = nullptr;  
  s_tone   = VFO_TONE_NONE;  
  
  // 2. Разгон актуален только для передачи: в RX — откат на номинал.  
  if (next == TRX_RX && clk_boosted) soft_tx_boost_exit();  
  
  // 3. Запустить бэкенд нового направления.  
  const VfoEngine *e = (next == TRX_TX) ? s_eng_tx : s_eng_rx;  
  if (e && e->start) {  
    s_active = e;  
    e->start(freq_hz);  
  }  
}  
  
// ============================================================================  
// ДИАГНОСТИКА  
// ============================================================================  
uint32_t vfo_clk_sys_hz(void) {  
  return soft_tx_clk_sys_hz();  
}  
  
void vfo_debug_dump(void) {  
  Serial.printf("[VFO] clk_sys=%lu Hz  boosted=%d  tone=%u  bank=%u  xosc=%llu Hz\n",  
                (unsigned long)vfo_clk_sys_hz(),  
                (int)clk_boosted,  
                (unsigned)s_tone,  
                (unsigned)s_bank.count,  
                (unsigned long long)vfo_xosc_hz());  
}
