// src/vfo/vfo.cpp  
// ============================================================================  
// Диспетчер VFO: банк тонов (Гц), выбор бэкенда, hook trx_control,  
// разгон и термогвард. Аппаратура — в движках.  
// ============================================================================  
#include "vfo.h"  
#include "soft_rx.h"  
#include "soft_tx.h"  
#include "si5351.h"  
#include "../mem/mem_arena.h"  
  
// ---- Константы ----  
#define VFO_XOSC_NOMINAL_HZ   12000000ULL  
#define VFO_XOSC_DEFAULT_PPB  (+25000LL)  
  
// Коррекция опоры, ppb. Заполняется из Flash до vfo_init().  
int32_t vfo_xosc_corr_ppb = VFO_XOSC_DEFAULT_PPB;  
  
// Пределы разгона по умолчанию (если soft_tx их не переопределит).  
// Паспорт OCTEST записывает сюда свои значения до vfo_init().  
__attribute__((weak)) uint32_t vfo_max_clk_sys_hz = 150000000UL;  
__attribute__((weak)) uint32_t vfo_max_pll_vco_hz = 1500000000ULL;  
  
// Промежуточная частота приёмника — квад считает LO = f_rx − RX_IF.  
#define RX_IF_SHIFT_HZ  11000UL  
  
// ---- Состояние ----  
static VfoBank    s_bank;  
static uint8_t    s_tone       = VFO_TONE_NONE;  
static const VfoEngine *s_eng_tx = NULL;  
static const VfoEngine *s_eng_rx = NULL;  
static const VfoEngine *s_active = NULL;  
  
extern volatile bool clk_boosted;   // soft_tx.cpp  
  
// ============================================================================  
// БАНК ТОНОВ (все частоты — uint32_t Гц)  
// ============================================================================  
void vfo_bank_set_grid(uint32_t base_hz, uint32_t step_hz, uint8_t count) {  
  MEM_ASSERT(count > 0 && count <= VFO_BANK_MAX_TONES);  
  s_bank.base_hz = base_hz;  
  s_bank.step_hz = step_hz;  
  s_bank.count   = count;  
  for (uint8_t i = 0; i < count; i++)  
    s_bank.freqs_hz[i] = base_hz + (uint32_t)i * step_hz;  
  
  // Если генератор жив — перезапуск на базовой частоте сетки.  
  // ВАЖНО: передаём base_hz, а НЕ vfo_actual_hz() — actual активного  
  // движка это LO (f_rx − IF), а не частота настройки.  
  if (s_active && s_active->start) s_active->start(base_hz);  
}  
  
void vfo_tone(uint8_t index) {  
  MEM_ASSERT(index < s_bank.count || index == VFO_TONE_NONE);  
  s_tone = index;  
  if (s_active && s_active->tone) s_active->tone(index);  
}  
  
uint8_t vfo_tone_active(void) { return s_tone; }  
  
const VfoBank *vfo_bank(void) { return &s_bank; }  
  
uint32_t vfo_bank_tone_hz(uint8_t index) {  
  return (index < s_bank.count) ? s_bank.freqs_hz[index] : 0;  
}  
  
uint32_t vfo_bank_step_hz(void) { return s_bank.step_hz; }  
  
// ============================================================================  
// ВЫБОР БЭКЕНДА  
// ============================================================================  
static const VfoEngine *vfo_backend(VfoBackendId b) {  
  switch (b) {  
    case VFO_BACKEND_SOFT_TX: return vfo_engine_soft_tx();  
    case VFO_BACKEND_SOFT_RX: return vfo_engine_soft_rx();  
    case VFO_BACKEND_SI5351:  return vfo_engine_si5351();  
    default:                  return NULL;  
  }  
}  
  
void vfo_set_backend(TrxState dir, VfoBackendId b) {  
  const VfoEngine *e = vfo_backend(b);  
  if (dir == TRX_TX) s_eng_tx = e;  
  else               s_eng_rx = e;  
}  
  
// ============================================================================  
// ИНИЦИАЛИЗАЦИЯ  
// ============================================================================  
void vfo_init(void) {  
  // Движки сами резервируют PIO/SM/DMA при init().  
  if (!s_eng_tx) vfo_set_backend(TRX_TX, VFO_BACKEND_SOFT_TX);  
  if (!s_eng_rx) vfo_set_backend(TRX_RX, VFO_BACKEND_SOFT_RX);  
  if (s_eng_tx && s_eng_tx->init) s_eng_tx->init();  
  if (s_eng_rx && s_eng_rx->init) s_eng_rx->init();  
}  
  
// ============================================================================  
// HOOK trx_control — единственная точка смены активного генератора  
// ============================================================================  
void vfo_on_trx_switch(TrxState next, uint32_t freq_hz) {  
  // 1. Погасить текущий генератор (полудуплекс — активен один).  
  if (s_active && s_active->stop) s_active->stop();  
  s_active = NULL;  
  s_tone   = VFO_TONE_NONE;  
  
  // 2. Разгон актуален только для TX: при RX — откат к номиналу.  
  if (next == TRX_RX && clk_boosted) soft_tx_boost_exit();  
  
  // 3. Запустить движок нового направления.  
  const VfoEngine *e = (next == TRX_TX) ? s_eng_tx : s_eng_rx;  
  Serial.printf("[VFO] switch -> %s eng=%s\n",  
                next == TRX_TX ? "TX" : "RX",  
                e == s_eng_tx ? "soft_tx" :  
                e == s_eng_rx ? "soft_rx" : "none");  
  if (e && e->start) {  
    s_active = e;  
    e->start(freq_hz);  
  }  
}  
  
// ============================================================================  
// СЕРВИС — из loop1() (дизеринг MASH в soft_tx)  
// ============================================================================  
void vfo_poll(void) {  
  if (s_active && s_active->poll) s_active->poll();  
}  
  
// ============================================================================  
// РАЗГОН / ТЕРМОГВАРД — прокси к soft_tx  
// ============================================================================  
void vfo_boost_enter(uint32_t target_hz) {  
  if (s_active == s_eng_tx) soft_tx_boost_enter(target_hz);  
}  
  
void vfo_thermal_guard(void) { soft_tx_thermal_guard(); }  
  
uint64_t vfo_xosc_hz(void) {  
  // Скорректированная опора: xosc * (1 + ppb*1e-9).  
  int64_t hz = (int64_t)VFO_XOSC_NOMINAL_HZ;  
  hz += (hz * (int64_t)vfo_xosc_corr_ppb) / 1000000000LL;  
  return (uint64_t)hz;  
}  
  
// ============================================================================  
// КВАДРАТУРНЫЙ LO (низкоуровневый API для тестов; нормальный путь — через  
// trx_switch/vfo_on_trx_switch)  
// ============================================================================  
void vfo_quad_start(uint32_t rx_freq_hz) {  
  if (s_eng_rx && s_eng_rx->start) s_eng_rx->start(rx_freq_hz);  
}  
  
void vfo_quad_stop(void) {  
  if (s_eng_rx && s_eng_rx->stop) s_eng_rx->stop();  
}  
  
uint32_t vfo_quad_lo_hz(void) {  
  return (s_eng_rx && s_eng_rx->actual_hz) ? s_eng_rx->actual_hz() : 0;  
}  
  
// ============================================================================  
// ДИАГНОСТИКА  
// ============================================================================  
uint32_t vfo_clk_sys_hz(void) { return soft_tx_clk_sys_hz(); }  
  
uint32_t vfo_actual_hz(void) {  
  return (s_active && s_active->actual_hz) ? s_active->actual_hz() : 0;  
}  
  
void vfo_debug_dump(void) {  
  Serial.printf("[VFO] clk_sys=%lu Hz  boosted=%d  tone=%u  bank=%u  xosc=%llu Hz\n",  
                (unsigned long)vfo_clk_sys_hz(),  
                (int)clk_boosted,  
                (unsigned)s_tone,  
                (unsigned)s_bank.count,  
                (unsigned long long)vfo_xosc_hz());  
}