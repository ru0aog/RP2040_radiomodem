// src/trx/trx_control.cpp  
#include "trx_control.h"  
#include "../mem/mem_arena.h"  
  
static volatile TrxState s_state = TRX_RX;  
static TrxVfoHook        s_vfo   = nullptr;  
  
// ---- Weak-заглушки конвейеров. При появлении реальных реализаций  
// в src/rx/ и src/tx/ линкер подставит их автоматически. ----  
__attribute__((weak)) void rx_pipeline_stop()  {}  
__attribute__((weak)) void rx_pipeline_start() {}  
__attribute__((weak)) void tx_pipeline_stop()  {}  
__attribute__((weak)) void tx_pipeline_start() {}  
  
// Переключение должно исполняться на Core 1, где живут тракты.  
// Простейший транспорт ядро→ядро через флаг: loop1() крутит  
// trx_poll_core1(), Core 0 ждёт подтверждения.  
static volatile TrxState   s_pending = TRX_RX;  
static volatile uint32_t   s_pending_freq = 0;  
static volatile bool       s_req_active  = false;  
static volatile bool       s_req_done    = false;  
  
void trx_init(TrxVfoHook vfo_hook) {  
  s_vfo   = vfo_hook;  
  s_state = TRX_RX;  
  mem_arena_init();  
}  
  
void trx_poll_core1() {  
  // Вызывать каждую итерацию loop1().  
  if (!s_req_active) return;  
  noInterrupts();  
  TrxState target    = s_pending;  
  uint32_t freq      = s_pending_freq;  
  interrupts();  
  
  s_state = TRX_SWITCHING;  
  
  // 1. Остановить активный тракт (оба уже должны быть тихими, но  
  //    дешевле дернуть оба, чем рассуждать о направлении).  
  rx_pipeline_stop();  
  tx_pipeline_stop();  
  
  // 2. Мёртвая фаза отдаёт память; новая становится активной.  
  mem_arena_switch(target == TRX_TX ? MEM_PHASE_TX : MEM_PHASE_RX);  
  
  // 3. VFO — последним до старта, чтобы не было ухода частоты  
  //    при живом тракте.  
  if (s_vfo) s_vfo(target, freq);  
  
  // 4. Старт нового тракта.  
  if (target == TRX_TX) tx_pipeline_start();  
  else                  rx_pipeline_start();  
  
  s_state     = target;  
  s_req_active = false;  
  __dmb();          // arm: гарантировать видимость флага на Core 0  
  s_req_done  = true;  
}  
  
bool trx_switch(TrxState target, uint32_t freq_hz) {  
  MEM_ASSERT(target == TRX_RX || target == TRX_TX);  
  if (get_core_num() != 0) return false;   // только с Core 0  
  if (target == s_state && !s_req_active) return true;  
  if (s_req_active) return false;          // переключение уже идёт  
  
  s_pending_freq = freq_hz;  
  s_pending      = target;  
  s_req_done     = false;  
  __dmb();  
  s_req_active   = true;  
  
  // Ждём, пока Core 1 обработает. Таймаут — страховка от мёртвого loop1().  
  uint32_t t0 = millis();  
  while (!s_req_done) {  
    if (millis() - t0 > 100) {             // не должно случаться  
      s_req_active = false;  
      return false;  
    }  
    tight_loop_contents();  
  }  
  return (s_state == target);  
}  
  
TrxState trx_state()           { return s_state; }  
bool     trx_is_transmitting() { return s_state == TRX_TX; }
