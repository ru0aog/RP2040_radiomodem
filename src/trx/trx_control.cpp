// src/trx/trx_control.cpp  
// ============================================================================  
// Машина состояний приём/передача (полудуплексный модем).  
//  
// Переключение инициируется с Core 0 через trx_switch(), исполняется  
// на Core 1 в trx_poll_core1().  
//  
// Последовательность на Core 1:  
//   1. stop обоих трактов;  
//   2. mem_arena_switch(фаза);  
//   3. VFO hook (может занять сотни мс — PLL-скан clk_sys);  
//   4. start нового тракта.  
//  
// Парковка ядер — через core_sync (единственный владелец протокола).    
// Core 0 обслуживает core_sync_poll() во всех циклах ожидания.
//
// Остаточная гонка при тайм-ауте (осознанное допущение, не правка): 
// если Core 1 ещё выполняет переход, когда Core 0 отвалился по таймауту и сбросил s_req_active, 
// следующий trx_switch() перезапишет s_pending/s_pending_freq в середине чужого перехода. 
// Вероятность мала (тайм-аут 3 с против типичных сотен мс), 
// полное закрытие — через состояния REQUESTED/RUNNING/DONE/FAILED, что выходит за рамки этого этапа.
//
// ============================================================================  
#include "trx_control.h"  
#include "../mem/mem_arena.h"  
#include "pico/multicore.h"  
#include "hardware/sync.h"
#include "core_sync.h"
  
#define TRX_SWITCH_TIMEOUT_MS  3000  
  
// ============================================================================  
// СОСТОЯНИЕ  
// ============================================================================  
static volatile TrxState s_state = TRX_RX;  
static TrxVfoHook        s_vfo   = nullptr;  
  
static volatile TrxState s_pending      = TRX_RX;    
static volatile uint32_t s_pending_freq = 0;    
static volatile bool     s_req_active   = false;    
static volatile bool     s_req_done     = false;    
static volatile uint32_t s_cur_freq     = 0;   // частота, реально стоящая в тракте
volatile uint32_t dbg_c1_stage = 0;   // диагностика: этап перехода на Core 1
  
extern "C" {    
__attribute__((weak)) void rx_pipeline_stop(void)              {}    
__attribute__((weak)) void rx_pipeline_start(void)             {}    
__attribute__((weak)) void rx_pipeline_set_freq(uint32_t hz)   {}    
__attribute__((weak)) void tx_pipeline_stop(void)              {}    
__attribute__((weak)) void tx_pipeline_start(void)             {}    
}
  
void trx_init(TrxVfoHook vfo_hook) {  
  s_vfo        = vfo_hook;  
  s_state      = TRX_RX;  
  s_req_active = false;  
  s_req_done   = false;  
  mem_arena_init();  
}  
  
// ============================================================================  
// CORE 1  
// ============================================================================  
void trx_poll_core1(void) {    
  if (!s_req_active) return;    
    
  uint32_t ints = save_and_disable_interrupts();      
  TrxState target = s_pending;      
  uint32_t freq   = s_pending_freq;      
  restore_interrupts(ints);  
    
  dbg_c1_stage = 10;               // переход принят  
  s_state = TRX_SWITCHING;    
  __dmb();    
    
  rx_pipeline_stop();    
  tx_pipeline_stop();    
    
  dbg_c1_stage = 11;               // mem_arena_switch  
  mem_arena_switch(target == TRX_TX ? MEM_PHASE_TX : MEM_PHASE_RX);    
    
  dbg_c1_stage = 12;               // VFO hook (PLL-скан, парковка Core 0)  
  if (s_vfo) s_vfo(target, freq);   // внутри — парковка Core 0 через core_sync  
    
  dbg_c1_stage = 13;               // start тракта  
  if (target == TRX_TX) {      
    tx_pipeline_start();      
  } else {      
    rx_pipeline_set_freq(freq);    // запрос из trx_switch — единственный источник      
    rx_pipeline_start();      
  }  
    
  s_state    = target;      
  s_cur_freq = freq;      
  __dmb();      
  s_req_done = true;  
  __dmb();    
  s_req_active = false;    
  dbg_c1_stage = 0;                // переход завершён  
}
  
// ============================================================================  
// CORE 0  
// ============================================================================  
bool trx_switch(TrxState target, uint32_t freq_hz) {  
  MEM_ASSERT(target == TRX_RX || target == TRX_TX);  
  if (get_core_num() != 0) return false;  
  if (s_req_active)        return false;    
  // RX→RX с той же частотой — no-op; с новой — полный цикл через Core 1.    
  if (target == s_state && freq_hz == s_cur_freq) return true;
  
  s_pending_freq = freq_hz;  
  s_pending      = target;  
  s_req_done     = false;  
  __dmb();  
  s_req_active   = true;
  uint32_t t0 = millis();
  
  while (!s_req_done) {    
    core_sync_poll();                    // парковка — ПЕРВОЙ, до millis/Serial    
    if (core_sync_core0_park_pending()) continue;  // окно открыто — без flash-кода
    if (millis() - t0 > TRX_SWITCH_TIMEOUT_MS) {    
      Serial.printf("[TRX] switch timeout -> %s (%lu Hz), state=%d\n",    
                    target == TRX_TX ? "TX" : "RX",    
                    (unsigned long)freq_hz, (int)s_state);    
      s_req_active = false;   // иначе следующий trx_switch навсегда вернёт false    
      __dmb();    
      return false;    
    }
    tight_loop_contents();  
  }  
  return (s_state == target);  
}  
  
TrxState trx_state(void)           { return s_state; }  
bool     trx_is_transmitting(void) { return s_state == TRX_TX; }