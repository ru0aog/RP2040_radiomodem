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
// Флаг-протокол парковки Core 0 (вместо rp2040.idleOtherCore):  
//   core0_park_req    — номер поколения запроса (0 = запроса нет), пишет Core 1  
//   core0_park_ack    — номер поколения, который Core 0 подтвердил  
//   core0_park_inside — Core 0 находится в RAM-цикле  
// Core 0 обслуживает запрос через core0_park_poll() во всех циклах ожидания.  
// ============================================================================  
#include "trx_control.h"  
#include "../mem/mem_arena.h"  
#include "pico/multicore.h"  
#include "hardware/sync.h"  
  
#define TRX_SWITCH_TIMEOUT_MS  3000  
  
// ---------------------------------------------------------------------------  
// Флаг-протокол парковки (не static — читается из quad_vfo_pll.cpp и скетча)  
// ---------------------------------------------------------------------------  
volatile uint32_t core0_park_req    = 0;  
volatile uint32_t core0_park_ack    = 0;  
volatile uint32_t core0_park_inside = 0;  
volatile uint32_t core0_park_enter  = 0;  
volatile uint32_t core0_park_exit   = 0;  
volatile uint32_t core0_loop_seen   = 0;  
  
// Целиком в RAM: во время смены clk_sys Core 0 не должен трогать XIP.  
void __not_in_flash_func(core0_wait_in_ram)(void) {  
  uint32_t gen = core0_park_req;  
  if (gen == 0) return;  
  core0_park_inside = 1;  
  core0_park_enter++;  
  __dmb();  
  core0_park_ack = gen;            // подтверждаем ИМЕННО это поколение  
  __dmb();  
  __sev();                         // на случай, если Core 1 ждёт в wfe  
  while (core0_park_req == gen) {  
    __wfe();                       // будит __sev() из unpark_core0  
  }  
  core0_park_exit++;  
  __dmb();  
  core0_park_inside = 0;           // теперь Core 1 может открывать следующее окно  
  __dmb();  
  __sev();  
}  
  
// Вызывать в каждом цикле ожидания Core 0.  
void __not_in_flash_func(core0_park_poll)(void) {  
  core0_loop_seen++;  
  if (core0_park_req != 0 && core0_park_req != core0_park_ack) {  
    core0_wait_in_ram();  
  }  
}  
  
// ============================================================================  
// СОСТОЯНИЕ  
// ============================================================================  
static volatile TrxState s_state = TRX_RX;  
static TrxVfoHook        s_vfo   = nullptr;  
  
static volatile TrxState s_pending      = TRX_RX;  
static volatile uint32_t s_pending_freq = 0;  
static volatile bool     s_req_active   = false;  
static volatile bool     s_req_done     = false;  
  
extern "C" {  
__attribute__((weak)) void rx_pipeline_stop(void)  {}  
__attribute__((weak)) void rx_pipeline_start(void) {}  
__attribute__((weak)) void tx_pipeline_stop(void)  {}  
__attribute__((weak)) void tx_pipeline_start(void) {}  
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
  
  noInterrupts();  
  TrxState target = s_pending;  
  uint32_t freq   = s_pending_freq;  
  interrupts();  
  
  s_state = TRX_SWITCHING;  
  __dmb();  
  
  rx_pipeline_stop();  
  tx_pipeline_stop();  
  
  mem_arena_switch(target == TRX_TX ? MEM_PHASE_TX : MEM_PHASE_RX);  
  
  if (s_vfo) s_vfo(target, freq);   // внутри — парковка Core 0 по флагам  
  
  if (target == TRX_TX) tx_pipeline_start();  
  else                  rx_pipeline_start();  
  
  s_state = target;  
  __dmb();  
  s_req_done = true;  
  __dmb();  
  s_req_active = false;  
}  
  
// ============================================================================  
// CORE 0  
// ============================================================================  
bool trx_switch(TrxState target, uint32_t freq_hz) {  
  MEM_ASSERT(target == TRX_RX || target == TRX_TX);  
  if (get_core_num() != 0) return false;  
  if (s_req_active)        return false;  
  if (target == s_state)   return true;  
  
  s_pending_freq = freq_hz;  
  s_pending      = target;  
  s_req_done     = false;  
  __dmb();  
  s_req_active   = true;  
  
  uint32_t t0 = millis();  
  while (!s_req_done) {  
    core0_park_poll();               // парковка — ПЕРВОЙ, до millis/Serial  
    if (core0_park_req != 0) continue;   // окно открыто — без flash-кода  
    if (millis() - t0 > TRX_SWITCH_TIMEOUT_MS) {  
      Serial.printf("[TRX] switch timeout -> %s (%lu Hz), state=%d\n",  
                    target == TRX_TX ? "TX" : "RX",  
                    (unsigned long)freq_hz, (int)s_state);  
      return false;  
    }  
    tight_loop_contents();  
  }  
  return (s_state == target);  
}  
  
TrxState trx_state(void)           { return s_state; }  
bool     trx_is_transmitting(void) { return s_state == TRX_TX; }