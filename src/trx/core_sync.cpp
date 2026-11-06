// src/trx/core_sync.cpp  
// ============================================================================  
// Реализация протокола парковки. Перенесено из:  
//   trx_control.cpp   — core0_park_* (поколения, инициатор Core 1);  
//   file_manager.cpp  — core1_park_* (bool-флаги, инициатор Core 0).  
// Отличие от старого кода: IRQ сохраняются/отключаются ДО ACK и  
// восстанавливаются после выхода — исключён уход в XIP через прерывание.  
// ============================================================================  
#include "core_sync.h"  
#include "hardware/sync.h"  
#include "hardware/timer.h"  
  
#define PARK0_TIMEOUT_US   1000000u   // 1 с — фактическое старое значение  
#define PARK0_SPIN_LIMIT   4000000u   // счётный потолок: таймер может встать  
#define PARK1_TIMEOUT_MS   200u  
  
// ---------------------------------------------------------------------------  
// Core 0 — обслуживающая сторона (не static: читаются из quad_vfo_pll.cpp  
// до его перевода на API — Этап 3)  
// ---------------------------------------------------------------------------  
volatile uint32_t core0_park_req    = 0;  
volatile uint32_t core0_park_ack    = 0;  
volatile uint32_t core0_park_inside = 0;  
volatile uint32_t core0_park_enter  = 0;  
volatile uint32_t core0_park_exit   = 0;  
volatile uint32_t core0_loop_seen   = 0;  
  
// ---------------------------------------------------------------------------  
// Core 1 — обслуживающая сторона (внутренние для core_sync)  
// ---------------------------------------------------------------------------  
static volatile bool core1_park_req = false;  
static volatile bool core1_parked   = false;  
static volatile bool core1_in_loop  = false;  
  
// Печатает Core 0 из check_and_handle_pc_changes() (file_manager.cpp).  
volatile bool core1_park_timed_out = false;  
  
static uint32_t        s_park0_gen = 0;  
static core_sync_stats s_stats;  
  
void core_sync_init(void) {  
    core0_park_req = core0_park_ack = core0_park_inside = 0;  
    core1_park_req = core1_parked = core1_in_loop = false;  
    s_park0_gen = 0;  
    memset(&s_stats, 0, sizeof(s_stats));  
    __dmb();  
}  
  
// ============================================================================  
// CORE 0 — обслуживание  
// ============================================================================  
static void __not_in_flash_func(core0_wait_in_ram)(void) {  
    uint32_t gen = core0_park_req;  
    if (gen == 0) return;  
    uint32_t ints = save_and_disable_interrupts();  // нет XIP через IRQ  
    core0_park_inside = 1;  
    core0_park_enter++;  
    __dmb();  
    core0_park_ack = gen;            // ACK только ПОСЛЕ входа в безопасное состояние  
    __dmb();  
    __sev();  
    while (core0_park_req == gen) {  
        __wfe();  
    }  
    core0_park_exit++;  
    __dmb();  
    core0_park_inside = 0;  
    __dmb();  
    __sev();  
    restore_interrupts(ints);  
}  
  
void __not_in_flash_func(core_sync_poll)(void) {  
    core0_loop_seen++;  
    if (core0_park_req != 0 && core0_park_req != core0_park_ack) {  
        core0_wait_in_ram();  
    }  
}  
  
// ============================================================================  
// CORE 1 — обслуживание  
// ============================================================================  
void __not_in_flash_func(core_sync_core1_poll)(void) {  
    core1_in_loop = true;  
    if (!core1_park_req) return;  
    uint32_t ints = save_and_disable_interrupts();  
    core1_parked = true;  
    __dmb();  
    __sev();  
    while (core1_park_req) { __wfe(); }  
    core1_parked = false;  
    __dmb();  
    __sev();  
    restore_interrupts(ints);  
}  
  
// ============================================================================  
// Парковка Core 0 — инициатор Core 1 (логика из quad_vfo_pll.cpp)  
// ============================================================================  
bool __not_in_flash_func(core_sync_park_core0)(uint32_t timeout_us) {  
    if (get_core_num() != 1) return false;  
    if (timeout_us == 0) timeout_us = PARK0_TIMEOUT_US;  
  
    uint32_t t0 = time_us_32();  
    uint32_t spins = PARK0_SPIN_LIMIT;  
    // 1. Дождаться выхода Core 0 из прошлого окна.  
    while (core0_park_inside) {  
        if ((time_us_32() - t0 > timeout_us) || --spins == 0) {  
            s_stats.park0_timeouts++;  
            return false;  
        }  
    }  
    // 2. Новое поколение (0 зарезервировано).  
    if (++s_park0_gen == 0) s_park0_gen = 1;  
    uint32_t gen = s_park0_gen;  
    __dmb();  
    core0_park_req = gen;  
    __dmb();  
    __sev();  
    s_stats.park0_requests++;  
    // 3. Ждать ACK именно этого поколения.  
    spins = PARK0_SPIN_LIMIT;  
    while (core0_park_ack != gen) {  
        if ((time_us_32() - t0 > timeout_us) || --spins == 0) {  
            core0_park_req = 0;  
            __dmb();  
            __sev();  
            s_stats.park0_timeouts++;  
            return false;  
        }  
    }  
    s_stats.park0_ack++;  
    return true;  
}  
  
void __not_in_flash_func(core_sync_unpark_core0)(void) {  
    core0_park_req = 0;  
    __dmb();  
    __sev();  
    s_stats.park_releases++;  
}  
  
// ============================================================================  
// Парковка Core 1 — инициатор Core 0 (логика из file_manager.cpp)  
// ============================================================================  
bool core_sync_park_core1(uint32_t timeout_ms) {  
    if (get_core_num() != 0) return false;  
    if (core1_parked) { s_stats.park_reentrant++; return true; }  
    if (!core1_in_loop) return false;   // Core 1 ещё в setup1 — не ждём  
    if (timeout_ms == 0) timeout_ms = PARK1_TIMEOUT_MS;  
  
    core1_park_req = true;  
    __dmb();  
    __sev();  
    s_stats.park1_requests++;  
  
    const uint32_t t0 = millis();  
    while (!core1_parked) {  
        if ((uint32_t)(millis() - t0) > timeout_ms) {  
            core1_park_req = false;  
            __dmb();  
            __sev();  
            core1_park_timed_out = true;   // печать — вне критического пути  
            s_stats.park1_timeouts++;  
            return false;  
        }  
        tight_loop_contents();  
    }  
    s_stats.park1_ack++;  
    return true;  
}  
  
void core_sync_unpark_core1(void) {  
    core1_park_req = false;  
    __dmb();  
    __sev();  
    s_stats.park_releases++;  
}  
  
// ============================================================================  
// Статус / статистика  
// ============================================================================  
bool core_sync_core0_parked(void)       { return core0_park_inside != 0; }  
bool core_sync_core0_park_pending(void) { return core0_park_req != 0; }  
bool core_sync_core1_ready(void)        { return core1_in_loop; }  
  
void core_sync_get_stats(core_sync_stats *out) {  
    if (out) *out = s_stats;  
}