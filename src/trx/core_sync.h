// src/trx/core_sync.h  
// ============================================================================  
// Единственный владелец протокола межъядерной парковки RP2040.  
//  
// Два направления:  
//   core_sync_park_core0() — инициатор Core 1 (PLL-окна clk_sys/SSI).  
//   core_sync_park_core1() — инициатор Core 0 (Flash erase/program,  
//                            PLL-вызовы с Core 0).  
//  
// Обслуживание — ОДНА реализация на ядро, вызывается из всех контекстов,  
// где ядро исполняет код или блокируется в ожидании:  
//   Core 0: core_sync_poll()       — loop(), while(!core1_pll_done),  
//                                    цикл ожидания trx_switch().  
//   Core 1: core_sync_core1_poll() — первой строкой loop1().  
//  
// Семантика ACK: подтверждение выставляется только после сохранения IRQ,  
// их отключения и фактического входа в RAM-цикл (__wfe). По освобождению  
// IRQ восстанавливаются, а не включаются безусловно.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
  
void core_sync_init(void);              // setup(), ДО core0_ready = true  
  
void core_sync_poll(void);              // Core 0: обслуживание запросов парковки  
void core_sync_core1_poll(void);        // Core 1: обслуживание + флаг готовности  
  
// Парковка Core 0. Вызов ТОЛЬКО с Core 1. timeout_us == 0 → дефолт 1 с.  
// false — отказ/таймаут (req сброшен, ядро не остаётся в неопределённом состоянии).  
bool core_sync_park_core0(uint32_t timeout_us);  
void core_sync_unpark_core0(void);  
  
// Парковка Core 1. Вызов ТОЛЬКО с Core 0. timeout_ms == 0 → дефолт 200 мс.  
// false, если Core 1 ещё не дошёл до loop1() (core_sync_core1_ready() == false).  
bool core_sync_park_core1(uint32_t timeout_ms);  
void core_sync_unpark_core1(void);  
  
bool core_sync_core0_parked(void);        // Core 0 внутри RAM-цикла  
bool core_sync_core0_park_pending(void);  // есть необслуженный запрос к Core 0  
bool core_sync_core1_ready(void);         // Core 1 дошёл до loop1()  
  
struct core_sync_stats {  
    uint32_t park0_requests;  
    uint32_t park0_ack;  
    uint32_t park0_timeouts;  
    uint32_t park1_requests;  
    uint32_t park1_ack;  
    uint32_t park1_timeouts;  
    uint32_t park_releases;  
    uint32_t park_reentrant;
    uint32_t park0_dead_initiator;   // Core 0 вышел из парковки по потолку  
    uint32_t park1_dead_initiator;   // Core 1 вышел из парковки по потолку
}; 

// Аппаратный спинлок для PLL-перестроек (общий для quad_vfo и soft_tx).  
// Неблокирующий захват: false = занято, контролируемый отказ.  
bool core_sync_pll_try_lock(void);  
void core_sync_pll_unlock(void);

void core_sync_get_stats(core_sync_stats *out);


