// src/trx/trx_control.cpp  
// ============================================================================  
// Машина состояний приём/передача (полудуплексный модем).  
//  
// Переключение инициируется с Core 0 через trx_switch(), а исполняется  
// на Core 1 в trx_poll_core1() — там живут DMA/PIO обоих трактов.  
// Транспорт ядро→ядро — флаги + барьеры памяти (__dmb), без спинлоков:  
// переключение событие редкое и взаимоисключающее, одного витка  
// pending-флага достаточно.  
//  
// Последовательность на Core 1 (жёстко фиксирована):  
//   1. stop обоих трактов          — тишина гарантирована;  
//   2. mem_arena_switch(фаза)      — таблицы ушедшей фазы мертвы;  
//   3. VFO hook                    — гашение старого генератора,  
//                                    запуск нового на freq_hz (split);  
//   4. start нового тракта.  
//  
// ВНИМАНИЕ: шаг 3 может занимать СОТНИ миллисекунд — PLL-сканер  
// VFO перебирает конфигурации clk_sys с замерами. Поэтому таймаут  
// trx_switch() — 3 секунды, а не «микросекунды перехода».  
// ============================================================================  
#include "trx_control.h"  
#include "../mem/mem_arena.h"  
#include "pico/multicore.h"     // get_core_num(), tight_loop_contents()  
#include "hardware/sync.h"      // __dmb()  
  
// Таймаут ожидания завершения переключения на Core 1.  
// Должен покрывать худший случай VFO-hook: полный PLL-скан clk_sys  
// (десятки кандидатов x замер качества) + старт PIO.  
#define TRX_SWITCH_TIMEOUT_MS  3000  
  
// ============================================================================  
// СОСТОЯНИЕ (volatile — видно обоим ядрам)  
// ============================================================================  
static volatile TrxState s_state = TRX_RX;  
static TrxVfoHook        s_vfo   = nullptr;  
  
// Отложенный запрос на переключение: Core 0 пишет, Core 1 исполняет.  
static volatile TrxState s_pending      = TRX_RX;  
static volatile uint32_t s_pending_freq = 0;  
static volatile bool     s_req_active   = false;  
static volatile bool     s_req_done     = false;  
  
// ============================================================================  
// WEAK-ЗАГЛУШКИ КОНВЕЙЕРОВ  
// extern "C" — то же соглашение, что в заголовке: при появлении реальных  
// реализаций в src/rx/ и src/tx/ линкер подставит их вместо заглушек.  
// ============================================================================  
extern "C" {  
__attribute__((weak)) void rx_pipeline_stop(void)  {}  
__attribute__((weak)) void rx_pipeline_start(void) {}  
__attribute__((weak)) void tx_pipeline_stop(void)  {}  
__attribute__((weak)) void tx_pipeline_start(void) {}  
}  
  
// ============================================================================  
// ИНИЦИАЛИЗАЦИЯ (Core 0, из setup() ПОСЛЕ vfo_init())  
// ============================================================================  
void trx_init(TrxVfoHook vfo_hook) {  
  s_vfo         = vfo_hook;  
  s_state       = TRX_RX;  
  s_req_active  = false;  
  s_req_done    = false;  
  mem_arena_init();  
}  
  
// ============================================================================  
// ОБРАБОТЧИК НА CORE 1 — дёргать каждую итерацию loop1()  
// ============================================================================  
void trx_poll_core1(void) {  
  if (!s_req_active) return;  
  
  noInterrupts();  
  TrxState target   = s_pending;  
  uint32_t freq     = s_pending_freq;  
  interrupts();  
  
  s_state = TRX_SWITCHING;  
  __dmb();  
  
  // 1. Остановить активный тракт. Дёргаем ОБА stop — один из них  
  //    и так бездействует, а гарантия тишины полная (каждый stop  
  //    идемпотентен).  
  rx_pipeline_stop();  
  tx_pipeline_stop();  
  
  // 2. Уходящая фаза отдаёт общую область SRAM; новая фаза  
  //    становится активной и начинает с чистого пула.  
  mem_arena_switch(target == TRX_TX ? MEM_PHASE_TX : MEM_PHASE_RX);  
  
  // 3. VFO — до старта тракта, чтобы частота не уходила при живом  
  //    генераторе. Здесь же гашение старого генератора (полудуплекс).  
  //    МОЖЕТ ЗАНЯТЬ СОТНИ МС — PLL-скан clk_sys внутри движка.  
  if (s_vfo) s_vfo(target, freq);  
  
  // 4. Старт нового тракта.  
  if (target == TRX_TX) tx_pipeline_start();  
  else                  rx_pipeline_start();  
  
  s_state = target;  
  __dmb();                  // гарантировать видимость состояния на Core 0  
  s_req_done = true;  
  __dmb();  
  s_req_active = false;     // закрыть запрос ПОСЛЕ done — иначе Core 0  
                          // увидит "свободно" при незавершённом переходе  
}  
  
// ============================================================================  
// ЗАПРОС ПЕРЕКЛЮЧЕНИЯ — только с Core 0  
// ============================================================================  
bool trx_switch(TrxState target, uint32_t freq_hz) {  
  MEM_ASSERT(target == TRX_RX || target == TRX_TX);  
  if (get_core_num() != 0) return false;   // только приложение  
  if (s_req_active)          return false; // переключение уже идёт  
  if (target == s_state)     return true;  // уже там — ничего делать  
  
  s_pending_freq = freq_hz;  
  s_pending      = target;  
  s_req_done     = false;  
  __dmb();  
  s_req_active   = true;                   // публиковать последним  
  
  // Ждём, пока Core 1 выполнит последовательность. Таймаут должен  
  // покрывать PLL-скан VFO — это СОТНИ миллисекунд, не микросекунды.  
  uint32_t t0 = millis();  
  while (!s_req_done) {  
    if (millis() - t0 > TRX_SWITCH_TIMEOUT_MS) {  
      // НЕ сбрасываем s_req_active: Core 1, если жив, завершит  
      // переход сам и выставит done/state корректно. Сброс здесь  
      // создавал бы расхождение "переключились, а Core 0 не знает".  
      Serial.printf("[TRX] switch timeout -> %s (%lu Hz), "  
                    "state=%d — core1 завис или VFO-скан слишком долгий\n",  
                    target == TRX_TX ? "TX" : "RX",  
                    (unsigned long)freq_hz, (int)s_state);  
      return false;  
    }  
    tight_loop_contents();  
  }  
  return (s_state == target);  
}  
  
// ============================================================================  
// СОСТОЯНИЕ  
// ============================================================================  
TrxState trx_state(void)           { return s_state; }  
bool     trx_is_transmitting(void) { return s_state == TRX_TX; }
