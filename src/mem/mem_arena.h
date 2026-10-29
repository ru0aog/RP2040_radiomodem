// src/mem/mem_arena.h  
// ============================================================================  
// Фазовый менеджер памяти для полудуплексного радиомодема.  
//  
// Идея: приём и передача НИКОГДА не работают одновременно, поэтому  
// RX- и TX-фазы занимают ОДНУ общую область SRAM (union-семантика).  
// При переключении направления вся область противоположной фазы  
// инвалидируется одной операцией mem_arena_release()/mem_arena_switch()  
// — без фрагментации и без malloc.  
//  
// Платим max(RX, TX) памяти вместо RX + TX — на 264 КБ SRAM RP2040  
// это существенно: RX-конвейер (FFT 4096, банки Гёрцеля, метрики  
// Витерби) и TX-цепочка переиспользуют одну область.  
//  
// Фаза SHARED — отдельная область (настройки, расписание, CAT-буферы),  
// не очищается никогда.  
//  
// Контракт вызовов:  
//   mem_arena_switch()/release() — только из trx_poll_core1() на Core 1,  
//     когда старый тракт уже остановлен;  
//   mem_arena_alloc(phase RX|TX) — только из кода активной фазы на  
//     Core 1 (заполняет таблицы при старте тракта);  
//   mem_arena_alloc(SHARED) — с любого ядра, но однопоточно (Core 0).  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
  
enum MemPhase : uint8_t {  
  MEM_PHASE_RX = 0,    // таблицы приёмного тракта  
  MEM_PHASE_TX,        // таблицы передающего тракта  
  MEM_PHASE_SHARED,    // долгоживущие данные (настройки, CAT, расписание)  
};  
  
// Инициализация арены. Вызывать один раз из trx_init() (до любых alloc).  
void      mem_arena_init(void);  
  
// Выделить size байт с выравниванием align (степень двойки, <= 16).  
// Возвращает nullptr при нехватке места.  
// alloc() из неактивной RX/TX-фазы — MEM_ASSERT (ловля ошибок  
// жизненного цикла: таблица, выделенная «не в свою» фазу, будет  
// уничтожена ближайшим trx_switch молча — лучше упасть здесь).  
void     *mem_arena_alloc(MemPhase phase, size_t size, size_t align = 8);  
  
// Сбросить общую область RX/TX до нуля. Для MEM_PHASE_SHARED — ASSERT.  
// Вызывать только когда противоположный тракт уже остановлен  
// (порядок обеспечивает trx_switch → trx_poll_core1).  
void      mem_arena_release(MemPhase phase);  
  
// Переключение активной фазы RX <-> TX: уходящая фаза теряет всю  
// общую область, новая начинает с чистого пула. Вызывается из  
// trx_poll_core1() на шаге 2 последовательности переключения.  
void      mem_arena_switch(MemPhase phase);  
  
// --- Диагностика ---  
size_t    mem_arena_watermark(void);          // пиковое заполнение RX/TX, байт  
size_t    mem_arena_capacity(MemPhase phase); // ёмкость области, байт  
size_t    mem_arena_used(MemPhase phase);     // текущее заполнение, байт  
MemPhase  mem_arena_active_phase(void);       // MEM_PHASE_RX или MEM_PHASE_TX  
  
// Ассерт без исключений (в прошивке нет RTTI): сообщение в Serial +  
// зависание с мигающим LED — видно сразу и на приборе, и в терминале.  
static inline void mem_assert_fail(const char *expr, const char *file, int line) {  
  Serial.printf("mem_arena assert failed: %s (%s:%d)\n", expr, file, line);  
  for (;;) {  
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  
    delay(150);  
  }  
}  
#define MEM_ASSERT(cond) do { if (!(cond)) mem_assert_fail(#cond, __FILE__, __LINE__); } while (0)
