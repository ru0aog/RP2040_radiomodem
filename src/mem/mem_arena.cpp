// src/mem/mem_arena.cpp  
#include "mem_arena.h"  
  
// ============================================================================  
// ПУЛЫ. Размеры подобрать по мере миграции буферов:  
//   - RX-конвейер — самый прожорливый потребитель (FFT-кольца 4096,  
//     GTZ-банки Гёрцеля, метрики/предшественники Витерби JTTY);  
//   - область RX/TX должна вместить max(RX-набор, TX-набор);  
//   - контролируется mem_arena_watermark() из loop() раз в 10 с.  
// Миграция: большие static-буферы исходников (rx_buf_*, rx_scratch_*,  
// g_metric_*, g_predecessor, TX-буферы символов) переводятся на  
// mem_arena_alloc() при переносе модулей — API стабилен с первого коммита.  
// ============================================================================  
#ifndef MEM_TRX_POOL_SIZE  
#define MEM_TRX_POOL_SIZE    (96u * 1024u)   // общая область RX|TX  
#endif  
#ifndef MEM_SHARED_POOL_SIZE  
#define MEM_SHARED_POOL_SIZE  (8u * 1024u)   // настройки, расписание, CAT  
#endif  
  
__attribute__((aligned(16))) static uint8_t s_trx_pool[MEM_TRX_POOL_SIZE];  
__attribute__((aligned(16))) static uint8_t s_shared_pool[MEM_SHARED_POOL_SIZE];  
  
static size_t   s_trx_offset   = 0;  
static size_t   s_shared_off   = 0;  
static size_t   s_trx_high_wm  = 0;      // водяной знак за всё время  
static MemPhase s_active       = MEM_PHASE_RX;  
static bool     s_initialized  = false;  
  
void mem_arena_init(void) {  
  s_trx_offset  = 0;  
  s_shared_off  = 0;  
  s_trx_high_wm = 0;  
  s_active      = MEM_PHASE_RX;  
  s_initialized = true;  
}  
  
void *mem_arena_alloc(MemPhase phase, size_t size, size_t align) {  
  MEM_ASSERT(s_initialized);  
  if (size == 0) return nullptr;  
  MEM_ASSERT(align != 0 && (align & (align - 1)) == 0 && align <= 16);  
  
  uint8_t *base;  
  size_t  *offset;  
  size_t   cap;  
  
  if (phase == MEM_PHASE_SHARED) {  
    base   = s_shared_pool;  
    offset = &s_shared_off;  
    cap    = MEM_SHARED_POOL_SIZE;  
  } else {  
    // RX/TX могут выделять память только в своей активной фазе.  
    if (phase != s_active) {  
      MEM_ASSERT(false);  
      return nullptr;  
    }  
    base   = s_trx_pool;  
    offset = &s_trx_offset;  
    cap    = MEM_TRX_POOL_SIZE;  
  }  
  
  uintptr_t p   = (uintptr_t)(base + *offset);  
  uintptr_t al  = (p + (align - 1)) & ~(uintptr_t)(align - 1);  
  size_t    pad = al - p;  
  if (*offset + pad + size > cap) return nullptr;  
  
  *offset += pad + size;  
  if (base == s_trx_pool && *offset > s_trx_high_wm) s_trx_high_wm = *offset;  
  return (void *)al;  
}  
  
void mem_arena_release(MemPhase phase) {  
  MEM_ASSERT(s_initialized);  
  MEM_ASSERT(phase != MEM_PHASE_SHARED);  
  // Область RX/TX общая — сброс действует независимо от того, какая  
  // фаза её занимала. Водяной знак НЕ сбрасываем: это диагностика.  
  s_trx_offset = 0;  
}  
  
void mem_arena_switch(MemPhase phase) {  
  MEM_ASSERT(s_initialized);  
  MEM_ASSERT(phase == MEM_PHASE_RX || phase == MEM_PHASE_TX);  
  if (phase == s_active) return;  
  // Уходящая фаза полностью освобождает общую область —  
  // единственная точка, где фаза меняется.  
  s_trx_offset = 0;  
  s_active = phase;  
}  
  
size_t mem_arena_watermark(void) { return s_trx_high_wm; }  
  
size_t mem_arena_capacity(MemPhase phase) {  
  return (phase == MEM_PHASE_SHARED) ? MEM_SHARED_POOL_SIZE  
                                     : MEM_TRX_POOL_SIZE;  
}  
  
size_t mem_arena_used(MemPhase phase) {  
  return (phase == MEM_PHASE_SHARED) ? s_shared_off : s_trx_offset;  
}  
  
MemPhase mem_arena_active_phase(void) { return s_active; }
