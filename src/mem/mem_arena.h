// src/mem/mem_arena.h  
// Фазовый менеджер памяти для полудуплексного радиомодема.  
// RX- и TX-фазы занимают ОДНУ общую область SRAM (union-семантика):  
// при переключении направления вся область противоположной фазы  
// инвалидируется одной операцией mem_arena_release().  
// Фаза SHARED живёт в отдельной области и не очищается никогда.  
#pragma once  
#include <Arduino.h>  
  
enum MemPhase : uint8_t {  
  MEM_PHASE_RX = 0,  
  MEM_PHASE_TX = 1,  
  MEM_PHASE_SHARED = 2,  
};  
  
// Инициализация арены. Вызывать один раз из setup().  
void   mem_arena_init();  
  
// Выделить size байт с выравниванием align (степень двойки, <= 16).  
// Возвращает nullptr при нехватке места или alloc() из неактивной фазы.  
void  *mem_arena_alloc(MemPhase phase, size_t size, size_t align = 8);  
  
// Сбросить всю область фазы (RX или TX) до нуля.  
// Для MEM_PHASE_SHARED — запрещено (ASSERT).  
void   mem_arena_release(MemPhase phase);  
  
// Переключение активной фазы: RX <-> TX. Вызывается из trx_switch().  
// Автоматически делает release() уходящей фазы.  
void   mem_arena_switch(MemPhase phase);  
  
// Диагностика: водяной знак максимального заполнения области RX/TX (байт).  
size_t mem_arena_watermark();  
// Общая ёмкость области RX/TX и SHARED (байт).  
size_t mem_arena_capacity(MemPhase phase);  
// Текущая активная фаза.  
MemPhase mem_arena_active_phase();  
  
// Ассерт без исключений (на RP2040 нет RTTI-исключений): зависаем с мигающим LED.  
#define MEM_ASSERT(cond)                                                \  
  do {                                                                  \  
    if (!(cond)) {                                                      \  
      Serial.printf("mem_arena assert failed: %s (%s:%d)\n",            \  
                    #cond, __FILE__, __LINE__);                         \  
      for (;;) {                                                        \  
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));           \  
        delay(150);                                                     \  
      }                                                                 \  
    }                                                                   \  
  } while (0)
