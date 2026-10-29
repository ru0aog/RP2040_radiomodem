// src/trx/trx_control.h  
// Единая точка переключения приём/передача (полудуплексный модем).  
// RX и TX никогда не работают одновременно: на Core 1 активна  
// либо RX-цепочка (ADC+DMA → DDC → демодуляция), либо TX-генератор  
// (модулятор → DDS/PIO). Порядок переключения:  
//   1. stop активного тракта          (на Core 1)  
//   2. mem_arena_release(старая фаза) — таблицы ушедшей фазы мертвы  
//   3. перенастройка VFO              (vfo_request, колбэк)  
//   4. start нового тракта            (на Core 1)  
// Все пути — и команды CAT, и планировщик — идут через trx_switch().  
#pragma once  
#include <Arduino.h>  
#include "../mem/mem_arena.h"  
  
enum TrxState : uint8_t {  
  TRX_RX = 0,        // приём: RX-конвейер активен на Core 1  
  TRX_TX,            // передача: TX-генератор активен на Core 1  
  TRX_SWITCHING,     // переходный период — тракты остановлены  
};  
  
// Колбэк перенастройки VFO при смене направления (split и т.п.).  
// Установить из app-слоя; freq_hz — целевая частота нового тракта.  
typedef void (*TrxVfoHook)(TrxState next, uint32_t freq_hz);  
  
void     trx_init(TrxVfoHook vfo_hook);  
  
// Запросить переключение. Блокирует до завершения перехода.  
// true — переключились; false — отказ (занято/ошибка).  
// Вызов допустим только с Core 0.  
bool     trx_switch(TrxState target, uint32_t freq_hz);  
  
TrxState trx_state();  
bool     trx_is_transmitting();  // удобно для PTT-индикации  
  
// --- Подключаемые конвейеры (реализуются в src/rx/ и src/tx/ позже).  
// Сейчас — weak-заглушки, чтобы каркас компилировался автономно.  
// Вызываются ИСКЛЮЧИТЕЛЬНО на Core 1 и только из trx_switch().  
extern "C" {  
void rx_pipeline_stop();   // глушит ADC/DMA, DDC  
void rx_pipeline_start();  // поднимает capture, чистые таблицы из арены  
void tx_pipeline_stop();   // глушит PIO/DDS  
void tx_pipeline_start();  // поднимает модулятор  
}
