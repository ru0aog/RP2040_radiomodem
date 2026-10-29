// RP2040_radiomodem.ino  
// Полудуплексный радиомодем RP2040: RX-конвейер (из RP2040_receiver)  
// и TX-генератор (из RP2040_beacon) разделяют Core 1 через trx_control.  
// Core 0: приложение — CAT/CLI, декодеры, планировщик, файлы.  
#include <Arduino.h>  
#include "src/mem/mem_arena.h"  
#include "src/trx/trx_control.h"  
#include "src/vfo/vfo.h"  
#include "src/vfo/soft_tx.h"      // soft_tx_poll() — дизеринг в фазе TX  
  
void setup() {  
  pinMode(LED_BUILTIN, OUTPUT);  
  Serial.begin(115200);  
  while (!Serial && millis() < 2000) {}  
  
  // VFO инициализируем ДО trx: trx_init() может вызвать hook  
  // (vfo_on_trx_switch), который обращается к зарегистрированным  
  // движкам.  
  vfo_init();  
  trx_init(vfo_on_trx_switch);  
  
  Serial.printf("mem_arena: TRX pool %u B, SHARED %u B\n",  
                (unsigned)mem_arena_capacity(MEM_PHASE_RX),  
                (unsigned)mem_arena_capacity(MEM_PHASE_SHARED));  
}  
  
// ---- Core 1: реальное время. Тракты работают только когда состояние  
// стабильно; во время TRX_SWITCHING оба заглушены. ----  
void setup1() {  
  // RX — стартовое состояние. Конвейер стартует на этом же ядре,  
  // где будет крутиться его poll-цикл.  
  rx_pipeline_start();  
}  
  
void loop1() {  
  trx_poll_core1();      // обработать pending trx_switch() с Core 0  
  
  if (trx_state() == TRX_RX) {  
    // TODO(src/rx): rx_capture pump → rx_dsp → audio/decoder queues  
  } else {  
    // MASH-дизеринг делителя PIO — одна запись clkdiv за итерацию.  
    soft_tx_poll();  
    // TODO(src/tx): symbol timing → modulator → vfo_tone()  
  }  
}  
  
// ---- Core 0: приложение ----  
void loop() {  
  // TODO(src/cat): Kenwood TS-2000 CAT parser → trx_switch()  
  // TODO(src/app): scheduler, decoders/RSID, files/MSC, sensors, LCD  
  
  // Термогвард VFO — понижение разгона при перегреве.  
  vfo_thermal_guard();  
  
  // Диагностика водяного знака раз в 10 с.  
  static uint32_t t = 0;  
  if (millis() - t > 10000) {  
    t = millis();  
    Serial.printf("arena watermark: %u / %u B, state=%s\n",  
                  (unsigned)mem_arena_watermark(),  
                  (unsigned)mem_arena_capacity(MEM_PHASE_RX),  
                  trx_state() == TRX_TX ? "TX" : "RX");  
  }  
}
