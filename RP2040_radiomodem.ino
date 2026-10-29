// RP2040_radiomodem.ino  
// Полудуплексный радиомодем RP2040: RX-конвейер (из RP2040_receiver)  
// и TX-генератор (из RP2040_beacon) разделяют Core 1 через trx_control.  
// Core 0: приложение — CAT/CLI, декодеры, планировщик, файлы.  
#include <Arduino.h>  
#include "src/mem/mem_arena.h"  
#include "src/trx/trx_control.h"  
  
// Перенастройка VFO при смене направления (заглушка до src/vfo/).  
static void vfo_switch_hook(TrxState next, uint32_t freq_hz) {  
  // TODO(src/vfo): tune_rx()/tune_tx(), split  
  Serial.printf("VFO -> %s %lu Hz\n", next == TRX_TX ? "TX" : "RX",  
                (unsigned long)freq_hz);  
}  
  
void setup() {  
  pinMode(LED_BUILTIN, OUTPUT);  
  Serial.begin(115200);  
  while (!Serial && millis() < 2000) {}  
  
  trx_init(vfo_switch_hook);  
  Serial.printf("mem_arena: TRX pool %u B, SHARED %u B\n",  
                (unsigned)mem_arena_capacity(MEM_PHASE_RX),  
                (unsigned)mem_arena_capacity(MEM_PHASE_SHARED));  
}  
  
// ---- Core 1: реальное время. Тракты работают только когда состояние  
// стабильно; во время TRX_SWITCHING оба заглушены. ----  
void setup1() {  
  // RX — стартовое состояние.  
  rx_pipeline_start();  
}  
  
void loop1() {  
  trx_poll_core1();      // обработать pending trx_switch() с Core 0  
  
  if (trx_state() == TRX_RX) {  
    // TODO(src/rx): rx_capture pump → rx_dsp → audio/decoder queues  
  } else {  
    // TODO(src/tx): symbol timing → modulator → PIO/DDS  
  }  
}  
  
// ---- Core 0: приложение ----  
void loop() {  
  // TODO(src/cat): Kenwood TS-2000 CAT parser → trx_switch()  
  // TODO(src/app): scheduler, decoders/RSID, files/MSC, sensors, LCD  
  
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
