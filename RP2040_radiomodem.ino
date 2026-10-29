// RP2040_radiomodem.ino  
// ============================================================================  
// Полудуплексный цифровой радиомодем на RP2040.  
//  
// Приёмный тракт (наследие RP2040_receiver: ADC+DMA → DDC → децимация →  
// демодуляция) и передающий тракт (наследие RP2040_beacon: модулятор →  
// дизеринг-DDS на PIO) ФИЗИЧЕСКИ раздельны и никогда не работают  
// одновременно — ядро 1 арендуется через trx_control.  
//  
// Core 1 (реальное время): либо RX-конвейер, либо TX-генератор.  
//   Плюс vfo_poll() — сервис активного VFO-движка (MASH-дизеринг в TX).  
// Core 0 (приложение): CAT/CLI, декодеры + RSID, планировщик, файлы/MSC,  
//   датчики, LCD. Единственный путь в эфир — trx_switch().  
//  
// Смена направления (trx_switch → Core 1):  
//   1) stop активного тракта  
//   2) mem_arena_release() — таблицы ушедшей фазы мертвы целиком  
//   3) vfo_on_trx_switch() — гашение старого генератора, запуск нового  
//   4) start нового тракта  
// ============================================================================  
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include "src/pins.h"  
#include "src/mem/mem_arena.h"  
#include "src/trx/trx_control.h"  
#include "src/vfo/vfo.h"  
  
// ============================================================================  
// setup() — Core 0. Порядок инициализации важен:  
//   pins   → аппаратная карта GPIO до того, как движки её прочтут;  
//   vfo    → регистрация движков до того, как trx вызовет hook;  
//   trx    → mem_arena_init() внутри + привязка vfo_on_trx_switch.  
// ============================================================================  
void setup() {  
  pinMode(LED_BUILTIN, OUTPUT);  
  digitalWrite(LED_BUILTIN, LOW);  
  
  Serial.begin(115200);  
  while (!Serial && millis() < 2000) {}  
  
  Serial.println();  
  Serial.println("RP2040_radiomodem — полудуплексный цифровой радиомодем");  
  Serial.println("====================================================");  
  
  // 1. Карта GPIO: квадратура 0/1, TX 2, ДПФ 6-9, I2C 4/5, энкодер 19-21.  
  pins_init();  
  
  // 2. VFO: регистрация и init движков (soft_tx→pio0, soft_rx→pio1,  
  //    si5351→I2C-детект). ДО trx_init() — hook может сработать сразу.  
  vfo_init();  
  
  // 3. TRX: инициализация арены + машина состояний. Hook — единая точка  
  //    перенастройки частоты при смене направления (split внутри).  
  trx_init(vfo_on_trx_switch);  
  
  // Диагностика каркаса.  
  Serial.printf("mem_arena: TRX pool %u B, SHARED %u B\n",  
                (unsigned)mem_arena_capacity(MEM_PHASE_RX),  
                (unsigned)mem_arena_capacity(MEM_PHASE_SHARED));  
  vfo_debug_dump();  
}  
  
// ============================================================================  
// Core 1 — реальное время.  
// ============================================================================  
  
// Стартовое состояние — RX. Конвейер поднимается на том же ядре, где  
// будет крутиться его сервисный цикл (DMA-callbacks привязаны к core1).  
void setup1() {  
  rx_pipeline_start();  
}  
  
void loop1() {  
  // Обработать отложенный trx_switch() с Core 0. Во время перехода  
  // (TRX_SWITCHING) оба тракта заглушены — дальше не идём.  
  trx_poll_core1();  
  
  // Сервис активного VFO-движка: в TX — MASH-дизеринг clkdiv,  
  // в RX — пустой вызов у квадратуры. Одна проверка указателя.  
  vfo_poll();  
  
  if (trx_state() == TRX_RX) {  
    // TODO(src/rx): rx_capture_pump() → rx_dsp() → очереди в декодеры  
    //   - перезапуск ADC/DMA по заполнению буфера  
    //   - DDC (NCO = f_rx − vfo_quad_lo_hz()), децимация  
    //   - демодуляция → базовая полоса в очередь на Core 0  
  } else {  
    // TODO(src/tx): такт символов → модулятор → vfo_tone(index)  
    //   - генератор символов режима (RTTY/CW/MFSK/JTTY...)  
    //   - выбор тона из VfoBank по символу  
    //   - ФЧХ-паузы: vfo_tone(VFO_TONE_NONE)  
  }  
}  
  
// ============================================================================  
// Core 0 — приложение.  
// ============================================================================  
void loop() {  
  // TODO(src/cat): Kenwood TS-2000 CAT-парсер  
  //   - команда TX/RX → trx_switch(TRX_TX/TRX_RX, freq)  
  //   - установка частоты → vfo_bank_set_*()  
  //   - выбор режима → подключение модулятора/демодулятора  
  //  
  // TODO(src/app):  
  //   - планировщик передач (maпки/окна из scheduler.h маяка)  
  //   - декодеры + RSID (очереди из loop1, без блокировок)  
  //   - файловая система / USB-MSC (SET.TXT: XOSC_PPB, пины, расписание)  
  //   - датчики (BME280), LCD, LED-индикация, кнопка/энкодер  
  
  // Термогвард VFO: >80°C → откат разгона, <60°C → возврат.  
  // В RX-фазе сам пропускает замер (ADC занят входом).  
  vfo_thermal_guard();  
  
  // Периодическая диагностика: водяной знак арены + состояние.  
  static uint32_t t_diag = 0;  
  if (millis() - t_diag > 10000) {  
    t_diag = millis();  
    Serial.printf("arena watermark: %u / %u B | state=%s | tone=%u | clk_sys=%lu Hz\n",  
                  (unsigned)mem_arena_watermark(),  
                  (unsigned)mem_arena_capacity(MEM_PHASE_RX),  
                  trx_state() == TRX_TX ? "TX" :  
                  trx_state() == TRX_RX ? "RX" : "SWITCHING",  
                  (unsigned)vfo_tone_active(),  
                  (unsigned long)vfo_clk_sys_hz());  
  }  
}
