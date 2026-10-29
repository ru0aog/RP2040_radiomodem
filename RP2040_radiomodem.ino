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
#include <Adafruit_TinyUSB.h>   // подтягивает реализацию USB CDC → Serial  
#include "src/pins.h"  
#include "src/mem/mem_arena.h"  
#include "src/trx/trx_control.h"  
#include "src/vfo/vfo.h"  
#include "src/vfo/soft_rx.h"    // soft_rx_quad_debug() — разовая диагностика  
  
// ============================================================================  
// РУКОПОЖАТИЕ ЯДЕР  
// ============================================================================  
// setup1() на Core 1 может стартовать РАНЬШЕ, чем setup() на Core 0  
// закончит vfo_init()/trx_init(). Без барьера vfo_on_trx_switch уйдёт  
// по s_eng_rx == nullptr и молча пропадёт — типичная причина «тишины».  
static volatile bool core0_ready = false;  
  
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
  
  pins_init();          // карта GPIO: квад, TX-выход, ДПФ, энкодер  
  vfo_init();           // регистрация движков: soft_rx / soft_tx / si5351  
  trx_init(vfo_on_trx_switch);   // mem_arena_init() + hook  
  
  Serial.printf("mem_arena: TRX pool %u B, SHARED %u B\n",  
                (unsigned)mem_arena_capacity(MEM_PHASE_RX),  
                (unsigned)mem_arena_capacity(MEM_PHASE_SHARED));  
  vfo_debug_dump();  
  
  // Публиковать ПОСЛЕДНИМ — Core 1 ждёт именно этот флаг.  
  core0_ready = true;  
}  
  
// ============================================================================  
// Core 1 — реальное время.  
// Тракты работают только когда состояние стабильно; во время  
// TRX_SWITCHING оба заглушены.  
// ============================================================================  
void setup1() {  
  // Дождаться инициализации Core 0: pins/vfo/trx должны быть готовы,  
  // иначе старт квада попадёт по незарегистрированным движкам.  
  while (!core0_ready) tight_loop_contents();  
  
  // RX — стартовое состояние. Конвейер (пока заглушка) стартует на  
  // этом же ядре, где будет крутиться его poll-цикл.  
  rx_pipeline_start();  
  
  // Явный старт квадратурного LO: trx_switch(TRX_RX) отсекается как  
  // "уже в состоянии", поэтому дёргаем hook напрямую.  
  // Частота приёма 14074 кГц → физический LO ~14063 кГц на GP0/GP1.  
  vfo_on_trx_switch(TRX_RX, 14074000);  
  
  // Разовая диагностика SM после перестройки clk_sys  
  // (delay нужен и для пересинхронизации USB CDC после смены шины).  
  delay(100);  
  soft_rx_quad_debug();  
}  
  
void loop1() {  
  trx_poll_core1();      // обработать pending trx_switch() с Core 0  
  
  if (trx_state() == TRX_RX) {  
    // TODO(src/rx): rx_capture pump → rx_dsp → audio/decoder queues  
  } else {  
    // TODO(src/tx): symbol timing → modulator → vfo_tone()  
  }  
  
  // Сервис активного VFO-движка: дизеринг clkdiv в TX-фазе  
  // (soft_tx_poll), у квадратуры и Si5351 — пустой poll.  
  vfo_poll();  
}  
  
// ============================================================================  
// Core 0 — приложение.  
// ============================================================================  
void loop() {



// Временный тест в loop(): каждые 5 с — 1 с передачи тона 0 на 14074 кГц.  
// Ищите несущую на SDR около 14.074 МГц (см. actual_hz в логе).  
static uint32_t t_test = 0;  
if (millis() - t_test > 5000) {  
  t_test = millis();  
  
  vfo_bank_set_grid(14074000, 1000, 4);   // 4 тона: 14074/14075/14076/14077 кГц  
  
  bool ok = trx_switch(TRX_TX, 14074000); // → stop RX → arena → vfo_on_trx_switch  
  Serial.printf("[TEST] switch TX: %s\n", ok ? "OK" : "FAIL");  
  if (!ok) return;                        // таймаут/занято — пропускаем цикл  
  
  vfo_tone(0);                            // тон 0 = несущая 14074 кГц  
  delay(1000);                            // держим 1 с — меандр на GP2  
  vfo_tone(VFO_TONE_NONE);                // несущая снята  
  
  Serial.printf("[TEST] actual=%lu Hz, clk_sys=%lu Hz\n",  
                (unsigned long)vfo_actual_hz(),  
                (unsigned long)vfo_clk_sys_hz());  
  
  trx_switch(TRX_RX, 14074000);           // обратно на квадратурный LO  
}




  // TODO(src/cat): Kenwood TS-2000 CAT-парсер  
  //   - команда TX/RX → trx_switch(TRX_TX/TRX_RX, freq)  
  //   - установка частоты → vfo_bank_set_*()  
  //   - выбор режима → подключение модулятора/демодулятора  
  //  
  // TODO(src/app):  
  //   - планировщик передач (карты/окна из scheduler.h маяка)  
  //   - декодеры + RSID (очереди из loop1, без блокировок)  
  //   - файловая система / USB-MSC (SET.TXT: XOSC_PPB, пины, расписание)  
  //   - датчики (BME280), LCD, LED-индикация, кнопка/энкодер  
  
  // Термогвард VFO: >80°C → откат разгона, <60°C → возврат.  
  // В RX-фазе сам пропускает замер (ADC занят входом).  
  vfo_thermal_guard();  
  
  // Тест передачи ОТКЛЮЧЁН — только приём.  
  // Раскомментировать для проверки TX на SDR:  
  /*  
  static uint32_t t_test = 0;  
  if (millis() - t_test > 5000) {  
    t_test = millis();  
    vfo_bank_set_grid(14074000, 1000, 4);   // 4 тона по 1 кГц от 14074 кГц  
    trx_switch(TRX_TX, 14074000);  
    vfo_tone(0);  
    delay(1000);  
    vfo_tone(VFO_TONE_NONE);  
    trx_switch(TRX_RX, 14074000);  
    Serial.printf("tx test done, actual=%lu Hz\n",  
                  (unsigned long)vfo_actual_hz());  
  }  
  */  
  
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