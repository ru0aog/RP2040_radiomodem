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
// Core 0 (приложение): CAT/CLI, декодеры + RSID, планировщик, Hayes-модем,  
//   датчики, LCD. Единственный путь в эфир — trx_switch().  
// ============================================================================  
#include "src/pins.h"  
#include "src/trx/trx_control.h"  
#include "src/vfo/vfo.h"  
#include "src/vfo/soft_rx.h"      // soft_rx_quad_debug() — разовая диагностика  
#include "src/mem/mem_arena.h"  
#include "src/rx/rx_capture.h"    // rx_capture_start() — КАРКАС  
#include "src/rx/rx_dsp.h"        // rx_dsp_init/rx_dsp_poll — КАРКАС  
#include "src/rx/rx_decoders.h"   // rx_decoders_init/poll — КАРКАС  
#include "src/rsid/rsid.h"        // rsid_init/poll — КАРКАС  
#include "src/usb/usb_audio.h"    // usb_audio_init/poll — КАРКАС  
#include "src/modem/modem.h"      // modem_init/poll — КАРКАС
#include <Adafruit_TinyUSB.h>
  
// ============================================================================  
// Перекрёстная синхронизация ядер.  
// Core 1 ждёт, пока Core 0 закончит setup() (пины, VFO, trx — движки  
// зарегистрированы, ресурсы захвачены), и только потом стартует quad LO.  
// ============================================================================  
volatile bool core0_ready = false;  
  
void setup() {  
  Serial.begin(115200);  
 
  while (!Serial && millis() < 5000) delay(1);

  Serial.println();  
  Serial.println("RP2040_radiomodem — полудуплексный цифровой радиомодем");  
  Serial.println("====================================================");  
  
  mem_arena_init();  
  Serial.printf("mem_arena: TRX pool %u B, SHARED %u B\n",  
                (unsigned)mem_arena_capacity(MEM_PHASE_RX),  
                (unsigned)mem_arena_capacity(MEM_PHASE_SHARED));  
  
  pins_init();  
  vfo_init();  
  trx_init(vfo_on_trx_switch);      // hook VFO на события RX/TX  
  
  // ---- Каркас приложения (заглушки — печатают [STUB], железа не трогают) ----  
  rx_dsp_init();  
  rx_decoders_init();  
  rsid_init();  
  usb_audio_init();  
  modem_init();  
  // --------------------------------------------------------------------------  
  
  vfo_debug_dump();                 // разовый снимок состояния  
  
  core0_ready = true;               // Core 1 может стартовать RX-генератор  
  delay(200);                       // дать Core 1 дойти до quad_start  
}  
  
void loop() {  
  modem_poll();                     // Hayes-модем (КАРКАС)  
  rx_decoders_poll();               // декодеры/RSID (КАРКАС)  
  rsid_poll();  
  usb_audio_poll();  
  vfo_thermal_guard();  
  
  /*  
  // Тест передачи: каждые 5 с — 1 с передачи тона 0 на 14074 кГц.  
  // ВКЛЮЧАТЬ только для проверки TX; мешает отладке RX-конвейера.  
  static uint32_t t_test = 0;  
  if (millis() - t_test > 5000) {  
    t_test = millis();  
    vfo_bank_set_grid(14074000, 1000, 4);  
    Serial.printf("tx test: switch=%d\n", (int)trx_switch(TRX_TX, 14074000));  
    vfo_tone(0);  
    soft_tx_debug();  
    delay(1000);  
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
  static uint32_t t_rxdbg = 0;  
  if (millis() - t_rxdbg > 30000) {  
    t_rxdbg = millis();  
    rx_dsp_debug();  
  }
    Serial.printf("cap_isr=%lu drop=%lu(scratch=%lu qfull=%lu busy=%lu) ",  
                  (unsigned long)rx_cap_isr_count,  
                  (unsigned long)rx_capture_dropped(),  
                  (unsigned long)rx_drop_scratch,  
                  (unsigned long)rx_drop_queue_full,  
                  (unsigned long)rx_drop_busy);
  }  
}  
  
// ============================================================================  
// Core 1 — хозяин реального времени  
// ============================================================================  
void setup1() {  
  while (!core0_ready) tight_loop_contents();  // ждём setup() на Core 0  
  
  trx_poll_core1();                            // инициализация TRX на core1  
  
  // Старт RX-генератора сразу при загрузке: квадратурный LO (soft_rx, pio1)  
  // на f_rx - RX_IF_SHIFT_HZ. Без явного вызова квад остаётся выключенным  
  // до первого trx_switch.  
  vfo_on_trx_switch(TRX_RX, 14074000);  
  delay(100);                       // пересинхронизация USB CDC + старт SM  
  soft_rx_quad_debug();             // [LODBG] clkdiv/enabled/lo_hz

  rx_dsp_start(14074000);           // NCO = f_rx - vfo_quad_lo_hz()  
  rx_capture_start();               // DMA запускаем ПОСЛЕ настройки NCO
}  
  
void loop1() {  
  trx_poll_core1();  
  if (trx_state() == TRX_RX) {  
    rx_dsp_poll();                  // выкачка DMA-блоков + DSP + RSID-сервис  
  } else {  
    vfo_poll();  
  }  
}