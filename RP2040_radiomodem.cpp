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
//   датчики, LCD, LED.  
// ============================================================================  
  
#include <Arduino.h>  
#include <Adafruit_TinyUSB.h>  
  
#include "src/pins.h"  
#include "src/mem/mem_arena.h"  
#include "src/filesys/file_manager.h"  
#include "src/cat/cat.h"  
#include "src/trx/trx_control.h"  
#include "src/trx/tx_job.h"  
#include "src/vfo/vfo.h"  
#include "src/vfo/quad_vfo/quad_vfo.h"  
#include "src/rx/rx_dsp.h"  
#include "src/rx/rx_capture.h"  
#include "src/rx/rx_usb_audio.h"  
#include "src/rx/rx_decoders.h"  
#include "src/rsid/rsid.h"  
#include "src/modem/modem.h"  
  
// --- межъядерные флаги -------------------------------------------------------  
volatile bool core0_ready  = false;  
volatile bool core1_pll_done = false;  

// ============================================================================  
// Флаг-протокол парковки Core 0 (объекты в src/trx/trx_control.cpp).  
// Core 1 запрашивает уход Core 0 в RAM-цикл на время переключения clk_sys.  
// ============================================================================  
extern volatile uint32_t core0_park_req;  
extern void core0_park_poll(void);           // НЕ static в trx_control.cpp

// ============================================================================  
// Core 0 — setup  
// ============================================================================  
void setup() {  
  // USB-стек и диск — до Serial.begin, чтобы композит (CDC+MSC+UAC)  
  // поднялся целиком при энумерации.  
  init_usb_msc_interface();          // регистрация MSC + колбэки  
  init_file_manager();               // FAT12 RAM-диск + загрузка слота из flash  
                                     // (чтение XIP — flash_safe не нужен;  
                                     //  если слот пуст, запись отложится  
                                     //  через state_save_pending)  
  usb_audio_init();                  // UAC-интерфейс в композите  
  
  Serial.begin(115200);  
  
  pins_init();  
  mem_arena_init();  
  
  Serial.println();  
  Serial.println("RP2040_radiomodem — полудуплексный цифровой радиомодем");  
  Serial.println("====================================================");  
  
  rx_dsp_init();  
  rx_decoders_init();  
  rsid_init();  
  modem_init();                      // Hayes-модем (skeleton)  
  vfo_init();  
  
  cat_init();                        // CAT + CLI на общем CDC-порту  
  
  // Запускаем Core 1: setup1() сделает file_manager_core1_init()  
  // (flash_safe_execute_core_init + flash_safe_ready), затем  
  // vfo_on_trx_switch() с разгоном PLL — Core 0 в это время  
  // ждёт ниже и обслуживает парк-протокол.  
  core0_ready = true;  
  while (!core1_pll_done) {  
    core0_park_poll();               // handshake для парковки при смене PLL  
    tight_loop_contents();  
  }  
  
  Serial.println("[DBG] setup done");  
}  
  
// ============================================================================  
// Core 0 — loop  
// ============================================================================  
void loop() {  
  cat_poll();                        // CAT (';') + CLI ('\n') на одном порту  
  tx_job_poll();                     // исполнение TX-заявки  
  check_and_handle_pc_changes();     // запись RAM-диска во flash + парсеры  
  usb_audio_poll();                  // выдача PCM в UAC  
}  
  
// ============================================================================  
// Core 1 — setup  
// ============================================================================  
void setup1() {  
  
  file_manager_core1_init();   // flash_safe_execute_core_init + flash_safe_ready=true  
                               // СТРОГО первой строкой: пока DSP не крутится,  
                               // парковка инициализируется безопасно.  
  
  while (!core0_ready) tight_loop_contents();  
  
  // Стартовый RX: VFO-hook → quad_vfo → set_clk_sys → разгон.  
  // Паркует Core 0 флаг-протоколом (Core 0 уже в while(!core1_pll_done)  
  // с core0_park_poll — handshake обслуживается).  
  vfo_on_trx_switch(TRX_RX, 3602000);  
  
  core1_pll_done = true;             // Core 0 выходит из ожидания  
  
  delay(100);                        // старт SM  
  vfo_quad_debug();                  // [LODBG] clkdiv/enabled/lo_hz  
  cat_set_freq_hz(3602000);          // зеркало FA для ответов CAT (s_rx_hz)  
  rx_dsp_set_demod(RXD_DEMOD_USB);   // демод до старта DSP  
  rx_dsp_start(3602000);             // NCO = f_rx - vfo_quad_lo_hz()  
  rx_capture_start();                // DMA запускаем ПОСЛЕ настройки NCO  
}  
  
// ============================================================================  
// Core 1 — loop (реальное время)  
// ============================================================================  
volatile uint32_t dbg_l1 = 0, dbg_poll_in = 0, dbg_poll_out = 0;  
  
void loop1() {  
  ++dbg_l1;  
  trx_poll_core1();  
  if (trx_state() == TRX_RX) {  
    ++dbg_poll_in;  
    rx_dsp_poll();  
    ++dbg_poll_out;  
  } else {  
    vfo_poll();  
  }  
}