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
#include "src/vfo/soft_tx.h"     // soft_tx_debug()    
    
#include "src/mem/mem_arena.h"    
#include "src/rx/rx_capture.h"    // rx_capture_start()    
#include "src/rx/rx_dsp.h"        // rx_dsp_init/rx_dsp_poll    
#include "src/rx/rx_decoders.h"   // rx_decoders_init/poll    
#include "src/rsid/rsid.h"        // rsid_init/poll    
#include "src/modem/modem.h"      // modem_init/poll    
#include "src/rx/rx_usb_audio.h"    
#include "src/cat/cat.h"          // cat_init/poll — Kenwood TS-2000    
#include <Adafruit_TinyUSB.h>    
    
// ============================================================================    
// Перекрёстная синхронизация ядер.    
//  - core0_ready: Core 0 закончил инициализацию (включая UAC), Core 1 может    
//    входить в PLL-перенастройку quad LO.    
//  - core1_pll_done: Core 1 завершил стартовую перенастройку clk_sys.    
//    
// Порядок важен: usb_audio_init() идёт ДО core0_ready — на штатных 133 МГц    
// и пока Core 1 ещё ждёт флаг (никакой парковки Core 0). Внутри    
// usb_audio_init() есть принудительная реэнумерация USB (detach/attach);    
// если она попадёт под idleOtherCore или на разогнанную шину — CDC умирает    
// (был завис на "[UAC] Re-enumerating USB").    
// ============================================================================    
volatile bool core0_ready    = false;    
volatile bool core1_pll_done = false;    
    
void setup() {    
  Serial.begin(115200);    
    
  while (!Serial && millis() < 5000) delay(1);    
    
  cat_init();                        // CAT на этом же порту    
    
  Serial.println();    
  Serial.println("RP2040_radiomodem — полудуплексный цифровой радиомодем");    
  Serial.println("====================================================");    
  Serial.flush();    
    
  mem_arena_init();    
  Serial.printf("mem_arena: TRX pool %u B, SHARED %u B\n",    
                (unsigned)mem_arena_capacity(MEM_PHASE_RX),    
                (unsigned)mem_arena_capacity(MEM_PHASE_SHARED));    
    
  pins_init();    
  vfo_init();    
  trx_init(vfo_on_trx_switch);       // hook VFO на события RX/TX    
    
  rx_dsp_init();    
  rx_decoders_init();    
  rsid_init();    
  modem_init();    
    
  // UAC ДО отпускания Core 1: шина ещё 133 МГц, Core 0 никто не паркует —    
  // реэнумерация CDC/UAC проходит безопасно.    
  Serial.println("[DBG] before usb_audio");  Serial.flush();    
  usb_audio_init();    
  Serial.println("[DBG] after usb_audio");   Serial.flush();    
    
  vfo_debug_dump();                  // разовый снимок состояния    
    
  // Отпускаем Core 1: он перенастроит clk_sys → 337.5 МГц, паркуя Core 0    
  // через idleOtherCore. В это время мы в пустом цикле ожидания —    
  // безопасная точка парковки (USB-стек кратковременно встаёт, но хост    
  // это переживает; никакой реэнумерации в этот момент нет).    
  core0_ready = true;    
  while (!core1_pll_done) delay(1);    
    
  Serial.println("[DBG] setup done");    
  Serial.flush();    
}    
    
void loop() {    
    
  // --- Тест: TX на 3.65 МГц, перебор 4 тонов шагом 170 Гц каждые 500 мс ---    
  static bool     tx_started = false;    
  static uint32_t t_tone = 0;    
  static uint8_t  tone_idx = 0;    
    
  if (!tx_started && millis() > 3000) {    
    tx_started = true;    
    vfo_bank_set_grid(3650000UL, 170, 4);          // сетка ДО переключения    
    bool ok = trx_switch(TRX_TX, 3650000UL);    
    Serial.printf("[TEST] trx_switch(TX, 3650000) = %d\n", (int)ok);    
    if (ok) {    
      soft_tx_debug();    
      Serial.printf("[TEST] actual=%lu Hz clk_sys=%lu Hz\n",    
                    (unsigned long)vfo_actual_hz(),    
                    (unsigned long)vfo_clk_sys_hz());    
    }    
  }    
    
  // Перебор тонов — только в TX    
  if (tx_started && trx_state() == TRX_TX && millis() - t_tone > 500) {    
    t_tone = millis();    
    vfo_tone(tone_idx);    
    tone_idx = (tone_idx + 1) % 4;    
  }    
    
  // Возврат в RX через 10 с TX — проверка разгонной шины и перезахвата quad-LO    
  static uint32_t t_tx_start = 0;    
  if (tx_started && trx_state() == TRX_TX) {    
    if (!t_tx_start) t_tx_start = millis();    
    if (millis() - t_tx_start > 10000) {    
      t_tx_start = 0;    
      bool ok = trx_switch(TRX_RX, 14074000UL);    
      Serial.printf("[TEST] trx_switch(RX) = %d clk_sys=%lu\n",    
                    (int)ok, (unsigned long)vfo_clk_sys_hz());    
      tx_started = false;          // для повтора цикла — сбросить и дать restart    
    }    
  }    
    
  cat_poll();                        // CAT-команды (Kenwood TS-2000)    
  modem_poll();                      // Hayes-модем    
  rx_decoders_poll();                // декодеры/RSID    
  rsid_poll();    
  usb_audio_poll();    
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
  while (!core0_ready) tight_loop_contents();  // ждём инициализацию (вкл. UAC)    
    
  trx_poll_core1();                            // инициализация TRX на core1    
    
  // Старт RX-генератора при загрузке: квадратурный LO на f_rx - ПЧ.    
  // Внутри — PLL-скан и смена clk_sys под парковкой Core 0 (idleOtherCore).    
  vfo_on_trx_switch(TRX_RX, 14074000);    
    
  core1_pll_done = true;             // Core 0 может выйти из ожидания    
    
  delay(100);                        // старт SM    
  vfo_quad_debug();                  // [LODBG] clkdiv/enabled/lo_hz    
  cat_set_freq_hz(14074000);         // зеркало FA для ответов CAT (s_rx_hz)    
  rx_dsp_start(14074000);            // NCO = f_rx - vfo_quad_lo_hz()    
  rx_capture_start();                // DMA запускаем ПОСЛЕ настройки NCO    
}    
    
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