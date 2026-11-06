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
#include <pico/flash.h>
#include "src/mem/mem_arena.h"  
#include "src/rx/rx_capture.h"  
#include "src/rx/rx_dsp.h"  
#include "src/rx/rx_decoders.h"  
#include "src/rx/rx_usb_audio.h"      // usb_audio_init — проверить по факту  
#include "src/rsid/rsid.h"  
#include "src/trx/trx_control.h"
#include "src/vfo/vfo.h"
#include "src/vfo/soft_tx.h"
#include "src/vfo/quad_vfo/quad_vfo.h"
#include "src/cat/cat.h"
#include "src/modem/modem.h"
#include "src/filesys/file_manager.h"
#include "src/trx/tx_job.h"
  
// ============================================================================  
// Межъядерная парковка — единственный владелец протокола: src/trx/core_sync.  
// Core 0 обслуживает core_sync_poll() в loop(), while(!core1_pll_done)  
// и внутри trx_switch(); Core 1 — core_sync_core1_poll() в loop1().  
// ============================================================================  
#include "src/trx/core_sync.h"
  
// ============================================================================  
// Глобальные  
// ============================================================================  
volatile bool core0_ready    = false;  
volatile bool core1_pll_done = false;  
  
// ============================================================================  
// SETUP — Core 0  
// ============================================================================  
void setup() {

  init_usb_msc_interface();   // до маунта TinyUSB — иначе хост диск не увидит
  usb_audio_init();            // регистрируем UAC-интерфейс ДО mount

  Serial.begin(115200);

  while (!Serial && millis() < 5000) delay(1);

  cat_init();                     // выведет "[CAT] init: shared CDC port" и armed cat_enabled  
  init_file_manager();            // загрузка образа из Flash или default-диск
  Serial.println();
  Serial.println("RP2040_radiomodem — полудуплексный цифровой радиомодем");  
  Serial.println("====================================================");  
  Serial.flush();  
  
  pinMode(LED_BUILTIN, OUTPUT);  
  
  vfo_init();                                // движки, банк тонов, хук  
  trx_init(vfo_on_trx_switch);               // mem_arena_init + привязка VFO  
  Serial.printf("mem_arena: TRX pool %u B, SHARED %u B\n",  
                (unsigned)mem_arena_capacity(MEM_PHASE_TX),  
                (unsigned)mem_arena_capacity(MEM_PHASE_SHARED));  
  
  rx_dsp_init();  
  Serial.println("[RX] dsp init");  
  rx_decoders_init();  
  Serial.println("[DEC] init");  
  rsid_init();  
  Serial.println("[RSID] init (verbatim port)");  
  modem_init();  
  Serial.println("[STUB] modem_init: Hayes core skeleton");  
  
  Serial.printf("[VFO] clk_sys=%lu Hz  tone=%u  xosc=%llu Hz\n",  
                (unsigned long)vfo_clk_sys_hz(),  
                (unsigned)vfo_tone_active(),  
                (unsigned long long)vfo_xosc_hz());  
  Serial.flush();  
  
  core_sync_init();                          // ДО запуска Core 1  
  core0_ready = true;                        // запуск loop1() на Core 1
  
  // Ждём окончания PLL-окна quad-движка (Core 1, setup1).  
  // ОБЯЗАТЕЛЬНО: обслуживаем park-протокол — Core 1 вызывает  
  // set_clk_sys с флаг-парковкой; без poll() парк уйдёт в PARK TIMEOUT.  
  // В этой точке на Core 0 — пустой цикл без USB: живой TinyUSB-стек  
  // во время парковки кладёт handshake (был завис на "before park").  
  while (!core1_pll_done) {    
      core_sync_poll();    
      tight_loop_contents();    
  }
  file_manager_set_flash_safe_ready(true);              // ← добавить  
  Serial.println("[FM] core1 init: flash writes ENABLED");
  
  Serial.println("[DBG] setup done");  
  Serial.flush();  
} 
  
volatile uint32_t dbg_l1 = 0, dbg_poll_in = 0, dbg_poll_out = 0;  
extern volatile uint32_t dbg_c1_stage;   // определён в trx_control.cpp (стадии 10-13)
// здесь пишем стадии 1-4: 1=park, 2=trx, 3=dsp, 4=vfo, 0=idle
volatile uint32_t dbg_c0_stage = 0;   // 1=cat 2=txjob 3=dec 4=rsid 5=fm 6=usbaudio 0=idle

// ============================================================================  
// LOOP — Core 0  
// ============================================================================  
void loop() {  
  core_sync_poll();           // ПЕРВОЙ строкой — обслуживание парковки Core 0

  // Маячок живости: 2 Гц. Замёрз — прошивка умерла, мигает — мёртв только вывод.  
  static uint32_t t_blink = 0;  
  static bool     led = false;  
  if (millis() - t_blink > 250) {  
    t_blink = millis();  
    led = !led;  
    digitalWrite(LED_BUILTIN, led);  
  }  
  
  // --- Тест: TX на 3.65 МГц, перебор 4 тонов шагом 170 Гц каждые 500 мс ---    
  // ОТКЛЮЧЁН для проверки RSID: RX стоит непрерывно, без прерываний на TX.  
  // Чтобы вернуть тест — поставить TX_TEST_ENABLED в true.  
  static constexpr bool TX_TEST_ENABLED = false;  
  static bool     tx_started = false;    
  static uint32_t t_tone = 0;    
  static uint8_t  tone_idx = 0;    
    
  if (TX_TEST_ENABLED && !tx_started && millis() > 3000) {    
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
  
  if (tx_started && trx_state() == TRX_TX &&  
      millis() - t_tone > 500) {  
    t_tone = millis();  
    vfo_tone(tone_idx);  
    Serial.printf("[TEST] tone %u -> %lu Hz\n",  
                  (unsigned)tone_idx,  
                  (unsigned long)vfo_bank_tone_hz(tone_idx));  
    tone_idx = (tone_idx + 1) & 3;  
  }  
  
  // --- Возврат в RX через 10 с (только при включённом TX-тесте) ---    
  static uint32_t t_back = 0;    
  if (TX_TEST_ENABLED && tx_started && trx_state() == TRX_TX) {
    if (t_back == 0) t_back = millis();  
    if (millis() - t_back > 10000) {  
      t_back = 0;  
      bool ok = trx_switch(TRX_RX, 3602000UL);    
      Serial.printf("[TEST] trx_switch(RX, 3602000) = %d\n", (int)ok);
    }  
  }  
  
  dbg_c0_stage = 1;  cat_poll();                     // разбор команд с основного CDC, разделитель ';'  
  dbg_c0_stage = 2;  tx_job_poll();  
  dbg_c0_stage = 3;  rx_decoders_poll();  
  dbg_c0_stage = 4;  rsid_poll();                 // печать "# RSID ..." + автопереключение моды  
  dbg_c0_stage = 5;  check_and_handle_pc_changes();  
  dbg_c0_stage = 6;  usb_audio_poll();            // выкачиваем аудио-очередь rx_dsp -> UAC  
  dbg_c0_stage = 0;
  
  // --- Пульс-строка раз в 10 сек ---  
  static uint32_t t_stat = 0;  
  if (millis() - t_stat > 10000) {  
    t_stat = millis();  
    Serial.printf("arena watermark: %u / %u B | state=%s | tone=%u | "  
                  "clk_sys=%lu Hz\n",  
                  (unsigned)mem_arena_watermark(),  
                  (unsigned)mem_arena_capacity(MEM_PHASE_TX),  
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
    core_sync_stats st;  
    core_sync_get_stats(&st);  
    Serial.printf("sync: p0 req=%lu ack=%lu to=%lu dead=%lu | "  
                  "p1 req=%lu ack=%lu to=%lu dead=%lu | "  
                  "c1stage=%lu l1=%lu pin=%lu pout=%lu\n",  
                  (unsigned long)st.park0_requests, (unsigned long)st.park0_ack,  
                  (unsigned long)st.park0_timeouts, (unsigned long)st.park0_dead_initiator,  
                  (unsigned long)st.park1_requests, (unsigned long)st.park1_ack,  
                  (unsigned long)st.park1_timeouts, (unsigned long)st.park1_dead_initiator,  
                  (unsigned long)dbg_c1_stage,  
                  (unsigned long)dbg_l1, (unsigned long)dbg_poll_in,  
                  (unsigned long)dbg_poll_out); 
  }  

  
  // --- sync: каждые 500 мс (диагностика зависания) ---  
  static uint32_t t_sync = 0;  
  if (millis() - t_sync > 150) {  
    t_sync = millis();  
    core_sync_stats st;  
    core_sync_get_stats(&st);  
    Serial.printf("sync: p0 req=%lu ack=%lu to=%lu dead=%lu | "  
                  "p1 req=%lu ack=%lu to=%lu dead=%lu | "  
                  "c0=%lu c1=%lu l1=%lu pin=%lu pout=%lu\n",  
                  (unsigned long)st.park0_requests, (unsigned long)st.park0_ack,  
                  (unsigned long)st.park0_timeouts, (unsigned long)st.park0_dead_initiator,  
                  (unsigned long)st.park1_requests, (unsigned long)st.park1_ack,  
                  (unsigned long)st.park1_timeouts, (unsigned long)st.park1_dead_initiator,  
                  (unsigned long)dbg_c0_stage,  
                  (unsigned long)dbg_c1_stage,  
                  (unsigned long)dbg_l1, (unsigned long)dbg_poll_in,  
                  (unsigned long)dbg_poll_out);  
  }  

}
  
// ============================================================================  
// SETUP1 — Core 1  
// ============================================================================  
void setup1() {  
 
  while (!core0_ready) tight_loop_contents();  

  // Стартовый RX: VFO-hook → quad_vfo → set_clk_sys → разгон 337.5 МГц.  
  // Паркует Core 0 флаг-протоколом (Core 0 уже в while(!core1_pll_done)  
  // с core0_park_poll — handshake обслуживается).  
  vfo_on_trx_switch(TRX_RX, 3602000);    
    
  core1_pll_done = true;             // Core 0 выходит из ожидания    
    
  delay(100);                        // старт SM    
  vfo_quad_debug();                  // [LODBG] clkdiv/enabled/lo_hz    
  cat_set_freq_hz(3602000);          // зеркало FA для ответов CAT (s_rx_hz)    
  rx_dsp_start(3602000);             // NCO = f_rx - vfo_quad_lo_hz()
  rx_dsp_set_demod(RXD_DEMOD_USB);   // или соответствующее имя API
  rx_capture_start();                // DMA запускаем ПОСЛЕ настройки NCO  
}  
  
void loop1() {  
  ++dbg_l1;  
  dbg_c1_stage = 1;  
  core_sync_core1_poll();  
  dbg_c1_stage = 2;  
  trx_poll_core1();   
  if (trx_state() == TRX_RX) {    
    dbg_c1_stage = 3;  
    ++dbg_poll_in;    
    rx_dsp_poll();    
    ++dbg_poll_out;    
  } else {    
    dbg_c1_stage = 4;  
    vfo_poll();    
  }    
  dbg_c1_stage = 0;  
}