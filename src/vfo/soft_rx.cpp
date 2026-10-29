// src/vfo/soft_rx.cpp  
// Квадратурный гетеродин для приёмника: 4-тактовая PIO-программа  
// выдаёт на 2 соседних пина последовательность Q,I = 01→11→10→00,  
// т.е. два меандра f/1 со сдвигом 90°.  
// Перенесено из RP2040_beacon/vfo_hardware.cpp (блок quad).  
#include "vfo.h"  
#include "../mem/mem_arena.h"  
#include "hardware/pio.h"  
#include "hardware/clocks.h"  
  
#define RX_IF_SHIFT_HZ  11000UL   // ПЧ приёмника: LO = f_rx − IF  
  
static const uint16_t pio_quad_instr[] = {  
  0xe001,  // set pins, 0b01: Q=1 I=0  
  0xe003,  // set pins, 0b11: Q=1 I=1  
  0xe002,  // set pins, 0b10: Q=0 I=1  
  0xe000,  // set pins, 0b00: Q=0 I=0  
};  
static const pio_program_t pio_quad = { pio_quad_instr, 4, -1, 0 };  
  
static PIO      s_pio = pio1;         // PIO1 — чтобы не делить pio0 с TX  
static uint     s_sm  = 0;  
static uint     s_off = 0;  
static int      s_pin_q = -1, s_pin_i = -1;   // соседние GPIO, из pins.h  
static bool     s_running = false;  
static uint32_t s_lo_hz = 0;  
  
static void quad_init() {  
  s_pin_q = 0; s_pin_i = 1;          // TODO: константы из pins.h  
  s_off = pio_add_program(s_pio, &pio_quad);  
  s_sm  = pio_claim_unused_sm(s_pio, true);  
  pio_sm_set_consecutive_pindirs(s_pio, s_sm, s_pin_q, 2, true);  
  pio_sm_config c = pio_get_default_sm_config();  
  sm_config_set_set_pins(&c, s_pin_q, 2);  
  sm_config_set_clkdiv(&c, 4.0f);  
  pio_sm_init(s_pio, s_sm, s_off, &c);  
}  
  
static void quad_start(uint32_t rx_freq_hz) {  
  // LO ниже частоты приёма на ПЧ — сигнал ложится в базовую полосу  
  // на +RX_IF_SHIFT_HZ, вдали от DC-дырки и 1/f-шума ADC.  
  uint64_t lo = (rx_freq_hz > RX_IF_SHIFT_HZ) ? (rx_freq_hz - RX_IF_SHIFT_HZ) : 1000;  
  // Период квадратуры = 4 такта → clkdiv = clk_sys / (4 * f_lo).  
  uint64_t clk = clock_get_hz(clk_sys);  
  float div = (float)clk / (4.0f * (float)lo);  
  if (div < 1.0f) div = 1.0f; else if (div > 65536.0f) div = 65536.0f;  
  pio_sm_set_clkdiv(s_pio, s_sm, div);  
  pio_sm_set_enabled(s_pio, s_sm, true);  
  s_running = true;  
  // Фактическая частота после квантования делителя — для точного NCO.  
  s_lo_hz = (uint32_t)((double)clk / (4.0 * (double)div));  
}  
  
static void quad_stop() {  
  pio_sm_set_enabled(s_pio, s_sm, false);  
  s_running = false;  
}  
static void quad_tone(uint8_t)  {}        // у LO банка нет — тон один  
static void quad_poll()         {}        // дизеринг не нужен: чистый clkdiv  
static uint32_t quad_hz()       { return s_lo_hz; }  
  
static const VfoEngine s_engine = {  
  quad_init, quad_start, quad_stop, quad_tone, quad_poll, quad_hz  
};  
const VfoEngine *vfo_engine_soft_rx() { return &s_engine; }
