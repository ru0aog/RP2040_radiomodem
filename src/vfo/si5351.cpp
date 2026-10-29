// src/vfo/si5351.cpp  
// Бэкенд внешнего синтезатора Si5351 (I2C). ЗАГЛУШКА:  
// интерфейс зафиксирован, аппаратная реализация — позже.  
// План: CLK0 = TX-несущая, CLK1/CLK2 = квадратурный LO приёмника  
// (PLL на одну частоту, фазовый сдвиг 90° через CLKx_PHOFF).  
#include "vfo.h"  
  
#define SI5351_I2C_ADDR  0x60  
static bool s_present = false;  
  
static void si_init() {  
  // TODO: Wire.beginTransmission(SI5351_I2C_ADDR); s_present = (rc==0);  
  s_present = false;  
}  
static void si_start(uint32_t freq_hz) {  
  if (!s_present) return;  
  // TODO: si5351 pll_setup + multisynth + output_enable(CLK0)  
  (void)freq_hz;  
}  
static void si_stop() {  
  if (!s_present) return;  
  // TODO: output_disable all  
}  
static void si_tone(uint8_t index) {  
  // TODO: пересчёт multisynth под freqs_mhz[index] из vfo_bank()  
  (void)index;  
}  
static void si_poll() {}                    // Si5351 автономен, сервиса нет  
static uint32_t si_hz() { return 0; }       // TODO: вернуть запрограммированную  
  
static const VfoEngine s_engine = {  
  si_init, si_start, si_stop, si_tone, si_poll, si_hz  
};  
const VfoEngine *vfo_engine_si5351() { return &s_engine; }
