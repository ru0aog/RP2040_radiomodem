// src/pins.cpp  
#include "pins.h"  
  
// Значения по умолчанию; переопределяются из Flash (SET.TXT)  
// файловым менеджером до pins_init().  
int pin_freq_out   = 2;                    // TX-меандр  
int subband_pins[4] = { 6, 7, 8, 9 };      // ДПФ-код  
  
void pins_init() {  
  pinMode(PIN_QUAD_I, OUTPUT);  
  pinMode(PIN_QUAD_Q, OUTPUT);  
  pinMode(pin_freq_out, OUTPUT);  
  for (int i = 0; i < 4; i++) pinMode(subband_pins[i], OUTPUT);  
  pinMode(PIN_PTT_LED, OUTPUT);  
}
