// src/pins.cpp  
// Определения переменных пинов и их инициализация.  
// Значения по умолчанию; переопределение из Flash (SET.TXT) —  
// когда появится файловый менеджер, до pins_init().  
#include "pins.h"  
  
int pin_freq_out       = 2;              // TX-меандр (soft_tx, PIO0)  
int subband_pins[4]    = { 6, 7, 8, 9 }; // ДПФ/субдиапазонный код  
  
void pins_init() {  
  // Квадратурный LO приёмника: направление окончательно выставит  
  // soft_rx через pio_sm_set_consecutive_pindirs — здесь только  
  // подготовка, чтобы пины не висели в воздухе.  
  pinMode(PIN_QUAD_I, OUTPUT);  
  pinMode(PIN_QUAD_Q, OUTPUT);  
  digitalWrite(PIN_QUAD_I, LOW);  
  digitalWrite(PIN_QUAD_Q, LOW);  
  
  // TX-выход: режим пина PIO выставит soft_tx_init (pio_gpio_init).  
  pinMode(pin_freq_out, OUTPUT);  
  digitalWrite(pin_freq_out, LOW);       // несущая выключена на старте  
  
  // ДПФ/субдиапазонные пины.  
  for (int i = 0; i < 4; i++) {  
    pinMode(subband_pins[i], OUTPUT);  
    digitalWrite(subband_pins[i], LOW);  
  }  
  
  pinMode(PIN_PTT_LED, OUTPUT);  
  digitalWrite(PIN_PTT_LED, LOW);  
  
  // Энкодер и кнопка — входы с подтяжкой (замыкание на GND).  
  pinMode(PIN_ENC_A, INPUT_PULLUP);  
  pinMode(PIN_ENC_B, INPUT_PULLUP);  
  pinMode(PIN_ENC_SW, INPUT_PULLUP);  
  pinMode(PIN_USR_BUTTON, INPUT_PULLUP);  
  
  // I2C инициализирует Wire.begin() в модуле датчиков — здесь не трогаем.  
}