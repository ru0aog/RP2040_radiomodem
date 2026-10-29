// src/pins.h  
// Единая карта GPIO RP2040_radiomodem.  
// Сведено из RP2040_receiver (src/vfo/vfo_rp2040.h + rx_capture) и  
// RP2040_beacon (vfo_hardware.h: RX_ADC_PIN_*, ENC_*, VFO_ENC_SW_PIN,  
// pin_freq_out, subband_pins[4]). ВСЕ пины устройства — только отсюда.  
#pragma once  
#include <Arduino.h>  
  
// ---------- ADC-вход приёмника (RX) ----------  
// FIFO: [31:16]=Q (ADC1), [15:0]=I (ADC0); 48 МГц / 192 = 250 ksps  
// суммарно = 125 ksps на канал. LO квадратурный — см. QUAD-пины ниже.  
#define PIN_ADC_I        26    // GPIO26 / ADC0 — канал I  
#define PIN_ADC_Q        27    // GPIO27 / ADC1 — канал Q  
  
// ---------- Квадратурный LO приёмника (soft_rx, PIO1) ----------  
// Соседние GPIO, sm_config_set_set_pins(PIN_QUAD_I, 2):  
// инструкция `set pins` пишет биты [1:0] на пины base и base+1:  
//   bit0 -> base (I), bit1 -> base+1 (Q).  
// Последовательность 01->11->10->00 даёт: I = 1,1,0,0; Q = 0,1,1,0 —  
// Q отстаёт от I на 90°, что и требуется смесителю.  
// Порядок I/Q в паре можно поменять местами — для приёма важен только  
// сдвиг 90°, знак сдвига меняет верхнюю/нижнюю боковую при демодуляции.  
#define PIN_QUAD_I        0    // базовый пин SET-группы (младший!)  
#define PIN_QUAD_Q        1
  
// ---------- TX-выход (soft_tx, PIO0 меандр) ----------  
// Аналог pin_freq_out маяка: пин вынесен в переменную, чтобы можно  
// было переназначать из SET.TXT без пересборки.  
extern int pin_freq_out;  
#define PIN_VFO_TX       (pin_freq_out)   // инициализировать в pins_init()  
  
// ---------- ДПФ/субдиапазонные фильтры ----------  
// 4-битный код диапазона (subband_pins[0..3], vfo_subband_apply()).  
extern int subband_pins[4];  
  
// ---------- Энкодер перестройки ----------  
#define PIN_ENC_A        20  
#define PIN_ENC_B        21  
#define PIN_ENC_SW       19    // нажатие = GND  
  
// ---------- Пользовательская кнопка ----------  
#define PIN_USR_BUTTON   22    // из receiver: PIN_USR_BUTTON  
  
// ---------- I2C (Si5351, BME280, LCD) ----------  
#define PIN_I2C_SDA       4  
#define PIN_I2C_SCL       5  
  
// ---------- PTT / ключ ----------  
#define PIN_PTT_LED      25    // встроенный LED = индикация TX  
  
// Инициализация переменных пинов значениями по умолчанию.  
// Вызывать из setup() ПЕРЕД vfo_init() — движки читают пины при init.  
void pins_init();
