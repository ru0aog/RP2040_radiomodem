#ifndef CAT_H  
#define CAT_H  
#include <Arduino.h>  
  
// CAT-интерфейс модема (Kenwood TS-2000 subset) на втором USB-CDC порту.  
// Портировано из RP2040_receiver/src/cat/fldigi_cat.cpp.  
void cat_init(void);                    // вызывать из setup() ДО Serial.begin()  
void cat_poll(void);                    // из loop()  
void cat_set_freq_hz(uint32_t hz);      // синхронизация при перестройке не из CAT  
  
#endif
