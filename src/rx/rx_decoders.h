// src/rx/rx_decoders.h  
void rx_decoders_init(void);  
void rx_decoders_select(const ModeIface* m); // активная мода → поток  
void rx_decoders_feed(int16_t mono);         // вызов из rx_dsp на каждый сэмпл  
void rx_decoders_on_rsid(uint8_t code);      // колбэк от rsid-модуля
