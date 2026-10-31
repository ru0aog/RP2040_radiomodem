void rtty_rx_init(uint32_t center_hz);   // центр RSID → mark/space ±85 Гц  
void rtty_rx_push(int16_t sample);       // моно-AF 15625 sps  
void rtty_rx_stop(void);                 // сброс при смене моды
