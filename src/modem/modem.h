// src/modem/modem.h  
enum ModemState { MODEM_COMMAND, MODEM_DATA, MODEM_ONLINE };  
void modem_init(void);  
void modem_poll(void);                    // из loop(), разбор входного потока  
bool modem_cmd_exec(const char* line);    // общий для USB CDC и локального CLI  
void modem_rx_frame(const uint8_t* data, uint16_t len); // моды → хост
