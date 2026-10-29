// src/modem/modem.h  
// ============================================================================  
// Hayes-совместимый радиомодем: AT-команды по USB CDC, состояния  
// COMMAND/ONLINE/DATA, S-регистры. Аналогично KISS-TNC, но текстовый  
// профиль Hayes. Управление — только через vfo.h / trx_control / modes.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
#include "modem_cfg.h"

enum ModemState { MODEM_COMMAND, MODEM_DATA, MODEM_ONLINE };  
  
void modem_init(void);  
void modem_poll(void);                 // из loop(), разбор входного потока, Core 0  
void modem_rx_byte(uint8_t b);         // байт от транспорта (USB CDC/UART)  
  
// Исполнение AT-строки из произвольного источника (общий вызов для  
// локального CLI). Буфер завершается '\0', без CR.  
void modem_cmd_exec(const char *line);  
  
// Точки выхода (modem_data.cpp):  
void modem_answer(const char *s);      // ответ хосту: "OK"/"ERROR"/коды

