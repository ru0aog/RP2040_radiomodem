// src/modem/modem_cmd.h — парсер и исполнение AT-команд. КАРКАС.  
#pragma once  
#include <Arduino.h>  
  
// Исполнение одной расширенной команды (без префикса "AT").  
// Возвращает строку ответа для modem_answer().  
const char *modem_cmd_one(const char *cmd);
