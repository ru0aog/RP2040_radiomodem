// src/cat/cli.h  
// Человеческий CLI на общем CDC-порту (порт системы команд RP2040_beacon).  
// Встраивается в cat_poll(): строка до '\n' -> cli_execute(), до ';' -> CAT.  
#pragma once  
#include <Arduino.h>  
  
void cli_execute(char* cmd);   // разбор одной строки (вызывается из cat.cpp)  
void cli_print_help(void);