// src/modem/modem_cfg.h  
// ============================================================================  
// Профиль Hayes-модема: тайминги ESC-последовательности, дефолты  
// S-регистров, параметры ответов.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
  
#define MODEM_SREG_COUNT     32        // S0..S31  
#define MODEM_LINE_MAX      128        // макс. длина AT-строки  
  
// Тайминги "+++" (мс): пауза до, пауза после.  
#define MODEM_ESC_GUARD_MS   1000  
#define MODEM_ESC_CHAR       '+'  
  
// Дефолты S-регистров (индекс = номер регистра).  
#define MODEM_S0_DEFAULT     0         // автоответ: выкл  
#define MODEM_S7_DEFAULT    50         // таймаут CONNECT, сек  
#define MODEM_S12_DEFAULT   50         // guard time "+++", ×20 мс
