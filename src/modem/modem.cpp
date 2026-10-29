// src/modem/modem.cpp — ядро Hayes-модема: FSM, разбор AT-строки.  
// КАРКАС: команды-заглушки отвечают OK/ERROR.  
#include "modem.h"  
#include "../trx/trx_control.h"  
#include "../vfo/vfo.h"  
  
static uint8_t  s_sreg[MODEM_SREG_COUNT];  
static char     s_line[MODEM_LINE_MAX];  
static size_t   s_line_len = 0;  
  
void modem_init(void) {  
  memset(s_sreg, 0, sizeof(s_sreg));  
  s_sreg[0]  = MODEM_S0_DEFAULT;  
  s_sreg[7]  = MODEM_S7_DEFAULT;  
  s_sreg[12] = MODEM_S12_DEFAULT;  
  Serial.println("[STUB] modem_init: Hayes core skeleton");  
}  
  
void modem_rx_byte(uint8_t b) {  
  if (b == '\r' || b == '\n') {  
    if (s_line_len) { s_line[s_line_len] = 0; modem_cmd_exec(s_line); s_line_len = 0; }  
    return;  
  }  
  if (s_line_len < MODEM_LINE_MAX - 1) s_line[s_line_len++] = (char)b;  
}  
  
void modem_cmd_exec(const char *line) {  
  if (strncmp(line, "AT", 2) != 0) { modem_answer("ERROR"); return; }  
  if (line[2] == 0)               { modem_answer("OK");    return; }  
  // TODO: парсер AT+..., ATD, ATO, ATH, ATSn?  
  Serial.printf("[STUB] modem_cmd_exec: '%s' not parsed\n", line);  
  modem_answer("OK");  
}  
  
void modem_answer(const char *s) { Serial.println(s); }  
void modem_poll(void) {}
