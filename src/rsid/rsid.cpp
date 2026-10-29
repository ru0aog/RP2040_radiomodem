// src/rsid/rsid.cpp — RSID-детектор. КАРКАС.  
#include "rsid.h"  
#include "../rx/rx_decoders.h"  
  
void rsid_init(void)              { Serial.println("[STUB] rsid_init"); }  
void rsid_rx_push(int16_t s)      { (void)s; }  
void rsid_poll(void)              {}  
// при детекте: rx_decoders_on_rsid(code);
