// src/modes/ax25/ax25_hdlc.cpp — общий слой: NRZI, bit-stuff/destuff,  
// CRC-16/X.25 (poly 0x8408, init 0xFFFF, residue 0xF0B8). КАРКАС.  
#include "ax25.h"  
  
// CRC-16/X.25: отражённый полином 0x8408, старт 0xFFFF,  
// хороший остаток с учётом FCS = 0xF0B8.  
uint16_t ax25_fcs(const uint8_t *data, size_t len) {  
  uint16_t crc = 0xFFFF;  
  for (size_t i = 0; i < len; i++) {  
    crc ^= data[i];  
    for (int b = 0; b < 8; b++)  
      crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : (crc >> 1);  
  }  
  return crc;  
}
