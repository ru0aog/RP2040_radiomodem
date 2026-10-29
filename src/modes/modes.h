// src/modes/modes.h — контракт одной моды для rx_decoders и modem  
struct ModeRx {  
  void (*init)(uint32_t center_hz);   // подстройка под центр ПЧ  
  void (*push)(int16_t mono);         // 15625 sps от rx_dsp  
  void (*stop)(void);  
};  
struct ModeTx {  
  bool (*send)(const uint8_t* data, uint16_t len); // кадр → TRX_TX  
  bool (*busy)(void);  
};  
struct ModeIface {  
  const char* name;        // "ax25"  
  uint8_t     rsid_code;   // код RSID для автодетекта (0 = нет)  
  ModeRx      rx;  
  ModeTx      tx;  
};  
const ModeIface* mode_find(const char* name);  
const ModeIface* mode_find_rsid(uint8_t code);
