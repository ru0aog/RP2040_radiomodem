// БЫЛО: .rsid_code = 0, .rx_init = nullptr, .rx_push = nullptr  
// СТАЛО:  
static const ModeIface s_mode = {  
  .id          = MODE_RTTY,  
  .name        = "rtty",  
  .rsid_code   = 37,            // RSID-код RTTY45 из RSID_LIST — сверить с таблицей  
  .rx_init     = rtty_rx_init,  
  .rx_push     = rtty_rx_push,  
  .tx_start    = nullptr,  
  .tx_poll     = nullptr,  
  .tx_stop     = nullptr,  
};
