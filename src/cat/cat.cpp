// CAT-интерфейс модема (Kenwood TS-2000 subset), второй USB-CDC порт.  
// Порт из RP2040_receiver/src/cat/fldigi_cat.cpp; retune через vfo_on_trx_switch.  
#include "cat.h"  
#include "../vfo/vfo.h"  
#include "../trx/trx_control.h"  
#include "../rx/rx_dsp.h"  
#include "../rx/rx_usb_audio.h"  
#include <Adafruit_TinyUSB.h>  
#include <stdio.h>  
#include <string.h>  
#include <stdlib.h>  
  
// CAT на основном USB-CDC (Serial) — тот же порт, что и логи.  
// Ответы CAT перемешиваются с логами; для WSJT-X это допустимо  
// (Hamlib фильтрует по ожидаемым ответам), но монитор порта будет  
// показывать и то, и другое.
  
static bool     cat_enabled = false;  
static uint32_t s_rx_hz     = 14074000;  // зеркало частоты приёма  
  
#define CAT_BUF_MAX 64  
static char    cat_buf[CAT_BUF_MAX];  
static uint8_t cat_len = 0;  
static bool    cat_prev_dtr = false;  
  
static char cat_mode = '2';              // 1=LSB 2=USB 3=CW 4=FM 5=AM  
  
static void cat_send(const char* s) {  
    Serial.write((const uint8_t*)s, strlen(s));  
    Serial.flush();  
}  
  
static void cat_reply_fa(void) {  
    char out[16];  
    snprintf(out, sizeof(out), "FA%011lu;", (unsigned long)s_rx_hz);  
    cat_send(out);  
}  
  
static void cat_reply_if(void) {  
    char out_buf[48];  
    snprintf(out_buf, sizeof(out_buf), "IF00000000000       -00300   20000000;");  
    uint32_t f = s_rx_hz;  
    for (int i = 12; i >= 2; i--) { out_buf[i] = '0' + (f % 10); f /= 10; }  
    cat_send(out_buf);  
}  
  
static void cat_reply_md(void) {  
    char out[8]; snprintf(out, sizeof(out), "MD%c;", cat_mode); cat_send(out);  
}  
static void cat_reply_err(void) { cat_send("?;"); }  
  
// Таблицы срезов TS-2000 (как в приёмнике)  
static const uint16_t SL_TABLE[12] = {0,50,100,200,300,400,500,600,700,800,900,1000};  
static const uint16_t SH_TABLE[12] = {1400,1600,1800,2000,2200,2400,2600,2800,3000,3400,4000,5000};  
  
static void cat_retune(uint32_t hz) {  
    s_rx_hz = hz;  
    vfo_on_trx_switch(TRX_RX, hz);   // новый quad_vfo_start + захват PLL  
    rx_dsp_start(hz);                // пересчёт NCO от vfo_quad_lo_hz()  
}  
  
static void cat_execute(const char* cmd) {  
    size_t n = strlen(cmd);  
    if (n < 2) { cat_reply_err(); return; }  
  
    if (cmd[0]=='F' && cmd[1]=='A') {  
        if (n == 2) { cat_reply_fa(); }  
        else if (n == 13) { cat_retune((uint32_t)strtoul(cmd+2, nullptr, 10)); }  
        else cat_reply_err();  
        return;  
    }  
    if (cmd[0]=='I' && cmd[1]=='F' && n==2) { cat_reply_if(); return; }  
    if (cmd[0]=='I' && cmd[1]=='D' && n==2) { cat_send("ID019;"); return; }  
  
    if (cmd[0]=='M' && cmd[1]=='D') {  
        if (n == 2) { cat_reply_md(); return; }  
        if (n == 3) {  
            switch (cmd[2]) {  
                case '1': rx_usb_audio_set_mode(UAC_MODE_SSB); rx_usb_audio_set_demod(DEMOD_LSB); cat_mode='1'; return;  
                case '2': rx_usb_audio_set_mode(UAC_MODE_SSB); rx_usb_audio_set_demod(DEMOD_USB); cat_mode='2'; return;  
                case '3': rx_usb_audio_set_mode(UAC_MODE_SSB); rx_usb_audio_set_demod(DEMOD_USB); cat_mode='3'; return;  
                case '4': rx_usb_audio_set_mode(UAC_MODE_SSB); rx_usb_audio_set_demod(DEMOD_FM);  cat_mode='4'; return;  
                case '5': rx_usb_audio_set_mode(UAC_MODE_SSB); rx_usb_audio_set_demod(DEMOD_AM);  cat_mode='5'; return;  
                default: cat_reply_err(); return;  
            }  
        }  
        cat_reply_err(); return;  
    }  
  
    if (cmd[0]=='T' && cmd[1]=='X' && cmd[2]=='1' && n==3) { return; }  // заглушка PTT  
    if (cmd[0]=='R' && cmd[1]=='X' && n==2) { return; }  
  
    if (cmd[0]=='N' && cmd[1]=='T') {  
        if (n == 3) { rx_dsp_set_notch(cmd[2] != '0'); return; }  
        cat_reply_err(); return;  
    }  
  
    if (cmd[0]=='S' && cmd[1]=='H') {  
        if (n == 2) { char o[12]; snprintf(o,sizeof(o),"SH%04lu;",(unsigned long)rx_dsp_af_hi_hz()); cat_send(o); return; }  
        uint32_t f_hi;  
        if (n == 4)      { int i=(int)strtoul(cmd+2,nullptr,10); if(i<0||i>11){cat_reply_err();return;} f_hi=SH_TABLE[i]; }  
        else if (n == 6) { f_hi=(uint32_t)strtoul(cmd+2,nullptr,10); }  
        else { cat_reply_err(); return; }  
        rx_dsp_set_af_band(rx_dsp_af_lo_hz(), f_hi);  
        return;  
    }  
  
    if (cmd[0]=='S' && cmd[1]=='L') {  
        if (n == 2) { char o[12]; snprintf(o,sizeof(o),"SL%04lu;",(unsigned long)rx_dsp_af_lo_hz()); cat_send(o); return; }  
        uint32_t f_lo;  
        if (n == 4)      { int i=(int)strtoul(cmd+2,nullptr,10); if(i<0||i>11){cat_reply_err();return;} f_lo=SL_TABLE[i]; }  
        else if (n == 6) { f_lo=(uint32_t)strtoul(cmd+2,nullptr,10); }  
        else { cat_reply_err(); return; }  
        rx_dsp_set_af_band(f_lo, rx_dsp_af_hi_hz());  
        return;  
    }  
  
    if (cmd[0]=='N' && cmd[1]=='R') {  
        if (n == 2) { char b[8]; snprintf(b,sizeof(b),"NR%d;", rx_dsp_get_nr()?1:0); cat_send(b); return; }  
        if (n == 3) {  
            if (cmd[2]=='1') { rx_dsp_set_nr(true);  return; }  
            if (cmd[2]=='0') { rx_dsp_set_nr(false); return; }  
        }  
        cat_reply_err(); return;  
    }  
  
    cat_reply_err();  
}  
  
void cat_init(void) {  
    // Транспорт — основной Serial (уже открыт в setup()).  
    s_rx_hz = 0;  
    s_line[0] = 0;  
    s_len = 0;  
    Serial.println("[CAT] init: shared CDC port");  
}
  
void cat_poll(void) {  
    if (!cat_enabled) return;  
    bool dtr = Serial.dtr();  
    if (!cat_prev_dtr && dtr) { while (Serial.available() > 0) Serial.read(); cat_len = 0; }  
    cat_prev_dtr = dtr;  
    while (Serial.available() > 0) {  
        char c = (char)Serial.read();  
        if (c == ';') { cat_buf[cat_len] = '\0'; cat_execute(cat_buf); cat_len = 0; }  
        else if (c != '\r' && c != '\n') {  
            if (cat_len < CAT_BUF_MAX-1) cat_buf[cat_len++] = c; else cat_len = 0;  
        }  
    }  
}  
  
void cat_set_freq_hz(uint32_t hz) { s_rx_hz = hz; }
