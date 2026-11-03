// src/cat/cli.cpp  
// Порт check_serial_commands() из RP2040_beacon.ino.  
// Команды без аналога в модеме — заглушки "[CLI] '<cmd>' не поддерживается".  
#include "cli.h"  
#include "cat.h"  
#include "../trx/trx_control.h"  
#include "../trx/tx_job.h"  
#include "../vfo/vfo.h"  
#include "../vfo/quad_vfo/quad_vfo.h"  
#include "../rx/rx_dsp.h"  
#include "../rx/rx_usb_audio.h"  
#include "../filesys/file_manager.h"  
#include "../mem/mem_arena.h"  
#include <hardware/watchdog.h>  
#include <string.h>  
#include <strings.h>   // strcasecmp / strncasecmp  
#include <stdlib.h>  
  
static void cli_send(const char* s) {  
    if (!Serial.dtr()) return;  
    Serial.write((const uint8_t*)s, strlen(s));  
}  
  
static void cli_stub(const char* cmd) {  
    Serial.printf("[CLI] '%s' не поддерживается в модеме\r\n", cmd);  
}  
  
// --- rx <mod> <f> / rx off -------------------------------------------------  
static void cli_rx(char* args) {  
    if (!args) { cli_send("[CLI] rx <mod> <f_hz> | rx off\r\n"); return; }  
    while (*args == ' ') args++;                    // пропуск ведущих пробелов  
    if (!*args) { cli_send("[CLI] rx <mod> <f_hz> | rx off\r\n"); return; }  
  
    if (!strncmp(args, "off", 3) && (args[3] == 0 || args[3] == ' ')) {  
        rx_dsp_stop();  
        vfo_quad_stop();  
        cli_send("[RX] stopped\r\n");  
        return;  
    }  
    char* sp = strchr(args, ' ');  
    if (!sp) { cli_send("[CLI] rx <mod> <f_hz>\r\n"); return; }  
    *sp = 0;  
    uint32_t f = strtoul(sp + 1, NULL, 10);  
    if (f < 100000 || f > 30000000) { cli_send("[CLI] freq 100kHz..30MHz\r\n"); return; }  
  
    DemodMode d;  
    if      (!strcasecmp(args, "lsb")) d = DEMOD_LSB;  
    else if (!strcasecmp(args, "usb")) d = DEMOD_USB;  
    else if (!strcasecmp(args, "am"))  d = DEMOD_AM;  
    else if (!strcasecmp(args, "fm"))  d = DEMOD_NFM;  
    else if (!strcasecmp(args, "raw")) d = DEMOD_RAW;  
    else { cli_send("[CLI] mod: lsb|usb|am|fm|raw\r\n"); return; }  
  
    rx_dsp_set_demod(d);  
    rx_dsp_start(f);  
    Serial.printf("[RX] %s %lu Hz\r\n", args, (unsigned long)f);  
}  
  
// --- setparam M=V ------------------------------------------------------------  
static void cli_set_param(char* cmd) {  
    char* v = strchr(cmd, '=');  
    if (!v || v - cmd < 10) { cli_send("[CLI] setparam M=V\r\n"); return; }  
    *v = 0;  
    char m = cmd[9];   // "setparam X=..."  
    const char* val = v + 1;  
    switch (m) {  
        case 'C': my_call_variable = val; break;  
        case 'Q': my_qth_variable  = val; break;  
        case 'T': my_text_variable = val; break;  
        case 'F': my_freq_rx_var   = val; break;  
        case 'D': my_demod_var     = val; break;  
        default:  cli_send("[CLI] M: C|Q|T|F|D\r\n"); return;  
    }  
    pc_file_written = true;  
    Serial.printf("[CLI] %c=%s\r\n", m, val);  
}  
  
// --- tx <mode> <f> [text] / tx off / tx --------------------------------------  
static void cli_tx(char* a) {  
    while (*a == ' ') a++;  
  
    if (!*a) {   // "tx" — статус заявки  
        const TxJob* j = tx_job_info();  
        Serial.printf("[TX] state=%d mode=%d f=%lu err=%d\r\n",  
                      (int)j->state, (int)j->mode,  
                      (unsigned long)j->freq_hz, j->error);  
        return;  
    }  
    if (!strncmp(a, "off", 3) && (a[3] == 0 || a[3] == ' ')) {  
        tx_job_abort();  
        cli_send("[TX] abort requested\r\n");  
        return;  
    }  
  
    // tx <mode> <freq> [text]  
    char* sp = strchr(a, ' ');  
    if (!sp) { cli_send("[CLI] tx <mode> <f_hz> [text]\r\n"); return; }  
    *sp = 0;  
    char* fstr = sp + 1;  
    while (*fstr == ' ') fstr++;  
    char* sp2 = strchr(fstr, ' ');  
    const char* text = NULL;  
    if (sp2) { *sp2 = 0; text = sp2 + 1; while (*text == ' ') text++; }  
    uint32_t f = strtoul(fstr, NULL, 10);  
    if (f < 100000 || f > 30000000) { cli_send("[CLI] freq 100kHz..30MHz\r\n"); return; }  
  
    // имя моды -> ModeId через реестр  
    ModeId mid = MODE_COUNT;  
    for (int i = 0; i < (int)MODE_COUNT; i++) {  
        const ModeIface* mi = modes_get((ModeId)i);  
        if (mi && mi->name && !strcasecmp(mi->name, a)) { mid = (ModeId)i; break; }  
    }  
    if (mid == MODE_COUNT) { cli_send("[CLI] неизвестная мода\r\n"); return; }  
  
    if (tx_job_submit(mid, f, text))  
        Serial.printf("[TX] queued %s %lu\r\n", a, (unsigned long)f);  
    else  
        cli_send("[TX] занято или мода без tx_frame\r\n");  
}  
  
// --- диспетчер ---------------------------------------------------------------  
void cli_execute(char* cmd) {  
    while (*cmd == ' ') cmd++;  
    size_t l = strlen(cmd);  
    while (l && (cmd[l-1] == ' ' || cmd[l-1] == '\t')) cmd[--l] = 0;  
    if (!l) return;  
  
    if (!strcmp(cmd, "?")) { print_current_settings(); return; }  
    if (!strcmp(cmd, "help")) { cli_print_help(); return; }  
  
    if (!strncmp(cmd, "text", 4)) {  
        char* t = cmd + 4;  
        while (*t == ' ') t++;  
        if (*t) { my_text_variable = t; pc_file_written = true; }  
        Serial.printf("[CLI] text='%s'\r\n", my_text_variable.c_str());  
        return;  
    }  
    if (!strncmp(cmd, "setparam ", 9)) { cli_set_param(cmd); return; }  
  
    if (!strncmp(cmd, "rx", 2) && (cmd[2] == ' ' || cmd[2] == 0)) {  
        cli_rx(cmd + 2);  
        return;  
    }  
    if (!strcmp(cmd, "q")) { cli_rx((char*)"off"); return; }  
  
    if (!strncmp(cmd, "tx", 2) && (cmd[2] == ' ' || cmd[2] == 0)) {  
        cli_tx(cmd + 2);  
        return;  
    }  
  
    if (!strncmp(cmd, "quad ", 5)) {  
        uint32_t f = strtoul(cmd + 5, NULL, 10);  
        if (f) { vfo_quad_start(f); Serial.printf("[QUAD] LO=%lu\r\n", (unsigned long)f); }  
        else   { vfo_quad_stop(); cli_send("[QUAD] off\r\n"); }  
        return;  
    }  
    if (!strcmp(cmd, "vfo"))    { vfo_debug_dump(); return; }  
    if (!strcmp(cmd, "limits")) {  
        Serial.printf("[VFO] clk_sys_max=%lu pll_vco_max=%lu xosc_corr=%ld ppb\r\n",  
                      (unsigned long)vfo_max_clk_sys_hz,  
                      (unsigned long)vfo_max_pll_vco_hz,  
                      (long)vfo_xosc_corr_ppb);  
        return;  
    }  
    if (!strcmp(cmd, "mem")) {  
        Serial.printf("[MEM] trx_wm=%u/%u heap=%u\r\n",  
                      (unsigned)mem_arena_watermark(),  
                      (unsigned)mem_arena_capacity(MEM_PHASE_RX),  
                      (unsigned)rp2040.getFreeHeap());  
        return;  
    }  
    if (!strcmp(cmd, "clear log"))   { log_file_clear(); cli_send("[FM] log cleared\r\n"); return; }  
    if (!strcmp(cmd, "format disk")) { force_reset_to_default_disk(); cli_send("[FM] disk reset\r\n"); return; }  
    if (!strcmp(cmd, "remount"))     { remount_usb_disk(); cli_send("[FM] remounted\r\n"); return; }  
    if (!strcmp(cmd, "reset"))       { cli_send("[CLI] reboot\r\n"); delay(50); watchdog_reboot(0,0,0); return; }  
  
    if (!strcmp(cmd, "status")) {  
        const char* st = (trx_state() == TRX_RX) ? "RX"  
                       : (trx_state() == TRX_TX) ? "TX" : "SWITCHING";  
        Serial.printf("[ST] %s f=%lu demod=%s call=%s\r\n",  
                      st, (unsigned long)atol(my_freq_rx_var.c_str()),  
                      my_demod_var.c_str(), my_call_variable.c_str());  
        return;  
    }  
  
    // ---- заглушки: нет аналога в модеме ------------------------------------  
    static const char* stubs[] = {  
        "time","date","tele","clim","clim_tx","sched","schedule",  
        "restart","stop","iq","OCTEST","PLLTEST",  
        "adc","adcr","adcq","bpf","modem","jtty","START","start"  
    };  
    for (size_t i = 0; i < sizeof(stubs)/sizeof(stubs[0]); i++) {  
        size_t n = strlen(stubs[i]);  
        if (!strncmp(cmd, stubs[i], n) && (cmd[n] == 0 || cmd[n] == ' ')) {  
            cli_stub(cmd);  
            return;  
        }  
    }  
  
    Serial.printf("[CLI] неизвестная команда '%s' — help для справки\r\n", cmd);  
}  
  
void cli_print_help(void) {  
    cli_send(  
        "Команды (терминатор NL):\r\n"  
        "  ?              - текущие настройки\r\n"  
        "  help           - эта справка\r\n"  
        "  text [txt]     - текст INFO.TXT\r\n"  
        "  setparam M=V   - параметр (C=call Q=qth T=text F=freq D=demod)\r\n"  
        "  rx <mod> <f>   - приём: lsb|usb|am|fm|raw, частота в Гц\r\n"  
        "  rx off | q     - стоп приём\r\n"  
        "  tx <mod> <f> [txt] - передать в моде (имя из реестра)\r\n"  
        "  tx off         - прервать передачу\r\n"  
        "  tx             - состояние TX-заявки\r\n"  
        "  quad <f>|off   - квадратурный LO\r\n"  
        "  vfo            - дамп VFO/PLL\r\n"  
        "  mem            - watermark арены + heap\r\n"  
        "  limits         - пределы разгона\r\n"  
        "  status         - состояние TRX\r\n"  
        "  clear log      - очистить LOG.TXT\r\n"  
        "  format disk    - сброс диска к дефолту\r\n"  
        "  remount        - перемонтировать USB-диск\r\n"  
        "  reset          - перезагрузка\r\n"  
        "Заглушки (нет аналога): time date tele clim sched restart stop\r\n"  
        "  iq OCTEST PLLTEST adc adcr adcq bpf modem jtty START ...\r\n"  
    );  
}