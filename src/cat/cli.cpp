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
    if (!args || !*args) { cli_send("[CLI] rx <mod> <f_hz> | rx off\r\n"); return; }  
    while (*args == ' ') args++;                       // пропуск ведущих пробелов  
    if (!strncmp(args, "off", 3) && (args[3] == 0 || args[3] == ' ')) {  
        rx_dsp_stop();  
        vfo_quad_stop();  
        cli_send("[RX] stopped\r\n");  
        return;  
    }  
    char* sp = strchr(args, ' ');  
    if (!sp) { cli_send("[CLI] rx <mod> <f_hz>\r\n"); return; }  
    *sp = 0;  
    uint32_t f = strtoul(sp + 1, nullptr, 10);  
    if (!f) { cli_send("[CLI] rx: частота?\r\n"); return; }  
  
    uint8_t d;  
    if      (!strcasecmp(args, "lsb")) d = RXD_DEMOD_LSB;  
    else if (!strcasecmp(args, "usb")) d = RXD_DEMOD_USB;  
    else if (!strcasecmp(args, "am"))  d = RXD_DEMOD_AM;  
    else if (!strcasecmp(args, "fm"))  d = RXD_DEMOD_FM;  
    else if (!strcasecmp(args, "raw")) d = RXD_DEMOD_RAW;  
    else { cli_send("[CLI] rx: mod = lsb|usb|am|fm|raw\r\n"); return; }  
  
    rx_dsp_set_demod(d);  
    rx_dsp_start(f);  
    cat_set_freq_hz(f);                 // синхронизация CAT-частоты  
    Serial.printf("[RX] %s %lu Hz\r\n", args, (unsigned long)f);  
}  
  
// --- tx <mode> <f> [text] / tx off / tx ------------------------------------  
static void cli_tx(char* args) {  
    while (*args == ' ') args++;  
    if (!*args) {   // "tx" — статус заявки  
        const TxJob* j = tx_job_info();  
        Serial.printf("[TX] state=%d mode=%d f=%lu err=%d\r\n",  
                      (int)j->state, (int)j->mode,  
                      (unsigned long)j->freq_hz, j->error);  
        return;  
    }  
    if (!strncmp(args, "off", 3) && (args[3] == 0 || args[3] == ' ')) {  
        tx_job_abort();  
        cli_send("[TX] abort requested\r\n");  
        return;  
    }  
    char* sp = strchr(args, ' ');  
    if (!sp) { cli_send("[CLI] tx <mode> <f_hz> [text]\r\n"); return; }  
    *sp = 0;  
    uint32_t f = strtoul(sp + 1, nullptr, 10);  
    if (!f) { cli_send("[CLI] tx: частота?\r\n"); return; }  
    char* text = sp + 1;  
    while (*text == ' ') text++;  
  
    // поиск моды по имени в реестре  
    const ModeIface* m = nullptr;  
    for (uint8_t i = 0; i < MODE_COUNT; i++) {  
        const ModeIface* cand = modes_get((ModeId)i);  
        if (cand && cand->name && !strcasecmp(cand->name, args)) { m = cand; break; }  
    }  
    if (!m) { cli_send("[CLI] tx: неизвестная мода\r\n"); return; }  
    if (!m->tx_frame) { cli_send("[CLI] tx: мода без передатчика\r\n"); return; }  
  
    if (tx_job_submit(m->id, f, text) == 0)  
        Serial.printf("[TX] queued %s %lu Hz\r\n", m->name, (unsigned long)f);  
    else  
        cli_send("[TX] busy / rejected\r\n");  
}  
  
// --- setparam M=V ------------------------------------------------------------  
static void cli_set_param(char* args) {  
    // args указывает на "M=V" (пробелы уже пропущены вызывающим)  
    char* v = strchr(args, '=');  
    if (!v || v - args != 1) { cli_send("[CLI] setparam M=V\r\n"); return; }  
    *v = 0;  
    char m = args[0];  
    const char* val = v + 1;  
    if      (m == 'C' || m == 'c') my_call_variable = val;  
    else if (m == 'Q' || m == 'q') my_qth_variable  = val;  
    else if (m == 'T' || m == 't') my_text_variable = val;  
    else if (m == 'F' || m == 'f') my_freq_rx_var   = val;  
    else if (m == 'D' || m == 'd') my_demod_var     = val;  
    else { cli_send("[CLI] setparam: M = C|Q|T|F|D\r\n"); return; }  
    pc_file_written = true;  
    Serial.printf("[CLI] %c=%s\r\n", m, val);  
}  

static void cli_help(void);

// --- диспетчер ---------------------------------------------------------------  
void cli_execute(char* cmd) {  
    while (*cmd == ' ') cmd++;  
    if (!*cmd) return;  
  
    if (!strcmp(cmd, "?") || !strcasecmp(cmd, "status")) {  
        print_current_settings();  
        Serial.printf("[CLI] trx=%s lo=%lu\r\n",  
                      trx_state() == TRX_RX ? "RX" :  
                      (trx_state() == TRX_TX ? "TX" : "SWITCHING"),  
                      (unsigned long)vfo_quad_lo_hz());  
        return;  
    }  
    if (!strcasecmp(cmd, "help")) { cli_help(); return; }  
  
    if (!strncmp(cmd, "text", 4) && (cmd[4] == 0 || cmd[4] == ' ')) {  
        char* t = cmd + 4;  
        while (*t == ' ') t++;  
        if (*t) { my_text_variable = t; pc_file_written = true; }  
        Serial.printf("[CLI] text='%s'\r\n", my_text_variable.c_str());  
        return;  
    }  
    if (!strncmp(cmd, "setparam", 8) && (cmd[8] == ' ')) {  
        char* a = cmd + 8;  
        while (*a == ' ') a++;  
        cli_set_param(a);  
        return;  
    }  
    if (!strncmp(cmd, "rx", 2) && (cmd[2] == ' ' || cmd[2] == 0)) {  
        cli_rx(cmd + 2);  
        return;  
    }  
    if (!strcasecmp(cmd, "q")) { cli_rx((char*)"off"); return; }  
  
    if (!strncmp(cmd, "tx", 2) && (cmd[2] == ' ' || cmd[2] == 0)) {  
        cli_tx(cmd + 2);  
        return;  
    }  
  
    if (!strncmp(cmd, "quad", 4) && (cmd[4] == 0 || cmd[4] == ' ')) {  
        char* a = cmd + 4;  
        while (*a == ' ') a++;  
        if (!*a || !strcasecmp(a, "off")) {  
            vfo_quad_stop();  
            cli_send("[QUAD] off\r\n");  
        } else {  
            uint32_t f = strtoul(a, nullptr, 10);  
            quad_vfo_set_quiet(true);  
            bool ok = quad_vfo_start(f);  
            quad_vfo_set_quiet(false);  
            Serial.printf("[QUAD] %s lo=%lu err=%ld\r\n",  
                          ok ? "on" : "FAIL",  
                          (unsigned long)quad_vfo_actual_hz(),  
                          (long)quad_vfo_error_hz());  
        }  
        return;  
    }  
    if (!strcasecmp(cmd, "vfo"))    { vfo_debug_dump(); return; }  
    if (!strcasecmp(cmd, "mem")) {  
        Serial.printf("[MEM] trx_wm=%lu/%lu heap=%lu\r\n",  
                      (unsigned long)mem_arena_watermark(),  
                      (unsigned long)mem_arena_capacity(MEM_PHASE_RX),  
                      (unsigned long)rp2040.getFreeHeap());  
        return;  
    }  
    if (!strcasecmp(cmd, "limits")) {  
        Serial.printf("[VFO] clk_sys_max=%lu pll_vco_max=%lu\r\n",  
                      (unsigned long)vfo_max_clk_sys_hz,  
                      (unsigned long)vfo_max_pll_vco_hz);  
        return;  
    }  
    if (!strcasecmp(cmd, "clear log"))   { log_file_clear(); cli_send("[FM] log cleared\r\n"); return; }  
    if (!strcasecmp(cmd, "format disk")) { force_reset_to_default_disk(); cli_send("[FM] disk reset\r\n"); return; }  
    if (!strcasecmp(cmd, "remount"))     { remount_usb_disk(); cli_send("[FM] remounted\r\n"); return; }  
    if (!strcasecmp(cmd, "reset")) {  
        cli_send("[CLI] reboot\r\n");  
        Serial.flush();  
        delay(50);  
        watchdog_reboot(0, 0, 0);  
        return;  
    }  
  
    // ---- заглушки: команд маяка без аналога в модеме ----  
    static const char* const stubs[] = {  
        "time", "date", "tele", "clim", "clim_tx", "sched", "schedule",  
        "restart", "stop", "iq", "OCTEST", "PLLTEST",  
        "adc", "adcr", "adcq", "bpf", "modem", "jtty",  
        "START", "start"  
    };  
    for (size_t i = 0; i < sizeof(stubs)/sizeof(stubs[0]); i++) {  
        size_t l = strlen(stubs[i]);  
        if (!strncmp(cmd, stubs[i], l) && (cmd[l] == 0 || cmd[l] == ' ')) {  
            cli_stub(cmd);  
            return;  
        }  
    }  
  
    Serial.printf("[CLI] неизвестная команда '%s' (help)\r\n", cmd);  
}  
  
// --- справка -----------------------------------------------------------------  
void cli_help(void) {  
    cli_send(  
        "Команды (терминатор NL):\r\n"  
        "  ?              - текущие настройки\r\n"  
        "  help           - эта справка\r\n"  
        "  text [txt]     - текст INFO.TXT\r\n"  
        "  setparam M=V   - параметр (C=call Q=qth T=text F=freq D=demod)\r\n"  
        "  rx <mod> <f>   - приём: lsb|usb|am|fm|raw, частота в Гц\r\n"  
        "  rx off | q     - стоп приём\r\n"  
        "  tx <mod> <f> [txt] - передать в моде (по реестру modes)\r\n"  
        "  tx off         - прервать передачу\r\n"  
        "  tx             - состояние TX-заявки\r\n"  
        "  quad <f>|off   - квадратурный LO\r\n"  
        "  vfo            - дамп VFO/PLL\r\n"  
        "  mem            - watermark арены + heap\r\n"  
        "  limits         - пределы разгона\r\n"  
        "  clear log      - очистить LOG.TXT\r\n"  
        "  format disk    - сброс диска к дефолту\r\n"  
        "  remount        - перемонтировать USB-диск\r\n"  
        "  reset          - перезагрузка\r\n"  
        "Заглушки (нет аналога): time date tele clim sched restart stop\r\n"  
        "  iq OCTEST PLLTEST adc adcr adcq bpf modem jtty START ...\r\n"  
    );  
}