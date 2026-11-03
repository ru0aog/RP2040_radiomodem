// src/cat/cli.cpp  
// Порт check_serial_commands() из RP2040_beacon.ino.  
// Команды без аналога в модеме — заглушки "[CLI] '<cmd>' не поддерживается".  
#include "cli.h"  
#include "cat.h"  
#include "../trx/trx_control.h"  
#include "../vfo/vfo.h"  
#include "../vfo/quad_vfo/quad_vfo.h"  
#include "../rx/rx_dsp.h"  
#include "../rx/rx_usb_audio.h"  
#include "../filesys/file_manager.h"  
#include "../mem/mem_arena.h"  
#include <hardware/watchdog.h>  
#include <string.h>  
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
    if (strncmp(args, "off", 3) == 0) {  
        rx_dsp_stop();  
        vfo_quad_stop();  
        cli_send("[CLI] RX остановлен\r\n");  
        return;  
    }  
    char* sp = strchr(args, ' ');  
    if (!sp) { cli_send("[CLI] rx <mod> <f_hz>\r\n"); return; }  
    *sp = 0;  
    uint32_t f = (uint32_t)strtoul(sp + 1, nullptr, 10);  
    if (f < 100000 || f > 30000000) { cli_send("[CLI] частота вне диапазона\r\n"); return; }  
  
    uint8_t demod = RXD_DEMOD_USB;  
    if      (!strcasecmp(args, "lsb")) demod = RXD_DEMOD_LSB;  
    else if (!strcasecmp(args, "usb")) demod = RXD_DEMOD_USB;  
    else if (!strcasecmp(args, "am"))  demod = RXD_DEMOD_AM;  
    else if (!strcasecmp(args, "fm"))  demod = RXD_DEMOD_FM;  
    else if (!strcasecmp(args, "raw")) demod = RXD_DEMOD_RAW;  
    else { cli_send("[CLI] мода: lsb|usb|am|fm|raw\r\n"); return; }  
  
    quad_vfo_set_quiet(true);  
    vfo_on_trx_switch(TRX_RX, f);  
    quad_vfo_set_quiet(false);  
    rx_dsp_start(f);  
    rx_dsp_set_demod(demod);  
    cat_set_freq_hz(f);                    // синхронизировать зеркало CAT  
    Serial.printf("[CLI] RX %s %lu Hz\r\n", args, (unsigned long)f);  
}  
  
// --- setparam M=V / text ----------------------------------------------------  
// Обновляет переменные в RAM. Персистентность во flash зависит от  
// file_manager: pc_file_written подхватывается check_and_handle_pc_changes().  
static bool cli_set_param(char m, const char* v) {  
    switch (m) {  
        case 'C': my_call_variable = v; break;  
        case 'Q': my_qth_variable  = v; break;  
        case 'T': my_text_variable = v; break;  
        case 'F': my_freq_rx_var   = v; break;  
        case 'D': my_demod_var     = v; break;  
        default: return false;  
    }  
    pc_file_written = true;  
    return true;  
}  
  
void cli_execute(char* cmd) {  
    // trim  
    while (*cmd == ' ') cmd++;  
    size_t n = strlen(cmd);  
    while (n && (cmd[n-1] == ' ')) cmd[--n] = 0;  
    if (!n) return;  
  
    if (!strcmp(cmd, "?")) { print_current_settings(); return; }  
    if (!strcmp(cmd, "help")) { cli_print_help(); return; }  
  
    if (!strncmp(cmd, "text", 4)) {  
        const char* v = cmd + 4; while (*v == ' ') v++;  
        if (*v) { my_text_variable = v; pc_file_written = true; }  
        Serial.printf("[CLI] TEXT='%s'\r\n", my_text_variable.c_str());  
        return;  
    }  
  
    if (!strncmp(cmd, "setparam", 8)) {  
        char* v = strchr(cmd, '=');  
        if (!v || v - cmd < 10) { cli_send("[CLI] setparam M=V (M: C,Q,T,F,D)\r\n"); return; }  
        *v = 0;  
        char m = cmd[9];  
        if (cli_set_param(m, v + 1)) Serial.printf("[CLI] %c='%s'\r\n", m, v + 1);  
        else cli_send("[CLI] неизвестный параметр (C,Q,T,F,D)\r\n");  
        return;  
    }  
  
    if (!strcmp(cmd, "clear log"))    { log_file_clear(); cli_send("[CLI] log cleared\r\n"); return; }  
    if (!strcmp(cmd, "format disk"))  { force_reset_to_default_disk(); cli_send("[CLI] disk formatted\r\n"); return; }  
    if (!strcmp(cmd, "remount"))      { remount_usb_disk(); cli_send("[CLI] remounted\r\n"); return; }  
  
    if (!strcmp(cmd, "reset")) {  
        log_file_write_line("[CLI] reset");  
        Serial.flush();  
        watchdog_reboot(0, 0, 0);  
        return;  
    }  
  
    if (!strcmp(cmd, "limits")) {  
        Serial.printf("[CLI] clk_sys_max=%lu Hz  pll_vco_max=%lu Hz  xosc_corr=%ld ppb\r\n",  
                      (unsigned long)vfo_max_clk_sys_hz,  
                      (unsigned long)vfo_max_pll_vco_hz,  
                      (long)vfo_xosc_corr_ppb);  
        return;  
    }  
  
    if (!strcmp(cmd, "mem")) {  
        Serial.printf("[CLI] trx_wm=%lu/%lu B  heap=%lu B  state=%s\r\n",  
                      (unsigned long)mem_arena_watermark(),  
                      (unsigned long)mem_arena_capacity(MEM_PHASE_RX),  
                      (unsigned long)rp2040.getFreeHeap(),  
                      trx_state() == TRX_RX ? "RX" : "TX");  
        return;  
    }  
  
    if (!strncmp(cmd, "rx", 2) && (cmd[2] == ' ' || cmd[2] == 0)) { cli_rx(cmd + 2); return; }  
    if (!strcmp(cmd, "q")) { cli_rx((char*)"off"); return; }  
  
    if (!strncmp(cmd, "quad", 4)) {  
        char* a = cmd + 4; while (*a == ' ') a++;  
        if (!strncmp(a, "off", 3)) { vfo_quad_stop(); cli_send("[CLI] quad off\r\n"); return; }  
        uint32_t f = (uint32_t)strtoul(a, nullptr, 10);  
        if (f < 100000 || f > 30000000) { cli_send("[CLI] quad <f_hz>|off\r\n"); return; }  
        vfo_quad_start(f);  
        Serial.printf("[CLI] quad LO=%lu Hz\r\n", (unsigned long)vfo_quad_lo_hz());  
        return;  
    }  
  
    if (!strcmp(cmd, "vfo")) { vfo_debug_dump(); return; }  
  
    // ---- Заглушки: аналогов в модеме нет ----------------------------------  
    static const char* stubs[] = {  
        "time", "date", "tele", "clim", "clim_tx", "sched", "schedule",  
        "restart", "stop", "iq", "OCTEST", "PLLTEST", "adc", "adcr", "adcq",  
        "bpf", "modem", "test jtty", "test jtty pack", "jtty",  
        "START", "start"  
    };  
    for (size_t i = 0; i < sizeof(stubs)/sizeof(stubs[0]); i++) {  
        size_t l = strlen(stubs[i]);  
        if (!strncmp(cmd, stubs[i], l) && (cmd[l] == 0 || cmd[l] == ' ')) {  
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