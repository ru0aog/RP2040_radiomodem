/**    
 * ============================================================================    
 *  file_manager.cpp — USB-MSC (FAT12, 20 КБ) + Wear Leveling во Flash    
 *  Порт RP2040_beacon v2.19 (RU0AOG): структура модуля сохранена,    
 *  изменена только геометрия диска (128 КБ → 20 КБ) и добавлены    
 *  радиомодемовские правки безопасности flash-write.    
 *    
 *  Разметка FAT12 (40 секторов × 512 Б):    
 *    сектор 0      — boot + BPB (48 записей каталога)    
 *    сектор 1      — FAT12    
 *    сектора 2..4  — корневой каталог    
 *    сектора 5..12 — INFO.TXT (кластеры 2..9,   4 КБ)    
 *    сектора 13..20— SET.TXT  (кластеры 10..17, 4 КБ)    
 *    сектора 21..36— LOG.TXT  (кластеры 18..33, 8 КБ)    
 *    сектора 37..39— свободны    
 *    
 *  Wear leveling: 8 слотов по 20 КБ начиная с FLASH_TARGET_OFFSET.    
 *  SlotMeta (magic/seq/лимиты разгона/коррекция кварца) вшивается в    
 *  хвост ram_disk_buffer и путешествует вместе со слотом.    
 *    
 *  РАДИОМОДЕМ-РАЗЛИЧИЯ от маякового оригинала:    
 *  1) save_ram_to_flash(): парковка Core 1 (rp2040.idleOtherCore) вокруг    
 *     erase+program — Core 1 исполняет XIP-код и без парковки зависает;    
 *  2) тот же выход из разгона (soft_tx_boost_exit/enter) до/после    
 *     программирования — как в маяке;    
 *  3) init_usb_msc_interface() вызывается из setup() ДО Serial.begin —    
 *     до энумерации, иначе хост диск не увидит;    
 *  4) save_ram_to_flash() вызывается только при trx_state()==TRX_RX    
 *     (страж — в check_and_handle_pc_changes и прочих точках).    
 * ============================================================================    
 */    
  
#include "file_manager.h"  
#include <hardware/flash.h>  
#include <hardware/sync.h>  
#include "../trx/trx_control.h"      // trx_state(), TRX_RX  
#include "../vfo/soft_tx.h"          // soft_tx_boost_enter/exit, clk_boosted, vfo_max_*  
#include "../vfo/vfo.h"              // vfo_xosc_corr_ppb  
#include "../pins.h"                 // pin_freq_out, subband_pins (определены в pins.cpp)  
  
// ============ ФИЗИЧЕСКОЕ ОПРЕДЕЛЕНИЕ ОБЪЕКТОВ ============  
Adafruit_USBD_MSC usb_msc;  
String my_call_variable   = "";  
String my_qth_variable    = "";  
String my_text_variable   = "";  
String my_rtty_variable   = "";  
String my_ifkp_variable   = "";  
String my_cw_variable     = "";  
String my_freq_cw_var     = "";  
String my_cw_wpm_var      = "";  
String my_rtty_baud_var   = "";  
String my_rtty_mark_var   = "";  
String my_rtty_shift_var  = "";  
String my_rtty_invert_var = "";  
String my_freq_ifkp_var   = "";  
String my_freq_olivia_var = "";  
String my_freq_thor_var   = "";  
String my_freq_mfsk16_var = "";  
String my_freq_dominoex_var = "";  
String my_freq_fsq_var    = "";  
String my_freq_contestia_var = "";  
String my_freq_jtty_var   = "";  
String my_freq_ax25_var   = "";  
String my_FAT             = "";  
String scan_exclude_list  = "";  
String scan_result_data   = "";  
String my_debug_var       = "";  
String s_led_enable       = "";  
  
String my_ax25_dest_var      = "APRS";  
String my_ax25_txdelay_var   = "30";  
String my_ax25_invert_var    = "0";  
  
uint32_t CW_DOT_TIME_MS  = 60;  
volatile uint32_t RTTY_BIT_TIME_US = 22000;  
  
uint32_t climate_sample_interval_min = 10;  
uint32_t climate_report_interval_min = 60;  
  
// pin_freq_out и subband_pins — НЕ здесь: живут в src/pins.cpp,  
// доступ через extern из ../pins.h (иначе multiple definition при линковке).  
  
int pin_amp_act  = 11;  
int pin_pwr_si   = -1;  
int pin_pwr_ds   = -1;  
int pin_pwr_bm   = -1;  
int pin_pwr_dl   = -1;  
  
bool debug_flag = false;  
  
TaskItem beacon_schedule[MAX_SCHEDULE_TASKS];  
  
int32_t  current_active_slot = -1;  
uint32_t current_max_seq     = 0;  
  
// vfo_max_clk_sys_hz / vfo_max_pll_vco_hz — НЕ здесь: живут в src/vfo/soft_tx.cpp,  
// extern-объявлены в soft_tx.h. vfo_xosc_corr_ppb — в src/vfo/vfo.cpp, extern в vfo.h.  
  
volatile bool pc_file_written = false;  
bool led_enable_flag = true;  
int  si5351_clk_tx_out = 0;  
  
alignas(4) static uint8_t ram_disk_buffer[DISK_SIZE_BYTES];  
  
volatile uint32_t last_msc_write_time = 0;  
bool pc_activity_detected = false;  
volatile uint8_t pc_written_regions = 0;   // биты: 0-система/FAT, 1-INFO, 2-SET, 3-LOG  
  
// ============ TinyUSB MSC callbacks ============  
void msc_flush_cb(void) {  
  last_msc_write_time = millis();  
}  
  
int32_t msc_read_cb(uint32_t lba, void* buffer, uint32_t bufsize) {  
  if (lba >= SECTOR_COUNT) return -1;  
  memcpy(buffer, &ram_disk_buffer[lba * SECTOR_SIZE], bufsize);  
  return bufsize;  
}  
  
int32_t msc_write_cb(uint32_t lba, uint8_t* buffer, uint32_t bufsize) {  
  if (lba >= SECTOR_COUNT) return -1;  
  memcpy(&ram_disk_buffer[lba * SECTOR_SIZE], buffer, bufsize);  
  last_msc_write_time = millis();  
  pc_activity_detected = true;  
  
  // Карта затронутого региона под новую геометрию  
  if      (lba < INFO_DATA_SECTOR)        pc_written_regions |= 0x01; // бут/FAT/каталог  
  else if (lba < SET_DATA_SECTOR)         pc_written_regions |= 0x02; // INFO.TXT  
  else if (lba < LOG_DATA_SECTOR)         pc_written_regions |= 0x04; // SET.TXT  
  else                                  pc_written_regions |= 0x08; // LOG.TXT  
  
  // Прерываем эфир при изменении INFO.TXT или SET.TXT  
  uint32_t last_lba = lba + (bufsize / SECTOR_SIZE) - 1;  
  if (last_lba >= INFO_DATA_SECTOR && lba < LOG_DATA_SECTOR) {  
    pc_file_written = true;  
  }  
  return bufsize;  
}
  
// ============ FAT helpers ============    
static uint16_t fat_time_now() {    
  return ((uint16_t)rtc_hour << 11) | ((uint16_t)rtc_min << 5) | (rtc_sec / 2);    
}    
static uint16_t fat_date_now() {    
  uint16_t y = (rtc_year >= 1980) ? (rtc_year - 1980) : 0;    
  return (y << 9) | ((uint16_t)rtc_month << 5) | rtc_day;    
}    
  
// dir_entry_idx: 0=INFO.TXT, 1=SET.TXT, 2=LOG.TXT    
static void set_dir_timestamp(uint8_t dir_entry_idx, bool with_create) {    
  uint32_t e = SECTOR_SIZE * 2 + dir_entry_idx * 32;    
  uint16_t t = fat_time_now(), d = fat_date_now();    
  if (with_create) {    
    ram_disk_buffer[e + 13] = 0;    
    memcpy(&ram_disk_buffer[e + 14], &t, 2);    
    memcpy(&ram_disk_buffer[e + 16], &d, 2);    
  }    
  memcpy(&ram_disk_buffer[e + 18], &d, 2);    
  memcpy(&ram_disk_buffer[e + 22], &t, 2);    
  memcpy(&ram_disk_buffer[e + 24], &d, 2);    
}    
  
// ============ Сохранение ОЗУ во Flash с ротацией слотов ============    
static void save_ram_to_flash() {    
  int32_t next_slot = (current_active_slot + 1) % FLASH_SLOTS;    
  uint32_t next_seq = current_max_seq + 1;    
  if (current_active_slot == -1) { next_slot = 0; next_seq = 1; }    
  
  uint32_t target_flash_addr = FLASH_TARGET_OFFSET + (next_slot * SLOT_SIZE);    
  
  SlotMeta* meta = (SlotMeta*)&ram_disk_buffer[DISK_SIZE_BYTES - sizeof(SlotMeta)];    
  meta->magic = SLOT_MAGIC;    
  meta->seq = next_seq;    
  meta->max_clk_sys_hz = vfo_max_clk_sys_hz;    
  meta->max_pll_vco_hz = vfo_max_pll_vco_hz;    
  meta->xosc_corr_ppb  = vfo_xosc_corr_ppb;    
  
  // Flash нельзя программировать на разогнанной clk_sys — откат на номинал    
  bool was_boosted = clk_boosted;    
  if (was_boosted) soft_tx_boost_exit();    
  
  // Радиомодем: Core 1 исполняет XIP-код — паркуем на время erase+program,    
  // иначе сохранение = deadklock ядра (симптом: всё USB пропадает).    
  rp2040.idleOtherCore();    
  uint32_t ints = save_and_disable_interrupts();    
  flash_range_erase(target_flash_addr, SLOT_SIZE);    
  flash_range_program(target_flash_addr, ram_disk_buffer, DISK_SIZE_BYTES);    
  restore_interrupts(ints);    
  rp2040.resumeOtherCore();    
  flash_flush_cache();    
  
  if (was_boosted) soft_tx_boost_enter(0);    
  
  current_active_slot = next_slot;    
  current_max_seq = next_seq;    
  
  Serial.printf("[FM] flash save: slot=%ld seq=%lu\n",    
                (long)current_active_slot, (unsigned long)current_max_seq);    
}    
  
uint64_t flash_limits_clk_sys_hz(void) { return (uint64_t)vfo_max_clk_sys_hz; }    
uint64_t flash_limits_vco_hz(void)     { return (uint64_t)vfo_max_pll_vco_hz; }    
  
// persist_clock_limits: пишет LimitsRec через штатный механизм слота.    
void persist_clock_limits(uint64_t new_clk_hz, uint64_t new_vco_hz) {    
  if (new_clk_hz == 0 || new_clk_hz < vfo_max_clk_sys_hz) new_clk_hz = vfo_max_clk_sys_hz;    
  if (new_vco_hz == 0 || new_vco_hz < vfo_max_pll_vco_hz) new_vco_hz = vfo_max_pll_vco_hz;    
  if (new_clk_hz == vfo_max_clk_sys_hz && new_vco_hz == vfo_max_pll_vco_hz) return;    
  
  vfo_max_clk_sys_hz = (uint32_t)new_clk_hz;    
  vfo_max_pll_vco_hz = (uint32_t)new_vco_hz;    
  
  save_ram_to_flash();    
  Serial.printf("[LIMITS] flash stored: clk_sys<=%lu Hz, VCO<=%lu Hz (slot=%ld seq=%lu)\n",    
                (unsigned long)vfo_max_clk_sys_hz,    
                (unsigned long)vfo_max_pll_vco_hz,    
                (long)current_active_slot, (unsigned long)current_max_seq);    
}    
  
static void set_fat12_entry(uint32_t fat_start_bytes, uint16_t cluster, uint16_t value) {    
  uint32_t byte_offset = fat_start_bytes + ((cluster * 3) / 2);    
  if (cluster % 2 == 0) {    
    ram_disk_buffer[byte_offset] = (uint8_t)(value & 0xFF);    
    ram_disk_buffer[byte_offset + 1] = (ram_disk_buffer[byte_offset + 1] & 0xF0) | ((value >> 8) & 0x0F);    
  } else {    
    ram_disk_buffer[byte_offset] = (ram_disk_buffer[byte_offset] & 0x0F) | ((value << 4) & 0xF0);    
    ram_disk_buffer[byte_offset + 1] = (uint8_t)((value >> 4) & 0xFF);    
  }    
}    
  
// ============ Генерация диска по умолчанию ============    
static void create_default_fat_with_info_file() {    
  Serial.println("[Система] Генерируем диск FAT12 20КБ (каталог: 48 записей)...");    
  memset(ram_disk_buffer, 0, DISK_SIZE_BYTES);    
  
  // СЕКТОР 0: boot + BPB    
  ram_disk_buffer[0] = 0xEB; ram_disk_buffer[1] = 0x3C; ram_disk_buffer[2] = 0x90;    
  memcpy(&ram_disk_buffer[3], "MSDOS5.0", 8);    
  ram_disk_buffer[11] = (uint8_t)(SECTOR_SIZE & 0xFF);    
  ram_disk_buffer[12] = (uint8_t)((SECTOR_SIZE >> 8) & 0xFF);    
  ram_disk_buffer[13] = 1;                 // сектор/кластер    
  ram_disk_buffer[14] = 1; ram_disk_buffer[15] = 0;   // reserved    
  ram_disk_buffer[16] = 1;                 // копий FAT    
  ram_disk_buffer[17] = 48;                // 48 записей каталога (сектора 2..4)    
  ram_disk_buffer[18] = 0;    
  ram_disk_buffer[19] = (uint8_t)(SECTOR_COUNT & 0xFF);    
  ram_disk_buffer[20] = (uint8_t)((SECTOR_COUNT >> 8) & 0xFF);    
  ram_disk_buffer[21] = 0xF8;              // media    
  ram_disk_buffer[22] = 1;                 // секторов/FAT    
  ram_disk_buffer[23] = 0;    
  ram_disk_buffer[24] = 0x01; ram_disk_buffer[25] = 0x00;    
  ram_disk_buffer[26] = 0x01; ram_disk_buffer[27] = 0x00;    
  ram_disk_buffer[28] = 0; ram_disk_buffer[29] = 0;    
  ram_disk_buffer[30] = 0; ram_disk_buffer[31] = 0;    
  ram_disk_buffer[32] = 0; ram_disk_buffer[33] = 0;    
  ram_disk_buffer[34] = 0; ram_disk_buffer[35] = 0;    
  ram_disk_buffer[36] = 0x80;    
  ram_disk_buffer[37] = 0x00;    
  ram_disk_buffer[38] = 0x29;    
  ram_disk_buffer[39] = 0xDE; ram_disk_buffer[40] = 0xAD;    
  ram_disk_buffer[41] = 0xBE; ram_disk_buffer[42] = 0xEF;    
  memcpy(&ram_disk_buffer[43], "PICO DRIVE ", 11);    
  memcpy(&ram_disk_buffer[54], "FAT12   ", 8);    
  ram_disk_buffer[510] = 0x55; ram_disk_buffer[511] = 0xAA;    
  
  // СЕКТОР 1: FAT12    
  uint32_t fat_offset = SECTOR_SIZE * 1;    
  ram_disk_buffer[fat_offset + 0] = 0xF8;    
  ram_disk_buffer[fat_offset + 1] = 0xFF;    
  ram_disk_buffer[fat_offset + 2] = 0xFF;    
  
  for (uint16_t i = INFO_FIRST_CLUSTER; i < INFO_FIRST_CLUSTER + INFO_CLUSTERS - 1; i++)    
    set_fat12_entry(fat_offset, i, i + 1);    
  set_fat12_entry(fat_offset, INFO_FIRST_CLUSTER + INFO_CLUSTERS - 1, 0xFFF);    
  
  for (uint16_t i = SET_FIRST_CLUSTER; i < SET_FIRST_CLUSTER + SET_CLUSTERS - 1; i++)    
    set_fat12_entry(fat_offset, i, i + 1);    
  set_fat12_entry(fat_offset, SET_FIRST_CLUSTER + SET_CLUSTERS - 1, 0xFFF);    
  
  for (uint16_t i = LOG_FIRST_CLUSTER; i < LOG_FIRST_CLUSTER + LOG_CLUSTERS - 1; i++)    
    set_fat12_entry(fat_offset, i, i + 1);    
  set_fat12_entry(fat_offset, LOG_FIRST_CLUSTER + LOG_CLUSTERS - 1, 0xFFF);    
  
  // СЕКТОР 2: корневой каталог    
  uint32_t root_offset = SECTOR_SIZE * 2;    
  
  memcpy(&ram_disk_buffer[root_offset + 0], "INFO    ", 8);    
  memcpy(&ram_disk_buffer[root_offset + 8], "TXT", 3);    
  ram_disk_buffer[root_offset + 26] = INFO_FIRST_CLUSTER;    
  
  const char* default_info_content =    
    "[CALL]=RU0AOG\r\n"    
    "[QTH]=NO66FC\r\n"    
    "[TEXT]=TESTING BEACON\r\n\r\n"    
    "[CW_WPM    ]=20\r\n"    
    "[RTTY_SPEED]=45.45\r\n\r\n"    
    "[FREQ_CW   ]=3600000\r\n"    
    "[RTTY_BASE ]=3600000\r\n"    
    "[RTTY_SHIFT]=170\r\n"    
    "[RTTY_INVERT]=0\r\n"    
    "[FREQ_IFKP ]=3600000\r\n"    
    "[FREQ_OLIVIA]=3600000\r\n"    
    "[FREQ_CONTESTIA]=3600000\r\n"    
    "[FREQ_THOR]=3600000\r\n"    
    "[FREQ_MFSK16]=3600000\r\n"    
    "[FREQ_DOMINOEX]=3600000\r\n"    
    "[FREQ_FSQ]=3600000\r\n"    
    "[FREQ_JTTY]=3600000\r\n"    
    "[FREQ_AX25]=3600000\r\n\r\n"    
    "=== МАТРИЦА РАСПИСАНИЯ ПЕРЕДАЧ ===\r\n"    
    "// ДНИ: 1=Пн..7=Вс, 0=Каждый день; МОДЫ: CW,RTTY,IFKP,...,SEQ\r\n"    
    "// [TASK_01]=0,09:00/22:00/5,3591500,RTTY\r\n"    
    "[EOF]";    
  
  uint32_t info_len = strlen(default_info_content);    
  ram_disk_buffer[root_offset + 28] = (uint8_t)(info_len & 0xFF);    
  ram_disk_buffer[root_offset + 29] = (uint8_t)((info_len >> 8) & 0xFF);    
  
  uint32_t set_entry_offset = root_offset + 32;    
  memcpy(&ram_disk_buffer[set_entry_offset + 0], "SET     ", 8);    
  memcpy(&ram_disk_buffer[set_entry_offset + 8], "TXT", 3);    
  ram_disk_buffer[set_entry_offset + 26] = SET_FIRST_CLUSTER;    
  
  auto pin_to_str = [](int p) -> String { return (p == -1) ? "NC" : String(p); };    
  
  String content = "";    
  content.reserve(512);    
  content += "=== ИНЖЕНЕРНЫЕ НАСТРОЙКИ ===\r\n";    
  content += "[PIN_FREQ_OUT ]=" + pin_to_str(pin_freq_out) + "\r\n";    
  content += "[PIN_AMP_ACT  ]=" + pin_to_str(pin_amp_act) + "\r\n\r\n";    
  content += "[SI5351_CLK_OUT]=" + String(si5351_clk_tx_out) + "\r\n\r\n";    
  content += "[PIN_SUBBAND_0]=" + pin_to_str(subband_pins[0]) + "\r\n";    
  content += "[PIN_SUBBAND_1]=" + pin_to_str(subband_pins[1]) + "\r\n";    
  content += "[PIN_SUBBAND_2]=" + pin_to_str(subband_pins[2]) + "\r\n";    
  content += "[PIN_SUBBAND_3]=" + pin_to_str(subband_pins[3]) + "\r\n\r\n";    
  content += "[PIN_PWR_SI   ]=" + pin_to_str(pin_pwr_si) + "\r\n";    
  content += "[PIN_PWR_DS   ]=" + pin_to_str(pin_pwr_ds) + "\r\n";    
  content += "[PIN_PWR_BM   ]=" + pin_to_str(pin_pwr_bm) + "\r\n";    
  content += "[PIN_PWR_DL   ]=" + pin_to_str(pin_pwr_dl) + "\r\n\r\n";    
  content += "[SCAN_EXCLUDE ]=16,23,24,25\r\n\r\n";    
  content += "[DEBUG]=" + String(debug_flag ? 1 : 0) + "\r\n\r\n";    
  content += "[LED_ENABLE]=" + String(led_enable_flag ? 1 : 0) + "\r\n\r\n";    
  content += "=== ПЕРИОДИЧНОСТЬ КЛИМАТИЧЕСКОГО МОНИТОРИНГА ===\r\n";    
  content += "[CLIM_SAMPLE_MIN]=" + String(climate_sample_interval_min) + "\r\n";    
  content += "[CLIM_REPORT_MIN]=" + String(climate_report_interval_min) + "\r\n\r\n";    
  content += "[XOSC_PPB]=" + String((long)vfo_xosc_corr_ppb) + "\r\n\r\n";    
  content += "=== СТАТИСТИКА ИЗНОСА ФЛЭШ-ПАМЯТИ ===\r\n";    
  content += "[FLASH_SLOT   ]=" + String(current_active_slot != -1 ? current_active_slot : 0) + "\r\n";    
  content += "[FLASH_SEQ    ]=" + String(current_max_seq != 0 ? current_max_seq : 1) + "\r\n\r\n";    
  content += "=== УСТРОЙСТВА НА ШИНЕ I2C ===\r\n";    
  content += "[SCAN_RESULT]\r\nСканирование не проводилось.\r\n\r\n";    
  content += "[EOF]";    
  
  uint32_t set_len = content.length();    
  ram_disk_buffer[set_entry_offset + 28] = (uint8_t)(set_len & 0xFF);    
  ram_disk_buffer[set_entry_offset + 29] = (uint8_t)((set_len >> 8) & 0xFF);    
  
  uint32_t log_entry_offset = root_offset + 64;    
  memcpy(&ram_disk_buffer[log_entry_offset + 0], "LOG     ", 8);    
  memcpy(&ram_disk_buffer[log_entry_offset + 8], "TXT", 3);    
  ram_disk_buffer[log_entry_offset + 26] = LOG_FIRST_CLUSTER;    
  
  String default_log_content = "           === ЖУРНАЛ РАБОТЫ РАДИОМОДЕМА ===\r\n"    
                               "        Версия ПО v." + String(BCN_VER) +    
                               " от " + String(BCN_DAT) + "\r\n";    
  uint32_t log_len = default_log_content.length();    
  ram_disk_buffer[log_entry_offset + 28] = (uint8_t)(log_len & 0xFF);    
  ram_disk_buffer[log_entry_offset + 29] = (uint8_t)((log_len >> 8) & 0xFF);    
  
  // Области данных: производные сектора из констант кластеров    
  memcpy(&ram_disk_buffer[SECTOR_SIZE * INFO_DATA_SECTOR], default_info_content, info_len);    
  memcpy(&ram_disk_buffer[SECTOR_SIZE * SET_DATA_SECTOR],  content.c_str(), set_len);    
  memcpy(&ram_disk_buffer[SECTOR_SIZE * LOG_DATA_SECTOR],  default_log_content.c_str(), log_len);    
  
  set_dir_timestamp(0, true);    
  set_dir_timestamp(1, true);    
  set_dir_timestamp(2, true);    
  
  save_ram_to_flash();    
  Serial.println("[Система] Диск обновлён: INFO(4K) SET(4K) LOG(8K)");    
}    
  
// ============ Парсер SET.TXT (helpers) ============    
static inline int parse_pin_value(const String& val) {    
  String tmp = val;    
  tmp.trim();    
  if (tmp.equalsIgnoreCase("NC") || tmp.length() == 0) return -1;    
  return tmp.toInt();    
}    
  
// ============ read_and_parse_INFO_txt (verbatim из маяка) ============    
void read_and_parse_INFO_txt() {    
  flash_flush_cache();    
  
  my_call_variable = "";  my_qth_variable  = "";  my_text_variable = "";    
  my_rtty_variable = "";  my_ifkp_variable = "";  my_cw_variable = "";    
  my_freq_cw_var = "";    my_freq_ifkp_var = "";  my_rtty_mark_var = "";    
  my_freq_olivia_var = ""; my_freq_thor_var = ""; my_freq_contestia_var = "";    
  my_freq_jtty_var = "";    
  my_freq_mfsk16_var = ""; my_freq_dominoex_var = ""; my_freq_fsq_var = "";    
  my_freq_ax25_var = "";    
  my_rtty_shift_var = ""; my_rtty_invert_var = ""; my_cw_wpm_var = "";    
  my_rtty_baud_var = "";    
  
  for (int t = 0; t < MAX_SCHEDULE_TASKS; t++) beacon_schedule[t].active = false;    
  
  uint32_t scan_start = INFO_DATA_SECTOR * SECTOR_SIZE;    
  uint32_t scan_end   = scan_start + (INFO_CLUSTERS * SECTOR_SIZE);    
  int task_counter = 0;    
  
  for (uint32_t i = scan_start; i < scan_end - 15; i++) {    
    if (ram_disk_buffer[i] == '[' &&    
    (i == scan_start || ram_disk_buffer[i-1] == '\n' ||    
     ram_disk_buffer[i-1] == '\r' || (uint8_t)ram_disk_buffer[i-1] < 32 ||    
     (uint8_t)ram_disk_buffer[i-1] >= 0x80)) {    
          int32_t start_idx = -1;    
          String* target_str = nullptr;    
          bool is_task_line = false;    
  
          uint32_t close_bracket_idx = 0;    
          for (uint32_t k = i + 1; k < i + 20 && k < scan_end; k++) {    
            if (ram_disk_buffer[k] == ']') { close_bracket_idx = k; break; }    
          }    
          if (close_bracket_idx == 0) continue;    
  
          String tag = "";    
          for (uint32_t k = i + 1; k < close_bracket_idx; k++) {    
            char c = (char)ram_disk_buffer[k];    
            if (c != ' ') tag += c;    
          }    
  
          if      (tag == "CALL")           target_str = &my_call_variable;    
          else if (tag == "QTH")            target_str = &my_qth_variable;    
          else if (tag == "TEXT")           target_str = &my_text_variable;    
          else if (tag == "CW_WPM")         target_str = &my_cw_wpm_var;    
          else if (tag == "RTTY_SPEED")     target_str = &my_rtty_baud_var;    
          else if (tag == "FREQ_CW")        target_str = &my_freq_cw_var;    
          else if (tag == "RTTY_BASE")      target_str = &my_rtty_mark_var;    
          else if (tag == "RTTY_SHIFT")     target_str = &my_rtty_shift_var;    
          else if (tag == "RTTY_INVERT")    target_str = &my_rtty_invert_var;    
          else if (tag == "FREQ_IFKP")      target_str = &my_freq_ifkp_var;    
          else if (tag == "FREQ_OLIVIA")    target_str = &my_freq_olivia_var;    
          else if (tag == "FREQ_THOR")      target_str = &my_freq_thor_var;    
          else if (tag == "FREQ_CONTESTIA") target_str = &my_freq_contestia_var;    
          else if (tag == "FREQ_MFSK16")    target_str = &my_freq_mfsk16_var;    
          else if (tag == "FREQ_DOMINOEX")  target_str = &my_freq_dominoex_var;    
          else if (tag == "FREQ_FSQ")       target_str = &my_freq_fsq_var;    
          else if (tag == "FREQ_JTTY")      target_str = &my_freq_jtty_var;    
          else if (tag == "CW")             target_str = &my_cw_variable;    
          else if (tag == "RTTY")           target_str = &my_rtty_variable;    
          else if (tag == "IFKP")           target_str = &my_ifkp_variable;    
          else if (tag == "FREQ_AX25")      target_str = &my_freq_ax25_var;    
          else if (tag == "AX25_DEST")      target_str = &my_ax25_dest_var;    
          else if (tag == "AX25_TXDELAY")   target_str = &my_ax25_txdelay_var;    
          else if (tag == "AX25_INVERT")    target_str = &my_ax25_invert_var;    
          else if (tag.startsWith("TASK_")) is_task_line = true;    
  
          if (target_str != nullptr || is_task_line) {    
            start_idx = close_bracket_idx + 1;    
          }    
  
      if (start_idx != -1) {    
        if (ram_disk_buffer[start_idx] == '=') start_idx++;    
  
        String value = "";    
        value.reserve(64);    
  
        for (uint32_t j = start_idx; j < scan_end; j++) {    
          char c = (char)ram_disk_buffer[j];    
          if (c == '\n' || c == '\r' || c == '[') { i = j - 1; break; }    
  
          if (!is_task_line && (target_str == &my_freq_ifkp_var || target_str == &my_rtty_mark_var || target_str == &my_freq_olivia_var || target_str == &my_freq_thor_var || target_str == &my_freq_contestia_var ||    
              target_str == &my_freq_mfsk16_var || target_str == &my_freq_dominoex_var || target_str == &my_freq_fsq_var || target_str == &my_freq_jtty_var || target_str == &my_freq_ax25_var ||    
              target_str == &my_rtty_shift_var || target_str == &my_rtty_invert_var || target_str == &my_freq_cw_var)) {    
            if (c >= '0' && c <= '9') value += c;    
          } else {    
            if (c >= 32) value += c;    
          }    
        }    
        value.trim();    
  
        if (is_task_line) {    
          int comma1 = value.indexOf(',');    
          int comma2 = value.indexOf(',', comma1 + 1);    
          int comma3 = value.indexOf(',', comma2 + 1);    
  
          if (comma1 != -1 && comma2 != -1 && comma3 != -1 && task_counter < MAX_SCHEDULE_TASKS) {    
            String s_days = value.substring(0, comma1);    
            String s_time = value.substring(comma1 + 1, comma2);    
            String s_freq = value.substring(comma2 + 1, comma3);    
            String s_mode = value.substring(comma3 + 1);    
            s_days.trim(); s_time.trim(); s_freq.trim(); s_mode.trim();    
  
            TaskItem& task = beacon_schedule[task_counter];    
            task.days = 0;    
  
            if (!s_days.equals("0")) {    
              for (size_t ch = 0; ch < s_days.length(); ch++) {    
                char d_char = s_days[ch];    
                if (d_char >= '1' && d_char <= '7') {    
                  task.days |= (1 << (d_char - '0'));    
                }    
              }    
            }    
            task.freq_hz = strtoul(s_freq.c_str(), NULL, 10);    
  
            if      (s_mode.equalsIgnoreCase("CW"))        task.mode = MODE_CW;    
            else if (s_mode.equalsIgnoreCase("RTTY"))      task.mode = MODE_RTTY;    
            else if (s_mode.equalsIgnoreCase("IFKP"))      task.mode = MODE_IFKP;    
            else if (s_mode.equalsIgnoreCase("OLIVIA"))    task.mode = MODE_OLIVIA;    
            else if (s_mode.equalsIgnoreCase("CONTESTIA")) task.mode = MODE_CONTESTIA;    
            else if (s_mode.equalsIgnoreCase("THOR"))      task.mode = MODE_THOR;    
            else if (s_mode.equalsIgnoreCase("MFSK16"))    task.mode = MODE_MFSK16;    
            else if (s_mode.equalsIgnoreCase("DOMINOEX"))  task.mode = MODE_DOMINOEX;    
            else if (s_mode.equalsIgnoreCase("FSQ"))       task.mode = MODE_FSQ;    
            else if (s_mode.equalsIgnoreCase("SEQ"))       task.mode = MODE_SEQ;    
            else continue;    
  
            int slash1 = s_time.indexOf('/');    
            if (slash1 == -1) {    
              int colon = s_time.indexOf(':');    
              if (colon != -1) {    
                task.start_hour = s_time.substring(0, colon).toInt();    
                task.start_min  = s_time.substring(colon + 1).toInt();    
                task.end_hour   = task.start_hour;    
                task.end_min    = task.start_min;    
                task.interval_min = 0;    
              }    
            } else {    
              int slash2 = s_time.indexOf('/', slash1 + 1);    
              if (slash2 != -1) {    
                String s_start = s_time.substring(0, slash1);    
                String s_end   = s_time.substring(slash1 + 1, slash2);    
                task.interval_min = s_time.substring(slash2 + 1).toInt();    
                int colon1 = s_start.indexOf(':');    
                int colon2 = s_end.indexOf(':');    
                if (colon1 != -1 && colon2 != -1) {    
                  task.start_hour = s_start.substring(0, colon1).toInt();    
                  task.start_min  = s_start.substring(colon1 + 1).toInt();    
                  task.end_hour   = s_end.substring(0, colon2).toInt();    
                  task.end_min    = s_end.substring(colon2 + 1).toInt();    
                }    
              }    
            }    
            task.active = true;    
            task_counter++;    
          }    
        } else if (target_str != nullptr) {    
          *target_str = value;    
        }    
      }    
    }    
  }    
  
  if (my_cw_wpm_var.length() > 0) {    
    int wpm = my_cw_wpm_var.toInt();    
    if (wpm < 5)  wpm = 5;    
    if (wpm > 50) wpm = 50;    
    my_cw_wpm_var = String(wpm);    
    CW_DOT_TIME_MS = 1200 / wpm;    
  } else {    
    CW_DOT_TIME_MS = 60;    
    my_cw_wpm_var = "20";    
  }    
  
  if (my_rtty_baud_var.length() > 0) {    
    float baud = my_rtty_baud_var.toFloat();    
    if (baud > 0.0f) RTTY_BIT_TIME_US = (uint32_t)(1000000.0f / baud);    
  }    
  
  if (debug_flag) Serial.println("[Система] парсинг INFO.TXT");    
}    
  
// ============ read_and_parse_SET_txt (verbatim) ============    
void read_and_parse_SET_txt() {    
  flash_flush_cache();    
  
  String s_freq_out = "", s_amp_act = "";    
  String s_subband[4] = {"", "", "", ""};    
  String s_pwr_si = "", s_pwr_ds = "", s_pwr_bm = "", s_pwr_dl = "";    
  scan_exclude_list = "";    
  scan_result_data  = "";    
  my_debug_var      = "";    
  s_led_enable = "";    
  String s_si_clk = "";    
  String s_clim_sample = "", s_clim_report = "", s_xosc_ppb = "";    
  
  uint32_t scan_start = SET_DATA_SECTOR * SECTOR_SIZE;    
  uint32_t scan_end   = scan_start + (SET_CLUSTERS * SECTOR_SIZE);    
  
  for (uint32_t i = scan_start; i < scan_end - 15; i++) {    
    if (ram_disk_buffer[i] == '[' &&    
    (i == scan_start || ram_disk_buffer[i-1] == '\n' ||    
     ram_disk_buffer[i-1] == '\r' || (uint8_t)ram_disk_buffer[i-1] < 32 ||    
     (uint8_t)ram_disk_buffer[i-1] >= 0x80)) {    
          int32_t start_idx = -1;    
          String* target_str = nullptr;    
  
          uint32_t close_bracket_idx = 0;    
          for (uint32_t k = i + 1; k < i + 20 && k < scan_end; k++) {    
            if (ram_disk_buffer[k] == ']') { close_bracket_idx = k; break; }    
          }    
          if (close_bracket_idx == 0) continue;    
  
          String tag = "";    
          for (uint32_t k = i + 1; k < close_bracket_idx; k++) {    
            char c = (char)ram_disk_buffer[k];    
            if (c != ' ') tag += c;    
          }    
  
          if      (tag == "PIN_FREQ_OUT")    target_str = &s_freq_out;    
          else if (tag == "PIN_AMP_ACT")     target_str = &s_amp_act;    
          else if (tag == "SI5351_CLK_OUT")  target_str = &s_si_clk;    
          else if (tag == "LED_ENABLE")      target_str = &s_led_enable;    
          else if (tag == "PIN_SUBBAND_0")   target_str = &s_subband[0];    
          else if (tag == "PIN_SUBBAND_1")   target_str = &s_subband[1];    
          else if (tag == "PIN_SUBBAND_2")   target_str = &s_subband[2];    
          else if (tag == "PIN_SUBBAND_3")   target_str = &s_subband[3];    
          else if (tag == "PIN_PWR_SI")      target_str = &s_pwr_si;    
          else if (tag == "PIN_PWR_DS")      target_str = &s_pwr_ds;    
          else if (tag == "PIN_PWR_BM")      target_str = &s_pwr_bm;    
          else if (tag == "PIN_PWR_DL")      target_str = &s_pwr_dl;    
          else if (tag == "SCAN_EXCLUDE")    target_str = &scan_exclude_list;    
          else if (tag == "DEBUG")           target_str = &my_debug_var;    
          else if (tag == "CLIM_SAMPLE_MIN") target_str = &s_clim_sample;    
          else if (tag == "CLIM_REPORT_MIN") target_str = &s_clim_report;    
          else if (tag == "XOSC_PPB")        target_str = &s_xosc_ppb;    
  
          if (target_str != nullptr) start_idx = close_bracket_idx + 1;    
  
      if (start_idx != -1 && target_str != nullptr) {    
        target_str->reserve(64);    
        if (ram_disk_buffer[start_idx] == '=') start_idx++;    
        for (uint32_t j = start_idx; j < scan_end; j++) {    
          char c = (char)ram_disk_buffer[j];    
          if (c == '\n' || c == '\r' || c == '[') { i = j - 1; break; }    
          if (c >= 32) *target_str += c;    
        }    
        target_str->trim();    
      }    
    }    
  }    
  
  debug_flag = (my_debug_var.toInt() != 0);    
  pin_freq_out = (s_freq_out.length() > 0) ? parse_pin_value(s_freq_out) : 14;    
  pin_amp_act  = (s_amp_act.length() > 0)  ? parse_pin_value(s_amp_act)  : 15;    
  for (int k = 0; k < 4; k++)    
    subband_pins[k] = (s_subband[k].length() > 0) ? parse_pin_value(s_subband[k]) : (2 + k);    
  pin_pwr_si = (s_pwr_si.length() > 0) ? parse_pin_value(s_pwr_si) : -1;    
  pin_pwr_ds = (s_pwr_ds.length() > 0) ? parse_pin_value(s_pwr_ds) : -1;    
  pin_pwr_bm = (s_pwr_bm.length() > 0) ? parse_pin_value(s_pwr_bm) : -1;    
  pin_pwr_dl = (s_pwr_dl.length() > 0) ? parse_pin_value(s_pwr_dl) : -1;    
  led_enable_flag = (s_led_enable.length() > 0) ? (s_led_enable.toInt() != 0) : true;    
  int parsed_clk = (s_si_clk.length() > 0) ? s_si_clk.toInt() : 0;    
  si5351_clk_tx_out = (parsed_clk >= 0 && parsed_clk <= 2) ? parsed_clk : 0;    
  if (s_clim_sample.length() > 0) {    
    uint32_t val = s_clim_sample.toInt();    
    climate_sample_interval_min = (val > 0) ? val : 1;    
  }    
  if (s_clim_report.length() > 0) {    
    uint32_t val = s_clim_report.toInt();    
    climate_report_interval_min = (val > 0) ? val : 1;    
  }    
  // Коррекция XOSC (в маяке закомментирована — оставляем как есть)    
  //if (s_xosc_ppb.length() > 0) {    
  //  vfo_xosc_corr_ppb = s_xosc_ppb.toInt();    
  //  if (vfo_xosc_corr_ppb >  100000) vfo_xosc_corr_ppb =  100000;    
  //  if (vfo_xosc_corr_ppb < -100000) vfo_xosc_corr_ppb = -100000;    
  //}    
  if (debug_flag) Serial.println("[Система] парсинг SET.TXT");    
}    
  
// ============ print_current_settings (verbatim) ============    
void print_current_settings() {    
  my_cw_variable   = fmt_next_start(get_next_start_minute(MODE_CW));    
  my_rtty_variable = fmt_next_start(get_next_start_minute(MODE_RTTY));    
  my_ifkp_variable = fmt_next_start(get_next_start_minute(MODE_IFKP));    
  Serial.println("=== ТЕКУЩИЕ НАСТРОЙКИ ===");    
  Serial.print("Позывной [CALL      ]: "); Serial.println(my_call_variable.length() > 0 ? my_call_variable : "Не задан");    
  Serial.print("Локатор  [QTH       ]: "); Serial.println(my_qth_variable.length()  > 0 ? my_qth_variable  : "Не задан");    
  Serial.print("Текст    [TEXT      ]: "); Serial.println(my_text_variable.length() > 0 ? my_text_variable : "Не задан");    
  Serial.print("Таймер   [START_CW  ]: "); Serial.println(my_cw_variable.length()   > 0 ? my_cw_variable   : "Не задан");    
  Serial.print("Таймер   [START_RTTY]: "); Serial.println(my_rtty_variable.length() > 0 ? my_rtty_variable : "Не задан");    
  Serial.print("Таймер   [START_IFKP]: "); Serial.println(my_ifkp_variable.length() > 0 ? my_ifkp_variable : "Не задан");    
  Serial.print("Скорость CW [CW_WPM ]: "); Serial.print(my_cw_wpm_var); Serial.print(" WPM (точка "); Serial.print(CW_DOT_TIME_MS); Serial.println(" мс)");    
  Serial.print("Скорость [RTTY_SPEED]: "); Serial.print(my_rtty_baud_var); Serial.print(" Baud (бит "); Serial.print(RTTY_BIT_TIME_US/1000); Serial.println(" мс)");    
  Serial.print("Частота  [FREQ_CW   ]: "); Serial.print(my_freq_cw_var); Serial.println(" Гц");    
  Serial.print("Частота  [RTTY_MARK ]: "); Serial.print(my_rtty_mark_var); Serial.println(" Гц");    
  Serial.print("Сдвиг    [RTTY_SHIFT]: "); Serial.print(my_rtty_shift_var); Serial.println(" Гц");    
  Serial.print("Инверсия [RTTY_INV  ]: "); Serial.println(my_rtty_invert_var == "1" ? "ВКЛ" : "ВЫКЛ");    
  Serial.println("--- Аппаратная конфигурация (SET.TXT) ---");    
  Serial.print("Пин ВЧ-выхода   (FREQ_OUT): "); Serial.println(pin_freq_out);    
  Serial.print("Пин активации УМ (AMP_ACT): "); Serial.println(pin_amp_act);    
  Serial.println("--- Статистика износа флеш-памяти ---");    
  Serial.print("Активный слот флеша   : "); Serial.println(current_active_slot);    
  Serial.print("Счетчик записей (seq) : "); Serial.println(current_max_seq);    
  Serial.print("Физический адрес флеш : 0x");    
  Serial.println(0x10000000 + FLASH_TARGET_OFFSET + (current_active_slot * SLOT_SIZE), HEX);    
  Serial.println("=====================================");    
}    
  
// ============ USB-MSC init ============    
// Регистрация MSC в TinyUSB — СТРОГО до маунта (в setup() до Serial.begin).    
void init_usb_msc_interface() {    
  usb_msc.setCapacity(SECTOR_COUNT, SECTOR_SIZE);    
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);    
  usb_msc.setID("RU0AOG", "Radiomodem", "2.0");    
  usb_msc.begin();    
  usb_msc.setUnitReady(false);    
}    
  
void init_flash_disk() {    
  usb_msc.setUnitReady(true);    
}    
  
// ============ init_file_manager: загрузка слота из Flash ============    
void init_file_manager() {    
  flash_flush_cache();    
  
  current_active_slot = -1;    
  current_max_seq = 0;    
  bool slot_found = false;    
  SlotMeta checked_meta;    
  
  for (int i = 0; i < FLASH_SLOTS; i++) {    
    uint32_t flash_addr_abs = 0x10000000 + FLASH_TARGET_OFFSET + (i * SLOT_SIZE);    
    uint32_t meta_offset_abs = flash_addr_abs + DISK_SIZE_BYTES - sizeof(SlotMeta);    
    memcpy(&checked_meta, (const void*)meta_offset_abs, sizeof(SlotMeta));    
  
    uint32_t boot_sig_offset_abs = flash_addr_abs + 510;    
    uint8_t sig_low  = *(const uint8_t*)(boot_sig_offset_abs);    
    uint8_t sig_high = *(const uint8_t*)(boot_sig_offset_abs + 1);    
    uint8_t root_entries_count = *(const uint8_t*)(flash_addr_abs + 17);    
  
    if (checked_meta.magic == SLOT_MAGIC && sig_low == 0x55 && sig_high == 0xAA && root_entries_count == 48) {    
      if (checked_meta.seq >= current_max_seq) {    
        current_max_seq = checked_meta.seq;    
        current_active_slot = i;    
        slot_found = true;    
        if (checked_meta.max_clk_sys_hz >= PASSPORT_CLK_SYS_MAX_HZ &&    
            checked_meta.max_clk_sys_hz <= 1000000000UL)    
          vfo_max_clk_sys_hz = checked_meta.max_clk_sys_hz;    
        if (checked_meta.max_pll_vco_hz >= PASSPORT_PLL_VCO_MAX_HZ &&    
            checked_meta.max_pll_vco_hz <= 6000000000ULL)    
          vfo_max_pll_vco_hz = checked_meta.max_pll_vco_hz;    
        if (checked_meta.xosc_corr_ppb > -500000 &&    
            checked_meta.xosc_corr_ppb <  500000 &&    
            checked_meta.xosc_corr_ppb != -1)    
          vfo_xosc_corr_ppb = checked_meta.xosc_corr_ppb;    
      }    
    }    
  }    
  
  if (slot_found) {    
    uint32_t final_flash_src = 0x10000000 + FLASH_TARGET_OFFSET + (current_active_slot * SLOT_SIZE);    
    memcpy(ram_disk_buffer, (const void*)final_flash_src, DISK_SIZE_BYTES);    
  
    uint8_t info_name_byte = ram_disk_buffer[SECTOR_SIZE * 2 + 0];    
    uint8_t set_name_byte  = ram_disk_buffer[SECTOR_SIZE * 2 + 32];    
    uint8_t log_name_byte  = ram_disk_buffer[SECTOR_SIZE * 2 + 64];    
  
    if (info_name_byte == 0x00 || info_name_byte == 0xE5 ||    
        set_name_byte  == 0x00 || set_name_byte  == 0xE5 ||    
        log_name_byte  == 0x00 || log_name_byte  == 0xE5) {    
      Serial.println(F("[Система] Файлы удалены ПК! Принудительное восстановление..."));    
      create_default_fat_with_info_file();    
      my_FAT = "Структура файлов была нарушена или удалена ПК. Всё восстановлено.";    
    } else {    
      my_FAT = "Файловая система корректна. Слот " + String(current_active_slot) +    
               " (seq=" + String(current_max_seq) + ")";    
    }    
  } else {    
    create_default_fat_with_info_file();    
    my_FAT = "Файловая система не найдена. Восстановлен дефолт.";    
  }    
  
  read_and_parse_INFO_txt();    
  read_and_parse_SET_txt();    
  
  const char* src = "passport";    
  if (vfo_max_clk_sys_hz > PASSPORT_CLK_SYS_MAX_HZ ||    
      vfo_max_pll_vco_hz > PASSPORT_PLL_VCO_MAX_HZ) src = "flash";    
  Serial.printf("[LIMITS] clk_sys<=%lu Hz, VCO<=%lu Hz (source=%s)\n",    
                (unsigned long)vfo_max_clk_sys_hz,    
                (unsigned long)vfo_max_pll_vco_hz, src);    
  Serial.flush();    
}    
  
// ============ LOG.TXT ============    
  
// -------------------------------------------------------------------------    
// Функция очистки (стирания) файла LOG.TXT    
// -------------------------------------------------------------------------    
void log_file_clear() {    
  uint32_t root_offset      = SECTOR_SIZE * 2;    
  uint32_t log_entry_offset = root_offset + 64;    
  // Данные LOG.TXT: сектор 5 + INFO_CLUSTERS(8) + SET_CLUSTERS(8) = 21    
  uint32_t log_data_offset  = SECTOR_SIZE * (5 + INFO_CLUSTERS + SET_CLUSTERS);    
  
  memset(&ram_disk_buffer[log_data_offset], 0, LOG_MAX_BYTES);    
  
  String header = "           === ЖУРНАЛ РАБОТЫ МОДЕМА ===\r\n"    
                  "        Версия ПО v." + String(BCN_VER) +    
                  " от " + String(BCN_DAT) + "\r\n";    
  uint32_t header_len = header.length();    
  memcpy(&ram_disk_buffer[log_data_offset], header.c_str(), header_len);    
  
  ram_disk_buffer[log_entry_offset + 28] = (uint8_t)(header_len & 0xFF);    
  ram_disk_buffer[log_entry_offset + 29] = (uint8_t)((header_len >> 8) & 0xFF);    
  ram_disk_buffer[log_entry_offset + 30] = 0x00;    
  ram_disk_buffer[log_entry_offset + 31] = 0x00;    
  
  set_dir_timestamp(2, false);    
  
  save_ram_to_flash();    
  Serial.println("[Журнал ]  Файл LOG.TXT успешно очищен.");    
  remount_usb_disk();    
}    
  
// -------------------------------------------------------------------------    
// Дозапись одной строки с меткой времени в конец LOG.TXT.    
// Одна строка = один цикл erase/program Flash + remount USB.    
// Для пакетной записи используйте log_file_write_block().    
// -------------------------------------------------------------------------    
void log_file_write_line(String message) {    
  uint32_t root_offset      = SECTOR_SIZE * 2;    
  uint32_t log_entry_offset = root_offset + 64;    
  uint32_t log_data_offset  = SECTOR_SIZE * (5 + INFO_CLUSTERS + SET_CLUSTERS);    
  
  uint32_t current_size = ram_disk_buffer[log_entry_offset + 28] |    
                          (ram_disk_buffer[log_entry_offset + 29] << 8) |    
                          (ram_disk_buffer[log_entry_offset + 30] << 16) |    
                          ((uint32_t)ram_disk_buffer[log_entry_offset + 31] << 24);    
  
  String formatted_msg = get_current_date() + " " + get_current_time() +    
                         " " + message + "\r\n";    
  uint32_t msg_len = formatted_msg.length();    
  
  if (current_size + msg_len >= (LOG_MAX_BYTES - 1)) {    
    Serial.println("[Журнал ]  Предупреждение: лог заполнен! Автоочистка...");    
    log_file_clear();    
    current_size = ram_disk_buffer[log_entry_offset + 28] |    
                   (ram_disk_buffer[log_entry_offset + 29] << 8);    
  }    
  
  uint8_t* write_pointer = &ram_disk_buffer[log_data_offset + current_size];    
  memcpy(write_pointer, formatted_msg.c_str(), msg_len);    
  
  uint32_t new_size = current_size + msg_len;    
  ram_disk_buffer[log_entry_offset + 28] = (uint8_t)(new_size & 0xFF);    
  ram_disk_buffer[log_entry_offset + 29] = (uint8_t)((new_size >> 8) & 0xFF);    
  ram_disk_buffer[log_entry_offset + 30] = (uint8_t)((new_size >> 16) & 0xFF);    
  ram_disk_buffer[log_entry_offset + 31] = (uint8_t)((new_size >> 24) & 0xFF);    
  
  set_dir_timestamp(2, false);    
  save_ram_to_flash();    
  Serial.print("[Журнал ]  Строка добавлена. Объем лога: ");    
  Serial.print(new_size); Serial.println(" байт.");    
  
  remount_usb_disk();    
}    
  
// -------------------------------------------------------------------------    
// Дозапись многострочного блока в LOG.TXT — одна транзакция на весь блок:    
// сохранение во Flash и remount выполняются один раз в конце.    
// -------------------------------------------------------------------------    
void log_file_write_block(const String& block) {    
  uint32_t root_offset      = SECTOR_SIZE * 2;    
  uint32_t log_entry_offset = root_offset + 64;    
  uint32_t log_data_offset  = SECTOR_SIZE * (5 + INFO_CLUSTERS + SET_CLUSTERS);    
  
  uint32_t current_size = ram_disk_buffer[log_entry_offset + 28] |    
                          (ram_disk_buffer[log_entry_offset + 29] << 8) |    
                          (ram_disk_buffer[log_entry_offset + 30] << 16) |    
                          ((uint32_t)ram_disk_buffer[log_entry_offset + 31] << 24);    
  
  uint32_t block_len = block.length();    
  
  if (current_size + block_len >= (LOG_MAX_BYTES - 1)) {    
    Serial.println("[Журнал ]  Предупреждение: лог заполнен! Автоочистка...");    
    log_file_clear();    
    current_size = ram_disk_buffer[log_entry_offset + 28] |    
                   (ram_disk_buffer[log_entry_offset + 29] << 8);    
  }    
  
  uint8_t* write_pointer = &ram_disk_buffer[log_data_offset + current_size];    
  memcpy(write_pointer, block.c_str(), block_len);    
  
  uint32_t new_size = current_size + block_len;    
  ram_disk_buffer[log_entry_offset + 28] = (uint8_t)(new_size & 0xFF);    
  ram_disk_buffer[log_entry_offset + 29] = (uint8_t)((new_size >> 8) & 0xFF);    
  ram_disk_buffer[log_entry_offset + 30] = (uint8_t)((new_size >> 16) & 0xFF);    
  ram_disk_buffer[log_entry_offset + 31] = (uint8_t)((new_size >> 24) & 0xFF);    
  
  set_dir_timestamp(2, false);    
  save_ram_to_flash();    
  remount_usb_disk();    
}    
  
// -------------------------------------------------------------------------    
// Сборка и запись актуального SET.TXT из текущих переменных железа.    
// Зеркало генератора внутри create_default_fat_with_info_file() —    
// изменения шаблона вносить в два места. Данные SET.TXT на секторе 13.    
// -------------------------------------------------------------------------    
void save_hardware_settings_to_file(String scan_results) {    
  uint32_t root_offset      = SECTOR_SIZE * 2;    
  uint32_t set_entry_offset = root_offset + 32;    
  uint32_t set_data_offset  = SECTOR_SIZE * (5 + INFO_CLUSTERS);   // сектор 13    
  
  auto pin_to_str = [](int p) -> String {    
    return (p == -1) ? "NC" : String(p);    
  };    
  
  scan_result_data = scan_results;    
  
  String content = "";    
  content.reserve(1024);    
  content += "=== ИНЖЕНЕРНЫЕ НАСТРОЙКИ МОДЕМА ===\r\n";    
  content += "[PIN_FREQ_OUT ]=" + pin_to_str(pin_freq_out) + "\r\n";    
  content += "[PIN_AMP_ACT  ]=" + pin_to_str(pin_amp_act) + "\r\n\r\n";    
  content += "// Выходной канал частоты передачи Si5351: 0 = CLK0, 1 = CLK1, 2 = CLK2\r\n";    
  content += "[SI5351_CLK_OUT]=" + String(si5351_clk_tx_out) + "\r\n\r\n";    
  content += "// Пины кода поддиапазона (4 пина)\r\n";    
  content += "[PIN_SUBBAND_0]=" + pin_to_str(subband_pins[0]) + "\r\n";    
  content += "[PIN_SUBBAND_1]=" + pin_to_str(subband_pins[1]) + "\r\n";    
  content += "[PIN_SUBBAND_2]=" + pin_to_str(subband_pins[2]) + "\r\n";    
  content += "[PIN_SUBBAND_3]=" + pin_to_str(subband_pins[3]) + "\r\n\r\n";    
  content += "// Пины питания шины (NC если не назначены)\r\n";    
  content += "[PIN_PWR_SI   ]=" + pin_to_str(pin_pwr_si) + "\r\n";    
  content += "[PIN_PWR_DS   ]=" + pin_to_str(pin_pwr_ds) + "\r\n";    
  content += "[PIN_PWR_BM   ]=" + pin_to_str(pin_pwr_bm) + "\r\n";    
  content += "[PIN_PWR_DL   ]=" + pin_to_str(pin_pwr_dl) + "\r\n\r\n";    
  content += "// Исключения из сканирования шин\r\n";    
  content += "[SCAN_EXCLUDE ]=" + scan_exclude_list + "\r\n\r\n";    
  content += "// Режим отладки: 1 — служебные сообщения в Serial, 0 — выкл\r\n";    
  content += "[DEBUG]=" + String(debug_flag ? 1 : 0) + "\r\n\r\n";    
  content += "// Управление светодиодной индикацией: 1 — включена, 0 — выключена\r\n";    
  content += "[LED_ENABLE]=" + String(led_enable_flag ? 1 : 0) + "\r\n\r\n";    
  content += "=== ПЕРИОДИЧНОСТЬ КЛИМАТИЧЕСКОГО МОНИТОРИНГА ===\r\n";    
  content += "[CLIM_SAMPLE_MIN]=" + String(climate_sample_interval_min) + "\r\n";    
  content += "[CLIM_REPORT_MIN]=" + String(climate_report_interval_min) + "\r\n\r\n";    
  content += "[XOSC_PPB      ]=" + String((long)vfo_xosc_corr_ppb) + "\r\n\r\n";    
  content += "=== СТАТИСТИКА ИЗНОСА ФЛЭШ-ПАМЯТИ ===\r\n";    
  content += "[FLASH_SLOT   ]=" + String(current_active_slot != -1 ? current_active_slot : 0) + "\r\n";    
  content += "[FLASH_SEQ    ]=" + String(current_max_seq != 0 ? current_max_seq : 1) + "\r\n\r\n";    
  content += "=== УСТРОЙСТВА НА ШИНЕ I2C ===\r\n";    
  content += "[SCAN_RESULT]\r\n" + scan_results + "\r\n\r\n";    
  content += "[EOF]";    
  
  uint32_t total_len = content.length();    
  if (total_len >= SET_MAX_BYTES) {    
    content = content.substring(0, SET_MAX_BYTES - 10) + "\r\n[EOF]";    
    total_len = content.length();    
  }    
  
  memset(&ram_disk_buffer[set_data_offset], 0, SET_MAX_BYTES);    
  memcpy(&ram_disk_buffer[set_data_offset], content.c_str(), total_len);    
  
  ram_disk_buffer[set_entry_offset + 28] = (uint8_t)(total_len & 0xFF);    
  ram_disk_buffer[set_entry_offset + 29] = (uint8_t)((total_len >> 8) & 0xFF);    
  ram_disk_buffer[set_entry_offset + 30] = 0x00;    
  ram_disk_buffer[set_entry_offset + 31] = 0x00;    
  
  set_dir_timestamp(1, false);    
  save_ram_to_flash();    
  remount_usb_disk();    
}    
  
// -------------------------------------------------------------------------    
// Проверка, входит ли пин в список исключений SCAN_EXCLUDE.    
// Авто-исключение всех назначенных пинов + ручной список.    
// -------------------------------------------------------------------------    
bool is_pin_excluded_from_scan(int pin) {    
  if (pin == pin_freq_out) return true;    
  if (pin == pin_amp_act)  return true;    
  
  for (int k = 0; k < 4; k++) {    
    if (pin == subband_pins[k]) return true;    
  }    
  
  if (pin_pwr_si != -1 && pin == pin_pwr_si) return true;    
  if (pin_pwr_ds != -1 && pin == pin_pwr_ds) return true;    
  if (pin_pwr_bm != -1 && pin == pin_pwr_bm) return true;    
  if (pin_pwr_dl != -1 && pin == pin_pwr_dl) return true;    
  
  if (scan_exclude_list.length() == 0 ||    
      scan_exclude_list.equalsIgnoreCase("NC")) return false;    
  
  String target = String(pin);    
  int start = 0;    
  
  while (true) {    
    int comma_idx = scan_exclude_list.indexOf(',', start);    
    String token = (comma_idx == -1)    
                 ? scan_exclude_list.substring(start)    
                 : scan_exclude_list.substring(start, comma_idx);    
    token.trim();    
  
    if (token.equals(target)) return true;    
    if (comma_idx == -1) break;    
    start = comma_idx + 1;    
  }    
  return false;    
}    
  
// -------------------------------------------------------------------------    
// Универсальное редактирование любого параметра в INFO.TXT из консоли.    
// Данные INFO.TXT на секторе 5. Хвост ограничен INFO_MAX_BYTES —    
// сектор SET.TXT (13) под полной защитой.    
// -------------------------------------------------------------------------    
void update_info_config_from_console(String marker, String new_value) {    
  uint32_t root_offset = SECTOR_SIZE * 2;    
  uint32_t data_offset = SECTOR_SIZE * 5;                      // сектор 5    
  
  if (new_value.length() > 32) {    
    new_value = new_value.substring(0, 32);    
  }    
  
  String search_tag = "[" + marker;    
  search_tag.trim();    
  
  uint8_t* data_ptr = &ram_disk_buffer[data_offset];    
  
  uint8_t* marker_ptr = (uint8_t*)strstr((const char*)data_ptr,    
                                         search_tag.c_str());    
  uint8_t* target_tag = nullptr;    
  
  if (marker_ptr != nullptr) {    
    for (int k = 0; k < 20; k++) {    
      if (marker_ptr[k] == ']' && marker_ptr[k + 1] == '=') {    
        target_tag = marker_ptr;    
        break;    
      }    
      if (marker_ptr[k] == '\n' || marker_ptr[k] == '\r' ||    
          marker_ptr[k] == '\0') {    
        break;    
      }    
    }    
  }    
  
  if (target_tag != nullptr) {    
    uint8_t* write_ptr = (uint8_t*)strchr((const char*)target_tag, '=') + 1;    
    uint8_t* end_of_old_line = (uint8_t*)strpbrk((const char*)write_ptr, "\r\n");    
  
    if (end_of_old_line != nullptr) {    
      uint32_t old_val_len = end_of_old_line - write_ptr;    
      uint32_t new_val_len = new_value.length();    
  
      uint32_t current_write_pos = end_of_old_line - ram_disk_buffer;    
  
      // Хвост ограничивается концом области INFO.TXT (4 КБ),    
      // а не концом всего диска — SET.TXT под защитой.    
      uint32_t tail_len = (data_offset + INFO_MAX_BYTES) - current_write_pos;    
  
      if (new_val_len != old_val_len) {    
        uint8_t* new_tail_pos = write_ptr + new_val_len;    
  
        if ((new_tail_pos - ram_disk_buffer) + tail_len <=    
            (data_offset + INFO_MAX_BYTES)) {    
          memmove(new_tail_pos, end_of_old_line, tail_len);    
        } else {    
          Serial.println(F("[Ошибка] Переполнение кластера INFO.TXT! Отмена."));    
          return;    
        }    
      }    
  
      memcpy(write_ptr, new_value.c_str(), new_val_len);    
  
      uint32_t total_file_size = strlen((const char*)data_ptr);    
      ram_disk_buffer[root_offset + 28] = (uint8_t)(total_file_size & 0xFF);    
      ram_disk_buffer[root_offset + 29] = (uint8_t)((total_file_size >> 8) & 0xFF);    
  
      set_dir_timestamp(0, false);    
      save_ram_to_flash();    
      read_and_parse_INFO_txt();    
      remount_usb_disk();    
  
      Serial.print(F("[Система] Записано: [")); Serial.print(marker);    
      Serial.print(F("] = [")); Serial.print(new_value); Serial.println(F("]"));    
    }    
  } else {    
    Serial.print(F("[Ошибка] Маркер [")); Serial.print(marker);    
    Serial.println(F("] не найден."));    
  }    
}    
  
// -------------------------------------------------------------------------    
// Принудительное форматирование и перезапись всех файлов на дефолтные.    
// -------------------------------------------------------------------------    
void force_reset_to_default_disk() {    
  Serial.println(F("[Система] Запущено полное восстановление диска..."));    
  
  // Пины по умолчанию (как в маяке)    
  subband_pins[0] = 6;    
  subband_pins[1] = 7;    
  subband_pins[2] = 8;    
  subband_pins[3] = 9;    
  pin_freq_out = 10;    
  pin_amp_act  = 11;    
  pin_pwr_si   = -1;    
  pin_pwr_ds   = -1;    
  pin_pwr_bm   = -1;    
  pin_pwr_dl   = -1;    
  
  // Лимиты возвращаются на паспортные    
  vfo_max_clk_sys_hz = PASSPORT_CLK_SYS_MAX_HZ;    
  vfo_max_pll_vco_hz = PASSPORT_PLL_VCO_MAX_HZ;    
  
  create_default_fat_with_info_file();   // сама вызывает save_ram_to_flash()    
  read_and_parse_INFO_txt();    
  read_and_parse_SET_txt();    
  remount_usb_disk();    
  
  Serial.println(F("[Система] Все файлы перезаписаны на дефолтные!"));    
}    

// -------------------------------------------------------------------------  
// Обновление временных меток каталога после правок файлов с ПК  
// -------------------------------------------------------------------------  
void ensure_file_timestamps() {  
  set_dir_timestamp(0, false);   // INFO.TXT  
  set_dir_timestamp(1, false);   // SET.TXT  
  set_dir_timestamp(2, false);   // LOG.TXT  
}

// -------------------------------------------------------------------------    
// Отложенная обработка изменений файлов, записанных с ПК (вызов из loop()).    
// ~1.5 с тишины после последней записи, перепарсинг затронутых файлов,    
// один save_ram_to_flash() на транзакцию.    
// -------------------------------------------------------------------------    
void check_and_handle_pc_changes() {    
  if (pc_activity_detected && (millis() - last_msc_write_time > 1500)) {    
    bool config_changed = (pc_written_regions & 0x02) ||    
                          (pc_written_regions & 0x04);    
    if (pc_written_regions & 0x02) {    
      Serial.println(F("[Система] Изменён файл: INFO.TXT"));    
      read_and_parse_INFO_txt();    
      ensure_file_timestamps();    
    }    
    if (pc_written_regions & 0x04) {    
      Serial.println(F("[Система] Изменён файл: SET.TXT"));    
      read_and_parse_SET_txt();    
      ensure_file_timestamps();    
    }    
    if (config_changed && trx_state() == TRX_RX) {    
      save_ram_to_flash();   // один раз на транзакцию, не в TX    
    }    
    pc_activity_detected = false;    
    pc_written_regions   = 0;    
    pc_file_written      = false;    
  }    
}