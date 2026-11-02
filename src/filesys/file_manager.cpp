// src/fmgr/file_manager.cpp — USB-MSC (FAT12, 20 КБ) + wear leveling во Flash.  
// Порт RP2040_beacon/file_manager.cpp: та же архитектура, без маяковых  
// зависимостей (расписание, пины, VFO-лимиты, RTC — метки времени статичны).  
  
#include "file_manager.h"  
#include <Adafruit_TinyUSB.h>  
#include <hardware/flash.h>  
#include <hardware/sync.h>  
#include <hardware/structs/ssi.h>  
#include "../trx/trx_control.h"   // trx_state(), TRX_RX — flash пишем только на приёме  
  
Adafruit_USBD_MSC usb_msc;  
  
String my_call_variable = "RU0AOG";  
String my_qth_variable  = "";  
String my_text_variable = "";  
String my_freq_rx_var   = "3602000";  
String my_demod_var     = "USB";  
bool   debug_flag = false;  
volatile bool pc_file_written = false;  
  
int32_t  current_active_slot = -1;  
uint32_t current_max_seq     = 0;  
  
alignas(4) static uint8_t ram_disk_buffer[DISK_SIZE_BYTES];  
  
volatile uint32_t last_msc_write_time = 0;  
bool pc_activity_detected = false;  
volatile uint8_t pc_written_regions = 0;  // 0=sys, 1=INFO, 3=LOG  
  
// ---------------------------------------------------------------------------  
// TinyUSB MSC callbacks  
// ---------------------------------------------------------------------------  
void msc_flush_cb(void) { last_msc_write_time = millis(); }  
  
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
  if      (lba < 5)                            pc_written_regions |= 0x01;  
  else if (lba < 5 + INFO_CLUSTERS)            pc_written_regions |= 0x02;  
  else                                         pc_written_regions |= 0x08;  
  if (lba < 5 + INFO_CLUSTERS) pc_file_written = true;  // изменён конфиг  
  return bufsize;  
}  
  
// ---------------------------------------------------------------------------  
// FAT12 helpers  
// ---------------------------------------------------------------------------  
static void set_fat12_entry(uint32_t fat_start_bytes, uint16_t cluster, uint16_t value) {  
  uint32_t byte_offset = fat_start_bytes + ((cluster * 3) / 2);  
  if (cluster % 2 == 0) {  
    ram_disk_buffer[byte_offset] = (uint8_t)(value & 0xFF);  
    ram_disk_buffer[byte_offset + 1] =  
        (ram_disk_buffer[byte_offset + 1] & 0xF0) | ((value >> 8) & 0x0F);  
  } else {  
    ram_disk_buffer[byte_offset] =  
        (ram_disk_buffer[byte_offset] & 0x0F) | ((value << 4) & 0xF0);  
    ram_disk_buffer[byte_offset + 1] = (uint8_t)((value >> 4) & 0xFF);  
  }  
}  
  
// FAT-время статично (RTC в радиомодеме нет)  
static void set_dir_timestamp(uint8_t dir_entry_idx) {  
  uint32_t e = SECTOR_SIZE * 2 + dir_entry_idx * 32;  
  uint16_t t = (12 << 11), d = ((2026 - 1980) << 9) | (1 << 5) | 1;  
  ram_disk_buffer[e + 13] = 0;  
  memcpy(&ram_disk_buffer[e + 14], &t, 2);  
  memcpy(&ram_disk_buffer[e + 16], &d, 2);  
  memcpy(&ram_disk_buffer[e + 18], &d, 2);  
  memcpy(&ram_disk_buffer[e + 22], &t, 2);  
  memcpy(&ram_disk_buffer[e + 24], &d, 2);  
}  
  
static void dir_entry(uint8_t idx, const char* name8_3, uint16_t cluster, uint32_t size) {  
  uint32_t e = SECTOR_SIZE * 2 + idx * 32;  
  memset(&ram_disk_buffer[e], ' ', 11);  
  memcpy(&ram_disk_buffer[e], name8_3, strlen(name8_3));  
  ram_disk_buffer[e + 11] = 0x20;                     // archive  
  memcpy(&ram_disk_buffer[e + 26], &cluster, 2);  
  memcpy(&ram_disk_buffer[e + 28], &size, 4);  
}  
  
// ---------------------------------------------------------------------------  
// Flash save/restore с ротацией слотов  
// ---------------------------------------------------------------------------  
static void save_ram_to_flash() {  
  int32_t  next_slot = (current_active_slot + 1) % FLASH_SLOTS;  
  uint32_t next_seq  = current_max_seq + 1;  
  if (current_active_slot == -1) { next_slot = 0; next_seq = 1; }  
  
  uint32_t target = FLASH_TARGET_OFFSET + next_slot * SLOT_SIZE;  
  
  SlotMeta* meta = (SlotMeta*)&ram_disk_buffer[DISK_SIZE_BYTES - sizeof(SlotMeta)];  
  meta->magic = SLOT_MAGIC;  
  meta->seq   = next_seq;  
  
  uint32_t ints = save_and_disable_interrupts();  
  flash_range_erase(target, SLOT_SIZE);  
  flash_range_program(target, ram_disk_buffer, DISK_SIZE_BYTES);  
  restore_interrupts(ints);  
  flash_flush_cache();  
  
  current_active_slot = next_slot;  
  current_max_seq     = next_seq;  
  Serial.printf("[FM] flash save: slot=%ld seq=%lu\n",  
                (long)next_slot, (unsigned long)next_seq);  
}  
  
static bool load_from_flash() {  
  int32_t best = -1; uint32_t best_seq = 0;  
  for (int32_t s = 0; s < FLASH_SLOTS; ++s) {  
    const uint8_t* base =  
        (const uint8_t*)(XIP_BASE + FLASH_TARGET_OFFSET + s * SLOT_SIZE);  
    const SlotMeta* m = (const SlotMeta*)(base + DISK_SIZE_BYTES - sizeof(SlotMeta));  
    if (m->magic == SLOT_MAGIC && m->seq > best_seq) { best = s; best_seq = m->seq; }  
  }  
  if (best < 0) return false;  
  memcpy(ram_disk_buffer,  
         (const uint8_t*)(XIP_BASE + FLASH_TARGET_OFFSET + best * SLOT_SIZE),  
         DISK_SIZE_BYTES);  
  current_active_slot = best;  
  current_max_seq     = best_seq;  
  return true;  
}  
  
// ---------------------------------------------------------------------------  
// Образ по умолчанию  
// ---------------------------------------------------------------------------  
static const char INFO_TEMPLATE[] =  
  "[CALL]RU0AOG[/CALL]\r\n"  
  "[QTH]--------[/QTH]\r\n"  
  "[TEXT]RP2040 radiomodem[/TEXT]\r\n"  
  "[FREQ_RX]3602000[/FREQ_RX]\r\n"  
  "[DEMOD]USB[/DEMOD]\r\n"  
  "[DEBUG]0[/DEBUG]\r\n";  
  
static void create_default_disk() {  
  Serial.println("[FM] create default FAT12 disk (20 KB)");  
  memset(ram_disk_buffer, 0, DISK_SIZE_BYTES);  
  
  // Boot sector  
  ram_disk_buffer[0] = 0xEB; ram_disk_buffer[1] = 0x3C; ram_disk_buffer[2] = 0x90;  
  memcpy(&ram_disk_buffer[3], "MSDOS5.0", 8);  
  ram_disk_buffer[11] = 0x00; ram_disk_buffer[12] = 0x02;   // 512 B/sector  
  ram_disk_buffer[13] = 1;                                   // 1 sector/cluster  
  ram_disk_buffer[14] = 1; ram_disk_buffer[15] = 0;          // reserved  
  ram_disk_buffer[16] = 1;                                   // FATs  
  ram_disk_buffer[17] = 48; ram_disk_buffer[18] = 0;         // root entries  
  ram_disk_buffer[19] = (uint8_t)(SECTOR_COUNT & 0xFF);  
  ram_disk_buffer[20] = (uint8_t)(SECTOR_COUNT >> 8);  
  ram_disk_buffer[21] = 0xF8;  
  ram_disk_buffer[22] = 1; ram_disk_buffer[23] = 0;          // sectors/FAT  
  ram_disk_buffer[38] = 0x29;  
  ram_disk_buffer[39] = 0x12; ram_disk_buffer[40] = 0x34;  
  ram_disk_buffer[41] = 0x56; ram_disk_buffer[42] = 0x78;  
  memcpy(&ram_disk_buffer[43], "PICO DRIVE ", 11);  
  memcpy(&ram_disk_buffer[54], "FAT12   ", 8);  
  ram_disk_buffer[510] = 0x55; ram_disk_buffer[511] = 0xAA;  
  
  // FAT: кластеры 0/1 резерв  
  uint32_t fat = SECTOR_SIZE * 1;  
  ram_disk_buffer[fat + 0] = 0xF8; ram_disk_buffer[fat + 1] = 0xFF;  
  ram_disk_buffer[fat + 2] = 0xFF;  
  // Цепочки: INFO (2..17), LOG (18..33)  
  for (uint16_t c = INFO_FIRST_CLUSTER; c < INFO_FIRST_CLUSTER + INFO_CLUSTERS - 1; ++c)  
    set_fat12_entry(fat, c, c + 1);  
  set_fat12_entry(fat, INFO_FIRST_CLUSTER + INFO_CLUSTERS - 1, 0xFFF);  
  for (uint16_t c = LOG_FIRST_CLUSTER; c < LOG_FIRST_CLUSTER + LOG_CLUSTERS - 1; ++c)  
    set_fat12_entry(fat, c, c + 1);  
  set_fat12_entry(fat, LOG_FIRST_CLUSTER + LOG_CLUSTERS - 1, 0xFFF);  
  
  // Каталог  
  dir_entry(0, "INFO    TXT", INFO_FIRST_CLUSTER, sizeof(INFO_TEMPLATE) - 1);  
  dir_entry(1, "LOG     TXT", LOG_FIRST_CLUSTER, 0);  
  set_dir_timestamp(0); set_dir_timestamp(1);  
  
  // Данные  
  memcpy(&ram_disk_buffer[SECTOR_SIZE * (3 + INFO_FIRST_CLUSTER)],  
         INFO_TEMPLATE, sizeof(INFO_TEMPLATE) - 1);  
  
  save_ram_to_flash();  
}  
  
// ---------------------------------------------------------------------------  
// Парсер тегов [TAG]...[/TAG] в INFO.TXT  
// ---------------------------------------------------------------------------  
static String tag_value(const char* tag) {  
  char open[32], close[36];  
  snprintf(open,  sizeof(open),  "[%s]", tag);  
  snprintf(close, sizeof(close), "[/%s]", tag);  
  const char* base = (const char*)&ram_disk_buffer[SECTOR_SIZE * (3 + INFO_FIRST_CLUSTER)];  
  const char* p = strstr(base, open);  
  if (!p) return "";  
  p += strlen(open);  
  const char* e = strstr(p, close);  
  if (!e) return "";  
  String s; s.reserve(e - p);  
  for (const char* c = p; c < e; ++c)  
    if (*c >= 32 && *c <= 126) s += *c;  
  s.trim();  
  return s;  
}  
  
void read_and_parse_INFO_txt() {  
  String v;  
  if ((v = tag_value("CALL"))    .length()) my_call_variable = v;  
  if ((v = tag_value("QTH"))     .length()) my_qth_variable  = v;  
  if ((v = tag_value("TEXT"))    .length()) my_text_variable = v;  
  if ((v = tag_value("FREQ_RX")) .length()) my_freq_rx_var   = v;  
  if ((v = tag_value("DEMOD"))   .length()) my_demod_var     = v;  
  debug_flag = (tag_value("DEBUG") == "1");  
}  
  
void print_current_settings() {  
  Serial.printf("[FM] call=%s qth=%s f_rx=%s demod=%s debug=%d\n",  
                my_call_variable.c_str(), my_qth_variable.c_str(),  
                my_freq_rx_var.c_str(), my_demod_var.c_str(), (int)debug_flag);  
}  
  
// ---------------------------------------------------------------------------  
// LOG.TXT — append внутри области данных + правка размера в каталоге  
// ---------------------------------------------------------------------------  
static uint32_t log_area_off() { return SECTOR_SIZE * (3 + LOG_FIRST_CLUSTER); }  
  
void log_file_write_line(String message) {  
  uint32_t* size_p = (uint32_t*)&ram_disk_buffer[SECTOR_SIZE * 2 + 1 * 32 + 28];  
  uint32_t sz = *size_p;  
  if (sz + message.length() + 2 > LOG_MAX_BYTES) sz = 0;   // wrap  
  uint32_t off = log_area_off() + sz;  
  memcpy(&ram_disk_buffer[off], message.c_str(), message.length());  
  off += message.length();  
  ram_disk_buffer[off++] = '\r'; ram_disk_buffer[off++] = '\n';  
  *size_p = off - log_area_off();  
  last_msc_write_time = millis();  
  pc_activity_detected = true;  
}  
  
void log_file_clear() {  
  memset(&ram_disk_buffer[log_area_off()], 0, LOG_MAX_BYTES);  
  uint32_t z = 0;  
  memcpy(&ram_disk_buffer[SECTOR_SIZE * 2 + 1 * 32 + 28], &z, 4);  
}  
  
// ---------------------------------------------------------------------------  
// API  
// ---------------------------------------------------------------------------  
void remount_usb_disk() {  
  usb_msc.setUnitReady(false);  
  delay(50);  
  usb_msc.setUnitReady(true);  
}  
  
void init_usb_msc_interface() {  
  usb_msc.setID("RP2040", "RADIOMODEM", "1.0");  
  usb_msc.setCapacity(SECTOR_COUNT, SECTOR_SIZE);  
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);  
  usb_msc.setUnitReady(true);  
  usb_msc.begin();  
}  
  
void init_file_manager() {  
  if (!load_from_flash() ||  
      ram_disk_buffer[510] != 0x55 || ram_disk_buffer[511] != 0xAA) {  
    create_default_disk();  
  }  
  init_usb_msc_interface();  
  read_and_parse_INFO_txt();  
  Serial.printf("[FM] init: slot=%ld seq=%lu\n",  
                (long)current_active_slot, (unsigned long)current_max_seq);  
}  
  
void force_reset_to_default_disk() {  
  create_default_disk();  
  read_and_parse_INFO_txt();  
  remount_usb_disk();  
}  
  
void check_and_handle_pc_changes() {  
  if (!pc_activity_detected) return;  
  if (millis() - last_msc_write_time < 1500) return;   // ждём тишины ПК  
  pc_activity_detected = false;  
  pc_written_regions   = 0;  
  if (trx_state() == TRX_RX) save_ram_to_flash();       // не писать flash в TX  
  read_and_parse_INFO_txt();  
  remount_usb_disk();  
  pc_file_written = false;  
}