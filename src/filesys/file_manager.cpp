// src/filesys/file_manager.cpp — USB-MSC (FAT12, 566 секторов) + кольцо  
// состояний 8x32 КБ + LOG.TXT 256 КБ напрямую из flash (XIP).  
// Flash пишется только при trx_state()==TRX_RX и только из главного цикла  
// (check_and_handle_pc_changes) — в boot flash не стирается.  
  
#include "file_manager.h"  
#include "../trx/trx_control.h"  
#include <Adafruit_TinyUSB.h>  
#include <hardware/flash.h>  
#include <hardware/sync.h>  
  
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
// маски: 0x01=sys(0-5), 0x02=INFO(6-21), 0x04=CONFIG(22-37), 0x08=SET(38-53)  
volatile uint8_t pc_written_regions = 0;  
  
static uint32_t log_write_ptr = 0;            // байт-смещение в LOG-области  
static volatile bool state_save_pending = false; // отложенная первая запись  
static bool usb_started = false;  
  
// ---------------------------------------------------------------------------  
// CRC32 (полином 0xEDB88320, как в zlib), инкрементальный  
// ---------------------------------------------------------------------------  
static uint32_t crc32_update(uint32_t crc, const uint8_t* p, uint32_t len) {  
  for (uint32_t i = 0; i < len; ++i) {  
    crc ^= p[i];  
    for (int b = 0; b < 8; ++b)  
      crc = (crc >> 1) ^ (0xEDB88320u & (-(int32_t)(crc & 1)));  
  }  
  return crc;  
}  
  
// ---------------------------------------------------------------------------  
// TinyUSB MSC callbacks  
// ---------------------------------------------------------------------------  
void msc_flush_cb(void) { last_msc_write_time = millis(); }  
  
int32_t msc_read_cb(uint32_t lba, void* buffer, uint32_t bufsize) {  
  if (lba >= SECTOR_COUNT) return -1;  
  if (lba < RAM_SECTORS) {  
    memcpy(buffer, &ram_disk_buffer[lba * SECTOR_SIZE], bufsize);  
  } else {  
    // LOG.TXT — прямо из flash через XIP  
    memcpy(buffer,  
           (const uint8_t*)(XIP_BASE + LOG_FLASH_OFFSET +  
                            (lba - RAM_SECTORS) * SECTOR_SIZE),  
           bufsize);  
  }  
  return bufsize;  
}  
  
int32_t msc_write_cb(uint32_t lba, uint8_t* buffer, uint32_t bufsize) {  
  if (lba >= SECTOR_COUNT) return -1;  
  if (lba >= RAM_SECTORS) return bufsize;   // LOG read-only: глотаем запись  
  memcpy(&ram_disk_buffer[lba * SECTOR_SIZE], buffer, bufsize);  
  last_msc_write_time = millis();  
  pc_activity_detected = true;  
  if      (lba < INFO_LBA)    pc_written_regions |= 0x01;  
  else if (lba < CONFIG_LBA)  pc_written_regions |= 0x02;  
  else if (lba < SET_LBA)     pc_written_regions |= 0x04;  
  else                        pc_written_regions |= 0x08;  
  if (lba >= INFO_LBA) pc_file_written = true;  
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
  
static void set_dir_timestamp(uint8_t dir_entry_idx) {  
  uint32_t e = SECTOR_SIZE * (1 + FAT_SECTORS) + dir_entry_idx * 32;  // сектор 3  
  uint16_t t = (12 << 11), d = ((2026 - 1980) << 9) | (1 << 5) | 1;  
  ram_disk_buffer[e + 13] = 0;  
  memcpy(&ram_disk_buffer[e + 14], &t, 2);  
  memcpy(&ram_disk_buffer[e + 16], &d, 2);  
  memcpy(&ram_disk_buffer[e + 18], &d, 2);  
  memcpy(&ram_disk_buffer[e + 22], &t, 2);  
  memcpy(&ram_disk_buffer[e + 24], &d, 2);  
}  
  
static void dir_entry(uint8_t idx, const char* name8_3, uint8_t attr,  
                      uint16_t cluster, uint32_t size) {  
  uint32_t e = SECTOR_SIZE * (1 + FAT_SECTORS) + idx * 32;  
  memset(&ram_disk_buffer[e], ' ', 11);  
  memcpy(&ram_disk_buffer[e], name8_3, strlen(name8_3));  
  ram_disk_buffer[e + 11] = attr;  
  memcpy(&ram_disk_buffer[e + 26], &cluster, 2);  
  memcpy(&ram_disk_buffer[e + 28], &size, 4);  
}  
  
// ---------------------------------------------------------------------------  
// Кольцо слотов состояний (8 x 32 КБ @ 0x180000)  
// ---------------------------------------------------------------------------  
  
// Источник байта слота по смещению (без буфера 32 КБ в RAM)  
static const uint8_t* slot_src(uint32_t off, uint32_t* avail) {  
  if (off < SLOT_CONFIG_OFF) {           // INFO 8 КБ  
    *avail = SLOT_CONFIG_OFF - off;  
    return &ram_disk_buffer[INFO_LBA * SECTOR_SIZE] + off;  
  }  
  if (off < SLOT_SET_OFF) {              // CONFIG 8 КБ  
    *avail = SLOT_SET_OFF - off;  
    return &ram_disk_buffer[CONFIG_LBA * SECTOR_SIZE] + (off - SLOT_CONFIG_OFF);  
  }  
  if (off < SLOT_RESERVED_OFF) {         // SET 8 КБ  
    *avail = SLOT_RESERVED_OFF - off;  
    return &ram_disk_buffer[SET_LBA * SECTOR_SIZE] + (off - SLOT_SET_OFF);  
  }  
  if (off < SLOT_META_OFF) {             // RESERVED 7.5 КБ — из текущего слота  
    *avail = SLOT_META_OFF - off;  
    if (current_active_slot >= 0)  
      return (const uint8_t*)(XIP_BASE + RING_OFFSET +  
             current_active_slot * SLOT_SIZE + off);  
    return nullptr;                      // нет активного слота — 0xFF  
  }  
  *avail = SLOT_SIZE - off;              // SlotMeta 512 Б  
  return nullptr;                        // заполняется отдельно  
}  
  
// CRC слота, читаемого напрямую из flash (для load_state)  
static uint32_t slot_crc_flash(const uint8_t* base) {  
  uint32_t crc = 0xFFFFFFFFu;  
  const uint32_t crc_off = SLOT_META_OFF + offsetof(SlotMeta, crc32);  
  uint8_t zeros[4] = {0, 0, 0, 0};  
  crc = crc32_update(crc, base, crc_off);  
  crc = crc32_update(crc, zeros, 4);  
  crc = crc32_update(crc, base + crc_off + 4, SLOT_SIZE - crc_off - 4);  
  return ~crc;  
}  
  
void save_state() {  
  int32_t  next_slot = (current_active_slot + 1) % SLOTS;  
  uint32_t next_seq  = current_max_seq + 1;  
  if (current_active_slot == -1) { next_slot = 0; next_seq = 1; }  
  
  // SlotMeta в маленьком буфере  
  SlotMeta meta;  
  memset(&meta, 0xFF, sizeof(meta));  
  meta.magic         = SLOT_MAGIC;  
  meta.seq           = next_seq;  
  meta.crc32         = 0;  
  meta.log_write_ptr = log_write_ptr;  
  
  // Проход 1: CRC по всему слоту (crc32 считается нулём)  
  uint32_t crc = 0xFFFFFFFFu;  
  {  
    uint32_t off = 0;  
    while (off < SLOT_META_OFF) {  
      uint32_t avail;  
      const uint8_t* src = slot_src(off, &avail);  
      if (src) crc = crc32_update(crc, src, avail);  
      else {  // RESERVED без активного слота — 0xFF  
        for (uint32_t n = 0; n < avail; ++n) {  
          crc ^= 0xFF;  
          for (int b = 0; b < 8; ++b)  
            crc = (crc >> 1) ^ (0xEDB88320u & (-(int32_t)(crc & 1)));  
        }  
      }  
      off += avail;  
    }  
    crc = crc32_update(crc, (const uint8_t*)&meta, offsetof(SlotMeta, crc32));  
    uint8_t z[4] = {0};  
    crc = crc32_update(crc, z, 4);  
    crc = crc32_update(crc, (const uint8_t*)&meta + offsetof(SlotMeta, crc32) + 4,  
                       sizeof(SlotMeta) - offsetof(SlotMeta, crc32) - 4);  
  }  
  meta.crc32 = ~crc;  
  
  // Проход 2: erase + program постранично  
  uint32_t target = RING_OFFSET + next_slot * SLOT_SIZE;  
  uint32_t ints = save_and_disable_interrupts();  
  flash_range_erase(target, SLOT_SIZE);            // 8 блоков по 4 КБ  
  uint32_t off = 0;  
  while (off < SLOT_SIZE) {  
    uint8_t page[FLASH_PAGE_SIZE];  
    uint32_t avail;  
    const uint8_t* src = slot_src(off, &avail);  
    uint32_t chunk = avail < FLASH_PAGE_SIZE ? avail : FLASH_PAGE_SIZE;  
    if (off >= SLOT_META_OFF) {  
      uint32_t mo = off - SLOT_META_OFF;  
      chunk = FLASH_PAGE_SIZE;  
      memset(page, 0xFF, chunk);  
      uint32_t n = sizeof(SlotMeta) - mo;  
      if (n > chunk) n = chunk;  
      memcpy(page, (const uint8_t*)&meta + mo, n);  
    } else if (src) {  
      memcpy(page, src, chunk);  
    } else {  
      memset(page, 0xFF, chunk);  
    }  
    flash_range_program(target + off, page, chunk);  
    off += chunk;  
  }  
  restore_interrupts(ints);  
  flash_flush_cache();  
  
  current_active_slot = next_slot;  
  current_max_seq     = next_seq;  
  Serial.printf("[FM] save_state: slot=%ld seq=%lu\n",  
                (long)next_slot, (unsigned long)next_seq);  
}  
  
bool load_state() {  
  int32_t best = -1; uint32_t best_seq = 0;  
  for (int32_t s = 0; s < SLOTS; ++s) {  
    const uint8_t* base =  
        (const uint8_t*)(XIP_BASE + RING_OFFSET + s * SLOT_SIZE);  
    const SlotMeta* m = (const SlotMeta*)(base + SLOT_META_OFF);  
    if (m->magic != SLOT_MAGIC) continue;  
    if (m->seq <= best_seq) continue;  
    if (slot_crc_flash(base) != m->crc32) continue;   // битый слот  
    best = s; best_seq = m->seq;  
  }  
  if (best < 0) return false;  
  
  const uint8_t* base =  
      (const uint8_t*)(XIP_BASE + RING_OFFSET + best * SLOT_SIZE);  
  memcpy(&ram_disk_buffer[INFO_LBA   * SECTOR_SIZE], base + SLOT_INFO_OFF,   SLOT_CONFIG_OFF - SLOT_INFO_OFF);  
  memcpy(&ram_disk_buffer[CONFIG_LBA * SECTOR_SIZE], base + SLOT_CONFIG_OFF, SLOT_SET_OFF - SLOT_CONFIG_OFF);  
  memcpy(&ram_disk_buffer[SET_LBA    * SECTOR_SIZE], base + SLOT_SET_OFF,    SLOT_RESERVED_OFF - SLOT_SET_OFF);  
  
  const SlotMeta* m = (const SlotMeta*)(base + SLOT_META_OFF);  
  log_write_ptr = m->log_write_ptr;  
  if (log_write_ptr >= LOG_MAX_BYTES) log_write_ptr = 0;  
  
  current_active_slot = best;  
  current_max_seq     = best_seq;  
  return true;  
}  
  
// ---------------------------------------------------------------------------  
// Дефолтный диск: BPB + FAT + каталог + содержимое файлов  
// ---------------------------------------------------------------------------  
static void create_default_fat_with_info_file() {  
  memset(ram_disk_buffer, 0, DISK_SIZE_BYTES);  
  
  // --- Boot sector / BPB (FAT12, 566 секторов, кластер = 1 сектор) ---  
  uint8_t* b = ram_disk_buffer;  
  b[0] = 0xEB; b[1] = 0x3C; b[2] = 0x90;  
  memcpy(&b[3], "MSDOS5.0", 8);  
  b[11] = 0x00; b[12] = 0x02;              // bytes/sector = 512  
  b[13] = 1;                               // sectors/cluster  
  b[14] = 1;  b[15] = 0;                   // reserved = 1  
  b[16] = 1;                               // FAT count  
  b[17] = 48; b[18] = 0;                   // root entries = 48 (3 сектора)  
  uint16_t tot = SECTOR_COUNT;  
  memcpy(&b[19], &tot, 2);                 // total sectors (16-bit) = 566  
  b[21] = 0xF8;                            // media  
  b[22] = FAT_SECTORS; b[23] = 0;          // sectors/FAT = 2  
  b[24] = 1;  b[25] = 0;                   // sectors/track  
  b[26] = 1;  b[27] = 0;                   // heads  
  b[36] = 0x80;                            // drive number  
  b[38] = 0x29;                            // boot signature  
  uint32_t vol = 0x20400603;  
  memcpy(&b[39], &vol, 4);  
  memcpy(&b[43], "RADIOMODEM ", 11);  
  memcpy(&b[54], "FAT12   ", 8);  
  b[510] = 0x55; b[511] = 0xAA;  
  
  // --- FAT (сектора 1-2) ---  
  uint32_t fat = SECTOR_SIZE;              // FAT начинается с сектора 1  
  ram_disk_buffer[fat + 0] = 0xF8;  
  ram_disk_buffer[fat + 1] = 0xFF;  
  ram_disk_buffer[fat + 2] = 0xFF;  
  // Цепочки: INFO 2-17, CONFIG 18-33, SET 34-49, LOG 50-561  
  for (uint16_t c = INFO_FIRST_CLUSTER;   c < INFO_FIRST_CLUSTER   + INFO_CLUSTERS   - 1; ++c) set_fat12_entry(fat, c, c + 1);  
  set_fat12_entry(fat, INFO_FIRST_CLUSTER   + INFO_CLUSTERS   - 1, 0xFFF);  
  for (uint16_t c = CONFIG_FIRST_CLUSTER; c < CONFIG_FIRST_CLUSTER + CONFIG_CLUSTERS - 1; ++c) set_fat12_entry(fat, c, c + 1);  
  set_fat12_entry(fat, CONFIG_FIRST_CLUSTER + CONFIG_CLUSTERS - 1, 0xFFF);  
  for (uint16_t c = SET_FIRST_CLUSTER;    c < SET_FIRST_CLUSTER    + SET_CLUSTERS    - 1; ++c) set_fat12_entry(fat, c, c + 1);  
  set_fat12_entry(fat, SET_FIRST_CLUSTER    + SET_CLUSTERS    - 1, 0xFFF);  
  for (uint16_t c = LOG_FIRST_CLUSTER;    c < LOG_FIRST_CLUSTER    + LOG_CLUSTERS    - 1; ++c) set_fat12_entry(fat, c, c + 1);  
  set_fat12_entry(fat, LOG_FIRST_CLUSTER    + LOG_CLUSTERS    - 1, 0xFFF);  
  
  // --- Корневой каталог (сектора 3-5), 4 записи ---  
  dir_entry(0, "INFO    TXT", 0x20, INFO_FIRST_CLUSTER,   0);  
  dir_entry(1, "CONFIG  TXT", 0x20, CONFIG_FIRST_CLUSTER, 0);  
  dir_entry(2, "SET     TXT", 0x20, SET_FIRST_CLUSTER,    0);  
  dir_entry(3, "LOG     TXT", 0x21, LOG_FIRST_CLUSTER,    LOG_MAX_BYTES); // read-only  
  for (uint8_t i = 0; i < 4; ++i) set_dir_timestamp(i);  
  
  // --- Содержимое файлов ---  
  String info =  
    "[CALL]"    + my_call_variable + "[/CALL]\r\n"  
    "[QTH]"     + my_qth_variable  + "[/QTH]\r\n"  
    "[TEXT]"    + my_text_variable + "[/TEXT]\r\n"  
    "[FREQ_RX]" + my_freq_rx_var   + "[/FREQ_RX]\r\n"  
    "[DEMOD]"   + my_demod_var     + "[/DEMOD]\r\n"  
    "[DEBUG]0[/DEBUG]\r\n";  
  memcpy(&ram_disk_buffer[INFO_LBA * SECTOR_SIZE], info.c_str(),  
         info.length() < INFO_MAX_BYTES ? info.length() : INFO_MAX_BYTES);  
  uint32_t* info_sz = (uint32_t*)&ram_disk_buffer[SECTOR_SIZE * (1 + FAT_SECTORS) + 0 * 32 + 28];  
  *info_sz = info.length();  
  
  String config = "# CONFIG.TXT\r\n";  
  memcpy(&ram_disk_buffer[CONFIG_LBA * SECTOR_SIZE], config.c_str(), config.length());  
  uint32_t* cfg_sz = (uint32_t*)&ram_disk_buffer[SECTOR_SIZE * (1 + FAT_SECTORS) + 1 * 32 + 28];  
  *cfg_sz = config.length();  
  
  String set =  
    "# SET.TXT — скрытые данные\r\n"  
    "[MAX_CLK_SYS]150000000[/MAX_CLK_SYS]\r\n"  
    "[MAX_PLL_VCO]1500000000[/MAX_PLL_VCO]\r\n"  
    "[XOSC_PPB]25000[/XOSC_PPB]\r\n"  
    "# [SCAN] ... [/SCAN]\r\n";  
  memcpy(&ram_disk_buffer[SET_LBA * SECTOR_SIZE], set.c_str(), set.length());  
  uint32_t* set_sz = (uint32_t*)&ram_disk_buffer[SECTOR_SIZE * (1 + FAT_SECTORS) + 2 * 32 + 28];  
  *set_sz = set.length();  
  
  // Первая запись во flash — отложена до RX в главном цикле  
  state_save_pending = true;  
}  
  
// ---------------------------------------------------------------------------  
// Парсер тегов [TAG]...[/TAG] внутри области файла  
// ---------------------------------------------------------------------------  
static String tag_value_at(uint32_t lba, uint32_t area_len, const char* tag) {  
  char open[32], close[36];  
  snprintf(open,  sizeof(open),  "[%s]", tag);  
  snprintf(close, sizeof(close), "[/%s]", tag);  
  const char* base = (const char*)&ram_disk_buffer[lba * SECTOR_SIZE];  
  const char* end  = base + area_len;  
  const char* p = nullptr;  
  for (const char* c = base; c + strlen(open) <= end; ++c)  
    if (!memcmp(c, open, strlen(open))) { p = c + strlen(open); break; }  
  if (!p) return "";  
  const char* e = nullptr;  
  for (const char* c = p; c + strlen(close) <= end; ++c)  
    if (!memcmp(c, close, strlen(close))) { e = c; break; }  
  if (!e) return "";  
  String s; s.reserve(e - p);  
  for (const char* c = p; c < e; ++c)  
    if (*c >= 32 && *c <= 126) s += *c;  
  s.trim();  
  return s;  
}  
  
void read_and_parse_INFO_txt() {  
  String v;  
  if ((v = tag_value_at(INFO_LBA, INFO_MAX_BYTES, "CALL"))   .length()) my_call_variable = v;  
  if ((v = tag_value_at(INFO_LBA, INFO_MAX_BYTES, "QTH"))    .length()) my_qth_variable  = v;  
  if ((v = tag_value_at(INFO_LBA, INFO_MAX_BYTES, "TEXT"))   .length()) my_text_variable = v;  
  if ((v = tag_value_at(INFO_LBA, INFO_MAX_BYTES, "FREQ_RX")).length()) my_freq_rx_var   = v;  
  if ((v = tag_value_at(INFO_LBA, INFO_MAX_BYTES, "DEMOD"))  .length()) my_demod_var     = v;  
  debug_flag = (tag_value_at(INFO_LBA, INFO_MAX_BYTES, "DEBUG") == "1");  
}  
  
void read_and_parse_CONFIG_txt() {  
  // TODO: теги CONFIG.TXT по мере появления  
}  
  
void read_and_parse_SET_txt() {  
  String v;  
  if ((v = tag_value_at(SET_LBA, SET_MAX_BYTES, "MAX_CLK_SYS")).length())  
    vfo_max_clk_sys_hz = (uint32_t)v.toInt();  
  if ((v = tag_value_at(SET_LBA, SET_MAX_BYTES, "MAX_PLL_VCO")).length())  
    vfo_max_pll_vco_hz = (uint32_t)strtoul(v.c_str(), nullptr, 10);  
  if ((v = tag_value_at(SET_LBA, SET_MAX_BYTES, "XOSC_PPB")).length())  
    vfo_xosc_corr_ppb = (int32_t)v.toInt();  
  // TODO: scan-таблица [SCAN]  
}  
  
void print_current_settings() {  
  Serial.printf("[FM] call=%s qth=%s f_rx=%s demod=%s debug=%d\n",  
                my_call_variable.c_str(), my_qth_variable.c_str(),  
                my_freq_rx_var.c_str(), my_demod_var.c_str(), (int)debug_flag);  
}  
  
// ---------------------------------------------------------------------------  
// LOG.TXT — кольцевая запись во flash @0x140000, страницами 256 Б  
// ---------------------------------------------------------------------------  
static void log_boot_recover_ptr() {  
  // Скан первого 0xFF-блока (4 КБ) — указатель на начало свободного блока  
  const uint8_t* base = (const uint8_t*)(XIP_BASE + LOG_FLASH_OFFSET);  
  for (uint32_t off = 0; off < LOG_MAX_BYTES; off += FLASH_SECTOR_SIZE) {  
    if (base[off] == 0xFF) { log_write_ptr = off; return; }  
  }  
  log_write_ptr = 0;   // область полна — начнём кольцо заново  
}  
  
void log_append(const uint8_t* data, uint32_t len) {  
  if (trx_state() != TRX_RX) return;          // flash пишем только на приёме  
  uint32_t off = 0;  
  while (off < len) {  
    if (log_write_ptr >= LOG_MAX_BYTES) log_write_ptr = 0;   // wrap  
    // erase 4 КБ-блока при входе в новый блок  
    if ((log_write_ptr % FLASH_SECTOR_SIZE) == 0) {  
      uint32_t ints = save_and_disable_interrupts();  
      flash_range_erase(LOG_FLASH_OFFSET + log_write_ptr, FLASH_SECTOR_SIZE);  
      restore_interrupts(ints);  
    }  
    uint32_t room = FLASH_PAGE_SIZE - (log_write_ptr % FLASH_PAGE_SIZE);  
    uint32_t n = len - off; if (n > room) n = room;  
    alignas(4) uint8_t page[FLASH_PAGE_SIZE];  
    memset(page, 0xFF, sizeof(page));  
    memcpy(page, data + off, n);  
    uint32_t ints = save_and_disable_interrupts();  
    flash_range_program(LOG_FLASH_OFFSET + log_write_ptr, page, FLASH_PAGE_SIZE);  
    restore_interrupts(ints);  
    log_write_ptr += FLASH_PAGE_SIZE;   // указатель всегда на границе страницы  
    off += n;  
  }  
  flash_flush_cache();  
  remount_usb_disk();  
}  
  
void log_file_write_line(String message) {  
  String line = message + "\r\n";  
  log_append((const uint8_t*)line.c_str(), line.length());  
}  
  
void log_file_clear() {  
  if (trx_state() != TRX_RX) return;  
  uint32_t ints = save_and_disable_interrupts();  
  flash_range_erase(LOG_FLASH_OFFSET, LOG_MAX_BYTES);  
  restore_interrupts(ints);  
  flash_flush_cache();  
  log_write_ptr = 0;  
}  
  
// ---------------------------------------------------------------------------  
// API  
// ---------------------------------------------------------------------------  
void remount_usb_disk() {  
  if (!usb_started) return;             // до usb_msc.begin() — no-op  
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
  usb_started = true;  
}  
  
void init_file_manager() {  
  log_boot_recover_ptr();  
  if (!load_state() ||  
      ram_disk_buffer[510] != 0x55 || ram_disk_buffer[511] != 0xAA) {  
    create_default_fat_with_info_file();   // только RAM; flash — позже, в RX  
  }  
  init_usb_msc_interface();  
  read_and_parse_INFO_txt();  
  read_and_parse_CONFIG_txt();  
  read_and_parse_SET_txt();  
  Serial.printf("[FM] init: slot=%ld seq=%lu log_ptr=%lu\n",  
                (long)current_active_slot, (unsigned long)current_max_seq,  
                (unsigned long)log_write_ptr);  
}  
  
void force_reset_to_default_disk() {  
  create_default_fat_with_info_file();  
  read_and_parse_INFO_txt();  
  read_and_parse_CONFIG_txt();  
  read_and_parse_SET_txt();  
  remount_usb_disk();  
}  
  
void check_and_handle_pc_changes() {  
  // Отложенная первая запись состояния + заголовок сессии в LOG.  
  // Выполняется только в RX, когда система полностью инициализирована.  
  if (state_save_pending) {  
    if (trx_state() != TRX_RX) return;  
    state_save_pending = false;  
    save_state();  
    String hdr = "\r\n=== session start, seq=" + String(current_max_seq) + " ===\r\n";  
    log_append((const uint8_t*)hdr.c_str(), hdr.length());  
    remount_usb_disk();  
  }  
  
  if (!pc_activity_detected) return;  
  if (millis() - last_msc_write_time < 1500) return;   // ждём тишины ПК  
  if (trx_state() != TRX_RX) return;                   // отложим до возврата в RX  
  pc_activity_detected = false;  
  uint8_t regions = pc_written_regions;  
  pc_written_regions = 0;  
  save_state();  
  if (regions & 0x02) read_and_parse_INFO_txt();  
  if (regions & 0x04) read_and_parse_CONFIG_txt();  
  if (regions & 0x08) read_and_parse_SET_txt();  
  remount_usb_disk();  
  pc_file_written = false;  
}