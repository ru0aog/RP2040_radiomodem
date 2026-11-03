// src/filesys/file_manager.h — USB-MSC хранилище конфигурации радиомодема  
// FAT12, 566 секторов: RAM-диск 54 сектора (27 КБ) + LOG.TXT из flash (XIP).  
#ifndef FILE_MANAGER_H  
#define FILE_MANAGER_H  
  
#include <Arduino.h>  
  
// ---------------------------------------------------------------------------  
// Геометрия диска: 566 секторов x 512 Б (~283 КБ), кластер = 1 сектор  
//   0        boot/BPB  
//   1-2      FAT (2 сектора)  
//   3-5      корневой каталог (4 записи)  
//   6-21     INFO.TXT    кластеры 2-17  
//   22-37    CONFIG.TXT  кластеры 18-33  
//   38-53    SET.TXT     кластеры 34-49  
//   54-565   LOG.TXT     кластеры 50-561 (виртуальные, читаются из flash)  
// ---------------------------------------------------------------------------  
#define SECTOR_SIZE        512  
#define SECTOR_COUNT       566  
#define FAT_SECTORS        2  
#define DIR_SECTORS        3  
#define DATA_START         6  
#define RAM_SECTORS        54  
#define DISK_SIZE_BYTES    (RAM_SECTORS * SECTOR_SIZE)   // 27 КБ  
  
#define INFO_FIRST_CLUSTER   2  
#define INFO_CLUSTERS        16  
#define INFO_MAX_BYTES       (INFO_CLUSTERS * 512)  
#define INFO_LBA             (DATA_START)                          // 6  
  
#define CONFIG_FIRST_CLUSTER (INFO_FIRST_CLUSTER + INFO_CLUSTERS)  // 18  
#define CONFIG_CLUSTERS      16  
#define CONFIG_MAX_BYTES     (CONFIG_CLUSTERS * 512)  
#define CONFIG_LBA           (DATA_START + INFO_CLUSTERS)          // 22  
  
#define SET_FIRST_CLUSTER    (CONFIG_FIRST_CLUSTER + CONFIG_CLUSTERS) // 34  
#define SET_CLUSTERS         16  
#define SET_MAX_BYTES        (SET_CLUSTERS * 512)  
#define SET_LBA              (DATA_START + INFO_CLUSTERS + CONFIG_CLUSTERS) // 38  
  
#define LOG_FIRST_CLUSTER    (SET_FIRST_CLUSTER + SET_CLUSTERS)    // 50  
#define LOG_CLUSTERS         512                                   // 50..561  
#define LOG_LBA              (DATA_START + INFO_CLUSTERS + CONFIG_CLUSTERS + SET_CLUSTERS) // 54  
  
// ---------------------------------------------------------------------------  
// Flash-карта (2 МБ):  
//   0x140000-0x180000  LOG-область, 256 КБ (кольцо, erase 4 КБ)  
//   0x180000-0x200000  кольцо состояний: 8 слотов x 32 КБ  
// Слот: INFO 8КБ @0, CONFIG 8КБ @0x2000, SET 8КБ @0x4000,  
//       RESERVED 7.5КБ @0x6000, SlotMeta 512Б @0x7E00  
// ---------------------------------------------------------------------------  
#define LOG_FLASH_OFFSET   0x140000u  
#define LOG_MAX_BYTES      (256u * 1024u)  
#define RING_OFFSET        0x180000u  
#define SLOT_SIZE          (32u * 1024u)  
#define SLOTS              8  
#define SLOT_MAGIC         0xBEA1C011  
  
#define SLOT_INFO_OFF      0x0000u  
#define SLOT_CONFIG_OFF    0x2000u  
#define SLOT_SET_OFF       0x4000u  
#define SLOT_RESERVED_OFF  0x6000u  
#define SLOT_RESERVED_SIZE (SLOT_SIZE - SLOT_RESERVED_OFF - 512)   // 7.5 КБ  
#define SLOT_META_OFF      0x7E00u  
  
struct __attribute__((packed)) SlotMeta {  
  uint32_t magic;          // SLOT_MAGIC  
  uint32_t seq;            // монотонный счётчик записей  
  uint32_t crc32;          // CRC32 слота (meta->crc32 обнуляется при подсчёте)  
  uint32_t log_write_ptr;  // смещение записи в LOG-области, байт от LOG_FLASH_OFFSET  
  uint8_t  reserved[496];  
};  
  
// Глобальные настройки (парсятся из INFO.TXT)  
extern String my_call_variable;  
extern String my_qth_variable;  
extern String my_text_variable;  
extern String my_freq_rx_var;  
extern String my_demod_var;  
extern bool   debug_flag;  
extern volatile bool pc_file_written;  
  
// Лимиты разгона / коррекция XOSC — определены в src/vfo/vfo.cpp,  
// парсятся из SET.TXT. Применяются при следующем vfo_init() (ребут).  
extern uint32_t vfo_max_clk_sys_hz;  
extern uint32_t vfo_max_pll_vco_hz;  
extern int32_t  vfo_xosc_corr_ppb;  
  
void init_usb_msc_interface(void);  
void init_file_manager();  
void check_and_handle_pc_changes();  
void force_reset_to_default_disk();  
void print_current_settings();  
void read_and_parse_INFO_txt();  
void read_and_parse_CONFIG_txt();  
void read_and_parse_SET_txt();  
void remount_usb_disk();  
void log_file_clear();  
void log_file_write_line(String message);  
void log_append(const uint8_t* data, uint32_t len);  
void save_state();  
bool load_state();  
  
extern int32_t  current_active_slot;  
extern uint32_t current_max_seq;  
  
#endif