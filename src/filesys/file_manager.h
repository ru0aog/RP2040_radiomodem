// src/fmgr/file_manager.h — USB-MSC хранилище конфигурации радиомодема  
// Порт file_manager.cpp из RP2040_beacon: RAM-диск FAT12 + flash-слоты.  
#ifndef FILE_MANAGER_H  
#define FILE_MANAGER_H  
  
#include <Arduino.h>  
  
// Геометрия диска: 40 секторов × 512 Б = 20 КБ  
#define SECTOR_SIZE        512  
#define SECTOR_COUNT       40  
#define DISK_SIZE_BYTES    (SECTOR_COUNT * SECTOR_SIZE)  
  
// Разметка: сектор 0 = boot, 1 = FAT, 2..4 = каталог, 5+ = данные  
#define INFO_FIRST_CLUSTER 2  
#define INFO_CLUSTERS      16                     // 8 КБ под INFO.TXT  
#define INFO_MAX_BYTES     (INFO_CLUSTERS * 512)  
#define LOG_FIRST_CLUSTER  (INFO_FIRST_CLUSTER + INFO_CLUSTERS)   // 18  
#define LOG_CLUSTERS       16                     // 8 КБ под LOG.TXT  
#define LOG_MAX_BYTES      (LOG_CLUSTERS * 512)  
  
// Flash: слот по 24 КБ (кратно стиранию 4 КБ), 8 слотов = 192 КБ  
#define FLASH_SLOTS        8  
#define SLOT_SIZE          (48u * 512u)           // 24 КБ: диск + хвост под SlotMeta  
#define SLOT_MAGIC         0xBEA1C011  
// Смещение области ФС во Flash — выбрать свободный сектор < 2 МБ!  
#ifndef FM_FS_OFFSET  
#define FM_FS_OFFSET       (PICO_FLASH_SIZE_BYTES - 192u * 1024u)  
#endif  
#define FLASH_TARGET_OFFSET FM_FS_OFFSET  
  
struct __attribute__((packed)) SlotMeta {  
  uint32_t magic;  
  uint32_t seq;  
};  
  
// Глобальные настройки (парсятся из INFO.TXT)  
extern String my_call_variable;  
extern String my_qth_variable;  
extern String my_text_variable;  
extern String my_freq_rx_var;      // частота приёма, Гц  
extern String my_demod_var;        // "USB"/"LSB"/"RAW"/"AM"/"FM"  
extern bool   debug_flag;  
extern volatile bool pc_file_written;   // ПК записал INFO.TXT  

void init_usb_msc_interface(void);  // регистрация MSC-интерфейса — строго ДО маунта TinyUSB  
void init_file_manager();  
void check_and_handle_pc_changes();  
void force_reset_to_default_disk();  
void print_current_settings();  
void read_and_parse_INFO_txt();  
void remount_usb_disk();  
void log_file_clear();  
void log_file_write_line(String message);  
  
extern int32_t  current_active_slot;  
extern uint32_t current_max_seq;  
  
#endif