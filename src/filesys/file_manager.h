// src/filesys/file_manager.h
// USB-MSC хранилище конфигурации радиомодема
// RAM-диск FAT12 + Flash-журнал + кольцевое хранилище состояний.

#ifndef FILE_MANAGER_H
#define FILE_MANAGER_H

#include <Arduino.h>
#include <stdint.h>

// ============================================================
// Геометрия USB-диска FAT12
// ============================================================

#define SECTOR_SIZE       512u
#define SECTOR_COUNT      566u

// Первые 54 сектора обслуживаются из RAM.
// Остальная область LOG.TXT читается непосредственно из Flash.
#define RAM_SECTORS       54u

#define DISK_SIZE_BYTES   (RAM_SECTORS * SECTOR_SIZE)

// Структура FAT12:
//   LBA 0       — Boot sector
//   LBA 1..2    — FAT
//   LBA 3..5    — Root directory
//   LBA 6..565  — Data area

#define FAT_SECTORS       2u
#define DIR_SECTORS      3u
#define DATA_START       6u

// ============================================================
// INFO.TXT
// ============================================================

#define INFO_FIRST_CLUSTER  2u
#define INFO_CLUSTERS       16u

#define INFO_MAX_BYTES      (INFO_CLUSTERS * SECTOR_SIZE)

#define INFO_LBA            DATA_START

// ============================================================
// CONFIG.TXT
// ============================================================

#define CONFIG_FIRST_CLUSTER \
    (INFO_FIRST_CLUSTER + INFO_CLUSTERS)

#define CONFIG_CLUSTERS     16u

#define CONFIG_MAX_BYTES    (CONFIG_CLUSTERS * SECTOR_SIZE)

#define CONFIG_LBA \
    (DATA_START + INFO_CLUSTERS)

// ============================================================
// SET.TXT
// ============================================================

#define SET_FIRST_CLUSTER \
    (CONFIG_FIRST_CLUSTER + CONFIG_CLUSTERS)

#define SET_CLUSTERS        16u

#define SET_MAX_BYTES       (SET_CLUSTERS * SECTOR_SIZE)

#define SET_LBA \
    (DATA_START + INFO_CLUSTERS + CONFIG_CLUSTERS)

// ============================================================
// LOG.TXT
// ============================================================

#define LOG_FIRST_CLUSTER \
    (SET_FIRST_CLUSTER + SET_CLUSTERS)

#define LOG_CLUSTERS       512u

#define LOG_LBA \
    (DATA_START + INFO_CLUSTERS + CONFIG_CLUSTERS + SET_CLUSTERS)

// Журнал занимает отдельную область Flash.
#define LOG_FLASH_OFFSET   0x140000u
#define LOG_MAX_BYTES      (256u * 1024u)

// ============================================================
// Кольцевое хранилище состояний во Flash
// ============================================================

// Начало кольцевого хранилища.
#define RING_OFFSET        0x180000u

// Размер одного слота — 32 КБ.
#define SLOT_SIZE          (32u * 1024u)

// Количество слотов.
#define SLOTS              8u

#define SLOT_MAGIC         0xBEA1C011u

// Смещения данных внутри слота.
#define SLOT_INFO_OFF      0x0000u
#define SLOT_CONFIG_OFF    0x2000u
#define SLOT_SET_OFF       0x4000u
#define SLOT_RESERVED_OFF  0x6000u

#define SLOT_RESERVED_SIZE \
    (SLOT_SIZE - SLOT_RESERVED_OFF - SECTOR_SIZE)

// Метаданные располагаются в последнем секторе слота.
#define SLOT_META_OFF      0x7E00u

// ============================================================
// Метаданные слота
// ============================================================

struct __attribute__((packed)) SlotMeta {
    uint32_t magic;
    uint32_t seq;
    uint32_t crc32;
    uint32_t log_write_ptr;

    uint8_t reserved[496];
};

// Размер структуры должен быть ровно 512 байт.
static_assert(sizeof(SlotMeta) == SECTOR_SIZE,
              "SlotMeta must be exactly 512 bytes");

// ============================================================
// Глобальные настройки
// ============================================================

extern String my_call_variable;
extern String my_qth_variable;
extern String my_text_variable;

extern String my_freq_rx_var;
extern String my_demod_var;

extern bool debug_flag;

// ПК записал один из файлов USB-диска.
extern volatile bool pc_file_written;

// ============================================================
// Параметры VFO
// ============================================================

extern uint32_t vfo_max_clk_sys_hz;
extern uint32_t vfo_max_pll_vco_hz;
extern int32_t  vfo_xosc_corr_ppb;

// ============================================================
// Состояние Flash-хранилища
// ============================================================

extern int32_t  current_active_slot;
extern uint32_t current_max_seq;

// ============================================================
// API файлового менеджера
// ============================================================

// Подготовка RAM FAT12, восстановление Flash state ring
// и загрузка настроек. Не регистрирует USB.
void file_manager_prepare(void);

// Регистрация USB MSC.
void init_usb_msc_interface(void);

// Совместимая полная инициализация файлового менеджера.
void init_file_manager(void);

void check_and_handle_pc_changes(void);

void force_reset_to_default_disk(void);

void print_current_settings(void);

void read_and_parse_INFO_txt(void);

void remount_usb_disk(void);

// ============================================================
// API журнала
// ============================================================

void log_file_clear(void);

void log_file_write_line(String message);

void file_manager_set_flash_safe_ready(bool ready);

#endif // FILE_MANAGER_H