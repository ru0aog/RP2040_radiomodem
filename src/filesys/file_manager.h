// src/filesys/file_manager.h — USB-MSC (FAT12, 20 КБ) + wear leveling во Flash.  
// Порт RP2040_beacon/file_manager — полная маяковая структура,  
// диск ужат со 128 до 20 КБ (вариант C: INFO 4K + SET 4K + LOG 8K).  
#ifndef FILE_MANAGER_H  
#define FILE_MANAGER_H  
  
#include <Arduino.h>  
#include <stdint.h>  
#include <Adafruit_TinyUSB.h>  
  
// ============ ГЕОМЕТРИЯ ДИСКА (20 КБ, 40 секторов) ============  
#define SECTOR_SIZE       512  
#define SECTOR_COUNT      40                      // 40 × 512 = 20 480 Б  
#define DISK_SIZE_BYTES   (SECTOR_COUNT * SECTOR_SIZE)  
  
// Разметка FAT12: сектор 0 — boot, 1 — FAT, 2..4 — каталог (48 записей),  
// данные с сектора 5 (кластер 2 = сектор 5).  
#define INFO_FIRST_CLUSTER 2                      // сектора 5..12  
#define INFO_CLUSTERS      8                      // 4 КБ  
#define INFO_MAX_BYTES     (INFO_CLUSTERS * SECTOR_SIZE)  
  
#define SET_FIRST_CLUSTER  (INFO_FIRST_CLUSTER + INFO_CLUSTERS)   // 10 → сектора 13..20  
#define SET_CLUSTERS       8                                      // 4 КБ  
#define SET_MAX_BYTES      (SET_CLUSTERS * SECTOR_SIZE)  
  
#define LOG_FIRST_CLUSTER  (SET_FIRST_CLUSTER + SET_CLUSTERS)     // 18 → сектора 21..36  
#define LOG_CLUSTERS       16                                     // 8 КБ  
#define LOG_MAX_BYTES      (LOG_CLUSTERS * SECTOR_SIZE)  
  
// Сектора данных (производные, везде вместо магических 5/25/45)  
#define INFO_DATA_SECTOR   5  
#define SET_DATA_SECTOR    (INFO_DATA_SECTOR + INFO_CLUSTERS)     // 13  
#define LOG_DATA_SECTOR    (SET_DATA_SECTOR + SET_CLUSTERS)       // 21  
  
// ============ WEAR LEVELING: 8 слотов во Flash ============  
#define SLOT_MAGIC      0xA55A0001  
#define FLASH_SLOTS     8  
#define SLOT_SIZE       DISK_SIZE_BYTES    // 20 480 Б, кратно 4096  
// 8 × 20K = 160K. ПРОВЕРЬТЕ: диапазон не должен пересекать прошивку  
// и область EEPROM-эмуляции ядра (последний сектор flash).  
#define FLASH_TARGET_OFFSET 0x001C0000UL    // 1 835 008 → конец 0x1E8000  
  
struct SlotMeta {  
  uint32_t magic;            // SLOT_MAGIC  
  uint32_t seq;              // счётчик версий образа  
  uint32_t max_clk_sys_hz;   // доказанный потолок clk_sys (маяк)  
  uint32_t max_pll_vco_hz;   // доказанный потолок PLL VCO  
  int32_t  xosc_corr_ppb;    // коррекция кварца, ppb  
};  
  
// ============ API ============  
extern Adafruit_USBD_MSC usb_msc;  
  
void init_usb_msc_interface(void);   // СТРОГО до маунта TinyUSB (в setup() до Serial.begin)  
void init_file_manager(void);        // загрузка слота из Flash + парсеры  
void init_flash_disk(void);          // setUnitReady(true) — «диск вставлен»  
void check_and_handle_pc_changes(void); // из loop(): тишина 1.5 с → parse+save  
void read_and_parse_INFO_txt(void);  
void read_and_parse_SET_txt(void);  
void update_info_config_from_console(String marker, String new_value);  
void print_current_settings(void);  
void save_hardware_settings_to_file(String scan_results);  
void persist_clock_limits(uint64_t new_clk_hz, uint64_t new_vco_hz);  
void log_file_write_line(String message);  
void log_file_clear(void);  
void force_reset_to_default_disk(void);  
void remount_usb_disk(void);  
void ensure_file_timestamps(void);  
uint64_t flash_limits_clk_sys_hz(void);  
uint64_t flash_limits_vco_hz(void);  
  
// TinyUSB MSC callbacks  
int32_t msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize);  
int32_t msc_write_cb(uint32_t lba, uint8_t* buffer, uint32_t bufsize);  
void    msc_flush_cb(void);  
  
// ============ ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ (маяковые, подключаются позже) ============  
extern String my_call_variable, my_qth_variable, my_text_variable;  
extern String my_cw_variable, my_rtty_variable, my_ifkp_variable;  
extern String my_freq_cw_var, my_cw_wpm_var, my_rtty_baud_var;  
extern String my_rtty_mark_var, my_rtty_shift_var, my_rtty_invert_var;  
extern String my_freq_ifkp_var, my_freq_olivia_var, my_freq_thor_var;  
extern String my_freq_mfsk16_var, my_freq_dominoex_var, my_freq_fsq_var;  
extern String my_freq_contestia_var, my_freq_jtty_var, my_freq_ax25_var;  
extern String my_ax25_dest_var, my_ax25_txdelay_var, my_ax25_invert_var;  
extern String my_FAT, scan_exclude_list, scan_result_data;  
extern String my_debug_var, s_led_enable;  
  
extern uint32_t CW_DOT_TIME_MS;  
extern volatile uint32_t RTTY_BIT_TIME_US;  
extern uint32_t climate_sample_interval_min, climate_report_interval_min;  
extern int subband_pins[4], pin_freq_out, pin_amp_act;  
extern int pin_pwr_si, pin_pwr_ds, pin_pwr_bm, pin_pwr_dl;  
extern int si5351_clk_tx_out;  
extern bool debug_flag, led_enable_flag;  
extern int32_t  current_active_slot;  
extern uint32_t current_max_seq;  
extern uint32_t vfo_max_clk_sys_hz, vfo_max_pll_vco_hz;  
extern int32_t  vfo_xosc_corr_ppb;  
extern volatile bool pc_file_written;  
extern volatile uint32_t last_msc_write_time;  
extern bool pc_activity_detected;  
extern volatile uint8_t pc_written_regions;  
  
// ============ COMPAT-БЛОК: маяковые зависимости ============  
// Временно определены здесь; при портировании scheduler.h/vfo_hardware.h  
// удалить этот блок и подключить реальные заголовки.  
#include "fmgr_compat.h"  
  
#endif