// src/filesys/file_manager.cpp
// USB-MSC FAT12 + Flash state ring + LOG.TXT
//
// RP2040_radiomodem.
//
// Сохранены:
//   FAT12 geometry и формат RAM-диска;
//   INFO.TXT / CONFIG.TXT / SET.TXT / LOG.TXT;
//   Flash state ring: 8 x 32 KiB;
//   SlotMeta и CRC32;
//   существующий USB MSC API.
//
// Flash writes:    
//   разрешены только после file_manager_set_flash_safe_ready(true);    
//   выполняются только с Core 0 через flash_safe_execute() SDK —    
//   Core 1 паркуется SDK-локаутом (multicore_lockout_victim_init в setup1).    
//   Ручной clk-switch на XOSC удалён — SDK управляет SSI сам.
//  
// ВАЖНО:  
//   file_manager_set_flash_safe_ready(true) вызывать только после  
//   core1_pll_done — Core 1 должен обслуживать core_sync_core1_poll().

#include "file_manager.h"
#include "../trx/trx_control.h"
#include "../trx/core_sync.h"  

#include <Adafruit_TinyUSB.h>

#include <hardware/flash.h>
#include <hardware/sync.h>
#include <hardware/clocks.h>

#include <pico/flash.h>
#include <pico/error.h>

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>


// ---------------------------------------------------------------------------
// USB MSC
// ---------------------------------------------------------------------------

Adafruit_USBD_MSC usb_msc;

// ---------------------------------------------------------------------------
// Global settings
// ---------------------------------------------------------------------------

String my_call_variable = "RU0AOG";
String my_qth_variable  = "";
String my_text_variable = "";

String my_freq_rx_var = "3602000";
String my_demod_var   = "USB";

bool debug_flag = false;

volatile bool pc_file_written = false;
static volatile bool flash_busy = false;

int32_t  current_active_slot = -1;
uint32_t current_max_seq = 0;

// ---------------------------------------------------------------------------
// RAM disk
// ---------------------------------------------------------------------------

alignas(4) static uint8_t ram_disk_buffer[DISK_SIZE_BYTES];

// ---------------------------------------------------------------------------
// MSC activity
// ---------------------------------------------------------------------------

volatile uint32_t last_msc_write_time = 0;
volatile bool pc_activity_detected = false;
volatile uint8_t pc_written_regions = 0;

// Определён в core_sync.cpp — печатается из check_and_handle_pc_changes().  
extern volatile bool core1_park_timed_out;
extern volatile uint32_t dbg_c0_stage;   // в начало file_manager.cpp
  
// ---------------------------------------------------------------------------
// Internal state
// ---------------------------------------------------------------------------

static uint32_t log_write_ptr = 0;

static volatile bool state_save_pending = false;

static bool usb_started = false;
static bool file_manager_prepared = false;

// Flash writes are disabled until explicitly enabled.
static volatile bool flash_safe_ready = false;

// ---------------------------------------------------------------------------
// Flash safety control
// ---------------------------------------------------------------------------

void file_manager_set_flash_safe_ready(bool ready)
{
    // Only Core 0 may change the Flash write permission.
    if (get_core_num() != 0) {
        return;
    }

    __dmb();
    flash_safe_ready = ready;
    __dmb();

    Serial.printf(
        "[FM] Flash writes %s\n",
        ready ? "ENABLED" : "DISABLED"
    );
}

static bool flash_write_allowed(void)
{
    if (!flash_safe_ready) {
        return false;
    }

    if (get_core_num() != 0) {
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------  
// Multicore-safe Flash helpers  
//  
// Все flash-операции идут через flash_safe_execute() SDK:  
// Core 1 паркуется SDK-локаутом (multicore_lockout_victim_init в setup1),  
// критичный код исполняется из RAM. Ручная парковка и clk-switch удалены.  
// ---------------------------------------------------------------------------  

// Аргументы стубов — статические: flash_safe_execute не копирует param,  
// передаёт указатель как есть.  
static size_t erase_size = 0;  
  
typedef struct {  
    uint32_t offset;  
    const uint8_t* data;  
    size_t size;  
} ProgArgs;  
  
static ProgArgs prog_args;  


// Стубы для flash_safe_execute: сигнатура void (*)(void*).  
// Выполняются в контексте локаута — только flash-вызов, ничего больше. 

static void flash_range_erase_stub(void* param) {  
    const uint32_t offset = (uint32_t)(uintptr_t)param;  
    flash_range_erase(offset, erase_size);  
}  
  
static void flash_range_program_stub(void* param) {  
    const ProgArgs* a = (const ProgArgs*)param;  
    flash_range_program(a->offset, a->data, a->size);  
}  
  
static bool do_flash_erase(uint32_t offset, size_t size) {  
    dbg_c0_stage = 51;  
    erase_size = size;  
    flash_busy = true;  
    __dmb();  
    const int rc = flash_safe_execute(  
        flash_range_erase_stub,  
        (void*)(uintptr_t)offset,  
        500);  
    flash_busy = false;  
    __dmb();  
    dbg_c0_stage = 0;  
    return (rc == PICO_OK);  
}  
  
static void do_flash_program(  
        uint32_t offset, const uint8_t* data, size_t size) {  
    dbg_c0_stage = 61;  
    prog_args.offset = offset;  
    prog_args.data   = data;  
    prog_args.size   = size;  
    flash_busy = true;  
    __dmb();  
    flash_safe_execute(flash_range_program_stub, &prog_args, 500);  
    flash_busy = false;  
    __dmb();  
    dbg_c0_stage = 0;  
}


static bool safe_flash_erase(uint32_t offset, size_t size) {  
    if (!flash_write_allowed()) return false;  
    if (size == 0 ||  
        (offset % FLASH_SECTOR_SIZE) != 0 ||  
        (size % FLASH_SECTOR_SIZE) != 0) {  
        Serial.println("[FM] ERROR: invalid Flash erase alignment");  
        return false;  
    }
    return do_flash_erase(offset, size);
}  
  
static bool safe_flash_program(uint32_t offset, const uint8_t* data, size_t size) {  
    if (!flash_write_allowed()) return false;  
    if (data == nullptr || size == 0 ||  
        (offset % FLASH_PAGE_SIZE) != 0 ||  
        (size % FLASH_PAGE_SIZE) != 0) {  
        Serial.println("[FM] ERROR: invalid Flash program alignment");  
        return false;  
    }  
    do_flash_program(offset, data, size);  
    return true;  
}


// ---------------------------------------------------------------------------
// CRC32
// Polynomial 0xEDB88320, compatible with zlib.
// ---------------------------------------------------------------------------

static uint32_t crc32_update(
    uint32_t crc,
    const uint8_t* p,
    uint32_t len)
{
    for (uint32_t i = 0; i < len; ++i) {
        crc ^= p[i];

        for (int b = 0; b < 8; ++b) {
            crc = (crc >> 1) ^
                (0xEDB88320u & (-(int32_t)(crc & 1)));
        }
    }

    return crc;
}

// ---------------------------------------------------------------------------
// TinyUSB MSC callbacks
// ---------------------------------------------------------------------------

void msc_flush_cb(void) {  
    pc_activity_detected = true;  
    // last_msc_write_time НЕ трогаем — иначе периодические flush  
    // от Windows вечно отодвигают 1500-мс окно сохранения.  
}

// src/filesys/file_manager.cpp  
int32_t msc_read_cb(  
    uint32_t lba,  
    void* buffer,  
    uint32_t bufsize)  
{  
    if (buffer == nullptr ||  
        bufsize == 0 ||  
        bufsize > SECTOR_SIZE ||  
        lba >= SECTOR_COUNT) {  
        return -1;  
    }  
  
    if (lba < RAM_SECTORS) {  
        const uint32_t off = lba * SECTOR_SIZE;  
  
        if (off + bufsize > DISK_SIZE_BYTES) {  
            return -1;  
        }  
  
        memcpy(buffer, &ram_disk_buffer[off], bufsize);  
    }  
    else {  
        // Защита от XIP-доступа во время flash-операции:  
        // колбэк вызывается из USB-стека на Core 0; если в этот момент  
        // идёт erase/program (flash_busy), чтение LOG.TXT из XIP уронит ядро.  
        // Отдаём нули — Windows перечитает сектор позже.  
        if (flash_busy) {  
            memset(buffer, 0, bufsize);  
            return (int32_t)bufsize;  
        }  
  
        const uint32_t off =  
            (lba - RAM_SECTORS) * SECTOR_SIZE;  
  
        if (off + bufsize > LOG_MAX_BYTES) {  
            return -1;  
        }  
  
        const uint8_t* flash_data =  
            reinterpret_cast<const uint8_t*>(  
                XIP_BASE + LOG_FLASH_OFFSET + off  
            );  
  
        memcpy(buffer, flash_data, bufsize);  
    }  
  
    return (int32_t)bufsize;  
}

int32_t msc_write_cb(
    uint32_t lba,
    uint8_t* buffer,
    uint32_t bufsize)
{
    if (buffer == nullptr ||
        bufsize == 0 ||
        bufsize > SECTOR_SIZE ||
        lba >= SECTOR_COUNT) {
        return -1;
    }

    // LOG.TXT is read-only through USB MSC.
    if (lba >= RAM_SECTORS) {
        return (int32_t)bufsize;
    }

    const uint32_t off = lba * SECTOR_SIZE;

    if (off + bufsize > DISK_SIZE_BYTES) {
        return -1;
    }

    memcpy(&ram_disk_buffer[off], buffer, bufsize);

    last_msc_write_time = millis();

    pc_activity_detected = true;

    // Region flags:
    // 0x01 — boot sector / FAT / root directory
    // 0x02 — INFO.TXT
    // 0x04 — CONFIG.TXT
    // 0x08 — SET.TXT

    if (lba < INFO_LBA) {
        pc_written_regions |= 0x01;
    }
    else if (lba < CONFIG_LBA) {
        pc_written_regions |= 0x02;
    }
    else if (lba < SET_LBA) {
        pc_written_regions |= 0x04;
    }
    else {
        pc_written_regions |= 0x08;
    }

    if (lba >= INFO_LBA) {
        pc_file_written = true;
    }

    return (int32_t)bufsize;
}

// ---------------------------------------------------------------------------
// FAT12 helpers
// ---------------------------------------------------------------------------

static void set_fat12_entry(
    uint32_t fat_start_bytes,
    uint16_t cluster,
    uint16_t value)
{
    const uint32_t byte_offset =
        fat_start_bytes + ((cluster * 3u) / 2u);

    if ((cluster & 1u) == 0) {
        ram_disk_buffer[byte_offset] =
            (uint8_t)(value & 0xFF);

        ram_disk_buffer[byte_offset + 1] =
            (ram_disk_buffer[byte_offset + 1] & 0xF0) |
            ((value >> 8) & 0x0F);
    }
    else {
        ram_disk_buffer[byte_offset] =
            (ram_disk_buffer[byte_offset] & 0x0F) |
            ((value << 4) & 0xF0);

        ram_disk_buffer[byte_offset + 1] =
            (uint8_t)((value >> 4) & 0xFF);
    }
}

static void set_dir_timestamp(uint8_t dir_entry_idx)
{
    const uint32_t e =
        SECTOR_SIZE * (1 + FAT_SECTORS) +
        dir_entry_idx * 32u;

    const uint16_t t = (12u << 11);

    const uint16_t d =
        ((2026u - 1980u) << 9) |
        (1u << 5) |
        1u;

    ram_disk_buffer[e + 13] = 0;

    memcpy(&ram_disk_buffer[e + 14], &t, 2);
    memcpy(&ram_disk_buffer[e + 16], &d, 2);
    memcpy(&ram_disk_buffer[e + 18], &d, 2);

    memcpy(&ram_disk_buffer[e + 22], &t, 2);
    memcpy(&ram_disk_buffer[e + 24], &d, 2);
}

static void dir_entry(
    uint8_t idx,
    const char* name8_3,
    uint8_t attr,
    uint16_t cluster,
    uint32_t size)
{
    const uint32_t e =
        SECTOR_SIZE * (1 + FAT_SECTORS) +
        idx * 32u;

    memset(&ram_disk_buffer[e], ' ', 11);

    size_t n = strlen(name8_3);

    if (n > 11) {
        n = 11;
    }

    memcpy(&ram_disk_buffer[e], name8_3, n);

    ram_disk_buffer[e + 11] = attr;

    memcpy(&ram_disk_buffer[e + 26], &cluster, 2);
    memcpy(&ram_disk_buffer[e + 28], &size, 4);
}

// ---------------------------------------------------------------------------
// State ring slots
// 8 x 32 KiB at 0x180000
// ---------------------------------------------------------------------------

static const uint8_t* slot_src(
    uint32_t off,
    uint32_t* avail)
{
    if (off < DISK_SIZE_BYTES) {
        *avail = DISK_SIZE_BYTES - off;
        return &ram_disk_buffer[off];
    }

    if (off < SLOT_META_OFF) {
        *avail = SLOT_META_OFF - off;
        return nullptr;
    }

    *avail = SLOT_SIZE - off;

    return nullptr;
}

static uint32_t slot_crc_flash(const uint8_t* base)
{
    uint32_t crc = 0xFFFFFFFFu;

    const uint32_t crc_off =
        SLOT_META_OFF + offsetof(SlotMeta, crc32);

    const uint8_t zeros[4] = {0, 0, 0, 0};

    crc = crc32_update(crc, base, crc_off);

    crc = crc32_update(crc, zeros, sizeof(zeros));

    crc = crc32_update(
        crc,
        base + crc_off + 4,
        SLOT_SIZE - crc_off - 4
    );

    return ~crc;
}

// ---------------------------------------------------------------------------
// Save state ring
// ---------------------------------------------------------------------------

void save_state(void)  
{  
    dbg_c0_stage = 55;  
  
    if (!flash_write_allowed()) {  
        dbg_c0_stage = 0;  
        return;  
    }  
  
    if (trx_state() != TRX_RX) {  
        dbg_c0_stage = 0;  
        return;  
    }  
  
    int32_t next_slot =  
        (current_active_slot + 1) % SLOTS;  
  
    uint32_t next_seq =  
        current_max_seq + 1;  
  
    if (current_active_slot == -1) {  
        next_slot = 0;  
        next_seq = 1;  
    }  
  
    SlotMeta meta;  
  
    memset(&meta, 0xFF, sizeof(meta));  
  
    meta.magic = SLOT_MAGIC;  
    meta.seq = next_seq;  
    meta.crc32 = 0;  
    meta.log_write_ptr = log_write_ptr;  
  
    uint32_t crc = 0xFFFFFFFFu;  
    uint32_t off = 0;  
  
    while (off < SLOT_META_OFF) {  
        uint32_t avail = 0;  
  
        const uint8_t* src =  
            slot_src(off, &avail);  
  
        if (avail == 0 || off + avail > SLOT_META_OFF) {  
            Serial.println("[FM] ERROR: invalid slot source range");  
            dbg_c0_stage = 0;  
            return;  
        }  
  
        if (src != nullptr) {  
            crc = crc32_update(crc, src, avail);  
        }  
        else {  
            for (uint32_t n = 0; n < avail; ++n) {  
                crc ^= 0xFF;  
  
                for (int b = 0; b < 8; ++b) {  
                    crc = (crc >> 1) ^  
                        (0xEDB88320u & (-(int32_t)(crc & 1)));  
                }  
            }  
        }  
  
        off += avail;  
    }  
  
    crc = crc32_update(  
        crc,  
        reinterpret_cast<const uint8_t*>(&meta),  
        offsetof(SlotMeta, crc32)  
    );  
  
    const uint8_t zeros[4] = {0, 0, 0, 0};  
  
    crc = crc32_update(crc, zeros, sizeof(zeros));  
  
    crc = crc32_update(  
        crc,  
        reinterpret_cast<const uint8_t*>(&meta) +  
            offsetof(SlotMeta, crc32) + 4,  
        sizeof(SlotMeta) -  
            offsetof(SlotMeta, crc32) - 4  
    );  
  
    meta.crc32 = ~crc;  
  
    const uint32_t target =  
        RING_OFFSET +  
        (uint32_t)next_slot * SLOT_SIZE;  
  
    if (!safe_flash_erase(target, SLOT_SIZE)) {  
        Serial.println("[FM] ERROR: state-slot erase failed");  
        dbg_c0_stage = 0;  
        return;  
    }  
  
    off = 0;  
  
    while (off < SLOT_SIZE) {  
        alignas(4) uint8_t page[FLASH_PAGE_SIZE];  
  
        uint32_t avail = 0;  
  
        const uint8_t* src =  
            slot_src(off, &avail);  
  
        uint32_t chunk =  
            avail < FLASH_PAGE_SIZE ?  
            avail : FLASH_PAGE_SIZE;  
  
        if (chunk == 0 ||  
            off + chunk > SLOT_SIZE) {  
            Serial.println("[FM] ERROR: invalid state-slot page range");  
            dbg_c0_stage = 0;  
            return;  
        }  
  
        if (off >= SLOT_META_OFF) {  
            const uint32_t mo = off - SLOT_META_OFF;  
  
            chunk = FLASH_PAGE_SIZE;  
  
            memset(page, 0xFF, chunk);  
  
            if (mo < sizeof(SlotMeta)) {  
                uint32_t n = sizeof(SlotMeta) - mo;  
  
                if (n > chunk) {  
                    n = chunk;  
                }  
  
                memcpy(  
                    page,  
                    reinterpret_cast<const uint8_t*>(&meta) + mo,  
                    n  
                );  
            }  
        }  
        else if (src != nullptr) {  
            memcpy(page, src, chunk);  
        }  
        else {  
            memset(page, 0xFF, chunk);  
        }  
  
        if (!safe_flash_program(target + off, page, chunk)) {  
            Serial.printf(  
                "[FM] ERROR: state-slot program failed at 0x%06lx\n",  
                (unsigned long)(target + off)  
            );  
  
            dbg_c0_stage = 0;  
            return;  
        }  
  
        off += chunk;  
    }  
  
    flash_flush_cache();  
  
    current_active_slot = next_slot;  
    current_max_seq = next_seq;  
  
    Serial.printf(  
        "[FM] save_state: slot=%ld seq=%lu\n",  
        (long)next_slot,  
        (unsigned long)next_seq  
    );  
  
    dbg_c0_stage = 0;  
}

// ---------------------------------------------------------------------------
// Load state ring
// ---------------------------------------------------------------------------

bool load_state(void)
{
    int32_t best = -1;
    uint32_t best_seq = 0;

    for (int32_t s = 0; s < SLOTS; ++s) {
        const uint8_t* base =
            reinterpret_cast<const uint8_t*>(
                XIP_BASE +
                RING_OFFSET +
                (uint32_t)s * SLOT_SIZE
            );

        const SlotMeta* m =
            reinterpret_cast<const SlotMeta*>(
                base + SLOT_META_OFF
            );

        if (m->magic != SLOT_MAGIC ||
            m->seq <= best_seq) {
            continue;
        }

        if (slot_crc_flash(base) != m->crc32) {
            continue;
        }

        best = s;
        best_seq = m->seq;
    }

    if (best < 0) {
        return false;
    }

    const uint8_t* base =
        reinterpret_cast<const uint8_t*>(
            XIP_BASE +
            RING_OFFSET +
            (uint32_t)best * SLOT_SIZE
        );

    // Restore complete RAM disk image.
    memcpy(ram_disk_buffer, base, DISK_SIZE_BYTES);

    const SlotMeta* m =
        reinterpret_cast<const SlotMeta*>(
            base + SLOT_META_OFF
        );

    log_write_ptr = m->log_write_ptr;

    if (log_write_ptr >= LOG_MAX_BYTES ||
        (log_write_ptr % FLASH_PAGE_SIZE) != 0) {
        log_write_ptr = 0;
    }

    current_active_slot = best;
    current_max_seq = best_seq;

    return true;
}

// ---------------------------------------------------------------------------
// Default FAT12 disk image
// ---------------------------------------------------------------------------

static void create_default_fat_with_info_file(void)
{
    memset(ram_disk_buffer, 0, sizeof(ram_disk_buffer));

    uint8_t* b = ram_disk_buffer;

    b[0] = 0xEB;
    b[1] = 0x3C;
    b[2] = 0x90;

    memcpy(&b[3], "MSDOS5.0", 8);

    b[11] = 0x00;
    b[12] = 0x02;

    b[13] = 1;

    b[14] = 1;
    b[15] = 0;

    b[16] = 1;

    b[17] = 48;
    b[18] = 0;

    const uint16_t total_sectors = SECTOR_COUNT;

    memcpy(&b[19], &total_sectors, 2);

    b[21] = 0xF8;

    b[22] = FAT_SECTORS;
    b[23] = 0;

    b[24] = 1;
    b[25] = 0;

    b[26] = 1;
    b[27] = 0;

    b[36] = 0x80;
    b[38] = 0x29;

    const uint32_t volume_id = 0x20400603;

    memcpy(&b[39], &volume_id, 4);
    memcpy(&b[43], "RADIOMODEM ", 11);
    memcpy(&b[54], "FAT12   ", 8);

    b[510] = 0x55;
    b[511] = 0xAA;

    const uint32_t fat = SECTOR_SIZE;

    ram_disk_buffer[fat] = 0xF8;
    ram_disk_buffer[fat + 1] = 0xFF;
    ram_disk_buffer[fat + 2] = 0xFF;

    for (uint16_t c = INFO_FIRST_CLUSTER;
         c < INFO_FIRST_CLUSTER + INFO_CLUSTERS - 1;
         ++c) {
        set_fat12_entry(fat, c, c + 1);
    }

    set_fat12_entry(
        fat,
        INFO_FIRST_CLUSTER + INFO_CLUSTERS - 1,
        0xFFF
    );

    for (uint16_t c = CONFIG_FIRST_CLUSTER;
         c < CONFIG_FIRST_CLUSTER + CONFIG_CLUSTERS - 1;
         ++c) {
        set_fat12_entry(fat, c, c + 1);
    }

    set_fat12_entry(
        fat,
        CONFIG_FIRST_CLUSTER + CONFIG_CLUSTERS - 1,
        0xFFF
    );

    for (uint16_t c = SET_FIRST_CLUSTER;
         c < SET_FIRST_CLUSTER + SET_CLUSTERS - 1;
         ++c) {
        set_fat12_entry(fat, c, c + 1);
    }

    set_fat12_entry(
        fat,
        SET_FIRST_CLUSTER + SET_CLUSTERS - 1,
        0xFFF
    );

    for (uint16_t c = LOG_FIRST_CLUSTER;
         c < LOG_FIRST_CLUSTER + LOG_CLUSTERS - 1;
         ++c) {
        set_fat12_entry(fat, c, c + 1);
    }

    set_fat12_entry(
        fat,
        LOG_FIRST_CLUSTER + LOG_CLUSTERS - 1,
        0xFFF
    );

    dir_entry(0, "INFO    TXT", 0x20, INFO_FIRST_CLUSTER, 0);
    dir_entry(1, "CONFIG  TXT", 0x20, CONFIG_FIRST_CLUSTER, 0);
    dir_entry(2, "SET     TXT", 0x20, SET_FIRST_CLUSTER, 0);
    dir_entry(3, "LOG     TXT", 0x21, LOG_FIRST_CLUSTER, LOG_MAX_BYTES);

    for (uint8_t i = 0; i < 4; ++i) {
        set_dir_timestamp(i);
    }

    String info =
        "[CALL]" + my_call_variable + "[/CALL]\r\n"
        "[QTH]" + my_qth_variable + "[/QTH]\r\n"
        "[TEXT]" + my_text_variable + "[/TEXT]\r\n"
        "[FREQ_RX]" + my_freq_rx_var + "[/FREQ_RX]\r\n"
        "[DEMOD]" + my_demod_var + "[/DEMOD]\r\n"
        "[DEBUG]0[/DEBUG]\r\n";

    const uint32_t info_off =
        INFO_LBA * SECTOR_SIZE;

    const uint32_t info_len =
        info.length() < INFO_MAX_BYTES ?
        info.length() : INFO_MAX_BYTES;

    memcpy(
        &ram_disk_buffer[info_off],
        info.c_str(),
        info_len
    );

    const uint32_t root =
        SECTOR_SIZE * (1 + FAT_SECTORS);

    memcpy(&ram_disk_buffer[root + 28], &info_len, 4);

    String config = "# CONFIG.TXT\r\n";

    const uint32_t config_off =
        CONFIG_LBA * SECTOR_SIZE;

    const uint32_t config_len =
        config.length() < CONFIG_MAX_BYTES ?
        config.length() : CONFIG_MAX_BYTES;

    memcpy(
        &ram_disk_buffer[config_off],
        config.c_str(),
        config_len
    );

    memcpy(&ram_disk_buffer[root + 32 + 28], &config_len, 4);

    String set =
        "# SET.TXT — скрытые данные\r\n"
        "[MAX_CLK_SYS]150000000[/MAX_CLK_SYS]\r\n"
        "[MAX_PLL_VCO]1500000000[/MAX_PLL_VCO]\r\n"
        "[XOSC_PPB]25000[/XOSC_PPB]\r\n"
        "# [SCAN] ... [/SCAN]\r\n";

    const uint32_t set_off =
        SET_LBA * SECTOR_SIZE;

    const uint32_t set_len =
        set.length() < SET_MAX_BYTES ?
        set.length() : SET_MAX_BYTES;

    memcpy(
        &ram_disk_buffer[set_off],
        set.c_str(),
        set_len
    );

    memcpy(&ram_disk_buffer[root + 2 * 32 + 28], &set_len, 4);

    state_save_pending = true;
}

// ---------------------------------------------------------------------------
// Bounded parser for [TAG]...[/TAG] inside a fixed file region
// ---------------------------------------------------------------------------

static String tag_value_at(
    uint32_t lba,
    uint32_t area_len,
    const char* tag)
{
    char open[32];
    char close[36];

    snprintf(open, sizeof(open), "[%s]", tag);
    snprintf(close, sizeof(close), "[/%s]", tag);

    const char* base =
        reinterpret_cast<const char*>(
            &ram_disk_buffer[lba * SECTOR_SIZE]
        );

    const char* end = base + area_len;

    const size_t open_len = strlen(open);
    const size_t close_len = strlen(close);

    const char* p = nullptr;

    for (const char* c = base;
         c + open_len <= end;
         ++c) {
        if (!memcmp(c, open, open_len)) {
            p = c + open_len;
            break;
        }
    }

    if (p == nullptr) {
        return "";
    }

    const char* e = nullptr;

    for (const char* c = p;
         c + close_len <= end;
         ++c) {
        if (!memcmp(c, close, close_len)) {
            e = c;
            break;
        }
    }

    if (e == nullptr) {
        return "";
    }

    String result;

    result.reserve((unsigned int)(e - p));

    for (const char* c = p; c < e; ++c) {
        if (*c >= 32 && *c <= 126) {
            result += *c;
        }
    }

    result.trim();

    return result;
}

// ---------------------------------------------------------------------------
// Parse INFO.TXT
// ---------------------------------------------------------------------------

void read_and_parse_INFO_txt(void)
{
    String v;

    if ((v = tag_value_at(
            INFO_LBA,
            INFO_MAX_BYTES,
            "CALL")).length()) {
        my_call_variable = v;
    }

    if ((v = tag_value_at(
            INFO_LBA,
            INFO_MAX_BYTES,
            "QTH")).length()) {
        my_qth_variable = v;
    }

    if ((v = tag_value_at(
            INFO_LBA,
            INFO_MAX_BYTES,
            "TEXT")).length()) {
        my_text_variable = v;
    }

    if ((v = tag_value_at(
            INFO_LBA,
            INFO_MAX_BYTES,
            "FREQ_RX")).length()) {
        my_freq_rx_var = v;
    }

    if ((v = tag_value_at(
            INFO_LBA,
            INFO_MAX_BYTES,
            "DEMOD")).length()) {
        my_demod_var = v;
    }

    debug_flag =
        (tag_value_at(INFO_LBA, INFO_MAX_BYTES, "DEBUG") == "1");
}

// ---------------------------------------------------------------------------
// Parse CONFIG.TXT
// ---------------------------------------------------------------------------

void read_and_parse_CONFIG_txt(void)
{
    // Reserved for future CONFIG.TXT tags.
}

// ---------------------------------------------------------------------------
// Parse SET.TXT
// ---------------------------------------------------------------------------

void read_and_parse_SET_txt(void)
{
    String v;

    if ((v = tag_value_at(
            SET_LBA,
            SET_MAX_BYTES,
            "MAX_CLK_SYS")).length()) {

        const long n = v.toInt();

        if (n >= 100000000L && n <= 420000000L) {
            vfo_max_clk_sys_hz = (uint32_t)n;
        }
        else {
            Serial.printf(
                "[FM] SET: invalid MAX_CLK_SYS='%s'; keeping default\n",
                v.c_str()
            );
        }
    }

    if ((v = tag_value_at(
            SET_LBA,
            SET_MAX_BYTES,
            "MAX_PLL_VCO")).length()) {

        char* end = nullptr;

        const unsigned long n =
            strtoul(v.c_str(), &end, 10);

        if (end != v.c_str() &&
            *end == '\0' &&
            n >= 400000000UL &&
            n <= 4200000000UL) {
            vfo_max_pll_vco_hz = (uint32_t)n;
        }
        else {
            Serial.printf(
                "[FM] SET: invalid MAX_PLL_VCO='%s'; keeping default\n",
                v.c_str()
            );
        }
    }

    if ((v = tag_value_at(
            SET_LBA,
            SET_MAX_BYTES,
            "XOSC_PPB")).length()) {

        const long n = v.toInt();

        if (n >= -100000L && n <= 100000L) {
            vfo_xosc_corr_ppb = (int32_t)n;
        }
        else {
            Serial.printf(
                "[FM] SET: invalid XOSC_PPB='%s'; keeping default\n",
                v.c_str()
            );
        }
    }

    // TODO: scan table [SCAN].
}

// ---------------------------------------------------------------------------
// Print current settings
// ---------------------------------------------------------------------------

void print_current_settings(void)
{
    Serial.printf(
        "[FM] call=%s qth=%s f_rx=%s demod=%s debug=%d\n",
        my_call_variable.c_str(),
        my_qth_variable.c_str(),
        my_freq_rx_var.c_str(),
        my_demod_var.c_str(),
        (int)debug_flag
    );
}

// ---------------------------------------------------------------------------
// LOG.TXT recovery
//
// LOG is programmed in 256-byte pages.
// Each page is programmed as a complete page, with unused bytes = 0xFF.
//
// Recovery scans page boundaries and finds the first fully erased page.
// This is consistent with the existing append/wrap implementation.
// ---------------------------------------------------------------------------

static bool log_page_is_erased(const uint8_t* page)
{
    for (uint32_t i = 0; i < FLASH_PAGE_SIZE; ++i) {
        if (page[i] != 0xFF) {
            return false;
        }
    }

    return true;
}

static void log_boot_recover_ptr(void)
{
    const uint8_t* base =
        reinterpret_cast<const uint8_t*>(
            XIP_BASE + LOG_FLASH_OFFSET
        );

    for (uint32_t off = 0;
         off < LOG_MAX_BYTES;
         off += FLASH_PAGE_SIZE) {

        if (log_page_is_erased(base + off)) {
            log_write_ptr = off;

            Serial.printf(
                "[FM] LOG recovery: ptr=%lu\n",
                (unsigned long)log_write_ptr
            );

            return;
        }
    }

    // All pages are programmed. The next append wraps to page zero,
    // where the first sector will be erased before programming.
    log_write_ptr = 0;

    Serial.println("[FM] LOG recovery: ring full, next ptr=0");
}

// ---------------------------------------------------------------------------
// LOG.TXT append
// ---------------------------------------------------------------------------

void log_append(const uint8_t* data, uint32_t len)
{
    // Flash writes are deliberately disabled until Core 1 Flash safety
    // initialization has been completed and explicitly enabled.
    if (!flash_write_allowed()) {
        return;
    }

    if (trx_state() != TRX_RX) {
        return;
    }

    if (data == nullptr || len == 0) {
        return;
    }

    uint32_t consumed = 0;

    while (consumed < len) {
        if (log_write_ptr >= LOG_MAX_BYTES) {
            log_write_ptr = 0;
        }

        // Erase the sector immediately before its first page is written.
        if ((log_write_ptr % FLASH_SECTOR_SIZE) == 0) {
            if (!safe_flash_erase(
                    LOG_FLASH_OFFSET + log_write_ptr,
                    FLASH_SECTOR_SIZE)) {

                Serial.println("[FM] ERROR: LOG sector erase failed");
                return;
            }
        }

        const uint32_t room =
            FLASH_PAGE_SIZE -
            (log_write_ptr % FLASH_PAGE_SIZE);

        uint32_t n = len - consumed;

        if (n > room) {
            n = room;
        }

        alignas(4) uint8_t page[FLASH_PAGE_SIZE];

        memset(page, 0xFF, sizeof(page));

        memcpy(page, data + consumed, n);

        if (!safe_flash_program(
                LOG_FLASH_OFFSET + log_write_ptr,
                page,
                FLASH_PAGE_SIZE)) {

            Serial.println("[FM] ERROR: LOG page program failed");
            return;
        }

        log_write_ptr += FLASH_PAGE_SIZE;
        consumed += n;
    }

    flash_flush_cache();

    // USB MSC operations are kept on Core 0.
    remount_usb_disk();
}

// ---------------------------------------------------------------------------
// Public LOG.TXT API
// ---------------------------------------------------------------------------

void log_file_write_line(String message)
{
    if (!flash_write_allowed()) {
        return;
    }

    String line = message + "\r\n";

    log_append(
        reinterpret_cast<const uint8_t*>(line.c_str()),
        (uint32_t)line.length()
    );
}

void log_file_clear(void)
{
    if (!flash_write_allowed()) {
        return;
    }

    if (trx_state() != TRX_RX) {
        return;
    }

    if (!safe_flash_erase(
            LOG_FLASH_OFFSET,
            LOG_MAX_BYTES)) {

        Serial.println("[FM] ERROR: LOG clear failed");
        return;
    }

    flash_flush_cache();

    log_write_ptr = 0;

    remount_usb_disk();
}

// ---------------------------------------------------------------------------
// USB MSC remount
// ---------------------------------------------------------------------------

void remount_usb_disk(void)
{
    // TinyUSB MSC API must only be called from Core 0.
    if (get_core_num() != 0) {
        return;
    }

    if (!usb_started) {
        return;
    }

    usb_msc.setUnitReady(false);

    delay(50);

    usb_msc.setUnitReady(true);
}

// ---------------------------------------------------------------------------
// File manager preparation
// USB is NOT registered here.
// ---------------------------------------------------------------------------

void file_manager_prepare(void)
{
    if (file_manager_prepared) {
        Serial.println("[FM] already prepared");
        return;
    }

    Serial.println("[FM] prepare: restoring filesystem");

    // Restore the most recent RAM disk image first.
    const bool state_loaded = load_state();

    if (!state_loaded ||
        ram_disk_buffer[510] != 0x55 ||
        ram_disk_buffer[511] != 0xAA) {

        Serial.println(
            "[FM] no valid full-disk image; creating default FAT12 disk"
        );

        create_default_fat_with_info_file();
    }

    // Recover LOG pointer from actual Flash page contents.
    // This intentionally overrides the possibly stale state-ring copy.
    log_boot_recover_ptr();

    // Load configuration.
    read_and_parse_INFO_txt();
    read_and_parse_CONFIG_txt();
    read_and_parse_SET_txt();

    file_manager_prepared = true;

    Serial.printf(
        "[FM] prepare OK: slot=%ld seq=%lu log_ptr=%lu\n",
        (long)current_active_slot,
        (unsigned long)current_max_seq,
        (unsigned long)log_write_ptr
    );
}

// ---------------------------------------------------------------------------
// USB MSC registration
// ---------------------------------------------------------------------------


void init_usb_msc_interface(void)
{
    if (usb_started) {
        Serial.println("[MSC] already registered");
        return;
    }

    // RAM disk must be ready before MSC registration.
    file_manager_prepare();

    Serial.println("[MSC] registering");

    usb_msc.setID("RP2040", "RADIOMODEM", "1.0"); 

    usb_msc.setCapacity(SECTOR_COUNT, SECTOR_SIZE);

    usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb); 

    usb_msc.setUnitReady(true);  
    
    usb_msc.begin();  

    Serial.printf(
        "[MSC] registered: %u sectors x %u bytes\n",
        (unsigned)SECTOR_COUNT,
        (unsigned)SECTOR_SIZE
    );
}


// ---------------------------------------------------------------------------
// Backward-compatible initialization
// ---------------------------------------------------------------------------

void init_file_manager(void)
{
    file_manager_prepare();
    init_usb_msc_interface();
}

// ---------------------------------------------------------------------------
// Reset disk to defaults
// ---------------------------------------------------------------------------

void force_reset_to_default_disk(void)
{
    create_default_fat_with_info_file();

    read_and_parse_INFO_txt();
    read_and_parse_CONFIG_txt();
    read_and_parse_SET_txt();

    remount_usb_disk();
}

// ---------------------------------------------------------------------------
// Process PC changes and deferred state saves
// ---------------------------------------------------------------------------

void check_and_handle_pc_changes(void)  
{  
  
    if (core1_park_timed_out) {    
        core1_park_timed_out = false;    
        Serial.println("[FM] ERROR: core1 park timeout");    
    }  
  
    // This function performs Flash operations and MSC operations.  
    // It must be called from Core 0 only.  
    if (get_core_num() != 0) {  
        return;  
    }  
  
    if (state_save_pending) {    
        if (!flash_write_allowed()) {    
            return;    
        }    
    
        if (trx_state() != TRX_RX) {    
            return;    
        }    
    
        save_state();          // внутри — dbg_c0_stage = 55  
  
        // save_state() may return without writing if Flash safety  
        // is not enabled. Keep the pending flag in that case.  
        if (current_active_slot < 0 ||  
            current_max_seq == 0) {  
            return;  
        }  
  
        state_save_pending = false;    
    
        String hdr =    
            "\r\n=== session start, seq=" +    
            String(current_max_seq) +    
            " ===\r\n";  
  
        dbg_c0_stage = 56;     // log_append — тоже flash-путь  
        log_append(  
            reinterpret_cast<const uint8_t*>(hdr.c_str()),  
            (uint32_t)hdr.length()  
        );  
        dbg_c0_stage = 0;  
  
        remount_usb_disk();  
    }  
  
    if (!pc_activity_detected) {    
        return;    
    }    
    
    if (pc_written_regions == 0) {    
        // Flush без записей — нечего сохранять.    
        pc_activity_detected = false;    
        return;    
    }    
    
    if ((uint32_t)(millis() - last_msc_write_time) < 1500u) {    
        return;    
    }  
  
    if (!flash_write_allowed()) {  
        return;  
    }  
  
    if (trx_state() != TRX_RX) {  
        return;  
    }  
  
    const uint8_t regions = pc_written_regions;    
    
    Serial.printf("[FM] PC write: regions=0x%02X\n", (unsigned)regions);    
    save_state();              // внутри — dbg_c0_stage = 55  
    
    pc_activity_detected = false;    
    pc_written_regions = 0;  
  
    if (regions & 0x02) {  
        read_and_parse_INFO_txt();  
    }  
  
    if (regions & 0x04) {  
        read_and_parse_CONFIG_txt();  
    }  
  
    if (regions & 0x08) {  
        read_and_parse_SET_txt();  
    }  
  
    remount_usb_disk();  
  
    pc_file_written = false;  
}
