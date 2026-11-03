// RP2040_radiomodem.cpp
//
// RP2040_radiomodem
// USB MSC + USB CDC CAT + VFO + RX DSP
//
// Core 0:
//   USB, файловый менеджер, CAT, LED.
//   Обслуживание парковки Core 0 при изменении clk_sys.
//
// Core 1:
//   Flash safety, VFO, ADC/DMA, RX DSP.
//
// CAT: ID; FA; IF;
// Изменение частоты через CAT пока отключено.

#include <Arduino.h>
#include <pico/flash.h>
#include <hardware/sync.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "src/mem/mem_arena.h"
#include "src/filesys/file_manager.h"
#include "src/vfo/vfo.h"
#include "src/trx/trx_control.h"
#include "src/rx/rx_dsp.h"
#include "src/rx/rx_capture.h"

// Реализация находится в src/trx/trx_control.cpp.
extern void core0_park_poll(void);

// -----------------------------------------------------------------------------
// Synchronization
// -----------------------------------------------------------------------------

static volatile bool flash_safe_ready = false;
static volatile bool core0_ready = false;
static volatile bool core1_ready = false;

// Диагностический этап запуска Core 1:
//
// 0 — setup1 ещё не начат
// 1 — выполняется flash_safe_execute_core_init()
// 2 — Flash safety инициализирован
// 3 — ожидание готовности Core 0
// 4 — настройка VFO
// 5 — запуск DSP
// 6 — запуск ADC/DMA
// 7 — инициализация завершена
//
// Если stage постоянно равен 1, зависание происходит
// внутри flash_safe_execute_core_init().

static volatile uint32_t core1_stage = 0;

static constexpr uint32_t START_FREQ_HZ = 3602000UL;

// -----------------------------------------------------------------------------
// CAT — команды чтения
// -----------------------------------------------------------------------------

static char cat_buf[32];
static uint8_t cat_len = 0;

static uint32_t cat_freq_hz = START_FREQ_HZ;

static void cat_send(const char* text)
{
    if (text == nullptr) {
        return;
    }

    // Не требуем DTR от терминала.
    Serial.write(
        reinterpret_cast<const uint8_t*>(text),
        strlen(text)
    );
}

static void cat_execute(const char* cmd)
{
    if (strcmp(cmd, "ID") == 0) {
        cat_send("ID019;");
        return;
    }

    if (strcmp(cmd, "FA") == 0) {
        char out[20];

        snprintf(
            out,
            sizeof(out),
            "FA%011lu;",
            static_cast<unsigned long>(cat_freq_hz)
        );

        cat_send(out);
        return;
    }

    if (strcmp(cmd, "IF") == 0) {
        char out[] = "IF00000000000       -00300   20000000;";

        uint32_t f = cat_freq_hz;

        for (int i = 12; i >= 2; --i) {
            out[i] = static_cast<char>('0' + (f % 10));
            f /= 10;
        }

        cat_send(out);
        return;
    }

    // Перестройка частоты через CAT пока отключена.
    // Для неё необходим отдельный запрос к TRX-контроллеру.
    if (strncmp(cmd, "FA", 2) == 0) {
        cat_send("?;");
        return;
    }

    cat_send("?;");
}

static void cat_poll_readonly()
{
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());

        if (c == ';') {
            cat_buf[cat_len] = '\0';

            cat_execute(cat_buf);

            cat_len = 0;
        }
        else if (c != '\r' && c != '\n') {
            if (cat_len < sizeof(cat_buf) - 1) {
                cat_buf[cat_len++] = c;
            }
            else {
                // Переполнение буфера — сброс команды.
                cat_len = 0;
            }
        }
    }
}

// -----------------------------------------------------------------------------
// Core 0 setup
// -----------------------------------------------------------------------------

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Не добавлять flash_safe_execute_core_init() сюда:
    // предыдущая версия с этим вызовом перестала запускаться.
    //
    // Сохраняем последовательность инициализации USB,
    // при которой диск и COM-порт определялись.

    init_usb_msc_interface();

    Serial.begin(115200);

    mem_arena_init();

    init_file_manager();

    Serial.println();
    Serial.println("[BOOT] RP2040_radiomodem");
    Serial.println("[BOOT] USB MSC + CAT + VFO + RX DSP");

    vfo_init();

    rx_dsp_init();

    trx_init(vfo_on_trx_switch);

    __dmb();
    core0_ready = true;
    __dmb();

    Serial.println("[BOOT] Core 0 ready");
}

// -----------------------------------------------------------------------------
// Core 0 loop
// -----------------------------------------------------------------------------

void loop()
{
    // Обслуживание парковки Core 0 при изменении тактовой частоты VFO.
    core0_park_poll();

    static uint32_t last_blink = 0;
    static uint32_t last_status = 0;

    static bool led_state = false;
    static bool core1_reported = false;

    const uint32_t now = millis();

    // Светодиод: переключение каждые 250 мс.
    if (static_cast<uint32_t>(now - last_blink) >= 250) {
        last_blink = now;

        led_state = !led_state;

        digitalWrite(
            LED_BUILTIN,
            led_state ? HIGH : LOW
        );
    }

    if (core1_ready && !core1_reported) {
        core1_reported = true;
        Serial.println("[BOOT] Core 1 initialized");
    }

    // Сохранение виртуального диска разрешается только после
    // инициализации Flash safety на Core 1.
    //
    // Пока Core 1 зависает на этапе 1, эта функция не вызывается.
    // Поэтому сохранение CONFIG.TXT этим кодом не исправлено.
    if (flash_safe_ready) {
        check_and_handle_pc_changes();
    }

    cat_poll_readonly();

    // Диагностика раз в секунду.
    if (static_cast<uint32_t>(now - last_status) >= 1000) {
        last_status = now;

        Serial.printf(
            "[STATUS] Core1=%u stage=%lu "
            "DSP_drop=%lu AUD_drop=%lu clip=%lu\n",
            core1_ready ? 1U : 0U,
            static_cast<unsigned long>(core1_stage),
            static_cast<unsigned long>(rx_dsp_out_dropped()),
            static_cast<unsigned long>(rx_dsp_aud_dropped()),
            static_cast<unsigned long>(rx_dsp_clip_count())
        );
    }
}

// -----------------------------------------------------------------------------
// Core 1 setup
// -----------------------------------------------------------------------------

void setup1()
{
    // Этап 1: диагностика.
    core1_stage = 1;
    __dmb();

    // ВРЕМЕННО ПРОПУСКАЕМ:
    // flash_safe_execute_core_init();
    //
    // По диагностике именно этот вызов блокирует Core 1.

    // Важно: flash_safe_ready НЕ устанавливаем!
    // До решения проблемы запрещаем операции сохранения во Flash.

    core1_stage = 2;
    __dmb();

    // Этап 3: ожидание готовности Core 0.
    core1_stage = 3;
    __dmb();

    while (!core0_ready) {
        tight_loop_contents();
    }

    // Этап 4: VFO.
    core1_stage = 4;
    __dmb();

    vfo_on_trx_switch(TRX_RX, START_FREQ_HZ);

    // Этап 5: DSP.
    core1_stage = 5;
    __dmb();

    rx_dsp_start(START_FREQ_HZ);

    // Этап 6: ADC/DMA.
    core1_stage = 6;
    __dmb();

    rx_capture_start();

    cat_freq_hz = START_FREQ_HZ;

    // Этап 7: запуск завершён.
    __dmb();
    core1_ready = true;
    core1_stage = 7;
    __dmb();

    vfo_quad_debug();

    Serial.println("[RX] ADC/DMA capture started");
}

// -----------------------------------------------------------------------------
// Core 1 loop
// -----------------------------------------------------------------------------

void loop1()
{
    trx_poll_core1();

    if (trx_state() == TRX_RX) {
        rx_dsp_poll();
    }

    vfo_poll();
}