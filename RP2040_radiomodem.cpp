
// RP2040_radiomodem.cpp
//
// USB MSC + USB CDC CAT + VFO + RX DSP.
//
// Core 0:
//   USB, файловый менеджер, CAT, LED.
//
// Core 1:
//   Flash-safe init, VFO, ADC/DMA, RX DSP.
//
// CAT пока только для чтения:
//   ID; FA; IF;
//
// Перестройка частоты CAT отключена до переноса её выполнения
// в корректный межъядерный механизм trx_switch().

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


extern void core0_park_poll(void);

// -----------------------------------------------------------------------------
// Синхронизация ядер
// -----------------------------------------------------------------------------

static volatile bool flash_safe_ready = false;
static volatile bool core0_ready = false;
static volatile bool core1_ready = false;

static constexpr uint32_t START_FREQ_HZ = 3602000UL;

// -----------------------------------------------------------------------------
// Минимальный CAT: только команды чтения
// -----------------------------------------------------------------------------

static char cat_buf[32];
static uint8_t cat_len = 0;

static uint32_t cat_freq_hz = START_FREQ_HZ;

static void cat_send(const char* text) {
    // Ответы CAT передаём только при подключённом терминале.
    if (!Serial.dtr()) {
        return;
    }

    Serial.write(
        reinterpret_cast<const uint8_t*>(text),
        strlen(text)
    );
}

static void cat_execute(const char* cmd) {
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

    // Изменение частоты пока не разрешено.
    if (strncmp(cmd, "FA", 2) == 0) {
        cat_send("?;");
        return;
    }

    cat_send("?;");
}

static void cat_poll_readonly() {
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());

        if (c == ';') {
            cat_buf[cat_len] = '\0';
            cat_execute(cat_buf);
            cat_len = 0;
        } else if (c != '\r' && c != '\n') {
            if (cat_len < sizeof(cat_buf) - 1) {
                cat_buf[cat_len++] = c;
            } else {
                // Переполнение: сбрасываем неполную команду.
                cat_len = 0;
            }
        }
    }
}

// -----------------------------------------------------------------------------
// Core 0
// -----------------------------------------------------------------------------

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Сохраняем ранее проверенную последовательность USB/MSC.
    init_usb_msc_interface();

    Serial.begin(115200);

    // Инициализация арены до использования RX/TX модулей.
    mem_arena_init();

    // Файловый менеджер загружает сохранённое состояние.
    init_file_manager();

    Serial.println();
    Serial.println("[BOOT] USB MSC + CAT + VFO + RX DSP");
    Serial.println("[CAT] Read-only commands: ID; FA; IF;");

    // Инициализация VFO до привязки его к TRX-контроллеру.
    vfo_init();

    // Таблицы DSP и фильтры инициализируются до старта захвата.
    rx_dsp_init();

    // TRX-контроллер принимает callback VFO.
    trx_init(vfo_on_trx_switch);

    __dmb();
    core0_ready = true;

    // Во время переключения PLL Core 0 обязан обслуживать парковку.
    while (!core1_ready) {
        core0_park_poll();
        tight_loop_contents();
    }

    Serial.println("[BOOT] Core 1 initialized");
}

// -----------------------------------------------------------------------------
// Core 0 loop
// -----------------------------------------------------------------------------

void loop() {
    static uint32_t last_blink = 0;
    static uint32_t last_status = 0;
    static bool led_state = false;

    const uint32_t now = millis();

    if (static_cast<uint32_t>(now - last_blink) >= 250) {
        last_blink = now;
        led_state = !led_state;
        digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW);
    }

    if (flash_safe_ready) {
        check_and_handle_pc_changes();
    }

    cat_poll_readonly();

    if (static_cast<uint32_t>(now - last_status) >= 1000) {
        last_status = now;

        Serial.printf(
            "[STATUS] DSP_drop=%lu AUD_drop=%lu clip=%lu\n",
            static_cast<unsigned long>(rx_dsp_out_dropped()),
            static_cast<unsigned long>(rx_dsp_aud_dropped()),
            static_cast<unsigned long>(rx_dsp_clip_count())
        );
    }
}

// -----------------------------------------------------------------------------
// Core 1 setup
// -----------------------------------------------------------------------------

void setup1() {
    // Нужна для безопасного доступа к Flash при работе двух ядер.
    flash_safe_execute_core_init();

    __dmb();
    flash_safe_ready = true;

    while (!core0_ready) {
        tight_loop_contents();
    }

    Serial.println("[VFO] Initializing RX LO");

    // Первичная настройка LO выполняется на Core 1.
    vfo_on_trx_switch(TRX_RX, START_FREQ_HZ);

    // Устанавливаем рабочую частоту RX DSP.
    rx_dsp_start(START_FREQ_HZ);

    // Запускаем ADC/DMA после подготовки DSP.
    rx_capture_start();

    // Публикуем частоту для ответов CAT.
    cat_freq_hz = START_FREQ_HZ;

    __dmb();
    core1_ready = true;

    vfo_quad_debug();

    Serial.println("[RX] ADC/DMA capture started");
}

// -----------------------------------------------------------------------------
// Core 1 loop
// -----------------------------------------------------------------------------

void loop1() {
    // Обслуживание запросов переключения RX/TX.
    trx_poll_core1();

    if (trx_state() == TRX_RX) {
        rx_dsp_poll();
    }

    vfo_poll();
}
