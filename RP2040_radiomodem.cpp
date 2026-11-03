// RP2040_radiomodem.cpp
//
// USB MSC + USB CDC CAT + VFO + RX DSP.
//
// Core 0:
//   USB, файловый менеджер, CAT, LED.
//   Обслуживание парковки Core 0 при изменении тактовой частоты.
//
// Core 1:
//   Flash-safe init, VFO, ADC/DMA, RX DSP.
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
// Синхронизация ядер
// -----------------------------------------------------------------------------

static volatile bool flash_safe_ready = false;
static volatile bool core0_ready = false;
static volatile bool core1_ready = false;

static constexpr uint32_t START_FREQ_HZ = 3602000UL;

// -----------------------------------------------------------------------------
// Минимальный CAT: команды чтения
// -----------------------------------------------------------------------------

static char cat_buf[32];
static uint8_t cat_len = 0;

static uint32_t cat_freq_hz = START_FREQ_HZ;

static void cat_send(const char* text)
{
    if (text == nullptr) {
        return;
    }

    // Не проверяем Serial.dtr():
    // некоторые терминалы не устанавливают DTR,
    // хотя COM-порт открыт и команды поступают.
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

    // Перестройка частоты пока не разрешена.
    // Для безопасной реализации требуется межъядерный
    // механизм перестройки через TRX-контроллер.
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
                // Переполнение буфера:
                // сбрасываем неполную команду.
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

    // Сохраняем последовательность инициализации USB MSC,
    // которая уже работала в диагностической прошивке.
    init_usb_msc_interface();

    Serial.begin(115200);

    // Инициализация памяти.
    mem_arena_init();

    // Инициализация файлового менеджера и восстановление
    // сохранённого состояния виртуального диска.
    init_file_manager();

    Serial.println();
    Serial.println("[BOOT] USB MSC + CAT + VFO + RX DSP");
    Serial.println("[CAT] Commands: ID; FA; IF;");

    // Инициализация VFO.
    vfo_init();

    // Подготовка таблиц и фильтров DSP.
    rx_dsp_init();

    // Подключаем VFO к TRX-контроллеру.
    trx_init(vfo_on_trx_switch);

    // Сообщаем Core 1, что подготовка Core 0 завершена.
    __dmb();
    core0_ready = true;

    // ВАЖНО:
    // Не ждём здесь core1_ready.
    // Если запуск VFO или ADC/DMA на Core 1 зависнет,
    // Core 0 всё равно должен продолжать обслуживать
    // LED, CAT и файловый менеджер.
    Serial.println("[BOOT] Core 0 ready");
}

// -----------------------------------------------------------------------------
// Core 0 loop
// -----------------------------------------------------------------------------

void loop()
{
    // Обязательно обслуживаем запросы парковки Core 0,
    // которые возникают при изменении тактовой частоты VFO.
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

    // Сообщение о завершении инициализации Core 1.
    if (core1_ready && !core1_reported) {
        core1_reported = true;
        Serial.println("[BOOT] Core 1 initialized");
    }

    // Обработка изменений виртуального диска.
    // Функция самостоятельно определяет, когда нужно
    // сохранять изменения во Flash.
    if (flash_safe_ready) {
        check_and_handle_pc_changes();
    }

    // Обработка CAT-команд.
    cat_poll_readonly();

    // Диагностика DSP раз в секунду.
    if (static_cast<uint32_t>(now - last_status) >= 1000) {
        last_status = now;

        Serial.printf(
            "[STATUS] Core1=%u DSP_drop=%lu AUD_drop=%lu clip=%lu\n",
            core1_ready ? 1U : 0U,
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
    // Подготовка Core 1 для безопасной работы с Flash.
    flash_safe_execute_core_init();

    __dmb();
    flash_safe_ready = true;

    // Ожидаем готовности Core 0.
    // Core 0 в это время завершает setup().
    while (!core0_ready) {
        tight_loop_contents();
    }

    Serial.println("[VFO] Initializing RX LO");

    // Первичная настройка гетеродина.
    // Core 0 продолжает обслуживать core0_park_poll()
    // из loop(), если изменение тактовой частоты потребует
    // парковки основного ядра.
    vfo_on_trx_switch(TRX_RX, START_FREQ_HZ);

    // Запуск DSP-приёмника на начальной частоте.
    rx_dsp_start(START_FREQ_HZ);

    // Запуск захвата АЦП/DMA после подготовки DSP.
    rx_capture_start();

    // Публикуем начальную частоту для CAT.
    cat_freq_hz = START_FREQ_HZ;

    __dmb();
    core1_ready = true;

    vfo_quad_debug();

    Serial.println("[RX] ADC/DMA capture started");
}

// -----------------------------------------------------------------------------
// Core 1 loop
// -----------------------------------------------------------------------------

void loop1()
{
    // Обработка запросов переключения RX/TX.
    trx_poll_core1();

    // Обработка DSP только в режиме приёма.
    if (trx_state() == TRX_RX) {
        rx_dsp_poll();
    }

    // Обслуживание VFO.
    vfo_poll();
}