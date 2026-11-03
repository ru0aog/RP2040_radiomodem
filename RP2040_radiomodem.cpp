
// RP2040_radiomodem.cpp
//
// RP2040_radiomodem
// USB MSC + USB CDC CAT + USB Audio + VFO + RX DSP
//
// Core 0:
//   USB, USB Audio, файловый менеджер, CAT, LED.
//   Обслуживание парковки Core 0 при изменении clk_sys.
//
// Core 1:
//   VFO, ADC/DMA, RX DSP.
//
// CAT: ID; FA; IF;
// Изменение частоты через CAT пока отключено.

#include <Arduino.h>
#include <pico/flash.h>
#include <hardware/sync.h>
#include <Adafruit_TinyUSB.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "src/mem/mem_arena.h"
#include "src/filesys/file_manager.h"
#include "src/vfo/vfo.h"
#include "src/trx/trx_control.h"
#include "src/rx/rx_dsp.h"
#include "src/rx/rx_capture.h"
#include "src/rx/rx_usb_audio.h"

// Реализация находится в src/trx/trx_control.cpp.
extern void core0_park_poll(void);

// -----------------------------------------------------------------------------
// Synchronization
// -----------------------------------------------------------------------------

// ВНИМАНИЕ:
// flash_safe_execute_core_init() пока намеренно пропущен:
// в текущей диагностической сборке его вызов на Core 1 блокировал запуск.
// Поэтому безопасная запись во Flash остаётся отключённой.

static volatile bool flash_safe_ready = false;
static volatile bool core0_ready = false;
static volatile bool core1_ready = false;

// Диагностический этап запуска Core 1:
//
// 0 — setup1 ещё не начат
// 1 — начало setup1
// 2 — Flash safety пропущен
// 3 — ожидание готовности Core 0
// 4 — настройка VFO
// 5 — запуск DSP
// 6 — запуск ADC/DMA
// 7 — инициализация завершена

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

    Serial.begin(115200);

    Serial.println();
    Serial.println("RP2040_radiomodem");
    Serial.println("[BOOT] Core 0 initialization");

    // 1. Арена должна быть готова до работы подсистем.
    mem_arena_init();

    // 2. Восстановление файловой системы и настроек.
    // USB-классы пока не регистрируются.
    file_manager_prepare();

    // 3. Инициализация VFO и DSP.
    vfo_init();
    rx_dsp_init();
    trx_init(vfo_on_trx_switch);

    // 4. Диагностика состояния TinyUSB.
    Serial.printf(
        "[USB] initialized=%u mounted=%u\n",
        (unsigned)TinyUSBDevice.isInitialized(),
        (unsigned)TinyUSBDevice.mounted()
    );

    // 5. Регистрация MSC.
    Serial.println("[USB] register MSC");
    init_usb_msc_interface();

    // 6. Регистрация UAC2.
    Serial.println("[USB] register Audio");
    usb_audio_init();

    // 7. Единое повторное перечисление.
    // Выполняется только после регистрации обоих классов.
    if (TinyUSBDevice.mounted()) {

        Serial.println("[USB] detach");

        TinyUSBDevice.detach();
        delay(20);

        TinyUSBDevice.attach();

        Serial.println("[USB] attach");
    }

    Serial.printf(
        "[USB] final: initialized=%u mounted=%u\n",
        (unsigned)TinyUSBDevice.isInitialized(),
        (unsigned)TinyUSBDevice.mounted()
    );

    // Core 1 разрешено запускать после подготовки Core 0.
    __dmb();
    core0_ready = true;

    Serial.println("[BOOT] Core 0 initialized");
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

    // USB Audio должен регулярно обслуживаться на Core 0.
    // Функция забирает аудиоблоки из очереди DSP и передаёт их
    // в существующий USB Audio Class.
    usb_audio_poll();

    if (core1_ready && !core1_reported) {
        core1_reported = true;
        Serial.println("[BOOT] Core 1 initialized");
    }

    // Сохранение виртуального диска разрешается только после
    // отдельного исправления инициализации Flash safety.
    //
    // Сейчас flash_safe_ready намеренно остаётся false:
    // не выполняем запись во Flash без безопасной остановки Core 1.
    if (flash_safe_ready) {
        check_and_handle_pc_changes();
    }

    // CAT: команды чтения ID; FA; IF;.
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

    // Временно пропускаем flash_safe_execute_core_init():
    // в предыдущей сборке функция блокировала Core 1.
    //
    // НЕ устанавливаем flash_safe_ready.
    // Запись во Flash остаётся запрещённой.

    core1_stage = 2;
    __dmb();

    // Этап 3: ожидание готовности Core 0.
    core1_stage = 3;
    __dmb();

    while (!core0_ready) {
        tight_loop_contents();
    }

    // Этап 4: настройка VFO.
    core1_stage = 4;
    __dmb();

    vfo_on_trx_switch(TRX_RX, START_FREQ_HZ);

    // Этап 5: запуск DSP.
    core1_stage = 5;
    __dmb();

    rx_dsp_start(START_FREQ_HZ);

    // Этап 6: запуск ADC/DMA.
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
    // Обработка запросов переключения RX/TX.
    trx_poll_core1();

    // DSP работает только в режиме приёма.
    if (trx_state() == TRX_RX) {
        rx_dsp_poll();
    }

    // Обслуживание VFO.
    vfo_poll();
}
