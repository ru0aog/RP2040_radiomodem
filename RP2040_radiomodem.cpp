
#include <Arduino.h>
#include <pico/flash.h>
#include <hardware/sync.h>

#include "src/mem/mem_arena.h"
#include "src/filesys/file_manager.h"

static volatile bool flash_safe_ready = false;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    mem_arena_init();
    init_file_manager();

    Serial.begin(115200);
    Serial.println("[BOOT] Minimal diagnostic firmware");
}

void loop() {
    static uint32_t last_blink = 0;
    static bool led_state = false;

    const uint32_t now = millis();

    if ((uint32_t)(now - last_blink) >= 250) {
        last_blink = now;
        led_state = !led_state;
        digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW);
    }

    if (flash_safe_ready) {
        check_and_handle_pc_changes();
    }
}

void setup1() {
    flash_safe_execute_core_init();
    __dmb();
    flash_safe_ready = true;
}

void loop1() {
    tight_loop_contents();
}
