[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/ru0aog/RP2040_radiomodem)

# RP2040_radiomodem  
  
Цифровой радиомодем на RP2040 (полудуплекс): SDR-приёмник и  
передатчик цифровых режимов (CW/RTTY/MFSK/JTTY/…) с управляющей  
логикой — CAT-интерфейс, планировщик, файловая система.  
  
- См. `docs/ARCHITECTURE.md` — ядра, T/R-переключение, менеджер памяти.  
- Сборка: Arduino-Pico (Earle Philhower), TinyUSB-стек, FQBN  
  `rp2040:rp2040:generic`.

### Структура проекта

```text
RP2040_radiomodem/  
├── RP2040_radiomodem.cpp    // Точка входа: setup(), loop(), setup1(), loop1()  
├── platformio.ini           // Сборка: earlephilhower/arduino-pico, TinyUSB-стек  
├── src/  
│   ├── pins.h/.cpp          // Определения выводов GPIO и вспомогательные функции инициализации  
│   ├── mem/                 // Двухфазная арена памяти: TRX-пул 36 КБ + shared 8 КБ (MEM_PHASE_*)  
│   ├── trx/                 // Конечный автомат полудуплекса (trx_switch, trx_poll_core1)  
│   ├── vfo/                 // Диспетчер VFO, quad_vfo (PLL/PIO), soft_tx (PIO0), soft_rx (PIO1)  
│   ├── rx/                  // Конвейер приёмника (rx_capture, rx_dsp, rx_decoders)  
│   │   └── rx_usb_audio.cpp // UAC2-микрофон: USBAudioDeviceTinyUSB, usb_audio_init/poll  
│   ├── filesys/             // USB-MSC диск FAT12 (566 секторов, RAM 54) + ring-хранилище во Flash  
│   ├── cat/                 // CAT-интерфейс (команды ID/FA/…, общий CDC-порт)  
│   ├── modes/               // Подключаемые режимы работы (ModeIface, AX.25, CW)  
│   ├── modem/               // Слой AT-команд, совместимый с Hayes, и S-регистры  
│   └── rsid/                // Корреляторы автоопределения RSID  
└── docs/                    // Документация по архитектуре и карты частот
```

