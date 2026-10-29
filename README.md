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
├── RP2040_radiomodem.ino    // Точка входа: setup(), loop(), setup1(), loop1()
├── src/  
│   ├── pins.h/.cpp          // Определения выводов GPIO и вспомогательные функции инициализации
│   ├── mem/                 // 96-килобайтная двухфазная арена памяти (MEM_PHASE_*)
│   ├── trx/                 // Конечный автомат полудуплекса (trx_switch, trx_poll_core1)
│   ├── vfo/                 // Диспетчер VFO, soft_tx (PIO0), soft_rx (PIO1)
│   ├── rx/                  // Конвейер приемника (rx_capture, rx_dsp, rx_decoders)
│   ├── modes/               // Подключаемые режимы работы (ModeIface, AX.25, CW)
│   ├── modem/               // Слой AT-команд, совместимый с Hayes, и S-регистры
│   ├── rsid/                // Корреляторы автоопределения RSID
│   └── usb/                 // Потоковая передача аудио TinyUSB UAC (микрофон)
└── docs/                    // Документация по архитектуре и карты частот
```

