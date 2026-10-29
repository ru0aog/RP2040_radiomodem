# RP2040_radiomodem  
  
Цифровой радиомодем на RP2040 (полудуплекс): SDR-приёмник и  
передатчик цифровых режимов (CW/RTTY/MFSK/JTTY/…) с управляющей  
логикой — CAT-интерфейс, планировщик, файловая система.  
  
- См. `docs/ARCHITECTURE.md` — ядра, T/R-переключение, менеджер памяти.  
- Сборка: Arduino-Pico (Earle Philhower), TinyUSB-стек, FQBN  
  `rp2040:rp2040:generic`.

## 📂 Структура исходного кода (`src/`)

src/
├── 📡 rx/                     # Тракт приёма
│   ├── rx_capture.h/.cpp      # ADC+DMA, GP26/27 — заглушка
│   ├── rx_dsp.h/.cpp          # DDC+децимация → 15625 sps — заглушка
│   ├── rx_decoders.h/.cpp     # маршрутизатор мод ↔ сэмплы, RSID
│   └── rx_usb_audio.h/.cpp    # обёртка: UAC-микрофон, push-блоки PCM
│
├── 🏎️ modes/                  # Моды
│   ├── modes.h                # общий интерфейс моды (ModeIface)
│   └── 🐳 ax25/               # AX.25
│       ├── ax25.h/.cpp        # API моды
│       ├── ax25_hdlc.cpp      # NRZI, stuffing, CRC-16/X.25
│       ├── ax25_tx.cpp        # TX: queue, timing, mark/space
│       └── ax25_rx.cpp        # RX: корреляторы → slicer → DPLL → HDLC
│
├── 🕹️ modem/                  # Модем
│   ├── modem.h/.cpp           # FSM COMMAND/DATA/ONLINE, V.25ter
│   ├── modem_at.cpp           # диспетчер AT-команд
│   ├── modem_sreg.h/.cpp      # S-регистры
│   └── modem_cfg.h            # дефолты
│
├── 🛰️ rsid/                   # RSID
│   └── rsid.h/.cpp            # RSID-детектор (порт из приёмника)
│
└── 🔌 usb/                    # USB
└── usb_audio.h/.cpp       # tusb_config + дескрипторы UAC mic (позже)
