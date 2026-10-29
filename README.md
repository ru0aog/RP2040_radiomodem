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
├── RP2040_radiomodem.ino    // Точка входа: setup()/loop()/loop1(). Core 0 —  
│                            // приложение (модем, декодеры), Core 1 — реальное  
│                            // время (rx_dsp, vfo_poll, trx_poll_core1).  
│  
├── src/  
│   ├── pins.h               // Карта пинов: GP0/GP1 квад-LO, GP2 TX-меандр,  
│   │                        // GP26/GP27 ADC I/Q, I2C для Si5351.  
│   │  
│   ├── mem/  
│   │   └── mem_arena.h/.cpp // Двухфазный пул 96K: RX-фаза/TX-фаза, сброс  
│   │                        // при trx_switch. Выделение через MEM_PHASE_*.  
│   │  
│   ├── trx/  
│   │   └── trx_control.h/.cpp // Полудуплексная машина RX/TX/SWITCHING.  
│   │                        // Все переключения через trx_switch(), исполнение  
│   │                        // на Core 1 в trx_poll_core1() (стоп трактов →  
│   │                        // mem_arena_switch → VFO hook → старт).  
│   │  
│   ├── vfo/  
│   │   ├── vfo.h/.cpp       // Диспетчер частот: банк тонов (freqs_hz, только  
│   │   │                    // Гц!), выбор бэкенда по направлению, hook  
│   │   │                    // trx_control, разгон/термогвард, XOSC-коррекция.  
│   │   ├── soft_tx.h/.cpp   // TX-движок: PIO-меандр pio0 на GP2 + кооперативный  
│   │   │                    // MASH-1-1 дизеринг clkdiv + автотюн PLL.  
│   │   │                    // Экспортирует PllConfig/soft_tx_set_clk_sys/  
│   │   │                    // soft_tx_vsel_for/pll_nominal для soft_rx.  
│   │   ├── soft_rx.h/.cpp   // RX-движок: квадратурный LO на GP0/GP1 (pio1).  
│   │   │                    // PLL-скан под целочисленный делитель, f_nco-  
│   │   │                    // остаток уходит в rx_dsp. LO = f_rx − RX_IF_SHIFT.  
│   │   └── si5351.h/.cpp    // Заглушка внешнего генератора по I2C (перспектива).  
│   │  
│   ├── rx/                  // Приёмный тракт (порт из RP2040_receiver) — КАРКАС  
│   │   ├── rx_capture.h/.cpp// ADC0/ADC1 (GP26/GP27) + DMA, интерлив  
│   │   │                    // [31:16]=Q [15:0]=I, буферы из mem_arena.  
│   │   ├── rx_dsp.h/.cpp    // Core 1: NCO-сдвиг, 3× half-band FIR /8 →  
│   │   │                    // 15625 sps, демодулятор, AF-фильтры, RSID-ветка.  
│   │   ├── rx_decoders.h/.cpp// Маршрутизатор Core 0: сэмплы → активная мода,  
│   │   │                    // RSID-код → выбор моды.  
│   │   └── rx_usb_audio.h/.cpp // Очередь PCM-блоков → UAC-микрофон (16k/16bit).  
│   │  
│   ├── modes/               // Моды: каждая — самодостаточная папка TX+RX  
│   │   ├── modes.h          // Интерфейс ModeIface {id, name, rsid_code,  
│   │   │                    // rx_init, rx_push, tx_frame} + реестр.  
│   │   └── ax25/  
│   │       ├── ax25.h       // Публичный API моды.  
│   │       ├── ax25_hdlc.cpp// Общий протокол: NRZI, bit-stuff/destuff,  
│   │       │                // CRC-16/X.25, сборка/разбор UI-фрейма.  
│   │       ├── ax25_tx.cpp  // TX: vfo_bank mark/space ±100 Гц, 300 бод,  
│   │       │                // по-битовое переключение через vfo_tone.  
│   │       ├── ax25_rx.cpp  // RX: корреляторы → дискриминатор → slicer →  
│   │       │                // DPLL → NRZI → destuff → FCS (15625 sps вход).  
│   │       └── ax25_mode.cpp// Регистрация ModeIface моды.  
│   │  
│   ├── modem/               // Hayes-совместимый радиомодем — КАРКАС  
│   │   ├── modem.h/.cpp     // FSM ONLINE/COMMAND/DATA, парсер AT-строк,  
│   │   │                    // эхо/ответы, S-регистры → vfo/trx_control.  
│   │   ├── modem_cmds.h/.cpp// Реестр команд: AT, ATD/ATA, ATO, ATH, AT&F,  
│   │   │                    // ATSn=x, AT+FREQ=, AT+MODE=, AT+PTT, AT+RSID?  
│   │   └── modem_cfg.h      // Дефолты S-регистров, профили, таймауты.  
│   │  
│   ├── rsid/  
│   │   └── rsid.h/.cpp      // RSID-детектор (КАРКАС): корреляторы из  
│   │                        // приёмника, выход — код моды → rx_decoders.  
│   │  
│   └── usb/  
│       └── usb_audio.h/.cpp // КАРКАС UAC-микрофона: TUD_AUDIO_MIC дескриптор,  
│                            // стрим 16kHz моно PCM на хост.  
│  
└── docs/                    // (по мере появления) карта частот, форматы кадров.
```

