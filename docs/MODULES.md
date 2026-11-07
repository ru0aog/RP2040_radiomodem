# MODULES.md — каталог модулей RP2040_radiomodem  
  
Статусы: **DONE** — работает и проверено; **WIP** — частично реализовано;  
**STUB** — заглушка; **UNTESTED** — код есть, на живом сигнале/нагрузке  
не проверено.  
  
---  
  
## 1. Платформа  
  
### 1.1 `src/pins` — карта выводов  
- **Файлы:** `src/pins.h`, `src/pins.cpp`  
- **Назначение:** единая таблица GPIO: ADC I/Q (GP26/GP27), PIO TX,  
  `PIN_PTT_LED` (GP25), I2C для Si5351 и пр.  
- **Интерфейсы:** константы `PIN_*`.  
- **Зависимости:** нет.  
- **Статус:** DONE.  
- **Проблемы:** аппаратного PTT-выхода нет — только светодиод  
  `PIN_PTT_LED`; для реального трансивера нужен отдельный пин PTT.  
  
### 1.2 `src/mem/mem_arena` — фазовый менеджер памяти  
- **Файлы:** `src/mem/mem_arena.h`, `src/mem/mem_arena.cpp`  
- **Назначение:** RX- и TX-фазы делят один TRX-пул (36 КБ,  
  union-семантика); SHARED-область (8 КБ) — настройки, CAT-буферы,  
  не очищается никогда. Без malloc, без фрагментации.  
- **Интерфейсы:** `mem_arena_switch(фаза)`, `mem_arena_release()`,  
  аллокация внутри фазы, watermark-диагностика.  
- **Зависимости:** вызывается из `trx_control` при каждом переключении.  
- **Статус:** DONE.  
- **Проблемы:** `arena watermark: 0/36864` — TRX-пул в RX, возможно,  
  не используется (либо watermark не обновляется); если реальный ноль —  
  пул можно урезать в пользу .bss.  
  
### 1.3 `src/trx/core_sync` — межъядерная синхронизация и парковка  
- **Файлы:** `src/trx/core_sync.h`, `src/trx/core_sync.cpp`  
- **Назначение:** парковка Core 1 для flash-erase/write и смены PLL  
  (XIP-off); per-core save/restore PRIMASK.  
- **Интерфейсы:** `core_sync_core1_poll()` (Core 1), запросы парковки  
  с Core 0, `core_sync_pll_*` (save/try_lock/unlock_unsafe/restore).  
- **Зависимости:** используется `file_manager` (flash), `quad_vfo` (PLL).  
- **Статус:** DONE — первопричина зависаний (межъядерное смешивание  
  PRIMASK) устранена в `a58c91f`.  
- **Проблемы:** `multicore_lockout_victim_init()` несовместим  
  (конфликт за SIO_IRQ_PROC1) — не использовать; `core0_wait_in_ram`  
  и `while(!core1_pll_done)` без второго таймаута (TODO).  
  
### 1.4 `src/trx/trx_control` — переключение RX/TX  
- **Файлы:** `src/trx/trx_control.h`, `src/trx/trx_control.cpp`  
- **Назначение:** единая точка полудуплексного переключения:  
  stop тракта → `mem_arena_switch` → VFO-hook → start тракта.  
  Исполняется на Core 1 (`trx_poll_core1`).  
- **Интерфейсы:** `trx_switch(dir, freq_hz)`, `trx_state()`,  
  `trx_poll_core1()`, `TrxVfoHook`.  
- **Зависимости:** `mem_arena`, `vfo`, `rx_capture`, `rx_dsp`, `tx_job`.  
- **Статус:** DONE для RX→TX→RX в тестах; UNTESTED под нагрузкой JTTY.  
- **Проблемы:** цикл RX→TX→RX при активной JTTY (перевыделение арены)  
  не тестирован; `tx_pipeline_start/stop` — weak-заглушки.  
  
### 1.5 `src/trx/tx_job` — очередь заданий на передачу  
- **Файлы:** `src/trx/tx_job.h`, `src/trx/tx_job.cpp`  
- **Назначение:** асинхронная очередь передач для всех источников  
  (CAT, CLI, будущий modem/BBS). FSM:  
  `TXJ_IDLE→QUEUED→ACTIVE→DONE/ABORTED/ERROR`.  
- **Интерфейсы:** `tx_job_submit(mode, freq, payload)`,  
  `tx_job_poll()` (Core 0), `tx_job_abort()`, `tx_job_state()`.  
  Поле `payload[TX_JOB_PAYLOAD_MAX]`, `error` (<0 — код ошибки).  
- **Зависимости:** `trx_control`, `modes` (`ModeIface::tx_frame`).  
- **Статус:** DONE (каркас), UNTESTED end-to-end.  
- **Проблемы:** `tx_frame` принимает `const uint8_t*` — каст от  
  `char*` обязателен; моды должны проверять `TXJ_ABORTED` в  
  посимвольном цикле (аналог `mfsk_abort()` маяка).  
  
---  
  
## 2. Радиотракт  
  
### 2.1 `src/rx/rx_capture` — захват ADC+DMA  
- **Файлы:** `src/rx/rx_capture.h`, `src/rx/rx_capture.cpp`  
- **Назначение:** ADC0+ADC1 (I/Q), DMA пинг-понг, IRQ0; Fs=250k  
  суммарно, блок 512 пар I/Q (~488 `cap_isr`/с).  
- **Интерфейсы:** `rx_capture_start()`, `rx_capture_stop()`,  
  счётчики `cap_isr/drop/fcsover/qfull/dead`.  
- **Зависимости:** `pins`, DMA/IRQ RP2040; потребитель — `rx_dsp`.  
- **Статус:** DONE — `drop=0`, `fcsover=0` на 373 МГц.  
- **Проблемы:** `Serial.println` в `rx_capture_start` — допустимо до  
  горячего цикла, убрать при переносе старта в парковочный путь.  
  
### 2.2 `src/rx/rx_dsp` — DDC и демодуляция (Core 1)  
- **Файлы:** `src/rx/rx_dsp.h`, `src/rx/rx_dsp.cpp`  
- **Назначение:** центрирование → NCO-сдвиг → CIC /2 → 2×half-band FIR  
  (/4) → 15625 sps I/Q → демодулятор RAW/USB/LSB/AM/FM → ФНЧ → notch →  
  NR → аудио-очередь. Встроен ресемплер Фарроу 15625→11025 для RSID.  
- **Интерфейсы:** `rx_dsp_init/start/stop/poll`,  
  `rx_dsp_set_nco_hz` (AFC), `rx_dsp_set_demod(RXD_DEMOD_*)`,  
  `rx_dsp_set_af_band/notch/nr`, очереди `rx_dsp_get_output` /  
  `rx_dsp_get_audio` (для Core 0), метрики `*_load_permille`,  
  `*_dropped`, `clip_count`.  
- **Зависимости:** `rx_capture`, `quad_vfo` (NCO = f_rx − lo_hz).  
- **Статус:** DONE — нагрузка `dsp≈390‰` на 373 МГц.  
- **Проблемы:** квант `DSP_MAX_BLOCKS_PER_POLL=2` за проход `loop1`  
  (коммит c208765) — защита core_sync/trx_poll, но требует проверки  
  влияния на RSID.  
  
### 2.3 `src/rx/rx_decoders` — маршрутизация в декодеры мод  
- **Файлы:** `src/rx/rx_decoders.h`, `src/rx/rx_decoders.cpp`  
- **Назначение:** выборка базовой полосы I/Q из `rx_dsp_get_output`  
  и раздача в `ModeIface::rx_push` активной моды (Core 0).  
- **Интерфейсы:** `rx_decoders_poll()`, регистрация/выбор моды.  
- **Зависимости:** `rx_dsp`, `modes`.  
- **Статус:** DONE.  
- **Проблемы:** дубли `[DEC] init` в логе — двойной init, не критично.  
  
### 2.4 `src/rx/rx_goertzel` — банки Гёрцеля  
- **Файлы:** `src/rx/rx_goertzel.h`, `src/rx/rx_goertzel.cpp`  
- **Назначение:** тональные детекторы для MFSK-мод (Olivia, Contestia,  
  IFKP, JTTY) и RSID-коррелятора.  
- **Интерфейсы:** инициализация банка под сетку тонов, `push(sample)`,  
  чтение мощностей (точная сигнатура — см. заголовок).  
- **Зависимости:** `mem_arena` (таблицы в RX-фазе).  
- **Статус:** DONE (используется мода́ми и RSID).  
- **Проблемы:** не выявлены.  
  
### 2.5 `src/rx/rx_pipeline` — конвейер RX  
- **Файлы:** `src/rx/rx_pipeline.cpp` (заголовок отсутствует)  
- **Назначение:** связывание этапов RX (capture→dsp→decoders);  
  часть функциональности, вероятно, weak-переопределяемая.  
- **Статус:** WIP — требует уточнения состава функций.  
- **Проблемы:** нет публичного заголовка — интерфейс не зафиксирован.  
  
### 2.6 `src/rx/rx_usb_audio` — вывод аудио в UAC2  
- **Файлы:** `src/rx/rx_usb_audio.h`, `src/rx/rx_usb_audio.cpp`  
- **Назначение:** моно-PCM из `rx_dsp_get_audio` → UAC2 на ПК.  
- **Интерфейсы:** `usb_audio_poll()` (Core 0), счётчики  
  `aud_fill/aud_drop`.  
- **Зависимости:** `rx_dsp`, TinyUSB UAC.  
- **Статус:** UNTESTED — `aud_fill=0`/`aud_drop=0`, на хосте не  
  проверено; при щелчках смотреть частоту `usb_audio_poll()`.  
  
### 2.7 `src/vfo/vfo` — фасад синтезатора  
- **Файлы:** `src/vfo/vfo.h`, `src/vfo/vfo.cpp`  
- **Назначение:** единый API частоты для trx/modes: `vfo_on_trx_switch`,  
  `vfo_poll()` (TX-фаза, Core 1), `vfo_tone(i)`, `vfo_bank_set_grid`,  
  `vfo_actual_hz`, `vfo_clk_sys_hz`.  
- **Зависимости:** `quad_vfo`, `soft_tx`, `si5351`.  
- **Статус:** DONE.  
- **Проблемы:** частота приёма зашита в нескольких точках `.ino` —  
  при смене править все вызовы.  
  
### 2.8 `src/vfo/quad_vfo` — квадратурный LO на PLL  
- **Файлы:** `src/vfo/quad_vfo/quad_vfo.{h,cpp}`,  
  `quad_vfo_pll.{h,cpp}`  
- **Назначение:** квадратурный LO до ~45 МГц на системной PLL;  
  сканер fbdiv/postdiv; измерение реальной частоты через FC0;  
  разгон clk_sys (рабочая точка 373.3 МГц, потолок 380 МГц,  
  VREG 1.30V).  
- **Интерфейсы:** `vfo_quad_lo_hz()`, `quad_vfo_clk_sys_hz()`,  
  `vfo_quad_debug()` ([LODBG]), потолки `vfo_max_clk_sys_hz`,  
  `vfo_max_pll_vco_hz`, `vfo_xosc_corr_ppb`.  
- **Зависимости:** `core_sync` (смена PLL через парковку).  
- **Статус:** DONE — стабильность подтверждена длительными прогонами.  
- **Проблемы:** VCO 3360 МГц вне спека PLL — на другом чипе может не  
  взять; `SET.TXT:MAX_CLK_SYS` молча перекрывает потолок; дискретность  
  решётки даёт LO err −355…+3500 Гц (не баг).  
  
### 2.9 `src/vfo/soft_tx` — PIO-генератор тонов  
- **Файлы:** `src/vfo/soft_tx.h`, `src/vfo/soft_tx.cpp`  
- **Назначение:** формирование AFSK/FSK-тонов на PIO0 для TX-мод.  
- **Интерфейсы:** `soft_tx_debug()`, установка тона/частоты.  
- **Зависимости:** `pins`, PIO0.  
- **Статус:** DONE (TX-тест 4 тонов по 170 Гц работал, сейчас  
  отключён флагом `TX_TEST_ENABLED=false`).  
  
### 2.10 `src/vfo/si5351` — внешний синтезатор  
- **Файлы:** `src/vfo/si5351.h`, `src/vfo/si5351.cpp`  
- **Назначение:** поддержка Si5351 по I2C как альтернативный LO/TX.  
- **Статус:** WIP — наличие кода есть; степень интеграции с `vfo`  
  требует уточнения.  
  
---  
  
## 3. Протоколы связи (`src/modes/`)  
  
### 3.0 `modes` — реестр и интерфейс мод  
- **Файлы:** `src/modes/modes.h`, `src/modes/modes_registry.cpp`  
- **Назначение:** `ModeIface { id, name, rsid_code, rx_init, rx_push,  
  tx_frame }`; `modes_get(id)`, `modes_get_by_rsid(code)`.  
- **Статус:** DONE — точка расширения: новая подпапка + строка в  
  `modes_table`.  
  
| Мода | Файлы | RX | TX | RSID | Статус |  
|---|---|---|---|---|---|  
| AX.25 | `ax25/{ax25_mode,ax25_rx,ax25_tx,ax25_hdlc}.cpp` | есть | есть | есть | WIP — полнота HDLC-фрейминга (FCS, bit-stuffing) требует верификации |  
| CW | `cw/{cw_mode,cw_rx,cw_tx}.cpp` | есть | есть | есть | WIP |  
| RTTY | `rtty/{rtty_mode,rtty_rx}.cpp` | есть | **нет** | есть | WIP — `tx_frame` отсутствует |  
| Olivia | `olivia/{olivia_mode,olivia_rx}.cpp` | есть | **нет** | есть | WIP — `tx_frame` отсутствует |  
| Contestia | `contestia/{contestia_mode,contestia_rx}.cpp` | есть | **нет** | есть | WIP — `tx_frame` отсутствует |  
| JTTY | `jtty/{jtty_mode,jtty_rx}.cpp` | есть | **нет** | 1172 | WIP — `tx_frame` отсутствует; цикл RX→TX→RX не тестирован |  
| IFKP | `ifkp/{ifkp_mode,ifkp_rx}.cpp` | есть | **нет** | есть | WIP — `tx_frame` отсутствует |  
  
Общие проблемы мод:  
- `tx_frame` блокирующий — обязательна проверка `TXJ_ABORTED` в  
  посимвольном цикле.  
- RX-only моды формально в реестре, но `tx_frame == nullptr` —  
  поведение `tx_job` при такой моде проверить (должен быть  
  TXJ_ERROR, не зависание).  
  
### 3.1 `src/rsid` — авто-детект моды  
- **Файлы:** `src/rsid/rsid.h`, `src/rsid/rsid.cpp`  
- **Назначение:** RSID-коррелятор (Рид–Соломон по тонам), выбор моды  
  через `modes_get_by_rsid`. Ресемплер Фарроу 15625→11025 встроен в  
  `rx_dsp`, работает постоянно.  
- **Интерфейсы:** `rsid_poll()` (Core 0), `rsid_rx_service_sliced()`  
  (таймер-срез `RSID_SLICE_US=2000` мкс), `rsid_load_permille`.  
- **Статус:** UNTESTED после кванта DSP (c208765): нагрузка упала  
  430‰→74‰; детект на живом слабом сигнале с QSB проверить, эталон —  
  откат на `5b0a017`. Критерий: детект не хуже при `drop=0`, `qfull=0`.  
- **Проблемы:** кормится только в SSB-ветке демодулятора; RSID-TX  
  (передача своего кода) не реализован.  
  
---  
  
## 4. Управление и интерфейсы  
  
### 4.1 `src/cat/cat` — CAT-интерфейс (Kenwood TS-2000 subset)  
- **Файлы:** `src/cat/cat.h`, `src/cat/cat.cpp`  
- **Назначение:** CAT на втором USB-CDC порту (порт из  
  RP2040_receiver/fldigi_cat): FA/FB-частоты, команды trx.  
- **Интерфейсы:** `cat_init()` (до `Serial.begin()`), `cat_poll()`,  
  `cat_set_freq_hz()` — зеркало FA для ответов.  
- **Зависимости:** `trx_control`, `vfo`, USB CDC.  
- **Статус:** DONE.  
- **Проблемы:** `cat_set_freq_hz` — одна из точек с зашитой частотой.  
  
### 4.2 `src/cat/cli` — текстовый CLI  
- **Файлы:** `src/cat/cli.h`, `src/cat/cli.cpp`  
- **Назначение:** командная консоль на CDC: диагностика, настройки,  
  ручные команды trx/vfo.  
- **Статус:** DONE.  
- **Проблемы:** не выявлены.  
  
### 4.3 `src/filesys/file_manager` — MSC-диск и flash-хранилище  
- **Файлы:** `src/filesys/file_manager.{h,cpp}`, `fmgr_stubs.cpp`,  
  `fmgr_compat.h`  
- **Назначение:** RAM-диск FAT12 (566 секторов, первые 54 в RAM —  
  27 КБ `ram_disk_buffer`), виртуальные файлы INFO.TXT / CONFIG.TXT /  
  SET.TXT (по 16 кластеров) и LOG.TXT (512 кластеров, читается из  
  flash @0x140000, 256 КБ); кольцо состояний 8×32 КБ @0x180000  
  (`SlotMeta`: magic/seq/crc32/log_write_ptr).  
- **Интерфейсы:** `file_manager_prepare()`, `init_usb_msc_interface()`,  
  `init_file_manager()`, `check_and_handle_pc_changes()`,  
  `force_reset_to_default_disk()`, `log_file_write_line()`,  
  `file_manager_set_flash_safe_ready()`, `file_manager_core1_init()`;  
  глобальные настройки `my_call_variable`, `my_qth_variable`,  
  `my_freq_rx_var`, `my_demod_var`, `debug_flag`,  
  `pc_file_written`, потолки `vfo_max_*`.  
- **Зависимости:** `core_sync` (flash только при припаркованном  
  Core 1), TinyUSB MSC.  
- **Статус:** DONE.  
- **Проблемы:** RAM-диск 27 КБ — крупнейший потребитель SRAM;  
  `SET.TXT:MAX_CLK_SYS` молча перекрывает кодовый потолок;  
  `pc_file_written` также служит сигналом abort для TX.  
  
---  
  
## 5. Сетевой уровень  
  
### 5.1 `src/modem/` — Hayes-совместимый радиомодем  
- **Файлы:** `modem.{h,cpp}`, `modem_cmd.{h,cpp}`,  
  `modem_data.{h,cpp}`, `modem_cfg.h`  
- **Назначение:** AT-команды по USB CDC, FSM  
  `MODEM_COMMAND/MODEM_DATA/MODEM_ONLINE`, S-регистры; аналог  
  KISS-TNC с текстовым профилем Hayes. Управление — только через  
  `vfo`/`trx_control`/`modes`.  
- **Интерфейсы:** `modem_init()`, `modem_poll()` (Core 0),  
  `modem_rx_byte()`, `modem_cmd_exec(line)`, `modem_answer()`.  
- **Статус:** **STUB** — `modem_cmd_exec` не парсит AT (только  
  `AT`→OK, остальное → `[STUB] not parsed` + OK); `modem_poll`  
  пустой; TODO: `AT+...`, `ATD`, `ATO`, `ATH`, `ATSn?`.  
- **Проблемы:** весь сетевой уровень (соединение, escape +++,  
  S-регистры, DATA-режим) предстоит реализовать; `tx_job` — готовый  
  транспорт для него.  
  
### 5.2 Маяк / BBS / маршрутизация  
- **Файлы:** отсутствуют.  
- **Статус:** не реализовано. Основа: `tx_job` (планировщик маяка),  
  `file_manager` (flash-кольцо и виртуальные файлы — хранилище почты),  
  `modem` (командный слой BBS). Ограничение: SRAM ~94.5% — буферы  
  почты только через `mem_arena` SHARED или flash.  
  
---  
  
## 6. Сводная таблица готовности  
  
| Модуль | Статус | Блокирует |  
|---|---|---|  
| pins | DONE | — |  
| mem_arena | DONE | — |  
| core_sync | DONE | — |  
| trx_control | DONE/UNTESTED | TX под нагрузкой |  
| tx_job | DONE/UNTESTED | modem, маяк, BBS |  
| rx_capture | DONE | — |  
| rx_dsp | DONE | — |  
| rx_decoders | DONE | — |  
| rx_goertzel | DONE | — |  
| rx_pipeline | WIP | — |  
| rx_usb_audio | UNTESTED | мониторинг на ПК |  
| vfo / quad_vfo | DONE | — |  
| soft_tx | DONE | — |  
| si5351 | WIP | внешний тракт |  
| modes (реестр) | DONE | — |  
| AX.25, CW | WIP (RX+TX) | BBS (AX.25) |  
| RTTY, Olivia, Contestia, JTTY, IFKP | WIP (RX only) | полноценный модем |  
| rsid | UNTESTED | авто-детект |  
| cat / cli | DONE | — |  
| file_manager | DONE | — |  
| modem (Hayes) | STUB | сетевой уровень |  
| маяк / BBS | отсутствует | цель проекта |