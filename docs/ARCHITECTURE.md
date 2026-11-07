# ARCHITECTURE.md — RP2040_radiomodem  
  
Полудуплексный радиомодем на RP2040: приём и передача никогда не работают  
одновременно. Документ описывает фактическую архитектуру кода на текущий  
коммит, а не целевую. Где поведение не подтверждено кодом — помечено  
«требует уточнения».  
  
---  
  
## 1. Общая модель
```
┌─────────────────────────────── RP2040 ───────────────────────────────┐
│ │
│ Core 0 (приложение, неблокирующий loop()) │
│ ┌───────────────────────────────────────────────────────────────┐ │
│ │ cat_poll → cli/cat-команды │ │
│ │ tx_job_poll → trx_switch(TX) → ModeIface::tx_frame │ │
│ │ rx_decoders_poll → посимвольные декодеры мод │ │
│ │ rsid_poll → авто-детект моды │ │
│ │ check_and_handle_pc_changes → файлы с ПК (SET.TXT и др.) │ │
│ │ usb_audio_poll → UAC2 хост │ │
│ └───────────────────────────────────────────────────────────────┘ │
│ ▲ spinlock / volatile-флаги ▲ │
│ Core 1 (реальное время, loop1 в RAM, __not_in_flash_func) │
│ ┌───────────────────────────────────────────────────────────────┐ │
│ │ core_sync_core1_poll → парковка при flash-операциях │ │
│ │ trx_poll_core1 → исполнение trx_switch на этом ядре │ │
│ │ if TRX_RX: rx_dsp_poll (квант ≤2 блоков/проход) │ │
│ │ else: vfo_poll (TX-генератор) │ │
│ └───────────────────────────────────────────────────────────────┘ │
│ │
│ Железо: ADC(GP26/GP27)+DMA → I/Q │ PIO0 soft_tx │ PLL quad_vfo LO │
│ USB: TinyUSB композит CDC + MSC + UAC2 │
└──────────────────────────────────────────────────────────────────────┘
```
Инварианты:  
  
- RX и TX взаимоисключающи; единственная точка переключения — `trx_switch()`  
  (`src/trx/trx_control.h`). Все пути (CAT-команды, планировщик, тесты)  
  идут через неё.  
- Весь код Core 1 — `__not_in_flash_func`, без `Serial`, без блокировок,  
  без malloc.  
- Память RX- и TX-фаз — одна общая область SRAM (`mem_arena`), фазы  
  инвалидируются при переключении.  
  
---  
  
## 2. Распределение задач между ядрами  
  
### Core 0 — `loop()` (`RP2040_radiomodem.cpp`)  
  
| Задача | Функция | Назначение |  
|---|---|---|  
| CAT/CLI | `cat_poll()` | разбор команд с USB-CDC, ответы, команды trx |  
| Передачи | `tx_job_poll()` | очередь TXJ_QUEUED→ACTIVE→DONE, вызов `trx_switch(TX)` |  
| Декодеры | `rx_decoders_poll()` | посимвольная обработка демодулированного потока |  
| RSID | `rsid_poll()` | сервис авто-детекта моды |  
| Файлы | `check_and_handle_pc_changes()` | приём файлов, записанных с ПК через MSC |  
| Аудио | `usb_audio_poll()` | откачка PCM в UAC2 |  
  
### Core 1 — `loop1()` (`RP2040_radiomodem.cpp`, `__not_in_flash_func`)  
  
Порядок фиксирован:  
  
1. `core_sync_core1_poll()` — обработка запросов парковки (flash-erase/write  
   требует остановки Core 1, т.к. код исполняется из XIP).  
2. `trx_poll_core1()` — фактическое выполнение переключения RX↔TX  
   (stop/start трактов происходит на этом ядре).  
3. `rx_dsp_poll()` при `TRX_RX` — DDC, децимация, демодуляция; квант  
   `DSP_MAX_BLOCKS_PER_POLL = 2` блока за проход, чтобы не задерживать  
   пункты 1–2 (~6 мс худший случай до квантования).  
4. `vfo_poll()` при `TRX_TX` — обслуживание TX-генератора.  
  
Сторож PRIMASK: при старте `loop1` проверяет и сбрасывает залипший  
`primask` (историческая гонка `s_pll_lock_irq`; после per-core фикса в  
`core_sync.cpp` срабатывать не должен).  
  
### Межъядерное взаимодействие  
  
- `core_sync` — протокол парковки Core 1 для flash-операций и смены PLL.  
  Критично: save/try_lock/unlock_unsafe/restore PRIMASK — строго per-core;  
  межъядерное смешивание сохранённых PRIMASK было первопричиной зависаний.  
  `multicore_lockout_victim_init()` НЕ использовать — конфликт за  
  `SIO_IRQ_PROC1`.  
- `trx_control` — Core 0 ставит запрос, Core 1 исполняет; состояние  
  `trx_state()` читается обоими ядрами.  
- SPSC-очередь блоков I/Q: producer — DMA ISR/`rx_capture` (Core 1),  
  consumer — `rx_dsp_poll` (Core 1); демодулированные сэмплы далее  
  передаются на Core 0 через очередь декодеров (механизм — см.  
  `src/rx/rx_decoders.*`, требует уточнения по деталям).  
  
---  
  
## 3. Переключение RX/TX — `trx_switch()`  
  
Файл: `src/trx/trx_control.h/.cpp`. Порядок фиксирован и выполняется  
на Core 1:
trx_switch(новое_направление, freq_hz)

stop активного тракта (rx_capture_stop / tx stop)
mem_arena_switch(новая фаза) — таблицы ушедшей фазы мертвы
перенастройка VFO (TrxVfoHook → vfo_on_trx_switch)
start нового тракта (rx_dsp_start + rx_capture_start
или TX-генератор)
Состояния: `TRX_RX`, `TRX_TX`, промежуточное переключение. Точка  
расширения — `TrxVfoHook`: подсистема VFO подписывается на смену фазы.  
  
Известные ограничения:  
  
- Цикл RX→TX→RX при активной JTTY (перевыделение арены) не тестирован.  
- Частота приёма зашита в нескольких точках скетча (`trx_switch` возврата,  
  `vfo_on_trx_switch`, `cat_set_freq_hz`, `rx_dsp_start`) — при смене  
  править все.  
- `tx_pipeline_start/stop` — weak-заглушки; реальный TX-тракт моды  
  обслуживает сама через `tx_frame` + `vfo_poll`.  
  
---  
  
## 4. Потоки данных  
  
### RX-конвейер
Антенна → ADC GP26/GP27 (I/Q)
→ DMA (пинг-понг, IRQ0, ~488 cap_isr/с при Fs=250k, блок 512 пар)
→ SPSC-очередь блоков
→ rx_dsp_poll (Core 1): NCO = f_rx − vfo_quad_lo_hz(),
децимация, демодулятор (RXD_DEMOD_USB / RAW)
→ демодулированные сэмплы
├→ rx_decoders_poll (Core 0) → ModeIface::rx_push(sample)
│ → посимвольный декодер активной моды → текст/кадры
├→ rsid_rx_service_sliced (таймер-срез RSID_SLICE_US=2000 мкс)
└→ rx_usb_audio → usb_audio_poll → UAC2 → ПК

Счётчики здоровья: `cap_isr`, `drop`, `fcsover`, `qfull`, `dead` —  
норма `drop=0`, `fcsover=0`. Нагрузка на 373 МГц: `dsp≈390‰`,  
`rsid≈74‰`, `total≈465‰` — запас ~50%.  
  
### TX-конвейер
Источник (CAT/CLI, tx_job, тест, будущий modem/BBS)
→ tx_job_submit(mode, freq, payload) [TXJ_QUEUED]
→ tx_job_poll (Core 0): trx_switch(TRX_TX, freq)
→ ModeIface::tx_frame(payload, len) — блокирующая посимвольная
модуляция; в цикле обязана проверять tx_job_state()==TXJ_ABORTED
→ тоны/символы → vfo_bank / soft_tx (PIO0) / quad_vfo LO
→ trx_switch(TRX_RX, f_rx) — возврат в приём

`tx_frame` возвращает длительность/код; abort — через `tx_job_abort()`  
или `pc_file_written`. Аппаратного PTT-выхода нет — только  
`PIN_PTT_LED` (GP25).  
  
### VFO / синтезаторы  
  
- `quad_vfo` — квадратурный LO на системной PLL: сканер fbdiv/postdiv,  
  LO до ~45 МГц, измерение реальной частоты через FC0. Рабочая точка  
  разгона: clk_sys 373.3 МГц (fbdiv=280, postdiv 3/3, VCO 3360 МГц —  
  вне спека, на конкретном чипе стабильно). Потолок в спеке: 380 МГц  
  (`pll_ceiling`, `vfo_max_clk_sys_hz`), VREG 1.30V; `SET.TXT:MAX_CLK_SYS`  
  перекрывает кодовый потолок.  
- `soft_tx` — PIO0-генератор тонов для AFSK/FSK-мод.  
- `vfo_bank` — сетка тонов (`vfo_bank_set_grid(f0, step, n)`,  
  `vfo_tone(i)`).  
- `si5351` — внешний синтезатор (опциональный тракт).  
  
---  
  
## 5. Память  
  
### SRAM — `mem_arena` (`src/mem/mem_arena.h`)  
  
Фазовый менеджер без malloc:  
  
| Фаза | Область | Содержимое | Время жизни |  
|---|---|---|---|  
| RX | TRX-пул (36 КБ) | FFT/Гёрцель-банки, метрики Витерби, DSP-таблицы | до trx_switch(TX) |  
| TX | тот же TRX-пул | TX-таблицы моды | до trx_switch(RX) |  
| SHARED | отдельные 8 КБ | настройки, расписание, CAT-буферы | никогда не очищается |  
  
RX и TX занимают одну область (union-семантика): платим max(RX,TX)  
вместо RX+TX. `mem_arena_switch()` инвалидирует фазу одной операцией —  
без фрагментации.  
  
Бюджет: SRAM занято ~94.5%, включая `ram_disk_buffer` 27 КБ (RAM-диск  
для MSC). Рост .bss уронит линковку или стек. Резервы: ужать  
`LOG_CLUSTERS` или `MEM_TRX_POOL_SIZE`; `arena watermark: 0/36864`  
намекает, что TRX-пул в RX может быть недоиспользован (проверить  
watermark).  
  
### Flash-карта (2 МБ)
0x000000–0x140000 прошивка
0x140000 LOG.TXT (виртуальный файл для MSC)
0x180000–0x200000 кольцо состояний: 8 слотов

- Flash-операции только через парковку Core 1 (`core_sync`) — XIP-off.  
- `file_manager` + `fmgr_stubs` — виртуальные файлы для MSC:  
  LOG.TXT (лог), SET.TXT (настройки, в т.ч. `MAX_CLK_SYS`).  
- `check_and_handle_pc_changes()` (Core 0) — детект записи файла с ПК  
  (`pc_file_written`) и применение изменений; также служит сигналом  
  abort для TX.  
  
---  
  
## 6. USB-композит (TinyUSB)  
  
Три интерфейса на одном USB:  
  
- **CDC** — консоль: CAT-команды (`cat`), CLI (`cli`), диагностический  
  вывод, пульс-строка нагрузки.  
- **MSC** — виртуальный диск: LOG.TXT, SET.TXT; запись с ПК →  
  `pc_file_written` → обработка на Core 0.  
- **UAC2** — аудио-приёмник: демодулированный PCM на хост.  
  `aud_fill`/`aud_drop` — счётчики; на хосте ещё не проверено.  
  
Порядок инициализации TinyUSB критичен (см. `setup()` в  
`RP2040_radiomodem.cpp` и AGENTS.md — дескрипторы и старт стека  
до начала захвата).  
  
---  
  
## 7. Состояния системы  
  
```mermaid  
stateDiagram-v2  
    [*] --> INIT: setup()  
    INIT --> RX: core1 PLL ready, rx_dsp_start, rx_capture_start  
    RX --> SWITCHING: trx_switch(TRX_TX)  
    SWITCHING --> TX: stop RX → arena_switch → VFO → start TX  
    TX --> SWITCHING: trx_switch(TRX_RX) / tx_job done / abort  
    SWITCHING --> RX: stop TX → arena_switch → VFO → start RX  
    RX --> RX: rx_dsp_poll (квант 2 блока), decoders, RSID  
    TX --> TX: vfo_poll, tx_frame посимвольно
Параллельно (любое состояние): core_sync может припарковать Core 1
для flash/PLL; CAT-команды обрабатываются на Core 0.

## 8. Подсистема мод — ModeIface

src/modes/modes.h:

struct ModeIface {  
  ModeId id;  
  const char *name;  
  uint16_t rsid_code;   // 0 = нет авто-детекта  
  void     (*rx_init)(uint32_t center_hz);  
  void     (*rx_push)(int16_t sample);  
  uint32_t (*tx_frame)(const uint8_t *payload, uint16_t len);  
};
Реестр (src/modes/modes_registry.cpp): AX.25, CW, RTTY, Olivia,
Contestia, JTTY, IFKP. Добавление моды = новая подпапка src/modes/<moda>/

строка в modes_table. Выбор активной моды — CAT-командой или
авто-детектом по rsid_code (modes_get_by_rsid).
RSID: коррелятор src/rsid/ работает по таймер-срезам
(rsid_rx_service_sliced, 2000 мкс), кормится только в SSB-ветке
демодулятора. После введения кванта DSP (коммит c208765) детект на
живом сигнале требует проверки — эталон для сравнения 5b0a017.

## 9. Сетевой уровень (текущее состояние)

src/modem/ — Hayes-модем: FSM COMMAND/DATA/ONLINE объявлена,
modem_cmd_exec — stub (не парсит AT), modem_poll пустой.
tx_job (src/trx/tx_job.h) — готовая очередь передач:
TXJ_IDLE→QUEUED→ACTIVE→DONE/ABORTED/ERROR, payload
TX_JOB_PAYLOAD_MAX, abort через tx_job_abort().
Маяк, почта, BBS, маршрутизация — не реализованы. Основа для
хранилища: flash-кольцо слотов и виртуальные файлы file_manager.

## 10. Правила для разработчиков (инварианты, нарушать нельзя)

Core 1: только __not_in_flash_func, без Serial, без ожиданий,
без malloc. Serial.println в rx_capture_start — известный
допустимый только до входа в горячий цикл долг.
Любое переключение RX/TX — только trx_switch().
Flash-erase/write и смена PLL — только через core_sync-парковку;
PRIMASK save/restore строго per-core.
rx_dsp_poll — не более DSP_MAX_BLOCKS_PER_POLL за проход.
tx_frame моды — проверять TXJ_ABORTED в посимвольном цикле.
Новые большие буферы — через mem_arena (фаза RX/TX/SHARED),
не через .bss: SRAM ~94.5% занято.
SET.TXT:MAX_CLK_SYS перекрывает кодовый потолок разгона — при
смене потолка править и файл на диске.

## 11. Диагностика

Пульс-строка: dsp, rsid, total (‰), cap_isr, drop,
fcsover, qfull, dead, aud_fill, aud_drop, clk_sys
(реальная, из vfo_clk_sys_hz), arena watermark.
[LODBG] — vfo_quad_debug(): clkdiv/enabled/lo_hz.
Дубли [RX] dsp init, [DEC] init, [MSC] registering в логе —
двойные вызовы init, не критично.