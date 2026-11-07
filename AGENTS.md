# AGENTS.md — RP2040_radiomodem  
  
Полудуплексный цифровой радиомодем на RP2040. Наследие двух проектов:  
приёмный тракт из RP2040_receiver (ADC+DMA → DDC → децимация → демодуляция),  
передающий из RP2040_beacon (модулятор → дизеринг-DDS на PIO).  
Тракты физически раздельны и никогда не работают одновременно.  
  
## Архитектура  
  
- **Core 0 (приложение)**: CAT/CLI, декодеры + RSID, планировщик, Hayes-модем,  
  file_manager (LittleFS + MSC), датчики, LCD, LED.  
- **Core 1 (реальное время)**: либо RX-конвейер, либо TX-генератор — ядро  
  арендуется через `trx_control`. Плюс `vfo_poll()` (MASH-дизеринг в TX).  
- **Парковка ядер** (`src/trx/core_sync.cpp`): единственный владелец протокола.  
  С Core 1 — `core_sync_park_core0()`, с Core 0 — `core_sync_park_core1()`.  
  IRQ сохраняются/отключаются ДО ACK и восстанавливаются после выхода —  
  исключён уход в XIP через прерывание во время flash-окон и PLL-перестроек.  
- **Захват** (`src/rx/rx_capture.cpp`): ADC0=I (GP26), ADC1=Q (GP27),  
  round-robin, 16-бит переносы в 32-бит слова [15:0]=I, [31:16]=Q.  
  Один DMA-канал + программный пинг-понг буферов в DMA IRQ0 на Core 1,  
  публикация блоков в SPSC-очередь. Буферы статические (DSP-путь живёт  
  до `mem_arena_switch`, пул сбрасывается при `trx_switch`).  
  
## КРИТИЧНО: гонка s_pll_lock_irq (корень зависаний на разгоне)  
  
`core_sync_pll_try_lock()`/`core_sync_pll_unlock()` используют  
`s_pll_lock_irq[2]` — **per-core** массив сохранённого PRIMASK.  
  
НЕ делать его общим `static uint32_t`. Старая схема с одной переменной:  
Core 1 сохранял свой saved_irq → Core 0 перезаписывал до unlock Core 1 →  
Core 1 восстанавливал чужое значение → `primask` залипал в 1 навсегда →  
DMA IRQ0 на Core 1 умирал → система зависала.  
  
Симптомы были обманчивы: зависание проявлялось только при двух DMA-каналах  
или `chain_to`, что указывало на DMA. Реальная причина — мёртвый IRQ.  
Диагностика: читать `primask` на Core 1 (`mrs %0, primask`), `nvic_hw->ispr`,  
`dma_hw->ch[N].transfer_count`. В `loop1` оставлен одноразовый сторож  
primask (`cpsie i` при залипании) — не удалять.  
  
## Проверенные решения (не откатывать)  
  
1. **Одноканальный DMA + программный пинг-понг в IRQ** — рабочая схема.  
   `chain_to` и два канала НЕ нужны: функционально эквивалентно, проще,  
   и двухканальная схема маскировала настоящий баг. Не возвращать.  
2. **`s_pll_lock_irq[2]` per-core** в `core_sync.cpp` — обязательно.  
3. **NULL-guard'ы** в `core_sync_pll_try_lock`/`unlock` (`if (!s_pll_lock)`).  
4. **`__not_in_flash_func`** на `loop1`, `rx_dsp_poll`, `trx_poll_core1`,  
   `rx_capture_start`, `rx_dma_irq`, всём пути парковки и PLL — обязательно:  
   во время flash-окон и PLL-перестроек XIP мёртв.  
5. **`irq_set_exclusive_handler` + `irq_set_enabled(DMA_IRQ_0)`** вызываются  
   на Core 1 (в `rx_capture_start`, есть `assert(get_core_num()==1)`) —  
   NVIC и таблица векторов у каждого ядра свои.  
  
## Разгон (провено стабильно)  
  
- Номинал 133 МГц → рабочая точка **373.3 МГц** (скан quad_vfo выбрал  
  fbdiv=280, postdiv 3/3, VCO 3360 МГц — формально вне спека PLL 750–1600 МГц,  
  работает на данном чипе; на другом экземпляре может не взять).  
- Потолок: `pll_ceiling = {95, 3, 1, 380000000}` (VCO 1140 МГц, в спеке),  
  `vfo_max_clk_sys_hz = 380000000`, `vfo_max_pll_vco_hz = 4200000000`,  
  VREG 1.30V. SET.TXT `MAX_CLK_SYS` на диске перекрывает кодовый потолок —  
  при смене потолка править и файл на диске.  
- Нагрузка на 373 МГц: `dsp=347‰`, `total=705‰` — запас ~30%.  
- `clk_sys` в статус-строке показывает кэш 133 МГц — косметика, не баг.  
  
## Тупиковые пути (проверено — не повторять)  
  
- Два DMA-канала + `chain_to` A↔B — не причина зависаний, но и не нужны.  
- Перенос DMA IRQ на Core 0 — не требуется, IRQ на Core 1 работает.  
- `__get_PRIMASK`/`__enable_irq` — CMSIS-интринсики недоступны в тулчейне,  
  использовать `__asm volatile ("mrs %0, primask" ...)` / `cpsie i`.  
- `nvic_hw->iser` — одиночный регистр, не массив (без `[0]`).  
  
## Известные хвосты  
  
- `dbg_l1`/`dbg_poll_in`/`dbg_poll_out` в `.ino` — мёртвые счётчики,  
  нигде не инкрементятся (l1/pin/pout в sync-строке всегда 0 — это норма).  
- Маркеры `m1`-`m11` и `Serial` в `rx_capture_start` — отладочные, чистить.  
- `dma_overclock_test` — мёртвый код.  
- SET.TXT `MAX_CLK_SYS` молча перекрывает потолок с диска — источник  
  «не применяется разгон».


## КРИТИЧНО: USB-стек и запись во flash (MSC-диск)  
  
### Порядок инициализации USB — строго  
  
`TinyUSBDevice.begin()` вызывается ТОЛЬКО один раз и ПОСЛЕДНИМ — неявно  
из `Serial.begin()`. Все `*.begin()` интерфейсов (`usb_msc.begin()`,  
`usb_out.begin()`) — это регистрация в списке интерфейсов Adafruit  
TinyUSB, они обязаны отработать ДО старта стека. Явный  
`TinyUSBDevice.begin(0)` внутри `usb_audio_init()` стартовал стек до  
регистрации UAC → композит ломался, MSC write/flush-колбэки до  
устройства не доходили (диск виден, читается, но записи «не было»).  
Порядок в setup(): init_usb_msc_interface() → init_file_manager() →  
usb_audio_init() → Serial.begin(115200).  
  
### flash_safe_execute / multicore_lockout — НЕ использовать  
  
`flash_safe_execute_core_init()` на Core 1 вызывает зависание при  
стартовом `vfo_on_trx_switch` (lockout-IRQ конфликтует с  
core0_park-протоколом). Проверено дважды. Запись во flash — только  
прямая: `core_sync_park_core1()` (флаг-протокол, core1_park_poll первой  
строкой loop1) → `save_and_disable_interrupts()` → `flash_range_erase/  
program` → `flash_flush_cache()` → unpark. Все функции пути —  
`__not_in_flash_func`.  
  
### Core 1 никогда не блокируется  
  
Никаких Serial.printf/delay/mutex/multicore_fifo на Core 1 — ни в init,  
ни в DSP, ни в VFO. Core 0 регулярно паркуется в RAM-ожидании  
(core0_park) и не обслуживает USB FIFO → любой printf с Core 1 висит на  
полном буфере → deadlock. Диагностика с Core 1 — только volatile-поля,  
печатает Core 0. Это была причина минимум трёх «зависаний» с разными  
симптомами.  
  
### MSC write/flush семантика  
  
- `msc_flush_cb` НЕ обновляет `last_msc_write_time` — Windows шлёт  
  SYNCHRONIZE CACHE периодически, иначе 1500-мс окно тишины никогда не  
  наступит и save_state не вызовется.  
- Гейт `pc_written_regions == 0` обязателен: пустые flush без записей не  
  должны гонять erase+program 32 КБ (износ flash).  
- LOG-регион (lba >= RAM_SECTORS) возвращает `bufsize` (фейковый успех),  
  не -1: Windows на ошибке сектора рвёт всю транзакцию копирования и  
  ретраит бесконечно.  
- `file_manager_set_flash_safe_ready(true)` — в setup() после  
  `while(!core1_pll_done)`, без него flash_write_allowed() молча  
  блокирует save_state.  
  
## CAT/CLI — общий CDC-порт  
  
- `cat_poll()` (`src/cat/cat.cpp`) — единственный читатель `Serial`.  
  Демультиплексор: байты до `;` → CAT-буфер → `cat_execute()`;  
  строка до `\n` с непустым буфером → `cli_execute()` (`src/cat/cli.cpp`).  
  `\r` игнорируется. CAT-команды без `;` уйдут в CLI — это осознанно.  
- CLI — порт `check_serial_commands()` из RP2040_beacon: help, status,  
  setparam, rx/quad/vfo/mem/limits, clear log, format disk, remount, reset,  
  tx. Команды без аналога — заглушки "[CLI] '<cmd>' не поддерживается".  
- Демодулятор задаётся `uint8_t` + макросы `RXD_DEMOD_*` (rx_dsp.h),  
  enum `DemodMode` НЕ существует — не использовать.

  ## КРИТИЧНО: цепочка записи Windows → flash  
  
Windows → `msc_write_cb` (пишет в ram_disk_buffer, ставит  
`pc_activity_detected`, `pc_written_regions`, `last_msc_write_time`)  
→ `check_and_handle_pc_changes()` в `loop()` (ждёт 1.5 с тишины,  
проверяет `flash_write_allowed()` и `trx_state()==TRX_RX`)  
→ `save_state()` (весь RAM-диск → ring-слот, SlotMeta+CRC32)  
→ парсеры по битам регионов (0x02 INFO / 0x04 CONFIG / 0x08 SET)  
→ `remount_usb_disk()`.  
  
Обязательные условия, без которых цепочка молча обрывается:  
1. `file_manager_core1_init()` — ПЕРВОЙ строкой `setup1()`, до старта  
   DSP: `flash_safe_execute_core_init()` + `flash_safe_ready=true`.  
   Без него `flash_write_allowed()` всегда false — записи нет, ошибок нет.  
2. `check_and_handle_pc_changes()` — каждый проход `loop()` на Core 0.  
3. `msc_flush_cb` НЕ должен обновлять `last_msc_write_time` — Windows  
   шлёт SYNCHRONIZE CACHE постоянно, иначе окно 1500 мс не наступит  
   никогда. Flush ставит только `pc_activity_detected`.  
4. LOG.TXT (lba >= RAM_SECTORS) — read-only для ПК, пишется только  
   изнутри через `log_append()`.

## TX по команде из COM-порта  
  
- `src/trx/tx_job.{h,cpp}` — заявка: `tx_job_submit(mode, freq, payload)`  
  ставит, `tx_job_poll()` в `loop()` исполняет:  
  QUEUED → `trx_switch(TRX_TX)` → ACTIVE → `mi->tx_frame()` (блокирующий,  
  как `send_*_string` в маяке) → гарантированный `trx_switch(TRX_RX)`.  
- Abort: `pc_file_written` или `tx_job_abort()` → TXJ_ABORTED.  
  Для реального прерывания в посимвольном цикле моды нужна проверка  
  `tx_job_state()==TXJ_ABORTED` (аналог `mfsk_abort()` маяка).  
- `tx_frame` принимает `const uint8_t*` — каст от `char*` обязателен.  
- PTT-выхода нет — только `PIN_PTT_LED` (GP25).

## Тупиковые пути (дополнение)  
  
- `file_manager_set_flash_safe_ready(true)` снаружи — не нужен и не  
  сработает до `flash_safe_execute_core_init()`; флаг выставляет сам  
  `file_manager_core1_init()` на Core 1.  
- Запись flash до инициализации парковки Core 1 — зависание в  
  `flash_safe_execute`. Отложенная запись через `state_save_pending`.  
- `core0_park_poll()` в `while(!core1_pll_done)` — обязателен: без него  
  парк-запрос от `vfo_on_trx_switch()` не обслуживается → PARK TIMEOUT.  
  Требует `extern void core0_park_poll(void);` в .ino.  
  
## Известные хвосты (дополнение)  
  
- `arena watermark: 0/36864` — TRX-пул в RX не используется (или  
  watermark не обновляется). Если реальный ноль — пул можно урезать,  
  heap сейчас ~9 КБ свободных.  
- `aud_fill=0..1` — PCM-буфер UAC почти пуст; при щелчках в звуке  
  смотреть частоту `usb_audio_poll()`.  
- Дубли в логе (`[RX] dsp init`, `[DEC] init`, `[MSC] registering`) —  
  двойные вызовы init, не критично.  
- PLL-расхождение: clk_sys 373501000 вместо 373509337, LO err −355 Hz.

## Файловая система (src/filesys/file_manager.cpp)  
  
- **Диск**: FAT12, 566 секторов (~283 КБ), кластер = 1 сектор.  
  RAM-буфер только первые 54 сектора (27 КБ): boot/FAT/каталог +  
  INFO.TXT (LBA 6-21), CONFIG.TXT (22-37), SET.TXT (38-53).  
  LBA ≥ 54 — виртуальный LOG.TXT, читается напрямую из XIP по  
  `LOG_FLASH_OFFSET = 0x140000` (256 КБ). Запись в LOG-сектора с ПК  
  молча отбрасывается (read-only).  
- **Кольцо состояний**: 8 слотов по 32 КБ @ `RING_OFFSET = 0x180000`.  
  Слот: INFO 8КБ @0, CONFIG @0x2000, SET @0x4000, RESERVED @0x6000,  
  SlotMeta @0x7E00 (magic, seq, crc32, log_write_ptr). Выбор слота —  
  max seq с валидными magic+CRC, сравнение seq wrap-aware.  
- **Flash-карта**: прошивка < 0x140000 (сейчас ~174 КБ — запас большой,  
  проверять по firmware.map при росте).  
  
## КРИТИЧНО: flash-операции и USB — проверенные кирпичи  
  
1. **В boot flash не стирается.** `init_file_manager()` только читает  
   XIP и строит RAM-диск. Первая запись — через `state_save_pending`  
   из `check_and_handle_pc_changes()` при `trx_state()==TRX_RX`,  
   когда система полностью жива. Erase в setup() при работающем  
   Core 1 = кирпич (XIP мёртв во время erase/program).  
2. **Все flash-операции — только `flash_safe_execute()`** (обёртки  
   `safe_flash_erase_4k`/`safe_flash_program`). Требуется  
   `flash_safe_execute_core_init()` первой строкой в `setup1()` —  
   без него запись молча фейлит по таймауту.  
   Голый `save_and_disable_interrupts()` НЕ паркует Core 1.  
3. **`multicore_lockout` — тупиковый путь**: без  
   `multicore_lockout_victim_init()` на Core 1 `lockout_start_blocking()`  
   висит навсегда. Не использовать — есть `flash_safe_execute`.  
4. **`usb_msc.begin()` — ровно один раз**, внутри `init_file_manager()`,  
   ПОСЛЕ `usb_audio_init()` и `cat_init()`. Двойной begin() или  
   регистрация UAC после старта стека ломает энумерацию всего  
   композита (CDC+MSC+UAC) — симптом «USB-устройство не опознано».  
5. **`while(!core1_pll_done)` — с таймаутом 10 с.** Без таймаута  
   зависание `vfo_on_trx_switch()` на Core 1 = вечный кирпич без  
   диагностики. С таймаутом — `[ERR]` в Serial и живой USB.  
6. **`remount_usb_disk()` защищён `usb_started`** — `setUnitReady()`  
   до `usb_msc.begin()` не вызывать.  
  
## SET.TXT → переменные разгона  
  
- Парсер пишет в `vfo_max_clk_sys_hz` / `vfo_max_pll_vco_hz`  
  (weak-дефолты в `src/vfo/vfo.cpp:24-25`, extern в `vfo.h:83-84`)  
  и `vfo_xosc_corr_ppb` (`vfo.cpp:20`, extern добавлен в file_manager.h).  
- Значения применяются в `quad_fill_cfg()` при `vfo_init()` — правка  
  SET.TXT с ПК вступает в силу только после ребута (переинициализации  
  VFO нет).  
- ВНИМАНИЕ: `clk_sys_min_hz` в `quad_fill_cfg()` должен быть ≤  
  `vfo_max_clk_sys_hz`. Пустой диапазон (min 240 МГц > max 150 МГц)  
  → PLL-сканер не находит конфигурацию → `vfo_on_trx_switch()` не  
  возвращается → зависание в `while(!core1_pll_done)`.  
  
## Известные хвосты (file_manager)  
  
- `pc_written_regions` маски: 0x01=sys, 0x02=INFO, 0x04=CONFIG, 0x08=SET.  
- `log_append()` пишет полными страницами 256 Б (требование  
  flash_range_program), `log_write_ptr` всегда кратен 256.  
- `log_boot_recover_ptr()` сканирует по 256-Б страницам — первый  
  0xFF-байт = конец лога.  
- RAM ~97% — `ram_disk_buffer` 27 КБ; рост .bss уронит линковку  
  (location counter backwards) или стек.

  ## file_manager: USB-MSC диск + кольцо состояний во flash  
  
- **Диск — эмуляция, не ФС.** `msc_read_cb`/`msc_write_cb` обслуживают LBA;  
  RAM-буфер покрывает только служебные сектора + INFO/CONFIG/SET.  
  LOG.TXT (256 КБ) виртуальный — читается хостом напрямую из XIP-flash,  
  RAM не занимает. Запись в его LBA молча проглатывается + атрибут 0x21.  
- **Геометрия**: FAT12, кластер=1 сектор, 566 секторов: boot(0), FAT(1-2),  
  каталог(3-5), INFO(6-21), CONFIG(22-37), SET(38-53), LOG(54-565).  
  RAM-буфер = 54 сектора (27 КБ).  
- **Кольцо состояний**: 8 слотов × 32 КБ @ 0x180000-0x200000.  
  Слот = INFO 8КБ + CONFIG 8КБ + SET 8КБ + RESERVED 7.5КБ + SlotMeta 512Б  
  (magic, seq, CRC, log_write_ptr). Запись = новый слот seq+1, boot берёт  
  max seq. Кольцо — для износа flash, не для отката.  
- **LOG-кольцо** @ 0x140000-0x180000: append 256-Б страницами, erase 4-КБ  
  блоками по кругу. Указатель в SlotMeta + boot-восстановление сканом  
  первого 0xFF-блока. После порции записи — remount_usb_disk()  
  (Windows кэширует FAT и данные).  
- **SET.TXT** — файл скрытых данных (scan-таблица, MAX_CLK_SYS, MAX_PLL_VCO,  
  XOSC_PPB). Виден на время отладки, потом скрыть из каталога.  
  CONFIG.TXT — бывший SET.TXT (переименован).  
  
## КРИТИЧНО: порядок USB-инициализации (корень «USB-устройство не опознано»)  
  
`usb_audio_init()` внутри вызывает `TinyUSBDevice.begin(0)` — это МАУНТ стека.  
Все интерфейсы обязаны регистрироваться ДО него:  
  
    init_usb_msc_interface();   // MSC — ПЕРВЫМ  
    usb_audio_init();           // UAC (внутри TinyUSBDevice.begin)  
    Serial.begin(115200);       // CDC — последним  
  
Обратный порядок (MSC после UAC) = дескрипторы меняются на лету в окне  
энумерации → «USB-устройство не опознано». Симптом обманчив: выглядит  
как битый кабель/драйвер, реально — гонка дескрипторов.  
  
## КРИТИЧНО: flash-запись и Core 1  
  
- `save_ram_to_flash()`/`save_state()`/`log_append()` — только при  
  `trx_state()==TRX_RX` и только с парковкой Core 1 через  
  `core_sync_park_core1()` (НЕ `rp2040.idleOtherCore()` — он не  
  согласован с нашим протоколом и вешал систему).  
- `init_file_manager()` вызывать в setup() ДО `core0_ready = true`:  
  в этот момент Core 1 стоит в `while(!core0_ready)` и erase безопасен.  
  Перенос вызова после `core1_pll_done` + idleOtherCore — проверенный  
  путь к зависанию (коммит 7687b78, откачен).  
- `check_and_handle_pc_changes()` в loop(): 1.5 с тишины после последнего  
  msc_write_cb → перепарсинг → один save_state() на транзакцию.  
  
## Тупиковые пути file_manager (проверено — не повторять)  
  
- `extern bool clk_boosted` без `volatile` — конфликт с soft_tx.h,  
  volatile часть типа.  
- Дубли определений pin_freq_out/subband_pins/vfo_max_*/vfo_xosc_corr_ppb  
  в file_manager.cpp — владельцы: pins.cpp, soft_tx.cpp, vfo.cpp.  
  В file_manager только extern через заголовки.  
- `"строка" + BCN_VER` — BCN_VER макрос-литерал, нужен String(BCN_VER).  
- rtc_* переменные и get_current_date/time — в радиомодеме нет RTC,  
  живут как weak-заглушки в fmgr_stubs.cpp.  
- `idleOtherCore()` вокруг flash-окон — заменён на core_sync_park_core1().

## Проверенные решения (дополнение)  
6. **mem_arena: TRX pool = 48 КБ** (`MEM_TRX_POOL_SIZE` в `src/mem/mem_arena.cpp`).  
   96 КБ не линковались (`.bss` уходил за SCRATCH_X → `cannot move location  
   counter backwards`). JTTY-блок = 29120 Б, watermark подтверждён на железе:  
   `29120 / 49152`. SHARED pool 8 КБ пока пуст.  
7. **`rx_pipeline_set_freq(freq)` перед `rx_pipeline_start()`** в  
   `trx_poll_core1()` — без этого `s_freq_hz` остаётся 0 и на возврате  
   TX→RX получаем `[RX] start f=0`, NCO уезжает на -14 МГц.  
8. **RSID кормится только в SSB-ветке демодулятора**: `rsid_resampler_push(m)`  
   стоит внутри `else` (не RAW). При `RXD_DEMOD_RAW` (дефолт) RSID молчит —  
   нужен `rx_dsp_set_demod(RXD_DEMOD_USB)` после `rx_dsp_start()`.  
9. **`rsid_poll()` обязателен в `loop()`** — без него детект не печатается  
   и не применяется (`rx_decoders_on_rsid` не вызывается).  
10. **MSC-диск: `usb_msc.begin()` строго до маунта TinyUSB** — первым  
    маунтит `TinyUSBDevice.begin(0)` внутри `usb_audio_init()`, поэтому  
    порядок в `setup()`: `init_usb_msc_interface()` → `usb_audio_init()` →  
    `Serial.begin()`. После `begin()` обязателен `usb_msc.setUnitReady(true)`  
    — с `false` хост видит устройство без тома.  
11. **Flash-save только с парковкой Core 1**: `rp2040.idleOtherCore()` /  
    `resumeOtherCore()` вокруг `flash_range_erase/program` + запрет  
    прерываний. Без парковки Core 1 исполняет XIP при мёртвом flash →  
    полное отваливание USB через несколько секунд (симптом: LED замирает).

## Тупиковые пути (дополнение)  
- Уменьшение RAM-диска «только размером» невозможно: геометрия FAT12  
  (кластерные константы, BPB, цепочки FAT) пересчитывается целиком.  
  Текущая разметка 20 КБ: INFO 8 + SET 8 + LOG 16 кластеров.  
- `core0_park_done` — переменной не существует, парковку из file_manager  
  делать через `rp2040.idleOtherCore()`, не через флаги trx_control.  
- Повторный `usb_msc.begin()`/`init_usb_msc_interface()` после маунта —  
  вешает USB целиком (COM+UAC+MSC пропадают).

  ## Известные хвосты (дополнение) 
  - RAM: 247648/262144 (94.5%) — свободно ~14.5 КБ. При нехватке: ужать  
  LOG_CLUSTERS или MEM_TRX_POOL_SIZE до 40K.  
- `cap_isr` — счётчик DMA IRQ0 (~488/с при Fs=250k, блок 512 пар), не ошибки.  
- `clk_sys=133000000` в статус-строке — кэш, реальная шина ~373 МГц.  
- `aud_fill=0`/`aud_drop=0` — UAC-аудио на хосте ещё не проверено.  
- Цикл RX→TX→RX при активной JTTY (перевыделение арены) не тестирован.  
- `remount_usb_disk()` — проверить, что определена и реально перемонтирует  
  том после записи с ПК.  
- Частота приёма зашита в 4 точках `.ino` (trx_switch возврата,  
  vfo_on_trx_switch, cat_set_freq_hz, rx_dsp_start) — при смене править все.

  ## Парковка Core 0 при перестройке PLL (зависания на смене clk_sys)  
  
Отдельный пласт причин, не связанный с гонкой `s_pll_lock_irq` (та отвечала  
за DMA-зависания). Здесь — зависания при переключении `clk_sys` в  
`quad_vfo_set_clk_sys()` (src/vfo/quad_vfo/quad_vfo_pll.cpp).  
  
1. **Парковка Core 0 обязательна.** Без неё Core 0 не переживает смену  
   `clk_sys` под ногами: исполняет flash-код и обращается к XIP в момент  
   переключения шины. Подтверждено прогонами: без парковки — смерть Core 0  
   внутри критической секции.  
  
2. **`rp2040.idleOtherCore()` ненадёжен в рантайме.** Handshake парковки  
   зависает, когда на Core 0 активен TinyUSB/CDC или ядро занято  
   `trx_switch`-ожиданием. На старте (пустой цикл `while(!core1_pll_done)`)  
   работал, в рантайме — нет. Заменён флаг-протоколом.  
  
3. **Флаг-протокол парковки.** Core 1: `ack=false; __dmb(); req++; __dmb();`  
   и ждёт `ack==req`. Core 0 в `core0_wait_in_ram()` (`__not_in_flash_func`):  
   `ack=req` на каждой итерации `while(req)`, `__wfe()` между проверками,  
   `ack=0` на выходе. Core 1 в `unpark`: `req=0; __dmb(); __sev();` — без  
   `__sev()` `__wfe()` не просыпается. Core 0 обслуживает `core0_park_poll()`  
   в обоих циклах ожидания: `while(!core1_pll_done)` и `trx_switch`.  
  
4. **Stale-ack — реальная гонка.** Если не гасить `ack` перед взводом `req`,  
   следующий `park` проходит мимо: Core 1 видит подтверждение от прошлого  
   окна и переключает шину под живым Core 0. Порядок `ack=false` → `req++`  
   обязателен; `ack` перевзводится внутри цикла ожидания, а не один раз  
   на входе.  
  
5. **Никакого `Serial`/`printf`/`flush` внутри парковочных окон.** CDC  
   обслуживается Core 0 — при припаркованном ядре любой вывод в порт это  
   дедлок. Диагностика внутри окон — только SRAM-счётчики  
   (`quad_vfo_pll_dbg_stage`, `core0_park_enter/exit/loop_seen`), печать —  
   после `unpark`. Это касается и `Serial.flush()` между сегментами.  
  
6. **`ssi_hw->baudr` перед `clk_sys_to_pll`.** При переходе на ~337 МГц  
   XIP-чтение flash без замедления делителя убивает обращение к flash.  
   Делитель ставится до переключения mux (сейчас `baudr=4`; при глитчах  
   поднять до 6). Нужен во всех трёх местах: основной путь, ретрай,  
   `rollback_nominal`.  
  
7. **Mux clk_sys — регистрами, без `clock_configure`.** SDK-обёртка тянет  
   flash-код и внутренние ожидания. Свой `clk_sys_src_to_pll()`: сначала  
   `src=0` (ref) + ждать `selected&1`, потом `auxsrc=clk_sys_pll`, потом  
   `src=1` + ждать `selected&2`. Менять `auxsrc` под выбранным aux нельзя —  
   глитч.  
  
8. **Порядок в `setup()`: `usb_audio_init` ПОСЛЕ PLL-окна.** В момент  
   парковки на Core 0 должен быть пустой цикл ожидания — живой TinyUSB-стек  
   кладёт handshake. Реэнумерация (`TinyUSBDevice.detach()/attach()`) в  
   `usb_audio_init` отключена: полураспавшийся дескриптор давал «Неизвестное  
   USB-устройство» и мёртвый порт. UAC поднимается без неё.  
  
9. **Диагностика, которую стоит держать в коде:**  
   - `PARK TIMEOUT`-дамп в `park_core0` (req/ack/enter/exit/loop_seen/  
     dbg_stage) вместо вечного спина;  
   - LED-маячок 2 Гц в `loop()`: LED замёрз = ядро умерло; мигает при  
     оборванном логе = мёртв только CDC-вывод, тракт жив;  
   - маркеры `[PLL] dbg:` на границах сегментов — по последней строке  
     видно сегмент зависания.  
  
10. **Известный остаточный риск.** Если Core 1 зависнет, держа `req`,  
    Core 0 заснёт в `core0_wait_in_ram` навсегда — `trx_switch`-таймаут  
    его не достанет. Для релиза можно добавить второй таймаут внутри  
    RAM-цикла.

    ## quad_vfo: проверенные решения и ловушки (по итогам выноса в библиотеку)  
  
### Верификация PLL — только измерением, не LOCK-битом  
  
- `quad_vfo_set_clk_sys()` подтверждает захват связкой «аппаратный LOCK +  
  измерение clk_sys счётчиком FC0 в окне ±0,3%», до 3 попыток с полным  
  циклом `clk_sys→XOSC→PLL`. LOCK-бит сам по себе недостаточен — на  
  оверклоке VCO он может врать.  
- Протокол возврата трёх состояний через `out_nominal_ok`: цель  
  подтверждена / цель не взяла, но номинал восстановлен и измерен /  
  шина на XOSC (полный отказ). `quad_vfo_start` при «номинал  
  восстановлен» НЕ останавливает генератор — пересчитывает делитель  
  под фактический clk_sys (`recompute_div`). Останавливать генерацию  
  на успешном rollback — ошибка, проверено.  
- `rollback_nominal` возвращает `bool` — результат отката тоже  
  верифицируется измерением, «молчаливый» откат на XOSC отличим.  
  
### FC0 (frequency counter) — не читать result вручную  
  
- Попытка обойти `frequency_count_khz()` прямой записью  
  `fc0.interval`/`fc0.src` и чтением `fc0.result` (KHZ+FRAC поля)  
  давала мусор ~+8 МГц — верификация ±0,3% валилась → rollback →  
  делитель на номинале не находился → генерация пропадала.  
  Причина точно не локализована (вероятно stale DONE/result).  
  Рабочий вариант: `frequency_count_khz(src) * 1000u` — квантование  
  ±1 кГц, для окна ±0,3% достаточно. «Герцового» разрешения на FC0  
  нет — для точной оценки LO нужен внешний частотомер.  
- `measured` vs `cached` расходятся на ±1 кГц — это квантование  
  счётчика, не ошибка PLL.  
  
### Сканер PLL  
  
- `refdiv` перебирается 1..3 — НЕ ограничивать до 1: refdiv=2/3 даёт  
  более мелкую решётку clk_sys и меньшую остаточную ошибку LO  
  (было err=2401 Гц при refdiv=1-only решётке).  
- `pll_vco_max_hz` в конфиге — пользовательский потолок, может  
  превышать даташитные 1600 МГц (проверено до 4,2 ГГц, VCO 3,6 ГГц  
  лочился). Валидация только снизу (>= 400 МГц). Профили  
  `pll_nominal`/`pll_ceiling` обязаны оставаться в спеке.  
- `p1 < 2` в профиле молча подменяется номиналом — недокументированное  
  деление пополам на VCO < ~2,8 ГГц.  
- Остаточная ошибка LO ~600–2500 Гц — дискретность решётки VCO,  
  не баг. В точном режиме (nco_max_hz=0) возможен err=0.  
  
### Электрический предел LO — 45 МГц  
  
- `QUAD_VFO_LO_MAX_HZ = 45000000` — выше ~45–50 МГц GPIO-пэды  
  заваливают фронты: квадратура вырождается в синус с потерей  
  амплитуды (осциллограммы: 14 МГц чисто, 28 МГц скругление,  
  51 МГц — синус 3,4Vpp). Надёжная квадратура — до ~30 МГц.  
- Проверка по `lo_hz` (не `freq_hz`) — ПЧ-режим с freq чуть выше  
  предела допустим, если LO в пределе.  
  
### Тихий режим  
  
- `quad_vfo_set_quiet(bool)` + `quad_vfo_pll_set_quiet(bool)` глушат  
  весь вывод [QUAD]/[PLL] — нужен при перестройке энкодером  
  (каждый детент = полный цикл stop→scan→PLL→start). Все Serial  
  в обоих .cpp завёрнуты в `QV_LOG`/`QP_LOG` — при добавлении  
  новых printf использовать макросы, не голый Serial.  
- `quad_vfo_debug()` тоже подавляется флагом (несовпадение с  
  документацией в заголовке — известный хвост).  
  
### Энкодер (порт из RP2040_beacon)  
  
- Пины маяка: ENC_A=GP20, ENC_B=GP21, BTN=GP19 (не 9/10/11).  
- `enc_lut` + `enc_pending_steps` + `ENC_TICKS_PER_DETENT=4` —  
  накапливать тики, отдавать только полные детенты; иначе один  
  щелчок даёт несколько шагов. Ускорение — в poll по dt, не в ISR.  
- Кнопка — опрос с дебаунсом 30 мс, не IRQ (дребезг кнопки  
  энкодера генерирует ложные прерывания).