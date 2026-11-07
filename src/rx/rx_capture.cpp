// src/rx/rx_capture.cpp  
// ============================================================================  
// Захват ADC+DMA: ADC0=I (GP26), ADC1=Q (GP27), round-robin, 16-бит переносы  
// в 32-бит слова [15:0]=I, [31:16]=Q. Пинг-понг двух DMA-каналов через  
// chain_to; DMA IRQ0 на Core 1 публикует блоки в SPSC-очередь для Core 1.  
// Порт из RP2040_receiver/src/dsp/rx_capture.cpp. Буферы статические  
// (DSP-путь не проходит через mem_arena — он живёт до mem_arena_switch,  
// а пул сбрасывается при trx_switch).  
// ============================================================================  
#include <Arduino.h>  
#include <string.h>  
  
#include "rx_capture.h"  
#include "rx_dsp.h"  
#include "../pins.h"  
  
#include "hardware/adc.h"  
#include "hardware/dma.h"  
#include "hardware/irq.h"  
#include "hardware/sync.h"
#include <hardware/structs/ssi.h>
  
// Пины ADC — из карты пинов проекта; запасные дефолты по pins.h.  
#ifndef RX_ADC_PIN_I  
#define RX_ADC_PIN_I 26   // GP26 = ADC0, канал I  
#endif  
#ifndef RX_ADC_PIN_Q  
#define RX_ADC_PIN_Q 27   // GP27 = ADC1, канал Q  
#endif  
  
// Один блок = RX_BLOCK_SAMPLES пар I/Q → вдвое больше 16-битных переносов.  
#define RX_BLOCK_TRANSFERS (2u * RX_BLOCK_SAMPLES)  
  
// Кольцевая очередь: один пустой слот отличает full от empty (ёмкость 3).  
#define RX_QUEUE_DEPTH 4u  
  
#if RX_BLOCK_SAMPLES == 0  
#error "RX_BLOCK_SAMPLES должен быть больше нуля"  
#endif  
  
// ---------------------------------------------------------------------------  
// Буферы DMA (volatile — пишутся DMA независимо от CPU)  
// ---------------------------------------------------------------------------  
static volatile uint32_t __attribute__((aligned(4))) rx_buf_a0[RX_BLOCK_SAMPLES];  
static volatile uint32_t __attribute__((aligned(4))) rx_buf_a1[RX_BLOCK_SAMPLES];  
static volatile uint32_t __attribute__((aligned(4))) rx_buf_b0[RX_BLOCK_SAMPLES];  
static volatile uint32_t __attribute__((aligned(4))) rx_buf_b1[RX_BLOCK_SAMPLES];  
  
// Scratch — приёмник на случай, когда оба рабочих буфера канала заняты DSP.  
static volatile uint32_t __attribute__((aligned(4))) rx_scratch_a[RX_BLOCK_SAMPLES];  
static volatile uint32_t __attribute__((aligned(4))) rx_scratch_b[RX_BLOCK_SAMPLES];  
  
// ---------------------------------------------------------------------------  
// SPSC-очередь завершённых блоков: пишет IRQ и читает DSP — оба на Core 1 
// ---------------------------------------------------------------------------  
static const volatile uint32_t* rx_queue[RX_QUEUE_DEPTH];  
static volatile uint8_t rx_q_head = 0;  
static volatile uint8_t rx_q_tail = 0;  
  
static volatile uint32_t rx_dropped = 0;  
  
// Диагностика — extern по rx_capture.h (считает modem/диагностика).  
volatile uint32_t rx_cap_isr_count    = 0;   // входов в DMA IRQ  
volatile uint32_t rx_drop_scratch     = 0;  
volatile uint32_t rx_drop_queue_full  = 0;  
volatile uint32_t rx_drop_busy        = 0;  
static volatile uint32_t rx_irq_both_pending = 0;  
  
// ---------------------------------------------------------------------------  
// Владение рабочими буферами (busy = опубликован DSP, не назначать DMA)  
// ---------------------------------------------------------------------------  
static volatile bool rx_busy_a[2] = { false, false };  
static volatile bool rx_busy_b[2] = { false, false };  
static volatile uint8_t rx_a_sel = 0;  
static volatile uint8_t rx_b_sel = 0;  
  
static volatile uint32_t* rx_dma_a_target = rx_buf_a0;  
static volatile uint32_t* rx_dma_b_target = rx_buf_b0;  
  
static int rx_dma_a = -1;  
static int rx_dma_b = -1;  
static volatile bool rx_running = false;  
  
// ---------------------------------------------------------------------------  
static inline volatile uint32_t* rx_buffer_for(bool channel_a, int idx) {  
    if (channel_a) return (idx == 0) ? rx_buf_a0 : rx_buf_a1;  
    return (idx == 0) ? rx_buf_b0 : rx_buf_b1;  
}  
static inline volatile uint32_t* rx_scratch_for(bool channel_a) {  
    return channel_a ? rx_scratch_a : rx_scratch_b;  
}  
static inline volatile bool* rx_busy_for(bool channel_a) {  
    return channel_a ? rx_busy_a : rx_busy_b;  
}  
static inline volatile uint8_t* rx_select_for(bool channel_a) {  
    return channel_a ? &rx_a_sel : &rx_b_sel;  
}  
  
static int __not_in_flash_func(rx_find_buffer)(  
    const volatile uint32_t* target, bool channel_a) {  
    if (channel_a) {  
        if (target == rx_buf_a0) return 0;  
        if (target == rx_buf_a1) return 1;  
    } else {  
        if (target == rx_buf_b0) return 0;  
        if (target == rx_buf_b1) return 1;  
    }  
    return -1;  
}  
  
// ---------------------------------------------------------------------------  
// Переназначение цели DMA «на лету» + восстановление регистров канала.  
// Вызывается из IRQ; канал перезапустится аппаратно через chain_to напарника.  
// ---------------------------------------------------------------------------  
static void __not_in_flash_func(rx_rearm_channel)(int channel, bool channel_a) {  
    volatile bool* busy = rx_busy_for(channel_a);  
    volatile uint8_t* sel = rx_select_for(channel_a);  
  
    int chosen = -1;  
    for (int k = 0; k < 2; ++k) {  
        uint8_t idx = (uint8_t)((*sel + (uint8_t)k) & 1u);  
        if (!busy[idx]) { chosen = (int)idx; break; }  
    }  
  
    volatile uint32_t* target;  
    if (chosen >= 0) {  
        target = rx_buffer_for(channel_a, chosen);  
        *sel = (uint8_t)(chosen ^ 1);  
    } else {  
        target = rx_scratch_for(channel_a);  // Core 1 не успел — сливаем в scratch  
    }  
  
    if (channel_a) rx_dma_a_target = target; else rx_dma_b_target = target;  
  
    dma_channel_set_write_addr(channel, (void*)target, false);  
    dma_channel_set_trans_count(channel, RX_BLOCK_TRANSFERS, false);  
}  
  
// ---------------------------------------------------------------------------  
// Обработка завершившегося канала: публикация блока + rearm  
// ---------------------------------------------------------------------------  
static void __not_in_flash_func(rx_handle_completed_channel)(  
    int channel, bool channel_a) {  
    dma_channel_acknowledge_irq0(channel);  
    if (!rx_running) return;  
  
    const volatile uint32_t* completed_target =  
        channel_a ? rx_dma_a_target : rx_dma_b_target;  
    const int idx = rx_find_buffer(completed_target, channel_a);  
  
    if (idx < 0) {                       // блок ушёл в scratch — отбрасываем  
        ++rx_dropped;  
        ++rx_drop_scratch;  
        rx_rearm_channel(channel, channel_a);  
        return;  
    }  
  
    volatile bool* busy = rx_busy_for(channel_a);  
    const uint8_t head = rx_q_head;  
    const uint8_t tail = rx_q_tail;  
    const uint8_t next = (uint8_t)((head + 1u) % RX_QUEUE_DEPTH);  
  
    if (next == tail) {                  // очередь полна  
        ++rx_dropped;  
        ++rx_drop_queue_full;  
        rx_rearm_channel(channel, channel_a);  
        return;  
    }  
  
    if (busy[idx]) {                     // рассинхрон владения  
        ++rx_dropped;  
        ++rx_drop_busy;  
        rx_rearm_channel(channel, channel_a);  
        return;  
    }  
  
    busy[idx] = true;  
    __dmb();  
    rx_queue[head] = completed_target;  
    __dmb();  
    rx_q_head = next;  
  
    rx_rearm_channel(channel, channel_a);  
}  
  
// ---------------------------------------------------------------------------  
// ISR DMA_IRQ_0 (Core 1)  
// ---------------------------------------------------------------------------  

static void __not_in_flash_func(rx_dma_irq)(void) {    
    ++rx_cap_isr_count;    
    const int a = rx_dma_a;    
    if (a < 0 || !dma_channel_get_irq0_status(a)) return;    
    dma_channel_acknowledge_irq0(a);    
    if (!rx_running) return;    
    
    volatile uint32_t* done_buf = (volatile uint32_t*)rx_dma_a_target;    
    
    // Флаг занятости завершённого буфера — из любого пула (a или b)    
    volatile bool* done_busy = NULL;    
    if      (done_buf == rx_buf_a0) done_busy = &rx_busy_a[0];    
    else if (done_buf == rx_buf_a1) done_busy = &rx_busy_a[1];    
    else if (done_buf == rx_buf_b0) done_busy = &rx_busy_b[0];    
    else if (done_buf == rx_buf_b1) done_busy = &rx_busy_b[1];    
    // done_buf == rx_scratch_a → done_busy == NULL, блок не публикуем    
    
    const uint8_t head = rx_q_head;    
    const uint8_t next = (uint8_t)((head + 1u) % RX_QUEUE_DEPTH);    
    
    if (done_busy && next != rx_q_tail) {    
        *done_busy = true;              // 1. владение — до публикации    
        __dmb();    
        rx_queue[head] = done_buf;      // 2. указатель в очередь    
        __dmb();    
        rx_q_head = next;               // 3. публикация    
    } else {    
        ++rx_dropped;    
        if (next == rx_q_tail) ++rx_drop_queue_full;    
        else                 ++rx_drop_scratch;   // блок был в scratch    
    }    
    
    // Следующий буфер: свободный a → свободный b → scratch    
    volatile uint32_t* next_buf;    
    if (!rx_busy_a[0] || !rx_busy_a[1]) {    
        rx_a_sel = rx_busy_a[0] ? 1u : 0u;    
        next_buf = rx_a_sel ? rx_buf_a1 : rx_buf_a0;    
    } else if (!rx_busy_b[0] || !rx_busy_b[1]) {    
        rx_b_sel = rx_busy_b[0] ? 1u : 0u;    
        next_buf = rx_b_sel ? rx_buf_b1 : rx_buf_b0;    
        ++rx_drop_busy;                 // a-пул исчерпан — диагностика    
    } else {    
        next_buf = rx_scratch_a;        // всё занято — сливаем в scratch    
        ++rx_drop_busy;    
    }    
    rx_dma_a_target = next_buf;    
    
    dma_channel_set_write_addr(a, next_buf, false);    
    dma_channel_set_trans_count(a, RX_BLOCK_TRANSFERS, true);    
}

// ---------------------------------------------------------------------------  
// Запуск захвата (после rx_dsp_start — NCO уже настроен)  
// ---------------------------------------------------------------------------  

void __not_in_flash_func(rx_capture_start)(void) {  
    // DMA_IRQ_0 живёт на Core 1 (там же DSP-поллинг очереди).  
    assert(get_core_num() == 1);  
  
    if (rx_running) return;  
  
    rx_q_head = 0;  
    rx_q_tail = 0;  
    rx_dropped = 0;  
    rx_cap_isr_count = 0;  
    rx_drop_scratch = 0;  
    rx_drop_queue_full = 0;  
    rx_drop_busy = 0;  
    rx_irq_both_pending = 0;  
    rx_busy_a[0] = rx_busy_a[1] = false;  
    rx_busy_b[0] = rx_busy_b[1] = false;  
    rx_a_sel = 0;  
    rx_b_sel = 0;  
    rx_dma_a_target = rx_buf_a0;  
    rx_dma_b_target = rx_buf_b0;  
  
    Serial.println("[CAP] m1: init done");  
  
    const uint32_t clk_sys_hz = clock_get_hz(clk_sys);  
    const uint32_t clk_adc_hz = clock_get_hz(clk_adc);  
    Serial.printf("[CAP] clk_adc=%lu clk_sys=%lu\n",  
                  (unsigned long)clk_adc_hz, (unsigned long)clk_sys_hz);  
  
    adc_init();  
    adc_gpio_init(RX_ADC_PIN_I);    
    adc_gpio_init(RX_ADC_PIN_Q);
    adc_set_round_robin((1u << 0) | (1u << 1));  
    adc_fifo_setup(true, true, 1, false, false);  
    float adc_div = (float)clk_adc_hz / (float)RX_ADC_SAMPLE_RATE_HZ - 1.0f;    
    if (adc_div < 0.0f) adc_div = 0.0f;
    adc_set_clkdiv(adc_div);  
    Serial.println("[CAP] m4: adc cfg");  
  
    // ОДИН канал — два активных канала вешают систему на разгоне  
    rx_dma_a = dma_claim_unused_channel(true);  
    rx_dma_b = -1;  
    Serial.println("[CAP] m5: dma claimed (single)");  
  
    dma_channel_config ca = dma_channel_get_default_config(rx_dma_a);    
    channel_config_set_transfer_data_size(&ca, DMA_SIZE_16);
    channel_config_set_read_increment(&ca, false);  
    channel_config_set_write_increment(&ca, true);  
    channel_config_set_dreq(&ca, DREQ_ADC);  
    // chain_to НЕ используется — перезапуск программный, в rx_dma_irq  
    dma_channel_configure(rx_dma_a, &ca, rx_dma_a_target, &adc_hw->fifo,  
                          RX_BLOCK_TRANSFERS, false);  
    Serial.println("[CAP] m6: dma configured (single)");  
  
    dma_channel_set_irq0_enabled(rx_dma_a, true);  
    irq_set_exclusive_handler(DMA_IRQ_0, rx_dma_irq);  
    irq_set_enabled(DMA_IRQ_0, true);  
    Serial.println("[CAP] m8: irq enabled");  
  
    dma_channel_start(rx_dma_a);  
    Serial.println("[CAP] m9: chA started");  
    adc_run(true);  
    Serial.println("[CAP] m10: adc running");  
  
    rx_running = true;  
  
    Serial.printf("[CAP] chA=%d (single+irq) block=%u pairs transfers=%u\n",  
                  rx_dma_a, (unsigned)RX_BLOCK_SAMPLES, (unsigned)RX_BLOCK_TRANSFERS);  
    Serial.printf("[CAP] ADC target=%lu/s clkdiv=%ld\n",  
                  (unsigned long)RX_ADC_SAMPLE_RATE_HZ, (long)(adc_div + 0.5f));  
    Serial.println("[CAP] m11: started ok");  
}



// ---------------------------------------------------------------------------  
// Останов DMA+ADC. Вызывается из core_sync_core1_poll при парковке — rx_dsp_stop не обязателен
// ---------------------------------------------------------------------------  
void __not_in_flash_func(rx_capture_stop)(void) {
    rx_running = false;  
    adc_run(false);  
    irq_set_enabled(DMA_IRQ_0, false);  
  
    if (rx_dma_a >= 0) {  
        dma_channel_set_irq0_enabled(rx_dma_a, false);  
        dma_channel_abort(rx_dma_a);  
        dma_channel_acknowledge_irq0(rx_dma_a);  
    }  
    if (rx_dma_b >= 0) {  
        dma_channel_set_irq0_enabled(rx_dma_b, false);  
        dma_channel_abort(rx_dma_b);  
        dma_channel_acknowledge_irq0(rx_dma_b);  
    }  
  
    adc_fifo_drain();  
    rx_q_head = 0;  
    rx_q_tail = 0;  
    rx_busy_a[0] = rx_busy_a[1] = false;  
    rx_busy_b[0] = rx_busy_b[1] = false;  
    rx_a_sel = 0;  
    rx_b_sel = 0;  
    rx_dma_a_target = rx_buf_a0;  
    rx_dma_b_target = rx_buf_b0;  
  
    if (rx_dma_a >= 0) { dma_channel_unclaim(rx_dma_a); rx_dma_a = -1; }  
    if (rx_dma_b >= 0) { dma_channel_unclaim(rx_dma_b); rx_dma_b = -1; }  
}  
  
// ---------------------------------------------------------------------------  
// API очереди — ТОЛЬКО Core 1 (из rx_dsp_poll)  
// ---------------------------------------------------------------------------  
const volatile uint32_t* rx_capture_get_block(void) {  
    const uint8_t tail = rx_q_tail;  
    if (tail == rx_q_head) return NULL;  
    __dmb();  
    return rx_queue[tail];  
}  
  
void rx_capture_release(void) {  
    const uint8_t tail = rx_q_tail;  
    if (tail == rx_q_head) return;  
  
    const volatile uint32_t* buf = rx_queue[tail];  
    __dmb();  
    rx_q_tail = (uint8_t)((tail + 1u) % RX_QUEUE_DEPTH);  
    __dmb();  
  
    if      (buf == rx_buf_a0) rx_busy_a[0] = false;  
    else if (buf == rx_buf_a1) rx_busy_a[1] = false;  
    else if (buf == rx_buf_b0) rx_busy_b[0] = false;  
    else if (buf == rx_buf_b1) rx_busy_b[1] = false;  
}  
  
uint32_t rx_capture_dropped(void) { return rx_dropped; }

bool rx_capture_is_running(void) {  
    return rx_running;  
}

// Минимальный тест ADC->DMA на разгоне. Всё в RAM, без IRQ, без DSP.  
// Результат — в SRAM-счётчиках, читаются с Core 0.  
  
volatile uint32_t dma_test_tc_start = 0;    // transfer_count на старте  
volatile uint32_t dma_test_tc_end   = 0;    // transfer_count через ~N итераций  
volatile uint32_t dma_test_iters    = 0;    // сколько итераций прожили  
volatile uint32_t dma_test_done     = 0;    // 1 = тест завершился  
  
static volatile uint32_t dma_test_buf[512]; // приёмный буфер в SRAM  
  
void __not_in_flash_func(dma_overclock_test)(void) {  
    // ADC: два канала round-robin, как в боевом захвате  
    adc_init();  
    adc_gpio_init(26);  
    adc_gpio_init(27);  
    adc_select_input(0);  
    adc_set_round_robin(0x03);  
    adc_fifo_setup(true, true, 1, false, false);  
    adc_set_clkdiv(191.0f);   // 250 kS/s, как в бою  
  
    // Один DMA-канал, без chain, без IRQ  
    int ch = dma_claim_unused_channel(true);  
    dma_channel_config c = dma_channel_get_default_config(ch);  
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);  
    channel_config_set_read_increment(&c, false);  
    channel_config_set_write_increment(&c, true);  
    channel_config_set_dreq(&c, DREQ_ADC);  
    dma_channel_configure(ch, &c, (void*)dma_test_buf,  
                          &adc_hw->fifo, 512, false);  
  
    adc_fifo_drain();  
    adc_run(true);  
    dma_channel_start(ch);  
  
    dma_test_tc_start = dma_hw->ch[ch].transfer_count;  
  
    // Ждём ~несколько мс чистым спином — никаких вызовов наружу  
    for (volatile uint32_t i = 0; i < 2000000u; ++i) {  
        dma_test_iters = i;  
    }  
  
    dma_test_tc_end = dma_hw->ch[ch].transfer_count;  
    dma_test_done = 1;  
  
    // Стоп — чтобы не мешать дальнейшему коду  
    adc_run(false);  
    dma_channel_abort(ch);  

    // в loop(), однократно после done:  
    static bool printed = false;  
    if (!printed && dma_test_done) {  
        printed = true;  
        Serial.printf("[DMAT] tc_start=%lu tc_end=%lu iters=%lu\n",  
                    (unsigned long)dma_test_tc_start,  
                    (unsigned long)dma_test_tc_end,  
                    (unsigned long)dma_test_iters);  
    }

}