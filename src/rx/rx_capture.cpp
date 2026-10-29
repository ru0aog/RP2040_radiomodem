// src/rx/rx_capture.cpp — DMA-движок приёмной части.  
// Порт из RP2040_receiver/src/dsp/rx_capture.cpp (GPLv3, RU0AOG).  
// Отличия: константы в rx_dsp.h; пины из ../pins.h.  
#include <Arduino.h>  
#include <string.h>  
#include "rx_capture.h"  
#include "../pins.h"            // RX_ADC_PIN_I / RX_ADC_PIN_Q  
#include "hardware/adc.h"  
#include "hardware/dma.h"  
#include "hardware/irq.h"  
#include "hardware/sync.h"  
  
#ifndef RX_ADC_PIN_I  
#define RX_ADC_PIN_I 26   // GPIO26 = ADC0, канал I  
#define RX_ADC_PIN_Q 27   // GPIO27 = ADC1, канал Q  
#endif  

volatile uint32_t rx_cap_isr_count = 0;   // ++ в DMA IRQ handler 

// Один блок = RX_BLOCK_SAMPLES пар I/Q = вдвое больше 16-бит переносов.  
#define RX_BLOCK_TRANSFERS (2u * RX_BLOCK_SAMPLES)  
// Кольцо с одним пустым слотом: при глубине 4 ёмкость = 3 указателя.  
#define RX_QUEUE_DEPTH 4u  
  
// ---- Буферы DMA ----  
static volatile uint32_t __attribute__((aligned(4))) rx_buf_a0[RX_BLOCK_SAMPLES];  
static volatile uint32_t __attribute__((aligned(4))) rx_buf_a1[RX_BLOCK_SAMPLES];  
static volatile uint32_t __attribute__((aligned(4))) rx_buf_b0[RX_BLOCK_SAMPLES];  
static volatile uint32_t __attribute__((aligned(4))) rx_buf_b1[RX_BLOCK_SAMPLES];  
// Scratch — когда оба рабочих буфера канала заняты DSP.  
static volatile uint32_t __attribute__((aligned(4))) rx_scratch_a[RX_BLOCK_SAMPLES];  
static volatile uint32_t __attribute__((aligned(4))) rx_scratch_b[RX_BLOCK_SAMPLES];  
  
// ---- SPSC-очередь: producer = DMA IRQ (Core 0), consumer = rx_dsp (Core 1) ----  
static const volatile uint32_t* rx_queue[RX_QUEUE_DEPTH];  
static volatile uint8_t  rx_q_head = 0;  
static volatile uint8_t  rx_q_tail = 0;  
static volatile uint32_t rx_dropped = 0;  
volatile uint32_t rx_drop_scratch = 0;  
volatile uint32_t rx_drop_queue_full = 0;  
volatile uint32_t rx_drop_busy = 0;
static volatile uint32_t rx_irq_both_pending = 0;  
  
// ---- Владение буферами ----  
static volatile bool    rx_busy_a[2] = { false, false };  
static volatile bool    rx_busy_b[2] = { false, false };  
static volatile uint8_t rx_a_sel = 0, rx_b_sel = 0;  
static volatile uint32_t* rx_dma_a_target = rx_buf_a0;  
static volatile uint32_t* rx_dma_b_target = rx_buf_b0;  
static int  rx_dma_a = -1, rx_dma_b = -1;  
static volatile bool rx_running = false;  
  
static inline volatile uint32_t* rx_buffer_for(bool a, int idx) {  
  return a ? ((idx == 0) ? rx_buf_a0 : rx_buf_a1)  
           : ((idx == 0) ? rx_buf_b0 : rx_buf_b1);  
}  
static inline volatile uint32_t* rx_scratch_for(bool a) {  
  return a ? rx_scratch_a : rx_scratch_b;  
}  
static inline volatile bool* rx_busy_for(bool a) {  
  return a ? rx_busy_a : rx_busy_b;  
}  
static inline volatile uint8_t* rx_select_for(bool a) {  
  return a ? &rx_a_sel : &rx_b_sel;  
}  
  
static int __not_in_flash_func(rx_find_buffer)(const volatile uint32_t* t, bool a) {  
  if (a) { if (t == rx_buf_a0) return 0; if (t == rx_buf_a1) return 1; }  
  else   { if (t == rx_buf_b0) return 0; if (t == rx_buf_b1) return 1; }  
  return -1;  
}  
  
// Перенастройка завершившего канала на следующий буфер (или scratch).  
// Вызывается из IRQ; канал перезапустится аппаратно через chain_to.  
static void __not_in_flash_func(rx_rearm_channel)(int channel, bool a) {  
  volatile bool*    busy = rx_busy_for(a);  
  volatile uint8_t* sel  = rx_select_for(a);  
  int chosen = -1;  
  for (int k = 0; k < 2; ++k) {  
    uint8_t idx = (uint8_t)((*sel + (uint8_t)k) & 1u);  
    if (!busy[idx]) { chosen = (int)idx; break; }  
  }  
  volatile uint32_t* target;  
  if (chosen >= 0) { target = rx_buffer_for(a, chosen); *sel = (uint8_t)(chosen ^ 1); }  
  else             { target = rx_scratch_for(a); }              // DSP перегружен — блок будет дропнут  
  if (a) rx_dma_a_target = target; else rx_dma_b_target = target;  
  dma_channel_set_write_addr(channel, (void*)target, false);  
  dma_channel_set_trans_count(channel, RX_BLOCK_TRANSFERS, false);  
}  
  
// Обработка завершившегося канала: публикация блока в SPSC + rearm.  
static void __not_in_flash_func(rx_handle_completed_channel)(int channel, bool a) {  
  dma_channel_acknowledge_irq0(channel);  
  if (!rx_running) return;  
  
  const volatile uint32_t* completed = a ? rx_dma_a_target : rx_dma_b_target;  
  const int idx = rx_find_buffer(completed, a);  
  
  if (idx < 0) {                       // блок ушёл в scratch — дроп  
    ++rx_dropped; ++rx_drop_scratch;  
    rx_rearm_channel(channel, a); return;  
  }  
  const uint8_t head = rx_q_head, tail = rx_q_tail;  
  const uint8_t next = (uint8_t)((head + 1u) % RX_QUEUE_DEPTH);  
  if (next == tail) {                  // очередь полна — дроп  
    ++rx_dropped; ++rx_drop_queue_full;  
    rx_rearm_channel(channel, a); return;  
  }  
  volatile bool* busy = rx_busy_for(a);  
  if (busy[idx]) {                     // буфер ещё занят Core 1 — дроп  
    ++rx_dropped; ++rx_drop_busy;  
    rx_rearm_channel(channel, a); return;  
  }  
  busy[idx] = true;  
  __dmb();  
  rx_queue[head] = completed;  
  __dmb();  
  rx_q_head = next;  
  rx_rearm_channel(channel, a);  
}  
  
// DMA_IRQ_0 — принадлежит целиком rx_capture.  
static void __not_in_flash_func(rx_dma_irq)(void) {  
  ++rx_cap_isr_count;  
  const int a = rx_dma_a, b = rx_dma_b;  
  const bool a_pending = (a >= 0) && dma_channel_get_irq0_status(a);  
  const bool b_pending = (b >= 0) && dma_channel_get_irq0_status(b);  
  if (a_pending && b_pending) ++rx_irq_both_pending;  
  if (a_pending) rx_handle_completed_channel(a, true);  
  if (b_pending) rx_handle_completed_channel(b, false);  
}
  
void rx_capture_start(void) {  
  if (rx_running) return;  
  rx_q_head = rx_q_tail = 0;  
  rx_dropped = rx_drop_scratch = rx_drop_queue_full = rx_drop_busy = 0;  
  rx_irq_both_pending = 0;  
  rx_busy_a[0] = rx_busy_a[1] = rx_busy_b[0] = rx_busy_b[1] = false;  
  rx_a_sel = rx_b_sel = 0;  
  rx_dma_a_target = rx_buf_a0; rx_dma_b_target = rx_buf_b0;  
  
  adc_init();  
  adc_gpio_init(RX_ADC_PIN_I);  
  adc_gpio_init(RX_ADC_PIN_Q);  
  adc_set_round_robin(0b00011);               // ADC0,ADC1,ADC0,ADC1,...  
  adc_fifo_setup(true, true, 1, false, false);  
  float adc_div = 48000000.0f / (float)RX_ADC_SAMPLE_RATE_HZ - 1.0f;  
  if (adc_div < 0.0f) adc_div = 0.0f;  
  adc_set_clkdiv(adc_div);  
  
  rx_dma_a = dma_claim_unused_channel(true);  
  rx_dma_b = dma_claim_unused_channel(true);  
  dma_channel_config ca = dma_channel_get_default_config(rx_dma_a);  
  channel_config_set_transfer_data_size(&ca, DMA_SIZE_16);  
  channel_config_set_read_increment(&ca, false);  
  channel_config_set_write_increment(&ca, true);  
  channel_config_set_dreq(&ca, DREQ_ADC);  
  channel_config_set_chain_to(&ca, rx_dma_b);  
  dma_channel_config cb = dma_channel_get_default_config(rx_dma_b);  
  channel_config_set_transfer_data_size(&cb, DMA_SIZE_16);  
  channel_config_set_read_increment(&cb, false);  
  channel_config_set_write_increment(&cb, true);  
  channel_config_set_dreq(&cb, DREQ_ADC);  
  channel_config_set_chain_to(&cb, rx_dma_a);  
  dma_channel_configure(rx_dma_a, &ca, (void*)rx_buf_a0, &adc_hw->fifo,  
                        RX_BLOCK_TRANSFERS, false);  
  dma_channel_configure(rx_dma_b, &cb, (void*)rx_buf_b0, &adc_hw->fifo,  
                        RX_BLOCK_TRANSFERS, false);  
  
  dma_channel_acknowledge_irq0(rx_dma_a);  
  dma_channel_acknowledge_irq0(rx_dma_b);  
  dma_channel_set_irq0_enabled(rx_dma_a, true);  
  dma_channel_set_irq0_enabled(rx_dma_b, true);  
  irq_set_exclusive_handler(DMA_IRQ_0, rx_dma_irq);  
  
  adc_run(false);  
  adc_fifo_drain();  
  adc_select_input(0);  
  
  rx_running = true;  
  irq_set_enabled(DMA_IRQ_0, true);  
  dma_channel_start(rx_dma_a);  
  adc_run(true);  
  
  Serial.printf("[CAP] chA=%d chB=%d block=%u pairs transfers=%u\n",  
                rx_dma_a, rx_dma_b,  
                (unsigned)RX_BLOCK_SAMPLES, (unsigned)RX_BLOCK_TRANSFERS);  
  Serial.printf("[CAP] ADC target=%lu/s clkdiv=%.3f\n",  
                (unsigned long)RX_ADC_SAMPLE_RATE_HZ, (double)adc_div);  
}  
  
// Контракт: сначала rx_dsp_stop(), потом rx_capture_stop().  
void rx_capture_stop(void) {  
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
  rx_q_head = rx_q_tail = 0;  
  rx_busy_a[0] = rx_busy_a[1] = rx_busy_b[0] = rx_busy_b[1] = false;  
  rx_a_sel = rx_b_sel = 0;  
  rx_dma_a_target = rx_buf_a0; rx_dma_b_target = rx_buf_b0;  
  if (rx_dma_a >= 0) { dma_channel_unclaim(rx_dma_a); rx_dma_a = -1; }  
  if (rx_dma_b >= 0) { dma_channel_unclaim(rx_dma_b); rx_dma_b = -1; }  
}  
  
// ---- API очереди для Core 1 ----  
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