// src/vfo/soft_rx.cpp  
// ============================================================================  
// Квадратурный гетеродин приёмника (soft_rx): 4-тактовая PIO-программа  
// выдаёт на 2 соседних пина последовательность Q,I = 01→11→10→00 —  
// два меандра f/1 со сдвигом 90° для SDR-смесителя.  
//  
// Портировано из RP2040_receiver/src/vfo/vfo_rp2040.cpp (GPLv3, RU0AOG).  
//  
// АДАПТАЦИИ под архитектуру RP2040_radiomodem:  
//  - quad живёт на pio1/SM (TX-меандр остаётся на pio0 — программы  
//    не перетирают друг друга при смене направления);  
//  - multicore_reset_core1() УБРАН: RX- и TX-фазы разводит trx_control,  
//    остановка TX-дизеринга происходит до вызова quad_start();  
//  - PLL-переключение переиспользует soft_tx_set_clk_sys() — единый  
//    безопасный путь (vreg → XOSC → pll_init → SSI BAUDR → pll_sys);  
//  - сканер clk_sys сохранён: подбирает шину под ЦЕЛЫЙ делитель LO и  
//    сладкую зону NCO (логика П1..П6 исходника).  
// ============================================================================  
#include "vfo.h"
#include "soft_rx.h"
#include "soft_tx.h"              // PllConfig, pll_nominal, set_clk_sys, vsel_for  
#include "../pins.h"  
#include "../mem/mem_arena.h"  
#include "hardware/pio.h"  
#include "hardware/clocks.h"
  
// ---- Параметры приёмного тракта ----------------------------------------  
// СВЕРИТЬ с src/dsp/rx_dsp.h приёмника при миграции rx_dsp:  
// NCO-зоны взяты из RP2040_receiver.ino:  
// сладкая зона [9600..14500], запретная [0..5000] (дырка в звуке),  
// NCO_MAX = половина RX_FS_IN_HZ = 62500 (250 ksps/2 канала /2).  
#define RX_NCO_SWEET_LO     9600u  
#define RX_NCO_SWEET_HI     14500u  
#define RX_NCO_FORBID_LO    0u  
#define RX_NCO_FORBID_HI    5000u  
#define RX_NCO_MAX_HZ       62500u  
#define RX_CLK_SYS_MIN_HZ   240000000ULL
#define VFO_CLK_SYS_NOMINAL 133000000ULL  
  
// ---- PIO-программа квадратуры: 4 такта на период, 2 соседних пина ------  
// базовый пин = I, следующий = Q (как в исходнике: pin_freq_out_i — base).  
static const uint16_t pio_quad_instructions[] = {  
  0xe001,  // set pins, 0b01: Q=1 I=0   (пины: base=I=0, base+1=Q=1)  
  0xe003,  // set pins, 0b11: Q=1 I=1  
  0xe002,  // set pins, 0b10: Q=0 I=1  
  0xe000,  // set pins, 0b00: Q=0 I=0  
};  
static const pio_program_t pio_quad_program = {  
  .instructions = pio_quad_instructions,  
  .length       = 4,  
  .origin       = -1,  
  .pio_version  = 0  
};  
  
static PIO      quad_pio            = pio1;   // отдельный блок от TX (pio0)  
static unsigned quad_sm             = 0;  
static unsigned quad_offset         = 0;  
static bool     quad_program_loaded = false;  
static bool     quad_active         = false;  
static uint32_t quad_lo_actual_hz   = 0;      // фактический LO для NCO  
  
uint32_t soft_rx_quad_lo_hz(void) { return quad_lo_actual_hz; }  
  
// ============================================================================  
// ИНИЦИАЛИЗАЦИЯ (из vfo_init) — только загрузка программы и резерв SM  
// ============================================================================  
static void quad_init(void) {  
  if (!quad_program_loaded) {  
    quad_offset = pio_add_program(quad_pio, &pio_quad_program);  
    quad_program_loaded = true;  
  }  
  quad_sm = pio_claim_unused_sm(quad_pio, true);  
  // Пины в output НЕ переводим — направление ставится последним в quad_start,  
  // чтобы не выпускать полуинициализированные переходы в смеситель.  
}  
  
// ============================================================================  
// ЗАПУСК: скан PLL → целый делитель → конфигурация SM  
// (vfo_quadrature_start() исходника, критерии П1..П6 сохранены)  
// ============================================================================  
static void quad_start(uint32_t freq_hz) {  
  // Целевая частота LO — ниже частоты приёма на величину ПЧ.  
  // ПЧ положительна: сигнал ложится в базовую полосу на +RX_IF_SHIFT_HZ.  
  uint32_t lo_hz = (freq_hz > RX_IF_SHIFT_HZ)  
                   ? (freq_hz - RX_IF_SHIFT_HZ) : freq_hz;  
  Serial.printf("[QUAD] Целевая частота LO %lu\n", (unsigned long)lo_hz);  
  
  // === Подгонка clk_sys под ЦЕЛЫЙ делитель LO ===  
  // LO = clk_sys/(4*D), D — делитель PIO. Целый D — чистый LO; дробный —  
  // джиттер → спуры. На фиксированных 133 МГц целый D слишком крупный на ВЧ  
  // (на 14 МГц ближайший D=2 => 16.625 МГц, промах > окна NCO ±62.5 кГц).  
  // Ищем достижимую PLL-комбинацию, пересчитываем делитель от реальной шины.  

const uint64_t xosc_hz = vfo_xosc_hz();  
  // Эффективный потолок скана = min(flash-паспорт, аппаратный барьер).  
  // Если паспорт не доказан (OCTEST не запускался — остался 133 МГц),  
  // окно [MIN, max] пустое → берём прошитый OCTEST-потолок pll_ceiling,  
  // как делает приёмник (380 МГц, VREG_1_30 — доказанная точка).  
  uint64_t clk_max = (vfo_max_clk_sys_hz < 420000000ULL)  
                     ? vfo_max_clk_sys_hz : 420000000ULL;  
  if (clk_max < RX_CLK_SYS_MIN_HZ) {  
    clk_max = pll_ceiling.clk_sys_hz;  
    Serial.printf("[QUAD] паспорт clk_max=%lu МГц не доказан — "  
                  "потолок из pll_ceiling=%lu МГц\n",  
                  (unsigned long)(vfo_max_clk_sys_hz / 1000000UL),  
                  (unsigned long)(clk_max / 1000000ULL));  
  }  
  const uint64_t vco_max = (vfo_max_pll_vco_hz < 4300000000ULL)  
                           ? vfo_max_pll_vco_hz : 4300000000ULL;
  
  // Веса лексикографики: W2 >> W3 >> W4 >> W5 (старший критерий доминирует).  
  const uint64_t W2 = 1000000000ull;  // П2: удаление от сладкой зоны (за 1 Гц)  
  const uint64_t W3 = 1000000ull;     // П3: штраф за дробный делитель  
  const uint64_t W4 = 1000ull;        // П4: близость дроби к k/8  
  const uint64_t W5 = 1ull;           // П5: шина ниже максимума (за 1 МГц)  
  
  PllConfig best        = pll_nominal;  
  uint64_t  best_cost   = UINT64_MAX;  
  uint32_t  best_div_q8 = 0;  
  bool      best_is_int = true;  
  bool      best_legal  = false;  
  
  // Fallback «наименьшее зло»: лучший среди НЕлегальных (дырка в 0..5 кГц).  
  PllConfig fb_best   = pll_nominal;  
  uint64_t  fb_bad    = UINT64_MAX;   // чем меньше — тем ближе дырка к краю 5 кГц  
  uint32_t  fb_div_q8 = 0;  
  bool      fb_is_int = true;  
  
  for (uint32_t refdiv = 1; refdiv <= 3; refdiv++) {  
    uint64_t ref_hz = xosc_hz / refdiv;  
    for (uint32_t p1 = 2; p1 <= 7; p1++) {  
      for (uint32_t p2 = 1; p2 <= 7; p2++) {  
        if (p1 < p2) continue;  
        uint32_t pdiv = p1 * p2;  
        for (uint32_t fbdiv = 16; fbdiv <= 320; fbdiv++) {  
          uint64_t vco = ref_hz * fbdiv;  
          if (vco < 750000000ULL || vco > vco_max) continue;  
          uint64_t clk = vco / pdiv;  
          if (clk < RX_CLK_SYS_MIN_HZ || clk > clk_max) continue;  
  
          // Два варианта делителя на эту шину: целый и дробный (Q8).  
          for (int frac_mode = 0; frac_mode < 2; frac_mode++) {  
            uint32_t div_q8;  
            if (frac_mode == 0) {                       // целый  
              uint64_t D = (clk + 2ull * lo_hz) / (4ull * lo_hz);  
              if (D < 2)      D = 2;  
              if (D > 0xFFFF) D = 0xFFFF;  
              div_q8 = (uint32_t)(D << 8);  
            } else {                                    // дробный Q8  
              uint64_t q = (clk * 256ull + 2ull * lo_hz) / (4ull * lo_hz);  
              if (q < (2u << 8))      q = 2u << 8;  
              if (q > (0xFFFFu << 8)) q = 0xFFFFu << 8;  
              div_q8 = (uint32_t)q;  
            }  
            uint64_t lo_got = (clk * 256ull) / (4ull * (uint64_t)div_q8);  
            if (lo_got >= freq_hz) continue;            // ПЧ должна быть > 0  
            int64_t f_nco = (int64_t)freq_hz - (int64_t)lo_got;  
            if (f_nco <= 0 || f_nco >= (int64_t)RX_NCO_MAX_HZ) continue;  
  
            uint32_t frac   = div_q8 & 0xFFu;  
            bool     is_int = (frac == 0);  
  
            // --- П1: жёсткий запрет дырки в звуке [0..5000] ---  
            if (f_nco >= (int64_t)RX_NCO_FORBID_LO &&  
                f_nco <= (int64_t)RX_NCO_FORBID_HI) {  
              // нелегален → только fallback; «меньшее зло» = ближе к 5 кГц  
              uint64_t bad = (uint64_t)((int64_t)RX_NCO_FORBID_HI - f_nco);  
              if (bad < fb_bad) {  
                fb_bad = bad; fb_div_q8 = div_q8; fb_is_int = is_int;  
                fb_best.refdiv=refdiv; fb_best.p1=p1; fb_best.p2=p2;  
                fb_best.fbdiv=fbdiv;  fb_best.clk_sys_hz=clk;  
                fb_best.vsel=soft_tx_vsel_for(clk);  
                fb_best.is_oc=(clk > VFO_CLK_SYS_NOMINAL);  
              }  
              continue;  
            }  
  
            // --- стоимость легального кандидата ---  
            uint64_t cost = 0;  
            // П2: удаление от сладкой зоны  
            if      (f_nco < (int64_t)RX_NCO_SWEET_LO)  
              cost += (uint64_t)((int64_t)RX_NCO_SWEET_LO - f_nco) * W2;  
            else if (f_nco > (int64_t)RX_NCO_SWEET_HI)  
              cost += (uint64_t)(f_nco - (int64_t)RX_NCO_SWEET_HI) * W2;  
            // П3: штраф за дробность  
            if (!is_int) cost += W3;  
            // П4: дробь подальше от k/8 (= кратных 32 в Q8)  
            if (!is_int) {  
              int32_t m    = (int32_t)frac % 32;        // 0..31  
              int32_t dist = (m < 16) ? m : 32 - m;     // 0..16 до ближайшего k/8  
              cost += (uint64_t)(16 - dist) * W4;       // ближе к k/8 → дороже  
            }  
            // П5: шина ближе к максимуму  
            cost += ((clk_max - clk) / 1000000ull) * W5;  
  
            if (cost < best_cost) {  
              best_cost   = cost;  
              best_div_q8 = div_q8;  
              best_is_int = is_int;  
              best_legal  = true;  
              best.refdiv=refdiv; best.p1=p1; best.p2=p2;  
              best.fbdiv=fbdiv;   best.clk_sys_hz=clk;  
              best.vsel=soft_tx_vsel_for(clk);  
              best.is_oc=(clk > VFO_CLK_SYS_NOMINAL);  
            }  
          }  
        }  
      }  
    }  
  }  
  
// П6: ничего легального — сначала «наименьшее зло», но только если оно  
  // вообще было найдено (fb_bad обновлялся). Пустой скан (ни одного  
  // кандидата в окне — бывает при битом паспорте clk_max) НЕ прошиваем:  
  // оставляем текущую шину и честно отказываемся, иначе LO будет битым.  
  if (!best_legal && fb_bad == UINT64_MAX) {  
    Serial.println("[QUAD] ОШИБКА: скан не дал ни одного кандидата — "  
                   "проверьте vfo_max_clk_sys_hz/pll_ceiling. LO не тронут.");  
    quad_lo_actual_hz = 0;  
    return;  
  }  
  if (!best_legal) {  
    Serial.println("[QUAD] ВНИМАНИЕ: нет кандидата с дыркой вне 0..5 кГц — "  
                   "ставлю наименьшее зло (дырка в полосе!)");  
    best        = fb_best;  
    best_div_q8 = fb_div_q8;  
    best_is_int = fb_is_int;  
  }
  
  // Прошиваем выбранную шину тем же путём, что TX (vreg + PLL + flash SSI).  
  // В отличие от исходника, ядро 1 НЕ сбрасываем — TX-тракт уже остановлен  
  // trx_switch() до вызова этого hook'а; петля loop1() переживёт смену  
  // clk_sys (set_clk_sys сам гасит прерывания и правит BAUDR флэша).  
  pio_sm_set_enabled(quad_pio, quad_sm, false);  
  
  uint32_t vsel = best.vsel;  
  if (vsel == 0 || vsel < (uint32_t)VREG_VOLTAGE_1_10)  
    vsel = soft_tx_vsel_for(best.clk_sys_hz);  
  soft_tx_set_clk_sys(best, vsel);  
  
  // Делитель пересчитываем от РЕАЛЬНОЙ шины (кэш частоты может отставать —  
  // в исходнике это давало уход LO на 2.8 кГц).  
  uint64_t clk_sys_hz = clock_get_hz(clk_sys);  
  uint32_t div_q8;  
  if (best_is_int) {  
    uint64_t D = (clk_sys_hz + 2ull * lo_hz) / (4ull * lo_hz);  
    if (D < 2)      D = 2;  
    if (D > 0xFFFF) D = 0xFFFF;  
    div_q8 = (uint32_t)(D << 8);                        // frac=0 — целый  
  } else {  
    uint64_t q = (clk_sys_hz * 256ull + 2ull * lo_hz) / (4ull * lo_hz);  
    if (q < (2u << 8))      q = 2u << 8;  
    if (q > (0xFFFFu << 8)) q = 0xFFFFu << 8;  
    div_q8 = (uint32_t)q;                               // дробный Q8 сохраняется  
  }  
  quad_lo_actual_hz = (uint32_t)((clk_sys_hz * 256ull) /  
                                 (4ull * (uint64_t)div_q8));  
  
  {  
    int32_t  f_nco  = (int32_t)freq_hz - (int32_t)quad_lo_actual_hz;  
    uint32_t frac   = div_q8 & 0xFFu;  
    bool     sweet  = (f_nco >= (int32_t)RX_NCO_SWEET_LO &&  
                       f_nco <= (int32_t)RX_NCO_SWEET_HI);  
    Serial.printf("[QUAD] clk_sys=%lu, div=%lu+%lu/256 (%s), lo_actual=%lu, "  
                  "f_nco=%ld, sweet=%d\n",  
                  (unsigned long)clk_sys_hz,  
                  (unsigned long)(div_q8 >> 8), (unsigned long)frac,  
                  frac ? "дробный" : "целый",  
                  (unsigned long)quad_lo_actual_hz,  
                  (long)f_nco, sweet ? 1 : 0);  
    if (f_nco > 62500 || f_nco < -62500) {  
      Serial.printf("[QUAD] ВНИМАНИЕ: |f_nco| > 62500 Гц (половина "  
                    "RX_FS_IN_HZ) — приём невозможен\n");  
    }  
  }  
  
  // === Конфигурация SM под квадратуру ===  
  quad_active = false;   // взводим только после завершения переконфигурации  
  
  // TX-пин в input — на pio0 он обслуживается своим SM; здесь просто  
  // страхуемся, что quad-пины не выпустят мусор в смеситель.  
  pio_sm_config cq = pio_get_default_sm_config();  
  sm_config_set_wrap(&cq, quad_offset + 0, quad_offset + 3);  
  sm_config_set_set_pins(&cq, PIN_QUAD_I, 2);   // I=base, Q=base+1  
  // Делитель в конфиге: 24.8 -> float, SDK упакует в регистр при sm_init.  
  sm_config_set_clkdiv(&cq, (float)div_q8 / 256.0f);  
  pio_gpio_init(quad_pio, PIN_QUAD_I);  
  pio_gpio_init(quad_pio, PIN_QUAD_Q);  
  
  pio_sm_init(quad_pio, quad_sm, quad_offset, &cq);  
  // Направление ставим ПОСЛЕДНИМ, после sm_init — пины не должны  
  // выпускать полуинициализированные переходы в смеситель.  
  pio_sm_set_consecutive_pindirs(quad_pio, quad_sm, PIN_QUAD_I, 2, true);  
  pio_sm_restart(quad_pio, quad_sm);            // PC -> wrap_target, чистая фаза  
  pio_sm_set_enabled(quad_pio, quad_sm, true);  
  
  quad_active = true;  
}  
  
// ============================================================================  
// ОСТАНОВКА (vfo_quadrature_stop() исходника, без возврата в TX — это  
// делает trx_switch через движок TX-направления)  
// ============================================================================  
static void quad_stop(void) {  
  pio_sm_set_enabled(quad_pio, quad_sm, false);  
  // Оба квадратурных пина в input — не защемлять последний уровень  
  // в смесителе на время TX-фазы.  
  pio_sm_set_consecutive_pindirs(quad_pio, quad_sm, PIN_QUAD_I, 2, false);  
  gpio_set_dir(PIN_QUAD_I, GPIO_IN);  
  gpio_set_dir(PIN_QUAD_Q, GPIO_IN);  
  quad_active = false;  
}  
  
// У квадратурного LO банка тонов нет — tone/poll пустые.  
static void     quad_tone(uint8_t index) { (void)index; }  
static void     quad_poll(void)          {}   // дизеринг не нужен: чистый clkdiv  
static uint32_t quad_hz(void)            { return quad_lo_actual_hz; }  
  
// ============================================================================  
// ДИАГНОСТИКА (порт vfo_quadrature_debug исходника)  
// ============================================================================  
void soft_rx_quad_debug(void) {  
  uint32_t meas_clk = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) * 1000u;  
  uint32_t cdiv     = quad_pio->sm[quad_sm].clkdiv;  
  
  Serial.printf("[LODBG] clk_sys: cached=%lu kHz  measured=%lu kHz\n",  
                (unsigned long)(clock_get_hz(clk_sys) / 1000ULL),  
                (unsigned long)(meas_clk / 1000u));  
  Serial.printf("[LODBG] PIO=%d SM=%d  clkdiv=0x%08lX (int=%lu frac=%lu)\n",  
                pio_get_index(quad_pio), (int)quad_sm,  
                (unsigned long)cdiv,  
                (unsigned long)(cdiv >> 16),  
                (unsigned long)((cdiv >> 8) & 0xFF));  
  Serial.printf("[LODBG] SM ctrl=0x%08lX enabled=%d  execctrl=0x%08lX\n",  
                (unsigned long)quad_pio->ctrl,  
                (int)((quad_pio->ctrl & (1u << quad_sm)) != 0),  
                (unsigned long)quad_pio->sm[quad_sm].execctrl);  
  Serial.printf("[LODBG] lo_hz=%lu  active=%d\n",  
                (unsigned long)quad_lo_actual_hz, (int)quad_active);  
}  
  
// ============================================================================  
// РЕГИСТРАЦИЯ ДВИЖКА В МЕНЕДЖЕРЕ  
// ============================================================================  
static const VfoEngine s_engine = {  
  quad_init, quad_start, quad_stop, quad_tone, quad_poll, quad_hz  
};  
const VfoEngine *vfo_engine_soft_rx(void) { return &s_engine; }
