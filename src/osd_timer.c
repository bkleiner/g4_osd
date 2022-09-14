#include "osd_timer.h"

#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_comp.h>
#include <stm32g4xx_ll_dac.h>
#include <stm32g4xx_ll_gpio.h>

#include "clock.h"
#include "osd_video.h"

#define TIMER_CLOCK 170000000

#define DETECT_TIMEOUT 500
#define DETECT_INCREMENT 25

#define TICKS(us, delta) ((us) * (SystemCoreClock / 1000000L) + (delta))

static osd_timer_state_t video_state = FIELD_START_BROAD;
static volatile uint32_t line_counter = 0;
uint16_t sync_level = 0;

extern volatile uint8_t video_ouput_active;

static void dac_init() {
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_DAC1);

  LL_DAC_SetSignedFormat(DAC1, LL_DAC_CHANNEL_2, LL_DAC_SIGNED_FORMAT_DISABLE);

  LL_DAC_InitTypeDef dac_init = {0};
  dac_init.TriggerSource = LL_DAC_TRIG_SOFTWARE;
  dac_init.TriggerSource2 = LL_DAC_TRIG_SOFTWARE;
  dac_init.WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE;
  dac_init.OutputBuffer = LL_DAC_OUTPUT_BUFFER_ENABLE;
  dac_init.OutputConnection = LL_DAC_OUTPUT_CONNECT_INTERNAL;
  dac_init.OutputMode = LL_DAC_OUTPUT_MODE_NORMAL;
  LL_DAC_Init(DAC1, LL_DAC_CHANNEL_2, &dac_init);
  LL_DAC_DisableTrigger(DAC1, LL_DAC_CHANNEL_2);
  LL_DAC_DisableDMADoubleDataMode(DAC1, LL_DAC_CHANNEL_2);

  LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_2);

  __IO uint32_t wait_loop_index = ((LL_DAC_DELAY_STARTUP_VOLTAGE_SETTLING_US * (SystemCoreClock / (100000 * 2))) / 10);
  while (wait_loop_index != 0) {
    wait_loop_index--;
  }

  LL_DAC_EnableTrigger(DAC1, LL_DAC_CHANNEL_2);
}

static void comp_init() {
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  LL_COMP_InitTypeDef comp_init = {};
  comp_init.InputPlus = LL_COMP_INPUT_PLUS_IO1;
  comp_init.InputMinus = LL_COMP_INPUT_MINUS_DAC1_CH2;
  comp_init.InputHysteresis = LL_COMP_HYSTERESIS_50MV;
  comp_init.OutputPolarity = LL_COMP_OUTPUTPOL_NONINVERTED;
  comp_init.OutputBlankingSource = LL_COMP_BLANKINGSRC_NONE;
  LL_COMP_Init(COMP2, &comp_init);

  __IO uint32_t wait_loop_index_stab = ((LL_COMP_DELAY_VOLTAGE_SCALER_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while (wait_loop_index_stab != 0) {
    wait_loop_index_stab--;
  }

  LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_22);
  LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_22);
  LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_22);

  LL_EXTI_DisableEvent_0_31(LL_EXTI_LINE_22);

  LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_22);

  LL_COMP_Enable(COMP2);

  __IO uint32_t wait_loop_index_start = ((LL_COMP_DELAY_STARTUP_US * (SystemCoreClock / (100000 * 2))) / 10);
  while (wait_loop_index_start != 0) {
    wait_loop_index_start--;
  }
}

void sync_level_update(uint32_t mv) {
  const uint32_t level = (mv * 0x0FFF) / (3.3 * 1000);

  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_2, level);
  LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_2);
}

static void comp_detect() {
  delay_ms(50);

  uint16_t min_level = 0;

  for (uint16_t mv = 0; mv < 700; mv += 5) {
    sync_level_update(mv);

    uint16_t line_counter = 0;

    volatile uint32_t start = DWT->CYCCNT;

    // lets try to detect approximately 100 lines
    // pal has 15625 lines per second = ~6.4ms per 100 lines
    // credits to fishpepper for coming up with this method
    while ((DWT->CYCCNT - start) < TICKS(6000, 0)) {
      if (LL_COMP_ReadOutputLevel(COMP2) == LL_COMP_OUTPUT_LEVEL_LOW) {
        while (LL_COMP_ReadOutputLevel(COMP2) == LL_COMP_OUTPUT_LEVEL_LOW && (DWT->CYCCNT - start) < TICKS(6000, 0))
          ;

        line_counter++;
      }
    }

    // try to find min level:
    if ((min_level == 0) && (line_counter > 30)) {
      // take this as min level
      min_level = mv;
    } else if (line_counter > 120) {
      sync_level = min_level + 0.6 * (mv - min_level);
      break;
    }
  }

  if (sync_level == 0) {
    // bad detection -> fallback
    sync_level = 100;
  }

  sync_level_update(sync_level);

  NVIC_SetPriority(COMP1_2_3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(COMP1_2_3_IRQn);
}

void osd_timer_init() {
  dac_init();

  comp_init();
  comp_detect();
}

#pragma GCC push_options
#pragma GCC optimize("O3")

void COMP1_2_3_IRQHandler(void) {
  volatile uint32_t now = DWT->CYCCNT;

  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_22) == RESET) {
    return;
  }
  LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_22);

  static volatile uint32_t sync_start = 0;
  static volatile uint8_t did_process = 0;

  if (video_ouput_active) {
    return;
  }

  if (LL_COMP_ReadOutputLevel(COMP2) == LL_COMP_OUTPUT_LEVEL_LOW) {
    // LOW == SYNC
    sync_start = now;
    did_process = 0;

    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9);
    return;
  }

  if ((now - sync_start) <= TICKS(1, 200)) {
    // invalid sync
    did_process = 1;
  }

  if (did_process == 1) {
    return;
  }

  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10);
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9);

  static uint32_t broad_counter = 0;
  static uint32_t short_counter = 0;

  switch (video_state) {
  case FIELD_START_BROAD:
    if ((now - sync_start) > TICKS(22, 0)) {
      broad_counter++;
    } else {
      broad_counter = 0;
    }
    if (broad_counter > 5) {
      video_state = FIELD_START_SHORT;
      broad_counter = 0;
    }
    did_process = 1;
    break;

  case FIELD_START_SHORT: {
    static uint32_t last_short = 0;

    if ((now - sync_start) <= TICKS(2, 100)) {
      short_counter++;
      last_short = now;
    } else {
      video_state = FIELD_LINES;
      if ((now - last_short) > TICKS(36, 0)) {
        // EVEN Field
        line_counter = 2;
      } else {
        // ODD Field
        line_counter = 1;
      }
      short_counter = 0;
    }

    did_process = 1;
    break;
  }

  case FIELD_LINES:
    if ((now - sync_start) > TICKS(3, 100)) {
      if (line_counter >= LINE_MIN && line_counter < LINE_MAX) {
        osd_video_fire(line_counter - LINE_MIN);
        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10);
      }

      line_counter += 2;
    } else {
      line_counter = 0;
      short_counter++;
      video_state = FIELD_END_SHORT;
    }
    did_process = 1;
    break;

  case FIELD_END_SHORT:
    if ((now - sync_start) <= TICKS(3, 100)) {
      short_counter++;
    } else {
      short_counter = 0;
    }
    if (short_counter >= 5) {
      video_state = FIELD_START_BROAD;
      short_counter = 0;
    }
    did_process = 1;
    break;

  default:
    break;
  }
}

#pragma GCC pop_options