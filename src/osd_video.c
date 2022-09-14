#include "osd_video.h"

#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_dac.h>
#include <stm32g4xx_ll_dma.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_tim.h>

#include "font.h"

#define TIMER_RELOAD 22

static volatile uint8_t line_buffer_index = 0;
static volatile uint32_t line_buffer[2][LINE_BUFFER_SIZE];

static volatile uint8_t fill_request = 0;
static volatile uint32_t line;

volatile uint8_t video_ouput_active = 0;

extern uint8_t screen_buffer[SCREEN_BUFFER_SIZE];

void black_level_update(uint32_t mv) {
  const uint32_t level = (mv * 0x0FFF) / (3.3 * 1000);

  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, level);
  LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_1);
}

static void dac_init() {
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_DAC1);

  LL_DAC_SetSignedFormat(DAC1, LL_DAC_CHANNEL_1, LL_DAC_SIGNED_FORMAT_DISABLE);

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  LL_DAC_InitTypeDef dac_init = {0};
  dac_init.TriggerSource = LL_DAC_TRIG_SOFTWARE;
  dac_init.TriggerSource2 = LL_DAC_TRIG_SOFTWARE;
  dac_init.WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE;
  dac_init.OutputBuffer = LL_DAC_OUTPUT_BUFFER_ENABLE;
  dac_init.OutputConnection = LL_DAC_OUTPUT_CONNECT_GPIO;
  dac_init.OutputMode = LL_DAC_OUTPUT_MODE_NORMAL;
  LL_DAC_Init(DAC1, LL_DAC_CHANNEL_1, &dac_init);
  LL_DAC_DisableTrigger(DAC1, LL_DAC_CHANNEL_1);
  LL_DAC_DisableDMADoubleDataMode(DAC1, LL_DAC_CHANNEL_1);

  LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);

  __IO uint32_t wait_loop_index = ((LL_DAC_DELAY_STARTUP_VOLTAGE_SETTLING_US * (SystemCoreClock / (100000 * 2))) / 10);
  while (wait_loop_index != 0) {
    wait_loop_index--;
  }

  LL_DAC_EnableTrigger(DAC1, LL_DAC_CHANNEL_1);
  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, 900);
  LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_1);
}

static void bitbang_init() {
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;

  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  LL_TIM_InitTypeDef tim_init = {0};
  tim_init.Autoreload = (TIMER_RELOAD - 1);
  tim_init.Prescaler = 0;
  tim_init.ClockDivision = 0;
  tim_init.CounterMode = LL_TIM_COUNTERMODE_UP;
  LL_TIM_Init(TIM1, &tim_init);
  LL_TIM_EnableARRPreload(TIM1);

  LL_TIM_OC_InitTypeDef tim_oc_init;
  LL_TIM_OC_StructInit(&tim_oc_init);
  tim_oc_init.OCMode = LL_TIM_OCMODE_PWM1;
  tim_oc_init.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
  tim_oc_init.OCState = LL_TIM_OCSTATE_ENABLE;
  tim_oc_init.OCPolarity = LL_TIM_OCPOLARITY_LOW;
  tim_oc_init.CompareValue = 10;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &tim_oc_init);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1 | LL_AHB1_GRP1_PERIPH_DMAMUX1);

  LL_DMA_InitTypeDef dma_init = {0};
  dma_init.PeriphOrM2MSrcAddress = (uint32_t)&GPIOB->BSRR;
  dma_init.MemoryOrM2MDstAddress = (uint32_t)&line_buffer[0];
  dma_init.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  dma_init.Mode = LL_DMA_MODE_NORMAL;
  dma_init.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  dma_init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  dma_init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
  dma_init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
  dma_init.NbData = LINE_BUFFER_SIZE;
  dma_init.PeriphRequest = LL_DMAMUX_REQ_TIM1_CH1;
  dma_init.Priority = LL_DMA_PRIORITY_VERYHIGH;
  LL_DMA_Init(DMA1, LL_DMA_CHANNEL_3, &dma_init);

  NVIC_SetPriority(DMA1_Channel3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x1, 0));
  NVIC_EnableIRQ(DMA1_Channel3_IRQn);

  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);

  LL_TIM_EnableCounter(TIM1);
}

void osd_video_init() {
  dac_init();
  bitbang_init();
}

#pragma GCC push_options
#pragma GCC optimize("O3")

void osd_video_update() {
  if (fill_request) {

    const uint32_t line_index = line / CHARCTER_HEIGHT;
    const uint32_t line_row = (line % CHARCTER_HEIGHT);

    uint32_t offset = 1;
    line_buffer[line_buffer_index][0] = (LL_GPIO_PIN_7 << 16) | (LL_GPIO_PIN_5 << 16);
    for (uint32_t i = 0; i < SCREEN_BUFFER_WIDTH; i++) {
      const uint8_t character = screen_buffer[line_index * SCREEN_BUFFER_WIDTH + i];

      const uint16_t color0 = font_data[0][character][line_row];
      const uint16_t color1 = font_data[1][character][line_row];

      for (int32_t j = 2; j < 14; j++) {
        uint32_t val = 0;
        val |= ((color0 >> j) & 0x1) ? LL_GPIO_PIN_5 : (LL_GPIO_PIN_5 << 16);
        val |= ((color1 >> j) & 0x1) ? LL_GPIO_PIN_7 : (LL_GPIO_PIN_7 << 16);
        line_buffer[line_buffer_index][offset] = val;

        offset++;
      }
    }
    line_buffer[line_buffer_index][LINE_BUFFER_SIZE - 1] = (LL_GPIO_PIN_7 << 16) | (LL_GPIO_PIN_5 << 16);

    fill_request = 0;
  }
}

void osd_video_fire(const uint32_t current_line) {
  LL_TIM_DisableCounter(TIM1);
  LL_TIM_SetCounter(TIM1, 0xFFFF - 29 * TIMER_RELOAD);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
  LL_TIM_EnableDMAReq_CC1(TIM1);
  LL_TIM_EnableCounter(TIM1);

  video_ouput_active = 1;
  line_buffer_index = !line_buffer_index;
  line = current_line;
  fill_request = 1;
}
#pragma GCC pop_options

void DMA1_Channel3_IRQHandler() {
  if (LL_DMA_IsActiveFlag_TC3(DMA1)) {
    LL_DMA_ClearFlag_TC3(DMA1);

    LL_TIM_DisableDMAReq_CC1(TIM1);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);

    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&line_buffer[line_buffer_index]);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&GPIOB->BSRR);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, LINE_BUFFER_SIZE);

    video_ouput_active = 0;
  }
}