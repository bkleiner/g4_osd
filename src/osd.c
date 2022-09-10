#include "osd.h"

#include <string.h>

#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_comp.h>
#include <stm32g4xx_ll_dac.h>
#include <stm32g4xx_ll_dma.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_i2c.h>
#include <stm32g4xx_ll_rcc.h>
#include <stm32g4xx_ll_spi.h>
#include <stm32g4xx_ll_tim.h>

#include "clock.h"
#include "video.h"

#include "font8x8_basic.h"
#include "tileset.h"

#define TIMER_CLOCK 170000000

#define BLANK_LEVEL 22
#define WHITE_LEVEL 72

#define LINE_MIN 16
#define LINE_MAX 248

#define SCREEN_BUFFER_HEIGTH 29
#define SCREEN_BUFFER_WIDTH 74

#define SCREEN_BUFFER_SIZE (SCREEN_BUFFER_HEIGTH * SCREEN_BUFFER_WIDTH)

#define TICKS(us, delta) ((us) * (SystemCoreClock / 1000000L) + (delta))

static uint8_t screen_buffer[SCREEN_BUFFER_SIZE];
static volatile uint8_t line_buffer[SCREEN_BUFFER_WIDTH];

static void comp_init() {
  {
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_DAC1);

    LL_DAC_SetSignedFormat(DAC1, LL_DAC_CHANNEL_1, LL_DAC_SIGNED_FORMAT_DISABLE);
    LL_DAC_SetHighFrequencyMode(DAC1, LL_DAC_HIGH_FREQ_MODE_DISABLE);

    LL_DAC_InitTypeDef dac_init = {0};
    dac_init.TriggerSource = LL_DAC_TRIG_SOFTWARE;
    dac_init.TriggerSource2 = LL_DAC_TRIG_SOFTWARE;
    dac_init.WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE;
    dac_init.OutputBuffer = LL_DAC_OUTPUT_BUFFER_ENABLE;
    dac_init.OutputConnection = LL_DAC_OUTPUT_CONNECT_INTERNAL;
    dac_init.OutputMode = LL_DAC_OUTPUT_MODE_NORMAL;
    LL_DAC_Init(DAC1, DAC2_CHANNEL_1, &dac_init);
    LL_DAC_DisableTrigger(DAC1, LL_DAC_CHANNEL_1);
    LL_DAC_DisableDMADoubleDataMode(DAC1, LL_DAC_CHANNEL_1);

    LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);

    __IO uint32_t wait_loop_index = ((LL_DAC_DELAY_STARTUP_VOLTAGE_SETTLING_US * (SystemCoreClock / (100000 * 2))) / 10);
    while (wait_loop_index != 0) {
      wait_loop_index--;
    }

    LL_DAC_EnableTrigger(DAC1, LL_DAC_CHANNEL_1);

    LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, 450);
    LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_1);
  }
  {

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    NVIC_SetPriority(COMP1_2_3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(COMP1_2_3_IRQn);

    LL_COMP_InitTypeDef comp_init = {};
    comp_init.InputPlus = LL_COMP_INPUT_PLUS_IO1;
    comp_init.InputMinus = LL_COMP_INPUT_MINUS_DAC1_CH1;
    comp_init.InputHysteresis = LL_COMP_HYSTERESIS_MEDIUM;
    comp_init.OutputPolarity = LL_COMP_OUTPUTPOL_NONINVERTED;
    comp_init.OutputBlankingSource = LL_COMP_BLANKINGSRC_NONE;
    LL_COMP_Init(COMP1, &comp_init);

    __IO uint32_t wait_loop_index = ((LL_COMP_DELAY_VOLTAGE_SCALER_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
    while (wait_loop_index != 0) {
      wait_loop_index--;
    }

    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_21);
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_21);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_21);

    LL_EXTI_DisableEvent_0_31(LL_EXTI_LINE_21);

    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_21);
  }
}

static void comp_enable() {
  LL_COMP_Enable(COMP1);

  __IO uint32_t wait_loop_index = ((LL_COMP_DELAY_STARTUP_US * (SystemCoreClock / (100000 * 2))) / 10);
  while (wait_loop_index != 0) {
    wait_loop_index--;
  }
}

static void spi_init() {
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  // SPI1_SCK
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // SPI1_MOSI
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  SPI_InitStruct.TransferDirection = LL_SPI_HALF_DUPLEX_TX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;
  SPI_InitStruct.BitOrder = LL_SPI_LSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_DisableNSSPulseMgt(SPI1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1 | LL_AHB1_GRP1_PERIPH_DMAMUX1);

  LL_DMA_InitTypeDef dma_init = {0};
  dma_init.PeriphOrM2MSrcAddress = LL_SPI_DMA_GetRegAddr(SPI1);
  dma_init.MemoryOrM2MDstAddress = (uint32_t)&line_buffer;
  dma_init.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  dma_init.Mode = LL_DMA_MODE_NORMAL;
  dma_init.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  dma_init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  dma_init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  dma_init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  dma_init.NbData = SCREEN_BUFFER_WIDTH;
  dma_init.PeriphRequest = LL_DMAMUX_REQ_SPI1_TX;
  dma_init.Priority = LL_DMA_PRIORITY_HIGH;
  LL_DMA_Init(DMA1, LL_DMA_CHANNEL_3, &dma_init);

  NVIC_SetPriority(DMA1_Channel3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x1, 0));
  NVIC_EnableIRQ(DMA1_Channel3_IRQn);

  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);

  LL_SPI_Enable(SPI1);
}

void screen_write(uint16_t x, uint16_t y, const char *str) {
  memcpy(screen_buffer + y * SCREEN_BUFFER_WIDTH + x, str, strlen(str));
}

void osd_init() {
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA | LL_AHB2_GRP1_PERIPH_GPIOB);

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  for (uint32_t i = 0; i < SCREEN_BUFFER_SIZE; i++) {
    screen_buffer[i] = 10;
  }

  line_buffer[0] = 0x0;
  for (uint32_t i = 1; i < SCREEN_BUFFER_WIDTH - 1; i++) {
    line_buffer[i] = 0xFF;
  }
  line_buffer[SCREEN_BUFFER_WIDTH - 1] = 0x0;

  spi_init();

  comp_init();
  comp_enable();
}

void osd_update() {
  static uint16_t loop = 0;
  static uint16_t left = 0;

  for (uint32_t i = 0; i < SCREEN_BUFFER_SIZE; i++) {
    screen_buffer[i] = 10;
  }

  screen_write(left, loop, "QUCIKSILVER");
  loop++;
  left++;

  if (loop == SCREEN_BUFFER_HEIGTH) {
    loop = 0;
  }

  if (left == SCREEN_BUFFER_WIDTH) {
    left = 0;
  }
}

static video_state_t video_state = FIELD_START_BROAD;

void COMP1_2_3_IRQHandler(void) {
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_21) == RESET) {
    return;
  }
  LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_21);

  const volatile uint32_t now = DWT->CYCCNT;

  static volatile uint32_t sync_start = 0;
  static volatile uint8_t did_process = 0;

  if (LL_COMP_ReadOutputLevel(COMP1) == LL_COMP_OUTPUT_LEVEL_LOW) {
    // LOW == SYNC
    sync_start = now;
    did_process = 0;

    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9);

    return;
  } else {
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9);
  }

  static uint32_t line_counter = 0;
  static uint32_t frame_counter = 0;

  static uint32_t broad_counter = 0;
  static uint32_t short_counter = 0;

  if (did_process == 1) {
    return;
  }

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

  case FIELD_START_SHORT:
    if ((now - sync_start) <= TICKS(2, 100)) {
      short_counter++;
    } else {
      video_state = FIELD_START_BROAD;
      short_counter = 0;
    }
    if (short_counter > 5) {
      video_state = FIELD_LINES;
      short_counter = 0;
    }
    did_process = 1;
    break;

  case FIELD_LINES:
    if ((now - sync_start) > TICKS(3, 100)) {
      if (line_counter >= LINE_MIN && line_counter < LINE_MAX) {
        while ((ticks() - sync_start) < TICKS(7, 100))
          ;

        LL_SPI_EnableDMAReq_TX(SPI1);
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
      }

      line_counter++;
    } else {
      line_counter = 0;
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

      frame_counter++;
    }
    did_process = 1;
    break;

  default:
    break;
  }
}

void DMA1_Channel3_IRQHandler() {
  if (LL_DMA_IsActiveFlag_TC3(DMA1)) {
    LL_DMA_ClearFlag_TC3(DMA1);

    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_SPI_DisableDMAReq_TX(SPI1);

    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&line_buffer);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_3, LL_SPI_DMA_GetRegAddr(SPI1));
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, SCREEN_BUFFER_WIDTH);
  }
}