#include "osd.h"

#include <string.h>

#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_comp.h>
#include <stm32g4xx_ll_dac.h>
#include <stm32g4xx_ll_dma.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_rcc.h>
#include <stm32g4xx_ll_spi.h>
#include <stm32g4xx_ll_tim.h>

#include "clock.h"

#include "font8x8_basic.h"
#include "tileset.h"

#define TIMER_CLOCK 170000000

#define BLANK_LEVEL 22
#define WHITE_LEVEL 72

#define LINE_SAMPLE_SIZE (50)

#define LINE_MIN 16
#define LINE_MAX 248

#define SCREEN_BUFFER_HEIGTH 29
#define SCREEN_BUFFER_WIDTH 35

#define SCREEN_BUFFER_SIZE (SCREEN_BUFFER_HEIGTH * SCREEN_BUFFER_WIDTH)

#define TICKS(us, delta) ((us) * (SystemCoreClock / 1000000L) + (delta))

static uint8_t screen_buffer[SCREEN_BUFFER_SIZE];

static void comp_init() {
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  NVIC_SetPriority(COMP1_2_3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(COMP1_2_3_IRQn);

  LL_COMP_InitTypeDef comp_init = {};
  comp_init.InputPlus = LL_COMP_INPUT_MINUS_1_4VREFINT;
  comp_init.InputMinus = LL_COMP_INPUT_PLUS_IO1;
  comp_init.InputHysteresis = LL_COMP_HYSTERESIS_70MV;
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

static void comp_enable() {
  LL_COMP_Enable(COMP1);

  __IO uint32_t wait_loop_index = ((LL_COMP_DELAY_STARTUP_US * (SystemCoreClock / (100000 * 2))) / 10);
  while (wait_loop_index != 0) {
    wait_loop_index--;
  }
}

static void spi_init() {

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

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
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV32;
  SPI_InitStruct.BitOrder = LL_SPI_LSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_DisableNSSPulseMgt(SPI1);

  LL_SPI_Enable(SPI1);
}

void screen_write(uint16_t x, uint16_t y, const char *str) {
  memcpy(screen_buffer + y * SCREEN_BUFFER_WIDTH + x, str, strlen(str));
}

void osd_init() {
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA | LL_AHB2_GRP1_PERIPH_GPIOB);

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  for (uint32_t i = 0; i < SCREEN_BUFFER_SIZE; i++) {
    screen_buffer[i] = 10;
  }

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

typedef enum {
  FIELD_START_BROAD,
  FIELD_START_SHORT,
  FIELD_LINES,
  FIELD_END_SHORT
} video_state_t;

static video_state_t video_state = FIELD_START_BROAD;

uint32_t ticks() {
  static uint32_t total_ticks = 0;
  static uint32_t last_ticks = 0;

  const uint32_t ticks = DWT->CYCCNT;
  if (ticks >= last_ticks) {
    total_ticks += ticks - last_ticks;
  } else {
    total_ticks += (UINT32_MAX + ticks) - last_ticks;
  }

  last_ticks = ticks;
  return total_ticks;
}

void delay_ticks(uint32_t ticks) {
  volatile uint32_t delay = ticks;
  volatile uint32_t start = DWT->CYCCNT;
  while (DWT->CYCCNT - start < delay) {
    __asm("NOP");
  }
}

void COMP1_2_3_IRQHandler(void) {
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_21) != RESET) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_21);

    const volatile uint32_t now = DWT->CYCCNT;

    static volatile uint32_t sync_start = 0;
    static volatile uint8_t did_process = 0;

    if (LL_COMP_ReadOutputLevel(COMP1) == LL_COMP_OUTPUT_LEVEL_LOW) {
      // LOW == SYNC
      sync_start = now;
      did_process = 0;

      //LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_3);

      return;
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
        //LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_3);

        if (line_counter >= LINE_MIN && line_counter < LINE_MAX) {
          while ((ticks() - sync_start) < TICKS(7, 50))
            ;

          const uint32_t line = (line_counter - LINE_MIN);
          const uint32_t line_index = line / 8;

          while (!LL_SPI_IsActiveFlag_TXE(SPI1))
            ;
          LL_SPI_TransmitData8(SPI1, 0);

          for (uint32_t i = 0; i < SCREEN_BUFFER_WIDTH; i++) {
            const uint8_t character = screen_buffer[line_index * SCREEN_BUFFER_WIDTH + i];
            const uint8_t tile = font8x8_basic[character][(line % 8)];
            while (!LL_SPI_IsActiveFlag_TXE(SPI1))
              ;
            LL_SPI_TransmitData8(SPI1, tile);
          }

          while (!LL_SPI_IsActiveFlag_TXE(SPI1))
            ;
          LL_SPI_TransmitData8(SPI1, 0);
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
}