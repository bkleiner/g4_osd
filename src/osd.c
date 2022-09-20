#include "osd.h"

#include <string.h>

#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_gpio.h>

#include "clock.h"
#include "osd_timer.h"
#include "osd_video.h"

uint8_t screen_buffer[SCREEN_BUFFER_SIZE];

static void screen_clear() {
  memset(screen_buffer, ' ', SCREEN_BUFFER_SIZE);
}

static void screen_write(uint16_t x, uint16_t y, const char *str) {
  memcpy(screen_buffer + y * SCREEN_BUFFER_WIDTH + x, str, strlen(str));
}

void osd_init() {
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA | LL_AHB2_GRP1_PERIPH_GPIOB);

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  osd_timer_init();
  osd_video_init();

  screen_clear();

  for (uint8_t row = 0; row < 4; row++) {
    uint8_t start = 160 + row * 24;
    for (uint8_t i = 0; i < 24; i++) {
      screen_buffer[(row)*SCREEN_BUFFER_WIDTH + i + 3] = start + i;
    }
  }
}

void osd_update() {
  /*
  static uint16_t loop = 0;
  static uint16_t left = 0;

  loop++;
  left++;

  screen_clear();
  screen_write(10, 10, "QUICKSILVER");

  if (loop == SCREEN_BUFFER_HEIGTH) {
    loop = 0;
  }

  if (left == SCREEN_BUFFER_WIDTH) {
    left = 0;
  }
  */

  osd_video_update();
}
