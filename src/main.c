#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>

#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_dac.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_pwr.h>
#include <stm32g4xx_ll_rcc.h>
#include <stm32g4xx_ll_usart.h>
#include <stm32g4xx_ll_utils.h>

#include "clock.h"
#include "osd.h"

void uart_init() {
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_USART_InitTypeDef USART_InitStruct = {0};
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART2);

  LL_USART_Enable(USART2);
  while ((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
    ;
}

bool uart_get_byte(uint8_t *byte) {
  if (!LL_USART_IsActiveFlag_RXNE(USART2)) {
    return false;
  }

  *byte = LL_USART_ReceiveData8(USART2);
  return true;
}

void uart_write_byte(uint8_t byte) {
  while (!LL_USART_IsActiveFlag_TXE(USART2))
    ;

  LL_USART_TransmitData8(USART2, byte);
}

void uart_printf(const char *fmt, ...) {
  va_list args1;
  va_start(args1, fmt);

  va_list args2;
  va_copy(args2, args1);

  const uint32_t len = vsnprintf(NULL, 0, fmt, args1);
  char buf[len + 1];
  va_end(args1);

  vsnprintf(buf, len + 1, fmt, args2);
  va_end(args2);

  for (uint32_t i = 0; i < len; i++) {
    uart_write_byte(buf[i]);
  }
}

extern uint16_t sync_level;
extern void sync_level_update(uint32_t mv);
extern void black_level_update(uint32_t mv);

void uart_update() {
  uint8_t cmd = 0;
  if (!uart_get_byte(&cmd)) {
    return;
  }

  uart_printf("%c\r\n", cmd);

  static uint32_t level = 0;

  switch (cmd) {
  case 'w':
    sync_level += 1;
    sync_level_update(sync_level);
    uart_printf("%d\r\n", sync_level);
    break;

  case 's':
    if (sync_level >= 1) {
      sync_level -= 1;
      sync_level_update(sync_level);
    }
    uart_printf("%d\r\n", sync_level);
    break;

  case 'e':
    level += 1;
    black_level_update(level);
    uart_printf("%d\r\n", level);
    break;

  case 'd':
    if (level >= 1) {
      level -= 1;
      black_level_update(level);
    }
    uart_printf("%d\r\n", level);
    break;

  default:
    break;
  }
}

int main() {
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  LL_PWR_DisableUCPDDeadBattery();

  SystemClock_Config();

  osd_init();
  uart_init();

  while (1) {
    osd_update();
    uart_update();
  }
}