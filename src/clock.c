#include "clock.h"

#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_pwr.h>
#include <stm32g4xx_ll_rcc.h>
#include <stm32g4xx_ll_system.h>
#include <stm32g4xx_ll_utils.h>

void SystemClock_Config(void) {
  /* Enable voltage range 1 boost mode for frequency above 150 Mhz */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_EnableRange1BoostMode();
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* Set Flash Latency */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);

  /* HSI already enabled at reset */

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_4, 85, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable();
  while (LL_RCC_PLL_IsReady() != 1) {
  };

  /* PLL system Output activation */
  LL_RCC_PLL_EnableDomain_SYS();

  /* Sysclk activation on the main PLL */
  /* Intermediate AHB prescaler 2 when target frequency clock is higher than 80 MHz */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
  };

  /* Insure 1µs transition state at intermediate medium speed clock based on DWT */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;
  while (DWT->CYCCNT < 100)
    ;

  /* AHB prescaler 1 */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms in using frequency set to 170MHz */
  /* This frequency can be calculated through LL RCC macro */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ(HSI_VALUE,
                                  LL_RCC_PLLM_DIV_4, 85, LL_RCC_PLLR_DIV_2) */
  LL_Init1msTick(170000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(170000000);
}

uint32_t micros() {
  static uint32_t total_micros = 0;
  static uint32_t last_micros = 0;

  const uint32_t micros = DWT->CYCCNT / (SystemCoreClock / 1000000L);
  if (micros >= last_micros) {
    total_micros += micros - last_micros;
  } else {
    total_micros += ((UINT32_MAX / (SystemCoreClock / 1000000L)) + micros) - last_micros;
  }

  last_micros = micros;
  return total_micros;
}

uint32_t millis() {
  static uint32_t total_millis = 0;
  static uint32_t last_millis = 0;

  const uint32_t millis = micros() / 1000;
  if (millis >= last_millis) {
    total_millis += millis - last_millis;
  } else {
    total_millis += ((UINT32_MAX / 1000) + millis) - last_millis;
  }

  last_millis = millis;
  return total_millis;
}

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

void delay_us(uint32_t us) {
  volatile uint32_t delay = us * (SystemCoreClock / 1000000L);
  volatile uint32_t start = DWT->CYCCNT;
  while (DWT->CYCCNT - start < delay) {
    __asm("NOP");
  }
}

void delay_ms(uint32_t ms) {
  delay_us(ms * 1000);
}

void delay_ticks(uint32_t ticks) {
  volatile uint32_t delay = ticks;
  volatile uint32_t start = DWT->CYCCNT;
  while (DWT->CYCCNT - start < delay) {
    __asm("NOP");
  }
}