#include "clock.h"

#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_pwr.h>
#include <stm32g4xx_ll_rcc.h>
#include <stm32g4xx_ll_utils.h>

#include "osd.h"

int main() {
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  LL_PWR_DisableUCPDDeadBattery();

  SystemClock_Config();

  osd_init();

  while (1) {
    osd_update();
  }
}