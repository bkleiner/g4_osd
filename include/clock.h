#pragma once

#include <stdint.h>

void SystemClock_Config(void);

uint32_t micros();
uint32_t millis();
uint32_t ticks();

void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void delay_ticks(uint32_t ticks);