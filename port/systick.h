/**
  ******************************************************************************
  * @file    systick.h
  * @author  Alexander Smirnov
  * @copyright Copyright FDC (C) 2024
  ******************************************************************************
*/

#pragma once

#include <stdint.h>

void systick_init(void);

uint32_t systick_get_tick_sec(void);

uint32_t systick_get_tick_ms(void);

void systick_delay_sec(uint32_t sec);

void systick_delay_ms(uint32_t ms);

void systick_interrupt_handler(void);
