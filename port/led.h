/**
  ******************************************************************************
  * @file    led.h
  * @author  Alexander Smirnov
  * @copyright Copyright FDC (C) 2025
  ******************************************************************************
*/

#pragma once

#include <stdint.h>

int led_init(void);

typedef enum
{
    LED_OFF = 0,
    LED_ON = 1,
    LED_ON_SMOOTH_FAST = 2,
    LED_ON_SMOOTH_SLOW = 3,
    LED_OFF_SMOOTH_FAST = 4,
    LED_OFF_SMOOTH_SLOW = 5
} led_state_t;

// Set led state
int led_set(led_state_t state);

// Get led state
led_state_t led_get(void);
