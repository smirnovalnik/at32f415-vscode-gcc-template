/**
  ******************************************************************************
  * @file    platform.h
  * @author  Alexander Smirnov
  * @version 1.0
  * @copyright Copyright FDC (C) 2024
  ******************************************************************************
*/

#pragma once

#include <stdint.h>
#include <stdbool.h>

void platform_init(void);

void platform_wdg_init(void);

void platform_wdg_feed(void);

void platform_reset(void);

typedef struct platform_rst_cause_t
{
    bool power_on;
    bool external;
    bool software;
    bool watchdog;
    bool brown_out;
} platform_rst_cause_t;

void platform_get_rst_cause(platform_rst_cause_t* rst_cause);

uint32_t platform_get_system_clock_mhz(void);

void platform_get_uid(uint32_t uid[3]);

uint16_t platform_get_flash_size_kb(void);
