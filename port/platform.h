/**
  ******************************************************************************
  * @file    platform.h
  * @author  Alexander Smirnov
  * @copyright Copyright FDC (C) 2024
  ******************************************************************************
*/

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "lfs.h"

void platform_init(void);

void platform_wdg_init(void);

void platform_wdg_feed(void);

void platform_reset(void);

void platform_sleep(void);

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

void clock_failure_detection_handler(void);

typedef void (*fn_t)(void);

void power_voltage_monitor_enable(fn_t cb);

void power_voltage_monitor_handler(void);

void platform_get_mcu_uid(uint32_t uid[3]);

uint32_t platform_get_uid(void);

uint16_t platform_get_flash_size_kb(void);

void platform_flash_protection_enable(void);

bool platform_is_flash_protection_enabled(void);

uint32_t platform_get_lfs_start(void);

lfs_t* platform_get_lfs(void);

struct lfs_config* platform_get_lfs_config(void);

int platform_fputc(int ch);
