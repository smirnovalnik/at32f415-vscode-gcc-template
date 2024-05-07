/**
  ******************************************************************************
  * @file    platform.c
  * @author  Alexander Smirnov
  * @version 1.0
  * @copyright Copyright FDC (C) 2024
  ******************************************************************************
*/

#include "platform.h"

#include "at32f415_clock.h"
#include "at32f415.h"
#include "hal.h"
#include "systick.h"

#include <stdio.h>

static void uart_print_init(uint32_t baudrate)
{
    gpio_init_type gpio_init_struct;

    #if defined (__GNUC__) && !defined (__clang__)
    setvbuf(stdout, NULL, _IONBF, 0);
    #endif

    crm_periph_clock_enable(USART1_CLK, TRUE);
    crm_periph_clock_enable(USART1_TX_PORT_CLK, TRUE);

    gpio_default_para_init(&gpio_init_struct);

    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = USART1_TX_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(USART1_TX_PORT, &gpio_init_struct);

    usart_init(USART1, baudrate, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(USART1, TRUE);
    usart_enable(USART1, TRUE);
}

void platform_init(void)
{
    system_clock_config();

    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

    systick_init();

    uart_print_init(115200);
}

void platform_wdg_init(void)
{
    wdt_register_write_enable(TRUE);
    wdt_divider_set(WDT_CLK_DIV_32);

    /*
        wdg_timeout = reload_value * (divider / lick_freq ) (s)
        lick_freq    = 40000 Hz
        divider      = 32
        reload_value = 4096

        timeout = 4096 * (32 / 40000 ) = 3.27s
    */
    wdt_reload_value_set(4096 - 1);

    wdt_counter_reload();
    wdt_enable();
}

void platform_wdg_feed(void)
{
    wdt_counter_reload();
}

void platform_reset(void)
{
    nvic_system_reset();
}

void platform_get_rst_cause(platform_rst_cause_t* rst_cause)
{
    if (crm_flag_get(CRM_POR_RESET_FLAG) != RESET)
    {
        crm_flag_clear(CRM_POR_RESET_FLAG);
        rst_cause->power_on = true;
    }
    else
    {
        rst_cause->power_on = false;
    }

    if (crm_flag_get(CRM_NRST_RESET_FLAG) != RESET)
    {
        crm_flag_clear(CRM_NRST_RESET_FLAG);
        rst_cause->external = true;
    }
    else
    {
        rst_cause->external = false;
    }

    if (crm_flag_get(CRM_SW_RESET_FLAG) != RESET)
    {
        crm_flag_clear(CRM_SW_RESET_FLAG);
        rst_cause->software = true;
    }
    else
    {
        rst_cause->software = false;
    }

    if (crm_flag_get(CRM_WDT_RESET_FLAG) != RESET)
    {
        crm_flag_clear(CRM_WDT_RESET_FLAG);
        rst_cause->watchdog = true;
    }
    else
    {
        rst_cause->watchdog = false;
    }

    if (crm_flag_get(CRM_LOWPOWER_RESET_FLAG) != RESET)
    {
        crm_flag_clear(CRM_LOWPOWER_RESET_FLAG);
        rst_cause->brown_out = true;
    }
    else
    {
        rst_cause->brown_out = false;
    }
}

uint32_t platform_get_system_clock_mhz(void)
{
    return system_core_clock / 1000000;
}

void platform_get_uid(uint32_t uid[3])
{
    uid[0] = *(uint32_t *)0x1FFFF7E8;
    uid[1] = *(uint32_t *)0x1FFFF7EC;
    uid[2] = *(uint32_t *)0x1FFFF7F0;
}

uint16_t platform_get_flash_size_kb(void)
{
    return *(uint16_t *)0x1FFFF7E0;
}
