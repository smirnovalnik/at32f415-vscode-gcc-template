/**
 ******************************************************************************
 * @file    led.c
 * @author  Alexander Smirnov
 * @copyright Copyright FDC (C) 2025
 ******************************************************************************
 */

#include <stdint.h>

#include "hal.h"
#include "led.h"

static volatile led_state_t _state = LED_OFF;
static volatile int32_t _oc_val = 0;

#define CNT_MAX      (1000 - 1)
#define FAST_INC_VAL 20 // 200 ms
#define SLOW_INC_VAL 4  // 500 ms

int led_init(void)
{
    crm_periph_clock_enable(LED_PORT_CLK, TRUE);
    crm_periph_clock_enable(LED_TMR_CLK, TRUE);

    // gpio
    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);

    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins = LED_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(LED_PORT, &gpio_init_struct);

    gpio_bits_set(LED_PORT, LED_PIN);

    // timer
    // tmr_clk = 500000 Hz
    // div = (apb1_freq * 2 / tmr_clk) - 1
    // pr = 2000 - 1
    // ovfclk = 500 Hz
    crm_clocks_freq_type crm_clocks_freq_struct = {0};
    crm_clocks_freq_get(&crm_clocks_freq_struct);
    uint16_t div_value = (uint16_t)(2 * crm_clocks_freq_struct.apb1_freq / 500000) - 1;

    // tmr base
    tmr_base_init(LED_TMR, CNT_MAX, div_value);
    tmr_cnt_dir_set(LED_TMR, TMR_COUNT_UP);
    tmr_clock_source_div_set(LED_TMR, TMR_CLOCK_DIV1);

    // output compare channel1
    tmr_output_config_type tmr_oc_init_structure;
    tmr_output_default_para_init(&tmr_oc_init_structure);
    tmr_oc_init_structure.oc_mode = TMR_OUTPUT_CONTROL_OFF;
    tmr_oc_init_structure.oc_idle_state = FALSE;
    tmr_oc_init_structure.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
    tmr_oc_init_structure.oc_output_state = FALSE;
    tmr_output_channel_config(LED_TMR, TMR_SELECT_CHANNEL_1, &tmr_oc_init_structure);
    tmr_channel_value_set(LED_TMR, TMR_SELECT_CHANNEL_1, 0);

    tmr_counter_enable(LED_TMR, TRUE);

    tmr_interrupt_enable(LED_TMR, TMR_OVF_INT, TRUE);

    nvic_irq_enable(LED_TMR_IRQN, LED_TMR_IRQ_PRIORITY, 0);

    return 0;
}

int led_set(led_state_t state)
{
    if (state == LED_ON)
    {
        gpio_bits_reset(LED_PORT, LED_PIN);
    }
    else if (state == LED_OFF)
    {
        gpio_bits_set(LED_PORT, LED_PIN);
    }
    else if (state == LED_ON_SMOOTH_FAST || state == LED_ON_SMOOTH_SLOW)
    {
        _oc_val = 0;
    }
    else if (state == LED_OFF_SMOOTH_FAST || state == LED_OFF_SMOOTH_SLOW)
    {
        _oc_val = CNT_MAX - 1;
    }

    _state = state;
    return 0;
}

led_state_t led_get(void)
{
    return _state;
}

void LED_IRQ_HANDLER(void)
{
    // ovf
    if (tmr_interrupt_flag_get(LED_TMR, TMR_OVF_FLAG) != RESET)
    {
        tmr_flag_clear(LED_TMR, TMR_OVF_FLAG);

        if (_state == LED_ON_SMOOTH_FAST)
        {
            _oc_val += FAST_INC_VAL;
            if (_oc_val >= CNT_MAX)
                _state = LED_ON;
        }
        else if (_state == LED_ON_SMOOTH_SLOW)
        {
            _oc_val += SLOW_INC_VAL;
            if (_oc_val >= CNT_MAX)
                _state = LED_ON;
        }
        else if (_state == LED_OFF_SMOOTH_FAST)
        {
            _oc_val -= FAST_INC_VAL;
            if (_oc_val <= 0)
                _state = LED_OFF;
        }
        else if (_state == LED_OFF_SMOOTH_SLOW)
        {
            _oc_val -= SLOW_INC_VAL;
            if (_oc_val <= 0)
                _state = LED_OFF;
        }

        if (_state == LED_OFF)
        {
            gpio_bits_set(LED_PORT, LED_PIN); // led off
            return;
        }

        gpio_bits_reset(LED_PORT, LED_PIN); // led on

        if (_state == LED_ON)
        {
            return;
        }

        tmr_channel_value_set(LED_TMR, TMR_SELECT_CHANNEL_1, _oc_val);
        tmr_flag_clear(LED_TMR, TMR_C1_FLAG);
        tmr_interrupt_enable(LED_TMR, TMR_C1_INT, TRUE);
    }
    // oc
    if (tmr_interrupt_flag_get(LED_TMR, TMR_C1_FLAG) != RESET)
    {
        tmr_flag_clear(LED_TMR, TMR_C1_FLAG);
        tmr_interrupt_enable(LED_TMR, TMR_C1_INT, FALSE);
        gpio_bits_set(LED_PORT, LED_PIN); // led off
    }
}
