/**
 ************************************************************************************************
 * @file    hal.h
 * @author  Smirnov Alexander (SA)
 * @copyright Copyright FDC (C) 2024
 ************************************************************************************************
 */

#pragma once

#include "at32f415.h"

// freeRTOS priority
#define SYSTEM_TASK_PRIORITY            3

// freeRTOS stack size
#define SYSTEM_TASK_STACK_SIZE          512

// Interrupt priority
#define LED_TMR_IRQ_PRIORITY            15
#define LOG_UART_DMA_IRQ_PRIORITY       15

// ISP/LOG
#define LOG_UART                        USART1
#define LOG_UART_CLK                    CRM_USART1_PERIPH_CLOCK

#define LOG_UART_TX_PORT                GPIOA
#define LOG_UART_TX_PIN                 GPIO_PINS_9
#define LOG_UART_TX_PORT_CLK            CRM_GPIOA_PERIPH_CLOCK

#define LOG_UART_RX_PORT                GPIOA
#define LOG_UART_RX_PIN                 GPIO_PINS_10
#define LOG_UART_RX_PORT_CLK            CRM_GPIOA_PERIPH_CLOCK

#define LOG_UART_DMA                    DMA1
#define LOG_UART_DMA_CLK                CRM_DMA1_PERIPH_CLOCK
#define LOG_UART_DMA_CH                 DMA1_CHANNEL7
#define LOG_UART_DMA_CH_MUX             DMA1MUX_CHANNEL7
#define LOG_UART_DMA_CH_FLAG            DMA1_FDT7_FLAG
#define LOG_UART_DMA_ERR_FLAG           DMA1_DTERR7_FLAG
#define LOG_UART_DMA_CH_IRQ_HANDLER     DMA1_Channel7_IRQHandler
#define LOG_UART_DMA_CH_IRQN            DMA1_Channel7_IRQn

#define LOG_UART_BAUDRATE               250000

// LED
#define LED_PORT                        GPIOF
#define LED_PIN                         GPIO_PINS_7
#define LED_PORT_CLK                    CRM_GPIOF_PERIPH_CLOCK

#define LED_TMR                         TMR4
#define LED_TMR_CLK                     CRM_TMR4_PERIPH_CLOCK
#define LED_TMR_IRQN                    TMR4_GLOBAL_IRQn
#define LED_IRQ_HANDLER                 TMR4_GLOBAL_IRQHandler
