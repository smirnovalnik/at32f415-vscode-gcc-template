/**
 ************************************************************************************************
 * @file    hal.h
 * @author  Smirnov Alexander (SA)
 * @copyright Copyright FDC (C) 2024
 ************************************************************************************************
 */

#pragma once

#include "at32f415.h"

// ISP/LOG
#define LOG_UART                USART1
#define LOG_UART_CLK            CRM_USART1_PERIPH_CLOCK

#define LOG_UART_TX_PORT        GPIOA
#define LOG_UART_TX_PIN         GPIO_PINS_9
#define LOG_UART_TX_PORT_CLK    CRM_GPIOA_PERIPH_CLOCK

#define LOG_UART_RX_PORT        GPIOA
#define LOG_UART_RX_PIN         GPIO_PINS_10
#define LOG_UART_RX_PORT_CLK    CRM_GPIOA_PERIPH_CLOCK

#define LOG_UART_BAUDRATE       250000