/**
 ************************************************************************************************
 * @file    hal.h
 * @author  Smirnov Alexander (SA)
 * @copyright Copyright FDC (C) 2024
 ************************************************************************************************
 */

#pragma once

#define USART1_CLK          CRM_USART1_PERIPH_CLOCK

#define USART1_TX_PORT      GPIOA
#define USART1_TX_PIN       GPIO_PINS_9
#define USART1_TX_PORT_CLK  CRM_GPIOA_PERIPH_CLOCK

#define USART1_RX_PORT      GPIOA
#define USART1_RX_PIN       GPIO_PINS_10
#define USART1_RX_PORT_CLK  CRM_GPIOA_PERIPH_CLOCK
