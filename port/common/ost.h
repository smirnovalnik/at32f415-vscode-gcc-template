/**
  ******************************************************************************
  * @file    ost.h
  * @brief   One-shot timers
  * @copyright Copyright (C) 2021 by Faraday Development Center
  ******************************************************************************
*/

#pragma once

#include <stdint.h>

enum ost_state
{
    OST_IDLE = 0,
    OST_RUN = 1,
};

/**
 * One-shot timer milliseconds (OST).
 */
typedef struct ost_t
{
    uint64_t timeout;
    uint8_t state;
} ost_t;

/**
 * One-shot timer seconds (OST).
 */
typedef struct ost_sec_t
{
    uint32_t timeout;
    uint8_t state;
} ost_sec_t;

/**
 * One-shot timer interrupt handler. Should be called from the interrupt handler each millisecond.
 */
void ost_interrupt_handler(void);


/** Millisecond one-shot timer */

/** Get millisecond one-shot timer clock */
uint64_t ost_get_clock(void);

/**
 * Arm a one-shot timer.
 * The timer will "fire" after the specified interval in milliseconds.
 */
void ost_arm(ost_t *timer, uint32_t ms);

/**
 * Return time before expiration.
 * Return zero if timer is disabled.
 */
uint64_t ost_left(ost_t *timer);

/**
 * Check, if one-shot timer is expired.
 * Return not zero one time after expiration.
 * Return zero if timer disabled.
 */
uint8_t ost_expired(ost_t *timer);

/**
 * Check, if one-shot timer enabled.
 */
uint8_t ost_enabled(ost_t *timer);

/**
 * Disable one-shot timer.
 */
void ost_disable(ost_t *timer);


/**
 * Get second one-shot timer clock.
 */
uint32_t ost_get_sec_clock(void);

/**
 * Arm a one-shot timer.
 * The timer will "fire" after the specified interval in seconds.
 */
void ost_sec_arm(ost_sec_t *timer, uint32_t seconds);

/**
 * Return time before expiration.
 * Return zero if timer is disabled.
 */
uint32_t ost_sec_left(ost_sec_t *timer);

/**
 * Check, if one-shot timer is expired.
 * Return not zero one time after expiration.
 * Return zero if timer disabled.
 */
uint8_t ost_sec_expired(ost_sec_t *timer);

/**
 * Check, if one-shot timer enabled.
 */
uint8_t ost_sec_enabled(ost_sec_t *timer);

/**
 * Disable one-shot timer.
 */
void ost_sec_disable(ost_sec_t *timer);
