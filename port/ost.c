/**
 ******************************************************************************
 * @file    ost.h
 * @brief   One-shot timers
 * @copyright Copyright (C) 2021 by Faraday Development Center
 ******************************************************************************
 */

#include "ost.h"

static volatile uint64_t clock_ms;
static volatile uint32_t clock_sec;

void ost_interrupt_handler(void)
{
    clock_ms++;
    if ((clock_ms % 1000) == 0)
    {
        clock_sec++;
    }
}

uint64_t ost_get_clock(void)
{
    return clock_ms;
}

void ost_arm(ost_t* timer, uint32_t ms)
{
    timer->state = OST_RUN;
    timer->timeout = clock_ms + ms;
}

uint64_t ost_left(ost_t* timer)
{
    if (timer->state == OST_RUN)
    {
        return timer->timeout - clock_ms;
    }
    else
    {
        return 0;
    }
}

uint8_t ost_enabled(ost_t* timer)
{
    return (timer->state != OST_IDLE);
}

void ost_disable(ost_t* timer)
{
    timer->state = OST_IDLE;
}

uint8_t ost_expired(ost_t* timer)
{
    if (timer->state == OST_IDLE)
    {
        return 0;
    }
    else if (timer->state == OST_RUN && timer->timeout > clock_ms)
    {
        return 0;
    }

    timer->state = OST_IDLE;
    return 1;
}

uint32_t ost_get_sec_clock(void)
{
    return clock_sec;
}

void ost_sec_arm(ost_sec_t* timer, uint32_t seconds)
{
    timer->state = OST_RUN;
    timer->timeout = clock_sec + seconds;
}

uint32_t ost_sec_left(ost_sec_t* timer)
{
    if (timer->state == OST_RUN)
    {
        return timer->timeout - clock_sec;
    }
    else
    {
        return 0;
    }
}

uint8_t ost_sec_expired(ost_sec_t* timer)
{
    if (timer->state == OST_IDLE)
    {
        return 0;
    }
    else if (timer->state == OST_RUN && timer->timeout > clock_sec)
    {
        return 0;
    }

    timer->state = OST_IDLE;
    return 1;
}

uint8_t ost_sec_enabled(ost_sec_t* timer)
{
    return (timer->state != OST_IDLE);
}

void ost_sec_disable(ost_sec_t* timer)
{
    timer->state = OST_IDLE;
}
