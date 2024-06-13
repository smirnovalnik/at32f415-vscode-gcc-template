
/**
 ************************************************************************************************
 * @file    syscalls.c
 * @author  Smirnov Alexander
 * @copyright Copyright (C) 2023 - 2024
 ************************************************************************************************
 */

#include <errno.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/times.h>

#include "at32f415.h"
#include "hal.h"
#include "systick.h"

int __io_putchar(int ch)
{
    while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET);
    usart_data_transmit(USART1, (uint16_t)ch);
    while (usart_flag_get(USART1, USART_TDC_FLAG) == RESET);
    return ch;
}

int _write(int fd, char *pbuffer, int size)
{
    for(int i = 0; i < size; i ++)
    {
        while( usart_flag_get(USART1, USART_TDBE_FLAG) == RESET);
        usart_data_transmit(USART1, (uint16_t)(*pbuffer++));
        while (usart_flag_get(USART1, USART_TDC_FLAG) == RESET);
    }

    return size;
}

clock_t _times(struct tms * tp)
{
    clock_t timeval;

    timeval = systick_get_tick_ms();	// TODO

    if (tp)
    {
        tp->tms_utime  = timeval;	/* user time */
        tp->tms_stime  = 0;	/* system time */
        tp->tms_cutime = 0;	/* user time, children */
        tp->tms_cstime = 0;	/* system time, children */
    }

    return timeval;
};

int _close_r(struct _reent *ptr, int fd)
{
    return -1;
}

int _fstat_r(struct _reent *ptr, int fd, struct stat *pstat)
{
    return -1;
}

off_t _lseek_r(struct _reent *ptr, int fd, off_t pos, int whence)
{
    return -1;
}

_ssize_t _read_r(struct _reent *ptr, int fd, void *buf, size_t cnt)
{
    return -1;
}

int _isatty_r(struct _reent *ptr, int fd)
{
  return 1;
}
