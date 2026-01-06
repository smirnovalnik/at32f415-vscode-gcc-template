/**
 ************************************************************************************************
 * @file    syscalls.c
 * @author  Smirnov Alexander
 * @copyright Copyright (C) 2023 - 2026
 ************************************************************************************************
 */

#include <errno.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/times.h>

#include "hal.h"
#include "platform.h"
#include "systick.h"

int __io_putchar(int ch)
{
    return platform_fputc(ch);
}

int _open(const char* name, int flags, int mode)
{
    return -1;
}

int _write(int fd, char* pbuffer, int size)
{
    for (int i = 0; i < size; i++)
    {
        platform_fputc((uint16_t)(*pbuffer++));
    }

    return size;
}

int _read(int file, char* ptr, int len)
{
    return 0;
}

int _close(int file)
{
    return -1;
}

int _fstat(int file, struct stat* st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

int _lseek(int file, int ptr, int dir)
{
    return 0;
}

clock_t _times(struct tms* tp)
{
    clock_t timeval;

    timeval = systick_get_tick_ms();

    if (tp)
    {
        tp->tms_utime = timeval; /* user time */
        tp->tms_stime = 0;       /* system time */
        tp->tms_cutime = 0;      /* user time, children */
        tp->tms_cstime = 0;      /* system time, children */
    }

    return timeval;
};

int _isatty(int file)
{
    return 1;
}

int _getpid(void)
{
    return 1;
}

int _kill(int pid, int sig)
{
    errno = EINVAL;
    return -1;
}
