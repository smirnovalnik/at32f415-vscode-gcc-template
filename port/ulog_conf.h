/**
  ******************************************************************************
  * @file    ulog_conf.h
  * @author  Alexander Smirnov
  * @brief   Configuration template
  * @copyright Copyright (C) 2016 - 2024 Alexander Smirnov
  ******************************************************************************
*/

#ifndef _ULOG_CONF_H_
#define _ULOG_CONF_H_

// General settings
#define ULOG_TIMESTAMP          0     // Timestamp or clock since startup
#define ULOG_TIMESTAMP_MS       0     // Add milliseconds to timestamp
#define ULOG_PRINT_TAG          1     // Print tag
#define ULOG_PRINT_COLOR        1     // Print messages in color
#define ULOG_PRINT_LEVEL        1     // Print level

#define ULOG_ENDLINE            "\n"  // End of line
#define ULOG_REMOVE_SPACES      0     // Remove spaces (> 2 in row) from the message
#define ULOG_MESSAGE_LEN        120   // Length of the message

#define ULOG_FILE_SYSTEM        0     // Enable log to file system
#define ULOG_FILE_NAME          "system.log"        // File name
#define ULOG_MAX_FILE_SIZE      (1 * 1024 * 1024)   // Max file size in bytes before rotation

// Target specific settings
#define ULOG_USE_MUTEX          0
#define ULOG_USE_POSIX_MUTEX    1
#define ULOG_USE_POSIX_IO       1

#if ULOG_USE_MUTEX == 1
#if ULOG_USE_POSIX_MUTEX == 1
#include <pthread.h>
#include <stdlib.h>
static pthread_mutex_t ulog_mutex = PTHREAD_MUTEX_INITIALIZER;
#define ULOG_CREATE_MUTEX()
#define ULOG_MUTEX_TAKE()   pthread_mutex_lock(&ulog_mutex);
#define ULOG_MUTEX_GIVE()   pthread_mutex_unlock(&ulog_mutex);
#else
#include "FreeRTOS.h"
#include "semphr.h"
static xSemaphoreHandle ulog_mutex;
#define ULOG_CREATE_MUTEX()  if (ulog_mutex == NULL) \
                                ulog_mutex = xSemaphoreCreateMutex()
#define ULOG_MUTEX_TAKE()    xSemaphoreTake(ulog_mutex, portMAX_DELAY)
#define ULOG_MUTEX_GIVE()    xSemaphoreGive(ulog_mutex)
#endif
#else
#define ULOG_CREATE_MUTEX()
#define ULOG_MUTEX_TAKE()
#define ULOG_MUTEX_GIVE()
#endif

#if ULOG_USE_POSIX_IO == 1

#else
#include "fatfs.h"      /* FatFs from Chan */
#endif

#endif /* _ULOG_CONF_H_ */
