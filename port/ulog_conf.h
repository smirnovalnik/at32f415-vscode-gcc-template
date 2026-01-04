/**
  ******************************************************************************
  * @file    ulog_conf.h
  * @author  Alexander Smirnov
  * @brief   Configuration template
  * @copyright Copyright (C) 2016 - 2026 Alexander Smirnov
  ******************************************************************************
*/

#ifndef _ULOG_CONF_H_
#define _ULOG_CONF_H_

// Master switch - set to 0 to completely disable logging (zero code/RAM)
#define ULOG_ENABLE             1

// General settings
#define ULOG_TIMESTAMP          0     // 1 = real timestamp, 0 = clock ticks since startup
#define ULOG_TIMESTAMP_MS       0     // Add milliseconds to timestamp (requires ULOG_TIMESTAMP=1)
#define ULOG_PRINT_TAG          1     // Print tag
#define ULOG_PRINT_COLOR        1     // Print messages in color (ANSI escape codes)
#define ULOG_PRINT_LEVEL        1     // Print level
#define ULOG_LEVEL_SHORT        1     // Short level name (T/D/I/W/E vs TRACE/DEBUG/INFO/WARN/ERR)

#define ULOG_ENDLINE            "\n"  // End of line
#define ULOG_REMOVE_SPACES      0     // Remove \n, \r, \t from the message
#define ULOG_MESSAGE_LEN        120   // Maximum length of the formatted message

// Binary dump settings
#define ULOG_DUMP_BYTES_PER_LINE 16   // Number of bytes per line in hex dump (8 or 16 recommended)
#define ULOG_DUMP_SHOW_ASCII     1    // Show ASCII representation of binary data

#define ULOG_FILE_SYSTEM        0     // Enable log to file system (disable for most MCUs)
#define ULOG_FILE_NAME          "system.log"        // File name
#define ULOG_MAX_FILE_SIZE      (1 * 1024 * 1024)   // Max file size in bytes before rotation

// Target specific settings
#define ULOG_USE_MUTEX          1     // Enable mutex for thread safety
#define ULOG_USE_POSIX_MUTEX    0     // 1 = POSIX pthread, 0 = FreeRTOS semaphore
#define ULOG_USE_POSIX_IO       1     // 1 = POSIX file I/O, 0 = FatFs

// Mutex configuration
#if ULOG_USE_MUTEX == 1
    #if ULOG_USE_POSIX_MUTEX == 1
        #include <pthread.h>
        static pthread_mutex_t ulog_mutex = PTHREAD_MUTEX_INITIALIZER;
        #define ULOG_CREATE_MUTEX()
        #define ULOG_MUTEX_TAKE()   pthread_mutex_lock(&ulog_mutex)
        #define ULOG_MUTEX_GIVE()   pthread_mutex_unlock(&ulog_mutex)
    #else
        #include "FreeRTOS.h"
        #include "semphr.h"
        static SemaphoreHandle_t ulog_mutex;
        #define ULOG_CREATE_MUTEX() do { if (ulog_mutex == NULL) \
                                        ulog_mutex = xSemaphoreCreateMutex(); } while(0)
        #define ULOG_MUTEX_TAKE()   xSemaphoreTake(ulog_mutex, portMAX_DELAY)
        #define ULOG_MUTEX_GIVE()   xSemaphoreGive(ulog_mutex)
    #endif
#else
    #define ULOG_CREATE_MUTEX()
    #define ULOG_MUTEX_TAKE()
    #define ULOG_MUTEX_GIVE()
#endif

// File system I/O configuration
#if ULOG_FILE_SYSTEM == 1
    #if ULOG_USE_POSIX_IO != 1
        #include "fatfs.h"      /* FatFs from Chan */
    #endif
#endif

#endif /* _ULOG_CONF_H_ */
