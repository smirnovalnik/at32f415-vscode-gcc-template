/**
 ************************************************************************************************
 * @file    main.c
 * @author  Smirnov Alexander (SA)
 * @copyright Copyright FDC (C) 2024
 ************************************************************************************************
 */

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "semphr.h"
#include "task.h"

#include "platform.h"
#include "systick.h"
#include "ulog.h"

const char* TAG = "main";

void system_task(void* pvParameters);

int main(void)
{
    platform_init();

    xTaskCreate(system_task, "system_task", 512, NULL, 2, NULL);

    vTaskStartScheduler();
}

void vApplicationIdleHook(void)
{
    platform_sleep();
}

void system_task(void* pvParameters)
{
    ulog_init(ULOG_STDOUT);
    ulog_set_level(ULOG_DEBUG_LVL);

    ULOG_INFO(TAG, "");
    ULOG_INFO(TAG, "----------------------------------------------");
    ULOG_INFO(TAG, "Copyright FDC (C) 2024");
    ULOG_INFO(TAG, "target: %s", TARGET);
    ULOG_INFO(TAG, "fw version: %s", VERSION);
    ULOG_INFO(TAG, "hw revision: %s", HW_REVISION);
    ULOG_INFO(TAG, "git commit hash: %s", GIT_HASH);
    ULOG_INFO(TAG, "build date: %s", __DATE__);
    ULOG_INFO(TAG, "build time: %s", __TIME__);

    uint32_t mcu_uid[3];
    platform_get_mcu_uid(mcu_uid);
    ULOG_INFO(TAG, "mcu uid: %08X %08X %08X", mcu_uid[0], mcu_uid[1], mcu_uid[2]);
    ULOG_INFO(TAG, "flash size: %d KB", platform_get_flash_size_kb());
    ULOG_INFO(TAG, "system_core_clock: %d MHz", platform_get_system_clock_mhz());
    platform_rst_cause_t rst_cause = {0};
    platform_get_rst_cause(&rst_cause);
    ULOG_INFO(TAG, "rst cause: power_on: %d, external: %d, software: %d, watchdog: %d, brown_out: %d",
              rst_cause.power_on, rst_cause.external, rst_cause.software, rst_cause.watchdog, rst_cause.brown_out);

    ULOG_WARN(TAG, "start");

    platform_wdg_init();

    uint32_t counter = 0;
    for (;;)
    {
        platform_wdg_feed();

        ULOG_INFO(TAG, "counter: %d", counter);
        counter++;

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
