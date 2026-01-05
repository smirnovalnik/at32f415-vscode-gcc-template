/**
 ************************************************************************************************
 * @file    main.c
 * @author  Smirnov Alexander (SA)
 * @copyright Copyright FDC (C) 2024-2026
 ************************************************************************************************
 */

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "semphr.h"
#include "task.h"

#include "cycle_timer.h"
#include "hal.h"
#include "led.h"
#include "ost.h"
#include "platform.h"
#include "systick.h"
#include "ulog.h"

const char* TAG = "main";

void system_task(void* pvParameters);

int main(void)
{
    platform_init();

    ulog_init(ULOG_STDOUT);
    ulog_set_level(ULOG_DEBUG_LVL);

    led_init();

    xTaskCreate(system_task, "system_task", SYSTEM_TASK_STACK_SIZE, NULL, SYSTEM_TASK_PRIORITY, NULL);

    vTaskStartScheduler();
}

void vApplicationIdleHook(void)
{
    platform_sleep();
}

void system_task(void* pvParameters)
{
    (void)pvParameters;

#if WDG_ENABLE == 1
    platform_wdg_init();
#endif

    // Print system info
    ULOG_INFO(TAG, "----------------------------------------------");
    ULOG_INFO(TAG, "Developed by FDC (C) %.4s", __DATE__ + 7);
    ULOG_INFO(TAG, "target: %s", TARGET);
    ULOG_INFO(TAG, "fw version: %s", VERSION);
    ULOG_INFO(TAG, "git commit hash: %s", GIT_HASH);
    ULOG_INFO(TAG, "build date: %s", __DATE__);
    ULOG_INFO(TAG, "build time: %s", __TIME__);
    ULOG_INFO(TAG, "hw revision: %s", HW_REVISION);
    ULOG_INFO(TAG, "FreeRTOS version: %s", tskKERNEL_VERSION_NUMBER);

    uint32_t mcu_uid[3];
    platform_get_mcu_uid(mcu_uid);
    uint32_t uid = platform_get_uid();
    ULOG_INFO(TAG, "uid: %08x", uid);
    ULOG_INFO(TAG, "mcu uid: %08x %08x %08x", mcu_uid[0], mcu_uid[1], mcu_uid[2]);
    ULOG_INFO(TAG, "flash size: %d KB", platform_get_flash_size_kb());
    ULOG_INFO(TAG, "sysclk: %d MHz", platform_get_sysclk_mhz());

    platform_rst_cause_t rst_cause = {0};
    platform_get_rst_cause(&rst_cause);
    ULOG_INFO(TAG,
              "rst cause: power_on: %d, external: %d, software: %d, watchdog: "
              "%d, brown_out: %d",
              rst_cause.power_on, rst_cause.external, rst_cause.software, rst_cause.watchdog, rst_cause.brown_out);
    ULOG_INFO(TAG, "lfs start: 0x%08x", platform_get_lfs_start());
    ULOG_WARN(TAG, "start");

    bool fap = platform_is_flash_protection_enabled();
    if (fap == false)
    {
        ULOG_WARN(TAG, "flash protection is disabled");
#if FAP_ENABLE == 1
        ULOG_WARN(TAG, "enable flash protection and reset");
        vTaskDelay(pdMS_TO_TICKS(1000));
        platform_flash_protection_enable();
        platform_reset();
        for (;;);
#endif
    }

    uint32_t counter = 0;

    ost_t ost_heartbeat;
    ost_arm(&ost_heartbeat, 100); // arm for 100 ms

    ost_t ost_counter_timer;
    ost_arm(&ost_counter_timer, 3000); // arm for 3000 ms
    ULOG_INFO(TAG, "ost timer armed for 3000 ms");

    int64_t start_ms = systick_get_tick_ms();
    ULOG_INFO(TAG, "current tick: %ld ms", start_ms);

    uint32_t t0 = cycle_timer_now();
    for (volatile int i = 0; i < 1000; i++); // dummy loop
    uint32_t t1 = cycle_timer_now();
    uint32_t elapsed_cycles = cycle_timer_elapsed(t0, t1);
    uint32_t elapsed_us = cycle_timer_cycles_to_us(elapsed_cycles);
    ULOG_INFO(TAG, "dummy loop: %lu cycles, %lu us", elapsed_cycles, elapsed_us);

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        platform_wdg_feed();

        if (ost_expired(&ost_counter_timer))
        {
            ost_arm(&ost_counter_timer, 1000); // re-arm for 1 second

            ULOG_INFO(TAG, "counter: %d", counter);
            counter++;
        }

        if (ost_expired(&ost_heartbeat))
        {
            ost_arm(&ost_heartbeat, 100);

            static uint32_t cnt = 0;
            cnt++;
            if ((cnt & 7) == 0 || (cnt & 7) == 2)
            {
                led_set(LED_ON_SMOOTH_FAST);
            }
            else
            {
                led_set(LED_OFF);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}
