/**
 ******************************************************************************
 * @file    platform.c
 * @author  Alexander Smirnov
 * @copyright Copyright FDC (C) 2024-2026
 ******************************************************************************
 */

#include "platform.h"
#include "at32f415_clock.h"
#include "hal.h"
#include "systick.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "semphr.h"
#include "task.h"

#define STDOUT_QUEUE_SIZE 1024

static QueueHandle_t stdout_queue;
static SemaphoreHandle_t uart_tx_cmplt_smphr = NULL;

static void stdout_init(uint32_t baudrate);
static void stdout_task(void* pvParameters);
static void stdout_tr(uint8_t* buf, uint32_t size);

void platform_init(void)
{
    system_clock_config_144mhz();

    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

    // make sense only for HEXT clock
    // crm_clock_failure_detection_enable(TRUE);

    systick_init();

    stdout_init(LOG_UART_BAUDRATE);
    stdout_queue = xQueueCreate(STDOUT_QUEUE_SIZE, sizeof(char));
    xTaskCreate(stdout_task, "stdout_task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}

static void stdout_init(uint32_t baudrate)
{
#if defined(__GNUC__) && !defined(__clang__)
    setvbuf(stdout, NULL, _IONBF, 0);
#endif

    // gpio
    crm_periph_clock_enable(LOG_UART_TX_PORT_CLK, TRUE);

    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);

    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = LOG_UART_TX_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(LOG_UART_TX_PORT, &gpio_init_struct);

    // uart
    crm_periph_clock_enable(LOG_UART_CLK, TRUE);

    usart_init(LOG_UART, baudrate, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(LOG_UART, TRUE);
    usart_dma_transmitter_enable(LOG_UART, TRUE);
    usart_enable(LOG_UART, TRUE);

    // dma
    crm_periph_clock_enable(LOG_UART_DMA_CLK, TRUE);

    dma_reset(LOG_UART_DMA_CH);
    dma_flexible_config(LOG_UART_DMA, FLEX_CHANNEL7, DMA_FLEXIBLE_UART1_TX);
    dma_interrupt_enable(LOG_UART_DMA_CH, DMA_FDT_INT, TRUE);
    nvic_irq_enable(LOG_UART_DMA_CH_IRQN, LOG_UART_DMA_IRQ_PRIORITY, 0);

    // rtos
    if (uart_tx_cmplt_smphr == NULL)
    {
        uart_tx_cmplt_smphr = xSemaphoreCreateBinary();
    }
}

static void stdout_deinit(void)
{
    usart_enable(LOG_UART, FALSE);
    usart_dma_transmitter_enable(LOG_UART, FALSE);
    usart_transmitter_enable(LOG_UART, FALSE);

    dma_channel_enable(LOG_UART_DMA_CH, FALSE);
    dma_interrupt_enable(LOG_UART_DMA_CH, DMA_FDT_INT, FALSE);
    nvic_irq_disable(LOG_UART_DMA_CH_IRQN);
}

static void stdout_tr(uint8_t* buf, uint32_t size)
{
    dma_init_type dma_init_struct;
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.buffer_size = size;
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_base_addr = (uint32_t)buf;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)&LOG_UART->dt;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_LOW;
    dma_init_struct.loop_mode_enable = FALSE;
    dma_init(LOG_UART_DMA_CH, &dma_init_struct);

    dma_channel_enable(LOG_UART_DMA_CH, TRUE);

    xSemaphoreTake(uart_tx_cmplt_smphr, pdMS_TO_TICKS(100));
}

void LOG_UART_DMA_CH_IRQ_HANDLER(void)
{
    if (dma_interrupt_flag_get(LOG_UART_DMA_CH_FLAG))
    {
        dma_flag_clear(LOG_UART_DMA_CH_FLAG);
        dma_channel_enable(LOG_UART_DMA_CH, FALSE);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(uart_tx_cmplt_smphr, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void platform_wdg_init(void)
{
    wdt_register_write_enable(TRUE);
    wdt_divider_set(WDT_CLK_DIV_32);

    /*
        wdg_timeout = reload_value * (divider / lick_freq ) (s)
        lick_freq    = 40000 Hz
        divider      = 32
        reload_value = 4096

        timeout = 4096 * (32 / 40000 ) = 3.27s
    */
    wdt_reload_value_set(4096 - 1);

    wdt_counter_reload();
    wdt_enable();
}

void platform_wdg_feed(void)
{
    wdt_counter_reload();
}

void platform_reset(void)
{
    nvic_system_reset();
}

void platform_sleep(void)
{
    pwc_sleep_mode_enter(PWC_SLEEP_ENTER_WFI);
}

void platform_bootloader_jump(void)
{
    // TODO
}

void platform_get_rst_cause(platform_rst_cause_t* rst_cause)
{
    if (crm_flag_get(CRM_POR_RESET_FLAG) != RESET)
    {
        crm_flag_clear(CRM_POR_RESET_FLAG);
        rst_cause->power_on = true;
    }
    else
    {
        rst_cause->power_on = false;
    }

    if (crm_flag_get(CRM_SW_RESET_FLAG) != RESET)
    {
        crm_flag_clear(CRM_SW_RESET_FLAG);
        rst_cause->software = true;
    }
    else
    {
        rst_cause->software = false;
    }

    if (crm_flag_get(CRM_WDT_RESET_FLAG) != RESET)
    {
        crm_flag_clear(CRM_WDT_RESET_FLAG);
        rst_cause->watchdog = true;
    }
    else
    {
        rst_cause->watchdog = false;
    }

    if (crm_flag_get(CRM_LOWPOWER_RESET_FLAG) != RESET)
    {
        crm_flag_clear(CRM_LOWPOWER_RESET_FLAG);
        rst_cause->brown_out = true;
    }
    else
    {
        rst_cause->brown_out = false;
    }

    if (crm_flag_get(CRM_NRST_RESET_FLAG) != RESET)
    {
        crm_flag_clear(CRM_NRST_RESET_FLAG);
        rst_cause->external = true;
    }
    else
    {
        rst_cause->external = false;
    }
}

void platform_set_sysclk(platform_sysclk_t sysclk)
{
    switch (sysclk)
    {
        case SYSCLK_144MHZ:
        {
            systick_deinit();
            stdout_deinit();

            system_clock_config_144mhz();

            systick_init();
            stdout_init(LOG_UART_BAUDRATE);
            break;
        }
        case SYSCLK_16MHZ:
        {
            systick_deinit();
            stdout_deinit();

            system_clock_config_16mhz();

            systick_init();
            stdout_init(LOG_UART_BAUDRATE);
            break;
        }
        default:
            break;
    }
}

uint32_t platform_get_sysclk_mhz(void)
{
    return system_core_clock / 1000000;
}

// Only for HEXT clock
void platform_clock_failure_detection_handler(void)
{
    if (crm_flag_get(CRM_CLOCK_FAILURE_INT_FLAG) != RESET)
    {
        crm_clock_failure_detection_enable(FALSE);

        // Enable HINT
        // e.g.
        // system_clock_config_hint_144mhz();

        crm_flag_clear(CRM_CLOCK_FAILURE_INT_FLAG);
    }
}

static fn_t pvm_cb = NULL;

void platform_power_voltage_monitor_enable(fn_t cb)
{
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);

    // set the threshold voltage to 2.9v
    pwc_pvm_level_select(PWC_PVM_VOLTAGE_2V9);

    pwc_power_voltage_monitor_enable(TRUE);

    exint_init_type exint_init_struct;
    // config the exint line of the power voltage monitor
    exint_init_struct.line_select = EXINT_LINE_16;
    exint_init_struct.line_enable = TRUE;
    exint_init_struct.line_mode = EXINT_LINE_INTERRUPUT;
    exint_init_struct.line_polarity = EXINT_TRIGGER_BOTH_EDGE;
    exint_init(&exint_init_struct);

    // when power voltage monitor enable, exint line16 flag will be set
    // while (exint_flag_get(EXINT_LINE_16) == RESET);

    // clear the unexpected exint line flag
    exint_flag_clear(EXINT_LINE_16);

    // clear the unexpected nvic pending flag
    NVIC_ClearPendingIRQ(PVM_IRQn);

    pvm_cb = cb;

    // enable power voltage monitor interrupt
    nvic_irq_enable(PVM_IRQn, 1, 0);
}

void platform_power_voltage_monitor_handler(void)
{
    if (exint_flag_get(EXINT_LINE_16) != RESET)
    {
        exint_flag_clear(EXINT_LINE_16);

        if (pvm_cb != NULL)
        {
            pvm_cb();
        }

        platform_reset();
    }
}

void platform_get_mcu_uid(uint32_t uid[3])
{
    uid[0] = *(uint32_t*)0x1FFFF7E8;
    uid[1] = *(uint32_t*)0x1FFFF7EC;
    uid[2] = *(uint32_t*)0x1FFFF7F0;
}

uint32_t platform_get_uid(void)
{
    crm_periph_clock_enable(CRM_CRC_PERIPH_CLOCK, TRUE);
    crc_data_reset();
    uint32_t crc = crc_block_calculate((uint32_t*)0x1FFFF7E8, 3);
    crm_periph_clock_enable(CRM_CRC_PERIPH_CLOCK, FALSE);
    return crc;
}

uint16_t platform_get_flash_size_kb(void)
{
    return *(uint16_t*)0x1FFFF7E0;
}

void platform_flash_protection_enable(void)
{
    flash_status_type status = FLASH_OPERATE_DONE;

    flash_unlock();
    status = flash_operation_wait_for(OPERATION_TIMEOUT);

    if (status != FLASH_OPERATE_TIMEOUT)
    {
        if ((status == FLASH_PROGRAM_ERROR) || (status == FLASH_EPP_ERROR))
        {
            flash_flag_clear(FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
        }

        status = flash_fap_enable(TRUE);
        if (status == FLASH_OPERATE_DONE)
        {
            nvic_system_reset();
        }
    }
}

bool platform_is_flash_protection_enabled(void)
{
    return (flash_fap_status_get() == RESET) ? false : true;
}

// Linker script symbols
extern uint32_t _lfs_start;
extern uint32_t _lfs_size;

#define LFS_START_ADDR     ((uint32_t)&_lfs_start)
#define LFS_SIZE           ((uint32_t)&_lfs_size)
#define LFS_CACHE_SIZE     256
#define LFS_LOOKAHEAD_SIZE 16

#define BLOCK_SIZE 2048

uint32_t platform_get_lfs_start(void)
{
    return LFS_START_ADDR;
}

static lfs_t lfs;

lfs_t* platform_get_lfs(void)
{
    return &lfs;
}

static int flash_read(const struct lfs_config* c, lfs_block_t block, lfs_off_t off, void* buffer, lfs_size_t size)
{
    memcpy(buffer, (void*)(platform_get_lfs_start() + block * BLOCK_SIZE + off), size);

    return 0;
}

static int flash_prog(const struct lfs_config* c, lfs_block_t block, lfs_off_t off, const void* buffer, lfs_size_t size)
{
    flash_status_type status = FLASH_OPERATE_DONE;

    flash_unlock();

    for (uint32_t i = 0; i < size; i += 4)
    {
        status
            = flash_word_program(platform_get_lfs_start() + block * BLOCK_SIZE + off + i, *((uint32_t*)(buffer + i)));
        if (status != FLASH_OPERATE_DONE)
        {
            if ((status == FLASH_PROGRAM_ERROR) || (status == FLASH_EPP_ERROR))
            {
                flash_flag_clear(FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
            }
            flash_lock();
            return -1;
        }
    }

    flash_lock();

    return 0;
}

static int flash_erase(const struct lfs_config* c, lfs_block_t block)
{
    flash_status_type status = FLASH_OPERATE_DONE;

    flash_unlock();

    status = flash_sector_erase(platform_get_lfs_start() + block * BLOCK_SIZE);
    if (status != FLASH_OPERATE_DONE)
    {
        if ((status == FLASH_PROGRAM_ERROR) || (status == FLASH_EPP_ERROR))
        {
            flash_flag_clear(FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
        }
        flash_lock();
        return -1;
    }

    flash_lock();
    return 0;
}

static int flash_sync(const struct lfs_config* c)
{
    return 0;
}

static uint8_t prog_buf[LFS_CACHE_SIZE];
static uint8_t read_buf[LFS_CACHE_SIZE];
static uint8_t lookahead_buf[LFS_LOOKAHEAD_SIZE];

static struct lfs_config cfg = {
    // block device operations
    .read = flash_read,
    .prog = flash_prog,
    .erase = flash_erase,
    .sync = flash_sync,

    // block device configuration
    .read_size = 4,
    .prog_size = 4,
    .block_size = BLOCK_SIZE,
    .block_count = 0, // initialized lazily
    .cache_size = LFS_CACHE_SIZE,
    .lookahead_size = LFS_LOOKAHEAD_SIZE,
    .block_cycles = 500,

    .prog_buffer = prog_buf,
    .read_buffer = read_buf,
    .lookahead_buffer = lookahead_buf,
};

struct lfs_config* platform_get_lfs_config(void)
{
    if (cfg.block_count == 0)
    {
        cfg.block_count = LFS_SIZE / BLOCK_SIZE;
    }
    return &cfg;
}

#define STDOUT_PP_BUF_SIZE 256

static void stdout_task(void* pvParameters)
{
    for (;;)
    {
        static uint8_t buf1[STDOUT_PP_BUF_SIZE], buf2[STDOUT_PP_BUF_SIZE];
        static uint32_t len1 = 0, len2 = 0;

        uint8_t* ping_buf;
        uint32_t *ping_len, *pong_len;

        if (len1 == 0)
        {
            ping_buf = buf1;
            ping_len = &len1;
            pong_len = &len2;
        }
        else
        {
            ping_buf = buf2;
            ping_len = &len2;
            pong_len = &len1;
        }

        if (xQueueReceive(stdout_queue, &ping_buf[*ping_len], portMAX_DELAY) == pdPASS)
        {
            while ((++(*ping_len) < STDOUT_PP_BUF_SIZE)
                   && (xQueueReceive(stdout_queue, &ping_buf[*ping_len], 0) == pdPASS));

            *pong_len = 0;
            stdout_tr(ping_buf, *ping_len);
        }
    }
}

int platform_fputc(int ch)
{
    char item = (char)ch;

    xQueueSendToBack(stdout_queue, &item, pdMS_TO_TICKS(10));

    return ch;
}

int platform_fgetc(void)
{
    return 0;
}
