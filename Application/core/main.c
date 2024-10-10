/*
 * Copyright (C) 2024 zhmaksim <zhiharev.maxim.alexandrovich@yandex.ru>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

/* Includes ---------------------------------------------------------------- */

#include "main.h"
#include "systick.h"
#include "pwr.h"
#include "flash.h"
#include "rcc.h"
#include "afio.h"
#include "gpio.h"
#include "adc.h"
#include "led.h"
#include "sensors.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Параметры отслеживания работы FreeRTOS */
static size_t free_heap_size;
static size_t minimum_ever_free_heap_size;
static uint32_t appl_idle_hook_counter;

/* Private function prototypes --------------------------------------------- */

static void setup_hardware(void);

static void setup_vector_table(void);

static void app_main(void *arg);

/* Private user code ------------------------------------------------------- */

int main(void)
{
    setup_hardware();

    xTaskCreate(app_main,
                "app_main",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);

    vTaskStartScheduler();
}
/* ------------------------------------------------------------------------- */

void error(void)
{
    __disable_irq();

    while (true) {}
}
/* ------------------------------------------------------------------------- */

static void app_main(void * arg)
{
    static const TickType_t frequency = pdMS_TO_TICKS(1000);

    /* INIT CODE BEGIN ----------------------------------------------------- */
    sensors_init();
    led_on(LED_BLUE);
    /* INIT CODE END ------------------------------------------------------- */

    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&last_wake_time, frequency);

        /* Обновить информацию об используемой памяти FreeRTOS */
        free_heap_size = xPortGetFreeHeapSize();
        minimum_ever_free_heap_size = xPortGetMinimumEverFreeHeapSize();
    }
}
/* ------------------------------------------------------------------------- */

void vApplicationIdleHook(void)
{
    /* Отслеживание свободного времени FreeRTOS */
    appl_idle_hook_counter++;
}
/* ------------------------------------------------------------------------- */

static void setup_hardware(void)
{
    setup_vector_table();

    systick_init(8000000);
    pwr_init();
    flash_init();
    rcc_init();
    systick_init(RCC_CPU_CLOCK);
    afio_init();
    gpio_init();
    adc_init();
}
/* ------------------------------------------------------------------------- */

static void setup_vector_table(void)
{
    __disable_irq();
    __set_PRIMASK(1);

    WRITE_REG(SCB->VTOR, 0x08000000);

    __set_PRIMASK(0);
    __enable_irq();
}
/* ------------------------------------------------------------------------- */
