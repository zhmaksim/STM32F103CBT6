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

#include "stm32f103xb_it.h"
#include "systick.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

void NMI_Handler(void)
{
    error();
}
/* ------------------------------------------------------------------------- */

void HardFault_Handler(void)
{
    error();
}
/* ------------------------------------------------------------------------- */

void MemManage_Handler(void)
{
    error();
}
/* ------------------------------------------------------------------------- */

void BusFault_Handler(void)
{
    error();
}
/* ------------------------------------------------------------------------- */

void UsageFault_Handler(void)
{
    error();
}
/* ------------------------------------------------------------------------- */

void SysTick_Handler(void)
{
    systick_it_handler();
}
/* ------------------------------------------------------------------------- */

void systick_period_elapsed_callback(void)
{
    /* Обработать системный таймер FreeRTOS */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}
/* ------------------------------------------------------------------------- */
