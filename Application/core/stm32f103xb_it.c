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
#include "pwr.h"
#include "sensors.h"
#include "w25q.h"

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

void PVD_IRQHandler(void)
{
    pwr_pvd_it_handler();
}
/* ------------------------------------------------------------------------- */

void DMA1_Channel1_IRQHandler(void)
{
    sensors_dma_it_handler();
}
/* ------------------------------------------------------------------------- */

void DMA1_Channel2_IRQHandler(void)
{
    w25q_spi_dma_rx_it_handler();
}
/* ------------------------------------------------------------------------- */

void DMA1_Channel3_IRQHandler(void)
{
    w25q_spi_dma_tx_it_handler();
}
/* ------------------------------------------------------------------------- */
