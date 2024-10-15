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

#include "spi.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

SemaphoreHandle_t spi1_mutex;
EventGroupHandle_t spi1_event_group;

/* Private function prototypes --------------------------------------------- */

static void spi1_init(void);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать SPI
 */
void spi_init(void)
{
    /* Включить тактирование */
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN_Msk);

    spi1_init();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать SPI1
 */
static void spi1_init(void)
{
    /* Настройка SPI ------------------------------------------------------- */
    /* Выключить SPI перед настройкой */
    CLEAR_BIT(SPI1->CR1, SPI_CR1_SPE_Msk);
    while (READ_BIT(SPI1->CR1, SPI_CR1_SPE_Msk))
        ;

    /* Настроить режим работы = Master */
    SET_BIT(SPI1->CR1, SPI_CR1_MSTR_Msk);

    /* Настроить делитель часов = Fpclk / 2 (72MHz / 2 = 36MHz) */
    CLEAR_BIT(SPI1->CR1, SPI_CR1_BR_Msk);

    /* Настроить режим работы часов = Mode 0 */
    CLEAR_BIT(SPI1->CR1,
              SPI_CR1_CPOL_Msk
            | SPI_CR1_CPHA_Msk);

    /* Настроить формат фрейма = 8bit, MSB first */
    CLEAR_BIT(SPI1->CR1,
              SPI_CR1_DFF_Msk
            | SPI_CR1_LSBFIRST_Msk);

    /* Настроить SS = Software */
    SET_BIT(SPI1->CR1,
            SPI_CR1_SSM_Msk
          | SPI_CR1_SSI_Msk);

    /* Настроить BIDIMODE = 0, RXONLY = 0 */
    CLEAR_BIT(SPI1->CR1,
              SPI_CR1_BIDIMODE_Msk
            | SPI_CR1_RXONLY_Msk);

    /* Включить DMA */
    SET_BIT(SPI1->CR2,
            SPI_CR2_TXDMAEN_Msk
          | SPI_CR2_RXDMAEN_Msk);

    /* Включить SPI */
    SET_BIT(SPI1->CR1, SPI_CR1_SPE_Msk);


    /* Настройка DMA ------------------------------------------------------- */
    /* Включить тактирование */
    SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN_Msk);

    /* Выключить DMA перед настройкой */
    CLEAR_BIT(DMA1_Channel2->CCR, DMA_CCR_EN_Msk);
    while (READ_BIT(DMA1_Channel2->CCR, DMA_CCR_EN_Msk))
        ;

    /* Настроить DMA для SPI1_RX */
    MODIFY_REG(DMA1_Channel2->CCR,
               DMA_CCR_DIR_Msk
             | DMA_CCR_CIRC_Msk
             | DMA_CCR_PINC_Msk
             | DMA_CCR_MINC_Msk
             | DMA_CCR_PSIZE_Msk
             | DMA_CCR_MSIZE_Msk
             | DMA_CCR_PL_Msk,
               0x00 << DMA_CCR_DIR_Pos         /* Peripheral - Memory */
             | 0x01 << DMA_CCR_MINC_Pos        /* Memory increment mode */
             | 0x00 << DMA_CCR_PSIZE_Pos       /* 8bits peripheral size */
             | 0x00 << DMA_CCR_MSIZE_Pos       /* 8bits memory size*/
             | 0x02 << DMA_CCR_PL_Pos);        /* High priority */

    /* Включить прерывания */
    SET_BIT(DMA1_Channel2->CCR,
            DMA_CCR_TCIE_Msk
          | DMA_CCR_TEIE_Msk);

    /* Настроить DMA для SPI1_TX */
    MODIFY_REG(DMA1_Channel3->CCR,
               DMA_CCR_DIR_Msk
             | DMA_CCR_CIRC_Msk
             | DMA_CCR_PINC_Msk
             | DMA_CCR_MINC_Msk
             | DMA_CCR_PSIZE_Msk
             | DMA_CCR_MSIZE_Msk
             | DMA_CCR_PL_Msk,
               0x01 << DMA_CCR_DIR_Pos         /* Memory - Peripheral */
             | 0x01 << DMA_CCR_MINC_Pos        /* Memory increment mode */
             | 0x00 << DMA_CCR_PSIZE_Pos       /* 8bits peripheral size */
             | 0x00 << DMA_CCR_MSIZE_Pos       /* 8bits memory size*/
             | 0x02 << DMA_CCR_PL_Pos);        /* High priority */

    /* Включить прерывания */
    SET_BIT(DMA1_Channel3->CCR,
            DMA_CCR_TCIE_Msk
          | DMA_CCR_TEIE_Msk);


    /* Настроить NVIC */
    NVIC_SetPriority(DMA1_Channel2_IRQn, 8);
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    NVIC_SetPriority(DMA1_Channel3_IRQn, 8);
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);

    /* Инициализировать Mutex */
    spi1_mutex = xSemaphoreCreateMutex();
    if (spi1_mutex == NULL) error();

    /* Инициализировать Event Group */
    spi1_event_group = xEventGroupCreate();
    if (spi1_event_group == NULL) error();
}
/* ------------------------------------------------------------------------- */
