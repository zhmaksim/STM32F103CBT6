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

#include "gpio.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void gpio_led_init(void);

static void gpio_w25q_init(void);

static void gpio_spi_init(void);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO
 */
void gpio_init(void)
{
    /* Включить тактирование */
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN_Msk);
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPBEN_Msk);

    gpio_led_init();
    gpio_w25q_init();
    gpio_spi_init();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO LED
 */
static void gpio_led_init(void)
{
    /* GPIOB2 LED_BLUE */

    /* Начальный уровень = Low */
    CLEAR_BIT(GPIOB->ODR, GPIO_ODR_ODR2_Msk);

    /* Output Push-Pull 2MHz */
    MODIFY_REG(GPIOB->CRL,
               GPIO_CRL_MODE2_Msk
             | GPIO_CRL_CNF2_Msk,
               0x02 << GPIO_CRL_MODE2_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO W25Q
 */
static void gpio_w25q_init(void)
{
    /* GPIOA4 W25Q_CS */

    /* Начальный уровень = High */
    SET_BIT(GPIOA->ODR, GPIO_ODR_ODR4_Msk);

    /* Output Push-Pull 50MHz */
    MODIFY_REG(GPIOA->CRL,
               GPIO_CRL_MODE4_Msk
             | GPIO_CRL_CNF4_Msk,
               0x03 << GPIO_CRL_MODE4_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO SPI
 */
static void gpio_spi_init(void)
{
    /*
     * GPIOA5 SPI1_SCK
     * GPIOA6 SPI1_MISO
     * GPIOA7 SPI1_MOSI
     */

    /* SPI1_MISO = Pull-Up */
    SET_BIT(GPIOA->ODR, GPIO_ODR_ODR6_Msk);

    /*
     * SPI1_SCK = Output AF Push-Pull 50MHz
     * SPI1_MOSI = Output AF Push-Pull 50MHz
     * SPI1_MISO = Input Pull-Up/Pull-Down
     */
    MODIFY_REG(GPIOA->CRL,
               GPIO_CRL_MODE5_Msk
             | GPIO_CRL_CNF5_Msk
             | GPIO_CRL_MODE6_Msk
             | GPIO_CRL_CNF6_Msk
             | GPIO_CRL_MODE7_Msk
             | GPIO_CRL_CNF7_Msk,
               0x03 << GPIO_CRL_MODE5_Pos
             | 0x02 << GPIO_CRL_CNF5_Pos
             | 0x02 << GPIO_CRL_CNF6_Pos
             | 0x03 << GPIO_CRL_MODE7_Pos
             | 0x02 << GPIO_CRL_CNF7_Pos);
}
/* ------------------------------------------------------------------------- */
