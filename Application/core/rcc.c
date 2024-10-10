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

#include "rcc.h"
#include "systick.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

#define RCC_HSERDY_TIMEOUT      100
#define RCC_LSERDY_TIMEOUT      5000

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void rcc_setup_hse(void);

static void rcc_setup_pll(void);

static void rcc_setup_bus(void);

static void rcc_setup_adc(void);

static void rcc_setup_usb(void);

static void rcc_setup_clksource_cpu(void);

static inline uint32_t rcc_tick(void);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать RCC
 */
void rcc_init(void)
{
    rcc_setup_hse();
    rcc_setup_pll();
    rcc_setup_bus();
    rcc_setup_adc();
    rcc_setup_usb();
    rcc_setup_clksource_cpu();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить HSE
 */
static void rcc_setup_hse(void)
{
    /* Включить HSE */
    SET_BIT(RCC->CR, RCC_CR_HSEON_Msk);

    uint32_t tickstart = rcc_tick();
    while (!READ_BIT(RCC->CR, RCC_CR_HSERDY_Msk)) {
        if (rcc_tick() - tickstart > RCC_HSERDY_TIMEOUT) {
            error();
        }
    }

    /* Включить CSS HSE */
    SET_BIT(RCC->CR, RCC_CR_CSSON_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить PLL
 */
static void rcc_setup_pll(void)
{
    /* Выключить PLL перед настройкой */
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON_Msk);

    /* Источник тактирования = HSE (8MHz) */
    SET_BIT(RCC->CFGR, RCC_CFGR_PLLSRC_Msk);

    /* Делитель PLL HSE = /1 (8MHz / 1 = 8MHz) */
    CLEAR_BIT(RCC->CFGR, RCC_CFGR_PLLXTPRE_Msk);

    /* Множитель PLL = x9 (8MHz * 9 = 72MHz */
    MODIFY_REG(RCC->CFGR,
               RCC_CFGR_PLLMULL_Msk,
               0x07 << RCC_CFGR_PLLMULL_Pos);

    /* Включить PLL */
    SET_BIT(RCC->CR, RCC_CR_PLLON_Msk);
    while (!READ_BIT(RCC->CR, RCC_CR_PLLRDY_Msk))
        ;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить AHB, APB1, APB2
 */
static void rcc_setup_bus(void)
{
    /* Делитель AHB = /1 (72MHz / 1 = 72MHz) */
    CLEAR_BIT(RCC->CFGR, RCC_CFGR_HPRE_Msk);

    /* Делитель APB1 = /2 (72MHz / 2 = 72MHz) */
    MODIFY_REG(RCC->CFGR,
               RCC_CFGR_PPRE1_Msk,
               0x04 << RCC_CFGR_PPRE1_Pos);

    /* Делитель APB2 = /1 (72MHz / 1 = 72MHz) */
    CLEAR_BIT(RCC->CFGR, RCC_CFGR_PPRE2_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить тактирование ADC
 */
static void rcc_setup_adc(void)
{
    /* Делитель ADC = /6 (72MHz / 6 = 12MHz) */
    MODIFY_REG(RCC->CFGR,
               RCC_CFGR_ADCPRE_Msk,
               0x02 << RCC_CFGR_ADCPRE_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить тактирование USB
 */
static void rcc_setup_usb(void)
{
    /* Делитель USB = /1.5 (72MHz / 1.5 = 48MHz) */
    CLEAR_BIT(RCC->CFGR, RCC_CFGR_USBPRE_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить источник тактирования CPU
 */
static void rcc_setup_clksource_cpu(void)
{
    /* Источник тактирования CPU = PLL */
    MODIFY_REG(RCC->CFGR,
               RCC_CFGR_SW_Msk,
               0x02 << RCC_CFGR_SW_Pos);
    while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS_Msk) !=
            0x02 << RCC_CFGR_SWS_Pos)
        ;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Значение системного таймера
 *
 * @return          Значение таймера
 */
static inline uint32_t rcc_tick(void)
{
    return systick_get_tick();
}
/* ------------------------------------------------------------------------- */
