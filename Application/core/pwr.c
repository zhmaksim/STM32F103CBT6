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

#include "pwr.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

static bool vdd_is_lower;

/* Private function prototypes --------------------------------------------- */

static void pwr_disable_backup_protect(void);

static void pwr_setup_pvd(void);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать PWR
 */
void pwr_init(void)
{
    /* Включить тактирование */
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN_Msk);

    pwr_disable_backup_protect();
    pwr_setup_pvd();

    /* Настроить NVIC */
    NVIC_SetPriority(PVD_IRQn, 5);
    NVIC_EnableIRQ(PVD_IRQn);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Отключить защиту от записи в домен резервного копирования
 */
static void pwr_disable_backup_protect(void)
{
    SET_BIT(PWR->CR, PWR_CR_DBP_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить PVD
 */
static void pwr_setup_pvd(void)
{
    MODIFY_REG(PWR->CR,
               PWR_CR_PVDE_Msk
             | PWR_CR_PLS_Msk,
               PWR_CR_PVDE_Msk              /* Включить PVD */
             | 0x07 << PWR_CR_PLS_Pos);     /* Настроить уровень PVD = 2.9V */
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывание PWR PVD
 */
void pwr_pvd_it_handler(void)
{
    /* Проверить статус прерывания EXTI */
    if (READ_BIT(EXTI->PR, EXTI_PR_PR16_Msk)) {
        /* Сбросить статус EXTI */
        SET_BIT(EXTI->PR, EXTI_PR_PR16_Msk);

        /* Обновить состояние VDD */
        if (READ_BIT(PWR->CSR, PWR_CSR_PVDO_Msk)) {
            vdd_is_lower = true;
        } else {
            vdd_is_lower = false;
        }
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Проверить состояние VDD
 *
 * @return          Состояние VDD
 */
inline bool pwr_vdd_is_lower(void)
{
    return vdd_is_lower;
}
/* ------------------------------------------------------------------------- */
