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

#include "exti.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void exti_pvd_init(void);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать EXTI
 */
void exti_init(void)
{
    exti_pvd_init();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать EXTI PVD
 */
static void exti_pvd_init(void)
{
    /* Разрешить прерывание EXTI PVD */
    SET_BIT(EXTI->IMR, EXTI_IMR_MR16_Msk);

    /* Включить Rising Trigger */
    SET_BIT(EXTI->RTSR, EXTI_RTSR_TR16_Msk);

    /* Включить Falling Trigger */
    SET_BIT(EXTI->FTSR, EXTI_FTSR_TR16_Msk);
}
/* ------------------------------------------------------------------------- */
