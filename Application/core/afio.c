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

#include "afio.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void afio_setup_debug(void);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать AFIO
 */
void afio_init(void)
{
    /* Включить тактирование */
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN_Msk);

    afio_setup_debug();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить Debug
 */
static void afio_setup_debug(void)
{
    /* Debug = SW */
    MODIFY_REG(AFIO->MAPR,
               AFIO_MAPR_SWJ_CFG_Msk,
               0x02 << AFIO_MAPR_SWJ_CFG_Pos);
}
/* ------------------------------------------------------------------------- */
