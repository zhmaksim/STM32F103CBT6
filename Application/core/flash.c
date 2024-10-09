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

#include "flash.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void flash_setup_latency(void);

static void flash_enable_prefetch(void);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать FLASH
 */
void flash_init(void)
{
    flash_setup_latency();
    flash_enable_prefetch();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить задержку чтения флэш-памяти
 */
static void flash_setup_latency(void)
{
    /* Задержка чтения флэш-памяти = 2WS */
    MODIFY_REG(FLASH->ACR,
               FLASH_ACR_LATENCY_Msk,
               0x02 << FLASH_ACR_LATENCY_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Включить предварительную выборку данных
 */
static void flash_enable_prefetch(void)
{
    SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE_Msk);
    while (!READ_BIT(FLASH->ACR, FLASH_ACR_PRFTBS_Msk))
        ;
}
/* ------------------------------------------------------------------------- */
