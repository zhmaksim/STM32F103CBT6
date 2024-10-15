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

#ifndef SPI_H_
#define SPI_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "main.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

#define SPI_EV_TX_RX_CPLT_Pos           1
#define SPI_EV_TX_RX_CPLT_Msk           (0x01 << SPI_EV_TX_RX_CPLT_Pos)
#define SPI_EV_TX_RX_CPLT               SPI_EV_TX_RX_CPLT_Msk

#define SPI_EV_ERR_Pos                  7
#define SPI_EV_ERR_Msk                  (0x01 << SPI_EV_ERR_Pos)
#define SPI_EV_ERR                      SPI_EV_ERR_Msk

/* Exported types ---------------------------------------------------------- */

/* Exported variables ------------------------------------------------------ */

extern SemaphoreHandle_t spi1_mutex;
extern EventGroupHandle_t spi1_event_group;

/* Exported function prototypes -------------------------------------------- */

void spi_init(void);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SPI_H_ */
