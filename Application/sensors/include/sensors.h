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

#ifndef SENSORS_H_
#define SENSORS_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "main.h"
#include "adc.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

#define SENSORS_ADC_MEASURE_COUNT       32

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение перечисления состояния датчиков
 */
enum sensors_state {
    SENSORS_NOT_READY,
    SENSORS_READY,
};


/**
 * @brief           Определение структуры данных обработчика датчиков
 */
struct sensors_handle {
    ADC_TypeDef            *adc;                                            /*!< Указатель на структуру данных ADC */

    SemaphoreHandle_t       adc_mutex;                                      /*!< Mutex ADC */

    EventGroupHandle_t      adc_event_group;                                /*!< Event Group ADC */

    DMA_TypeDef            *dma;                                            /*!< Указатель на структуру данных DMA */

    DMA_Channel_TypeDef    *dma_channel;                                    /*!< Указатель на структуру данных канала DMA */

    uint8_t                 dma_channel_nb;                                 /*!< Номер канала DMA */

    enum sensors_state      state;                                          /*!< Состояние */

    uint16_t                vref_adc_value[SENSORS_ADC_MEASURE_COUNT];      /*!< Значения ADC датчика опорного напряжения */

    float                   vref;                                           /*!< Опорное напряжение */

    uint16_t                temp_adc_value[SENSORS_ADC_MEASURE_COUNT];      /*!< Значения ADC датчика температуры */

    float                   temp;                                           /*!< Температура */
};

/* Exported variables ------------------------------------------------------ */

extern struct sensors_handle sensors;

/* Exported function prototypes -------------------------------------------- */

void sensors_init(void);

void sensors_dma_it_handler(void);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SENSORS_H_ */
