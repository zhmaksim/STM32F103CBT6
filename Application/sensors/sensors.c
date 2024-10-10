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

#include "sensors.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

struct sensors_handle sensors = {
    .adc = ADC1,
    .dma = DMA1,
    .dma_channel = DMA1_Channel1,
    .dma_channel_nb = 1,
};

/* Private function prototypes --------------------------------------------- */

static void sensors_process(void *arg);

static void sensors_measure_vref_temp(uint32_t measure);

static float sensors_compute_vref(void);

static float sensors_compute_temp(void);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать датчики
 */
void sensors_init(void)
{
    assert(sensors.adc != NULL);

    /* Назначить Mutex и Event Group */
    if (sensors.adc == ADC1) {
        sensors.adc_mutex = adc1_mutex;
        sensors.adc_event_group = adc1_event_group;
    }

    xTaskCreate(sensors_process,
                "sensors",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать события датчиков
 *
 * @param[in]       arg: Указатель на параметры
 */
static void sensors_process(void *arg)
{
    static const TickType_t frequency = pdMS_TO_TICKS(25);

    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&last_wake_time, frequency);

        for (uint32_t measure = 0; measure < SENSORS_ADC_MEASURE_COUNT; measure++) {
            sensors_measure_vref_temp(measure);
        }

        taskENTER_CRITICAL();
        {
            sensors.vref = sensors_compute_vref();
            sensors.temp = sensors_compute_temp();
            sensors.state = SENSORS_READY;
        }
        taskEXIT_CRITICAL();
    }

    vTaskDelete(NULL);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывание DMA измерения ADC датчиков
 */
void sensors_dma_it_handler(void)
{
    uint32_t DMA_TCIF_Msk = 0x02 << ((sensors.dma_channel_nb - 1) * 4);
    uint32_t DMA_TEIF_Msk = 0x08 << ((sensors.dma_channel_nb - 1) * 4);

    /* Проверить статусы DMA */
    uint32_t DMA_ISR = READ_REG(sensors.dma->ISR);

    if (READ_BIT(DMA_ISR, DMA_TCIF_Msk)) {
        /* Очистить статусы DMA */
        SET_BIT(sensors.dma->IFCR, DMA_TCIF_Msk);

        /* Установить событие завершения преобразования */
        BaseType_t higher_priority_task_woken = pdFALSE;

        if (xEventGroupSetBitsFromISR(sensors.adc_event_group,
                                      ADC_EV_MEASURE_CPLT,
                                      &higher_priority_task_woken) == pdPASS) {
            portYIELD_FROM_ISR(higher_priority_task_woken);
        }
    } else if (READ_BIT(DMA_ISR, DMA_TEIF_Msk)) {
        /* Очистить статусы DMA */
        SET_BIT(sensors.dma->IFCR, DMA_TEIF_Msk);

        /* Установить событие завершения преобразования */
        BaseType_t higher_priority_task_woken = pdFALSE;

        if (xEventGroupSetBitsFromISR(sensors.adc_event_group,
                                      ADC_EV_MEASURE_ERR,
                                      &higher_priority_task_woken) == pdPASS) {
            portYIELD_FROM_ISR(higher_priority_task_woken);
        }
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Измерение VREF и TEMP
 *
 * @param[in]       measure: Преобразование
 */
static void sensors_measure_vref_temp(uint32_t measure)
{
    uint16_t adc_value[2];

    if (xSemaphoreTake(sensors.adc_mutex, pdMS_TO_TICKS(50)) != pdPASS) return;
    {
        /* Выключить DMA перед настройкой */
        CLEAR_BIT(sensors.dma_channel->CCR, DMA_CCR_EN_Msk);
        while (READ_BIT(sensors.dma_channel->CCR, DMA_CCR_EN_Msk))
            ;

        /* Настроить параметры DMA */
        WRITE_REG(sensors.dma_channel->CPAR, (uint32_t) &sensors.adc->DR);
        WRITE_REG(sensors.dma_channel->CMAR, (uint32_t) &adc_value);
        WRITE_REG(sensors.dma_channel->CNDTR, sizeof(adc_value) / sizeof(uint16_t));

        /* Очистить статусы DMA */
        uint32_t DMA_TCIF_Msk = 0x02 << ((sensors.dma_channel_nb - 1) * 4);
        uint32_t DMA_TEIF_Msk = 0x08 << ((sensors.dma_channel_nb - 1) * 4);

        SET_BIT(sensors.dma->IFCR,
                DMA_TCIF_Msk
              | DMA_TEIF_Msk);

        /* Включить DMA */
        SET_BIT(sensors.dma_channel->CCR, DMA_CCR_EN_Msk);

        /* Очистить статусы ADC */
        CLEAR_BIT(sensors.adc->SR,
                  ADC_SR_STRT_Msk
                | ADC_SR_EOS_Msk);

        /* Запустить преобразование */
        SET_BIT(sensors.adc->CR2, ADC_CR2_SWSTART_Msk);

        /* Ожидание завершения преобразования */
        EventBits_t bits = xEventGroupWaitBits(sensors.adc_event_group,
                                               ADC_EV_MEASURE_CPLT | ADC_EV_MEASURE_ERR,
                                               pdTRUE,
                                               pdFALSE,
                                               portMAX_DELAY);

        if (READ_BIT(bits, ADC_EV_MEASURE_CPLT_Msk)) {
            sensors.vref_adc_value[measure] = adc_value[0];
            sensors.temp_adc_value[measure] = adc_value[1];
        }

        /* Выключить DMA */
        CLEAR_BIT(sensors.dma_channel->CCR, DMA_CCR_EN_Msk);
    }
    xSemaphoreGive(sensors.adc_mutex);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Рассчитать значение опорного напряжения
 *
 *  @return         Значение опорного напряжения
 */
static float sensors_compute_vref(void)
{
    float sum = 0.0;
    uint32_t count = 0;

    for (uint32_t measure = 0; measure < SENSORS_ADC_MEASURE_COUNT; measure++) {
        if (sensors.vref_adc_value[measure]) {
            sum += 4095 * 1.20 / sensors.vref_adc_value[measure];
            count++;
        }
    }

    if (count > 0) {
        return (sum / count);
    } else {
        return 3.3;
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Рассчитать значение температуры
 *
 * @return          Значение температуры
 */
static float sensors_compute_temp(void)
{
    static const float slope = 0.0043;

    float sum = 0.0;
    uint32_t count = 0;

    for (uint32_t measure = 0; measure < SENSORS_ADC_MEASURE_COUNT; measure++) {
        if (sensors.temp_adc_value[measure]) {
            sum +=  (1.43 - ((float) sensors.temp_adc_value[measure] / 4095 * sensors.vref)) / slope + 25.0;
            count++;
        }
    }

    if (count > 0) {
        return (sum / count);
    } else {
        return -255.0;
    }
}
/* ------------------------------------------------------------------------- */
