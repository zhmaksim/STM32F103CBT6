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

#include "adc.h"
#include "systick.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

SemaphoreHandle_t adc1_mutex;
EventGroupHandle_t adc1_event_group;

/* Private function prototypes --------------------------------------------- */

static void adc_delay(uint32_t delay);

static inline uint32_t adc_tick(void);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать ADC
 */
void adc_init(void)
{
    /* Настройка ADC ------------------------------------------------------- */
    /* Включить тактирование */
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN_Msk);

    /* Настроить выравнивание данных = Right */
    CLEAR_BIT(ADC1->CR2, ADC_CR2_ALIGN_Msk);

    /* Включить преобразование VREF и TEMP */
    SET_BIT(ADC1->CR2, ADC_CR2_TSVREFE_Msk);

    /* Настроить Cycles = 239.5 для VREF и TEMP */
    SET_BIT(ADC1->SMPR1,
            ADC_SMPR1_SMP16_Msk
          | ADC_SMPR1_SMP17_Msk);

    /* Включить режим сканирования */
    SET_BIT(ADC1->CR1, ADC_CR1_SCAN_Msk);

    /* Настроить количество последовательность преобразований */
    MODIFY_REG(ADC1->SQR1,
               ADC_SQR1_L_Msk,
               0x01 << ADC_SQR1_L_Pos);
    /* VREF = №1 и TEMP = №2 */
    MODIFY_REG(ADC1->SQR3,
               ADC_SQR3_SQ1_Msk
             | ADC_SQR3_SQ2_Msk,
               0x11 << ADC_SQR3_SQ1_Pos
             | 0x10 << ADC_SQR3_SQ2_Pos);

    /* Включить DMA */
    SET_BIT(ADC1->CR2, ADC_CR2_DMA_Msk);

    /* Запуск преборазования = Software */
    MODIFY_REG(ADC1->CR2,
               ADC_CR2_EXTSEL_Msk,
               ADC_CR2_EXTTRIG_Msk
            |  0x07 << ADC_CR2_EXTSEL_Pos);

    /* Включить ADC */
    SET_BIT(ADC1->CR2, ADC_CR2_ADON_Msk);
    /* Задержка */
    adc_delay(3);

    /* Сброс калибровки */
    SET_BIT(ADC1->CR2, ADC_CR2_RSTCAL_Msk);
    while (READ_BIT(ADC1->CR2, ADC_CR2_RSTCAL_Msk))
        ;
    /* Калибровка */
    SET_BIT(ADC1->CR2, ADC_CR2_CAL_Msk);
    while (READ_BIT(ADC1->CR2, ADC_CR2_CAL_Msk))
        ;


    /* Настройка DMA ------------------------------------------------------- */
    /* Включить тактирование */
    SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN_Msk);

    /* Выключить DMA перед настройкой */
    CLEAR_BIT(DMA1_Channel1->CCR, DMA_CCR_EN_Msk);
    while (READ_BIT(DMA1_Channel1->CCR, DMA_CCR_EN_Msk))
        ;

    /* Настроить DMA для ADC преобразований VREF и TEMP */
    MODIFY_REG(DMA1_Channel1->CCR,
               DMA_CCR_DIR_Msk
             | DMA_CCR_CIRC_Msk
             | DMA_CCR_PINC_Msk
             | DMA_CCR_MINC_Msk
             | DMA_CCR_PSIZE_Msk
             | DMA_CCR_MSIZE_Msk
             | DMA_CCR_PL_Msk,
               0x00 << DMA_CCR_DIR_Pos         /* Peripheral - Memory */
             | 0x01 << DMA_CCR_MINC_Pos        /* Memory increment mode */
             | 0x01 << DMA_CCR_PSIZE_Pos       /* 16bits peripheral size */
             | 0x01 << DMA_CCR_MSIZE_Pos       /* 16bits memory size*/
             | 0x00 << DMA_CCR_PL_Pos);        /* Low priority */

    /* Включить прерывания */
    SET_BIT(DMA1_Channel1->CCR,
            DMA_CCR_TCIE_Msk
          | DMA_CCR_TEIE_Msk);

    /* Настроить NVIC */
    NVIC_SetPriority(DMA1_Channel1_IRQn, 15);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    /* Инициализировать Mutex */
    adc1_mutex = xSemaphoreCreateMutex();
    if (adc1_mutex == NULL) error();

    /* Инициализировать Event Group */
    adc1_event_group = xEventGroupCreate();
    if (adc1_event_group == NULL) error();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Задержка (блокирующий метод)
 *
 * @param[in]       delay: Значение задержки (мс)
 */
static void adc_delay(uint32_t delay)
{
    uint32_t tickstart = adc_tick();

    while (adc_tick() - tickstart < delay)
        ;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Значение системного таймера
 *
 * @return          Значение таймера
 */
static inline uint32_t adc_tick(void)
{
    return systick_get_tick();
}
/* ------------------------------------------------------------------------- */
