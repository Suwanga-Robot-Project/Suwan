/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#pragma pack(push, 1)
    typedef struct
    {
        uint8_t header[2]; // 0xAA 0x55
        uint8_t msg_type;  // 0x01 = ADC 데이터
        uint16_t seq_num;
        uint16_t mux_adc[16];
        uint16_t adc_ind[4];
        uint8_t sw0_toggle;
        uint8_t sw1_toggle;
        uint16_t crc;
    } AdcPacket_t;
#pragma pack(pop)
    /* USER CODE END ET */

    /* Exported constants --------------------------------------------------------*/
    /* USER CODE BEGIN EC */

    /* USER CODE END EC */

    /* Exported macro ------------------------------------------------------------*/
    /* USER CODE BEGIN EM */

    /* USER CODE END EM */

    /* Exported functions prototypes ---------------------------------------------*/
    void Error_Handler(void);

    /* USER CODE BEGIN EFP */
    uint16_t Calc_CRC16(const uint8_t *data, uint16_t length);
    void MUX_Select(uint8_t ch);
    void Read_MUX_ADC(uint8_t mux_ch, uint32_t *value);

    extern uint32_t mux_adc[16];
    extern uint32_t adc_ind[4];
    extern uint8_t sw0_toggle;
    extern uint8_t sw1_toggle;
    extern uint16_t seq_counter;
    extern uint32_t crc_integrity_fail_count;
    extern volatile uint16_t adc_ind_dma_buf[4];
    extern uint32_t adc_dma_error_count;
    /* USER CODE END EFP */

    /* Private defines -----------------------------------------------------------*/

    /* USER CODE BEGIN Private defines */

    /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
