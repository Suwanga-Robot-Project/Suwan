/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "iwdg.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#pragma pack(push, 1)   // 아래 구조체는 빈틈(패딩) 없이 꽉 채워서 저장하라는 지시
typedef struct
{
    uint8_t  header[2];      // 0xAA 0x55
    uint8_t  msg_type;       // 0x01 = ADC 데이터
    uint16_t seq_num;
    uint16_t mux_adc[16];
    uint16_t adc_ind[5];
    uint8_t  sw0_toggle;
    uint8_t  sw1_toggle;
    uint16_t crc;
} AdcPacket_t;
#pragma pack(pop)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
static uint16_t Calc_CRC16(const uint8_t *data, uint16_t length); //CRC 계산 함수 추가
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t mux_adc[16] = {0};
uint32_t adc_ind[5] = {0};
uint8_t sw0 = 0;
uint8_t sw1 = 0;
char msg[256];

uint8_t sw0_toggle = 0;
uint8_t sw1_toggle = 0;

uint8_t sw0_prev = 1;
uint8_t sw1_prev = 1;

static uint16_t seq_counter = 0;
static uint32_t crc_integrity_fail_count = 0;   // 자체검증 실패 누적 횟수
static volatile uint16_t adc_ind_dma_buf[5];  //  DMA가 값을 채워줄 버퍼
static volatile uint8_t adc_ind_dma_ready = 0; //  다 채워졌다는 신호
static uint32_t adc_dma_error_count = 0;       //  DMA 에러 발생 횟수 (재시작용)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MUX_Select(uint8_t ch)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, (ch >> 0) & 0x01);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, (ch >> 1) & 0x01);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, (ch >> 2) & 0x01);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, (ch >> 3) & 0x01);
}

void Read_MUX_ADC(uint8_t mux_ch, uint32_t *value)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    MUX_Select(mux_ch);

    for (volatile int i = 0; i < 300; i++);

    //  mux 읽을 때는 ADC를 "1채널만 스캔"하도록 명시적으로 전환
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    HAL_ADC_Init(&hadc1);

    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    sConfig.Offset = 0;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    *value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
}

void Read_ADC_Channel(uint32_t channel, uint32_t *value)
{ //지워도 되고 남겨두됨!
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    sConfig.Offset = 0;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    *value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB,
       GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6,
       GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  for (uint8_t i = 0; i < 16; i++)
	         {
	             Read_MUX_ADC(i, &mux_adc[i]);
	         }

	  // mux 읽기(1채널 모드)에서 adc_ind 읽기(4채널 스캔+DMA 모드)로 전환
	  hadc1.Init.ScanConvMode = ENABLE;
	  hadc1.Init.NbrOfConversion = 5;
	  HAL_ADC_Init(&hadc1);

	  ADC_ChannelConfTypeDef sConfig_ind = {0};

	  // 수정 — Rank 5 추가
	  sConfig_ind.Channel = ADC_CHANNEL_4;
	  sConfig_ind.Rank = 1;
	  sConfig_ind.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	  HAL_ADC_ConfigChannel(&hadc1, &sConfig_ind);

	  sConfig_ind.Channel = ADC_CHANNEL_9;
	  sConfig_ind.Rank = 2;
	  HAL_ADC_ConfigChannel(&hadc1, &sConfig_ind);

	  sConfig_ind.Channel = ADC_CHANNEL_10;
	  sConfig_ind.Rank = 3;
	  HAL_ADC_ConfigChannel(&hadc1, &sConfig_ind);

	  sConfig_ind.Channel = ADC_CHANNEL_11;
	  sConfig_ind.Rank = 4;
	  HAL_ADC_ConfigChannel(&hadc1, &sConfig_ind);

	  sConfig_ind.Channel = ADC_CHANNEL_5;   // ← 새로 추가 (PA5)
	  sConfig_ind.Rank = 5;
	  HAL_ADC_ConfigChannel(&hadc1, &sConfig_ind);

	  	  // [수정] DMA로 4채널 한 번에 읽기
		  adc_ind_dma_ready = 0;
		  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_ind_dma_buf, 5);

	  	  uint32_t dma_wait_start = HAL_GetTick();
	  	  while (!adc_ind_dma_ready)
	  	  {
	  		  if (HAL_GetTick() - dma_wait_start > 10)
	  	  	  {
	  			  adc_dma_error_count++;
	  	  	      break;
	  	  	  }
	  	  }

	  	// Rank 순서(PA4, PB1, PC0, PC1)대로 채워진 걸 기존 배열 순서(PB1, PC0, PC1, PA4)에 맞게 옮김
	  	  	  	  adc_ind[0] = adc_ind_dma_buf[1];  // PB1
	  		  	  adc_ind[1] = adc_ind_dma_buf[2];  // PC0
	  		  	  adc_ind[2] = adc_ind_dma_buf[3];  // PC1
	  		  	  adc_ind[3] = adc_ind_dma_buf[0];  // PA4
	  		  	  adc_ind[4] = adc_ind_dma_buf[4];  // PA5 (상하이동)  ← 새로 추가


	  		  	  // DMA 완전히 종료 — 다음 루프의 mux 폴링 읽기가 DMA 잔여 상태에 영향받지 않도록
	  		  	  HAL_ADC_Stop_DMA(&hadc1);


	  sw0 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET);
	  sw1 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET);

	  /* PB2 */
	  if (sw0_prev == 1 && sw0 == 0)
	  {
	      sw0_toggle ^= 1;
	  }
	  sw0_prev = sw0;

	  /* PB10 */
	  if (sw1_prev == 1 && sw1 == 0)
	  {
	      sw1_toggle ^= 1;
	  }
	  sw1_prev = sw1;



	  AdcPacket_t tx_packet;

	  tx_packet.header[0] = 0xAA;
	  tx_packet.header[1] = 0x55;
	  tx_packet.msg_type  = 0x01;
	  tx_packet.seq_num   = seq_counter++;

	  for (int i = 0; i < 16; i++)
	  {
		  tx_packet.mux_adc[i] = (uint16_t)mux_adc[i];
	  }
	  for (int i = 0; i < 5; i++)
	  {
		  tx_packet.adc_ind[i] = (uint16_t)adc_ind[i];
	  }
	  tx_packet.sw0_toggle = sw0_toggle;
	  tx_packet.sw1_toggle = sw1_toggle;

	  tx_packet.crc = Calc_CRC16((uint8_t*)&tx_packet, sizeof(AdcPacket_t) - sizeof(uint16_t));

	  // ===== 자체 검증: 방금 계산한 CRC가 지금 패킷 내용과 실제로 맞는지 재확인 =====
	  uint16_t self_check_crc = Calc_CRC16((uint8_t*)&tx_packet, sizeof(AdcPacket_t) - sizeof(uint16_t));

	  if (self_check_crc == tx_packet.crc)
	  {
		  // 검증 통과 → 정상 전송
	  	  HAL_UART_Transmit(
	  			  &huart2,
	  	  	      (uint8_t*)&tx_packet,
	  	  	      sizeof(AdcPacket_t),
				  HAL_MAX_DELAY
	  	  	  );
	  }
	  else
	  {
		  // 검증 실패 → 이번 패킷은 보내지 않고 건너뜀 (손상된 데이터를 내보내지 않음)
	  	  crc_integrity_fail_count++;
	  }
	  // =============================================


	  HAL_Delay(10);

	  HAL_IWDG_Refresh(&hiwdg);   // ← 이 줄 새로 추가  IWDG
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static uint16_t Calc_CRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF; //파이썬에서 만든 calc_crc16_ccitt()랑 완전히 똑같은 계산법
    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}
//  DMA로 ADC 변환이 끝났을 때 자동으로 호출되는 함수
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_ind_dma_ready = 1;   // 다 됐다는 신호만 켜줌
    }
}

//  DMA 에러 발생 시 자동으로 호출되는 함수
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_dma_error_count++;
        HAL_ADC_Stop_DMA(hadc);
        HAL_ADC_Start_DMA(hadc, (uint32_t*)adc_ind_dma_buf, 5);  //  에러 시 재시작
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
