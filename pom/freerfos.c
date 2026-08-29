/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h> // memcpy 사용
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint32_t mux_adc[16];
extern uint32_t adc_ind[4];
extern uint8_t sw0_toggle;
extern uint8_t sw1_toggle;
extern uint16_t seq_counter;
extern uint32_t crc_integrity_fail_count;
extern volatile uint16_t adc_ind_dma_buf[4];
extern uint32_t adc_dma_error_count;
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart2;
extern IWDG_HandleTypeDef hiwdg;

volatile uint32_t g_adcHeartbeat = 0;
volatile uint32_t g_uartHeartbeat = 0;
volatile uint32_t g_fsmHeartbeat = 0;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for ADCProcessTask */
osThreadId_t ADCProcessTaskHandle;
const osThreadAttr_t ADCProcessTask_attributes = {
    .name = "ADCProcessTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for UARTCommTask */
osThreadId_t UARTCommTaskHandle;
const osThreadAttr_t UARTCommTask_attributes = {
    .name = "UARTCommTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};
/* Definitions for WatchdogTask */
osThreadId_t WatchdogTaskHandle;
const osThreadAttr_t WatchdogTask_attributes = {
    .name = "WatchdogTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};
/* Definitions for FSMTask */
osThreadId_t FSMTaskHandle;
const osThreadAttr_t FSMTask_attributes = {
    .name = "FSMTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for UARTRxQueue */
osMessageQueueId_t UARTRxQueueHandle;
const osMessageQueueAttr_t UARTRxQueue_attributes = {
    .name = "UARTRxQueue"};
/* Definitions for SensorDataMutex */
osMutexId_t SensorDataMutexHandle;
const osMutexAttr_t SensorDataMutex_attributes = {
    .name = "SensorDataMutex"};
/* Definitions for ADCConvSemaphore */
osSemaphoreId_t ADCConvSemaphoreHandle;
const osSemaphoreAttr_t ADCConvSemaphore_attributes = {
    .name = "ADCConvSemaphore"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartADCProcessTask(void *argument);
void StartUARTCommTask(void *argument);
void StartWatchdogTask(void *argument);
void StartFSMTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */
    /* Create the mutex(es) */
    /* creation of SensorDataMutex */
    SensorDataMutexHandle = osMutexNew(&SensorDataMutex_attributes);

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* Create the semaphores(s) */
    /* creation of ADCConvSemaphore */
    ADCConvSemaphoreHandle = osSemaphoreNew(1, 0, &ADCConvSemaphore_attributes);

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the queue(s) */
    /* creation of UARTRxQueue */
    UARTRxQueueHandle = osMessageQueueNew(10, 4, &UARTRxQueue_attributes);

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* creation of ADCProcessTask */
    ADCProcessTaskHandle = osThreadNew(StartADCProcessTask, NULL, &ADCProcessTask_attributes);

    /* creation of UARTCommTask */
    UARTCommTaskHandle = osThreadNew(StartUARTCommTask, NULL, &UARTCommTask_attributes);

    /* creation of WatchdogTask */
    WatchdogTaskHandle = osThreadNew(StartWatchdogTask, NULL, &WatchdogTask_attributes);

    /* creation of FSMTask */
    FSMTaskHandle = osThreadNew(StartFSMTask, NULL, &FSMTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
    /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartADCProcessTask */
/**
 * @brief Function implementing the ADCProcessTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartADCProcessTask */
void StartADCProcessTask(void *argument)
{
    /* USER CODE BEGIN StartADCProcessTask */
    ADC_ChannelConfTypeDef sConfig_ind = {0};
    uint32_t local_mux[16];
    uint32_t local_ind[4];
    uint8_t sw0 = 0, sw1 = 0;
    uint8_t sw0_prev = 1, sw1_prev = 1;
    uint8_t local_sw0_toggle = 0, local_sw1_toggle = 0;
    /* Infinite loop */
    for (;;)
    {
        /* 1) mux 채널 16개 폴링 읽기 */
        for (uint8_t i = 0; i < 16; i++)
        {
            Read_MUX_ADC(i, &local_mux[i]);
        }

        /* 2) 4채널 스캔+DMA 모드로 전환 */
        hadc1.Init.ScanConvMode = ENABLE;
        hadc1.Init.NbrOfConversion = 4;
        HAL_ADC_Init(&hadc1);

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

        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_ind_dma_buf, 4);

        /* 3) DMA 완료를 세마포어로 대기 (기존 busy-wait 폴링 대체) */
        if (osSemaphoreAcquire(ADCConvSemaphoreHandle, 10) != osOK)
        {
            adc_dma_error_count++; /* 타임아웃 = DMA 실패로 간주 */
        }

        local_ind[0] = adc_ind_dma_buf[1]; /* PB1 */
        local_ind[1] = adc_ind_dma_buf[2]; /* PC0 */
        local_ind[2] = adc_ind_dma_buf[3]; /* PC1 */
        local_ind[3] = adc_ind_dma_buf[0]; /* PA4 */

        HAL_ADC_Stop_DMA(&hadc1);

        /* 4) 스위치 토글 감지 */
        sw0 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET);
        sw1 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET);

        if (sw0_prev == 1 && sw0 == 0)
        {
            local_sw0_toggle ^= 1;
        }
        sw0_prev = sw0;

        if (sw1_prev == 1 && sw1 == 0)
        {
            local_sw1_toggle ^= 1;
        }
        sw1_prev = sw1;

        /* 5) 공유 데이터 갱신 (Mutex로 보호) */
        osMutexAcquire(SensorDataMutexHandle, osWaitForever);
        memcpy(mux_adc, local_mux, sizeof(local_mux));
        memcpy(adc_ind, local_ind, sizeof(local_ind));
        sw0_toggle = local_sw0_toggle;
        sw1_toggle = local_sw1_toggle;
        osMutexRelease(SensorDataMutexHandle);

        g_adcHeartbeat = osKernelGetTickCount();
        osDelay(10);
    }
    /* USER CODE END StartADCProcessTask */
}

/* USER CODE BEGIN Header_StartUARTCommTask */
/**
 * @brief Function implementing the UARTCommTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUARTCommTask */
void StartUARTCommTask(void *argument)
{
    /* USER CODE BEGIN StartUARTCommTask */
    AdcPacket_t tx_packet;
    uint32_t local_mux[16];
    uint32_t local_ind[4];
    uint8_t local_sw0, local_sw1;
    uint32_t tick = osKernelGetTickCount();
    const uint32_t period_ms = 50; /* 20Hz */
                                   /* Infinite loop */
    for (;;)
    {
        tick += period_ms;

        osMutexAcquire(SensorDataMutexHandle, osWaitForever);
        memcpy(local_mux, mux_adc, sizeof(local_mux));
        memcpy(local_ind, adc_ind, sizeof(local_ind));
        local_sw0 = sw0_toggle;
        local_sw1 = sw1_toggle;
        osMutexRelease(SensorDataMutexHandle);

        tx_packet.header[0] = 0xAA;
        tx_packet.header[1] = 0x55;
        tx_packet.msg_type = 0x01;
        tx_packet.seq_num = seq_counter++;

        for (int i = 0; i < 16; i++)
            tx_packet.mux_adc[i] = (uint16_t)local_mux[i];
        for (int i = 0; i < 4; i++)
            tx_packet.adc_ind[i] = (uint16_t)local_ind[i];
        tx_packet.sw0_toggle = local_sw0;
        tx_packet.sw1_toggle = local_sw1;

        tx_packet.crc = Calc_CRC16((uint8_t *)&tx_packet, sizeof(AdcPacket_t) - sizeof(uint16_t));

        uint16_t self_check_crc = Calc_CRC16((uint8_t *)&tx_packet, sizeof(AdcPacket_t) - sizeof(uint16_t));

        if (self_check_crc == tx_packet.crc)
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)&tx_packet, sizeof(AdcPacket_t), 20);
        }
        else
        {
            crc_integrity_fail_count++;
        }

        g_uartHeartbeat = osKernelGetTickCount();
        osDelayUntil(tick);
    }
    /* USER CODE END StartUARTCommTask */
}

/* USER CODE BEGIN Header_StartWatchdogTask */
/**
 * @brief Function implementing the WatchdogTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartWatchdogTask */
void StartWatchdogTask(void *argument)
{
    /* USER CODE BEGIN StartWatchdogTask */
    const uint32_t HEARTBEAT_TIMEOUT_MS = 300;
    /* Infinite loop */
    for (;;)
    {
        uint32_t now = osKernelGetTickCount();

        uint8_t adc_alive = (now - g_adcHeartbeat) < HEARTBEAT_TIMEOUT_MS;
        uint8_t uart_alive = (now - g_uartHeartbeat) < HEARTBEAT_TIMEOUT_MS;
        uint8_t fsm_alive = (now - g_fsmHeartbeat) < HEARTBEAT_TIMEOUT_MS;

        if (adc_alive && uart_alive && fsm_alive)
        {
            HAL_IWDG_Refresh(&hiwdg);
        }
        /* 하나라도 heartbeat가 끊기면 refresh를 건너뜀 → IWDG가 자동으로 재시작시킴 */

        osDelay(100);
    }
    /* USER CODE END StartWatchdogTask */
}

/* USER CODE BEGIN Header_StartFSMTask */
/**
 * @brief Function implementing the FSMTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartFSMTask */
void StartFSMTask(void *argument)
{
    /* USER CODE BEGIN StartFSMTask */
    /* Infinite loop */
    for (;;)
    {
        /* 추후: adc_dma_error_count, crc_integrity_fail_count 등을 보고
           ERROR 상태로 래치하는 로직 여기에 추가 예정 */

        g_fsmHeartbeat = osKernelGetTickCount();
        osDelay(50);
    }
    /* USER CODE END StartFSMTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
