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
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "debug.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Motor_SetDuty(uint16_t duty)
{
  if(duty > 7999) duty = 7999;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty);
}
void Motor_CheckBreak(void)
{
  if(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) != RESET)
  {
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET)
    {
      __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);

      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

      debug("Break Release!\r\n");
    }
  }
}
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[64];
uint8_t RxData[64];
void FDCAN1_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;

  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x000;
  sFilterConfig.FilterID2 = 0x7FF;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

   if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
  
  HAL_FDCAN_Start(&hfdcan1);
  
}
void FDCAN_BusOff_Recovery(void)
{
    HAL_FDCAN_Stop(&hfdcan1);

    __HAL_FDCAN_CLEAR_FLAG(&hfdcan1, FDCAN_FLAG_BUS_OFF);
    __HAL_FDCAN_CLEAR_FLAG(&hfdcan1, FDCAN_FLAG_ERROR_PASSIVE);
    __HAL_FDCAN_CLEAR_FLAG(&hfdcan1, FDCAN_FLAG_ERROR_WARNING);

    hfdcan1.Instance->TXFQS |= FDCAN_TXFQS_TFQF;

    hfdcan1.Instance->RXF0S |= FDCAN_RXF0S_F0F;
    hfdcan1.Instance->RXF1S |= FDCAN_RXF1S_F1F;

    hfdcan1.Instance->CCCR |= FDCAN_CCCR_CCE;
    hfdcan1.Instance->ECR = 0;
    hfdcan1.Instance->CCCR &= ~FDCAN_CCCR_CCE;

    HAL_FDCAN_Init(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan1);
}
HAL_StatusTypeDef FDCAN_Send_Frame(uint32_t id, uint8_t *data, uint8_t len)
{
  HAL_StatusTypeDef ret;
  FDCAN_TxHeaderTypeDef TxHeader = {0};
  uint32_t dlc;

  if(len <= 8)       dlc = (uint32_t)len;
  else if(len <= 12) dlc = FDCAN_DLC_BYTES_12;
  else if(len <= 16) dlc = FDCAN_DLC_BYTES_16;
  else if(len <= 20) dlc = FDCAN_DLC_BYTES_20;
  else if(len <= 24) dlc = FDCAN_DLC_BYTES_24;
  else if(len <= 32) dlc = FDCAN_DLC_BYTES_32;
  else if(len <= 48) dlc = FDCAN_DLC_BYTES_48;
  else               dlc = FDCAN_DLC_BYTES_64;
  
  TxHeader.Identifier = id;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = dlc;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_ON;
  TxHeader.FDFormat = FDCAN_FD_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 33;

  ret = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data);
  if (ret == HAL_ERROR)
  {
	  FDCAN_BusOff_Recovery();
	  ret = HAL_OK;
  }

  return ret;
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
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_InjectedStart(&hadc1);
  HAL_ADC_Start_IT(&hadc1);
  __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);

	FDCAN1_Config();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if 0
    HAL_Delay(500);
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
#endif

#if 0
    static float x = 0.0f;
    x += 0.01f;
    debug("Current x = %.2f\t\n", x);
    HAL_Delay(100);
#endif

#if 0
    static uint16_t pwm_duty = 2000;
    Motor_CheckBreak();
    Motor_SetDuty(pwm_duty);
    HAL_Delay(50);
#endif

#if 0
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 4000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 2000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 3600);
    extern float vin_adc, temp_adc, v_adc, u_adc;
    debug("vin_adc:%.3fV | temp_adc:%.3fV | v_adc:%.3fV | u_adc:%.3fV\r\n", vin_adc, temp_adc, v_adc, u_adc);
    HAL_Delay(100);
#endif

#if 1
    static uint8_t fdcan_Data = 0;
    TxData[0] = fdcan_Data++;
    FDCAN_Send_Frame(0x222, TxData, 32);
    FDCAN_Send_Frame(0x666, TxData, 64);
    HAL_Delay(500);
#endif

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
