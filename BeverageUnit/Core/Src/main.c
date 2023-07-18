/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Common config 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ACWPUMP1_PE2_Pin|ACWPUMP2_PE3_Pin|STEAMH_PE4_Pin|STEAML_PE5_Pin 
                          |ESPPUMP_PE6_Pin|MOTOR4_PH_PE12_Pin|ACWPUMP3_PE0_Pin|ACWPUMP4_PE1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BOILER_PC13_Pin|CBREW_PH_PC11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, VALVE2_PF0_Pin|VALVE3_PF1_Pin|GRINDER_PF2_Pin|VALVE1_PF3_Pin 
                          |RS485_DIR_Pin|HSW_PF11_Pin|HSW_PF12_Pin|WPUMP1_PF13_Pin 
                          |WPUMP2_PF14_Pin|WPUMP3_PF15_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, WPUMP4_PG0_Pin|SYRUP2_PH_PG4_Pin|SYRUP1_PH_PG6_Pin|MILK_PH_PG8_Pin 
                          |HSW_PG9_Pin|HSW_PG10_Pin|KTYPE_SPI3__CS2_Pin|PT100_SPI3__CS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR3_PH_PB12_Pin|SYRUP3_PH_PB13_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GADJ_PH_PD8_Pin|SYRUP4_PH_PD10_Pin|U12_PH_PD0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, FMILK_PH_PA9_Pin|SPUMP_PH_PA11_Pin|WPUMP_PH_PA15__Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ACWPUMP1_PE2_Pin ACWPUMP2_PE3_Pin STEAMH_PE4_Pin STEAML_PE5_Pin 
                           ESPPUMP_PE6_Pin MOTOR4_PH_PE12_Pin ACWPUMP3_PE0_Pin ACWPUMP4_PE1_Pin */
  GPIO_InitStruct.Pin = ACWPUMP1_PE2_Pin|ACWPUMP2_PE3_Pin|STEAMH_PE4_Pin|STEAML_PE5_Pin 
                          |ESPPUMP_PE6_Pin|MOTOR4_PH_PE12_Pin|ACWPUMP3_PE0_Pin|ACWPUMP4_PE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : BOILER_PC13_Pin CBREW_PH_PC11_Pin */
  GPIO_InitStruct.Pin = BOILER_PC13_Pin|CBREW_PH_PC11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VALVE2_PF0_Pin VALVE3_PF1_Pin GRINDER_PF2_Pin VALVE1_PF3_Pin 
                           RS485_DIR_Pin HSW_PF11_Pin HSW_PF12_Pin WPUMP1_PF13_Pin 
                           WPUMP2_PF14_Pin WPUMP3_PF15_Pin */
  GPIO_InitStruct.Pin = VALVE2_PF0_Pin|VALVE3_PF1_Pin|GRINDER_PF2_Pin|VALVE1_PF3_Pin 
                          |RS485_DIR_Pin|HSW_PF11_Pin|HSW_PF12_Pin|WPUMP1_PF13_Pin 
                          |WPUMP2_PF14_Pin|WPUMP3_PF15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : MTEMP_PF4_Pin MTEMP_PF5_Pin DHT11_DATA_Pin */
  GPIO_InitStruct.Pin = MTEMP_PF4_Pin|MTEMP_PF5_Pin|DHT11_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : DOORSW_EXTI6_Pin */
  GPIO_InitStruct.Pin = DOORSW_EXTI6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DOORSW_EXTI6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_TX_Pin */
  GPIO_InitStruct.Pin = RS485_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RS485_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_RX_Pin */
  GPIO_InitStruct.Pin = RS485_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RS485_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : WPUMP4_PG0_Pin SYRUP2_PH_PG4_Pin SYRUP1_PH_PG6_Pin MILK_PH_PG8_Pin 
                           HSW_PG9_Pin HSW_PG10_Pin KTYPE_SPI3__CS2_Pin PT100_SPI3__CS1_Pin */
  GPIO_InitStruct.Pin = WPUMP4_PG0_Pin|SYRUP2_PH_PG4_Pin|SYRUP1_PH_PG6_Pin|MILK_PH_PG8_Pin 
                          |HSW_PG9_Pin|HSW_PG10_Pin|KTYPE_SPI3__CS2_Pin|PT100_SPI3__CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : HALLA_EXTI1_Pin WASTEL_EXTI11_Pin WASTESW_EXTI12_Pin AIRBREAK_EXTI13_Pin */
  GPIO_InitStruct.Pin = HALLA_EXTI1_Pin|WASTEL_EXTI11_Pin|WASTESW_EXTI12_Pin|AIRBREAK_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : HALLB_PE7_Pin SENSORA_PE9_Pin */
  GPIO_InitStruct.Pin = HALLB_PE7_Pin|SENSORA_PE9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSORB_EXTI8_Pin MOTOR4_NF_EXTI10_Pin MOTOR3_NF_EXTI15_Pin */
  GPIO_InitStruct.Pin = SENSORB_EXTI8_Pin|MOTOR4_NF_EXTI10_Pin|MOTOR3_NF_EXTI15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR4_EN_TIM1_CH2_Pin MOTOR3_EN_TIM1_CH3_Pin GADJ_EN_TIM1_CH4_Pin */
  GPIO_InitStruct.Pin = MOTOR4_EN_TIM1_CH2_Pin|MOTOR3_EN_TIM1_CH3_Pin|GADJ_EN_TIM1_CH4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 TEMP_SPI3_SCK_Pin TEMP_SPI3_MISO_Pin TEMP_SPI3_MOSI_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_10|TEMP_SPI3_SCK_Pin|TEMP_SPI3_MISO_Pin|TEMP_SPI3_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR3_PH_PB12_Pin SYRUP3_PH_PB13_Pin */
  GPIO_InitStruct.Pin = MOTOR3_PH_PB12_Pin|SYRUP3_PH_PB13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SYRUP4_EN_TIM12_CH1_Pin SYRUP3_EN_TIM12_CH2_Pin */
  GPIO_InitStruct.Pin = SYRUP4_EN_TIM12_CH1_Pin|SYRUP3_EN_TIM12_CH2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GADJ_PH_PD8_Pin SYRUP4_PH_PD10_Pin U12_PH_PD0_Pin */
  GPIO_InitStruct.Pin = GADJ_PH_PD8_Pin|SYRUP4_PH_PD10_Pin|U12_PH_PD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : GADJ_NF_EXTI9_Pin FLOW2_EXTI2_Pin FLOW_EXTI7_Pin */
  GPIO_InitStruct.Pin = GADJ_NF_EXTI9_Pin|FLOW2_EXTI2_Pin|FLOW_EXTI7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SYRUP4_NF_PD11_Pin LQDET_PD1_Pin LQDET_PD3_Pin LQDET_PD4_Pin 
                           LQDET_PD5_Pin */
  GPIO_InitStruct.Pin = SYRUP4_NF_PD11_Pin|LQDET_PD1_Pin|LQDET_PD3_Pin|LQDET_PD4_Pin 
                          |LQDET_PD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SYRUP2_TIM4_CH1_Pin SYRUP1_TIM4_CH2_Pin MILK_EN_TIM4_CH3_Pin FMILK_EN_TIM4_CH4_Pin */
  GPIO_InitStruct.Pin = SYRUP2_TIM4_CH1_Pin|SYRUP1_TIM4_CH2_Pin|MILK_EN_TIM4_CH3_Pin|FMILK_EN_TIM4_CH4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SYRUP3_NF_PG2_Pin SYRUP2_NF_PG3_Pin SYRUP1_NF_PG5_Pin MILK_NF_PG7_Pin */
  GPIO_InitStruct.Pin = SYRUP3_NF_PG2_Pin|SYRUP2_NF_PG3_Pin|SYRUP1_NF_PG5_Pin|MILK_NF_PG7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : SPUMP_EN_TIM3_CH1_Pin WPUMP_EN_TIM3_CH2_Pin CBREW_EN_TIM3_CH3_Pin U12_EN_TIM3_CH4_Pin */
  GPIO_InitStruct.Pin = SPUMP_EN_TIM3_CH1_Pin|WPUMP_EN_TIM3_CH2_Pin|CBREW_EN_TIM3_CH3_Pin|U12_EN_TIM3_CH4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : FMILK_NF_PA8_Pin SPUMP_NF_PA10_Pin WPUMP_NF_PA12_Pin */
  GPIO_InitStruct.Pin = FMILK_NF_PA8_Pin|SPUMP_NF_PA10_Pin|WPUMP_NF_PA12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : FMILK_PH_PA9_Pin SPUMP_PH_PA11_Pin WPUMP_PH_PA15__Pin */
  GPIO_InitStruct.Pin = FMILK_PH_PA9_Pin|SPUMP_PH_PA11_Pin|WPUMP_PH_PA15__Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CBREW_NF_PC10_Pin U12_NF_PC12_Pin */
  GPIO_InitStruct.Pin = CBREW_NF_PC10_Pin|U12_NF_PC12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LOADCELL_I2C1_SCL_Pin LOADCELL_I2C1_SDA_Pin */
  GPIO_InitStruct.Pin = LOADCELL_I2C1_SCL_Pin|LOADCELL_I2C1_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_TIM1_ENABLE();

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_TIM4_ENABLE();

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_TIM3_ENABLE();

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
