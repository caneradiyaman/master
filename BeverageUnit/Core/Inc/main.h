/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ACWPUMP1_PE2_Pin GPIO_PIN_2
#define ACWPUMP1_PE2_GPIO_Port GPIOE
#define ACWPUMP2_PE3_Pin GPIO_PIN_3
#define ACWPUMP2_PE3_GPIO_Port GPIOE
#define STEAMH_PE4_Pin GPIO_PIN_4
#define STEAMH_PE4_GPIO_Port GPIOE
#define STEAML_PE5_Pin GPIO_PIN_5
#define STEAML_PE5_GPIO_Port GPIOE
#define ESPPUMP_PE6_Pin GPIO_PIN_6
#define ESPPUMP_PE6_GPIO_Port GPIOE
#define BOILER_PC13_Pin GPIO_PIN_13
#define BOILER_PC13_GPIO_Port GPIOC
#define VALVE2_PF0_Pin GPIO_PIN_0
#define VALVE2_PF0_GPIO_Port GPIOF
#define VALVE3_PF1_Pin GPIO_PIN_1
#define VALVE3_PF1_GPIO_Port GPIOF
#define GRINDER_PF2_Pin GPIO_PIN_2
#define GRINDER_PF2_GPIO_Port GPIOF
#define VALVE1_PF3_Pin GPIO_PIN_3
#define VALVE1_PF3_GPIO_Port GPIOF
#define MTEMP_PF4_Pin GPIO_PIN_4
#define MTEMP_PF4_GPIO_Port GPIOF
#define MTEMP_PF5_Pin GPIO_PIN_5
#define MTEMP_PF5_GPIO_Port GPIOF
#define DOORSW_EXTI6_Pin GPIO_PIN_6
#define DOORSW_EXTI6_GPIO_Port GPIOF
#define DHT11_DATA_Pin GPIO_PIN_7
#define DHT11_DATA_GPIO_Port GPIOF
#define STEMP_ADC3_IN4_Pin GPIO_PIN_8
#define STEMP_ADC3_IN4_GPIO_Port GPIOF
#define RS485_DIR_Pin GPIO_PIN_9
#define RS485_DIR_GPIO_Port GPIOF
#define EFAST_ADC3_IN8_Pin GPIO_PIN_10
#define EFAST_ADC3_IN8_GPIO_Port GPIOF
#define BEAN_ADC3_IN10_Pin GPIO_PIN_0
#define BEAN_ADC3_IN10_GPIO_Port GPIOC
#define PT100_ADC2_IN11_Pin GPIO_PIN_1
#define PT100_ADC2_IN11_GPIO_Port GPIOC
#define MOTOR4_ADC2_IN12_Pin GPIO_PIN_2
#define MOTOR4_ADC2_IN12_GPIO_Port GPIOC
#define CBREW_ADC2_IN13_Pin GPIO_PIN_3
#define CBREW_ADC2_IN13_GPIO_Port GPIOC
#define WPUMP_ADC2_IN0_Pin GPIO_PIN_0
#define WPUMP_ADC2_IN0_GPIO_Port GPIOA
#define SPUMP_ADC2_IN1_Pin GPIO_PIN_1
#define SPUMP_ADC2_IN1_GPIO_Port GPIOA
#define RS485_TX_Pin GPIO_PIN_2
#define RS485_TX_GPIO_Port GPIOA
#define RS485_RX_Pin GPIO_PIN_3
#define RS485_RX_GPIO_Port GPIOA
#define FMILK_ADC2_IN4_Pin GPIO_PIN_4
#define FMILK_ADC2_IN4_GPIO_Port GPIOA
#define MILK_ADC2_IN5_Pin GPIO_PIN_5
#define MILK_ADC2_IN5_GPIO_Port GPIOA
#define SYRUP1_ADC2_IN6_Pin GPIO_PIN_6
#define SYRUP1_ADC2_IN6_GPIO_Port GPIOA
#define SYRUP2_ADC2_IN7_Pin GPIO_PIN_7
#define SYRUP2_ADC2_IN7_GPIO_Port GPIOA
#define SYRUP3_ADC2_IN14_Pin GPIO_PIN_4
#define SYRUP3_ADC2_IN14_GPIO_Port GPIOC
#define SYRUP4_ADC2_IN15_Pin GPIO_PIN_5
#define SYRUP4_ADC2_IN15_GPIO_Port GPIOC
#define GADJ_ADC2_IN8_Pin GPIO_PIN_0
#define GADJ_ADC2_IN8_GPIO_Port GPIOB
#define MOTOR3_ADC2_IN9_Pin GPIO_PIN_1
#define MOTOR3_ADC2_IN9_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define HSW_PF11_Pin GPIO_PIN_11
#define HSW_PF11_GPIO_Port GPIOF
#define HSW_PF12_Pin GPIO_PIN_12
#define HSW_PF12_GPIO_Port GPIOF
#define WPUMP1_PF13_Pin GPIO_PIN_13
#define WPUMP1_PF13_GPIO_Port GPIOF
#define WPUMP2_PF14_Pin GPIO_PIN_14
#define WPUMP2_PF14_GPIO_Port GPIOF
#define WPUMP3_PF15_Pin GPIO_PIN_15
#define WPUMP3_PF15_GPIO_Port GPIOF
#define WPUMP4_PG0_Pin GPIO_PIN_0
#define WPUMP4_PG0_GPIO_Port GPIOG
#define HALLA_EXTI1_Pin GPIO_PIN_1
#define HALLA_EXTI1_GPIO_Port GPIOG
#define HALLB_PE7_Pin GPIO_PIN_7
#define HALLB_PE7_GPIO_Port GPIOE
#define SENSORB_EXTI8_Pin GPIO_PIN_8
#define SENSORB_EXTI8_GPIO_Port GPIOE
#define SENSORA_PE9_Pin GPIO_PIN_9
#define SENSORA_PE9_GPIO_Port GPIOE
#define MOTOR4_NF_EXTI10_Pin GPIO_PIN_10
#define MOTOR4_NF_EXTI10_GPIO_Port GPIOE
#define MOTOR4_EN_TIM1_CH2_Pin GPIO_PIN_11
#define MOTOR4_EN_TIM1_CH2_GPIO_Port GPIOE
#define MOTOR4_PH_PE12_Pin GPIO_PIN_12
#define MOTOR4_PH_PE12_GPIO_Port GPIOE
#define MOTOR3_EN_TIM1_CH3_Pin GPIO_PIN_13
#define MOTOR3_EN_TIM1_CH3_GPIO_Port GPIOE
#define GADJ_EN_TIM1_CH4_Pin GPIO_PIN_14
#define GADJ_EN_TIM1_CH4_GPIO_Port GPIOE
#define MOTOR3_NF_EXTI15_Pin GPIO_PIN_15
#define MOTOR3_NF_EXTI15_GPIO_Port GPIOE
#define MOTOR3_PH_PB12_Pin GPIO_PIN_12
#define MOTOR3_PH_PB12_GPIO_Port GPIOB
#define SYRUP3_PH_PB13_Pin GPIO_PIN_13
#define SYRUP3_PH_PB13_GPIO_Port GPIOB
#define SYRUP4_EN_TIM12_CH1_Pin GPIO_PIN_14
#define SYRUP4_EN_TIM12_CH1_GPIO_Port GPIOB
#define SYRUP3_EN_TIM12_CH2_Pin GPIO_PIN_15
#define SYRUP3_EN_TIM12_CH2_GPIO_Port GPIOB
#define GADJ_PH_PD8_Pin GPIO_PIN_8
#define GADJ_PH_PD8_GPIO_Port GPIOD
#define GADJ_NF_EXTI9_Pin GPIO_PIN_9
#define GADJ_NF_EXTI9_GPIO_Port GPIOD
#define SYRUP4_PH_PD10_Pin GPIO_PIN_10
#define SYRUP4_PH_PD10_GPIO_Port GPIOD
#define SYRUP4_NF_PD11_Pin GPIO_PIN_11
#define SYRUP4_NF_PD11_GPIO_Port GPIOD
#define SYRUP2_TIM4_CH1_Pin GPIO_PIN_12
#define SYRUP2_TIM4_CH1_GPIO_Port GPIOD
#define SYRUP1_TIM4_CH2_Pin GPIO_PIN_13
#define SYRUP1_TIM4_CH2_GPIO_Port GPIOD
#define MILK_EN_TIM4_CH3_Pin GPIO_PIN_14
#define MILK_EN_TIM4_CH3_GPIO_Port GPIOD
#define FMILK_EN_TIM4_CH4_Pin GPIO_PIN_15
#define FMILK_EN_TIM4_CH4_GPIO_Port GPIOD
#define SYRUP3_NF_PG2_Pin GPIO_PIN_2
#define SYRUP3_NF_PG2_GPIO_Port GPIOG
#define SYRUP2_NF_PG3_Pin GPIO_PIN_3
#define SYRUP2_NF_PG3_GPIO_Port GPIOG
#define SYRUP2_PH_PG4_Pin GPIO_PIN_4
#define SYRUP2_PH_PG4_GPIO_Port GPIOG
#define SYRUP1_NF_PG5_Pin GPIO_PIN_5
#define SYRUP1_NF_PG5_GPIO_Port GPIOG
#define SYRUP1_PH_PG6_Pin GPIO_PIN_6
#define SYRUP1_PH_PG6_GPIO_Port GPIOG
#define MILK_NF_PG7_Pin GPIO_PIN_7
#define MILK_NF_PG7_GPIO_Port GPIOG
#define MILK_PH_PG8_Pin GPIO_PIN_8
#define MILK_PH_PG8_GPIO_Port GPIOG
#define SPUMP_EN_TIM3_CH1_Pin GPIO_PIN_6
#define SPUMP_EN_TIM3_CH1_GPIO_Port GPIOC
#define WPUMP_EN_TIM3_CH2_Pin GPIO_PIN_7
#define WPUMP_EN_TIM3_CH2_GPIO_Port GPIOC
#define CBREW_EN_TIM3_CH3_Pin GPIO_PIN_8
#define CBREW_EN_TIM3_CH3_GPIO_Port GPIOC
#define U12_EN_TIM3_CH4_Pin GPIO_PIN_9
#define U12_EN_TIM3_CH4_GPIO_Port GPIOC
#define FMILK_NF_PA8_Pin GPIO_PIN_8
#define FMILK_NF_PA8_GPIO_Port GPIOA
#define FMILK_PH_PA9_Pin GPIO_PIN_9
#define FMILK_PH_PA9_GPIO_Port GPIOA
#define SPUMP_NF_PA10_Pin GPIO_PIN_10
#define SPUMP_NF_PA10_GPIO_Port GPIOA
#define SPUMP_PH_PA11_Pin GPIO_PIN_11
#define SPUMP_PH_PA11_GPIO_Port GPIOA
#define WPUMP_NF_PA12_Pin GPIO_PIN_12
#define WPUMP_NF_PA12_GPIO_Port GPIOA
#define WPUMP_PH_PA15__Pin GPIO_PIN_15
#define WPUMP_PH_PA15__GPIO_Port GPIOA
#define CBREW_NF_PC10_Pin GPIO_PIN_10
#define CBREW_NF_PC10_GPIO_Port GPIOC
#define CBREW_PH_PC11_Pin GPIO_PIN_11
#define CBREW_PH_PC11_GPIO_Port GPIOC
#define U12_NF_PC12_Pin GPIO_PIN_12
#define U12_NF_PC12_GPIO_Port GPIOC
#define U12_PH_PD0_Pin GPIO_PIN_0
#define U12_PH_PD0_GPIO_Port GPIOD
#define LQDET_PD1_Pin GPIO_PIN_1
#define LQDET_PD1_GPIO_Port GPIOD
#define FLOW2_EXTI2_Pin GPIO_PIN_2
#define FLOW2_EXTI2_GPIO_Port GPIOD
#define LQDET_PD3_Pin GPIO_PIN_3
#define LQDET_PD3_GPIO_Port GPIOD
#define LQDET_PD4_Pin GPIO_PIN_4
#define LQDET_PD4_GPIO_Port GPIOD
#define LQDET_PD5_Pin GPIO_PIN_5
#define LQDET_PD5_GPIO_Port GPIOD
#define FLOW_EXTI7_Pin GPIO_PIN_7
#define FLOW_EXTI7_GPIO_Port GPIOD
#define HSW_PG9_Pin GPIO_PIN_9
#define HSW_PG9_GPIO_Port GPIOG
#define HSW_PG10_Pin GPIO_PIN_10
#define HSW_PG10_GPIO_Port GPIOG
#define WASTEL_EXTI11_Pin GPIO_PIN_11
#define WASTEL_EXTI11_GPIO_Port GPIOG
#define WASTESW_EXTI12_Pin GPIO_PIN_12
#define WASTESW_EXTI12_GPIO_Port GPIOG
#define AIRBREAK_EXTI13_Pin GPIO_PIN_13
#define AIRBREAK_EXTI13_GPIO_Port GPIOG
#define KTYPE_SPI3__CS2_Pin GPIO_PIN_14
#define KTYPE_SPI3__CS2_GPIO_Port GPIOG
#define PT100_SPI3__CS1_Pin GPIO_PIN_15
#define PT100_SPI3__CS1_GPIO_Port GPIOG
#define TEMP_SPI3_SCK_Pin GPIO_PIN_3
#define TEMP_SPI3_SCK_GPIO_Port GPIOB
#define TEMP_SPI3_MISO_Pin GPIO_PIN_4
#define TEMP_SPI3_MISO_GPIO_Port GPIOB
#define TEMP_SPI3_MOSI_Pin GPIO_PIN_5
#define TEMP_SPI3_MOSI_GPIO_Port GPIOB
#define LOADCELL_I2C1_SCL_Pin GPIO_PIN_6
#define LOADCELL_I2C1_SCL_GPIO_Port GPIOB
#define LOADCELL_I2C1_SDA_Pin GPIO_PIN_7
#define LOADCELL_I2C1_SDA_GPIO_Port GPIOB
#define ACWPUMP3_PE0_Pin GPIO_PIN_0
#define ACWPUMP3_PE0_GPIO_Port GPIOE
#define ACWPUMP4_PE1_Pin GPIO_PIN_1
#define ACWPUMP4_PE1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
