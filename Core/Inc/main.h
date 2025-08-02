/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCU_STATUS_LED1_Pin GPIO_PIN_2
#define MCU_STATUS_LED1_GPIO_Port GPIOE
#define MCU_STATUS_LED2_Pin GPIO_PIN_3
#define MCU_STATUS_LED2_GPIO_Port GPIOE
#define WATCHDOG_INPUT_Pin GPIO_PIN_4
#define WATCHDOG_INPUT_GPIO_Port GPIOE
#define HV_FUSE_TEMPERATURE_Pin GPIO_PIN_0
#define HV_FUSE_TEMPERATURE_GPIO_Port GPIOC
#define PRECHARGE_TEMPEREATURE_Pin GPIO_PIN_1
#define PRECHARGE_TEMPEREATURE_GPIO_Port GPIOC
#define AUX_TEMPERATURE1_Pin GPIO_PIN_3
#define AUX_TEMPERATURE1_GPIO_Port GPIOC
#define SPI1_SSN_Pin GPIO_PIN_4
#define SPI1_SSN_GPIO_Port GPIOC
#define MCU_STATUS_LED3_Pin GPIO_PIN_5
#define MCU_STATUS_LED3_GPIO_Port GPIOC
#define MCU_STATUS_LED4_Pin GPIO_PIN_0
#define MCU_STATUS_LED4_GPIO_Port GPIOB
#define AMS_ERROR_LATCHED_Pin GPIO_PIN_2
#define AMS_ERROR_LATCHED_GPIO_Port GPIOB
#define AIR_P_ENABLE_Pin GPIO_PIN_7
#define AIR_P_ENABLE_GPIO_Port GPIOE
#define AIR_N_CLOSED_Pin GPIO_PIN_8
#define AIR_N_CLOSED_GPIO_Port GPIOE
#define AIR_P_CLOSED_Pin GPIO_PIN_9
#define AIR_P_CLOSED_GPIO_Port GPIOE
#define PRECHARGE_ENABLE_Pin GPIO_PIN_10
#define PRECHARGE_ENABLE_GPIO_Port GPIOE
#define AIR_N_ENABLE_Pin GPIO_PIN_11
#define AIR_N_ENABLE_GPIO_Port GPIOE
#define SC_RESET_Pin GPIO_PIN_12
#define SC_RESET_GPIO_Port GPIOE
#define FAN_PWM_Pin GPIO_PIN_13
#define FAN_PWM_GPIO_Port GPIOE
#define IMD_ERROR_LATCHED_Pin GPIO_PIN_14
#define IMD_ERROR_LATCHED_GPIO_Port GPIOE
#define MCU_AMS_ERROR_N_Pin GPIO_PIN_8
#define MCU_AMS_ERROR_N_GPIO_Port GPIOD
#define PRECHARGE_CLOSED_SIGNAL_Pin GPIO_PIN_10
#define PRECHARGE_CLOSED_SIGNAL_GPIO_Port GPIOD
#define IMD_DATA_Pin GPIO_PIN_15
#define IMD_DATA_GPIO_Port GPIOA
#define SC_PROBE_Pin GPIO_PIN_12
#define SC_PROBE_GPIO_Port GPIOC
#define RST_OUT_Pin GPIO_PIN_4
#define RST_OUT_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
