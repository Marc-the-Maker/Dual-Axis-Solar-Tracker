/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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
#define MPPT_B_Pin GPIO_PIN_13
#define MPPT_B_GPIO_Port GPIOC
#define PF0_OSC_IN_Pin GPIO_PIN_0
#define PF0_OSC_IN_GPIO_Port GPIOF
#define CURR_BAT_Pin GPIO_PIN_0
#define CURR_BAT_GPIO_Port GPIOA
#define CCCV_B_Pin GPIO_PIN_1
#define CCCV_B_GPIO_Port GPIOA
#define VOLT_PANEL_Pin GPIO_PIN_2
#define VOLT_PANEL_GPIO_Port GPIOA
#define CURR_PANEL_Pin GPIO_PIN_3
#define CURR_PANEL_GPIO_Port GPIOA
#define AZI_ENC_A_Pin GPIO_PIN_4
#define AZI_ENC_A_GPIO_Port GPIOA
#define VERT_ENC_A_Pin GPIO_PIN_5
#define VERT_ENC_A_GPIO_Port GPIOA
#define MOTOR_A_Pin GPIO_PIN_6
#define MOTOR_A_GPIO_Port GPIOA
#define MOTOR_B_Pin GPIO_PIN_7
#define MOTOR_B_GPIO_Port GPIOA
#define VOLT_BAT_Pin GPIO_PIN_1
#define VOLT_BAT_GPIO_Port GPIOB
#define MOT_SWITCH_Pin GPIO_PIN_13
#define MOT_SWITCH_GPIO_Port GPIOB
#define CCCV_A_Pin GPIO_PIN_14
#define CCCV_A_GPIO_Port GPIOB
#define CCCV_BB15_Pin GPIO_PIN_15
#define CCCV_BB15_GPIO_Port GPIOB
#define MPPT_A_Pin GPIO_PIN_8
#define MPPT_A_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VERT_ENC_B_Pin GPIO_PIN_3
#define VERT_ENC_B_GPIO_Port GPIOB
#define AZI_ENC_B_Pin GPIO_PIN_4
#define AZI_ENC_B_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_6
#define LED_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_8
#define LED3_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
