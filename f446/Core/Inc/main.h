/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "pid.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NRF_CS_Pin GPIO_PIN_5
#define NRF_CS_GPIO_Port GPIOC
#define BL2_Pin GPIO_PIN_2
#define BL2_GPIO_Port GPIOB
#define BR2_Pin GPIO_PIN_12
#define BR2_GPIO_Port GPIOB
#define BR1_Pin GPIO_PIN_13
#define BR1_GPIO_Port GPIOB
#define FR2_Pin GPIO_PIN_14
#define FR2_GPIO_Port GPIOB
#define FR1_Pin GPIO_PIN_15
#define FR1_GPIO_Port GPIOB
#define BL1_Pin GPIO_PIN_8
#define BL1_GPIO_Port GPIOC
#define FL2_Pin GPIO_PIN_9
#define FL2_GPIO_Port GPIOC
#define CH1_FL_Pin GPIO_PIN_8
#define CH1_FL_GPIO_Port GPIOA
#define CH2_FR_Pin GPIO_PIN_9
#define CH2_FR_GPIO_Port GPIOA
#define CH3_BL_Pin GPIO_PIN_10
#define CH3_BL_GPIO_Port GPIOA
#define CH4_BR_Pin GPIO_PIN_11
#define CH4_BR_GPIO_Port GPIOA
#define FL1_Pin GPIO_PIN_12
#define FL1_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define Constrain(AMT, MIN, MAX) ((AMT) < (MIN)? (MIN):( (AMT) > (MAX)?(MAX):(AMT) ))

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
