/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define LED_FR_Pin GPIO_PIN_13
#define LED_FR_GPIO_Port GPIOC
#define IR_DATA_Pin GPIO_PIN_0
#define IR_DATA_GPIO_Port GPIOC
#define LED_BR_Pin GPIO_PIN_1
#define LED_BR_GPIO_Port GPIOC
#define PC2_Pin GPIO_PIN_2
#define PC2_GPIO_Port GPIOC
#define LED_BL_Pin GPIO_PIN_0
#define LED_BL_GPIO_Port GPIOA
#define BTN1_Pin GPIO_PIN_1
#define BTN1_GPIO_Port GPIOA
#define BTN1_EXTI_IRQn EXTI1_IRQn
#define LED_FL_Pin GPIO_PIN_4
#define LED_FL_GPIO_Port GPIOA
#define IR_LOCK_Pin GPIO_PIN_6
#define IR_LOCK_GPIO_Port GPIOA
#define TRIG_F_Pin GPIO_PIN_4
#define TRIG_F_GPIO_Port GPIOC
#define ECHO_F_Pin GPIO_PIN_5
#define ECHO_F_GPIO_Port GPIOC
#define ECHO_F_EXTI_IRQn EXTI9_5_IRQn
#define BTN2_Pin GPIO_PIN_2
#define BTN2_GPIO_Port GPIOB
#define BTN2_EXTI_IRQn EXTI2_IRQn
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define BEEP_Pin GPIO_PIN_8
#define BEEP_GPIO_Port GPIOA
#define ECHO_B_Pin GPIO_PIN_12
#define ECHO_B_GPIO_Port GPIOC
#define ECHO_B_EXTI_IRQn EXTI15_10_IRQn
#define TRIG_B_Pin GPIO_PIN_2
#define TRIG_B_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
