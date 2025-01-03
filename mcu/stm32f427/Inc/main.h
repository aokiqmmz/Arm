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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DRDY_IST8310_Pin GPIO_PIN_3
#define DRDY_IST8310_GPIO_Port GPIOE
#define RSTN_IST8310_Pin GPIO_PIN_2
#define RSTN_IST8310_GPIO_Port GPIOE
#define IMU_HEAT_Pin GPIO_PIN_5
#define IMU_HEAT_GPIO_Port GPIOB
#define LASER_Pin GPIO_PIN_13
#define LASER_GPIO_Port GPIOG
#define DC24V_2_Pin GPIO_PIN_2
#define DC24V_2_GPIO_Port GPIOH
#define DC24V_3_Pin GPIO_PIN_3
#define DC24V_3_GPIO_Port GPIOH
#define DC24V_4_Pin GPIO_PIN_4
#define DC24V_4_GPIO_Port GPIOH
#define LED7_Pin GPIO_PIN_8
#define LED7_GPIO_Port GPIOG
#define DC24V_5_Pin GPIO_PIN_5
#define DC24V_5_GPIO_Port GPIOH
#define LED6_Pin GPIO_PIN_7
#define LED6_GPIO_Port GPIOG
#define LED5_Pin GPIO_PIN_6
#define LED5_GPIO_Port GPIOG
#define NSS_Pin GPIO_PIN_6
#define NSS_GPIO_Port GPIOF
#define LED4_Pin GPIO_PIN_5
#define LED4_GPIO_Port GPIOG
#define LED3_Pin GPIO_PIN_4
#define LED3_GPIO_Port GPIOG
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOG
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOG
#define KEY_Pin GPIO_PIN_2
#define KEY_GPIO_Port GPIOB
#define LED0_Pin GPIO_PIN_1
#define LED0_GPIO_Port GPIOG
#define BUZZER_Pin GPIO_PIN_6
#define BUZZER_GPIO_Port GPIOH
#define LED_RED_Pin GPIO_PIN_11
#define LED_RED_GPIO_Port GPIOE
#define LED_GREEN_Pin GPIO_PIN_14
#define LED_GREEN_GPIO_Port GPIOF

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
