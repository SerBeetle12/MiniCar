/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define PWM_SERVO_1_Pin GPIO_PIN_5
#define PWM_SERVO_1_GPIO_Port GPIOE
#define PWM_SERVO_2_Pin GPIO_PIN_6
#define PWM_SERVO_2_GPIO_Port GPIOE
#define Btn_SS_Pin GPIO_PIN_0
#define Btn_SS_GPIO_Port GPIOC
#define Led_SS_Pin GPIO_PIN_1
#define Led_SS_GPIO_Port GPIOC
#define Btn_AP_Enable_Pin GPIO_PIN_2
#define Btn_AP_Enable_GPIO_Port GPIOC
#define Btn_AP_Start_Pin GPIO_PIN_3
#define Btn_AP_Start_GPIO_Port GPIOC
#define PWM_IN_Ch2_Pin GPIO_PIN_0
#define PWM_IN_Ch2_GPIO_Port GPIOB
#define PWM_IN_Ch3_Pin GPIO_PIN_1
#define PWM_IN_Ch3_GPIO_Port GPIOB
#define DRV_SCS_Ch4_Pin GPIO_PIN_8
#define DRV_SCS_Ch4_GPIO_Port GPIOE
#define DRV_RESET_Ch4_Pin GPIO_PIN_12
#define DRV_RESET_Ch4_GPIO_Port GPIOE
#define DRV_ENABLE_Ch4_Pin GPIO_PIN_13
#define DRV_ENABLE_Ch4_GPIO_Port GPIOE
#define DRV_BRAKE_Ch4_Pin GPIO_PIN_14
#define DRV_BRAKE_Ch4_GPIO_Port GPIOE
#define DRV_DIR_Ch4_Pin GPIO_PIN_15
#define DRV_DIR_Ch4_GPIO_Port GPIOE
#define DRV_CLKIN_Ch4_Pin GPIO_PIN_10
#define DRV_CLKIN_Ch4_GPIO_Port GPIOB
#define DRV_SCS_Ch3_Pin GPIO_PIN_11
#define DRV_SCS_Ch3_GPIO_Port GPIOB
#define DRV_RESET_Ch3_Pin GPIO_PIN_8
#define DRV_RESET_Ch3_GPIO_Port GPIOH
#define DRV_ENABLE_Ch3_Pin GPIO_PIN_9
#define DRV_ENABLE_Ch3_GPIO_Port GPIOH
#define DRV_BRAKE_Ch3_Pin GPIO_PIN_10
#define DRV_BRAKE_Ch3_GPIO_Port GPIOH
#define DRV_DIR_Ch3_Pin GPIO_PIN_11
#define DRV_DIR_Ch3_GPIO_Port GPIOH
#define DRV_CLKIN_Ch3_Pin GPIO_PIN_12
#define DRV_CLKIN_Ch3_GPIO_Port GPIOH
#define DRV_SCS_Ch2_Pin GPIO_PIN_13
#define DRV_SCS_Ch2_GPIO_Port GPIOB
#define DRV_RESET_Ch2_Pin GPIO_PIN_9
#define DRV_RESET_Ch2_GPIO_Port GPIOD
#define DRV_ENABLE_Ch2_Pin GPIO_PIN_10
#define DRV_ENABLE_Ch2_GPIO_Port GPIOD
#define DRV_BRAKE_Ch2_Pin GPIO_PIN_11
#define DRV_BRAKE_Ch2_GPIO_Port GPIOD
#define DRV_DIR_Ch2_Pin GPIO_PIN_12
#define DRV_DIR_Ch2_GPIO_Port GPIOD
#define DRV_CLKIN_Ch2_Pin GPIO_PIN_13
#define DRV_CLKIN_Ch2_GPIO_Port GPIOD
#define DRV_SCS_Ch1_Pin GPIO_PIN_15
#define DRV_SCS_Ch1_GPIO_Port GPIOD
#define DRV_RESET_Ch1_Pin GPIO_PIN_9
#define DRV_RESET_Ch1_GPIO_Port GPIOJ
#define DRV_ENABLE_Ch1_Pin GPIO_PIN_10
#define DRV_ENABLE_Ch1_GPIO_Port GPIOJ
#define DRV_BRAKE_Ch1_Pin GPIO_PIN_11
#define DRV_BRAKE_Ch1_GPIO_Port GPIOJ
#define DRV_DIR_Ch1_Pin GPIO_PIN_0
#define DRV_DIR_Ch1_GPIO_Port GPIOK
#define DRV_CLKIN_Ch1_Pin GPIO_PIN_1
#define DRV_CLKIN_Ch1_GPIO_Port GPIOK
#define PWM_IN_Ch1_Pin GPIO_PIN_6
#define PWM_IN_Ch1_GPIO_Port GPIOC
#define PWM_IN_Ch4_Pin GPIO_PIN_7
#define PWM_IN_Ch4_GPIO_Port GPIOC
#define DRV_SPI_SCK_Pin GPIO_PIN_10
#define DRV_SPI_SCK_GPIO_Port GPIOC
#define DRV_SPI_MISO_Pin GPIO_PIN_11
#define DRV_SPI_MISO_GPIO_Port GPIOC
#define DRV_SPI_MOSI_Pin GPIO_PIN_12
#define DRV_SPI_MOSI_GPIO_Port GPIOC
#define Led_1_Pin GPIO_PIN_0
#define Led_1_GPIO_Port GPIOD
#define Led_2_Pin GPIO_PIN_1
#define Led_2_GPIO_Port GPIOD
#define Led_3_Pin GPIO_PIN_2
#define Led_3_GPIO_Port GPIOD
#define Led_4_Pin GPIO_PIN_3
#define Led_4_GPIO_Port GPIOD
#define SBUS2_Tx_Enable_Pin GPIO_PIN_4
#define SBUS2_Tx_Enable_GPIO_Port GPIOD
#define SBUS2_Rx_Enable_Pin GPIO_PIN_7
#define SBUS2_Rx_Enable_GPIO_Port GPIOD
#define Power_PSU_12v_Pin GPIO_PIN_14
#define Power_PSU_12v_GPIO_Port GPIOJ
#define Power_LTM_C_Pin GPIO_PIN_15
#define Power_LTM_C_GPIO_Port GPIOJ
#define Power_PSU_5v_Pin GPIO_PIN_9
#define Power_PSU_5v_GPIO_Port GPIOG
#define Power_LTM_B_Pin GPIO_PIN_4
#define Power_LTM_B_GPIO_Port GPIOK
#define Power_LTM_A_Pin GPIO_PIN_5
#define Power_LTM_A_GPIO_Port GPIOK
#define Power_DRV_4_Pin GPIO_PIN_6
#define Power_DRV_4_GPIO_Port GPIOK
#define Power_DRV_3_Pin GPIO_PIN_7
#define Power_DRV_3_GPIO_Port GPIOK
#define Power_DRV_2_Pin GPIO_PIN_15
#define Power_DRV_2_GPIO_Port GPIOG
#define Power_DRV_1_Pin GPIO_PIN_3
#define Power_DRV_1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
