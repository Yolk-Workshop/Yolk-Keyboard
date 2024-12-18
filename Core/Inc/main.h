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
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "usbd_hid.h"
#include "keys.h"
#include "usb_device.h"
#include <string.h>

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
#define C10_Pin GPIO_PIN_2
#define C10_GPIO_Port GPIOE
#define C11_Pin GPIO_PIN_3
#define C11_GPIO_Port GPIOE
#define C12_Pin GPIO_PIN_4
#define C12_GPIO_Port GPIOE
#define C13_Pin GPIO_PIN_5
#define C13_GPIO_Port GPIOE
#define TOUCH_INT_Pin GPIO_PIN_0
#define TOUCH_INT_GPIO_Port GPIOA
#define BMS_ALERT_Pin GPIO_PIN_4
#define BMS_ALERT_GPIO_Port GPIOA
#define LMAT_RESET_Pin GPIO_PIN_5
#define LMAT_RESET_GPIO_Port GPIOA
#define LMAT_INT_Pin GPIO_PIN_7
#define LMAT_INT_GPIO_Port GPIOA
#define HAPTIC_PWM_EN_Pin GPIO_PIN_9
#define HAPTIC_PWM_EN_GPIO_Port GPIOE
#define HAPTIC_EN_PWM_Pin GPIO_PIN_10
#define HAPTIC_EN_PWM_GPIO_Port GPIOE
#define BLE_STAT1_Pin GPIO_PIN_13
#define BLE_STAT1_GPIO_Port GPIOB
#define BLE_STAT2_Pin GPIO_PIN_14
#define BLE_STAT2_GPIO_Port GPIOB
#define BT_SIGNAL_GOOD_Pin GPIO_PIN_15
#define BT_SIGNAL_GOOD_GPIO_Port GPIOB
#define VBUS_DETECT_Pin GPIO_PIN_8
#define VBUS_DETECT_GPIO_Port GPIOD
#define PROX_SENSE_INT_Pin GPIO_PIN_9
#define PROX_SENSE_INT_GPIO_Port GPIOD
#define LMAT_SDB_Pin GPIO_PIN_10
#define LMAT_SDB_GPIO_Port GPIOD
#define UART_MODE_SW_Pin GPIO_PIN_6
#define UART_MODE_SW_GPIO_Port GPIOC
#define BT_RX_INPUT_Pin GPIO_PIN_7
#define BT_RX_INPUT_GPIO_Port GPIOC
#define EEPROM_WC_Pin GPIO_PIN_8
#define EEPROM_WC_GPIO_Port GPIOC
#define AMB_I2C_INT_Pin GPIO_PIN_9
#define AMB_I2C_INT_GPIO_Port GPIOC
#define BT_PAIR_SW_Pin GPIO_PIN_10
#define BT_PAIR_SW_GPIO_Port GPIOC
#define BT_RESET_Pin GPIO_PIN_11
#define BT_RESET_GPIO_Port GPIOC
#define BT_TX_ALERT_Pin GPIO_PIN_12
#define BT_TX_ALERT_GPIO_Port GPIOC
#define R0_Pin GPIO_PIN_1
#define R0_GPIO_Port GPIOD
#define R1_Pin GPIO_PIN_2
#define R1_GPIO_Port GPIOD
#define R2_Pin GPIO_PIN_3
#define R2_GPIO_Port GPIOD
#define R3_Pin GPIO_PIN_4
#define R3_GPIO_Port GPIOD
#define R4_Pin GPIO_PIN_5
#define R4_GPIO_Port GPIOD
#define R5_Pin GPIO_PIN_6
#define R5_GPIO_Port GPIOD
#define C0_Pin GPIO_PIN_7
#define C0_GPIO_Port GPIOD
#define C1_Pin GPIO_PIN_3
#define C1_GPIO_Port GPIOB
#define C2_Pin GPIO_PIN_4
#define C2_GPIO_Port GPIOB
#define C3_Pin GPIO_PIN_5
#define C3_GPIO_Port GPIOB
#define C4_Pin GPIO_PIN_6
#define C4_GPIO_Port GPIOB
#define C5_Pin GPIO_PIN_7
#define C5_GPIO_Port GPIOB
#define C6_Pin GPIO_PIN_8
#define C6_GPIO_Port GPIOB
#define C7_Pin GPIO_PIN_9
#define C7_GPIO_Port GPIOB
#define C8_Pin GPIO_PIN_0
#define C8_GPIO_Port GPIOE
#define C9_Pin GPIO_PIN_1
#define C9_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
