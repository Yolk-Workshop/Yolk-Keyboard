/*
 * main.h
 *
 * Created on: Jan 7, 2025
 * Author: bettysidepiece
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "core_cm0plus.h"

#include "kb_driver.h"
#include "keys.h"
#include "keycodes.h"
#include "watchdog.h"
#include "pmsm.h"
#include "kb_ble_api.h"
#include "auto_switch.h"
#include "backlight.h"
#include "effects.h"

#include "logger.h"
#include "usb_device.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* Exported Types -----------------------------------------------------------*/
/* Timer Handlers */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim21;

/* UART/USART Handlers */
extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;

/* I2C Handlers */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

/* Keyboard State */
extern keyboard_state_t kb_state;

/* Exported Constants ------------------------------------------------------*/
/* Matrix Dimensions */
#define KEY_ROWS 6
#define KEY_COLS 14

/* External Variables -----------------------------------------------------*/
extern volatile ITStatus uart2_tc_flag;

/* Function Prototypes --------------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void IWDG_Init(void);

/*BLE UART Prototypes*/
void MX_LPUART1_UART_Init(void);
void bm7x_process_timer(void);
void tim7_ble_init(void);
void tim7_ble_stop(void);
bool uart_transmit_ready(void);
bool uart_data_ready(void);
void uart_write(uint8_t data);
uint8_t uart_read(void);
void tx_irq_handler(void);
void rx_irq_handler(void);

/* Keyboard Matrix State */
extern const uint16_t row_pins[KEY_ROWS];
extern GPIO_TypeDef* const row_ports[KEY_ROWS];
extern GPIO_TypeDef* const col_ports[KEY_COLS];
extern const uint16_t col_pins[KEY_COLS];


/* Pin Definitions -----------------------------------------------------*/
/* Column Pins */
#define C0_Pin              GPIO_PIN_7
#define C0_GPIO_Port        GPIOD
#define C1_Pin              GPIO_PIN_3
#define C1_GPIO_Port        GPIOB
#define C2_Pin              GPIO_PIN_4
#define C2_GPIO_Port        GPIOB
#define C3_Pin              GPIO_PIN_5
#define C3_GPIO_Port        GPIOB
#define C4_Pin              GPIO_PIN_6
#define C4_GPIO_Port        GPIOB
#define C5_Pin              GPIO_PIN_7
#define C5_GPIO_Port        GPIOB
#define C6_Pin              GPIO_PIN_8
#define C6_GPIO_Port        GPIOB
#define C7_Pin              GPIO_PIN_9
#define C7_GPIO_Port        GPIOB
#define C8_Pin              GPIO_PIN_0
#define C8_GPIO_Port        GPIOE
#define C9_Pin              GPIO_PIN_1
#define C9_GPIO_Port        GPIOE
#define C10_Pin             GPIO_PIN_2
#define C10_GPIO_Port       GPIOE
#define C11_Pin             GPIO_PIN_3
#define C11_GPIO_Port       GPIOE
#define C12_Pin             GPIO_PIN_4
#define C12_GPIO_Port       GPIOE
#define C13_Pin             GPIO_PIN_5
#define C13_GPIO_Port       GPIOE

/* Row Pins */
#define R0_Pin              GPIO_PIN_1
#define R0_GPIO_Port        GPIOD
#define R1_Pin              GPIO_PIN_2
#define R1_GPIO_Port        GPIOD
#define R2_Pin              GPIO_PIN_3
#define R2_GPIO_Port        GPIOD
#define R3_Pin              GPIO_PIN_4
#define R3_GPIO_Port        GPIOD
#define R4_Pin              GPIO_PIN_5
#define R4_GPIO_Port        GPIOD
#define R5_Pin              GPIO_PIN_6
#define R5_GPIO_Port        GPIOD

/* Bluetooth Related Pins */
#define BT_RESET_Pin        GPIO_PIN_11
#define BT_RESET_GPIO_Port  GPIOC
#define BT_RTS_Pin			GPIO_PIN_1
#define BT_RTS_GPIO_Port	GPIOB
#define BT_CTS_Pin			GPIO_PIN_6
#define BT_CTS_GPIO_Port	GPIOA
#define BLE_WAKEUP_Pin		GPIO_PIN_7
#define BT_WAKEUP_GPIO_Port  GPIOC
#define BLE_TXALERT_Pin		GPIO_PIN_12
#define BT_TXALERT_GPIO_Port  GPIOC
#define BLE_STAT1_Pin       GPIO_PIN_13
#define BLE_STAT1_GPIO_Port GPIOB
#define BLE_STAT2_Pin       GPIO_PIN_14
#define BLE_STAT2_GPIO_Port GPIOB
#define UART_MODE_SW_Pin    GPIO_PIN_6
#define UART_MODE_SW_GPIO_Port GPIOC

/* Battery Management Pins */
#define BATT_CHRG_EN_Pin      GPIO_PIN_10
#define BATT_CHRG_EN_GPIO_Port GPIOC
#define BATT_PG_Pin          GPIO_PIN_15
#define BATT_PG_GPIO_Port    GPIOB
#define BATT_STAT_Pin        GPIO_PIN_9
#define BATT_STAT_Port       GPIOC
#define BMS_ALERT_Pin        GPIO_PIN_4
#define BMS_ALERT_GPIO_Port  GPIOA

/* USB Related Pins */
#define VBUS_DETECT_Pin     GPIO_PIN_8
#define VBUS_DETECT_GPIO_Port GPIOD
#define USB_DM_Pin          GPIO_PIN_11
#define USB_DM_GPIO_Port    GPIOA
#define USB_DP_Pin          GPIO_PIN_12
#define USB_DP_GPIO_Port    GPIOA

/* Board Version Detection Pins */
#define HAPTIC_BRD_VERSION_Pin GPIO_PIN_5
#define HAPTIC_BRD_VERSION_Port GPIOA
#define KB_BRD_VERSION_Pin     GPIO_PIN_7
#define KB_BRD_VERSION_Port    GPIOA
#define PS_BRD_VERSION_Pin     GPIO_PIN_0
#define PS_BRD_VERSION_Port    GPIOB

/* Haptic Feedback Pins */
#define HAPTIC_PWM_EN_Pin     GPIO_PIN_9
#define HAPTIC_PWM_EN_GPIO_Port GPIOE
#define HAPTIC_EN_PWM_Pin     GPIO_PIN_10
#define HAPTIC_EN_PWM_GPIO_Port GPIOE

/* LMAT and Touch Pins */
#define LMAT_SDB_Pin        GPIO_PIN_10
#define LMAT_SDB_GPIO_Port  GPIOD
#define LMAT_RESET_Pin      GPIO_PIN_0
#define LMAT_RESET_GPIO_Port GPIOD
#define TOUCH_INT_Pin       GPIO_PIN_0
#define TOUCH_INT_GPIO_Port GPIOA

/* Proximity Sensor Pin */
#define PROX_SENSE_INT_Pin  GPIO_PIN_9
#define PROX_SENSE_INT_GPIO_Port GPIOD

/* Other Control Pins */
#define EEPROM_WC_Pin       GPIO_PIN_8
#define EEPROM_WC_GPIO_Port GPIOC

/* I2C Pins */
#define I2C1_SCL_Pin        GPIO_PIN_8
#define I2C1_SCL_GPIO_Port  GPIOB
#define I2C1_SDA_Pin        GPIO_PIN_9
#define I2C1_SDA_GPIO_Port  GPIOB
#define I2C2_SCL_Pin        GPIO_PIN_10
#define I2C2_SCL_GPIO_Port  GPIOB
#define I2C2_SDA_Pin        GPIO_PIN_11
#define I2C2_SDA_GPIO_Port  GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
