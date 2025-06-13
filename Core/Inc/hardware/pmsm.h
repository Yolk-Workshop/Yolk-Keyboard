/**
* @file pmsm.h
* @brief Power Management State Machine Header
* @author Yolk Workshop
* @version 5.0
* @date 2025
*/

#ifndef PMSM_H
#define PMSM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32l0xx.h"
#include "main.h"
#include "keys.h"

/**
* @brief Power management states - simplified two-state model
*/
typedef enum {
	PM_STATE_ACTIVE = 0,
	PM_STATE_STOP = 1,
} pm_state_t;

/**
* @brief Connection mode
*/
typedef enum {
    CONNECTION_USB = 0, /*!< USB connection mode */
    CONNECTION_BLE      /*!< Bluetooth Low Energy connection mode */
} connection_mode_t;

// Constants for timeout values (in milliseconds)
#define PM_IDLE_TIMEOUT       30000  // 45 seconds before entering LPR mode
#define PM_DEEP_SLEEP_TIMEOUT   12000  // 2 minutes before considering standby
#define PM_IWDG_TIMEOUT         4000    // 4 seconds IWDG timeout

// Global variables
extern volatile pm_state_t current_pm_state;
extern volatile connection_mode_t g_connection_mode;

// Function declarations
void PM_Init(void);
void PM_SetConnectionMode(connection_mode_t mode);
void PM_RecordActivity(void);
void PM_EnterState(pm_state_t new_state);
void PM_ConfigureKeyEXTI(bool enable);
void PM_HandleWakeup(void);
void PM_Update(void);
uint8_t PM_GetBatteryLevel(void);

// Low-power clock configuration for different MSI ranges
//void SystemClock_LowPower_Config(uint8_t msi_range);

// External variables
extern volatile pm_state_t current_pm_state;
extern volatile connection_mode_t g_connection_mode;
extern volatile uint32_t last_activity_time;

// Debug support
#ifdef DEBUG_POWER
extern volatile uint8_t dbg_pm_events;
#endif

#ifdef __cplusplus
}
#endif

#endif /* PMSM_H */
