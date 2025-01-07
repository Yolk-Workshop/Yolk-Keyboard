/*
 * rnbd_interface.h
 *
 *  Created on: Jan 7, 2025
 *      Author: bettysidepiece
 */

#ifndef RNBD_RNBD_INTERFACE_H_
#define RNBD_RNBD_INTERFACE_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32l0xx_hal.h"


/**
 * @ingroup rnbd_interface
 * @enum RNBD_SYSTEM_MODES_t
 * @brief Enum of the RNBD System Configuration Modes
 */
typedef enum
{
    TEST_MODE           = 0x00,
    APPLICATION_MODE    = 0x01
}RNBD_sys_modes_t;

typedef enum {
	RNBD_OK = 0,
	RNBD_ERROR,
	RNBD_TIMEOUT,
	RNBD_BAD_RESPONSE
}RNBD_status_t;

/**
 * @ingroup rnbd_interface
 * @struct iRNBD_FunctionPtrs_t
 * @brief Struct of RNBD Interface Function Pointer Prototypes
 */
typedef struct
{
    // RNBD UART interface control
    void (*write)(uint8_t);
    uint8_t (*read)(void);
    bool (*transmitReady)(void);
    bool (*dataReady)(void);
    // RNBD RX_IND pin control
    void (*rxIndicate)(bool);
    // RNBD Reset pin control
    void (*resetModule)(bool);
    // RNBD Mode pin set
    void (*systemModeset)(RNBD_sys_modes_t);
    // Delay API
    void (*delayMs)(uint16_t);
    // Status Message Handler
    void (*asyncHandler)(char*);
}RNBD_FuncPtrs_t;




typedef struct
{
	GPIO_TypeDef *reset_port;         // Reset pin port
	uint16_t reset_pin;               // Reset pin number
	GPIO_TypeDef *wakeup_port;         // Reset pin port
	uint16_t wakeup_pin;               // Reset pin number
	GPIO_TypeDef *status1_port;
	uint16_t status1_pin;
	GPIO_TypeDef *status2_port;
	uint16_t status2_pin;
}RNBD_Pins;


typedef struct
{
	bool connected;
	bool OTA_connected;
	RNBD_Pins gpio;
	RNBD_FuncPtrs_t callback;
}rnbd_interface_t;

/**
 * @ingroup rnbd_interface
 * @brief Checks Connected State of RNBD
 * @retval true - Connected
 * @retval false - Not Connected
 */
bool RNBD_IsConnected(void);

/**
 * @ingroup rnbd_interface
 * @brief Checks OTA Connected State of RNBD
 * @retval true - Connected
 * @retval false - Not Connected
 */
bool RNBD_IsOTAComplete(void);

#endif	/* RNBD_INTERFACE_H */
