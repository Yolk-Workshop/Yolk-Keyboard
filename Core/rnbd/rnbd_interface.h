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

#define COMMAND_BUFFER_SIZE 32
#define RX_BUFFER_SIZE     128
#define TX_BUFFER_SIZE     128

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
    // RNBD Mode pin set
    void (*systemModeset)(RNBD_sys_modes_t);
    // RNBD RX_IND pin control
    void (*rxIndicate)(bool);
    // RNBD Reset pin control
    void (*resetModule)(bool);

    // Delay API
    void (*delayMs)(uint32_t);
    // Status Message Handler
    void (*asyncHandler)(char*);
    bool (*dataReady)(void);
}RNBD_FuncPtrs_t;


typedef struct
{
	char Demiliter;
	bool skip_delimter;
	char* async_buffer;
	uint8_t asyncBufferSize;
	char *async_pHead;
	uint8_t peek;

}RNBD_async_t;


typedef struct
{
	/* Device Identification */
	char device_name[32];         // Current device name
	char firmware_version[16];     // Firmware version string (e.g., "v1.2.3")
	char mac_address[18];

	/* Configuration Parameters */
	uint8_t tx_power;             // Transmit power level (0-15)
	uint32_t baud_rate;           // UART baud rate
	uint8_t service_bitmap;       // Enabled services bitmap
	bool flow_control;            // Hardware flow control enabled/disabled
	int8_t rssi;                 // Signal strength

	/* Error Tracking */
	uint32_t error_count;        // Number of communication errors
	uint8_t last_error;         // Last error code
}RNBD_dev_t;


typedef struct {
    /* Command buffer for storing commands before sending */
    uint8_t command_buffer[COMMAND_BUFFER_SIZE];
    uint8_t cmd_length;  // Current length of command in buffer
	uint8_t resp[COMMAND_BUFFER_SIZE];
    /* Status flags */
    volatile bool dataReady;    // Flag for received data ready
    volatile bool txBusy;       // Flag for transmission in progress
    volatile bool overflow;     // Buffer overflow indicator
} RNBD_uart_t;


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


/* Main interface structure */
typedef struct {
    bool connected;             // Connection state
    bool OTAComplete;         // OTA update state
    RNBD_Pins gpio;            // GPIO pins configuration
    RNBD_FuncPtrs_t callback;  // Function callbacks
    RNBD_dev_t device;         // Device parameters
    RNBD_uart_t uart;          // UART communication buffers
    RNBD_async_t async;        // Asynchronous communication handling
} rnbd_interface_t;

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
