/**
 * @file rnbd_stm32.h
 * @brief STM32 HAL Driver for RNBD Bluetooth Module using Interrupts
 */

#ifndef RNBD350_H
#define RNBD350_H

#include "stm32l0xx_hal.h"
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <logger.h>

// Configuration constants
#define RNBD_UART_TIMEOUT          1000
#define RNBD_BUFFER_SIZE           256
#define RNBD_MAX_CMD_SIZE          64
#define RNBD_RESET_DELAY_MS        1
#define RNBD_STARTUP_DELAY_MS      300

// Status codes
typedef enum {
    RNBD_OK = 0,
    RNBD_ERROR,
    RNBD_TIMEOUT,
    RNBD_BUSY
} rnbd_status_t;

// Module states
typedef enum {
    RNBD_STATE_RESET,
    RNBD_STATE_READY,
    RNBD_STATE_CMD_MODE,
    RNBD_STATE_DATA_MODE,
    RNBD_STATE_ERROR
} rnbd_state_t;

// UART transfer status
typedef volatile enum {
    UART_IDLE = 0,
    UART_BUSY,
    UART_COMPLETE,
    UART_ERROR
} uart_status_t;

typedef struct {
	GPIO_TypeDef *reset_port;         // Reset pin port
	uint16_t reset_pin;               // Reset pin number
	GPIO_TypeDef *wakeup_port;         // Reset pin port
	uint16_t wakeup_pin;               // Reset pin number
} rnbd_phy_hw_t;

// RNBD context structure
typedef struct {
    UART_HandleTypeDef *huart;        // UART handle
    rnbd_phy_hw_t gpio;
    uint8_t rx_buffer[RNBD_BUFFER_SIZE];  // Receive buffer
    uint8_t tx_buffer[RNBD_BUFFER_SIZE];  // Transmit buffer
    uint16_t rx_index;                // Current receive index
    uint16_t tx_index;                // Current transmit index
    uart_status_t tx_status;          // Transmission status
    uart_status_t rx_status;          // Reception status
    rnbd_state_t state;              // Current module state
    void (*delay_ms)(uint32_t);
    void (*status_callback)(char*);   // Status message callback
} rnbd_handle_t;

// Driver initialization and configuration

//rnbd_status_t RNBD_Init(rnbd_handle_t *handle);
void RNBD_Reset(rnbd_handle_t *handle);
void RNBD_wakeup(rnbd_handle_t *handle);

// Command mode functions
//rnbd_status_t RNBD_EnterCmdMode(rnbd_handle_t *handle);
rnbd_status_t RNBD_ExitCmdMode(rnbd_handle_t *handle);
rnbd_status_t RNBD_SendCommand(rnbd_handle_t *handle, const char *cmd, char *response);

// Configuration functions
rnbd_status_t RNBD_SetDeviceName(rnbd_handle_t *handle, const char *name);
rnbd_status_t RNBD_SetServices(rnbd_handle_t *handle, uint8_t services);
rnbd_status_t RNBD_StartAdvertising(rnbd_handle_t *handle);
rnbd_status_t RNBD_StopAdvertising(rnbd_handle_t *handle);

// Data transmission functions
rnbd_status_t RNBD_SendData(rnbd_handle_t *handle, const uint8_t *data, uint16_t len);
void RNBD_RegisterStatusCallback(rnbd_handle_t *handle, void (*callback)(char*));

// Interrupt handlers
void RNBD_UART_TxCpltCallback(rnbd_handle_t *handle);
void RNBD_UART_RxCpltCallback(rnbd_handle_t *handle);

#endif // RNBD350_H
