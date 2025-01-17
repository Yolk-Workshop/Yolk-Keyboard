/*
 * rnbd_interface.h
 * Modified by Kuzipa Mumba in 2025
 * The original code has been modified and adapted for use on the :
 * STM32L02 Micro-controller
 * Yolk Keyboard
 *
 *  Created on: Jan 7, 2025
 *      Author: bettysidepiece
 */

#ifndef BLE_DRIVER_RNBD_INTERFACE_H_
#define BLE_DRIVER_RNBD_INTERFACE_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32l0xx_hal.h"

#define COMMAND_BUFFER_SIZE 32
#define RX_BUFFER_SIZE     128
#define TX_BUFFER_SIZE     128
#define GATT_HANDLE_SIZE 4

#define MAX_CONNECTIONS 8
#define MAC_ADDR_LEN 18   // "A434D98188CC"
#define CONN_HANDLE_LEN 4 // "0072"

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
	//Wrapper for UART RX
    void (*write)(uint8_t);
    //Wrapper for UART TX
    uint8_t (*read)(void);
    //TX buffer red to transmit data
    bool (*transmitReady)(void);
    // RNBD Mode pin set
    void (*systemModeset)(RNBD_sys_modes_t);
    // RNBD RX_IND pin control
    void (*rxIndicate)(bool);
    // RNBD Reset pin control
    void (*resetModule)(bool);
    // Delay API
    void (*delayMs)(uint32_t);

    void (*nonBlockDelayMs)(uint32_t);
    // Status Message Handler
    void (*asyncHandler)(char*);
    //Get BLE connection status
    bool (*getConnStatus)(void);
    //RX buffer ready to be read
    bool (*dataReady)(void);
}RNBD_FuncPtrs_t;


typedef struct {
	bool (*gatt_configure_hid)(void);
	bool (*gatt_notification)(const uint8_t* handle, const uint8_t* value, uint8_t len);
	bool (*gatt_indication)(const uint8_t* handle, const uint8_t* value, uint8_t len);
	bool (*gatt_write)(const uint8_t* handle, const uint8_t* value, uint8_t len);
	bool (*gatt_cccd)(const uint8_t* handle, bool notifications, bool indications);
	void (*gatt_memory_error)(void);
    bool (*rediscovery_required)(void);
} gatt_event_callbacks_t;


typedef struct
{
	char Demiliter;
	bool skip_delimter;
	char async_buffer[RX_BUFFER_SIZE];
	uint16_t asyncBufferSize;
	char *async_pHead;
	uint8_t peek;
	volatile bool msg_in_progress;
}RNBD_async_t;


typedef struct {
    bool input_report_notifications;
    bool boot_input_notifications;
} gatt_ccd_state_t;


typedef struct {
    uint8_t conn_handle[CONN_HANDLE_LEN];
    uint8_t mac_addr[MAC_ADDR_LEN];
    uint8_t address_type;
    bool active;
} connection_info_t;


typedef struct
{
	/* Device Identification */
	const uint8_t *vendorName;
	const uint8_t *productName;
	const uint8_t *hwVersion;
	const uint8_t *fwVersion;
	const uint8_t *driverVersion;
	uint16_t bleSdaHid;

	/* Configuration Parameters */
	uint8_t tx_power;             // Transmit power level (0-15)
	int8_t rssi;                // Signal strength

	uint32_t connection_timer;
}RNBD_dev_t;


typedef struct {
	uint8_t referenceReportHandle[GATT_HANDLE_SIZE];   // 100A - READ
    uint8_t inputReportHandle[GATT_HANDLE_SIZE];       // 100B - NOTIFY
    uint8_t outputReportHandle[GATT_HANDLE_SIZE];      // 100D - READ|WRITE|WRITE_NO_RESPONSE
    uint8_t protocolModeHandle[GATT_HANDLE_SIZE];      // 1002

    uint8_t bootReferenceHandle[GATT_HANDLE_SIZE];     // 100F - READ
    uint8_t bootInputHandle[GATT_HANDLE_SIZE];         // 1010 - NOTIFY
    uint8_t bootOutputHandle[GATT_HANDLE_SIZE];        // 1012 - READ|WRITE|WRITE_NO_RESPONSE
    gatt_ccd_state_t ccd_state;

    uint8_t hidInfoHandle[GATT_HANDLE_SIZE];           // 1006
    uint8_t reportMapHandle[GATT_HANDLE_SIZE];         // 1004
    uint8_t hidCtrlPointHandle[GATT_HANDLE_SIZE];      // 1008

    uint8_t ledStateBitmap;
    uint8_t mode;

    // Connection management
	connection_info_t connections[MAX_CONNECTIONS];
	uint8_t current_connection;  // Index of active connection
	uint8_t num_connections;     // Number of stored connections

	uint8_t batterylevelHandle[GATT_HANDLE_SIZE];

	//callbacks
	gatt_event_callbacks_t events;
} RNBD_gatt_state_t;


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
    RNBD_gatt_state_t gatt;
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
