/*
 * rnbd.h
 *
 *  Created on: Jan 7, 2025
 *      Author: bettysidepiece
 */

#ifndef BLE_DRIVER_RNBD_H_
#define BLE_DRIVER_RNBD_H_
/**
 * RNBD Generated Driver API Header File
 *
 * @file rnbd.h
 *
 * @defgroup  rnbd RNBD
 *
 * The original code has been modified and adapted for use on the :
 * STM32L02 Micro-controller
 * Yolk Keyboard
 *
 * @brief This is the generated header file for the RNBD driver
 *
 * @version RNBD Driver Version  2.0.0
*/
/*
© [2024] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip
    software and any derivatives exclusively with Microchip products.
    You are responsible for complying with 3rd party license terms
    applicable to your use of 3rd party software (including open source
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.?
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR
    THIS SOFTWARE.
*/

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "rnbd_interface.h"

/**
 * @ingroup rnbd
 * @def RNBD_RESET_DELAY_TIME
 * This macro defines the time needed to place RNBD device in reset.
 */
/* This value depends upon the System Clock Frequency, Baudrate value and Error percentage of Baudrate*/
#define RESPONSE_TIMEOUT 		65
#define RNBD_UART_TIMEOUT       1000
#define RNBD_BUFFER_SIZE        256
#define RNBD_MAX_CMD_SIZE       64
#define RNBD_RESET_DELAY_TIME   (1)
#define RNBD_STARTUP_DELAY      (300)
#define RNBD_DELIMITER 			'%'

#define NIBBLE2ASCII(nibble) (((((nibble) & 0x0F) < 0x0A) ? ((nibble) & 0x0F) + '0' : ((nibble) & 0x0F) + 0x57))

#define RNBD_DRIVER_VERSION_LEN 	5  // length of "2.0.0"
#define VENDOR_NAME_LEN 			13         // length of "Yolk Workshop"
#define PRODUCT_NAME_LEN 			13        // length of "Yolk Keyboard"
#define HW_VERSION_LEN 				5          // length of "0.1.0"
#define SW_VERSION_LEN 				5          // length of "0.0.5"

#define BLE_STD_INTERVAL 			6U


// Define encryption properties as additional bits
typedef enum {
    PROPERTY_READ = 0x02,
    PROPERTY_WRITE_NO_ACK = 0x04,
    PROPERTY_WRITE = 0x08,
    PROPERTY_NOTIFY = 0x10,
    PROPERTY_INDICATE = 0x20,
    PROPERTY_AUTHENTICATED = 0x40,
    PROPERTY_BROADCAST = 0x80,
    PROPERTY_READ_ENCRYPT = (PROPERTY_READ | PROPERTY_AUTHENTICATED),      // 0x42
    PROPERTY_WRITE_ENCRYPT = (PROPERTY_WRITE | PROPERTY_AUTHENTICATED)    // 0x48
} gatt_property_bits_t;

/**
 * @ingroup rnbd
 * @enum RNBD_SET_IO_CAPABILITY_t
 * @brief Defines the different capability options for {nameUpperCase}
 */
typedef enum
{
    SET_IO_CAP_0,    /*!< No input no output with bonding */
    SET_IO_CAP_1,    /*!< Display YesNo */
    SET_IO_CAP_2,    /*!< No input no output */
    SET_IO_CAP_3,    /*!< Key board only */
    SET_IO_CAP_4,    /*!< Display only */
    SET_IO_CAP_5,    /*!< Keyboard display */
}RNBD_SET_IO_CAPABILITY_t;


/**
 * @ingroup rnbd
 * @enum RNBD_BAUDRATE_t
 * @brief Defines the different baudrates supported by the {nameUpperCase} driver
 */
typedef enum
{
    BR_921600,
    BR_460800,
    BR_230400,
    BR_115200,
    BR_57600,
    BR_38400,
    BR_28800,
    BR_19200,
    BR_14400,
    BR_9600,
    BR_4800,
    BR_2400,
}RNBD_BAUDRATE_t;


/**
 * @ingroup rnbd
 * @enum RNBD_FEATURES_BITMAP_t
 * @brief Defines the different feature modes supported by the {nameUpperCase} driver
 */
typedef enum
{
    RNBD_ENABLE_FLOW_CONTROL = 0x8000,
    RNBD_NO_PROMPT = 0x4000,
    RNBD_FAST_MODE = 0x2000,
    RNBD_NO_BEACON_SCAN = 0x1000,
    RNBD_NO_CONNECT_SCAN = 0x0800,
    RNBD_NO_DUPLICATE_SCAN_RESULT_FILTER = 0x0400,
    RNBD_PASSIVE_SCAN = 0x0200,
    RNBD_UART_TRANSPARENT_WITHOUT_ACK = 0x0100,
    RNBD_REBOOT_AFTER_DISCONNECTION = 0x0080,
    RNBD_RUNNING_SCRIPT_AFTER_POWER_ON = 0x0040,
    RNBD_SUPPORT_RN4020_MLDP_STREAMING_SERVICE = 0x0020,
    RNBD_DATA_LENGTH_EXTENSION = 0x0010,
    RNBD_COMMAND_MODE_GUARD = 0x0008,
}RNBD_FEATURES_BITMAP_t;


/**
 * @ingroup rnbd
 * @enum RNBD_FACTORY_RESET_MODE_t
 * @brief Defines the different factory reset options for the {nameUpperCase} driver
 */
typedef enum
{
    SF_1 = 1,   //Reset to factory default
    SF_2 = 2,   //Reset to factory default including private services and script
}RNBD_FACTORY_RESET_MODE_t;

/**
 * @ingroup rnbd
 * @union rnbd_gpio_ioBitMap_t
 * @brief Bits (pins) set as (1) are considered OUTPUTS
 */
typedef union
{
    uint8_t gpioBitMap;
    struct
    {
        unsigned p2_2 : 1;          // 01
        unsigned p2_4 : 1;          // 02
        unsigned p3_5 : 1;          // 04
        unsigned p1_2 : 1;          // 08
        unsigned p1_3 : 1;          // 10
        unsigned reserved : 3;
    };
}rnbd_gpio_ioBitMap_t;


/**
 * @ingroup rnbd
 * @union rnbd_gpio_stateBitMap_t
 * @brief  Bits (states) set as (1) are considered HIGH
 */
typedef union
{
    uint8_t gpioStateBitMap;
    struct
    {
        unsigned p2_2_state : 1;
        unsigned p2_4_state : 1;
        unsigned p3_5_state : 1;
        unsigned p1_2_state : 1;
        unsigned p1_3_state : 1;
        unsigned reserved : 3;
    };
}rnbd_gpio_stateBitMap_t;


/**
 * @ingroup rnbd
 * @union rnbd_gpio_bitmap_t
 * @brief  Bits used for I/O (|) related commands
 */
typedef union
{
    uint16_t gpioMap;
    struct
    {
        rnbd_gpio_ioBitMap_t ioBitMap;
        rnbd_gpio_stateBitMap_t ioStateBitMap;
    };
}rnbd_gpio_bitmap_t;



//RNBD350 Instance
extern rnbd_interface_t RNBD;

//RNBD350 GATT functions
bool GATT_ble_Init(void);
void GATT_List_Services(void);
bool GATT_clearProfile(void);
bool GATT_defineService(const uint8_t *uuid, uint8_t uuidLen);
bool GATT_defineCharacteristic(const uint8_t *uuid, uint8_t uuidLen, uint8_t properties, uint8_t maxSize);
bool GATT_writeCharacteristic(const uint8_t *handle, const uint8_t *data, uint8_t dataLen);
void GATT_readCharacteristic(const uint8_t* handle, uint8_t dataLen);
bool GATT_transmitCharacteristic(const uint8_t* handle, const uint8_t* data, uint8_t dataLen);
bool GATT_sendHIDReport(const uint8_t *data, uint8_t dataLen);
bool GATT_configureCCCD(const uint8_t *handle, bool notifications, bool indications);
void GATT_updateLEDState(uint8_t ledBitmap);


//RNBD350 Command Functions
bool RNBD_BondToConnectedDevice(void);
bool RNBD_ServiceChangeIndicator(void);
bool RNBD_SendData(uint8_t* data, uint16_t len);
bool RNBD_SetAppearance(uint16_t appearanceCode);
bool RNBD_ConnectToLastBondedDevice(void);
bool RNBD_SetHWVersion(const uint8_t *hardwareVersion);
bool RNBD_SetFWVersion(const uint8_t *firmwareVersion);
bool RNBD_SetModelName(const uint8_t *modelName);
bool RNBD_SetMakerName(const uint8_t *manufacturerName);
bool RNBD_SetSerialNumber(const uint8_t *serialNumber);
bool RNBD_SetDeepSleepAdvertising(uint8_t enable, uint16_t interval);
bool RNBD_SetVendorName(const uint8_t *name, uint8_t nameLen);
bool RNBD_StopAdvertising(void);
bool RNBD_EnableAdvertising(void);
bool RNBD_SetFastAdvertisementParameters(uint16_t fastAdvInterval, uint16_t fastAdvTimeout, uint16_t slowAdvInterval);
bool RNBD_SetName(const uint8_t *name, uint8_t nameLen);
bool RNBD_SetBaudRate(uint8_t baudRate);
bool RNBD_SetServiceBitmap(uint8_t serviceBitmap);
bool RNBD_SetFeaturesBitmap(uint16_t featuresBitmap);
bool RNBD_SetIOCapability(uint8_t ioCapability);
bool RNBD_SetPinCode(const uint8_t *pinCode, uint8_t pinCodeLen);
bool RNBD_SetStatusMsgDelimiter(char preDelimiter, char postDelimiter);
bool RNBD_SetOutputs(rnbd_gpio_bitmap_t bitMap);
rnbd_gpio_stateBitMap_t RNBD_GetInputsValues(rnbd_gpio_ioBitMap_t getGPIOs);
uint8_t * RNBD_GetRSSIValue(void);
bool RNBD_RebootCmd(void);
bool RNBD_FactoryReset(RNBD_FACTORY_RESET_MODE_t resetMode);
bool RNBD_Disconnect(void);
bool RNBD_AsyncMessageHandlerSet(char* pBuffer, uint8_t len);
bool RNBD_isDataReady(void);
bool RNBD_isConnected(void);
void RNBD_Reset(void);


//RNBD350 System Functions
bool RNBD_Init(void);
void RNBD_SendCmd(const uint8_t *cmd, uint8_t cmdLen);
uint8_t RNBD_GetCmd(const uint8_t *getCmd, uint8_t getCmdLen);
bool RNBD_ReadMsg(const uint8_t *expectedMsg, uint8_t msgLen);
bool RNBD_ReadDefaultResponse(void);
bool RNBD_SendCommand_ReceiveResponse(const uint8_t *cmdMsg, uint8_t cmdLen, const uint8_t *responsemsg, uint8_t responseLen);
bool RNBD_EnterCmdMode(void);
bool RNBD_EnterDataMode(void);
uint16_t RNBD_GetGRCommand(void);
uint8_t RNBD_Read(void);
void RNBD_set_StatusDelimter(char Delimter_Character);
char RNBD_get_StatusDelimter();
void RNBD_set_NoDelimter(bool value);
bool RNBD_get_NoDelimter();
void asyncMessageHandler(char* message);

#endif /* BLE_DRIVER_RNBD_H_ */
