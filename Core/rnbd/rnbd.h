/*
 * rnbd.h
 *
 *  Created on: Jan 7, 2025
 *      Author: bettysidepiece
 */

#ifndef RNBD_RNBD_H_
#define RNBD_RNBD_H_
/**
 * RNBD Generated Driver API Header File
 *
 * @file rnbd.h
 *
 * @defgroup  rnbd RNBD
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
#define RNBD_RESET_DELAY_TIME         (1)

/**
 * @ingroup rnbd
 * @def RNBD_STARTUP_DELAY
 * This macro defines the RNBD boot time.
 */
#define RNBD_STARTUP_DELAY            (300)

/**
 * @ingroup rnbd
 * @def NIBBLE2ASCII
 * Converts nibble to ASCII
 */
#define NIBBLE2ASCII(nibble) (((nibble < 0x0A) ? (nibble + '0') : (nibble + 0x57)))

extern rnbd_interface_t RNBD;
/**
  Section: Data Type Definitions
*/

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
 * @enum RNBD_DEFAULT_SERVICE_BITMAP_t
 * @brief Defines the default services options for the {nameUpperCase} driver
 */
typedef enum
{
    SS_DEVICE_INFO = 0X80,
    SS_UART_TRANSPARENT = 0x40,
    SS_BEACON = 0x20,
    SS_RESERVED = 0x10,
}RNBD_DEFAULT_SERVICE_BITMAP_t;


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


 /**
  *  @ingroup rnbd
  *  @brief Initializes RNBD Device
  *  @retval true - Initialization Success
  *  @retval false - Initialization Failure
  */
bool RNBD_Init(void);

 /**
  * @ingroup rnbd
  * @brief Sends out command to RNBD.
  * @param cmd - RNBD command
  * @param cmdLen - RNBD command length
  * @return none
  */
void RNBD_SendCmd(const uint8_t *cmd, uint8_t cmdLen);

/**
 * @ingroup rnbd
 * @brief Gets config value from RNBD by sending get command
 * @param getCmd - Get command to send
 * @param getCmdLen - Get command length
 * @return index - tracked command response length.
 */
uint8_t RNBD_GetCmd(const uint8_t *getCmd, uint8_t getCmdLen);


 /**
  * @ingroup rnbd
  * @brief Reads specific message from RNBD.
  * @param expectedMsg - Expected response/status message from RNBD
  * @param msgLen - Expected response/status message length.
  * @retval true - Expected Message Received
  * @retval false - Expected Message NOT Received
  */
bool RNBD_ReadMsg(const uint8_t *expectedMsg, uint8_t msgLen);

 /**
  * @ingroup rnbd
  * @brief Reads default response from RNBD.
  * @retval true - Default Response Received
  * @retval false - Default Response NOT Received
  */
bool RNBD_ReadDefaultResponse(void);

 /**
  *  @ingroup rnbd
  *  @brief Puts the RNBD for Command mode ascii set.
  *  Paramater 1 - Command to be sent
  *  Parameter 2 - Length of Command to be sent
  *  Parameter 3 - Expected Response message
  *  Parameter 4 - Expected Response length
  *  @retval true - Command Mode Status = Success
  *  @retval false - Command Mode Status = Failure
  */
bool RNBD_SendCommand_ReceiveResponse(const uint8_t *cmdMsg, uint8_t cmdLen, const uint8_t *responsemsg, uint8_t responseLen);


 /**
  *  @ingroup rnbd
  *  @brief Puts the RNBD in command mode.
  *  @retval true - Command Mode Status = Success
  *  @retval false - Command Mode Status = Failure
  */
bool RNBD_EnterCmdMode(void);

 /**
  *  @ingroup rnbd
  *  @brief Puts the RNBD in data mode.
  *  @retval true - Data Mode Status = Success
  *  @retval false -Data Mode Status = Failure
  */
bool RNBD_EnterDataMode(void);

 /**
  * @ingroup rnbd
  * @brief Gets GR value from RNBD by sending gr command
  * @return 16-bit bitmap that indicates the features supported.
  */
uint16_t RNBD_GetGRCommand(void);


 /**
  * @ingroup rnbd
  * @brief Sets device name.
  * @param name - Device name [20 alphanumeric characters max]
  * @param nameLen - Device name length
  * @retval true - Sets device name
  * @retval false - Failure
  */
bool RNBD_SetName(const uint8_t *name, uint8_t nameLen);


 /**
  * @ingroup rnbd
  * @brief Sets Modules communication Baud Rate.
  * @param name - (2) character hex value coordinate to Command options 2.4.5
  * @retval true - Sets device name
  * @retval false - Failure
  */
bool RNBD_SetBaudRate(uint8_t baudRate);


 /**
  * @ingroup rnbd
  * @brief Sets the default supported services in RNBD.
  * @pre RNBD should be in command mode.
  * @param serviceBitmap - Supported services bitmap in RNBD
  * @retval true - Sets service bitmap
  * @retval false - Failure
  */
bool RNBD_SetServiceBitmap(uint8_t serviceBitmap);


 /**
  * @ingroup rnbd
  * @brief Sets the supported features of RNBD.
  * @0pre RNBD should be in command mode.
  * @param featuresBitmap - Supported features bitmap in RNBD
  * @retval true - Sets features bitmap
  * @retval false - Failure
  */
bool RNBD_SetFeaturesBitmap(uint16_t featuresBitmap);


 /**
  * @ingroup rnbd
  * @brief Sets the IO capability of RNBD and the system.
  * @pre RNBD should be in command mode.
  * @param ioCapability - IO capability of RNBD
  * @retval true - Sets IO capability
  * @retval false - Failure
  */
bool RNBD_SetIOCapability(uint8_t ioCapability);


 /**
  * @ingroup rnbd
  * @brief Sets the security pin code on RNBD.
  * @preconditions RNBD should be in command mode.
  * @param pinCode - Security pin code
  * @param pinCodeLen - Security pin code length (4 or 6)
  * @retval true - Sets security pin code
  * @retval false - Failure
  */
bool RNBD_SetPinCode(const uint8_t *pinCode, uint8_t pinCodeLen);


 /**
  * @ingroup rnbd
  * @brief Sets status message delimiter on RNBD.
  * @preconditions RNBD should be in command mode.
  * @param preDelimiter - Character to be use for Pre-delimiter
  * @param postDelimiter - Character to be use for Post-delimiter
  * @retval true - Delimiters are Set to new characters
  * @retval false - Failure
  */
bool RNBD_SetStatusMsgDelimiter(char preDelimiter, char postDelimiter);


 /**
  * @ingroup rnbd
  * @brief Configures RNBD GPIO pins as output, and sets state
  * @param bitMap RNBD GPIO Output I/O & Low/High State
  * @retval true - Set State Success
  * @retval false - Set State Failure
  */
bool RNBD_SetOutputs(rnbd_gpio_bitmap_t bitMap);


 /**
  * @ingroup rnbd
  * @brief Get RNBD GPIO pins input state status (high/low)
  * @param getGPIOs RNBD pins to read state status from
  * @return RNBD_gpio_stateBitMap_t - 8bit value coordinated to possible pin options
  */
rnbd_gpio_stateBitMap_t RNBD_GetInputsValues(rnbd_gpio_ioBitMap_t getGPIOs);


 /**
  * @ingroup rnbd
  * @brief Gets Latest RSSI value.
  * @retval <RSSI>
  * @retval ERR - Not Connected to RNBD
  */
uint8_t * RNBD_GetRSSIValue(void);


 /**
  * @ingroup rnbd
  * @brief Resets RNBD.
  * @retval true - Reboot Success
  * @retval false - Reboot Failure
  */
bool RNBD_RebootCmd(void);


 /**
  * @ingroup rnbd
  * @brief Restores Factory Settings of RNBD module.
  * @retval true - Reset Success
  * @retval false - Reset Failure
  */
bool RNBD_FactoryReset(RNBD_FACTORY_RESET_MODE_t resetMode);


 /**
  * @ingroup rnbd
  * @brief Disconnects the BLE link between RNBD and remote device.
  * @retval true - Disconnect Success
  * @retval false - Disconnect Failure
  */
bool RNBD_Disconnect(void);

 /**
  * @ingroup rnbd
  * @brief Sets up the Buffer and Buffer Size for Aysnc Message Handler
  * @param pBuffer - Passed buffer
  * @param len - Size of passed buffer
  * @retval true - Set Success
  * @retval false - Set Failure
  */
bool RNBD_AsyncMessageHandlerSet(char* pBuffer, uint8_t len);

 /**
  * @ingroup rnbd
  * @brief Checks to see if there is Data Ready using Async Message Handling
  * @param none
  * @retval true - Data is ready
  * @retval false - Data is not ready
  */
bool RNBD_isDataReady(void);

 /**
  * @ingroup rnbd
  * @brief Read incoming Data using Async Message Handling
  * @param none
  * @return Data Read
  */
uint8_t RNBD_Read(void);

 /**
  * \ingroup rnbd
  * \brief Sets StatusDelimter value.
  *
  * This variable sets the RNBD devices PRE/POST status message delimiter.
  *
  * \return Nothing
  */
void RNBD_set_StatusDelimter(char Delimter_Character);

/**
  * \ingroup rnbd
  * \brief Get StatusDelimter value.
  *
  * This variable gets the RNBD devices PRE/POST status message delimiter.
  *
  * \returns the current StatusDelimter value
  */
char RNBD_get_StatusDelimter();

/**
  * \ingroup rnbd
  * \brief Sets the No Delimter check during HOST OTA Update.
  *
  * This variable is used to set and not setting the Delimter check .
  *
  * \returns Nothing
  */
void RNBD_set_NoDelimter(bool value);

/**
  * \ingroup rnbd
  * \brief Returns the current status No Delimter status.
  *
  * This variable return true or false current status No Delimter status.
  *
  * \returns true or false
  */
bool RNBD_get_NoDelimter();

#endif /* RNBD_RNBD_H_ */
