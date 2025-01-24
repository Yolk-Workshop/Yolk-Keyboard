/**
 * Modified by Kuzipa Mumba in 2025
 * The original code has been modified and adapted for use on the :
 * STM32L02 Micro-controller
 * Yolk Keyboard
 *
 * RNBD Generated Driver Source File
 *
 * @file rnbd.c
 *
 * @ingroup rnbd
 *
 * @brief This is the generated driver source file for RNBD driver using RNBD.
 *
 * @version RNBD Driver Version  2.0.0
 *
 *
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

#include "rnbd.h"
#include "logger.h"


//Intialise Global RNBD Instance
rnbd_interface_t RNBD = {0};

typedef enum {
    CONN_CHECK_IDLE,
    CONN_CHECK_SEND,
    CONN_CHECK_WAIT,
    CONN_CHECK_READ
} ConnectionCheckState;

typedef struct {
    ConnectionCheckState state;
    uint32_t startTime;
    uint8_t responseBuffer[128];
    uint8_t responseRead;
    bool isConnected;
} ConnectionCheck_t;

static ConnectionCheck_t connCheck = {
    .state = CONN_CHECK_IDLE,
    .isConnected = false
};


/**
 * @ingroup rnbd
 * @brief This function filters status messages from RNBD data.
 * @param void This function takes no params.
 * @retval true - data is ready
 * @retval false -data is not ready
 */

void RNBD_Reset(void)
{
	//Enter reset
	RNBD.callback.resetModule(true);
	//Wait for Reset
	RNBD.callback.delayMs(RNBD_RESET_DELAY_TIME);
	//Exit reset
	RNBD.callback.resetModule(false);
	//Wait while RNBD is booting up
	RNBD.callback.delayMs(RNBD_STARTUP_DELAY);
}


bool RNBD_Init(void)
{
	//Enter reset
	RNBD.callback.resetModule(true);
	//Wait for Reset
	RNBD.callback.delayMs(RNBD_RESET_DELAY_TIME);
	//Exit reset
	RNBD.callback.resetModule(false);
	//Wait while RNBD is booting up
	RNBD.callback.delayMs(RNBD_STARTUP_DELAY);
	//Remove unread data sent by RNBD, if any
	while (RNBD.callback.dataReady()) {
		RNBD.callback.read();
	}
	LOG_DEBUG("RNBD350 Reboot Finished");

	RNBD.async.Demiliter = RNBD_DELIMITER;
	RNBD.callback.asyncHandler = asyncMessageHandler;
	RNBD.async.asyncBufferSize = RNBD_BUFFER_SIZE;

	return true;
}


void RNBD_SendCmd(const uint8_t *cmd, uint8_t cmdLen)
{

	for(uint8_t index=0; index < cmdLen; index++){
		//LOG_DEBUG("Data[%d]: %c",index, cmd[index]);
		RNBD.callback.write(cmd[index]);
		while(!RNBD.callback.transmitReady());
	}
}


uint8_t RNBD_GetCmd(const uint8_t *getCmd, uint8_t getCmdLen)
{
	uint8_t index = 0, ResponseTime = 0;

	RNBD_SendCmd(getCmd, getCmdLen);

	//Wait for the response time
	while (!RNBD.callback.dataReady() && ResponseTime <= RESPONSE_TIMEOUT) {
		RNBD.callback.delayMs(1);
		ResponseTime++;
	}

	do {
		//Read Ready data
		if (RNBD.callback.dataReady()) {
			RNBD.uart.resp[index++] = RNBD.callback.read();
		}
	} while (RNBD.uart.resp[index - 1U] != '>');

	return index;
}


bool RNBD_ReadMsg(const uint8_t *expectedMsg, uint8_t msgLen)
{
	unsigned int ResponseRead = 0, ResponseTime = 0, ResponseCheck = 0;

	//Wait for the response time
	while (!RNBD.callback.dataReady() || ResponseTime <= RESPONSE_TIMEOUT) {
		RNBD.callback.delayMs(1);
		ResponseTime++;
	}

	//Read Ready data
	while (RNBD.callback.dataReady()) {
		RNBD.uart.resp[ResponseRead] = RNBD.callback.read();
		ResponseRead++;
	}

	//Comparing length of response expected
	if (ResponseRead != msgLen) {
		return false;
	}

	//Comparing the Response with expected result
	for (ResponseCheck = 0; ResponseCheck < ResponseRead; ResponseCheck++) {
		if (RNBD.uart.resp[ResponseCheck] != expectedMsg[ResponseCheck]) {
			return false;
		}
	}

	return true;
}


bool RNBD_ReadDefaultResponse(void)
{
	uint8_t DefaultResponse[30];
	bool status = false;

	unsigned int ResponseWait = 0, DataReadcount = 0;
	while (!RNBD.callback.dataReady() || ResponseWait <= RESPONSE_TIMEOUT) {
		RNBD.callback.delayMs(1);
		ResponseWait++;
	}

	while (RNBD.callback.dataReady()) {
		DefaultResponse[DataReadcount] = RNBD.callback.read();
		DataReadcount++;
	}

	switch (DefaultResponse[0]) {
	case 'A': {
		if ((DefaultResponse[1] == 'O') && (DefaultResponse[2] == 'K'))
			status = true;

		break;
	}
	case 'E': {
		if ((DefaultResponse[1] == 'r') && (DefaultResponse[2] == 'r'))
			status = false;

		break;
	}
	default: {
		return status;
	}
	}

	return status;
}


bool RNBD_SendCommand_ReceiveResponse(const uint8_t *cmdMsg, uint8_t cmdLen,
		const uint8_t *responsemsg, uint8_t responseLen)
{
	int ResponseRead = 0;
	int ResponseCheck = 0;
	//Flush out any read data
	while (RNBD.callback.dataReady()) {
		RNBD.callback.read();
	}

	//Sending Command to UART
	RNBD_SendCmd(cmdMsg, cmdLen);
	//LOG_DEBUG("Command Sent");
	//Waiting for the response time
	uint32_t startTime = HAL_GetTick();
	while (!RNBD.callback.dataReady() || (HAL_GetTick() - startTime)  <= RESPONSE_TIMEOUT) {
	}

	//Read Ready data
	while (RNBD.callback.dataReady()) {
		RNBD.uart.resp[ResponseRead] = RNBD.callback.read();
		ResponseRead++;
	}
	LOG_DEBUG("%s", RNBD.uart.resp);

	//Comparing length of response expected
	if (ResponseRead != responseLen) {
		return false;
	}

	//Comparing the Response with expected result
	for (ResponseCheck = 0; ResponseCheck < ResponseRead; ResponseCheck++) {
		if (RNBD.uart.resp[ResponseCheck] != responsemsg[ResponseCheck]) {
			return false;
		}
	}

	memset(RNBD.uart.command_buffer, 0, sizeof(RNBD.uart.command_buffer));
	return true;
}


bool RNBD_EnterCmdMode(void)
{
	const uint8_t cmdModeResponse[] = { 'C', 'M', 'D', '>', ' ' };

	RNBD.uart.command_buffer[0] = '$';
	RNBD.uart.command_buffer[1] = '$';
	RNBD.uart.command_buffer[2] = '$';

	return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer, 3U,
			cmdModeResponse, 5U);
}


bool RNBD_EnterDataMode(void)
{
	const uint8_t dataModeResponse[] = { 'E', 'N', 'D', '\r', '\n' };

	RNBD.uart.command_buffer[0] = '-';
	RNBD.uart.command_buffer[1] = '-';
	RNBD.uart.command_buffer[2] = '-';
	RNBD.uart.command_buffer[3] = '\r';
	RNBD.uart.command_buffer[4] = '\n';

	return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer, 5,
			dataModeResponse, 5U);
}


bool RNBD_SendData(uint8_t* data, uint16_t len)
{
	uint32_t start = HAL_GetTick();

	__disable_irq();
	for(uint8_t index=0; index < len; index++){
		RNBD.callback.write(data[index]);
		while(!RNBD.callback.transmitReady()){;
			if((HAL_GetTick()- start) > 1){
				__enable_irq();
				LOG_DEBUG("Send Data Timeout");
				return false;
			}
		}
	}
	__enable_irq();
	LOG_DEBUG("Data Sent");
	return true;
}


uint16_t RNBD_GetGRCommand(void)
{
    const uint8_t getGRModeResponse[] = { '1', '0', '0', '0', '\r', '\n', 'C',
            'M', 'D', '>', ' ' };

    static uint8_t response[32];  // Define explicit size that matches RNBD.uart.resp
    uint16_t GR_Bitmap_Value = 0;

    RNBD.uart.command_buffer[0] = 'g';
    RNBD.uart.command_buffer[1] = 'r';
    RNBD.uart.command_buffer[2] = '\r';
    RNBD.uart.command_buffer[3] = '\n';

    RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer, 4,
            getGRModeResponse, 11);

    // Copy the response safely
    size_t copy_size = sizeof(RNBD.uart.resp);
    if (copy_size > sizeof(response)) {
        copy_size = sizeof(response);
    }
    memcpy(response, RNBD.uart.resp, copy_size);

    // Convert the response string to a hexadecimal value
    char *endptr;
    GR_Bitmap_Value = (uint16_t) strtol((char*) response, &endptr, 16);

    return GR_Bitmap_Value;
}


bool RNBD_isConnected(void) {
    static const uint8_t CMD_GK[] = "GK\r\n";
    static bool prev_status = false;  // Previous connection status
    bool current_status = false;      // Current connection status

    // Clear buffers before sending the command
    while (RNBD.callback.dataReady()) RNBD.callback.read();
    memset(connCheck.responseBuffer, 0, sizeof(connCheck.responseBuffer));
    connCheck.responseRead = 0;

    // Send "GK" command
    RNBD_SendCmd((uint8_t *)CMD_GK, sizeof(CMD_GK) - 1);

    // Wait for a response or timeout
    uint32_t startTime = HAL_GetTick();
    while ((HAL_GetTick() - startTime) < RESPONSE_TIMEOUT) {
        if (RNBD.callback.dataReady()) {
            // Read the response into the buffer
            while (RNBD.callback.dataReady() &&
                   connCheck.responseRead < sizeof(connCheck.responseBuffer)) {
                connCheck.responseBuffer[connCheck.responseRead++] = RNBD.callback.read();
            }
            break;  // Exit the loop once data is available
        }
    }

    // Process the response
    if (connCheck.responseRead == 0) {
        current_status = false;  // No response received
    } else if (connCheck.responseBuffer[0] == 'n'
    		|| connCheck.responseBuffer[3] == 'e') {
        current_status = false;  // "none" response
    } else if (connCheck.responseBuffer[0] == 'E'
    		|| connCheck.responseBuffer[2] == 'r') {
        current_status = false;  // "Err" response
    } else {
        current_status = true;   // Valid response indicates connection
    }

    // Log status change if different from the previous status
    if (current_status != prev_status) {
        LOG_DEBUG("Connection status changed: %s, Response: %s",
                  current_status ? "Connected" : "Disconnected",
                  connCheck.responseBuffer);
        prev_status = current_status;  // Update the previous status
    }

    //LOG_DEBUG("BLE status : %s", current_status?"connected":"disconnected");
    return current_status;
}



bool RNBD_SetName(const uint8_t *name, uint8_t nameLen)
{
    uint8_t index;
    const uint8_t cmdPrompt[] = { 'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>', ' ' };

    // Ensure name and nameLen are valid
    if (name == NULL || nameLen == 0 || nameLen > RNBD_BUFFER_SIZE - 5) {
        return false; // Invalid input or name too long for the buffer
    }

    // Validate name characters (optional, ensures only printable ASCII)
    for (index = 0; index < nameLen; index++) {
        if (name[index] < 32 || name[index] > 126) { // Non-printable ASCII range
            return false;
        }
    }

    // Construct the "SN,<name>\r\n" command
    RNBD.uart.command_buffer[0] = 'S';
    RNBD.uart.command_buffer[1] = 'N';
    RNBD.uart.command_buffer[2] = ',';

    for (index = 0; index < nameLen; index++) {
        RNBD.uart.command_buffer[3 + index] = name[index];
    }
    index = index + 3;

    RNBD.uart.command_buffer[index++] = '\r';
    RNBD.uart.command_buffer[index++] = '\n';
    //LOG_DEBUG("DATA SENT: %s", RNBD.uart.command_buffer);
    //LOG_DEBUG("DATA LENGTH; %d | %d",index, (nameLen+5));
    // Send the command and check the response
    return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer,
                                            index, cmdPrompt, sizeof(cmdPrompt));
}



bool RNBD_SetFastAdvertisementParameters(uint16_t fastAdvInterval, uint16_t fastAdvTimeout, uint16_t slowAdvInterval)
{
    uint8_t index = 0;
    const uint8_t expectedResponse[] = { 'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>', ' ' };

    // Construct the command in RNBD.uart.command_buffer
    RNBD.uart.command_buffer[index++] = 'S';
    RNBD.uart.command_buffer[index++] = 'T';
    RNBD.uart.command_buffer[index++] = 'A';
    RNBD.uart.command_buffer[index++] = ',';

    RNBD.uart.command_buffer[index++] = NIBBLE2ASCII((fastAdvInterval >> 12) & 0x0F);
    RNBD.uart.command_buffer[index++] = NIBBLE2ASCII((fastAdvInterval >> 8) & 0x0F);
    RNBD.uart.command_buffer[index++] = NIBBLE2ASCII((fastAdvInterval >> 4) & 0x0F);
    RNBD.uart.command_buffer[index++] = NIBBLE2ASCII(fastAdvInterval & 0x0F);

    RNBD.uart.command_buffer[index++] = ',';

    RNBD.uart.command_buffer[index++] = NIBBLE2ASCII((fastAdvTimeout >> 12) & (0x0F));
    RNBD.uart.command_buffer[index++] = NIBBLE2ASCII((fastAdvTimeout >> 8) & 0x0F);
    RNBD.uart.command_buffer[index++] = NIBBLE2ASCII((fastAdvTimeout >> 4) & 0x0F);
    RNBD.uart.command_buffer[index++] = NIBBLE2ASCII(fastAdvTimeout & 0x0F);

    RNBD.uart.command_buffer[index++] = ',';

    RNBD.uart.command_buffer[index++] = NIBBLE2ASCII((slowAdvInterval >> 12) & 0x0F);
    RNBD.uart.command_buffer[index++] = NIBBLE2ASCII((slowAdvInterval >> 8) & 0x0F);
    RNBD.uart.command_buffer[index++] = NIBBLE2ASCII((slowAdvInterval >> 4) & 0x0F);
    RNBD.uart.command_buffer[index++] = NIBBLE2ASCII(slowAdvInterval & 0x0F);

    RNBD.uart.command_buffer[index++] = '\r';
    RNBD.uart.command_buffer[index++] = '\n';

    // Send the command and wait for the expected response
    return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer, index, expectedResponse, sizeof(expectedResponse));
}


bool RNBD_EnableAdvertising(void)
{
	const uint8_t expectedResponse[] = { 'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>', ' ' };
	RNBD.uart.command_buffer[0] = 'A';
	RNBD.uart.command_buffer[1] = '\r';
	RNBD.uart.command_buffer[2] = '\n';

	return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer, 3, expectedResponse, 10);
}


bool RNBD_StopAdvertising(void)
{
	const uint8_t expectedResponse[] = { 'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>', ' ' };
	RNBD.uart.command_buffer[0] = 'Y';
	RNBD.uart.command_buffer[1] = '\r';
	RNBD.uart.command_buffer[2] = '\n';

	return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer, 3, expectedResponse, 10);
}


bool RNBD_SetAppearance(uint16_t appearanceCode)
{
    uint8_t index = 0;
    const uint8_t expectedResponse[] = { 'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>', ' ' };

    // Construct the SDA command: SDA,<hex16>\r\n
    RNBD.uart.command_buffer[index++] = 'S';
    RNBD.uart.command_buffer[index++] = 'D';
    RNBD.uart.command_buffer[index++] = 'A';
    RNBD.uart.command_buffer[index++] = ',';

    // Convert the 16-bit appearance code to ASCII and add to buffer
    RNBD.uart.command_buffer[index++] = NIBBLE2ASCII((appearanceCode >> 12) & 0x0F);
    RNBD.uart.command_buffer[index++] = NIBBLE2ASCII((appearanceCode >> 8) & 0x0F);
    RNBD.uart.command_buffer[index++] = NIBBLE2ASCII((appearanceCode >> 4) & 0x0F);
    RNBD.uart.command_buffer[index++] = NIBBLE2ASCII(appearanceCode & 0x0F);

    // Add carriage return and newline to terminate the command
    RNBD.uart.command_buffer[index++] = '\r';
    RNBD.uart.command_buffer[index++] = '\n';

    // Send the command and wait for the expected response
    return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer, index, expectedResponse, 10);
}


bool RNBD_SetVendorName(const uint8_t *name, uint8_t nameLen)
{
	uint8_t index;
	const uint8_t cmdPrompt[] = { 'A', 'O', 'K', '\r',
			'\n', 'C', 'M', 'D', '>',' ' };

	RNBD.uart.command_buffer[0] = 'S';
	RNBD.uart.command_buffer[1] = 'V';
	RNBD.uart.command_buffer[2] = 'D';
	RNBD.uart.command_buffer[3] = ',';

	for (index = 0; index < nameLen; index++) {
		RNBD.uart.command_buffer[4 + index] = name[index];
	}
	index += 4;

	// Add the carriage return and newline to terminate the command
	RNBD.uart.command_buffer[index++] = '\r';
	RNBD.uart.command_buffer[index++] = '\n';

	// Send the command and wait for the expected response
	return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer, index, cmdPrompt, 10);
}


bool RNBD_SetHWVersion(const uint8_t *hardwareVersion)
{
    uint8_t index = 0;
    const uint8_t expectedResponse[] = { 'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>', ' ' };

    // Construct the SDH command: SDH,<text>\r\n
    RNBD.uart.command_buffer[index++] = 'S';
    RNBD.uart.command_buffer[index++] = 'D';
    RNBD.uart.command_buffer[index++] = 'H';
    RNBD.uart.command_buffer[index++] = ',';

    // Copy the hardware version into the command buffer
    while (*hardwareVersion != '\0') {
        RNBD.uart.command_buffer[index++] = *hardwareVersion++;
    }

    // Add carriage return and newline to terminate the command
    RNBD.uart.command_buffer[index++] = '\r';
    RNBD.uart.command_buffer[index++] = '\n';

    // Send the command and wait for the expected response
    return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer, index, expectedResponse, 10);
}


bool RNBD_SetFWVersion(const uint8_t *firmwareVersion)
{
    uint8_t index = 0;
    const uint8_t expectedResponse[] = { 'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>', ' ' };

    // Construct the SDF command: SDF,<text>\r\n
    RNBD.uart.command_buffer[index++] = 'S';
    RNBD.uart.command_buffer[index++] = 'D';
    RNBD.uart.command_buffer[index++] = 'F';
    RNBD.uart.command_buffer[index++] = ',';

    // Copy the firmware version into the command buffer
    while (*firmwareVersion != '\0') {
        RNBD.uart.command_buffer[index++] = *firmwareVersion++;
    }

    // Add carriage return and newline to terminate the command
    RNBD.uart.command_buffer[index++] = '\r';
    RNBD.uart.command_buffer[index++] = '\n';

    // Send the command and wait for the expected response
    return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer, index, expectedResponse, 10);
}


bool RNBD_SetModelName(const uint8_t *modelName)
{
    uint8_t index = 0;
    const uint8_t expectedResponse[] = { 'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>', ' ' };

    // Construct the SDM command: SDM,<text>\r\n
    RNBD.uart.command_buffer[index++] = 'S';
    RNBD.uart.command_buffer[index++] = 'D';
    RNBD.uart.command_buffer[index++] = 'M';
    RNBD.uart.command_buffer[index++] = ',';

    // Copy the model name into the command buffer
    while (*modelName != '\0') {
        RNBD.uart.command_buffer[index++] = *modelName++;
    }

    // Add carriage return and newline to terminate the command
    RNBD.uart.command_buffer[index++] = '\r';
    RNBD.uart.command_buffer[index++] = '\n';

    // Send the command and wait for the expected response
    return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer, index, expectedResponse, 10);
}


bool RNBD_SetMakerName(const uint8_t *manufacturerName)
{
    uint8_t index = 0;
    const uint8_t expectedResponse[] = { 'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>', ' ' };

    // Construct the SDN command: SDN,<text>\r\n
    RNBD.uart.command_buffer[index++] = 'S';
    RNBD.uart.command_buffer[index++] = 'D';
    RNBD.uart.command_buffer[index++] = 'N';
    RNBD.uart.command_buffer[index++] = ',';

    // Copy the manufacturer name into the command buffer
    while (*manufacturerName != '\0') {
        RNBD.uart.command_buffer[index++] = *manufacturerName++;
    }

    // Add carriage return and newline to terminate the command
    RNBD.uart.command_buffer[index++] = '\r';
    RNBD.uart.command_buffer[index++] = '\n';

    // Send the command and wait for the expected response
    return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer, index, expectedResponse, 10);
}

bool RNBD_SetSerialNumber(const uint8_t *serialNumber)
{
    uint8_t index = 0;
    const uint8_t expectedResponse[] = { 'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>', ' ' };

    // Construct the SDS command: SDS,<text>\r\n
    RNBD.uart.command_buffer[index++] = 'S';
    RNBD.uart.command_buffer[index++] = 'D';
    RNBD.uart.command_buffer[index++] = 'S';
    RNBD.uart.command_buffer[index++] = ',';

    // Copy the serial number into the command buffer
    while (*serialNumber != '\0') {
        RNBD.uart.command_buffer[index++] = *serialNumber++;
    }

    // Add carriage return and newline to terminate the command
    RNBD.uart.command_buffer[index++] = '\r';
    RNBD.uart.command_buffer[index++] = '\n';

    // Send the command and wait for the expected response
    return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer, index, expectedResponse, sizeof(expectedResponse));
}


bool RNBD_SetDeepSleepAdvertising(uint8_t enable, uint16_t interval)
{
    uint8_t index = 0;
    const uint8_t expectedResponse[] = { 'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>', ' ' };

    // Construct the command in RNBD.uart.command_buffer
    RNBD.uart.command_buffer[index++] = 'S';
    RNBD.uart.command_buffer[index++] = 'D';
    RNBD.uart.command_buffer[index++] = 'O';
    RNBD.uart.command_buffer[index++] = ',';

    RNBD.uart.command_buffer[index++] = NIBBLE2ASCII(enable); // Convert enable to ASCII
    RNBD.uart.command_buffer[index++] = ',';

    RNBD.uart.command_buffer[index++] = NIBBLE2ASCII((interval >> 12) & 0x0F);
    RNBD.uart.command_buffer[index++] = NIBBLE2ASCII((interval >> 8) & 0x0F);
    RNBD.uart.command_buffer[index++] = NIBBLE2ASCII((interval >> 4) & 0x0F);
    RNBD.uart.command_buffer[index++] = NIBBLE2ASCII(interval & 0x0F);

    RNBD.uart.command_buffer[index++] = '\r';
    RNBD.uart.command_buffer[index++] = '\n';

    // Send the command and wait for the expected response
    return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer, index, expectedResponse, 10);
}


bool RNBD_SetBaudRate(uint8_t baudRate)
{
	uint8_t temp = (baudRate >> 4);
	const uint8_t cmdPrompt[] = { 'A', 'O', 'K', '\r', '\n',
			'C', 'M', 'D', '>',' ' };

	RNBD.uart.command_buffer[0] = 'S';
	RNBD.uart.command_buffer[1] = 'B';
	RNBD.uart.command_buffer[2] = ',';
	RNBD.uart.command_buffer[3] = NIBBLE2ASCII(temp);
	temp = (baudRate & 0x0F);
	RNBD.uart.command_buffer[4] = NIBBLE2ASCII(temp);
	RNBD.uart.command_buffer[5] = '\r';
	RNBD.uart.command_buffer[6] = '\n';

	return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer, 7U,
			cmdPrompt, 10);
}


bool RNBD_BondToConnectedDevice(void)
{
	const uint8_t connectCmd[] = { 'B', '\r', '\n' };
    const uint8_t expectedResponse[] ={ 'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>',' ' };

	// Send the command and wait for the expected response
    //LOG_DEBUG("Bonding to connected Device : %s", connectCmd);
	return RNBD_SendCommand_ReceiveResponse(connectCmd, sizeof(connectCmd), expectedResponse, sizeof(expectedResponse));
}

bool RNBD_ConnectToLastBondedDevice(void)
{
    const uint8_t connectCmd[] = { 'C', '\r', '\n' };
    const uint8_t expectedResponse[] = { 'T', 'r', 'y', 'i', 'n', 'g', '\r', '\n', '%', 'C', 'O', 'N', 'N', 'E', 'C', 'T', '%' };

    // Send the command and wait for the expected response
    return RNBD_SendCommand_ReceiveResponse(connectCmd, sizeof(connectCmd), expectedResponse, sizeof(expectedResponse));
}


bool RNBD_ServiceChangeIndicator(void)
{
    const uint8_t connectCmd[] = { 'S', 'I' , '\r', '\n' };
    const uint8_t expectedResponse[] ={ 'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>',' ' };

    // Send the command and wait for the expected response
    return RNBD_SendCommand_ReceiveResponse(connectCmd, sizeof(connectCmd), expectedResponse, sizeof(expectedResponse));
}


bool RNBD_SetServiceBitmap(uint8_t serviceBitmap)
{
	const uint8_t cmdPrompt[] = { 'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>',
			' ' };
	uint8_t temp = (serviceBitmap >> 4);

	RNBD.uart.command_buffer[0] = 'S';
	RNBD.uart.command_buffer[1] = 'S';
	RNBD.uart.command_buffer[2] = ',';
	RNBD.uart.command_buffer[3] = NIBBLE2ASCII(temp);
	temp = (serviceBitmap & 0x0F);
	RNBD.uart.command_buffer[4] = NIBBLE2ASCII(temp);
	RNBD.uart.command_buffer[5] = '\r';
	RNBD.uart.command_buffer[6] = '\n';

	return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer, 7U,
			cmdPrompt, 10);
}


bool RNBD_SetFeaturesBitmap(uint16_t featuresBitmap)
{
	const uint8_t cmdPrompt[] = { 'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>',
			' ' };
	uint8_t temp = (uint8_t) (featuresBitmap >> 12);

	RNBD.uart.command_buffer[0] = 'S';
	RNBD.uart.command_buffer[1] = 'R';
	RNBD.uart.command_buffer[2] = ',';
	temp = temp & 0x0F;
	RNBD.uart.command_buffer[3] = NIBBLE2ASCII(temp);
	temp = (uint8_t) (featuresBitmap >> 8);
	temp = temp & 0x0F;
	RNBD.uart.command_buffer[4] = NIBBLE2ASCII(temp);
	temp = (uint8_t) (featuresBitmap >> 4);
	temp = temp & 0x0F;
	RNBD.uart.command_buffer[5] = NIBBLE2ASCII(temp);
	temp = (uint8_t) featuresBitmap;
	temp = temp & 0x0F;
	RNBD.uart.command_buffer[6] = NIBBLE2ASCII(temp);
	RNBD.uart.command_buffer[7] = '\r';
	RNBD.uart.command_buffer[8] = '\n';

	return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer, 9U,
			cmdPrompt, 10);
}


bool RNBD_SetIOCapability(uint8_t ioCapability)
{
	const uint8_t cmdPrompt[] = { 'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>',
			' ' };
	RNBD.uart.command_buffer[0] = 'S';
	RNBD.uart.command_buffer[1] = 'A';
	RNBD.uart.command_buffer[2] = ',';
	RNBD.uart.command_buffer[3] = NIBBLE2ASCII(ioCapability);
	RNBD.uart.command_buffer[4] = '\r';
	RNBD.uart.command_buffer[5] = '\n';

	return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer, 6U,
			cmdPrompt, 10);
}


bool RNBD_SetPinCode(const uint8_t *pinCode, uint8_t pinCodeLen)
{
	uint8_t index;
	const uint8_t cmdPrompt[] = { 'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>',
			' ' };

	RNBD.uart.command_buffer[0] = 'S';
	RNBD.uart.command_buffer[1] = 'P';
	RNBD.uart.command_buffer[2] = ',';

	for (index = 0; index < pinCodeLen; index++) {
		RNBD.uart.command_buffer[3 + index] = pinCode[index];
	}
	index = index + 3;
	RNBD.uart.command_buffer[index++] = '\r';
	RNBD.uart.command_buffer[index++] = '\n';

	return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer, index,
			cmdPrompt, 10);
}


bool RNBD_SetStatusMsgDelimiter(char preDelimiter, char postDelimiter)
{
	const uint8_t cmdPrompt[] = { 'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>',
			' ' };

	RNBD.uart.command_buffer[0] = 'S';
	RNBD.uart.command_buffer[1] = '%';
	RNBD.uart.command_buffer[2] = ',';
	RNBD.uart.command_buffer[3] = preDelimiter;
	RNBD.uart.command_buffer[4] = ',';
	RNBD.uart.command_buffer[5] = postDelimiter;
	RNBD.uart.command_buffer[6] = '\r';
	RNBD.uart.command_buffer[7] = '\n';

	return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer, 8,
			cmdPrompt, 10);
}

bool RNBD_SetOutputs(rnbd_gpio_bitmap_t bitMap)
{
	const uint8_t cmdPrompt[] = { 'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>',
			' ' };

	char ioHighNibble = '0';
	char ioLowNibble = '0';
	char stateHighNibble = '0';
	char stateLowNibble = '0';

	// Output pins configurations
	if (bitMap.ioBitMap.p1_3) {
		ioHighNibble = '1';
	} else {
		ioHighNibble = '0';
	}
	ioLowNibble = ((0x0F & bitMap.ioBitMap.gpioBitMap) + '0');

	// High/Low Output settings
	if (bitMap.ioStateBitMap.p1_3_state) {
		stateHighNibble = '1';
	} else {
		stateHighNibble = '0';
	}
	stateLowNibble = ((0x0F & bitMap.ioStateBitMap.gpioStateBitMap) + '0');

	RNBD.uart.command_buffer[0] = '|';    // I/O
	RNBD.uart.command_buffer[1] = 'O';    // Output
	RNBD.uart.command_buffer[2] = ',';
	RNBD.uart.command_buffer[3] = ioHighNibble;       // - | - | - | P1_3
	RNBD.uart.command_buffer[4] = ioLowNibble;      // P1_2 | P3_5 | P2_4 | P2_2
	RNBD.uart.command_buffer[5] = ',';
	RNBD.uart.command_buffer[6] = stateHighNibble;    // - | - | - | P1_3
	RNBD.uart.command_buffer[7] = stateLowNibble;   // P1_2 | P3_5 | P2_4 | P2_2
	RNBD.uart.command_buffer[8] = '\r';
	RNBD.uart.command_buffer[9] = '\n';

	return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer, 10U,
			cmdPrompt, 10U);
}

rnbd_gpio_stateBitMap_t RNBD_GetInputsValues(rnbd_gpio_ioBitMap_t getGPIOs)
{
	char ioHighNibble = '0';
	char ioLowNibble = '0';
	uint8_t ioValue[] = { '0', '0' };
	rnbd_gpio_stateBitMap_t ioBitMapValue;
	ioBitMapValue.gpioStateBitMap = 0x00;

	// Output pins configurations
	if (getGPIOs.p1_3) {
		ioHighNibble = '1';
	} else {
		ioHighNibble = '0';
	}
	ioLowNibble = ((0x0F & getGPIOs.gpioBitMap) + '0');

	RNBD.uart.command_buffer[0] = '|';    // I/O
	RNBD.uart.command_buffer[1] = 'I';    // Output
	RNBD.uart.command_buffer[2] = ',';
	RNBD.uart.command_buffer[3] = ioHighNibble;       // - | - | - | P1_3
	RNBD.uart.command_buffer[4] = ioLowNibble;      // P1_2 | P3_5 | P2_4 | P2_2
	RNBD.uart.command_buffer[5] = '\r';
	RNBD.uart.command_buffer[6] = '\n';

	RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer, 7, ioValue,
			sizeof(ioValue));

	ioBitMapValue.gpioStateBitMap = ((((ioValue[0] - '0') & 0x0F) << 4)
			| ((ioValue[1] - '0') & 0x0F));
	return ioBitMapValue;
}


uint8_t* RNBD_GetRSSIValue(void)
{
	static uint8_t rssiResp[20];
	unsigned int ResponseRead = 0, ResponseTime = 0;

	RNBD.uart.command_buffer[0] = 'M';
	RNBD.uart.command_buffer[1] = '\r';
	RNBD.uart.command_buffer[2] = '\n';

	RNBD_SendCmd(RNBD.uart.command_buffer, 3);

	//Wait for the response time
	while (!RNBD.callback.dataReady() || ResponseTime <= RESPONSE_TIMEOUT) {
		RNBD.callback.delayMs(1);
		ResponseTime++;
	}

	//Read Ready data
	while (RNBD.callback.dataReady()) {
		RNBD.uart.resp[ResponseRead] = RNBD.callback.read();
		ResponseRead++;
	}

	rssiResp[0] = RNBD.uart.resp[0];
	rssiResp[1] = RNBD.uart.resp[1];
	rssiResp[2] = RNBD.uart.resp[2];

	return rssiResp;
}


bool RNBD_RebootCmd(void) {
	bool RebootStatus = false;
	const uint8_t rebootResponse[] = { 'R', 'e', 'b', 'o', 'o', 't', 'i', 'n',
			'g', '\r', '\n' };
	RNBD.uart.command_buffer[0] = 'R';
	RNBD.uart.command_buffer[1] = ',';
	RNBD.uart.command_buffer[2] = '1';
	RNBD.uart.command_buffer[4] = '\r';
	RNBD.uart.command_buffer[5] = '\n';

	RebootStatus = RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer,
			5U, rebootResponse, 11U);

	return RebootStatus;
}


bool RNBD_FactoryReset(RNBD_FACTORY_RESET_MODE_t resetMode)
{
	bool FactoryResetStatus = false;
	const uint8_t reboot[] = { 'R', 'e', 'b', 'o', 'o', 't', ' ', 'a', 'f', 't',
			'e', 'r', ' ', 'F', 'a', 'c', 't', 'o', 'r', 'y', ' ', 'R', 'e',
			's', 'e', 't', '\r', '\n' };

	RNBD.uart.command_buffer[0] = 'S';
	RNBD.uart.command_buffer[1] = 'F';
	RNBD.uart.command_buffer[2] = ',';
	RNBD.uart.command_buffer[3] = resetMode + '0';
	RNBD.uart.command_buffer[4] = '\r';
	RNBD.uart.command_buffer[5] = '\n';

	FactoryResetStatus = RNBD_SendCommand_ReceiveResponse(
			RNBD.uart.command_buffer, 6U, reboot, 28U);

	RNBD.callback.delayMs(350);

	return FactoryResetStatus;
}


bool RNBD_Disconnect(void)
{
	RNBD.uart.command_buffer[0] = 'K';
	RNBD.uart.command_buffer[1] = ',';
	RNBD.uart.command_buffer[2] = '1';
	RNBD.uart.command_buffer[3] = '\r';
	RNBD.uart.command_buffer[4] = '\n';

	RNBD_SendCmd(RNBD.uart.command_buffer, 5U);

	return RNBD_ReadDefaultResponse();
}

void RNBD_set_StatusDelimter(char Delimter_Character) {
	RNBD.async.Demiliter = Delimter_Character;
}


char RNBD_get_StatusDelimter() {
	return RNBD.async.Demiliter;
}


void RNBD_set_NoDelimter(bool value) {
	RNBD.async.skip_delimter = value;
}


bool RNBD_get_NoDelimter() {
	return RNBD.async.skip_delimter;
}

