/**
 * RNBD Generated Driver Source File
 *
 * @file rnbd.c
 *
 * @ingroup rnbd
 *
 * @brief This is the generated driver source file for RNBD driver using RNBD.
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

#include "rnbd.h"

/* This value depends upon the System Clock Frequency, Baudrate value and Error percentage of Baudrate*/
#define RESPONSE_TIMEOUT 65



rnbd_interface_t RNBD;


/**
 * @def STATUS_MESSAGE_DELIMITER
 * This Variable provide a definition of the RNBD devices PRE/POST status message delimiter.
 */
char RNBD_STATUS_MESSAGE_DELIMITER = '%';

bool skip_Delimter = false;

uint8_t RNBD_cmdBuf[64];                                 /**< Command TX Buffer */

const char * const RNBD_driver_version = "2.0.0"; /**<  Current RNBD Driver Version */

static char *RNBD_asyncBuffer;                           /**< Async Message Buffer */
static uint8_t RNBD_asyncBufferSize;                     /**< Size of the Async Message Buffer */
static char *RNBD_pHead;                                 /**< Pointer to the Head of the Async Message Buffer */
static uint8_t RNBD_peek = 0;                            /**< Recieved Non-Status Message Data */
static bool RNBD_dataReady = false;                      /**< Flag which indicates whether Non-Status Message Data is ready */

uint8_t resp[100];

/**
 * @ingroup rnbd
 * @brief This function filters status messages from RNBD data.
 * @param void This function takes no params.
 * @retval true - data is ready
 * @retval false -data is not ready
 */
static bool RNBD_DataFilter(void);



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
    while (RNBD.callback.dataReady())
    {
        RNBD.callback.read();
    }

    return true;
}

void RNBD_SendCmd(const uint8_t *cmd, uint8_t cmdLen)
{
    uint8_t index = 0;

	do{
        if (RNBD.callback.transmitReady())
        {
            RNBD.callback.write(cmd[index]);
            index++;
        }
    }
    while(index < cmdLen);

    while(!RNBD.callback.transmitReady());
}

uint8_t RNBD_GetCmd(const uint8_t *getCmd, uint8_t getCmdLen)
{
    uint8_t index = 0, ResponseTime=0;

    RNBD_SendCmd(getCmd, getCmdLen);

	//Wait for the response time
    while(!RNBD.callback.dataReady() && ResponseTime<=RESPONSE_TIMEOUT)
    {
        RNBD.callback.delayMs(1);
        ResponseTime++;
    }

    do
    {
        //Read Ready data
        if(RNBD.callback.dataReady())
        {
            resp[index++]=RNBD.callback.read();
        }
    }
    while (resp[index - 1U] != '>');

    return index;
}

bool RNBD_ReadMsg(const uint8_t *expectedMsg, uint8_t msgLen)
{
    unsigned int ResponseRead=0,ResponseTime=0,ResponseCheck=0;

	//Wait for the response time
    while(!RNBD.callback.dataReady() || ResponseTime<=RESPONSE_TIMEOUT)
    {
        RNBD.callback.delayMs(1);
        ResponseTime++;
    }

	//Read Ready data
    while(RNBD.callback.dataReady())
    {
        resp[ResponseRead]=RNBD.callback.read();
        ResponseRead++;
    }

	//Comparing length of response expected
    if (ResponseRead != msgLen)
    {
        return false;
    }

	//Comparing the Response with expected result
    for(ResponseCheck=0;ResponseCheck<ResponseRead;ResponseCheck++)
    {
        if (resp[ResponseCheck] != expectedMsg[ResponseCheck])
        {
            return false;
        }
    }

    return true;
}


bool RNBD_ReadDefaultResponse(void)
{
    uint8_t DefaultResponse[30];
    bool status = false;

    unsigned int ResponseWait=0,DataReadcount=0;
    while(!RNBD.callback.dataReady() || ResponseWait<=RESPONSE_TIMEOUT)
    {
        RNBD.callback.delayMs(1);
        ResponseWait++;
    }

    while(RNBD.callback.dataReady())
    {
        DefaultResponse[DataReadcount]=RNBD.callback.read();
        DataReadcount++;
    }

    switch (DefaultResponse[0])
    {
        case 'A':
        {
            if ((DefaultResponse[1] == 'O') && (DefaultResponse[2] == 'K'))
                status = true;

            break;
        }
        case 'E':
        {
            if ((DefaultResponse[1] == 'r') && (DefaultResponse[2] == 'r'))
                status = false;

            break;
        }
        default:
        {
            return status;
        }
    }

    return status;
}


bool RNBD_SendCommand_ReceiveResponse(const uint8_t *cmdMsg, uint8_t cmdLen, const uint8_t *responsemsg, uint8_t responseLen)
{
    int ResponseRead=0,ResponseTime=0,ResponseCheck=0;

    //Flush out any read data
    while (RNBD.callback.dataReady())
    {
        RNBD.callback.read();
    }

    //Sending Command to UART
    RNBD_SendCmd(cmdMsg, cmdLen);

    //Waiting for the response time
    while(!RNBD.callback.dataReady() || ResponseTime<=RESPONSE_TIMEOUT)
    {
        RNBD.callback.delayMs(1);
        ResponseTime++;
    }

    //Read Ready data
    while(RNBD.callback.dataReady())
    {
        resp[ResponseRead]=RNBD.callback.read();
        ResponseRead++;
    }

	//Comparing length of response expected
	if (ResponseRead != responseLen)
    {
        return false;
    }

    //Comparing the Response with expected result
    for(ResponseCheck=0;ResponseCheck<ResponseRead;ResponseCheck++)
    {
        if (resp[ResponseCheck] != responsemsg[ResponseCheck])
        {
            return false;
        }
    }

    return true;
}

bool RNBD_EnterCmdMode(void)
{
    const uint8_t cmdModeResponse[] = {'C', 'M', 'D', '>', ' '};

	RNBD_cmdBuf[0] = '$';
    RNBD_cmdBuf[1] = '$';
    RNBD_cmdBuf[2] = '$';

    return RNBD_SendCommand_ReceiveResponse(RNBD_cmdBuf, 3U, cmdModeResponse, 5U);
}

bool RNBD_EnterDataMode(void)
{
    const uint8_t dataModeResponse[] = {'E', 'N', 'D', '\r', '\n'};

	RNBD_cmdBuf[0] = '-';
    RNBD_cmdBuf[1] = '-';
    RNBD_cmdBuf[2] = '-';
    RNBD_cmdBuf[3] = '\r';
    RNBD_cmdBuf[4] = '\n';

    return RNBD_SendCommand_ReceiveResponse(RNBD_cmdBuf, 5, dataModeResponse, 5U);
}

uint16_t RNBD_GetGRCommand(void)
{
	const uint8_t getGRModeResponse[] = {'1', '0', '0', '0', '\r', '\n', 'C', 'M', 'D', '>', ' '};

    static uint8_t response[20];
    uint16_t GR_Bitmap_Value=0;

    RNBD_cmdBuf[0] = 'g';
    RNBD_cmdBuf[1] = 'r';
    RNBD_cmdBuf[2] = '\r';
    RNBD_cmdBuf[3] = '\n';

    RNBD_SendCommand_ReceiveResponse(RNBD_cmdBuf,4,getGRModeResponse,11);

    response[0] =resp[0];
    response[1] =resp[1];
    response[2] =resp[2];
    response[3] =resp[3];

    //GR_Bitmap_Value = (int)strtol(response, NULL, 16);


    return GR_Bitmap_Value;
}


bool RNBD_SetName(const uint8_t *name, uint8_t nameLen)
{
    uint8_t index;
	const uint8_t cmdPrompt[] = {'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>', ' '};

    RNBD_cmdBuf[0] = 'S';
    RNBD_cmdBuf[1] = 'N';
    RNBD_cmdBuf[2] = ',';


    for (index = 0; index < nameLen; index++)
    {
        RNBD_cmdBuf[3 + index] = name[index];
    }
    index = index + 3;

    RNBD_cmdBuf[index++] = '\r';
    RNBD_cmdBuf[index++] = '\n';

    return RNBD_SendCommand_ReceiveResponse(RNBD_cmdBuf, nameLen + 5U, cmdPrompt, 10);
}

bool RNBD_SetBaudRate(uint8_t baudRate)
{
	uint8_t temp = (baudRate >> 4);
	const uint8_t cmdPrompt[] = {'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>', ' '};
    RNBD_cmdBuf[0] = 'S';
    RNBD_cmdBuf[1] = 'B';
    RNBD_cmdBuf[2] = ',';
    RNBD_cmdBuf[3] = NIBBLE2ASCII(temp);
	temp = (baudRate & 0x0F);
    RNBD_cmdBuf[4] = NIBBLE2ASCII(temp);
    RNBD_cmdBuf[5] = '\r';
    RNBD_cmdBuf[6] = '\n';

    return RNBD_SendCommand_ReceiveResponse(RNBD_cmdBuf, 7U, cmdPrompt, 10);
}


bool RNBD_SetServiceBitmap(uint8_t serviceBitmap)
{
	const uint8_t cmdPrompt[] = {'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>', ' '};
    uint8_t temp = (serviceBitmap >> 4);

    RNBD_cmdBuf[0] = 'S';
    RNBD_cmdBuf[1] = 'S';
    RNBD_cmdBuf[2] = ',';
    RNBD_cmdBuf[3] = NIBBLE2ASCII(temp);
    temp = (serviceBitmap & 0x0F);
    RNBD_cmdBuf[4] = NIBBLE2ASCII(temp);
    RNBD_cmdBuf[5] = '\r';
    RNBD_cmdBuf[6] = '\n';

    return RNBD_SendCommand_ReceiveResponse(RNBD_cmdBuf, 7U, cmdPrompt, 10);
}


bool RNBD_SetFeaturesBitmap(uint16_t featuresBitmap)
{
	const uint8_t cmdPrompt[] = {'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>', ' '};
    uint8_t temp = (uint8_t) (featuresBitmap >> 12);

    RNBD_cmdBuf[0] = 'S';
    RNBD_cmdBuf[1] = 'R';
    RNBD_cmdBuf[2] = ',';
    temp = temp & 0x0F;
    RNBD_cmdBuf[3] = NIBBLE2ASCII(temp);
    temp = (uint8_t) (featuresBitmap >> 8);
    temp = temp & 0x0F;
    RNBD_cmdBuf[4] = NIBBLE2ASCII(temp);
    temp = (uint8_t) (featuresBitmap >> 4);
    temp = temp & 0x0F;
    RNBD_cmdBuf[5] = NIBBLE2ASCII(temp);
    temp = (uint8_t) featuresBitmap;
    temp = temp & 0x0F;
    RNBD_cmdBuf[6] = NIBBLE2ASCII(temp);
    RNBD_cmdBuf[7] = '\r';
    RNBD_cmdBuf[8] = '\n';

    return RNBD_SendCommand_ReceiveResponse(RNBD_cmdBuf, 9U, cmdPrompt, 10);
}


bool RNBD_SetIOCapability(uint8_t ioCapability)
{
	const uint8_t cmdPrompt[] = {'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>', ' '};
    RNBD_cmdBuf[0] = 'S';
    RNBD_cmdBuf[1] = 'A';
    RNBD_cmdBuf[2] = ',';
    RNBD_cmdBuf[3] = NIBBLE2ASCII(ioCapability);
    RNBD_cmdBuf[4] = '\r';
    RNBD_cmdBuf[5] = '\n';

    return RNBD_SendCommand_ReceiveResponse(RNBD_cmdBuf, 6U, cmdPrompt, 10);
}


bool RNBD_SetPinCode(const uint8_t *pinCode, uint8_t pinCodeLen)
{
    uint8_t index;
	const uint8_t cmdPrompt[] = {'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>', ' '};

    RNBD_cmdBuf[0] = 'S';
    RNBD_cmdBuf[1] = 'P';
    RNBD_cmdBuf[2] = ',';

    for (index = 0; index < pinCodeLen; index++)
    {
        RNBD_cmdBuf[3 + index] = pinCode[index];
    }
    index = index + 3;
    RNBD_cmdBuf[index++] = '\r';
    RNBD_cmdBuf[index++] = '\n';

    return RNBD_SendCommand_ReceiveResponse(RNBD_cmdBuf, index, cmdPrompt, 10);
}

bool RNBD_SetStatusMsgDelimiter(char preDelimiter, char postDelimiter)
{
	const uint8_t cmdPrompt[] = {'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>', ' '};

    RNBD_cmdBuf[0] = 'S';
    RNBD_cmdBuf[1] = '%';
    RNBD_cmdBuf[2] = ',';
    RNBD_cmdBuf[3] = preDelimiter;
    RNBD_cmdBuf[4] = ',';
    RNBD_cmdBuf[5] = postDelimiter;
    RNBD_cmdBuf[6] = '\r';
    RNBD_cmdBuf[7] = '\n';

    return RNBD_SendCommand_ReceiveResponse(RNBD_cmdBuf, 8, cmdPrompt, 10);
}


bool RNBD_SetOutputs(rnbd_gpio_bitmap_t bitMap)
{
	const uint8_t cmdPrompt[] = {'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>', ' '};

    char ioHighNibble = '0';
    char ioLowNibble = '0';
    char stateHighNibble = '0';
    char stateLowNibble = '0';

    // Output pins configurations
    if (bitMap.ioBitMap.p1_3)
    {
        ioHighNibble = '1';
    }
    else
    {
        ioHighNibble = '0';
    }
    ioLowNibble = ( (0x0F & bitMap.ioBitMap.gpioBitMap) + '0');

    // High/Low Output settings
    if (bitMap.ioStateBitMap.p1_3_state)
    {
        stateHighNibble = '1';
    }
    else
    {
        stateHighNibble = '0';
    }
    stateLowNibble = ( (0x0F & bitMap.ioStateBitMap.gpioStateBitMap) + '0');

    RNBD_cmdBuf[0] = '|';    // I/O
    RNBD_cmdBuf[1] = 'O';    // Output
    RNBD_cmdBuf[2] = ',';
    RNBD_cmdBuf[3] = ioHighNibble;       // - | - | - | P1_3
    RNBD_cmdBuf[4] = ioLowNibble;        // P1_2 | P3_5 | P2_4 | P2_2
    RNBD_cmdBuf[5] = ',';
    RNBD_cmdBuf[6] = stateHighNibble;    // - | - | - | P1_3
    RNBD_cmdBuf[7] = stateLowNibble;     // P1_2 | P3_5 | P2_4 | P2_2
    RNBD_cmdBuf[8] = '\r';
    RNBD_cmdBuf[9] = '\n';

    return RNBD_SendCommand_ReceiveResponse(RNBD_cmdBuf, 10U, cmdPrompt, 10U);
}


rnbd_gpio_stateBitMap_t RNBD_GetInputsValues(rnbd_gpio_ioBitMap_t getGPIOs)
{
    char ioHighNibble = '0';
    char ioLowNibble = '0';
    uint8_t ioValue[] = {'0', '0'};
    rnbd_gpio_stateBitMap_t ioBitMapValue;
    ioBitMapValue.gpioStateBitMap = 0x00;

    // Output pins configurations
    if (getGPIOs.p1_3)
    {
        ioHighNibble = '1';
    }
    else
    {
        ioHighNibble = '0';
    }
    ioLowNibble = ( (0x0F & getGPIOs.gpioBitMap) + '0');

    RNBD_cmdBuf[0] = '|';    // I/O
    RNBD_cmdBuf[1] = 'I';    // Output
    RNBD_cmdBuf[2] = ',';
    RNBD_cmdBuf[3] = ioHighNibble;       // - | - | - | P1_3
    RNBD_cmdBuf[4] = ioLowNibble;        // P1_2 | P3_5 | P2_4 | P2_2
    RNBD_cmdBuf[5] = '\r';
    RNBD_cmdBuf[6] = '\n';

    RNBD_SendCommand_ReceiveResponse(RNBD_cmdBuf, 7, ioValue, sizeof (ioValue));

    ioBitMapValue.gpioStateBitMap = ( (((ioValue[0] - '0') & 0x0F) << 4) | ((ioValue[1] - '0') & 0x0F) );
    return ioBitMapValue;
}


uint8_t * RNBD_GetRSSIValue(void)
{
    static uint8_t rssiResp[20];
    unsigned int ResponseRead=0,ResponseTime=0;

    RNBD_cmdBuf[0] = 'M';
    RNBD_cmdBuf[1] = '\r';
    RNBD_cmdBuf[2] = '\n';

	RNBD_SendCmd(RNBD_cmdBuf, 3);

	//Wait for the response time
    while(!RNBD.callback.dataReady() || ResponseTime<=RESPONSE_TIMEOUT)
    {
        RNBD.callback.delayMs(1);
        ResponseTime++;
    }

	//Read Ready data
    while(RNBD.callback.dataReady())
    {
        resp[ResponseRead]=RNBD.callback.read();
        ResponseRead++;
    }

	rssiResp[0]=resp[0];
	rssiResp[1]=resp[1];
	rssiResp[2]=resp[2];

	return rssiResp;
}


bool RNBD_RebootCmd(void)
{
    bool RebootStatus = false;
    const uint8_t rebootResponse[] = {'R', 'e', 'b', 'o', 'o', 't', 'i', 'n', 'g', '\r', '\n'};
	RNBD_cmdBuf[0] = 'R';
    RNBD_cmdBuf[1] = ',';
    RNBD_cmdBuf[2] = '1';
    RNBD_cmdBuf[4] = '\r';
    RNBD_cmdBuf[5] = '\n';

    RebootStatus = RNBD_SendCommand_ReceiveResponse(RNBD_cmdBuf, 5U, rebootResponse, 11U);

	RNBD.callback.delayMs(350);

	return RebootStatus;
}


bool RNBD_FactoryReset(RNBD_FACTORY_RESET_MODE_t resetMode)
{
    bool FactoryResetStatus = false;
    const uint8_t reboot[] = {'R', 'e', 'b', 'o', 'o', 't', ' ', 'a', 'f', 't', 'e', 'r', ' ', 'F', 'a', 'c', 't', 'o', 'r', 'y', ' ', 'R', 'e', 's', 'e', 't', '\r', '\n'};
    RNBD_cmdBuf[0] = 'S';
    RNBD_cmdBuf[1] = 'F';
    RNBD_cmdBuf[2] = ',';
    RNBD_cmdBuf[4] = (char)resetMode;
    RNBD_cmdBuf[5] = '\r';
    RNBD_cmdBuf[6] = '\n';

    FactoryResetStatus = RNBD_SendCommand_ReceiveResponse(RNBD_cmdBuf, 6U, reboot, 28U);

    RNBD.callback.delayMs(350);

	return FactoryResetStatus;
}


bool RNBD_Disconnect(void)
{
    RNBD_cmdBuf[0] = 'K';
    RNBD_cmdBuf[1] = ',';
    RNBD_cmdBuf[2] = '1';
    RNBD_cmdBuf[3] = '\r';
    RNBD_cmdBuf[4] = '\n';

    RNBD_SendCmd(RNBD_cmdBuf, 5U);

    return RNBD_ReadDefaultResponse();
}

void RNBD_set_StatusDelimter(char Delimter_Character)
{
	RNBD_STATUS_MESSAGE_DELIMITER = Delimter_Character;
}

char RNBD_get_StatusDelimter()
{
	return RNBD_STATUS_MESSAGE_DELIMITER;
}

void RNBD_set_NoDelimter(bool value)
{
    skip_Delimter=value;
}

bool RNBD_get_NoDelimter()
{
    return skip_Delimter;
}


bool RNBD_AsyncMessageHandlerSet(char* pBuffer, uint8_t len)
{
    if ((pBuffer != NULL) && (len > 1))
    {
      RNBD_asyncBuffer = pBuffer;
      RNBD_asyncBufferSize = len - 1;
      return true;
    }
    else
    {
        return false;
    }
}

bool RNBD_isDataReady(void)
{
    if (RNBD_dataReady)
    {
        return true;
    }

    if (RNBD.callback.dataReady())
    {
        return RNBD_DataFilter();
    }
    return false;
}

uint8_t RNBD_Read(void)
{
    while(RNBD_isDataReady() == false); // Wait
    RNBD_dataReady = false;
    return RNBD_peek;
}

static bool RNBD_DataFilter(void)
{
    static bool asyncBuffering = false;

    uint8_t readChar = RNBD.callback.read();

    if(asyncBuffering == true)
    {
        if(readChar == RNBD_STATUS_MESSAGE_DELIMITER)
        {
            asyncBuffering = false;
            *RNBD_pHead = '\0';
            RNBD.callback.asyncHandler(RNBD_asyncBuffer);
        }
        else if (RNBD_pHead < RNBD_asyncBuffer + RNBD_asyncBufferSize)
        {
            *RNBD_pHead++ = readChar;
        }
		else
        {
            //do nothing
        }
    }
    else
    {
        if (readChar == RNBD_STATUS_MESSAGE_DELIMITER && (skip_Delimter == false))
        {
            asyncBuffering = true;
            RNBD_pHead = RNBD_asyncBuffer;
        }
        else
        {
			skip_Delimter = true;
            RNBD_dataReady = true;
            RNBD_peek = readChar;
        }
    }
    return RNBD_dataReady;
}
