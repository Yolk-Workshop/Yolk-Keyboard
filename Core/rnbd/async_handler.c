/*
 * async_handler.c
 *
 *  Created on: Jan 7, 2025
 *      Author: bettysidepiece
 */

#include "logger.h"
#include "rnbd.h"


typedef enum
{
    DISCONNECT_MSG  = 0,
    STREAM_OPEN_MSG = 1,
    GENERAL_MSG     = 2,
}RNBD_MESSAGE_TYPE;

#define GENERAL_PRINT_SIZE_LIMIT        (80)

/*****************************************************
*   Driver Public API
******************************************************/

bool RNBD_IsConnected(void)
{
    return RNBD.connected;
}

bool RNBD_IsOTAComplete(void)
{
    return RNBD.OTAComplete;
}


/*****************************************************
*   *Optional* Message Formatting Private API(s)
******************************************************/

static inline void RNBD_PrintMessageStart(void)
{
	LOG_INFO("<");
    LOG_INFO("<");
    LOG_INFO("<");
    LOG_INFO(" ");
}

static inline void RNBD_PrintMessageEnd(void)
{
    LOG_INFO(" ");
    LOG_INFO(">");
    LOG_INFO(">");
    LOG_INFO(">");
    LOG_INFO(" ");
    LOG_INFO(" ");
    LOG_INFO(" ");
}

static inline void RNBD_PrintIndicatorCharacters(RNBD_MESSAGE_TYPE messageType)
{
    if (DISCONNECT_MSG == messageType)
    {
        LOG_INFO("[");
    }
    else if (STREAM_OPEN_MSG == messageType)
    {
        LOG_INFO("[");
    }
    else
    {

    }
}

static inline void RNBD_PrintMessage(char* passedMessage)
{
    char printCharacter [GENERAL_PRINT_SIZE_LIMIT];
	uint8_t messageIndex;

    strcpy(printCharacter, passedMessage);
    for (messageIndex = 0; messageIndex < strlen(passedMessage); messageIndex++)
    {
        LOG_INFO("%c",printCharacter[messageIndex]);
    }
}

void asyncMessageHandler(char* message)
{

    RNBD_MESSAGE_TYPE messageType;
    RNBD_PrintMessageStart();


    if (strstr(message, "DISCONNECT"))
    {
        messageType = DISCONNECT_MSG;
        RNBD.connected = false;
        RNBD.OTAComplete = false;
    }
    else if (strstr(message, "STREAM_OPEN"))
    {
        messageType = STREAM_OPEN_MSG;
        RNBD.connected = true;
    }
    else if (strstr(message, "OTA_REQ"))
    {
        RNBD.OTAComplete = true;
        RNBD.callback.write('\r');
        RNBD.callback.write('\n');
//

        RNBD.callback.write('O');
        RNBD.callback.write('T');
        RNBD.callback.write('A');
        RNBD.callback.write('A');
        RNBD.callback.write(',');
        RNBD.callback.write('0');
        RNBD.callback.write('1');
        RNBD.callback.write('\r');
        RNBD.callback.write('\n');
    }
    else
    {
        messageType = GENERAL_MSG;
    }
    RNBD_PrintMessage(message);
    RNBD_PrintMessageEnd();
    RNBD_PrintIndicatorCharacters(messageType);
}
