/*
 * rnbd_gatt_api.c
 *
 *  Created on: Jan 9, 2025
 *      Author: bettysidepiece
 */
#include "logger.h"
#include "ble_hid_keyboard.h"
#include "rnbd.h"


static const uint8_t hidInfo[] = {0x11, 0x01, 0x00, 0x02}; // HID Version 1.11, No Country Code, Boot Protocol Supported


static const uint8_t keyboardReportDescriptor[] = {
    0x05, 0x01, // Usage Page (Generic Desktop Ctrls)
    0x09, 0x06, // Usage (Keyboard)
    0xA1, 0x01, // Collection (Application)

    // Modifier Keys
    0x05, 0x07, // Usage Page (Keyboard/Keypad)
    0x19, 0xE0, // Usage Minimum (224)
    0x29, 0xE7, // Usage Maximum (231)
    0x15, 0x00, // Logical Minimum (0)
    0x25, 0x01, // Logical Maximum (1)
    0x75, 0x01, // Report Size (1)
    0x95, 0x08, // Report Count (8)
    0x81, 0x02, // Input (Data, Variable, Absolute)

    // Reserved Byte
    0x75, 0x08, // Report Size (8)
    0x95, 0x01, // Report Count (1)
    0x81, 0x01, // Input (Constant)

    // Boot Protocol Keys (6KRO)
    0x05, 0x07, // Usage Page (Keyboard/Keypad)
    0x19, 0x00, // Usage Minimum (0)
    0x29, 0x65, // Usage Maximum (101)
    0x15, 0x00, // Logical Minimum (0)
    0x25, 0x65, // Logical Maximum (101)
    0x75, 0x08, // Report Size (8)
    0x95, 0x06, // Report Count (6)
    0x81, 0x00, // Input (Data, Array)

    // NKRO Extended Keys (allows up to 64 additional keys)
    0x75, 0x01, // Report Size (1)
    0x95, 0x50, // Report Count (80)
    0x05, 0x07, // Usage Page (Keyboard/Keypad)
    0x19, 0x00, // Usage Minimum (0)
    0x29, 0x4F, // Usage Maximum (79)
    0x15, 0x00, // Logical Minimum (0)
    0x25, 0x01, // Logical Maximum (1)
    0x81, 0x02, // Input (Data, Variable, Absolute)

    0xC0        // End Collection
};

// Expected response for successful commands
static const uint8_t expectedResponse[] = { 'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>', ' ' };
static bool GATT_enableProtocolMode(void);

static void set_Handle(uint16_t value, uint8_t* handle)
{
    const char hex_chars[] = "0123456789ABCDEF";
    handle[0] = hex_chars[(value >> 12) & 0xF];
    handle[1] = hex_chars[(value >> 8) & 0xF];
    handle[2] = hex_chars[(value >> 4) & 0xF];
    handle[3] = hex_chars[value & 0xF];
}


static bool GATT_HID_Init(void)
{
    bool result = true;
    // Initialise current handle
    uint16_t current_handle = 0x1000;

    // Define HID Service (UUID 0x1812)
    result &= GATT_defineService((uint8_t*)HID_SERVICE_UUID, 4);

    // Protocol Mode characteristic (UUID 0x2A4E)
    result &= GATT_defineCharacteristic((uint8_t*)HID_PROTOCOL_MODE_UUID, 4,
                                        PROPERTY_READ | PROPERTY_WRITE_NO_ACK,
                                        1);  // 1 byte size
    current_handle += 2;
    set_Handle(current_handle, RNBD.gatt.protocolModeHandle);

    // Report Map characteristic (UUID 0x2A4B)
    result &= GATT_defineCharacteristic((uint8_t*)HID_REPORT_MAP_UUID, 4,
                                        PROPERTY_READ,
                                        sizeof(keyboardReportDescriptor));

    current_handle += 2;
    set_Handle(current_handle, RNBD.gatt.reportMapHandle);

    // HID Information characteristic (UUID 0x2A4A)
    result &= GATT_defineCharacteristic((uint8_t*)HID_INFO_UUID, 4,
                                        PROPERTY_READ,
                                        sizeof(hidInfo));

    current_handle += 2;
    set_Handle(current_handle, RNBD.gatt.hidInfoHandle);

    // HID Control Point characteristic (UUID 0x2A4C)
    result &= GATT_defineCharacteristic((uint8_t*)HID_CONTROL_POINT_UUID, 4,
                                        PROPERTY_WRITE_NO_ACK,
                                        1);  // 1 byte size

    current_handle += 2;
    set_Handle(current_handle, RNBD.gatt.hidCtrlPointHandle);

    // Input Report characteristic (UUID 0x2A4D)
    result &= GATT_defineCharacteristic((uint8_t*)HID_REPORT_UUID, 4,
                                        PROPERTY_READ | PROPERTY_NOTIFY,
                                        sizeof(hid_report_t));

    current_handle += 2;
    set_Handle(current_handle, RNBD.gatt.referenceReportHandle);
    current_handle += 1;
        set_Handle(current_handle, RNBD.gatt.inputReportHandle);

    // Output Report characteristic (UUID 0x2A4D)
    result &= GATT_defineCharacteristic((uint8_t*)HID_REPORT_UUID, 4,
                                        PROPERTY_READ | PROPERTY_WRITE | PROPERTY_WRITE_NO_ACK,
                                        1);  // 1 byte for LED states
    current_handle += 2;
	set_Handle(current_handle, RNBD.gatt.outputReportHandle);

    // Boot Keyboard Input Report (UUID 0x2A22)
    result &= GATT_defineCharacteristic((uint8_t*)HID_BOOT_KEYBOARD_IN_UUID, 4,
                                        PROPERTY_READ | PROPERTY_NOTIFY,
                                        8);  // 8 bytes - modifiers(1) + reserved(1) + keys(6)
    current_handle += 2;
    set_Handle(current_handle, RNBD.gatt.bootReferenceHandle);
    current_handle += 1;
	set_Handle(current_handle, RNBD.gatt.bootInputHandle);

    // Boot Keyboard Output Report (UUID 0x2A32)
    result &= GATT_defineCharacteristic((uint8_t*)HID_BOOT_KEYBOARD_OUT_UUID, 4,
                                        PROPERTY_READ | PROPERTY_WRITE | PROPERTY_WRITE_NO_ACK,
                                        1);  // 1 byte for LED states

    current_handle += 2;
    set_Handle(current_handle, RNBD.gatt.bootOutputHandle);

    // Set Protocol Mode Callback
    RNBD.gatt.setProtocolMode = GATT_enableProtocolMode;

    // Update last handle
    RNBD.gatt.lastHandle = current_handle;

    LOG_DEBUG("=== Final Handle Values ===");
    LOG_DEBUG("Protocol Mode[2A4E]: %.4s", RNBD.gatt.protocolModeHandle);     // 1002
    LOG_DEBUG("Report Map[2A4B]: %.4s", RNBD.gatt.reportMapHandle);          // 1004
    LOG_DEBUG("HID Info[2A4A]: %.4s", RNBD.gatt.hidInfoHandle);             // 1006
    LOG_DEBUG("HID Control Point[2A4C]: %.4s", RNBD.gatt.hidCtrlPointHandle); // 1008
    RNBD.callback.delayMs(100);
    LOG_DEBUG("Reference Report[2A4D]: %.4s", RNBD.gatt.referenceReportHandle); // 100A
    LOG_DEBUG("Input Report[2A4D]: %.4s", RNBD.gatt.inputReportHandle);      // 100B
    LOG_DEBUG("Output Report[2A4D]: %.4s", RNBD.gatt.outputReportHandle);    // 100D
    LOG_DEBUG("Boot Reference[2A22]: %.4s", RNBD.gatt.bootReferenceHandle);  // 100F
    LOG_DEBUG("Boot Input[2A22]: %.4s", RNBD.gatt.bootInputHandle);         // 1010
    LOG_DEBUG("Boot Output[2A32]: %.4s", RNBD.gatt.bootOutputHandle);       // 1012
    RNBD.callback.delayMs(100);

    return result;
}


bool GATT_ble_Init(void)
{
	// Issue PZ command to clear GATT services
	LOG_DEBUG("Clearing Previous GATT Profile");
	if (!GATT_clearProfile()) {
		LOG_ERROR("Failed to clear GATT services with PZ command");
		return false;//Factory Reset Module
	}

	 LOG_DEBUG("Initialising GATT Profile");
	// Attempt GATT initialisation
	if (GATT_HID_Init()) {
		LOG_DEBUG("GATT Initialisation Successful");
		return true;
	}

	return false;
}



static bool GATT_enableProtocolMode(void)
{
    LOG_DEBUG("Enabling Protocol Mode");

    uint8_t protocolMode = PROTOCOL_MODE_REPORT;
    bool result = true;

    // Write Report Map Descriptor
    LOG_DEBUG("Writing Report Map Handle: %.4s", RNBD.gatt.reportMapHandle);
    result &= GATT_writeCharacteristic(RNBD.gatt.reportMapHandle, keyboardReportDescriptor, sizeof(keyboardReportDescriptor));

    LOG_DEBUG("Writing CCCD report Mode Handle: %.4s", RNBD.gatt.inputReportHandle);
    result &= GATT_configureCCCD(RNBD.gatt.outputReportHandle,true, false);

    LOG_DEBUG("Writing CCCD boot Mode Handle: %.4s", RNBD.gatt.bootInputHandle);
    result &= GATT_configureCCCD(RNBD.gatt.bootOutputHandle,true, false);
    // Write Protocol Mode
    LOG_DEBUG("Writing Protocol Mode Handle: %.4s", RNBD.gatt.protocolModeHandle);
    result &= GATT_writeCharacteristic(RNBD.gatt.protocolModeHandle, &protocolMode, 1);

    // Write HID Info
    LOG_DEBUG("Writing HID Info Handle: %.4s", RNBD.gatt.hidInfoHandle);
    result &= GATT_writeCharacteristic(RNBD.gatt.hidInfoHandle, hidInfo, sizeof(hidInfo));

    LOG_DEBUG("Protocol Mode := %s", result ? "set" : "not set");
    return result;
}


/**
 * Define a new service using PS command
 */
bool GATT_defineService(const uint8_t *uuid, uint8_t uuidLen) {
    if (uuid == NULL || uuidLen == 0 || uuidLen > RNBD_BUFFER_SIZE - 4) {
        return false;
    }

    // Construct "PS,<uuid>\r\n"
    RNBD.uart.command_buffer[0] = 'P';
    RNBD.uart.command_buffer[1] = 'S';
    RNBD.uart.command_buffer[2] = ',';

    for (uint8_t i = 0; i < uuidLen; i++) {
        RNBD.uart.command_buffer[3 + i] = uuid[i];
    }
    uint8_t cmdLen = uuidLen + 3;

    RNBD.uart.command_buffer[cmdLen++] = '\r';
    RNBD.uart.command_buffer[cmdLen++] = '\n';

    return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer,
                                          cmdLen, expectedResponse, sizeof(expectedResponse));
}

/**
 * Define a characteristic using PC command
 */
bool GATT_defineCharacteristic(const uint8_t *uuid, uint8_t uuidLen, uint8_t properties, uint8_t maxSize) {
    if (uuid == NULL || uuidLen == 0 || uuidLen > RNBD_BUFFER_SIZE - 8) {
        return false;
    }

    // Construct "PC,<uuid>,<properties>,<maxSize>\r\n"
    RNBD.uart.command_buffer[0] = 'P';
    RNBD.uart.command_buffer[1] = 'C';
    RNBD.uart.command_buffer[2] = ',';

    // Add UUID
    uint8_t cmdLen = 3;
    for (uint8_t i = 0; i < uuidLen; i++) {
        RNBD.uart.command_buffer[cmdLen++] = uuid[i];
    }

    // Add properties in hex
    RNBD.uart.command_buffer[cmdLen++] = ',';
    RNBD.uart.command_buffer[cmdLen++] = NIBBLE2ASCII(properties >> 4);
    RNBD.uart.command_buffer[cmdLen++] = NIBBLE2ASCII(properties & 0x0F);

    // Add max size in hex
    RNBD.uart.command_buffer[cmdLen++] = ',';
    RNBD.uart.command_buffer[cmdLen++] = NIBBLE2ASCII(maxSize >> 4);
    RNBD.uart.command_buffer[cmdLen++] = NIBBLE2ASCII(maxSize & 0x0F);

    RNBD.uart.command_buffer[cmdLen++] = '\r';
    RNBD.uart.command_buffer[cmdLen++] = '\n';

    return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer,
                                          cmdLen, expectedResponse, sizeof(expectedResponse));
}

/**
 * Write to a characteristic using SHW command
 */
bool GATT_writeCharacteristic(const uint8_t* handle, const uint8_t* data, uint8_t dataLen) {
    if (data == NULL || dataLen == 0 || dataLen > RNBD_BUFFER_SIZE - 10) {
        return false;
    }

    // Construct "SHW,<handle>,<data>\r\n"
    uint8_t cmdLen = 0;
    memset(RNBD.uart.command_buffer, 0, COMMAND_BUFFER_SIZE);  // Clear buffer first

    // Add command prefix
    const char* prefix = "SHW,";
    memcpy(RNBD.uart.command_buffer, prefix, 4);
    cmdLen = 4;

    // Add handle (exactly 4 ASCII chars)
    memcpy(&RNBD.uart.command_buffer[cmdLen], handle, 4);
    cmdLen += 4;

    RNBD.uart.command_buffer[cmdLen++] = ',';

    // Add data
    for (uint8_t i = 0; i < dataLen && cmdLen < (COMMAND_BUFFER_SIZE - 3); i++) {
        RNBD.uart.command_buffer[cmdLen++] = NIBBLE2ASCII(data[i] >> 4);
        RNBD.uart.command_buffer[cmdLen++] = NIBBLE2ASCII(data[i] & 0x0F);
    }

    RNBD.uart.command_buffer[cmdLen++] = '\r';
    RNBD.uart.command_buffer[cmdLen++] = '\n';

    return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer,
                                          cmdLen,
                                          expectedResponse,
                                          sizeof(expectedResponse));
}


void GATT_readCharacteristic(const uint8_t* handle, uint8_t dataLen)
{
	uint8_t cmdLen = 0;
	int responseRead = 0;
	uint8_t responseBuffer[RNBD_BUFFER_SIZE] = {0};

	 // Construct "SHR,<handle>\r\n"
	RNBD.uart.command_buffer[cmdLen++] = 'S';
	RNBD.uart.command_buffer[cmdLen++] = 'H';
	RNBD.uart.command_buffer[cmdLen++] = 'R';
	RNBD.uart.command_buffer[cmdLen++] = ',';

	for (uint8_t i = 0; i < dataLen; i++) {
		RNBD.uart.command_buffer[cmdLen++] = handle[i];
	}

	RNBD.uart.command_buffer[cmdLen++] = '\r';
	RNBD.uart.command_buffer[cmdLen++] = '\n';

	// Clear any pending data
	while (RNBD.callback.dataReady()) {
		RNBD.callback.read();
	}

	RNBD_SendCmd(RNBD.uart.command_buffer, cmdLen);

	// Wait for data
	int ResponseTime = 0;
	while (!RNBD.callback.dataReady() && ResponseTime < RESPONSE_TIMEOUT) {
		RNBD.callback.delayMs(1);
		ResponseTime++;
	}

	// Read data
	ResponseTime = 0;
	while (responseRead < sizeof(responseBuffer)) {
		if (RNBD.callback.dataReady()) {
			responseBuffer[responseRead++] = RNBD.callback.read();
			ResponseTime = 0;
		} else {
			RNBD.callback.delayMs(1);
			if (++ResponseTime >= 10) {
				break;
			}
		}
	}

	// Debug output
	LOG_DEBUG("Total bytes read: %d", responseRead);
	LOG_INFO("SHR: %s", responseBuffer);
}

/**
 * Send notification using IE command
 */
bool GATT_transmitCharacteristic(const uint8_t* handle, const uint8_t* data, uint8_t dataLen)
{
	if (data == NULL || handle == NULL || dataLen == 0 ||
	    ((dataLen * 2) + 15) > RNBD_BUFFER_SIZE) {
	    return false;
	}

    // Construct "IE,<handle>,<length>,<data>\r\n"
    uint8_t cmdLen = 0;
    RNBD.uart.command_buffer[cmdLen++] = 'I';
    RNBD.uart.command_buffer[cmdLen++] = 'E';
    RNBD.uart.command_buffer[cmdLen++] = ',';

    // Add handle (already in string format)
    for (uint8_t i = 0; handle[i] != '\0' && i < 4; i++) {
        RNBD.uart.command_buffer[cmdLen++] = handle[i];
    }

    // Add length as 16-bit hex (4 characters)
    RNBD.uart.command_buffer[cmdLen++] = ',';
    RNBD.uart.command_buffer[cmdLen++] = NIBBLE2ASCII((dataLen >> 12) & 0x0F);
    RNBD.uart.command_buffer[cmdLen++] = NIBBLE2ASCII((dataLen >> 8) & 0x0F);
    RNBD.uart.command_buffer[cmdLen++] = NIBBLE2ASCII((dataLen >> 4) & 0x0F);
    RNBD.uart.command_buffer[cmdLen++] = NIBBLE2ASCII(dataLen & 0x0F);

    // Add data as hex
    RNBD.uart.command_buffer[cmdLen++] = ',';
    for (uint8_t i = 0; i < dataLen; i++) {
        RNBD.uart.command_buffer[cmdLen++] = NIBBLE2ASCII(data[i] >> 4);
        RNBD.uart.command_buffer[cmdLen++] = NIBBLE2ASCII(data[i] & 0x0F);
    }

    RNBD.uart.command_buffer[cmdLen++] = '\r';
    RNBD.uart.command_buffer[cmdLen++] = '\n';

	LOG_DEBUG("Report : %s", RNBD.uart.command_buffer);

    return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer,
                                          cmdLen,
                                          expectedResponse,
                                          sizeof(expectedResponse));
}


bool GATT_clearProfile(void)
{
	uint8_t cmdLen = 0;
	RNBD.uart.command_buffer[cmdLen++] = 'P';
	RNBD.uart.command_buffer[cmdLen++] = 'Z';
	RNBD.uart.command_buffer[cmdLen++] = '\r';
	RNBD.uart.command_buffer[cmdLen++] = '\n';

	return RNBD_SendCommand_ReceiveResponse(RNBD.uart.command_buffer,cmdLen,
			expectedResponse, sizeof(expectedResponse));
}


void GATT_List_Services(void) {
    uint8_t cmdLen = 0;
    int responseRead = 0;
    uint8_t responseBuffer[RNBD_BUFFER_SIZE] = {0};

    // Build command
    RNBD.uart.command_buffer[cmdLen++] = 'L';
    RNBD.uart.command_buffer[cmdLen++] = 'S';
    RNBD.uart.command_buffer[cmdLen++] = '\r';
    RNBD.uart.command_buffer[cmdLen++] = '\n';

    // Clear any pending data
    while (RNBD.callback.dataReady()) {
        RNBD.callback.read();
    }

    RNBD_SendCmd(RNBD.uart.command_buffer, cmdLen);

    // Wait for data
    int ResponseTime = 0;
    while (!RNBD.callback.dataReady() && ResponseTime < RESPONSE_TIMEOUT) {
        RNBD.callback.delayMs(1);
        ResponseTime++;
    }

    // Read data
    ResponseTime = 0;
    while (responseRead < sizeof(responseBuffer)) {
        if (RNBD.callback.dataReady()) {
            responseBuffer[responseRead++] = RNBD.callback.read();
            ResponseTime = 0;
        } else {
            RNBD.callback.delayMs(1);
            if (++ResponseTime >= 10) {
                break;
            }
        }
    }

    // Debug output
    LOG_DEBUG("Total bytes read: %d", responseRead);
    LOG_DEBUG("LS: %.50s", responseBuffer);
    LOG_DEBUG("LS: %.50s", &responseBuffer[50]);
    LOG_DEBUG("LS: %.50s", &responseBuffer[100]);
    LOG_DEBUG("LS: %.50s", &responseBuffer[150]);
}

/**
 * Send HID report using IE command
 */
bool GATT_sendHIDReport(const uint8_t *data, uint8_t dataLen) {
    return GATT_transmitCharacteristic(RNBD.gatt.referenceReportHandle, data, dataLen);
}

/**
 * Configure CCCD using SHW command
 */
bool GATT_configureCCCD(const uint8_t* handle, bool notifications, bool indications)
{
    // For BLE, CCCD is little-endian
    // notifications = 0x0001
    // indications = 0x0002
    uint8_t value[2];
    if (notifications) {
        value[0] = 0x00;  // Low byte
        value[1] = 0x01;  // High byte
    } else if (indications) {
        value[0] = 0x00;  // Low byte
        value[1] = 0x02;  // High byte
    } else {
        value[0] = 0x00;
        value[1] = 0x00;
    }

    return GATT_writeCharacteristic(handle, value, sizeof(value));
}

/**
 * Update LED state bitmap
 */
void GATT_updateLEDState(uint8_t ledBitmap) {
    RNBD.gatt.ledStateBitmap = ledBitmap;

    LOG_DEBUG("LED State Updated: Num Lock=%d, Caps Lock=%d, Scroll Lock=%d",
              (ledBitmap & 0x01) ? 1 : 0,
              (ledBitmap & 0x02) ? 1 : 0,
              (ledBitmap & 0x04) ? 1 : 0);
}
