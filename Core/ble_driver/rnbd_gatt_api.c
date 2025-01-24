/*
 * rnbd_gatt_api.c
 *
 *  Created on: Jan 9, 2025
 *      Author: bettysidepiece
 */
#include "logger.h"
#include "ble_hid_keyboard.h"
#include "rnbd.h"


#define DESCRIPTOR_UUID_16 0x2908  // Report Reference Descriptor UUID

static const uint8_t hidInfo[] = {
		0x11, 0x01,
		0x00, 0x01
}; // HID Version 1.11, No Country Code, Boot Protocol Supported

static const uint8_t pnp_id[] = {
    0x01,       // Vendor ID Source: Bluetooth SIG
    0xFF, 0xFF, // Vendor ID: Reserved for testing
    0x00, 0x01, // Product ID: Reserved for testing
    0x00, 0x01  // Product Version: 1.0.0
};

__ALIGN_BEGIN static const uint8_t HID_KEYBOARD_ReportDesc[80] __ALIGN_END =
{
    0x05, 0x01,       // Usage Page (Generic Desktop Controls)
    0x09, 0x06,       // Usage (Keyboard)
    0xA1, 0x01,       // Collection (Application)

    // Modifier Keys
    0x05, 0x07,       // Usage Page (Keyboard/Keypad)
    0x19, 0xE0,       // Usage Minimum (224 - Left Control)
    0x29, 0xE7,       // Usage Maximum (231 - Right GUI)
    0x15, 0x00,       // Logical Minimum (0)
    0x25, 0x01,       // Logical Maximum (1)
    0x75, 0x01,       // Report Size (1)
    0x95, 0x08,       // Report Count (8)
    0x81, 0x02,       // Input (Data, Variable, Absolute)

    // Reserved Byte
    0x75, 0x08,       // Report Size (8)
    0x95, 0x01,       // Report Count (1)
    0x81, 0x01,       // Input (Constant)

    // LED Output Report (Num Lock, Caps Lock, etc.)
    0x95, 0x05,       // Report Count (5 LEDs)
    0x75, 0x01,       // Report Size (1)
    0x05, 0x08,       // Usage Page (LEDs)
    0x19, 0x01,       // Usage Minimum (1 - Num Lock)
    0x29, 0x05,       // Usage Maximum (5 - Kana)
    0x91, 0x02,       // Output (Data, Variable, Absolute)

    // Padding for LED output
    0x95, 0x01,       // Report Count (1)
    0x75, 0x03,       // Report Size (3)
    0x91, 0x01,       // Output (Constant)

    // Boot Protocol Keys (6KRO)
    0x05, 0x07,       // Usage Page (Keyboard/Keypad)
    0x19, 0x00,       // Usage Minimum (Reserved)
    0x29, 0x65,       // Usage Maximum (101 - Application)
    0x15, 0x00,       // Logical Minimum (0)
    0x25, 0x65,       // Logical Maximum (101)
    0x75, 0x08,       // Report Size (8)
    0x95, 0x06,       // Report Count (6)
    0x81, 0x00,       // Input (Data, Array)

    // NKRO Bitmap (80 Keys = 10 Bytes)
    0x75, 0x01,       // Report Size (1)
    0x95, 0x50,       // Report Count (80 keys)
    0x05, 0x07,       // Usage Page (Keyboard/Keypad)
    0x19, 0x00,       // Usage Minimum (Reserved)
    0x29, 0x4F,       // Usage Maximum (79)
    0x15, 0x00,       // Logical Minimum (0)
    0x25, 0x01,       // Logical Maximum (1)
    0x81, 0x02,       // Input (Data, Variable, Absolute)

    0xC0              // End Collection
};

// Expected response for successful commands
static const uint8_t expectedResponse[] = { 'A', 'O', 'K', '\r', '\n', 'C', 'M', 'D', '>', ' ' };
static bool GATT_configureHID(void);

static void set_Handle(uint16_t value, uint8_t* handle)
{
    const char hex_chars[] = "0123456789ABCDEF";
    handle[0] = hex_chars[(value >> 12) & 0xF];
    handle[1] = hex_chars[(value >> 8) & 0xF];
    handle[2] = hex_chars[(value >> 4) & 0xF];
    handle[3] = hex_chars[value & 0xF];
}


static void GATT_setEventCallbacks(void){
	RNBD.gatt.events.gatt_cccd = GATT_configureCCCD;
	RNBD.gatt.events.gatt_write = GATT_writeCharacteristic;
	RNBD.gatt.events.gatt_notification = GATT_writeCharacteristic;
	RNBD.gatt.events.gatt_configure_hid = GATT_configureHID;
	RNBD.gatt.events.rediscovery_required = NULL;
}

static bool GATT_InitDeviceInfoService(void) {
    bool result = true;
    // Initialize Device Information Service
    result &= GATT_defineService((uint8_t*)DEVICE_INFO_UUID, sizeof(DEVICE_INFO_UUID));

    // Add PnP ID Characteristic
    result &= GATT_defineCharacteristic(
        (uint8_t*)PNP_ID_UUID,
        4,
        PROPERTY_READ,
        sizeof(pnp_id)  // Size: 7 bytes
    );

    // Write the PnP ID value
    result &= GATT_writeCharacteristic((const uint8_t*)"1002", pnp_id, sizeof(pnp_id));

    if (!result) {
        LOG_ERROR("Device Information Service Initialization Failed");
    }

    return result;
}

static bool GATT_InitBatteryService(void) {
    // Initialize Battery Service
	 bool result = true;

	// Battery Service (Primary)
	result &= GATT_defineService((uint8_t*)BATTERY_SERVICE_UUID, 4);

	// Battery Level Characteristic
	result &= GATT_defineCharacteristic(
		(uint8_t*)BATTERY_LEVEL_UUID,
		4,
		PROPERTY_READ | PROPERTY_NOTIFY | PROPERTY_AUTHENTICATED,
		1  // Size: 1 byte (0-100%)
	);

	set_Handle(0x1005, RNBD.gatt.batterylevelHandle);

	uint8_t battery_level = 95;
	GATT_writeCharacteristic((const uint8_t*)RNBD.gatt.batterylevelHandle,&battery_level, 1);

	return result;
}

static bool GATT_InitHIDService(void) {
    bool result = true;
    uint16_t current_handle = 0x1007;

    // Initialize HID Service
    result &= GATT_defineService((uint8_t*)HID_SERVICE_UUID, 4);
//**
//----------------------------------------------------------------------------------------------------
    // Report Map characteristic
	result &= GATT_defineCharacteristic(
		(uint8_t*)HID_REPORT_MAP_UUID,
		4,
		PROPERTY_READ | PROPERTY_AUTHENTICATED,
		sizeof(HID_KEYBOARD_ReportDesc)
	);

	current_handle += 2;
	set_Handle(current_handle, RNBD.gatt.reportMapHandle);

	// Write Report Map Descriptor
	result &= GATT_writeCharacteristic(RNBD.gatt.reportMapHandle,
			HID_KEYBOARD_ReportDesc, sizeof(HID_KEYBOARD_ReportDesc));

//------------------------------------------------------------------------------------------------------
	// Protocol Mode characteristic
	result &= GATT_defineCharacteristic(
		(uint8_t*)HID_PROTOCOL_MODE_UUID,
		4,
		PROPERTY_READ | PROPERTY_WRITE_NO_ACK | PROPERTY_AUTHENTICATED,
		1  // Size: 1 byte
	);

	current_handle += 2;
	set_Handle(current_handle, RNBD.gatt.protocolModeHandle);

	uint8_t data = 0x01;
	result &= GATT_writeCharacteristic((const uint8_t*)RNBD.gatt.protocolModeHandle, &(data), sizeof(data));
//----------------------------------------------------------------------------------------------------

    // HID Information characteristic
    result &= GATT_defineCharacteristic(
        (uint8_t*)HID_INFO_UUID,
        4,
        PROPERTY_READ | PROPERTY_AUTHENTICATED,
        sizeof(hidInfo)
    );

    current_handle += 2;
    set_Handle(current_handle, RNBD.gatt.hidInfoHandle);

    // Write HID Info
	result &= GATT_writeCharacteristic(RNBD.gatt.hidInfoHandle,
			hidInfo, sizeof(hidInfo));

    // HID Control Point characteristic
    result &= GATT_defineCharacteristic(
        (uint8_t*)HID_CONTROL_POINT_UUID,
        4,
        PROPERTY_WRITE_NO_ACK,
        1  // Size: 1 byte
    );

    current_handle += 2;
    set_Handle(current_handle, RNBD.gatt.hidCtrlPointHandle);

///--------------------------------------------------------------------------------------------------
    uint8_t enable_notifications[2] = {0x01, 0x01}; // Little-endian format

    // Input Report characteristic
    result &= GATT_defineCharacteristic(
        (uint8_t*)HID_REPORT_UUID,
        4,
        PROPERTY_READ | PROPERTY_NOTIFY,
        sizeof(hid_report_t)
    );

    current_handle += 2;
    set_Handle(current_handle, RNBD.gatt.referenceReportHandle);

    result &= GATT_writeCharacteristic(RNBD.gatt.referenceReportHandle,
    		enable_notifications, 2);

    current_handle += 1;  // Skip handle for CCCD

    set_Handle(current_handle, RNBD.gatt.inputReportHandle);

    // Output Report characteristic
    result &= GATT_defineCharacteristic(
        (uint8_t*)HID_REPORT_UUID,
        4,
        PROPERTY_READ | PROPERTY_WRITE | PROPERTY_WRITE_NO_ACK,
        1  // Size: 1 byte for LED states
    );
    current_handle += 2;
    set_Handle(current_handle, RNBD.gatt.outputReportHandle);

    uint8_t outputReport = 0x00; // Initial LED state
    result &= GATT_writeCharacteristic(RNBD.gatt.outputReportHandle, &outputReport, 1);


    if (!result) {
        LOG_ERROR("HID Service Initialization Failed");
    }

    return result;
}

bool GATT_HID_Init(void) {
    bool result = true;

    // Initialize Device Information Service
    result &= GATT_InitDeviceInfoService();
    LOG_DEBUG("GATT Device Info Initialised");

    // Initialize Battery Service
    result &= GATT_InitBatteryService();
    LOG_DEBUG("GATT Battery Service Initialised");

    // Initialize HID Service
    result &= GATT_InitHIDService();
    LOG_DEBUG("GATT HID Service Initialised");

    // Set up event callbacks
    GATT_setEventCallbacks();

    LOG_DEBUG("GATT Profile Initialization %s", result ? "Successful" : "Failed");
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



static bool GATT_configureHID(void)
{;
    bool result = true;
    //RNBD_ServiceChangeIndicator();
    //enableHIDNotifications();

    LOG_DEBUG("Protocol Mode = %s", result ? "set" : "not set");

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

    LOG_DEBUG("%s",RNBD.uart.command_buffer);

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
    /*
    LOG_DEBUG("Total bytes read: %d", responseRead);
    LOG_DEBUG("LS: %.50s", responseBuffer);
    LOG_DEBUG("LS: %.50s", &responseBuffer[50]);
    LOG_DEBUG("LS: %.50s", &responseBuffer[100]);
    LOG_DEBUG("LS: %.50s", &responseBuffer[150]);
    LOG_DEBUG("LS: %.50s", &responseBuffer[200]);
    */
}



bool GATT_sendHIDReport(const uint8_t *data, uint8_t dataLen) {
    if(!RNBD.connected){
        LOG_DEBUG("No connection");
        return false;
    }
    return GATT_writeCharacteristic(RNBD.gatt.referenceReportHandle, data, dataLen);
}

/**
 * Configure CCCD using SHW command
 */
bool GATT_configureCCCD(const uint8_t* handle, bool notifications, bool indications) {
    uint8_t value[2] = {0};

    // Convert handle to string for comparison
    char handle_str[5] = {0};
    memcpy(handle_str, handle, 4);

    if (notifications) {
        value[1] = 0x00;  // Low byte
        value[0] = 0x01;  // High byte

        // Track which characteristic is being enabled
        if (strcmp(handle_str, "100B") == 0) {
            RNBD.gatt.input_report_notifications = true;
            LOG_DEBUG("Input Report notifications enabled");
        }

    } else {
        value[0] = 0x00;
        value[1] = 0x00;

        // Track which characteristic is being disabled
        if (strcmp(handle_str, "100B") == 0) {
            RNBD.gatt.input_report_notifications = false;
            LOG_DEBUG("Input Report notifications disabled");
        }
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
