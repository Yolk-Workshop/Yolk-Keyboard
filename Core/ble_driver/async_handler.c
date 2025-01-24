/*
 * async_handler.c
 */

#include "logger.h"
#include "rnbd.h"

// Message type definitions
typedef enum {
    MSG_UNKNOWN = 0,
    MSG_CONNECT,
    MSG_DISCONNECT,
    MSG_SECURED,
    MSG_STREAM_OPEN,
    MSG_OTA_REQUEST,
    MSG_WRITE_VALUE,
    MSG_NOTIFICATION,
    MSG_INDICATION,
    MSG_WRITE_CONFIG,
    MSG_REDISCOVERY,
    MSG_MEMORY_ERROR,
	MSG_ERROR_CONN
} message_type_t;

uint8_t scanMsg(const char* msg, char* handle, char* val) {
    // Initialize return value to 0 (no fields parsed)
    uint8_t parsed_count = 0;

    // Validate input pointers
    if (!msg || !handle || !val) {
        return 0;
    }

    // Find first comma
    const char* start_handle = strstr(msg, ",");
    if (!start_handle) {
        return 0;  // No first comma found
    }
    start_handle++;  // Skip past first comma

    // Find second comma
    const char* end_handle = strstr(start_handle, ",");
    if (!end_handle) {
        return 0;  // No second comma found
    }

    // Extract handle
    size_t handle_len = end_handle - start_handle;
    if (handle_len > 0) {
        memcpy(handle, start_handle, handle_len);
        handle[handle_len] = '\0';  // Null-terminate
        parsed_count++;
    }

    // Find start of value
    const char* start_val = end_handle + 1;

    // Find ending %
    const char* end_val = strstr(start_val, "%");
    if (end_val) {
        size_t val_len = end_val - start_val;
        if (val_len > 0) {
            memcpy(val, start_val, val_len);
            val[val_len] = '\0';  // Null-terminate
            parsed_count++;
        }
    }

    return parsed_count;
}

// Helper function to identify message type
static message_type_t getMessageType(const char* message) {
    if (!message) return MSG_UNKNOWN;

    if (strstr(message, "CONNECT")) return MSG_CONNECT;
    if (strstr(message, "DISCONNECT")) return MSG_DISCONNECT;
    if (strstr(message, "SECURED")) return MSG_SECURED;
    if (strstr(message, "STREAM_OPEN")) return MSG_STREAM_OPEN;
    if (strstr(message, "OTA_REQ")) return MSG_OTA_REQUEST;
    if (strstr(message, "WV,")) return MSG_WRITE_VALUE;
    if (strstr(message, "NOTI,")) return MSG_NOTIFICATION;
    if (strstr(message, "INDI,")) return MSG_INDICATION;
    if (strstr(message, "WC,")) return MSG_WRITE_CONFIG;
    if (strstr(message, "RE_DISCV")) return MSG_REDISCOVERY;
    if (strstr(message, "ERR_MEMORY")) return MSG_MEMORY_ERROR;
    if (strstr(message, "ERR_CONN")) return MSG_ERROR_CONN;
    return MSG_UNKNOWN;
}

// Handle device connection
static void handleConnect(const char* message) {
    char* parts[4] = {0};
    char* token = strtok((char*)message, ",");
    int i = 0;

    while (token != NULL && i < 4) {
        parts[i++] = token;
        token = strtok(NULL, ",");
    }

    if (i == 4) {
        int slot = -1;
        uint8_t addr_type = parts[1][0] - '0';
        char* mac = parts[2];
        char* handle = parts[3];  // Has % at end

        // Find available slot
        for(int j = 0; j < MAX_CONNECTIONS; j++) {
            if(RNBD.gatt.connections[j].active) {
                if(memcmp(RNBD.gatt.connections[j].mac_addr, mac, MAC_ADDR_LEN) == 0) {
                    slot = j;
                    break;
                }
            } else if(slot == -1) {
                slot = j;
            }
        }

        if(slot >= 0 && slot < MAX_CONNECTIONS) {
            connection_info_t* conn = &RNBD.gatt.connections[slot];
            memcpy(conn->mac_addr, mac, MAC_ADDR_LEN);
            memcpy(conn->conn_handle, handle, CONN_HANDLE_LEN);
            conn->address_type = addr_type;
            conn->active = true;

            if(RNBD.gatt.num_connections <= slot) {
                RNBD.gatt.num_connections = slot + 1;
            }
            RNBD.gatt.current_connection = slot;

            LOG_DEBUG("Connection %d: Handle=%.4s, MAC=%.12s, Type=%d",
                     slot, handle, mac, addr_type);
        }
    }
}

// Handle device disconnection
static void handleDisconnect(const char* message) {
    char* mac = (char*)message + 11;  // Skip "DISCONNECT,"
    for(int i = 0; i < RNBD.gatt.num_connections; i++) {
        if(memcmp(RNBD.gatt.connections[i].mac_addr, mac, MAC_ADDR_LEN) == 0) {
            RNBD.gatt.connections[i].active = false;
            // Reset CCD states
            RNBD.gatt.input_report_notifications = false;
            LOG_DEBUG("Device disconnected: %.12s", mac);
            break;
        }
    }

    RNBD.connected = false;
    RNBD.OTAComplete = false;
    RNBD.device.connection_timer = 1000UL;
}

// Handle OTA request
static void handleOTARequest(void) {
    RNBD.OTAComplete = true;
    const char* response = "OTA,01\r\n";
    for(const char* p = response; *p; p++) {
        RNBD.callback.write(*p);
    }
}

static void getValue(const char* value, uint8_t len, uint8_t* data_buffer)
{
	char temp[3] = {0}; // Temporary buffer for 2 hex chars + null terminator

	// Parse the string into the buffer
	for (size_t i = 0; i < len; i += 2) {
		temp[0] = value[i];
		temp[1] = (i + 1 < len) ? value[i + 1] : '0'; // Handle odd length
		data_buffer[i / 2] = (uint8_t)strtol(temp, NULL, 16);
	}
}

// Handle characteristic write
static void handleWriteValue(const char* message) {
    char handle[5] = {0};
    char value[RNBD_BUFFER_SIZE] = {0};
    RNBD.callback.nonBlockDelayMs(6); // 7.5 ms delay

    if (scanMsg(message, handle, value) == 2) {
        size_t value_len = strlen(value);
        if (value_len % 2 != 0) {
            LOG_WARNING("Hexadecimal string has odd length: %s", value);
        }

        getValue(value, value_len, RNBD.gatt.buffer);

        LOG_DEBUG("Handle: %s, Data: %s", handle, value);

        if (strcmp(handle, "100B") == 0) { // Protocol Mode
            if (RNBD.gatt.events.gatt_write) {
                RNBD.gatt.events.gatt_write(
                    (const uint8_t*)handle, RNBD.gatt.buffer, value_len / 2);
                LOG_DEBUG("Protocol Mode changed");
                return;
            }
        }

        if (strcmp(handle, "1006") == 0) { // Another handle
            if (RNBD.gatt.events.gatt_write) {
                RNBD.gatt.events.gatt_write(
                    (const uint8_t*)handle, RNBD.gatt.buffer, value_len / 2);
                return;
            }
        }

        // Output Report write (LED states)
        if (strcmp(handle, "100D") == 0) {
            GATT_updateLEDState(RNBD.gatt.buffer[0]);
            return;
        }

        // Generic write callback
        if (RNBD.gatt.events.gatt_write) {
            LOG_DEBUG("Calling generic write callback");
            RNBD.gatt.events.gatt_write(
                (const uint8_t*)handle, RNBD.gatt.buffer, value_len / 2);
        }
    } else {
        LOG_WARNING("Failed to parse write value message");
    }
}


// Handle notification
static void handleNotification(const char* message) {
    char handle[5] = {0};              // Buffer for the handle
    char value[RNBD_BUFFER_SIZE] = {0}; // Buffer for the value

    if (scanMsg(message, handle, value) == 2) {
        LOG_DEBUG("Notification from handle %.4s: %s", handle, value);

        // Calculate the length of the value string
        size_t value_len = strlen(value);
        if (value_len == 0 || value_len % 2 != 0) {
            LOG_ERROR("Invalid value length for notification: %s", value);
            return;
        }

        // Buffer to hold parsed data
        uint8_t data_buffer[RNBD_BUFFER_SIZE / 2] = {0}; // Max size for half of RNBD_BUFFER_SIZE

        // Parse the value into the data buffer
        getValue(value, value_len, data_buffer);

        // Trigger the notification callback
        if (RNBD.gatt.events.gatt_notification) {
            RNBD.gatt.events.gatt_notification((const uint8_t*)handle, data_buffer, value_len / 2);
        } else {
            LOG_WARNING("No gatt_notification callback registered");
        }
    } else {
        LOG_WARNING("Failed to parse notification message");
	}
}

// Handle indication
static void handleIndication(const char* message) {
    char handle[5] = {0};
    char value[RNBD_BUFFER_SIZE] = {0};

    if (scanMsg(message, handle, value) == 2) {
        LOG_DEBUG("Indication from handle %.4s: %s", handle, value);
    }
}


// Handle CCCD write
static void handleWriteConfig(const char* message) {
    char handle[5] = {0}; // Buffer for the handle
    char value[5] = {0};  // Buffer for the 4-character hexadecimal value

    if (scanMsg(message, handle, value) == 2) {
        // Validate and convert value to a 16-bit configuration
        char* endptr;
        uint16_t config = (uint16_t)strtol(value, &endptr, 16);

        if (*endptr != '\0') {
            LOG_ERROR("Invalid CCCD value: %s", value);
            return;
        }

        // Decode CCCD bits
        bool notifications = (config & 0x0001) != 0;
        bool indications = (config & 0x0002) != 0;

        LOG_DEBUG("CCCD Write on handle %.4s: notifications=%d, indications=%d",
                  handle, notifications, indications);

        // Trigger the CCCD callback
        if (RNBD.gatt.events.gatt_cccd) {
            RNBD.gatt.events.gatt_cccd((const uint8_t*)handle, notifications, indications);
        } else {
            LOG_WARNING("No CCCD callback registered");
        }
    } else {
        LOG_WARNING("Failed to parse CCCD write message");
    }
}



// Main message handler
void asyncMessageHandler(char* message)
{
    // Sanity check the message
	//__disable_irq();
    if (!message) {
        LOG_ERROR("NULL message received");
        return;
    }

    // Get message type with additional error checking
    message_type_t msgType;
    msgType = getMessageType(message);
    //LOG_DEBUG("Detected message type: %d", msgType);

    // Log the full message before processing
    LOG_INFO("%s", message);

    switch (msgType) {
        case MSG_CONNECT:
            LOG_DEBUG("Processing CONNECT message");
            handleConnect(message);
            break;

        case MSG_DISCONNECT:
            LOG_DEBUG("Processing DISCONNECT message");
            handleDisconnect(message);
            break;

        case MSG_SECURED:
            //LOG_DEBUG("Processing SECURED message");
            if (RNBD.gatt.num_connections > 0 &&
                RNBD.gatt.connections[RNBD.gatt.current_connection].active) {
                RNBD.connected = true;
                RNBD.device.connection_timer = 5000UL;
                RNBD_BondToConnectedDevice();
                LOG_DEBUG("Connection secured and active");
            }
            break;

        case MSG_OTA_REQUEST:
           // LOG_DEBUG("Processing OTA REQUEST message");
            handleOTARequest();
            break;

        case MSG_WRITE_VALUE:
            //LOG_DEBUG("Processing WRITE VALUE message");
            handleWriteValue(message);
            RNBD.gatt.mode = 1;
            LOG_DEBUG("BLE HID MODE : %d",RNBD.gatt.mode);
            break;

        case MSG_NOTIFICATION:
           // LOG_DEBUG("Processing NOTIFICATION message");
            handleNotification(message);
            break;

        case MSG_INDICATION:
            //LOG_DEBUG("Processing INDICATION message");
            handleIndication(message);
            break;

        case MSG_WRITE_CONFIG:
           // LOG_DEBUG("Processing WRITE CONFIG message");
            handleWriteConfig(message);
            break;

        case MSG_REDISCOVERY:
           // LOG_DEBUG("Processing REDISCOVERY message");
            if (RNBD.gatt.events.rediscovery_required &&
                RNBD.gatt.events.rediscovery_required()) {
              //  LOG_DEBUG("Calling GATT_ble_Init()");
                GATT_ble_Init();
            }
            break;

        case MSG_MEMORY_ERROR:
           LOG_ERROR("RNBD Memory Error Detected");
            break;

        default:
           // LOG_DEBUG("Unknown message type, full contents: %s", message);
            break;

    }

    // Final diagnostic log
    //__enable_irq();
    //LOG_DEBUG("Message processing completed");
}

// Public API functions
bool RNBD_IsConnected(void) {
    return RNBD.connected;
}

bool RNBD_IsOTAComplete(void) {
    return RNBD.OTAComplete;
}
