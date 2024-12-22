/*
 * rnbd350.c
 *
 *  Created on: Dec 21, 2024
 *      Author: bettysidepiece
 */

/**
 * @file rnbd350.c
 * @brief RNBD350 Bluetooth Low Energy Module Driver Implementation
 */

#include "rnbd350.h"



// Private helper functions
static rnbd350_status_t send_command(rnbd350_handle_t* handle, const char* cmd, char* response);
static rnbd350_status_t wait_for_response(rnbd350_handle_t* handle, char* response);

extern UART_HandleTypeDef hlpuart1;
uint8_t ble_initialized = 0;
rnbd350_handle_t rnbd350;

void uart_write_wrapper(uint8_t* data, uint16_t len)
{
    HAL_UART_Transmit(&hlpuart1, data, len, 100); // 100ms timeout
}

uint16_t uart_read_wrapper(uint8_t* data, uint16_t len)
{
    if (HAL_UART_Receive(&hlpuart1, data, len, 100) == HAL_OK) {
        return len;
    }
    return 0;
}

rnbd350_status_t rnbd350_init(rnbd350_handle_t* handle, const rnbd350_config_t* config)
{
    if (!handle || !config) {
        return RNBD350_INVALID_PARAM;
    }

    // Copy configuration
    memcpy(&handle->config, config, sizeof(rnbd350_config_t));
    handle->mode = RNBD350_MODE_DATA;

    // Reset module to ensure clean state
    return rnbd350_reset(handle);
}

rnbd350_status_t rnbd350_reset(rnbd350_handle_t* handle)
{
    if (!handle) {
        return RNBD350_INVALID_PARAM;
    }

    char response[RNBD350_MAX_RESPONSE_SIZE];
    rnbd350_status_t status;

    // Enter command mode first
    status = rnbd350_enter_command_mode(handle);
    if (status != RNBD350_OK) {
        return status;
    }

    // Send reset command
    status = send_command(handle, "R,1\r", response);
    if (status != RNBD350_OK) {
        return status;
    }

    // Wait for reboot
    handle->delay_ms(1000);

    // Module will be in data mode after reset
    handle->mode = RNBD350_MODE_DATA;

    return RNBD350_OK;
}

rnbd350_status_t rnbd350_enter_command_mode(rnbd350_handle_t* handle)
{
    if (!handle) {
        return RNBD350_INVALID_PARAM;
    }

    if (handle->mode == RNBD350_MODE_COMMAND) {
        return RNBD350_OK;  // Already in command mode
    }

    char response[RNBD350_MAX_RESPONSE_SIZE];

    // Send $$$ to enter command mode
    handle->uart_write((uint8_t*)"$$$", 3);

    // Wait for CMD> prompt
    rnbd350_status_t status = wait_for_response(handle, response);
    if (status == RNBD350_OK && strstr(response, "CMD>")) {
        handle->mode = RNBD350_MODE_COMMAND;
        return RNBD350_OK;
    }

    return RNBD350_ERROR;
}

rnbd350_status_t rnbd350_exit_command_mode(rnbd350_handle_t* handle)
{
    if (!handle) {
        return RNBD350_INVALID_PARAM;
    }

    if (handle->mode == RNBD350_MODE_DATA) {
        return RNBD350_OK;  // Already in data mode
    }

    char response[RNBD350_MAX_RESPONSE_SIZE];

    // Send --- to exit command mode
    rnbd350_status_t status = send_command(handle, "---\r", response);
    if (status == RNBD350_OK && strstr(response, "END")) {
        handle->mode = RNBD350_MODE_DATA;
        return RNBD350_OK;
    }

    return RNBD350_ERROR;
}

rnbd350_status_t rnbd350_start_advertising(rnbd350_handle_t* handle,
                                         uint16_t interval_ms,
                                         uint16_t timeout_ms)
{
    if (!handle) {
        return RNBD350_INVALID_PARAM;
    }

    char cmd[32];
    char response[RNBD350_MAX_RESPONSE_SIZE];
    rnbd350_status_t status;

    // Enter command mode
    status = rnbd350_enter_command_mode(handle);
    if (status != RNBD350_OK) {
        return status;
    }

    // Format advertising command
    if (interval_ms == 0 && timeout_ms == 0) {
        // Use default parameters
        strcpy(cmd, "A\r");
    } else {
        // Convert ms to required format (0.625ms units)
        uint16_t interval = interval_ms * 8 / 5;  // Convert to 0.625ms units
        uint16_t timeout = timeout_ms / 10;       // Convert to 10ms units

        // Manual string construction
        cmd[0] = 'A';
        cmd[1] = ',';

        // Convert interval to hex string
        for(int i = 0; i < 4; i++) {
            uint8_t nibble = (interval >> (12 - i*4)) & 0xF;
            cmd[2+i] = nibble < 10 ? '0' + nibble : 'A' + (nibble - 10);
        }

        cmd[6] = ',';

        // Convert timeout to hex string
        for(int i = 0; i < 4; i++) {
            uint8_t nibble = (timeout >> (12 - i*4)) & 0xF;
            cmd[7+i] = nibble < 10 ? '0' + nibble : 'A' + (nibble - 10);
        }

        cmd[11] = '\r';
        cmd[12] = '\0';
    }

    // Send command
    status = send_command(handle, cmd, response);
    if (status != RNBD350_OK) {
        return status;
    }

    // Return to data mode
    return rnbd350_exit_command_mode(handle);
}


rnbd350_status_t rnbd350_start_advanced_advertising(rnbd350_handle_t* handle,
                                                   const rnbd350_adv_config_t* config) {
    if (!handle || !config) {
        return RNBD350_INVALID_PARAM;
    }

    char cmd[32];
    char response[RNBD350_MAX_RESPONSE_SIZE];
    rnbd350_status_t status;

    // Enter command mode
    status = rnbd350_enter_command_mode(handle);
    if (status != RNBD350_OK) {
        return status;
    }

    // Convert intervals to 0.625ms units
    uint16_t fast_units = (uint16_t)(config->fast_interval * 8.0f / 5.0f);
    uint16_t slow_units = (uint16_t)(config->slow_interval * 8.0f / 5.0f);
    uint16_t timeout_units = config->fast_timeout / 10; // Convert to 10ms units

    // Build STA command for two-phase advertising
    if (snprintf(cmd, sizeof(cmd), "STA,%04X,%04X,%04X\r",
                fast_units, timeout_units, slow_units) >= sizeof(cmd)) {
        return RNBD350_ERROR;
    }

    // Send command to set advertising parameters
    status = send_command(handle, cmd, response);
    if (status != RNBD350_OK) {
        return status;
    }

    // Start advertising
    status = send_command(handle, "A\r", response);
    if (status != RNBD350_OK) {
        return status;
    }

    // Exit command mode
    return rnbd350_exit_command_mode(handle);
}

rnbd350_status_t rnbd350_stop_advertising(rnbd350_handle_t* handle)
{
    if (!handle) {
        return RNBD350_INVALID_PARAM;
    }

    char response[RNBD350_MAX_RESPONSE_SIZE];
    rnbd350_status_t status;

    // Enter command mode
    status = rnbd350_enter_command_mode(handle);
    if (status != RNBD350_OK) {
        return status;
    }

    // Send stop advertising command
    status = send_command(handle, "Y\r", response);
    if (status != RNBD350_OK) {
        return status;
    }

    // Return to data mode
    return rnbd350_exit_command_mode(handle);
}


static rnbd350_status_t send_command(rnbd350_handle_t* handle, const char* cmd, char* response)
{
    if (!handle || !cmd || !response) {
        return RNBD350_INVALID_PARAM;
    }

    // Send command
    handle->uart_write((uint8_t*)cmd, strlen(cmd));

    // Wait for response
    return wait_for_response(handle, response);
}


static rnbd350_status_t wait_for_response(rnbd350_handle_t* handle, char* response)
{
   if (!handle || !response) {
       return RNBD350_INVALID_PARAM;
   }

   uint32_t timeout_ms = RNBD350_CMD_TIMEOUT_MS;
   uint16_t index = 0;
   uint8_t byte;
   rnbd350_status_t status = RNBD350_TIMEOUT;

   while (timeout_ms > 0) {
       // Check for received byte
       if (handle->uart_read(&byte, 1) == 1) {
           response[index++] = byte;
           // Check for command termination (usually \r\n)
           if (byte == '\n' && index >= 2) {
               response[index] = '\0';
               // Check for AOK response
               if (strstr(response, "AOK")) {
                   status = RNBD350_OK;
                   break;
               }
               // Check for error response
               if (strstr(response, "ERR")) {
                   status = RNBD350_ERROR;
                   break;
               }
               // Keep reading if neither AOK nor ERR found
           }
       }
       handle->delay_ms(1);
       timeout_ms--;
   }

   return status;
}

rnbd350_status_t rnbd350_reset_module(rnbd350_handle_t* handle) {
    if (!handle) {
        return RNBD350_INVALID_PARAM;
    }

    // Enter command mode
    handle->uart_write((uint8_t*)"$$$", 3);
    handle->delay_ms(100);  // Wait for command mode

    // Send reset command
    handle->uart_write((uint8_t*)"R,1\r", 4);
    handle->delay_ms(2000);  // Give module time to complete reset

    // Module will be in data mode after reset
    handle->mode = RNBD350_MODE_DATA;

    return RNBD350_OK;
}


rnbd350_status_t rnbd350_get_connection_status(rnbd350_handle_t* handle)
{
    if (!handle) {  // Fix null check
        return RNBD350_INVALID_PARAM;
    }

    char response[RNBD350_MAX_RESPONSE_SIZE];
    rnbd350_status_t status;

    // Enter command mode
    status = rnbd350_enter_command_mode(handle);
    if (status != RNBD350_OK) {
        return status;
    }

    // Send GK command
    status = send_command(handle, "GK\r", response);
    if (status != RNBD350_OK) {
        return status;
    }

    // Parse response
    if (strstr(response, "None")) {
        // No connection
        memset(&handle->conn_info, 0, sizeof(rnbd350_connection_info_t));  // Fix memset
        status = RNBD350_OK;
    } else {
        // Parse connection info
        char* token;
        char* saveptr;

        // First token: BT Address
        token = strtok_r(response, ",", &saveptr);
        if (token) {
            // Convert hex string to bytes
            for (int i = 0; i < 6; i++) {
                char byte_str[3] = {token[i*2], token[i*2+1], '\0'};
                handle->conn_info.bt_address[i] = (uint8_t)strtol(byte_str, NULL, 16);  // Fix array access
            }

            // Address type
            token = strtok_r(NULL, ",", &saveptr);
            if (token) {
                handle->conn_info.address_type = (uint8_t)atoi(token);

                // Connection type
                token = strtok_r(NULL, ",", &saveptr);
                if (token) {
                    handle->conn_info.conn_type = (uint8_t)atoi(token);
                    status = RNBD350_OK;
                }
            }
        }

        if (!token) {
            status = RNBD350_ERROR;
        }
    }

    // Exit command mode
    rnbd350_status_t exit_status = rnbd350_exit_command_mode(handle);
    if (status == RNBD350_OK) {
        status = exit_status;
    }

    return status;
}


rnbd350_status_t rnbd350_init_hid_keyboard(rnbd350_handle_t* handle) {
   if (!handle) {
       return RNBD350_INVALID_PARAM;
   }

   char response[RNBD350_MAX_RESPONSE_SIZE];
   char cmd[64];
   rnbd350_status_t status;

   // Enter command mode
   status = rnbd350_enter_command_mode(handle);
   if (status != RNBD350_OK) {
       return status;
   }

   // Set appearance as HID device
   if (snprintf(response, RNBD350_MAX_RESPONSE_SIZE, "SDA,%04X\r", BLE_SDA_HID) >= RNBD350_MAX_RESPONSE_SIZE) {
       return RNBD350_ERROR;
   }
   status = send_command(handle, response, response);
   if (status != RNBD350_OK) {
       return status;
   }

   // Set device name and serialized name if not already set
   if (!handle->state.name_set) {
       // Set device name
       if (snprintf(cmd, sizeof(cmd), "SN,%s\r", PRODUCT_NAME) >= sizeof(cmd)) {
           return RNBD350_ERROR;
       }
       status = send_command(handle, cmd, response);
       if (status != RNBD350_OK) return status;

       // Set serialized name (shorter version of product name)
       if (snprintf(cmd, sizeof(cmd), "S-,%s\r", "YolkKbd") >= sizeof(cmd)) {
           return RNBD350_ERROR;
       }
       status = send_command(handle, cmd, response);
       if (status != RNBD350_OK) return status;

       handle->state.name_set = true;
   }

   // Set device information if not already set
   if (!handle->state.device_info_set) {
       // Set Software Version
       if (snprintf(cmd, sizeof(cmd), "SDR,%s\r", SW_VERSION) >= sizeof(cmd)) {
           return RNBD350_ERROR;
       }
       status = send_command(handle, cmd, response);
       if (status != RNBD350_OK) return status;

       // Set Hardware Version
       if (snprintf(cmd, sizeof(cmd), "SDH,%s\r", HW_VERSION) >= sizeof(cmd)) {
           return RNBD350_ERROR;
       }
       status = send_command(handle, cmd, response);
       if (status != RNBD350_OK) return status;

       // Set Product Name
       if (snprintf(cmd, sizeof(cmd), "SDM,%s\r", PRODUCT_NAME) >= sizeof(cmd)) {
           return RNBD350_ERROR;
       }
       status = send_command(handle, cmd, response);
       if (status != RNBD350_OK) return status;

       // Set Manufacturer Name
       if (snprintf(cmd, sizeof(cmd), "SDN,%s\r", VENDOR_NAME) >= sizeof(cmd)) {
           return RNBD350_ERROR;
       }
       status = send_command(handle, cmd, response);
       if (status != RNBD350_OK) return status;

       handle->state.device_info_set = true;
   }

   // Exit command mode
   return rnbd350_exit_command_mode(handle);
}


rnbd350_status_t rnbd350_get_device_info(rnbd350_handle_t* handle)
{
    if (!handle) {
        return RNBD350_INVALID_PARAM;
    }

    char response[RNBD350_MAX_RESPONSE_SIZE];
    rnbd350_status_t status;

    // Enter command mode
    status = rnbd350_enter_command_mode(handle);
    if (status != RNBD350_OK) {
        return status;
    }

    // Send D command
    status = send_command(handle, "D\r", response);
    if (status != RNBD350_OK) {
        return status;
    }

    // Parse response line by line
    char* line = strtok(response, "\r\n");
    while (line != NULL) {
        // Skip empty lines
        if (strlen(line) == 0) {
            line = strtok(NULL, "\r\n");
            continue;
        }

        if (strncmp(line, "MAC=", 4) == 0) {
            // Parse MAC address (format: MAC=001122334455)
            for (int i = 0; i < 6; i++) {
                char byte_str[3] = {line[4+i*2], line[5+i*2], '\0'};
                handle->device_info.mac_address[i] = (uint8_t)strtol(byte_str, NULL, 16);
            }
        }
        else if (strncmp(line, "Name=", 5) == 0) {
            // Parse device name
            strncpy(handle->device_info.device_name, line + 5, sizeof(handle->device_info.device_name) - 1);
            handle->device_info.device_name[sizeof(handle->device_info.device_name) - 1] = '\0';
        }
        else if (strncmp(line, "Connected=", 10) == 0) {
            if (strcmp(line + 10, "no") == 0) {
            	handle->device_info.is_connected = false;
                memset(handle->device_info.connected_mac, 0, 6);
                handle->device_info.connected_addr_type = 0;
            } else {
            	handle->device_info.is_connected = true;
                // Parse connected device info (format: Connected=112233445566,0)
                char* addr_str = line + 10;
                char* addr_type_str = strchr(addr_str, ',');
                if (addr_type_str) {
                    *addr_type_str = '\0';
                    addr_type_str++;

                    // Parse MAC address
                    for (int i = 0; i < 6; i++) {
                        char byte_str[3] = {addr_str[i*2], addr_str[i*2+1], '\0'};
                        handle->device_info.connected_mac[i] = (uint8_t)strtol(byte_str, NULL, 16);
                    }

                    // Parse address type
                    handle->device_info.connected_addr_type = (uint8_t)atoi(addr_type_str);
                }
            }
        }
        else if (strncmp(line, "Auth=", 5) == 0) {
            // Parse authentication method
        	handle->device_info.auth_method = (uint8_t)atoi(line + 5);
        }
        else if (strncmp(line, "Features=", 9) == 0) {
            // Parse device features (hex format)
        	handle->device_info.device_features = (uint16_t)strtol(line + 9, NULL, 16);
        }
        else if (strncmp(line, "Services=", 9) == 0) {
            // Parse server services bitmap (hex format)
        	handle->device_info.server_services = (uint8_t)strtol(line + 9, NULL, 16);
        }
        else if (strncmp(line, "Pin=", 4) == 0) {
            // Parse fixed pin code
            strncpy(handle->device_info.pin_code, line + 4, sizeof(handle->device_info.pin_code) - 1);
            handle->device_info.pin_code[sizeof(handle->device_info.pin_code) - 1] = '\0';
        }

        line = strtok(NULL, "\r\n");
    }

    // Exit command mode
    rnbd350_status_t exit_status = rnbd350_exit_command_mode(handle);
    if (status == RNBD350_OK) {
        status = exit_status;
    }

    return status;
}

rnbd350_status_t rnbd350_send_data(rnbd350_handle_t* handle, const uint8_t* data, uint16_t len)
{
    if (!handle || !data || !len) {
        return RNBD350_INVALID_PARAM;
    }

    // Ensure we're in data mode
    if (handle->mode != RNBD350_MODE_DATA) {
        return RNBD350_ERROR;
    }

    // Send data directly over UART
    handle->uart_write((uint8_t*)data, len);

    // Note: Since this is transparent UART mode, there's no explicit acknowledgment
    // Success is assumed unless the UART write fails internally
    return RNBD350_OK;
}

rnbd350_status_t rnbd350_receive_data(rnbd350_handle_t* handle, uint8_t* data, uint16_t len, uint16_t* received)
{
    if (!handle || !data || !len || !received) {
        return RNBD350_INVALID_PARAM;
    }

    // Ensure we're in data mode
    if (handle->mode != RNBD350_MODE_DATA) {
        return RNBD350_ERROR;
    }

    // Try to read data
    *received = handle->uart_read(data, len);

    // If we got any data, consider it success
    return (*received > 0) ? RNBD350_OK : RNBD350_TIMEOUT;
}


rnbd350_status_t rnbd350_set_serialized_name(rnbd350_handle_t* handle, const char* name)
{
    if (!handle || !name) {
        return RNBD350_INVALID_PARAM;
    }

    // Check name length
    if (strlen(name) > RNBD350_MAX_SERIAL_NAME_LEN) {
        return RNBD350_INVALID_PARAM;
    }

    char cmd[RNBD350_MAX_SERIAL_NAME_LEN + 5]; // "S-," + name + \r + \0
    char response[RNBD350_MAX_RESPONSE_SIZE];
    rnbd350_status_t status;

    // Enter command mode
    status = rnbd350_enter_command_mode(handle);
    if (status != RNBD350_OK) {
        return status;
    }

    // Build command string using snprintf
    if (snprintf(cmd, sizeof(cmd), "S-,%s\r", name) >= sizeof(cmd)) {
        return RNBD350_ERROR; // Buffer would have overflowed
    }

    status = send_command(handle, cmd, response);
    if (status != RNBD350_OK) {
        return status;
    }

    return rnbd350_exit_command_mode(handle);
}

rnbd350_status_t rnbd350_set_name(rnbd350_handle_t* handle, const char* name)
{
    if (!handle || !name) {
        return RNBD350_INVALID_PARAM;
    }

    // Check name length
    if (strlen(name) > RNBD350_MAX_NAME_LEN) {
        return RNBD350_INVALID_PARAM;
    }

    char cmd[RNBD350_MAX_NAME_LEN + 5]; // "SN," + name + \r + \0
    char response[RNBD350_MAX_RESPONSE_SIZE];
    rnbd350_status_t status;

    // Enter command mode
    status = rnbd350_enter_command_mode(handle);
    if (status != RNBD350_OK) {
        return status;
    }

    if (snprintf(cmd, sizeof(cmd), "SN,%s\r", name) >= sizeof(cmd)) {
        return RNBD350_ERROR; // Buffer would have overflowed
    }

    status = send_command(handle, cmd, response);
    if (status != RNBD350_OK) {
        return status;
    }

    return rnbd350_exit_command_mode(handle);
}
