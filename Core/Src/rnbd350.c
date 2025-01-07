#include "rnbd350.h"

// Private function prototypes
static rnbd_status_t RNBD_WaitResponse(rnbd_handle_t *handle, char *response, uint32_t timeout);
static bool RNBD_ValidateResponse(const char *response);
static rnbd_status_t RNBD_StartReceive(rnbd_handle_t *handle);

/**
 * @brief Initialize RNBD module

rnbd_status_t RNBD_Init(rnbd_handle_t *handle) {
    if (!handle) {
        return RNBD_ERROR;
    }

    LOG_DEBUG("RNBD Handle Initialised");
    // Initialize handle structure
    handle->rx_index = 0;
    handle->tx_index = 0;
    handle->tx_status = UART_IDLE;
    handle->rx_status = UART_IDLE;
    handle->state = RNBD_STATE_RESET;
    handle->status_callback = NULL;
    handle->delay_ms = HAL_Delay;

    // Reset module
    RNBD_Reset(handle);

    handle->delay_ms(100);
    // Start initial reception
    RNBD_StartReceive(handle);

    handle->delay_ms(5000);
    // Enter command mode
    if (RNBD_EnterCmdMode(handle) != RNBD_OK) {
        return RNBD_ERROR;
    }

    handle->state = RNBD_STATE_READY;
    return RNBD_OK;
}

/**
 * @brief Reset RNBD module
 */
void RNBD_Reset(rnbd_handle_t *handle) {
    // Assert reset
	handle->gpio.reset_port->ODR &= ~handle->gpio.reset_pin;
    handle->delay_ms(RNBD_RESET_DELAY_MS);

    // Release reset
    handle->gpio.reset_port->ODR |= handle->gpio.reset_pin;
    handle->delay_ms(RNBD_STARTUP_DELAY_MS);

    // Assert reset
	handle->gpio.wakeup_port->ODR &= ~handle->gpio.wakeup_pin;
	handle->delay_ms(RNBD_RESET_DELAY_MS);
	handle->delay_ms(5);

    // Clear buffers and flags
    memset(handle->rx_buffer, 0, RNBD_BUFFER_SIZE);
    memset(handle->tx_buffer, 0, RNBD_BUFFER_SIZE);
    handle->rx_index = 0;
    handle->tx_index = 0;
    handle->tx_status = UART_IDLE;
    handle->rx_status = UART_IDLE;
    handle->state = RNBD_STATE_RESET;
}

/**
 * @brief Enter command mode
 *
rnbd_status_t RNBD_EnterCmdMode(rnbd_handle_t *handle) {
    char response[RNBD_BUFFER_SIZE];
    const char cmd[] = "$$$\r\n";

    if (handle->state == RNBD_STATE_CMD_MODE) {
        return RNBD_OK;
    }

	handle->tx_status = UART_BUSY;
	if (HAL_UART_Transmit_IT(handle->huart, (uint8_t*)cmd, 3) != HAL_OK) {
		LOG_ERROR("UART ERROR");
		return RNBD_ERROR;
	}

	// Wait for transmission complete
	uint32_t start = HAL_GetTick();
	while (handle->tx_status == UART_BUSY) {
		if (HAL_GetTick() - start > RNBD_UART_TIMEOUT) {
			LOG_WARNING("UART TIMEOUT");
			return RNBD_TIMEOUT;
		}
	}


	if (HAL_UART_Receive_IT(handle->huart, &handle->rx_buffer[handle->rx_index], 1) != HAL_OK) {
		LOG_ERROR("Failed to start UART reception");
		handle->rx_status = UART_ERROR;
		return RNBD_ERROR;
	}

	// Wait for CMD> prompt
	if (RNBD_WaitResponse(handle, response, RNBD_UART_TIMEOUT) == RNBD_OK) {
		if (response[0] == 'C') {
			handle->state = RNBD_STATE_CMD_MODE;
			LOG_DEBUG("CMD MODE Enabled");
			return RNBD_OK;
		} else {
			LOG_ERROR("BAD_RESP");
		}
	} else {
		LOG_ERROR("ACK_ERROR");
	}

    return RNBD_ERROR;
}

/**
 * @brief Exit command mode
 */
rnbd_status_t RNBD_ExitCmdMode(rnbd_handle_t *handle) {
    char response[RNBD_BUFFER_SIZE];
    const char cmd[] = "---\r\n";

    if (handle->state != RNBD_STATE_CMD_MODE) {
        return RNBD_ERROR;
    }

    handle->tx_status = UART_BUSY;
    if (HAL_UART_Transmit_IT(handle->huart, (uint8_t*)cmd, 5) != HAL_OK) {
        return RNBD_ERROR;
    }

    // Wait for transmission complete
    uint32_t start = HAL_GetTick();
    while (handle->tx_status == UART_BUSY) {
        if (HAL_GetTick() - start > RNBD_UART_TIMEOUT) {
            return RNBD_TIMEOUT;
        }
    }

    if (RNBD_WaitResponse(handle, response, RNBD_UART_TIMEOUT) != RNBD_OK) {
        return RNBD_ERROR;
    }

    if (strstr(response, "END") == NULL) {
        return RNBD_ERROR;
    }

    handle->state = RNBD_STATE_DATA_MODE;
    return RNBD_OK;
}

/**
 * @brief Send command to RNBD module
 */
rnbd_status_t RNBD_SendCommand(rnbd_handle_t *handle, const char *cmd, char *response) {
    char cmd_buf[RNBD_MAX_CMD_SIZE];
    uint16_t cmd_len;

    if (!handle || !cmd) {
        return RNBD_ERROR;
    }

    // Ensure in command mode
    if (handle->state != RNBD_STATE_CMD_MODE) {
        if (RNBD_EnterCmdMode(handle) != RNBD_OK) {
            return RNBD_ERROR;
        }
    }

    // Format command with CR+LF
    cmd_len = snprintf(cmd_buf, sizeof(cmd_buf), "%s\r\n", cmd);
    if (cmd_len >= sizeof(cmd_buf)) {
        return RNBD_ERROR;
    }

    // Send command
    handle->tx_status = UART_BUSY;
    if (HAL_UART_Transmit_IT(handle->huart, (uint8_t*)cmd_buf, cmd_len) != HAL_OK) {
        return RNBD_ERROR;
    }

    // Wait for transmission complete
    uint32_t start = HAL_GetTick();
    while (handle->tx_status == UART_BUSY) {
        if (HAL_GetTick() - start > RNBD_UART_TIMEOUT) {
            return RNBD_TIMEOUT;
        }
    }

    // Wait for response if buffer provided
    if (response) {
        return RNBD_WaitResponse(handle, response, RNBD_UART_TIMEOUT);
    }

    return RNBD_OK;
}

/**
 * @brief Configure RNBD device name
 */
rnbd_status_t RNBD_SetDeviceName(rnbd_handle_t *handle, const char *name) {
    char cmd[RNBD_MAX_CMD_SIZE];
    char response[RNBD_BUFFER_SIZE];

    snprintf(cmd, sizeof(cmd), "SN,%s", name);
    return RNBD_SendCommand(handle, cmd, response);
}

/**
 * @brief Configure RNBD services
 */
rnbd_status_t RNBD_SetServices(rnbd_handle_t *handle, uint8_t services) {
    char cmd[RNBD_MAX_CMD_SIZE];
    char response[RNBD_BUFFER_SIZE];

    snprintf(cmd, sizeof(cmd), "SS,%02X", services);
    return RNBD_SendCommand(handle, cmd, response);
}

/**
 * @brief Start advertising
 */
rnbd_status_t RNBD_StartAdvertising(rnbd_handle_t *handle) {
    char response[RNBD_BUFFER_SIZE];
    return RNBD_SendCommand(handle, "A", response);
}

/**
 * @brief Stop advertising
 */
rnbd_status_t RNBD_StopAdvertising(rnbd_handle_t *handle) {
    char response[RNBD_BUFFER_SIZE];
    return RNBD_SendCommand(handle, "Y", response);
}

/**
 * @brief Send data in transparent mode
 */
rnbd_status_t RNBD_SendData(rnbd_handle_t *handle, const uint8_t *data, uint16_t len) {
    if (!handle || !data || !len || len > RNBD_BUFFER_SIZE) {
        return RNBD_ERROR;
    }

    if (handle->state != RNBD_STATE_DATA_MODE) {
        return RNBD_ERROR;
    }

    if (handle->tx_status == UART_BUSY) {
        return RNBD_BUSY;
    }

    // Copy data to transmit buffer
    memcpy(handle->tx_buffer, data, len);
    handle->tx_status = UART_BUSY;

    if (HAL_UART_Transmit_IT(handle->huart, handle->tx_buffer, len) != HAL_OK) {
        handle->tx_status = UART_ERROR;
        return RNBD_ERROR;
    }

    return RNBD_OK;
}

/**
 * @brief Register status callback
 */
void RNBD_RegisterStatusCallback(rnbd_handle_t *handle, void (*callback)(char*)) {
    if (handle) {
        handle->status_callback = callback;
    }
}

/**
 * @brief Start receive operation
 */
static rnbd_status_t RNBD_StartReceive(rnbd_handle_t *handle) {
    uint32_t start_time = HAL_GetTick();;

    if (!handle) {
        return RNBD_ERROR;
    }

    LOG_DEBUG("UART STATE: %d", handle->rx_status );
    while (HAL_GetTick() - start_time < 1000){
		if (handle->rx_status != UART_BUSY) {
			LOG_DEBUG("Starting new UART reception");
			handle->rx_status = UART_BUSY;

			// Start UART reception
			if (HAL_UART_Receive_IT(handle->huart, &handle->rx_buffer[handle->rx_index], 1) != HAL_OK) {
					LOG_ERROR("Failed to start UART reception");
					handle->rx_status = UART_ERROR;
					return RNBD_ERROR;
			}

			LOG_DEBUG("Buffer Content:");
			for (uint16_t i = 0; i < RNBD_BUFFER_SIZE; i++) {
				LOG_DEBUG("0x%02X ", handle->rx_buffer[i]);
			}

			LOG_DEBUG("UART reception started successfully");
			return RNBD_OK;
    	}
    }

    LOG_DEBUG("RX TIMEOUT");
    return RNBD_TIMEOUT;
}
/**
 * @brief UART TX complete callback
 */
void RNBD_UART_TxCpltCallback(rnbd_handle_t *handle) {
	LOG_DEBUG("TX Complete Callback");
    handle->tx_status = UART_COMPLETE;
    LOG_DEBUG("UART TX state: %d",handle->tx_status = UART_COMPLETE);
}

/**
 * @brief UART RX complete callback
 */
void RNBD_UART_RxCpltCallback(rnbd_handle_t *handle) {
    static struct {
        bool in_status_msg;
        char status_buffer[RNBD_BUFFER_SIZE];
        uint16_t status_index;
    } status = {false, {0}, 0};

    if (!handle) {
        return;
    }

    LOG_DEBUG("RX Complete Callback");
    // Get the received byte
    uint8_t rx_byte = handle->rx_buffer[handle->rx_index];

    // Process received byte
    if (status.in_status_msg) {
        // Currently collecting status message
        status.status_buffer[status.status_index++] = rx_byte;

        if (rx_byte == '%') {  // End of status message
            status.status_buffer[status.status_index] = '\0';
            if (handle->status_callback) {
                handle->status_callback(status.status_buffer);
            }
            status.in_status_msg = false;
            status.status_index = 0;
        } else if (status.status_index >= RNBD_BUFFER_SIZE - 1) {
            // Status message too long, reset
            status.in_status_msg = false;
            status.status_index = 0;
        }
    } else {
        // Check for start of status message
        if (rx_byte == '%') {
            status.in_status_msg = true;
            status.status_buffer[0] = rx_byte;
            status.status_index = 1;
        } else {
            // Normal data byte
            handle->rx_buffer[handle->rx_index++] = rx_byte;
            if (handle->rx_index >= RNBD_BUFFER_SIZE) {
                handle->rx_index = 0;  // Buffer wrap-around
            }
        }
    }

    // Update status and start next reception
    handle->rx_status = UART_IDLE;
    LOG_DEBUG("RX CPLT Status: %d",handle->rx_status);
    if (RNBD_StartReceive(handle) != RNBD_OK) {
        handle->state = RNBD_STATE_ERROR;
    }
    handle->rx_status = UART_COMPLETE;
}

/**
 * @brief Wait for response with timeout
 */
static rnbd_status_t RNBD_WaitResponse(rnbd_handle_t *handle, char *response, uint32_t timeout) {
    uint32_t start = HAL_GetTick();
    uint16_t index = 0;
    bool response_complete = false;
    LOG_DEBUG("Waiting for ACK");
    while (!response_complete && ((HAL_GetTick() - start) < timeout)) {
        if (handle->rx_status == UART_COMPLETE) {
        	LOG_DEBUG("PROCESSING DATA");
            // Copy received data until we find response termination
            while (index < RNBD_BUFFER_SIZE - 1) {
                if (handle->rx_index > 0) {
                    handle->rx_index--;
                    response[index] = handle->rx_buffer[handle->rx_index];
                    LOG_DEBUG("Idx[0]: %c", response[0]);
                    // Check for response termination
                    if (index >= 1)
                    {
                    	LOG_DEBUG("RESP[0]: %c", response[0]);
                        response_complete = true;
                        break;
                    }
                    index++;
                } else {
                    break;
                }
            }
            LOG_DEBUG("UART IDLE");
            handle->rx_status = UART_IDLE;
        }
    }

    if (!response_complete) {
    	LOG_DEBUG("TIMEOUT");
        return RNBD_TIMEOUT;
    }

    LOG_DEBUG("SUCCESS");
    return RNBD_ValidateResponse(response) ? RNBD_OK : RNBD_ERROR;
}

/**
 * @brief Validate response
 */
static bool RNBD_ValidateResponse(const char *response) {
    if (!response) {
        return false;
    }

    // Check for error responses
    if (strstr(response, "ERR") || strstr(response, "ERROR")) {
        return false;
    }

    // Check for success responses
    if (strstr(response, "OK") || strstr(response, "CMD>")) {
        return true;
    }

    // Consider valid if no error detected
    return true;
}
