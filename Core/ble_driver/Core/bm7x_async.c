#include <bm7x_async.h>
#include "logger.h"
#include <string.h>

/* ============================================================================
 * Private Constants and Macros
 * ============================================================================ */

#define CMD_RETRY_COUNT                 3       // Number of command retries
#define CMD_GUARD_TIMEOUT               2000    // Command guard timeout (ms)
#define CHECKSUM_XOR_BASE              0x100    // Checksum calculation base

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */

static bm70_error_t send_command(bm70_handle_t *handle, uint8_t opcode,
                                const uint8_t *params, uint16_t len);
static bm70_error_t wait_response(bm70_handle_t *handle, uint8_t expected_opcode,
                                 uint32_t timeout);

bm70_error_t bm70_read_local_service_characteristics(bm70_handle_t* handle, uint16_t service_uuid);
static void process_rx_byte(bm70_handle_t *handle, uint8_t byte);
static void process_event(bm70_handle_t *handle, uint8_t opcode,
                         const uint8_t *data, uint16_t len);
static bool verify_packet_checksum(const uint8_t *packet, uint16_t len);

static void process_local_info_response(bm70_handle_t *handle);
static void process_device_name_response(bm70_handle_t *handle);
static void process_pairing_mode_response(bm70_handle_t *handle);
static void parse_service_discovery_response(bm70_handle_t* handle, const uint8_t* data, uint16_t len);
static void parse_characteristic_discovery_response(bm70_handle_t* handle, const uint8_t* data, uint16_t len);

static const char* get_command_error_string(uint8_t error_code);
static const char* get_hardware_version_string(uint8_t hw_version);
static bool is_valid_connection_handle(bm70_handle_t *handle);

extern uint32_t g_last_status_event_time;
extern volatile uint8_t ble_conn_flag;
/* ============================================================================
 * Core Communication Functions
 * ============================================================================ */

static bm70_error_t send_command(bm70_handle_t *handle, uint8_t opcode,
                                const uint8_t *params, uint16_t param_len)
{
    if (!handle || !handle->uart.write) {
        return BM70_ERROR_UART;
    }

    // Calculate total length (OPCODE + PARAMS)
    uint16_t total_len = 1 + param_len;
    uint16_t packet_size = 4 + param_len + 1; // SYNC + LEN(2) + OPCODE + PARAMS + CHECKSUM

    if (packet_size > BM70_MAX_PACKET_SIZE) {
        return BM70_ERROR_INVALID_LENGTH;
    }

    // Build packet
    uint8_t packet[BM70_MAX_PACKET_SIZE];
    packet[0] = BM70_SYNC_WORD;           // SYNC (0xAA)
    packet[1] = (total_len >> 8) & 0xFF;  // LEN_MSB
    packet[2] = total_len & 0xFF;         // LEN_LSB
    packet[3] = opcode;                   // OPCODE

    // Add parameters
    if (params && param_len > 0) {
        memcpy(&packet[4], params, param_len);
    }

    // Calculate checksum
    uint16_t sum = 0;
    for (uint16_t i = 1; i < 4 + param_len; i++) {
        sum += packet[i];
    }
    packet[4 + param_len] = (uint8_t)(CHECKSUM_XOR_BASE - (sum & 0xFF));

    // Send packet
    handle->uart.write(packet, packet_size);

    LOG_DEBUG("CMD: 0x%02X, Len: %d", opcode, param_len);

    return BM70_OK;
}

static bm70_error_t wait_response(bm70_handle_t *handle, uint8_t expected_opcode, uint32_t timeout)
{
    if (!handle) {
        return BM70_ERROR_INVALID_PARAM;
    }

    uint32_t start = HAL_GetTick();
    handle->response_ready = false;
    handle->response_opcode = expected_opcode;

    while (!handle->response_ready) {
        if ((HAL_GetTick() - start) > timeout) {
            LOG_WARNING("Timeout waiting for event 0x%02X", expected_opcode);
            return BM70_ERROR_TIMEOUT;
        }

        bm70_process_rx(handle);
        HAL_Delay(1);
    }

    // Check command complete status if applicable
    if (expected_opcode == BM70_EVT_COMMAND_COMPLETE &&
        handle->response_len > 1 &&
        handle->response_buffer[1] != 0x00) {

        LOG_ERROR("Command 0x%02X failed with status 0x%02X (%s)",
                 handle->response_buffer[0],
                 handle->response_buffer[1],
                 get_command_error_string(handle->response_buffer[1]));
        return BM70_ERROR_COMMAND_FAILED;
    }

    return BM70_OK;
}

/* ============================================================================
 * Packet Processing Functions
 * ============================================================================ */

static void process_rx_byte(bm70_handle_t *handle, uint8_t byte)
{
    uint8_t *pkt = handle->packet_buffer;
    uint16_t *pidx = &handle->packet_idx;

    // Handle mid-stream SYNC byte
    if (byte == BM70_SYNC_WORD && handle->rx_state != RX_STATE_WAIT_START) {
        handle->rx_state = RX_STATE_WAIT_LEN_H;
        *pidx = 0;
        pkt[(*pidx)++] = byte;
        return;
    }

    switch (handle->rx_state) {
        case RX_STATE_WAIT_START:
            if (byte == BM70_SYNC_WORD) {
                pkt[(*pidx)++] = byte;
                handle->rx_state = RX_STATE_WAIT_LEN_H;
            }
            break;

        case RX_STATE_WAIT_LEN_H:
            pkt[(*pidx)++] = byte;
            handle->rx_expected_len = (uint16_t)byte << 8;
            handle->rx_state = RX_STATE_WAIT_LEN_L;
            break;

        case RX_STATE_WAIT_LEN_L:
            pkt[(*pidx)++] = byte;
            handle->rx_expected_len |= byte;

            if (4U + handle->rx_expected_len > BM70_MAX_PACKET_SIZE) {
                LOG_WARNING("Invalid packet length %u", handle->rx_expected_len);
                handle->rx_state = RX_STATE_WAIT_START;
                *pidx = 0;
            } else {
                handle->rx_index = 0;
                handle->rx_state = RX_STATE_WAIT_OPCODE;
            }
            break;

        case RX_STATE_WAIT_OPCODE:
            pkt[(*pidx)++] = byte;
            handle->rx_buffer[handle->rx_index++] = byte;
            handle->rx_state = (handle->rx_expected_len > 1) ? RX_STATE_WAIT_DATA : RX_STATE_WAIT_CHECKSUM;
            break;

        case RX_STATE_WAIT_DATA:
            pkt[(*pidx)++] = byte;
            handle->rx_buffer[handle->rx_index++] = byte;
            if (handle->rx_index >= handle->rx_expected_len) {
                handle->rx_state = RX_STATE_WAIT_CHECKSUM;
            }
            break;

        case RX_STATE_WAIT_CHECKSUM:
            pkt[(*pidx)++] = byte;

            if (verify_packet_checksum(pkt, *pidx)) {
                uint8_t opcode = handle->rx_buffer[0];
                uint8_t *data = &handle->rx_buffer[1];
                uint16_t data_len = handle->rx_expected_len - 1;
                process_event(handle, opcode, data, data_len);
            } else {
                LOG_WARNING("Packet checksum verification failed");
                if (handle->error_cb) {
                    handle->error_cb(BM70_ERROR_CHECKSUM);
                }
            }

            handle->rx_state = RX_STATE_WAIT_START;
            *pidx = 0;
            break;

        default:
            handle->rx_state = RX_STATE_WAIT_START;
            *pidx = 0;
            break;
    }
}

static void process_event(bm70_handle_t *handle, uint8_t opcode,
                         const uint8_t *data, uint16_t len)
{
    LOG_DEBUG("RX Event: 0x%02X, len=%d", opcode, len);

    // Store response data if this is what we're waiting for
    if (opcode == handle->response_opcode ||
        (handle->response_opcode == BM70_EVT_COMMAND_COMPLETE && opcode == BM70_EVT_COMMAND_COMPLETE)) {

        if (len <= BM70_MAX_PACKET_SIZE) {
            memcpy(handle->response_buffer, data, len);
            handle->response_len = len;
            handle->response_ready = true;
        }
    }

    // Process asynchronous events
    switch (opcode) {
        case BM70_EVT_STATUS_REPORT: // 0x81
            g_last_status_event_time = HAL_GetTick();

            if (len > 0) {
                bm70_status_t old_status = handle->status;
                handle->status = (bm70_status_t)data[0];

                if (len <= BM70_MAX_PACKET_SIZE) {
                    memcpy(handle->response_buffer, data, len);
                    handle->response_len = len;
                }

                // Log status transitions for debugging
                if (old_status != handle->status) {
                    LOG_INFO("Status transition: %s -> %s",
                            bm70_status_to_string(old_status),
                            bm70_status_to_string(handle->status));
                }

                switch (handle->status) {
                    case BM70_STATUS_STANDBY:
                        // Only update to advertising if we're not already connected
                        if (handle->conn_state == BM70_CONN_STATE_DISCONNECTED) {
                            handle->conn_state = BM70_CONN_STATE_ADVERTISING;
                            ble_conn_flag = false;
                            LOG_INFO("Module entered advertising mode");
                        }
                        break;

                    case BM70_STATUS_CONNECTED:
                        // Module reports connected status
                        if (handle->conn_state != BM70_CONN_STATE_CONNECTED) {
                            handle->conn_state = BM70_CONN_STATE_CONNECTED;
                            ble_conn_flag = true;
                            LOG_INFO("Module status: connected");
                        }
                        break;

                    case BM70_STATUS_IDLE:
                        if (handle->conn_state == BM70_CONN_STATE_CONNECTED) {
                            LOG_DEBUG("Module status: idle (connection still active, handle=0x%02X)",
                                     handle->conn_info.handle);
                        } else if (handle->conn_state == BM70_CONN_STATE_DISCONNECTED) {
                            LOG_DEBUG("Module status: idle (no active connection)");
                        } else {
                            // Only log the transition, don't force disconnect
                            LOG_DEBUG("Module status: idle (state=%s)",
                                     bm70_conn_state_to_string(handle->conn_state));
                        }
                        break;

                    default:
                        // Other statuses don't affect connection state
                        break;
                }

                if (handle->status_cb) {
                    handle->status_cb(handle->status);
                }
            }
            break;

        case BM70_EVT_LE_CONNECTION_COMPLETE:
			if (len >= 2) {
				uint8_t status = data[0];

				if (status == 0x00) {
					// Connection successful - immediately set connected state
					handle->conn_state = BM70_CONN_STATE_CONNECTED;
					ble_conn_flag = true;

					// Parse connection info according to BM70 documentation
					if (len >= 16) { // Minimum length for full event
						handle->conn_info.handle = data[1];                    // Connection Handle (1 byte)
						handle->conn_info.role = data[2];                      // Role (1 byte)
						handle->conn_info.peer_addr_type = data[3];            // Peer Address Type (1 byte)
						memcpy(handle->conn_info.peer_addr, &data[4], 6);      // Peer Address (6 bytes)

						// Parse connection parameters (2 bytes each, little endian)
						handle->conn_info.conn_interval = data[10] | (data[11] << 8);      // Conn_Interval
						handle->conn_info.slave_latency = data[12] | (data[13] << 8);      // Conn_Latency
						handle->conn_info.supervision_timeout = data[14] | (data[15] << 8); // Supervision_Timeout
					}

					LOG_INFO("BLE Connected: Handle=0x%02X, Role=%s",
							handle->conn_info.handle,
							handle->conn_info.role ? "client" : "server");

					LOG_INFO("Peer: %02X:%02X:%02X:%02X:%02X:%02X (Type=%d)",
							handle->conn_info.peer_addr[0], handle->conn_info.peer_addr[1],
							handle->conn_info.peer_addr[2], handle->conn_info.peer_addr[3],
							handle->conn_info.peer_addr[4], handle->conn_info.peer_addr[5],
							handle->conn_info.peer_addr_type);

					if (handle->conn_cb) {
						handle->conn_cb(BM70_CONN_STATE_CONNECTED, &handle->conn_info);
					}
				} else {
					LOG_WARNING("Connection failed with status: 0x%02X", status);
					handle->conn_state = BM70_CONN_STATE_DISCONNECTED;
					ble_conn_flag = false;

					// Auto-restart advertising on connection failure
					if (handle->config.auto_reconnect) {
						LOG_INFO("Auto-restarting advertising after connection failure...");
						HAL_Delay(100);
						bm70_start_advertising(handle);
					}
				}
			}
            break;

        case BM70_EVT_DISCONNECT_COMPLETE:
            LOG_INFO("BLE Disconnected");
            // Clear connection state completely on disconnect
            handle->conn_state = BM70_CONN_STATE_DISCONNECTED;
            ble_conn_flag = false;
            memset(&handle->conn_info, 0, sizeof(handle->conn_info));
            handle->hid_state.service_ready = false;
            handle->hid_state.suspended = false;
            handle->hid_state.notifications_enabled = false;
            handle->hid_state.waiting_for_conn_update = false;

            if (handle->conn_cb) {
                handle->conn_cb(BM70_CONN_STATE_DISCONNECTED, NULL);
            }

            if (handle->device_manager.switch_device) {
				handle->device_manager.switch_device = false;
				HAL_Delay(100);
				bm70_start_directed_advertising(handle);  // Uses directed advertising parameters
				break;
			}
			else if (handle->config.auto_reconnect) {
				// Normal auto-reconnect (uses current advertising parameters)
				LOG_INFO("Auto-reconnect enabled, restarting advertising...");
				HAL_Delay(100);
				bm70_start_advertising(handle);
			}
            break;

        case BM70_EVT_PAIRING_COMPLETE:
            if (len >= 2) {
                uint8_t conn_handle = data[0];
                uint8_t result = data[1];
                if (result == 0x00) {
                    LOG_INFO("Pairing completed successfully (handle=0x%02X)", conn_handle);
                } else {
                    LOG_WARNING("Pairing failed: result=0x%02X", result);
                }
                if (handle->pairing_cb) {
                    handle->pairing_cb(BM70_EVT_PAIRING_COMPLETE, data, len);
                }
            }
            break;

        case BM70_EVT_PASSKEY_ENTRY_REQ:
            if (len >= 1) {
                uint8_t conn_handle = data[0];
                LOG_INFO("Passkey entry request (handle=0x%02X)", conn_handle);
                if (handle->pairing_cb) {
                    handle->pairing_cb(BM70_EVT_PASSKEY_ENTRY_REQ, data, len);
                }
            }
            break;

        case BM70_EVT_PASSKEY_CONFIRM_REQ:
            if (len >= 2) {
                uint8_t conn_handle = data[0];
                uint8_t passkey = data[1];
                LOG_INFO("Passkey confirmation request (handle=0x%02X, passkey=%d)",
                        conn_handle, passkey);
                if (handle->pairing_cb) {
                    handle->pairing_cb(BM70_EVT_PASSKEY_CONFIRM_REQ, data, len);
                }
            }
            break;

        case BM70_EVT_CONNECTION_UPDATE:
            if (len >= 7) {
                handle->conn_info.conn_interval = (data[2] << 8) | data[1];
                handle->conn_info.slave_latency = (data[4] << 8) | data[3];
                handle->conn_info.supervision_timeout = (data[6] << 8) | data[5];

                handle->hid_state.waiting_for_conn_update = false;

                LOG_INFO("Connection parameters updated: Interval=%d, Latency=%d, Timeout=%d",
                        handle->conn_info.conn_interval,
                        handle->conn_info.slave_latency,
                        handle->conn_info.supervision_timeout);
            }
            break;

        case BM70_EVT_ATT_MTU_UPDATE:
            if (len >= 3) {
                uint8_t conn_handle = data[0];
                uint16_t mtu_size = data[2] | (data[1] << 8);

                // ATT MTU update confirms active connection
                handle->hid_state.mtu_size = mtu_size;
                handle->conn_info.handle = conn_handle;
                handle->conn_state = BM70_CONN_STATE_CONNECTED;
                ble_conn_flag = true;  // Force connection flag

                LOG_INFO("ATT MTU updated: Handle=0x%02X, MTU=%d", conn_handle, mtu_size);

                // Mark HID as ready
                handle->hid_state.service_ready = true;
                handle->hid_state.notifications_enabled = true;
                handle->hid_state.mtu_received_time = HAL_GetTick();


                if (handle->conn_cb) {
                    handle->conn_cb(BM70_CONN_STATE_CONNECTED, &handle->conn_info);
                }
            }
            break;

        case BM70_EVT_TRANSPARENT_DATA:
            LOG_DEBUG("Transparent data received, len=%d", len);
            if (handle->data_cb) {
                handle->data_cb(data, len);
            }
            break;

        case BM70_EVT_DISCOVER_ALL_SERVICES_RSP: // 0x90
            LOG_DUMP("\t Service Response - #%d\n",
                     handle->service_discovery_active ? handle->service_responses_received + 1 : 0);

            if (handle->service_discovery_active) {
                handle->service_responses_received++;
                if (len > 0) {
                    parse_service_discovery_response(handle, data, len);
                }
            }

            if (handle->response_opcode == BM70_EVT_DISCOVER_ALL_SERVICES_RSP) {
                memcpy(handle->response_buffer, data, len);
                handle->response_len = len;
                handle->response_ready = true;
            }
            break;

        case BM70_EVT_DISCOVER_CHAR_RSP: // 0x91
           LOG_DEBUG("=== CHARACTERISTIC DISCOVERY RESPONSE (0x91) ===");
           LOG_DEBUG("Length: %d bytes", len);
           if (len > 0) {
               parse_characteristic_discovery_response(handle, data, len);
           }
           if (handle->response_opcode == BM70_EVT_DISCOVER_CHAR_RSP) {
               memcpy(handle->response_buffer, data, len);
               handle->response_len = len;
               handle->response_ready = true;
           }
           break;

        case BM70_EVT_DISCOVER_DESC_RSP: // 0x92
           LOG_DEBUG("=== DESCRIPTOR DISCOVERY RESPONSE (0x92) ===");
           LOG_DEBUG("Length: %d bytes", len);
           LOG_DUMP("-> Raw data: ");
           for (uint16_t i = 0; i < len && i < 32; i++) {
               LOG_DUMP("%02X ", data[i]);
           }
           LOG_DUMP("\n");
           break;

        case BM70_EVT_COMMAND_COMPLETE:
            if (handle->service_discovery_active && len >= 2 &&
                data[0] == BM70_CMD_READ_LOCAL_ALL_SERVICES) {
                handle->discovery_command_complete = true;
                LOG_INFO("Service discovery command complete received");
            }
            break;

        case BM70_EVT_CLIENT_WRITE_CHAR: // 0x98
            if (len >= 3) {
                uint8_t conn_handle = data[0];
                uint16_t char_handle = (data[1] << 8) | data[2];

                // Client writing confirms active connection
                ble_conn_flag = true;
                handle->conn_state = BM70_CONN_STATE_CONNECTED;
                handle->hid_state.waiting_for_conn_update = true;

                LOG_INFO("Client wrote to characteristic 0x%04X on connection 0x%02X",
                         char_handle, conn_handle);

                if (len > 3) {
                    bm70_handle_cccd_request(handle, char_handle, &data[3], len - 3);
                }
            }
            break;

        default:
            LOG_DEBUG("Unhandled event: 0x%02X (len=%d)", opcode, len);
            break;
    }
}

static bool verify_packet_checksum(const uint8_t *packet, uint16_t len)
{
    if (len < 5) return false;

    uint16_t sum = 0;
    for (uint16_t i = 1; i < len; i++) {
        sum += packet[i];
    }
    return (sum & 0xFF) == 0;
}

/* ============================================================================
 * Service Discovery Functions
 * ============================================================================ */

static void parse_service_discovery_response(bm70_handle_t* handle, const uint8_t* data, uint16_t len)
{
    if (len < 3) {
        LOG_WARNING("Response too short");
        return;
    }
    HAL_Delay(1);
    // Your BM71 puts the real attr_len at byte 1, not byte 0
    uint8_t attr_len = data[1];
    //LOG_INFO("Parsing: attr_len=%d, total_len=%d", attr_len, len);

    if (attr_len != 6 && attr_len != 20) {
        LOG_WARNING("Unexpected attr_len: %d", attr_len);
        return;
    }

    // Start parsing from byte 2 (skip the two length bytes)
    uint16_t offset = 2;
    int services_parsed = 0;

    while (offset + attr_len <= len && handle->num_services < BM70_MAX_SERVICES) {
        uint16_t start_handle = data[offset] | (data[offset + 1] << 8);
        uint16_t end_handle = data[offset + 2] | (data[offset + 3] << 8);
        uint16_t service_uuid = data[offset + 4] | (data[offset + 5] << 8);

        LOG_INFO("Service: UUID=0x%04X, Start=0x%04X, End=0x%04X",
                service_uuid, start_handle, end_handle);
        HAL_Delay(1);
        if (start_handle > 0 && end_handle > 0 && start_handle <= end_handle) {

            services_parsed++;
        }

        offset += attr_len;
    }

    LOG_INFO("Stored %d services from this response", services_parsed);
    HAL_Delay(1);
}

static void parse_characteristic_discovery_response(bm70_handle_t* handle, const uint8_t* data, uint16_t len)
{
   LOG_INFO("=== RAW CHARACTERISTIC DATA ===");
   // Show as one continuous hex string too
   LOG_DUMP("RESP: ");
   for (uint16_t i = 0; i < len; i++) {
       LOG_DUMP("%02X ", data[i]);
       HAL_Delay(1);
   }
   LOG_DUMP("\r\n");
   // Don't parse or store anything yet
}

/* ============================================================================
 * Core API Implementation
 * ============================================================================ */

void bm70_hwreset_with_handle(bm70_handle_t* handle)
{
    if (!handle || !handle->config.rst_port) {
        LOG_WARNING("Cannot perform hardware reset - invalid configuration");
        return;
    }

    HAL_GPIO_WritePin(handle->config.rst_port, handle->config.rst_pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(handle->config.rst_port, handle->config.rst_pin, GPIO_PIN_SET);
    HAL_Delay(500);

    LOG_INFO("BM70/71 hardware reset completed");
}

bm70_error_t bm70_init_with_uart(bm70_handle_t* handle,
                                 const bm70_config_t* config,
                                 const bm70_uart_interface_t* uart)
{
    if (!handle || !config || !uart) {
        LOG_ERROR("Invalid parameters for initialization");
        return BM70_ERROR_INVALID_PARAM;
    }

    memset(handle, 0, sizeof(*handle));
    memcpy(&handle->config, config, sizeof(handle->config));
    memcpy(&handle->uart, uart, sizeof(handle->uart));

    handle->rx_state = RX_STATE_WAIT_START;
    handle->conn_state = BM70_CONN_STATE_DISCONNECTED;
    handle->status = BM70_STATUS_IDLE;
    handle->hid_state.protocol_mode = HID_PROTOCOL_REPORT;
    handle->hid_state.mtu_size = 23;
    handle->hid_state.waiting_for_conn_update = false;
	handle->hid_state.mtu_received_time = 0;
	handle->device_manager.current_adv_mode = 0;
    LOG_INFO("Initializing BM70/71 HID Module");

    bm70_hwreset_with_handle(handle);
    handle->uart.flush_rxbuffer();


    for (int i = 0; i < 10; i++) {
        bm70_process_rx(handle);
        HAL_Delay(20);
    }

    bm70_verify_configuration(handle);

    handle->initialized = true;

    LOG_INFO("BM70/71 initialization completed successfully");
    LOG_INFO("Current Status: 0x%02X (%s)", handle->status, bm70_status_to_string(handle->status));
    LOG_INFO("Connection State: %s", bm70_conn_state_to_string(handle->conn_state));

    return BM70_OK;
}

bm70_error_t bm70_deinit(bm70_handle_t* handle)
{
    if (!handle) {
        return BM70_ERROR_INVALID_PARAM;
    }

    if (handle->conn_state == BM70_CONN_STATE_CONNECTED) {
        bm70_disconnect(handle);
        HAL_Delay(100);
    }

    if (handle->conn_state == BM70_CONN_STATE_ADVERTISING) {
        bm70_stop_advertising(handle);
        HAL_Delay(100);
    }

    memset(handle, 0, sizeof(*handle));
    LOG_INFO("BM70/71 deinitialized");
    return BM70_OK;
}

void bm70_process_rx(bm70_handle_t* handle)
{
    if (!handle) return;

    if (!handle->uart.data_ready || !handle->uart.data_ready()) {
        return;
    }

    while (handle->uart.rx_not_empty && handle->uart.rx_not_empty()) {
        uint8_t byte = handle->uart.read();
        process_rx_byte(handle, byte);
    }

    if (handle->uart.flush_rxbuffer) {
        handle->uart.flush_rxbuffer();
    }
}

/* ============================================================================
 * GATT Service Discovery with Patient Waiting
 * ============================================================================ */

bm70_error_t bm70_read_all_local_services(bm70_handle_t* handle)
{
    if (!handle) {
        return BM70_ERROR_INVALID_PARAM;
    }

    LOG_INFO("========================================");
    LOG_INFO("Starting Patient GATT Service Discovery");
    LOG_INFO("========================================");

    // Clear existing services and reset state
    handle->num_services = 0;
    memset(handle->services, 0, sizeof(handle->services));

    handle->service_discovery_active = true;
    handle->discovery_command_complete = false;
    handle->service_responses_received = 0;
    handle->discovery_start_time = HAL_GetTick();

    bm70_error_t err = send_command(handle, BM70_CMD_READ_LOCAL_ALL_SERVICES, NULL, 0);
    if (err != BM70_OK) {
        LOG_ERROR("Failed to send service discovery command: %s", bm70_error_to_string(err));
        handle->service_discovery_active = false;
        return err;
    }

    LOG_INFO("Service discovery command sent, waiting for responses...");

    // Patient waiting - let process_event() handle the responses
    uint32_t total_timeout = 100;      // 15 second total timeout
    uint32_t response_timeout = 10;    // 3 seconds between responses
    uint32_t last_response_time = HAL_GetTick();

    while (handle->service_discovery_active &&
           (HAL_GetTick() - handle->discovery_start_time) < total_timeout) {

        // Process incoming data - this calls process_event()
        bm70_process_rx(handle);

        // Check if we got responses and update timing
        if (handle->service_responses_received > 0) {
            uint32_t current_time = HAL_GetTick();
            if (current_time - last_response_time < response_timeout) {
                last_response_time = current_time;
            } else {
                // No new response for a while
                if (handle->discovery_command_complete) {
                    LOG_INFO("Command complete received and response timeout - discovery done");
                    break;
                }

                if (handle->service_responses_received > 0 && !handle->discovery_command_complete) {
                    LOG_INFO("Got %d responses, still waiting for command complete...",
                             handle->service_responses_received);
                    // Continue waiting a bit more
                } else {
                    LOG_INFO("Response timeout reached, ending discovery");
                    break;
                }
            }
        }

        HAL_Delay(20);
    }

    // End discovery
    handle->service_discovery_active = false;
    uint32_t total_time = HAL_GetTick() - handle->discovery_start_time;

    LOG_INFO("========================================");
    LOG_INFO("Service Discovery Results");
    LOG_INFO("========================================");
    LOG_INFO("Total time: %lu ms", total_time);
    LOG_INFO("Service responses received: %d", handle->service_responses_received);
    LOG_INFO("Services discovered and stored: %d", handle->num_services);
    LOG_INFO("Command complete received: %s", handle->discovery_command_complete ? "Yes" : "No");

    // Print what we found
    if (handle->num_services > 0) {
        LOG_INFO("\n--- Discovered Services ---");
        for (uint8_t i = 0; i < handle->num_services; i++) {
            bm70_service_t* svc = &handle->services[i];
            LOG_INFO("Service %d: UUID=0x%04X, Handles=0x%04X-0x%04X",
                    i, svc->uuid, svc->start_handle, svc->end_handle);
        }
        handle->discovery_complete = true;
        handle->hid_state.service_ready = true;
        return BM70_OK;
    } else {
        LOG_WARNING("No services discovered - using fallback");
        handle->hid_state.service_ready = true;
        return BM70_OK;
    }
}



bm70_error_t bm70_read_local_service_characteristics(bm70_handle_t* handle, uint16_t service_uuid)
{
    if (!handle) {
        return BM70_ERROR_INVALID_PARAM;
    }

    HAL_Delay(1);

    LOG_INFO("Discovering characteristics for service 0x%04X...", service_uuid);
	uint8_t params[2];
	params[0] = (service_uuid >> 8) & 0xFF;
	params[1] = service_uuid & 0xFF;

	bm70_error_t err = send_command(handle, BM70_CMD_READ_LOCAL_SPECIFIC_SERVICE, params, 2);
	if (err != BM70_OK) {
		return err;
	}

	err = wait_response(handle, BM70_EVT_DISCOVER_CHAR_RSP, 200);
	if (err != BM70_OK) {
		LOG_INFO("Testing module responsiveness after 0x3C...");
		bm70_error_t test = bm70_read_local_information(handle);
		LOG_INFO("Module test result: %s", bm70_error_to_string(test));
		return err;
	}
	return err;
}

/* ============================================================================
 * Configuration and Status Functions
 * ============================================================================ */

bm70_error_t bm70_reset(bm70_handle_t* handle)
{
    if (!handle || !handle->initialized) {
        return BM70_ERROR_INVALID_PARAM;
    }

    LOG_INFO("Resetting BM70/71 module...");

    bm70_error_t err = send_command(handle, BM70_CMD_RESET, NULL, 0);
    if (err != BM70_OK) {
        return err;
    }

    return wait_response(handle, BM70_EVT_STATUS_REPORT, CMD_GUARD_TIMEOUT);
}

bm70_error_t bm70_get_status(bm70_handle_t* handle, bm70_status_t* status)
{
    if (!handle || !handle->initialized || !status) {
        return BM70_ERROR_INVALID_PARAM;
    }

    bm70_error_t err = send_command(handle, BM70_CMD_READ_STATUS, NULL, 0);
    if (err != BM70_OK) {
        return err;
    }

    err = wait_response(handle, BM70_EVT_STATUS_REPORT, CMD_GUARD_TIMEOUT);
    if (err == BM70_OK) {
        *status = handle->status;
    }

    return err;
}

bm70_error_t bm70_read_local_information(bm70_handle_t* handle)
{
    if (!handle) {
        return BM70_ERROR_INVALID_PARAM;
    }

    bm70_error_t err = send_command(handle, BM70_CMD_READ_LOCAL_INFO, NULL, 0);
    if (err != BM70_OK) {
        return err;
    }

    err = wait_response(handle, BM70_EVT_COMMAND_COMPLETE, CMD_GUARD_TIMEOUT);
    if (err == BM70_OK) {
        process_local_info_response(handle);
    }

    return err;
}

bm70_error_t bm70_read_device_name(bm70_handle_t* handle)
{
    if (!handle) {
        return BM70_ERROR_INVALID_PARAM;
    }

    bm70_error_t err = send_command(handle, BM70_CMD_READ_DEVICE_NAME, NULL, 0);
    if (err != BM70_OK) {
        return err;
    }

    err = wait_response(handle, BM70_EVT_COMMAND_COMPLETE, CMD_GUARD_TIMEOUT);
    if (err == BM70_OK) {
        process_device_name_response(handle);
    }

    return err;
}

bm70_error_t bm70_write_device_name(bm70_handle_t* handle, const char* name)
{
    if (!handle || !handle->initialized || !name) {
        return BM70_ERROR_INVALID_PARAM;
    }

    uint8_t name_len = strlen(name);
    if (name_len == 0 || name_len > 31) {
        return BM70_ERROR_INVALID_LENGTH;
    }

    uint8_t params[32];
    params[0] = 0x00;
    memcpy(&params[1], name, name_len);

    LOG_INFO("Setting device name to: %s", name);

    bm70_error_t err = send_command(handle, BM70_CMD_WRITE_DEVICE_NAME, params, name_len + 1);
    if (err != BM70_OK) {
        return err;
    }

    return wait_response(handle, BM70_EVT_COMMAND_COMPLETE, CMD_GUARD_TIMEOUT);
}

bm70_error_t bm70_read_pairing_mode(bm70_handle_t* handle)
{
    if (!handle) {
        return BM70_ERROR_INVALID_PARAM;
    }

    bm70_error_t err = send_command(handle, BM70_CMD_READ_PAIRING_MODE, NULL, 0);
    if (err != BM70_OK) {
        return err;
    }

    err = wait_response(handle, BM70_EVT_COMMAND_COMPLETE, CMD_GUARD_TIMEOUT);
    if (err == BM70_OK) {
        process_pairing_mode_response(handle);
    }

    return err;
}

bm70_error_t bm70_set_pairing_mode(bm70_handle_t* handle, bm70_io_capability_t io_capability)
{
    if (!handle || !handle->initialized) {
        return BM70_ERROR_INVALID_PARAM;
    }

    uint8_t params[2] = { 0x00, (uint8_t)io_capability };

    LOG_INFO("Setting pairing mode to: 0x%02X (%s)",
             io_capability, bm70_io_capability_to_string(io_capability));

    bm70_error_t err = send_command(handle, BM70_CMD_WRITE_PAIRING_MODE, params, 2);
    if (err != BM70_OK) {
        return err;
    }

    return wait_response(handle, BM70_EVT_COMMAND_COMPLETE, CMD_GUARD_TIMEOUT);
}

bm70_error_t bm70_leave_configure_mode(bm70_handle_t* handle)
{
    if (!handle) {
        return BM70_ERROR_INVALID_PARAM;
    }

    uint8_t params[1] = { 0x00 };

    LOG_INFO("Leaving configure mode...");

    bm70_error_t err = send_command(handle, BM70_CMD_LEAVE_CONFIGURE_MODE, params, 1);
    if (err != BM70_OK) {
        return err;
    }

    return wait_response(handle, BM70_EVT_COMMAND_COMPLETE, CMD_GUARD_TIMEOUT);
}

/* ============================================================================
 * Connection Management Functions
 * ============================================================================ */
static bool advertising_params_stale(bm70_handle_t* handle)
{
    if (!handle || handle->device_manager.current_adv_mode == BM70_ADV_MODE_UNKNOWN) {
        return true;
    }

    // Consider params stale after 30 seconds (in case BM71 reset internally)
    uint32_t age = HAL_GetTick() - handle->device_manager.adv_param_set_time;
    return age > 30000;
}

bm70_error_t bm70_start_advertising(bm70_handle_t* handle)
{

    if (!handle || !handle->initialized) {
        return BM70_ERROR_INVALID_PARAM;
    }

    // Don't try to advertise if already connected
    if (handle->conn_state == BM70_CONN_STATE_CONNECTED) {
        LOG_DEBUG("Already connected - not starting advertising");
        return BM70_OK;
    }

    // Don't try to advertise if already advertising
    if (handle->conn_state == BM70_CONN_STATE_ADVERTISING) {
        LOG_DEBUG("Already advertising");
        return BM70_OK;
    }

    // Rate limiting
    static uint32_t last_adv_attempt = 0;
    uint32_t current_time = HAL_GetTick();

    if (current_time - last_adv_attempt < 5000) {
        LOG_DEBUG("Rate limited - skipping advertising attempt");
        return BM70_OK;
    }
    last_adv_attempt = current_time;

    uint8_t enable = 0x01;
    LOG_INFO("Starting BLE advertising...");

    bm70_error_t err = send_command(handle, BM70_CMD_SET_ADV_ENABLE, &enable, 1);
    if (err != BM70_OK) {
        LOG_ERROR("Failed to send advertising command: %s", bm70_error_to_string(err));
        return err;
    }
    // Try status report first (shorter timeout)
    err = wait_response(handle, BM70_EVT_STATUS_REPORT, 500);
    if (err == BM70_OK) {
        if (handle->status == BM70_STATUS_STANDBY) {
            handle->conn_state = BM70_CONN_STATE_ADVERTISING;
            LOG_INFO("Advertising started (status report)");
            return BM70_OK;
        }
    }

    // Try command complete (shorter timeout)
    err = wait_response(handle, BM70_EVT_COMMAND_COMPLETE, 500);
    if (err == BM70_OK) {
        // Assume advertising started and mark as advertising
        handle->conn_state = BM70_CONN_STATE_ADVERTISING;
        LOG_INFO("Advertising command completed (marked as advertising)");
        return BM70_OK;
    }

    // If module is stuck in idle, try a different approach
    if (handle->status == BM70_STATUS_IDLE) {
        LOG_WARNING("Module stuck in idle - attempting alternative approach");

        // Hardware Reset
        bm70_hwreset_with_handle(handle);
        HAL_Delay(70); // Shorter delay

        // Try advertising again
        err = send_command(handle, BM70_CMD_SET_ADV_ENABLE, &enable, 1);
        if (err == BM70_OK) {
            LOG_INFO("Retry advertising command sent");
            // Assume it's working and mark as advertising
            err = wait_response(handle, BM70_EVT_STATUS_REPORT, 500);
			if (err == BM70_OK) {
				if (handle->status == BM70_STATUS_STANDBY) {
					handle->conn_state = BM70_CONN_STATE_ADVERTISING;
					LOG_INFO("✓ Advertising started (status report)");
					return BM70_OK;
				}
			}
        }
    }

    LOG_ERROR("Failed to start advertising - module may need reset");
    return BM70_ERROR_TIMEOUT;
}

bm70_error_t bm70_stop_advertising(bm70_handle_t* handle)
{
    if (!handle || !handle->initialized) {
        return BM70_ERROR_INVALID_PARAM;
    }

    uint8_t disable = 0x00;

    LOG_INFO("Stopping BLE advertising...");

    bm70_error_t err = send_command(handle, BM70_CMD_SET_ADV_ENABLE, &disable, 1);
    if (err != BM70_OK) {
        return err;
    }

    err = wait_response(handle, BM70_EVT_STATUS_REPORT, CMD_GUARD_TIMEOUT);
    if (err == BM70_OK) {
        handle->conn_state = BM70_CONN_STATE_DISCONNECTED;
        LOG_INFO("Advertising stopped");
    }

    return err;
}

bm70_error_t bm70_disconnect(bm70_handle_t* handle)
{
    if (!handle || !handle->initialized) {
        return BM70_ERROR_INVALID_PARAM;
    }

    if (handle->conn_state != BM70_CONN_STATE_CONNECTED) {
        return BM70_ERROR_NOT_CONNECTED;
    }

    uint8_t reserved = 0x00;

    LOG_INFO("Disconnecting from peer device...");

    bm70_error_t err = send_command(handle, BM70_CMD_DISCONNECT, &reserved, 1);
    if (err != BM70_OK) {
        return err;
    }

    return wait_response(handle, BM70_EVT_DISCONNECT_COMPLETE, CMD_GUARD_TIMEOUT);
}

bm70_error_t bm70_read_rssi(bm70_handle_t* handle, int8_t* rssi)
{
    if (!handle || !handle->initialized || !rssi) {
        return BM70_ERROR_INVALID_PARAM;
    }

    if (!is_valid_connection_handle(handle)) {
        return BM70_ERROR_NOT_CONNECTED;
    }

    uint8_t conn_handle = handle->conn_info.handle;

    bm70_error_t err = send_command(handle, BM70_CMD_READ_RSSI, &conn_handle, 1);
    if (err != BM70_OK) {
        return err;
    }

    err = wait_response(handle, BM70_EVT_COMMAND_COMPLETE, CMD_GUARD_TIMEOUT);
    if (err == BM70_OK && handle->response_len >= 3) {
        *rssi = (int8_t)handle->response_buffer[2];
        handle->conn_info.rssi = *rssi;
        LOG_DEBUG("RSSI: %d dBm", *rssi);
    }

    return err;
}

/* ============================================================================
 * GATT Service Functions
 * ============================================================================ */

bm70_error_t bm70_discover_service_characteristics(bm70_handle_t* handle, uint16_t service_uuid)
{
    if (!handle || !handle->initialized) {
        return BM70_ERROR_INVALID_PARAM;
    }

    LOG_INFO("Discovering characteristics for service 0x%04X...", service_uuid);

    uint8_t params[2];
    params[0] = service_uuid & 0xFF;
    params[1] = (service_uuid >> 8) & 0xFF;

    bm70_error_t err = send_command(handle, BM70_CMD_DISCOVER_CHAR, params, 2);
    if (err != BM70_OK) {
        return err;
    }

    return wait_response(handle, BM70_EVT_DISCOVER_CHAR_RSP, CMD_GUARD_TIMEOUT);
}


bm70_error_t bm70_send_char_value(bm70_handle_t* handle, uint16_t char_handle,
                                  const uint8_t* data, uint8_t len)
{
    if (!handle || !handle->initialized || !data || len == 0) {
        return BM70_ERROR_INVALID_PARAM;
    }

    if (!is_valid_connection_handle(handle)) {
        return BM70_ERROR_NOT_CONNECTED;
    }

    if (len > 244) {
        return BM70_ERROR_INVALID_LENGTH;
    }

    uint8_t params[247];
	params[0] = handle->conn_info.handle;           // Connection handle
	params[1] = (char_handle >> 8) & 0xFF;          // Char handle high
	params[2] = char_handle & 0xFF;                 // Char handle low
	memcpy(&params[3], data, len);

	// DEBUG: Log the exact command being sent
	LOG_DEBUG("BLE Command 0x38: conn=0x%02X, char=0x%04X, len=%d",
			  params[0], char_handle, len);
	LOG_DEBUG("Raw params: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
			  params[0], params[1], params[2], params[3], params[4],
			  params[5], params[6], params[7], params[8], params[9], params[10]);

    bm70_error_t err = send_command(handle, BM70_CMD_SEND_CHAR_VALUE, params, len + 3);
    if (err != BM70_OK) {
    	LOG_ERROR("send_command failed: %s", bm70_error_to_string(err));
        return err;
    }

    return wait_response(handle, BM70_EVT_COMMAND_COMPLETE, CMD_GUARD_TIMEOUT);
}

bm70_error_t bm70_verify_configuration(bm70_handle_t* handle)
{
    if (!handle) {
        return BM70_ERROR_INVALID_PARAM;
    }

    LOG_INFO("========================================");
    LOG_INFO("BM70/71 Configuration Verification");
    LOG_INFO("========================================");

    bm70_error_t err = bm70_read_local_information(handle);
    if (err != BM70_OK) {
        LOG_ERROR("Failed to read local information: %s", bm70_error_to_string(err));
        return err;
    }

    bm70_status_t status;
    err = bm70_get_status(handle, &status);
    if (err != BM70_OK) {
        LOG_WARNING("Failed to read status: %s", bm70_error_to_string(err));
    }

    err = bm70_read_device_name(handle);
    if (err != BM70_OK) {
        LOG_WARNING("Failed to read device name: %s", bm70_error_to_string(err));
    }

    err = bm70_read_pairing_mode(handle);
    if (err != BM70_OK) {
        LOG_WARNING("Failed to read pairing mode: %s", bm70_error_to_string(err));
    }

    err = bm70_load_paired_devices(handle);
    if (err != BM70_OK) {
        LOG_WARNING("Failed to read paired devices: %s", bm70_error_to_string(err));
    }
    err = bm70_read_all_local_services(handle);
    if (err != BM70_OK) {
		LOG_WARNING("Failed to read GATT services: %s", bm70_error_to_string(err));
	}
    HAL_Delay(2);
    LOG_INFO("Configuration verification completed");
    LOG_INFO("Module Status: %s", bm70_status_to_string(handle->status));
    LOG_INFO("Connection State: %s", bm70_conn_state_to_string(handle->conn_state));
    LOG_INFO("HID Ready: %s", bm70_is_hid_ready(handle) ? "Yes" : "No");
    LOG_INFO("========================================");
    HAL_Delay(2);

    return BM70_OK;
}

/* ============================================================================
 * HID Functions
 * ============================================================================ */

bm70_error_t bm70_send_keyboard_report(bm70_handle_t* handle, const bm70_kbd_report_t* report)
{
    if (!handle || !handle->initialized || !report) {
        return BM70_ERROR_INVALID_PARAM;
    }

    if (!is_valid_connection_handle(handle)) {
        return BM70_ERROR_NOT_CONNECTED;
    }

    uint8_t data[8];
    data[0] = report->modifiers;
    data[1] = report->reserved;
    memcpy(&data[2], report->keys, 6);

    bm70_error_t err = bm70_send_char_value(handle,HID_KB_INPUT_HANDLE, data, 8);
    if (err == BM70_OK) {
        LOG_DEBUG("KB Report sent: Mod=0x%02X, Keys=[0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X]",
                 report->modifiers, report->keys[0], report->keys[1], report->keys[2],
                 report->keys[3], report->keys[4], report->keys[5]);
    }

    return err;
}

bm70_error_t bm70_send_consumer_report(bm70_handle_t* handle, const bm70_consumer_report_t* report)
{
    if (!handle || !handle->initialized || !report) {
        return BM70_ERROR_INVALID_PARAM;
    }

    if (!is_valid_connection_handle(handle)) {
        return BM70_ERROR_NOT_CONNECTED;
    }

    if (!handle->hid_state.service_ready) {
        return BM70_ERROR_HID_NOT_READY;
    }

    uint8_t data[2];
    data[0] = report->usage[0];
    data[1] = report->usage[1];

    bm70_error_t err = bm70_send_char_value(handle, HID_CONSUMER_INPUT_HANDLE, data, 2);
    if (err == BM70_OK) {
        uint16_t usage = BM70_GET_CONSUMER_USAGE(*report);
        LOG_DEBUG("Consumer Report sent: Usage=0x%04X", usage);
    }

    return err;
}

bm70_error_t bm70_update_battery_level(bm70_handle_t* handle, uint8_t level)
{
    if (!handle || !handle->initialized) {
        return BM70_ERROR_INVALID_PARAM;
    }

    if (level > 100) {
        level = 100;
    }

    uint8_t params[3];
    params[0] = (BATTERY_LEVEL_HANDLE >> 8) & 0xFF; //Big Endian
    params[1] = BATTERY_LEVEL_HANDLE & 0xFF;
    params[2] = level;

    bm70_error_t err = send_command(handle, BM70_CMD_UPDATE_CHAR_VALUE, params, 3);
    if (err == BM70_OK) {
        LOG_DEBUG("Battery level updated: %d%%", level);
        err = wait_response(handle, BM70_EVT_COMMAND_COMPLETE, CMD_GUARD_TIMEOUT);
    }

    return err;
}

bm70_error_t bm70_set_hid_protocol_mode(bm70_handle_t* handle, uint8_t mode)
{
    if (!handle || !handle->initialized) {
        return BM70_ERROR_INVALID_PARAM;
    }

    if (mode > HID_PROTOCOL_REPORT) {
        return BM70_ERROR_INVALID_PARAM;
    }

    if (!is_valid_connection_handle(handle)) {
        handle->hid_state.protocol_mode = mode;
        return BM70_OK;
    }

    uint8_t params[3];
    params[0] =  HID_PROTOCOL_MODE_HANDLE & 0xFF;
    params[1] = ( HID_PROTOCOL_MODE_HANDLE >> 8) & 0xFF;
    params[2] = mode;

    bm70_error_t err = send_command(handle, BM70_CMD_SEND_CHAR_VALUE, params, 3);
    if (err == BM70_OK) {
        handle->hid_state.protocol_mode = mode;
        LOG_INFO("HID protocol mode set to: %s",
                mode == HID_PROTOCOL_BOOT ? "Boot" : "Report");
        err = wait_response(handle, BM70_EVT_COMMAND_COMPLETE, CMD_GUARD_TIMEOUT);
    }

    return err;
}

bool bm70_is_hid_ready(bm70_handle_t* handle)
{
    return (handle &&
            handle->initialized &&
            handle->conn_state == BM70_CONN_STATE_CONNECTED &&
            handle->hid_state.service_ready &&
            !handle->hid_state.suspended);
}

/* ============================================================================
 * Pairing Functions
 * ============================================================================ */

bm70_error_t bm70_initiate_pairing(bm70_handle_t* handle)
{
    if (!handle || !handle->initialized) {
        return BM70_ERROR_INVALID_PARAM;
    }

    if (!is_valid_connection_handle(handle)) {
        return BM70_ERROR_NOT_CONNECTED;
    }

    uint8_t conn_handle = handle->conn_info.handle;

    LOG_INFO("Initiating pairing with connected device...");

    bm70_error_t err = send_command(handle, BM70_CMD_PAIRING_REQUEST, &conn_handle, 1);
    if (err != BM70_OK) {
        return err;
    }

    return wait_response(handle, BM70_EVT_COMMAND_COMPLETE, CMD_GUARD_TIMEOUT);
}

bm70_error_t bm70_read_all_paired_devices(bm70_handle_t* handle)
{
    if (!handle) {
        return BM70_ERROR_INVALID_PARAM;
    }

    LOG_INFO("Reading all paired devices...");

    bm70_error_t err = send_command(handle, BM70_CMD_READ_PAIRED_LIST, NULL, 0);
    if (err != BM70_OK) {
        return err;
    }

    err = wait_response(handle, BM70_EVT_COMMAND_COMPLETE, CMD_GUARD_TIMEOUT);
    if (err == BM70_OK && handle->response_len >= 3) {
        uint8_t status = handle->response_buffer[1];
        if (status == 0x00) {
            uint8_t num_devices = handle->response_buffer[2];
            LOG_INFO("Number of paired devices: %d", num_devices);

            if (num_devices == 0) {
                LOG_INFO("No devices currently paired");
            } else {
                int offset = 3;
                for (int i = 0; i < num_devices && offset + 13 <= handle->response_len; i++) {
                    uint8_t device_index = handle->response_buffer[offset];
                    uint8_t priority = handle->response_buffer[offset + 1];
                    uint8_t addr_type = handle->response_buffer[offset + 2];

                    LOG_INFO("Device %d: Index=%d, Priority=%d, AddrType=%d",
                            i, device_index, priority, addr_type);
                    LOG_INFO("  Address: %02X:%02X:%02X:%02X:%02X:%02X",
                            handle->response_buffer[offset + 8], handle->response_buffer[offset + 7],
                            handle->response_buffer[offset + 6], handle->response_buffer[offset + 5],
                            handle->response_buffer[offset + 4], handle->response_buffer[offset + 3]);

                    offset += 14;
                }
            }
        } else {
            LOG_ERROR("Read paired devices failed with status: 0x%02X", status);
        }
    }

    return err;
}

bm70_error_t bm70_delete_paired_device(bm70_handle_t* handle, uint8_t device_index)
{
    if (!handle || !handle->initialized) {
        return BM70_ERROR_INVALID_PARAM;
    }

    if (device_index > 7) {
        return BM70_ERROR_INVALID_PARAM;
    }

    LOG_INFO("Deleting paired device index: %d", device_index);

    bm70_error_t err = send_command(handle, BM70_CMD_DELETE_PAIRED_DEVICE, &device_index, 1);
    if (err != BM70_OK) {
        return err;
    }

    return wait_response(handle, BM70_EVT_COMMAND_COMPLETE, CMD_GUARD_TIMEOUT);
}

bm70_error_t bm70_erase_all_paired_devices(bm70_handle_t* handle)
{
    if (!handle || !handle->initialized) {
        return BM70_ERROR_INVALID_PARAM;
    }

    LOG_INFO("Erasing all paired devices...");

    bm70_error_t err = send_command(handle, BM70_CMD_ERASE_PAIRED_LIST, NULL, 0);
    if (err != BM70_OK) {
        return err;
    }

    return wait_response(handle, BM70_EVT_COMMAND_COMPLETE, CMD_GUARD_TIMEOUT);
}

/* ============================================================================
 * Response Processing Functions
 * ============================================================================ */

static void process_local_info_response(bm70_handle_t *handle)
{
    if (handle->response_len >= 12) {
        LOG_INFO("========= BM70/71 Local Information =========");
        LOG_INFO("Command: 0x%02X, Status: 0x%02X",
                handle->response_buffer[0], handle->response_buffer[1]);

        if (handle->response_buffer[1] == 0x00) {
            LOG_INFO("Firmware: %02X.%02X.%02X.%02X",
                    handle->response_buffer[2], handle->response_buffer[3],
                    handle->response_buffer[4], handle->response_buffer[5]);

            LOG_INFO("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
                    handle->response_buffer[11], handle->response_buffer[10],
                    handle->response_buffer[9], handle->response_buffer[8],
                    handle->response_buffer[7], handle->response_buffer[6]);

            if (handle->response_len > 12) {
                uint8_t hw_version = handle->response_buffer[12];
                LOG_INFO("Hardware: 0x%02X (%s)", hw_version,
                        get_hardware_version_string(hw_version));
            }
        }
        LOG_INFO("============================================");
    }
}

static void process_device_name_response(bm70_handle_t *handle)
{
    if (handle->response_len >= 2) {
        LOG_INFO("Command: 0x%02X, Status: 0x%02X",
                handle->response_buffer[0], handle->response_buffer[1]);

        if (handle->response_buffer[1] == 0x00 && handle->response_len > 2) {
            uint16_t name_len = handle->response_len - 2;
            char device_name[BM70_MAX_DEVICE_NAME_LEN + 1] = {0};

            if (name_len > BM70_MAX_DEVICE_NAME_LEN) {
                name_len = BM70_MAX_DEVICE_NAME_LEN;
            }

            memcpy(device_name, &handle->response_buffer[2], name_len);
            device_name[name_len] = '\0';

            LOG_INFO("Device Name: '%s' (length: %d)", device_name, name_len);
        } else {
            LOG_INFO("No device name data returned");
        }
    }
}

static void process_pairing_mode_response(bm70_handle_t *handle)
{
    if (handle->response_len >= 3) {
        LOG_INFO("Command: 0x%02X, Status: 0x%02X",
                handle->response_buffer[0], handle->response_buffer[1]);

        if (handle->response_buffer[1] == 0x00) {
            uint8_t io_capability = handle->response_buffer[2];
            LOG_INFO("I/O Capability: 0x%02X (%s)", io_capability,
                    bm70_io_capability_to_string((bm70_io_capability_t)io_capability));
        }
    }
}

/* ============================================================================
 * Utility Helper Functions
 * ============================================================================ */

static const char* get_command_error_string(uint8_t error_code)
{
    switch (error_code) {
        case 0x00: return "Success";
        case 0x01: return "Unknown HCI Command";
        case 0x02: return "Unknown Connection Identifier";
        case 0x03: return "Hardware Failure";
        case 0x05: return "Authentication Failure";
        case 0x06: return "PIN or Key Missing";
        case 0x07: return "Memory Capacity Exceeded";
        case 0x08: return "Connection Timeout";
        case 0x0C: return "Command Disallowed";
        case 0x12: return "Invalid HCI Command Parameters";
        case 0x16: return "Connection Terminated By Local Host";
        case 0x1F: return "Unspecified Error";
        case 0x2F: return "Insufficient Security";
        case 0x3B: return "Unacceptable Connection Interval";
        case 0x3E: return "Connection Failed to be Established";
        case 0xFF: return "UART Checksum Error";
        default: return "Unknown Error";
    }
}

static const char* get_hardware_version_string(uint8_t hw_version)
{
    switch (hw_version) {
        case 0x00: return "BM70";
        case 0x01: return "BM71";
        case 0x02: return "IS1870";
        case 0x03: return "IS1871";
        default: return "Unknown";
    }
}

static bool is_valid_connection_handle(bm70_handle_t *handle)
{
    return (handle &&
            handle->conn_state == BM70_CONN_STATE_CONNECTED &&
            handle->conn_info.handle != 0);
}

/* ============================================================================
 * Callback Registration Functions
 * ============================================================================ */

void bm70_set_connection_callback(bm70_handle_t* handle, bm70_connection_cb_t callback)
{
    if (handle) {
        handle->conn_cb = callback;
        LOG_DEBUG("Connection callback registered");
    }
}

void bm70_set_status_callback(bm70_handle_t* handle, bm70_status_cb_t callback)
{
    if (handle) {
        handle->status_cb = callback;
        LOG_DEBUG("Status callback registered");
    }
}

void bm70_set_data_callback(bm70_handle_t* handle, bm70_data_cb_t callback)
{
    if (handle) {
        handle->data_cb = callback;
        LOG_DEBUG("Data callback registered");
    }
}

void bm70_set_error_callback(bm70_handle_t* handle, bm70_error_cb_t callback)
{
    if (handle) {
        handle->error_cb = callback;
        LOG_DEBUG("Error callback registered");
    }
}

void bm70_set_pairing_callback(bm70_handle_t* handle, bm70_pairing_cb_t callback)
{
    if (handle) {
        handle->pairing_cb = callback;
        LOG_DEBUG("Pairing callback registered");
    }
}

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

const char* bm70_error_to_string(bm70_error_t error)
{
    switch (error) {
        case BM70_OK: return "Success";
        case BM70_ERROR_TIMEOUT: return "Timeout";
        case BM70_ERROR_UART: return "UART Error";
        case BM70_ERROR_INVALID_PARAM: return "Invalid Parameter";
        case BM70_ERROR_NOT_CONNECTED: return "Not Connected";
        case BM70_ERROR_BUSY: return "Busy";
        case BM70_ERROR_COMMAND_FAILED: return "Command Failed";
        case BM70_ERROR_CHECKSUM: return "Checksum Error";
        case BM70_ERROR_NO_MEMORY: return "No Memory";
        case BM70_ERROR_WRONG_MODE: return "Wrong Mode";
        case BM70_ERROR_HID_NOT_READY: return "HID Not Ready";
        case BM70_ERROR_NOT_INITIALIZED: return "Not Initialized";
        case BM70_ERROR_INVALID_LENGTH: return "Invalid Length";
        case BM70_ERROR_PROTOCOL_ERROR: return "Protocol Error";
        default: return "Unknown Error";
    }
}

const char* bm70_status_to_string(bm70_status_t status)
{
    switch (status) {
        case BM70_STATUS_SCANNING: return "Scanning";
        case BM70_STATUS_CONNECTING: return "Connecting";
        case BM70_STATUS_STANDBY: return "Standby";
        case BM70_STATUS_BROADCAST: return "Broadcast";
        case BM70_STATUS_TRANSPARENT: return "Transparent";
        case BM70_STATUS_IDLE: return "Idle";
        case BM70_STATUS_SHUTDOWN: return "Shutdown";
        case BM70_STATUS_CONFIGURE: return "Configure";
        case BM70_STATUS_CONNECTED: return "Connected";
        default: return "Unknown Status";
    }
}

const char* bm70_conn_state_to_string(bm70_conn_state_t state)
{
    switch (state) {
        case BM70_CONN_STATE_DISCONNECTED: return "Disconnected";
        case BM70_CONN_STATE_ADVERTISING: return "Advertising";
        case BM70_CONN_STATE_CONNECTING: return "Connecting";
        case BM70_CONN_STATE_CONNECTED: return "Connected";
        case BM70_CONN_STATE_PAIRING: return "Pairing";
        default: return "Unknown State";
    }
}

const char* bm70_io_capability_to_string(bm70_io_capability_t capability)
{
    switch (capability) {
        case BM70_IO_CAP_DISPLAY_ONLY: return "Display Only";
        case BM70_IO_CAP_DISPLAY_YESNO: return "Display Yes/No";
        case BM70_IO_CAP_KEYBOARD_ONLY: return "Keyboard Only";
        case BM70_IO_CAP_NO_INPUT_OUTPUT: return "No Input/Output";
        case BM70_IO_CAP_KEYBOARD_DISPLAY: return "Keyboard Display";
        default: return "Unknown Capability";
    }
}


bm70_error_t bm70_handle_cccd_request(bm70_handle_t* handle, uint16_t char_handle,
                                     const uint8_t* data, uint8_t len)
{
    if (!handle || !data || len < 2) {
        return BM70_ERROR_INVALID_PARAM;
    }

    if (!is_valid_connection_handle(handle)) {
        return BM70_ERROR_NOT_CONNECTED;
    }

    // Parse what client wants (little endian)
    uint16_t cccd_value = data[0] | (data[1] << 8);
    bool enable_notifications = (cccd_value & 0x0001) != 0;
    bool enable_indications = (cccd_value & 0x0002) != 0;

    // Map CCCD handles to their characteristic names for logging
    const char* char_name = "Unknown";
    uint16_t char_value_handle = 0;

    switch (char_handle) {
        case HID_KB_INPUT_CCCD_HANDLE:  // 0x006F
            char_value_handle = HID_KB_INPUT_HANDLE;  // 0x006D
            char_name = "Keyboard Report";
			handle->hid_state.keyboard_cccd_enabled = true;
            break;

        case HID_CONSUMER_INPUT_CCCD_HANDLE:  // 0x0078
            char_value_handle = HID_CONSUMER_INPUT_HANDLE;  // 0x0076
            char_name = "Consumer Report";
			handle->hid_state.consumer_cccd_enabled = true;
            break;

        case BATTERY_LEVEL_CCCD:  // 0x007C
            char_value_handle = BATTERY_LEVEL_HANDLE;  // 0x007B
            char_name = "Battery Level";
			handle->hid_state.battery_cccd_enabled = true;
            break;

        case 0x0072:  // Boot Input CCCD
            char_value_handle = 0x0071;
            char_name = "Boot Input Report";
            break;

        default:
            LOG_INFO("Client wrote to unknown CCCD 0x%04X: Notifications=%s, Indications=%s",
                     char_handle,
                     enable_notifications ? "YES" : "NO",
                     enable_indications ? "YES" : "NO");
            return BM70_OK;
    }

    LOG_INFO("Client %s notifications for %s (CCCD=0x%04X, Char=0x%04X)",
             enable_notifications ? "ENABLED" : "DISABLED",
             char_name, char_handle, char_value_handle);

    if (enable_indications) {
        LOG_INFO("Client also enabled indications for %s", char_name);
    }

    if (enable_notifications) {
        handle->hid_state.notifications_enabled = true;
    }

    return BM70_OK;
}

bm70_error_t bm70_enable_cccd_notifications(bm70_handle_t* handle, uint16_t cccd_handle)
{
    if (!handle) {
        return BM70_ERROR_INVALID_PARAM;
    }

   uint8_t params[6];
   params[0] = handle->conn_info.handle;       // Connection handle
   params[1] = 0x01;                           // Write with response
   params[2] = (cccd_handle >> 8) & 0xFF;      // CCCD handle high byte
   params[3] = cccd_handle & 0xFF;             // CCCD handle low byte
   params[4] = 0x01;                           // Enable notifications (0x0001)
   params[5] = 0x00;

   LOG_INFO("Enabling notifications on CCCD 0x%04X",
            cccd_handle);

   bm70_error_t err = send_command(handle, BM70_CMD_SEND_CHAR_VALUE, params, 6);
   if (err != BM70_OK) {
       LOG_ERROR("Failed to enable notifications on 0x%04X: %s",
    		   cccd_handle, bm70_error_to_string(err));
       return err;
   }

   err = wait_response(handle, BM70_EVT_COMMAND_COMPLETE, CMD_GUARD_TIMEOUT);
   if (err == BM70_OK) {
       LOG_INFO("✓ Notifications enabled on CCCD 0x%04X",
                cccd_handle);
       handle->hid_state.notifications_enabled = true;
   } else {
       LOG_ERROR("✗ Failed to enable notifications on CCCD 0x%04X: %s",
                 cccd_handle, bm70_error_to_string(err));
   }

   return err;
}

void bm70_read_and_log_characteristic_value(bm70_handle_t* handle,
                                                   uint16_t char_value_handle,
                                                   const char* char_name)
{
    if (!handle) {
        return;
    }

    HAL_Delay(1);

    const char* name = char_name ? char_name : "Unknown";
    LOG_INFO("Reading characteristic %s (0x%04X)...", name, char_value_handle);

    uint8_t params[2];
    params[0] = (char_value_handle >> 8) & 0xFF;  // Handle high byte
    params[1] = char_value_handle & 0xFF;         // Handle low byte

    bm70_error_t err = send_command(handle, 0x3A, params, 2);
    if (err != BM70_OK) {
        LOG_ERROR("✗ Failed to send read command for %s: %s", name, bm70_error_to_string(err));
        return;
    }

    err = wait_response(handle, BM70_EVT_COMMAND_COMPLETE, 200);  // Changed to 0x81
    if (err != BM70_OK) {
        LOG_ERROR("✗ Failed to receive response for %s: %s", name, bm70_error_to_string(err));
        return;
    }

    // Parse and log the response
    if (handle->response_len < 2) {
        LOG_ERROR("✗ Response too short for %s: %d bytes", name, handle->response_len);
        return;
    }

    uint8_t command = handle->response_buffer[0];
    uint8_t status = handle->response_buffer[1];

    if (command != 0x3A) {
        LOG_ERROR("✗ Unexpected command in response for %s: 0x%02X", name, command);
        return;
    }

    if (status != 0x00) {
        LOG_ERROR("✗ %s read failed with status: 0x%02X (%s)",
                  name, status, get_command_error_string(status));
        return;
    }

    // Log the characteristic value
    if (handle->response_len > 2) {
        uint16_t value_length = handle->response_len - 2;

        LOG_INFO("✓ %s (0x%04X): %d bytes", name, char_value_handle, value_length);

        // Log as hex string on one line
        LOG_DUMP("  Hex: ");
        for (uint16_t i = 0; i < value_length; i++) {
            LOG_DUMP("%02X ", handle->response_buffer[2 + i]);
        }
        LOG_DUMP("\r\n");

        // For small values, also interpret them
        if (value_length == 1) {
            LOG_INFO("  Decimal: %d", handle->response_buffer[2]);
        } else if (value_length == 2) {
            uint16_t val16 = handle->response_buffer[2] | (handle->response_buffer[3] << 8);
            LOG_INFO("  16-bit LE: 0x%04X (%d)", val16, val16);
        } else if (value_length == 4 && char_value_handle == HID_INFORMATION_HANDLE) {
            // Special handling for HID Information
            uint16_t bcd_hid = handle->response_buffer[2] | (handle->response_buffer[3] << 8);
            uint8_t country = handle->response_buffer[4];
            uint8_t flags = handle->response_buffer[5];
            LOG_INFO("  HID Info: bcdHID=0x%04X, Country=0x%02X, Flags=0x%02X", bcd_hid, country, flags);
            LOG_INFO("    RemoteWake: %s, NormallyConnectable: %s",
                     (flags & 0x01) ? "Yes" : "No", (flags & 0x02) ? "Yes" : "No");
        }
    } else {
        LOG_INFO("✓ %s (0x%04X): No value data", name, char_value_handle);
    }
}

void bm70_validate_state(bm70_handle_t* handle)
{
    if (!handle) return;

    // Ensure software flag matches hardware state
    if (handle->conn_state == BM70_CONN_STATE_CONNECTED &&
        handle->conn_info.handle != 0) {
        ble_conn_flag = true;
    } else if (handle->conn_state == BM70_CONN_STATE_DISCONNECTED) {
        ble_conn_flag = false;
    }

    // Ensure HID state is consistent
    if (handle->conn_state != BM70_CONN_STATE_CONNECTED) {
        handle->hid_state.service_ready = false;
        handle->hid_state.notifications_enabled = false;
    }

    // Device manager state validation and recovery
    if (handle->device_manager.devices_loaded) {
        bool state_corrupted = false;

        // Validate device count bounds
        if (handle->device_manager.device_count > BM70_MAX_PAIRED_DEVICES) {
            LOG_WARNING("Device count corrupted: %d > %d, resetting to 0",
                        handle->device_manager.device_count, BM70_MAX_PAIRED_DEVICES);
            handle->device_manager.device_count = 0;
            state_corrupted = true;
        }

        // Validate current device slot
        if (handle->device_manager.current_device_slot >= BM70_MAX_PAIRED_DEVICES) {
            LOG_WARNING("Current device slot corrupted: %d >= %d, resetting to 0",
                        handle->device_manager.current_device_slot, BM70_MAX_PAIRED_DEVICES);
            handle->device_manager.current_device_slot = 0;
            state_corrupted = true;
        }

        // Validate next device slot
        if (handle->device_manager.next_device_slot >= BM70_MAX_PAIRED_DEVICES) {
            LOG_WARNING("Next device slot corrupted: %d >= %d, resetting to 0",
                        handle->device_manager.next_device_slot, BM70_MAX_PAIRED_DEVICES);
            handle->device_manager.next_device_slot = 0;
            state_corrupted = true;
        }

        // Clear switch flag if it's been stuck too long (safety mechanism)
        static uint32_t switch_start_time = 0;

        if (handle->device_manager.switch_device) {
            if (switch_start_time == 0) {
                switch_start_time = HAL_GetTick();
            } else if ((HAL_GetTick() - switch_start_time) > 10000) {  // 10 second timeout
                LOG_WARNING("Device switch timeout - clearing switch flag");
                handle->device_manager.switch_device = false;
                switch_start_time = 0;

                // Try to recover to standard advertising
                bm70_set_standard_advertising(handle);
            }
        } else {
            switch_start_time = 0;  // Reset timer when not switching
        }

        // If state corruption detected, try to reload from BM71
        if (state_corrupted) {
            LOG_INFO("State corruption detected, attempting to reload from BM71...");
            bm70_error_t err = bm70_load_paired_devices(handle);
            if (err != BM70_OK) {
                LOG_ERROR("Failed to reload paired devices: %s", bm70_error_to_string(err));
                // Reset device manager to safe state
                memset(&handle->device_manager, 0, sizeof(handle->device_manager));
            }
        }
    }
}

/* ============================================================================
 * Device Management Implementation
 * ============================================================================ */

bm70_error_t bm70_load_paired_devices(bm70_handle_t* handle)
{
    if (!handle) {
        return BM70_ERROR_INVALID_PARAM;
    }

    // Clear existing device list
    memset(&handle->device_manager, 0, sizeof(handle->device_manager));

    bm70_error_t err = send_command(handle, BM70_CMD_READ_PAIRED_LIST, NULL, 0);
    if (err != BM70_OK) {
        return err;
    }

    err = wait_response(handle, BM70_EVT_COMMAND_COMPLETE, CMD_GUARD_TIMEOUT);
    if (err != BM70_OK) {
        return err;
    }

    if (handle->response_len < 3) {
        return BM70_ERROR_PROTOCOL_ERROR;
    }

    uint8_t status = handle->response_buffer[1];
    uint8_t num_devices = handle->response_buffer[2];

    if (status != 0x00) {
        return BM70_ERROR_COMMAND_FAILED;
    }

    LOG_INFO("BM71 reports %d paired devices", num_devices);

    if (num_devices == 0) {
        handle->device_manager.devices_loaded = true;
        return BM70_OK;
    }

    if (num_devices > BM70_MAX_PAIRED_DEVICES) {
        num_devices = BM70_MAX_PAIRED_DEVICES;
    }

    int offset = 3;
    uint8_t loaded_count = 0;

    for (uint8_t i = 0; i < num_devices && loaded_count < BM70_MAX_PAIRED_DEVICES; i++) {
        if (offset + 14 > handle->response_len) {
            break;
        }

        bm70_paired_device_t* device = &handle->device_manager.devices[loaded_count];

        // Save raw data from BM71 response
        device->device_index = handle->response_buffer[offset + 0];
        device->priority = handle->response_buffer[offset + 1];
        device->addr_type = handle->response_buffer[offset + 2];

        // Save address - try reversed first (usually correct for BT)
        device->bd_addr[0] = handle->response_buffer[offset + 8];
        device->bd_addr[1] = handle->response_buffer[offset + 7];
        device->bd_addr[2] = handle->response_buffer[offset + 6];
        device->bd_addr[3] = handle->response_buffer[offset + 5];
        device->bd_addr[4] = handle->response_buffer[offset + 4];
        device->bd_addr[5] = handle->response_buffer[offset + 3];

        device->is_valid = true;
        device->is_current = false;
        device->last_connected = 0;

        LOG_INFO("Device %d: Idx=%d, Pri=%d, Type=%d, Addr=%02X:%02X:%02X:%02X:%02X:%02X",
                 loaded_count, device->device_index, device->priority, device->addr_type,
                 device->bd_addr[0], device->bd_addr[1], device->bd_addr[2],
                 device->bd_addr[3], device->bd_addr[4], device->bd_addr[5]);

        loaded_count++;
        offset += 14;
    }

    handle->device_manager.device_count = loaded_count;
    handle->device_manager.current_device_slot = 0;
    handle->device_manager.next_device_slot = 0;
    handle->device_manager.devices_loaded = true;

    LOG_INFO("Loaded %d paired devices", loaded_count);
    return BM70_OK;
}

bm70_error_t bm70_set_directed_advertising_to_device(bm70_handle_t* handle, uint8_t device_slot)
{
    if (!handle || !handle->initialized) {
        return BM70_ERROR_INVALID_PARAM;
    }

    if (device_slot >= BM70_MAX_PAIRED_DEVICES) {
        LOG_ERROR("Invalid device slot: %d >= %d", device_slot, BM70_MAX_PAIRED_DEVICES);
        return BM70_ERROR_INVALID_PARAM;
    }

    if (!handle->device_manager.devices_loaded || handle->device_manager.device_count == 0) {
        LOG_ERROR("No paired devices loaded");
        return BM70_ERROR_INVALID_PARAM;
    }

    bm70_paired_device_t* target_device = &handle->device_manager.devices[device_slot];
    if (!target_device->is_valid) {
        LOG_ERROR("Device slot %d is not valid", device_slot);
        return BM70_ERROR_INVALID_PARAM;
    }

    // Smart redundancy check - only skip if already set for same device AND not stale
    if (handle->device_manager.current_adv_mode == BM70_ADV_MODE_DIRECTED &&
        handle->device_manager.directed_device_slot == device_slot &&
        !advertising_params_stale(handle)) {
        LOG_DEBUG("Directed advertising already configured for device slot %d", device_slot);
        return BM70_OK;
    }

    LOG_INFO("Setting directed advertising to device %d (Index=%d, Type=%d)",
             device_slot, target_device->device_index, target_device->addr_type);

    // Build Set_Advertising_Parameter (0x13) command
    uint8_t params[10];
    uint16_t adv_interval = BM70_ADV_INTERVAL_100MS;
    params[0] = adv_interval & 0xFF;
    params[1] = (adv_interval >> 8) & 0xFF;
    params[2] = BM70_ADV_TYPE_DIRECTED;
    params[3] = target_device->addr_type;
    memcpy(&params[4], target_device->bd_addr, BM70_BT_ADDR_LEN);

    bm70_error_t err = send_command(handle, BM70_CMD_SET_ADV_PARAMETER, params, 10);
    if (err != BM70_OK) {
        LOG_ERROR("Failed to send directed advertising command: %s", bm70_error_to_string(err));
        return err;
    }

    err = wait_response(handle, BM70_EVT_COMMAND_COMPLETE, CMD_GUARD_TIMEOUT);
    if (err != BM70_OK) {
        LOG_ERROR("Directed advertising command failed: %s", bm70_error_to_string(err));
        return err;
    }

    // Update state only on success
    handle->device_manager.current_adv_mode = BM70_ADV_MODE_DIRECTED;
    handle->device_manager.directed_device_slot = device_slot;
    handle->device_manager.adv_param_set_time = HAL_GetTick();
    handle->device_manager.adv_param_set = true;

    LOG_INFO("✓ Directed advertising configured for device slot %d", device_slot);
    return BM70_OK;
}

bm70_error_t bm70_set_standard_advertising(bm70_handle_t* handle)
{
    if (!handle || !handle->initialized) {
        return BM70_ERROR_INVALID_PARAM;
    }

    // Smart redundancy check - only skip if already in standard mode AND not stale
    if (handle->device_manager.current_adv_mode == BM70_ADV_MODE_STANDARD &&
        !advertising_params_stale(handle)) {
        LOG_DEBUG("Already in standard advertising mode");
        return BM70_OK;
    }

    LOG_INFO("Setting standard (undirected) advertising parameters");

    // Build Set_Advertising_Parameter (0x13) command
    uint8_t params[10];
    uint16_t adv_interval = BM70_ADV_INTERVAL_100MS;
    params[0] = adv_interval & 0xFF;
    params[1] = (adv_interval >> 8) & 0xFF;
    params[2] = BM70_ADV_TYPE_UNDIRECTED;
    params[3] = BM70_ADDR_TYPE_PUBLIC;
    memset(&params[4], 0, BM70_BT_ADDR_LEN);

    bm70_error_t err = send_command(handle, BM70_CMD_SET_ADV_PARAMETER, params, 10);
    if (err != BM70_OK) {
        LOG_ERROR("Failed to send standard advertising command: %s", bm70_error_to_string(err));
        return err;
    }

    err = wait_response(handle, BM70_EVT_COMMAND_COMPLETE, CMD_GUARD_TIMEOUT);
    if (err != BM70_OK) {
        LOG_ERROR("Standard advertising command failed: %s", bm70_error_to_string(err));
        return err;
    }

    // Update state only on success
    handle->device_manager.current_adv_mode = BM70_ADV_MODE_STANDARD;
    handle->device_manager.directed_device_slot = 0;  // Not applicable
    handle->device_manager.adv_param_set_time = HAL_GetTick();
    handle->device_manager.adv_param_set = false;  // Legacy flag for standard mode

    LOG_INFO("✓ Standard advertising parameters configured");
    return BM70_OK;
}

bm70_error_t bm70_start_directed_advertising(bm70_handle_t* handle)
{
    if (!handle || !handle->initialized) {
        return BM70_ERROR_INVALID_PARAM;
    }

    uint8_t enable = 0x02; //Enter Standby Mode and only connectable for trust device
	bm70_error_t err = send_command(handle, BM70_CMD_SET_ADV_ENABLE, &enable, 1);
	if (err != BM70_OK) {
		LOG_ERROR("Failed to send advertising command: %s", bm70_error_to_string(err));
		return err;
	}

	// Try status report first
	err = wait_response(handle, BM70_EVT_STATUS_REPORT, 200);
	if (err == BM70_OK) {
		if (handle->status == BM70_STATUS_STANDBY) {
			handle->conn_state = BM70_CONN_STATE_ADVERTISING;
			LOG_INFO("✓ Advertising started (status report)");
			return BM70_OK;
		}
	}

	// Try command complete
	err = wait_response(handle, BM70_EVT_COMMAND_COMPLETE, 200);
	if (err == BM70_OK) {
		// Assume advertising started and mark as advertising
		handle->conn_state = BM70_CONN_STATE_ADVERTISING;
		LOG_INFO("✓ Advertising command completed (marked as advertising)");
		return BM70_OK;
	}

	return err;
}

bm70_error_t bm70_cycle_to_next_device(bm70_handle_t* handle)
{
    if (!handle || !handle->initialized) {
        return BM70_ERROR_INVALID_PARAM;
    }

    if (!handle->device_manager.devices_loaded || handle->device_manager.device_count == 0) {
        LOG_ERROR("No paired devices available for cycling");
        return BM70_ERROR_INVALID_PARAM;
    }

    // Validate current slot bounds
    if (handle->device_manager.current_device_slot >= BM70_MAX_PAIRED_DEVICES) {
        LOG_WARNING("Current device slot corrupted: %d, resetting to 0",
                    handle->device_manager.current_device_slot);
        handle->device_manager.current_device_slot = 0;
    }

    // Find next valid device slot
    uint8_t start_slot = handle->device_manager.current_device_slot;
    uint8_t attempts = 0;
    uint8_t next_slot = start_slot;

    do {
        next_slot = (next_slot + 1) % BM70_MAX_PAIRED_DEVICES;
        attempts++;

        // Check if this slot has a valid device
        if (next_slot < handle->device_manager.device_count &&
            handle->device_manager.devices[next_slot].is_valid) {
            handle->device_manager.current_device_slot = next_slot;
            LOG_INFO("device slot %d", next_slot);
            return BM70_OK;
        }

        // Prevent infinite loop
        if (attempts >= BM70_MAX_PAIRED_DEVICES) {
            break;
        }
    } while (next_slot != start_slot);

    LOG_ERROR("No valid devices found for cycling");
    return BM70_ERROR_INVALID_PARAM;
}

bm70_error_t bm70_disconnect_for_switch(bm70_handle_t* handle)
{
    if (!handle || !handle->initialized) {
        return BM70_ERROR_INVALID_PARAM;
    }

    if (handle->conn_state != BM70_CONN_STATE_CONNECTED) {
        return BM70_ERROR_NOT_CONNECTED;
    }

    bm70_error_t err = bm70_disconnect(handle);
    if (err != BM70_OK) {
        return err;
    }

    return err;
}

bm70_error_t bm70_switch_to_next_device(bm70_handle_t* handle)
{
    if (!handle || !handle->initialized) {
        return BM70_ERROR_INVALID_PARAM;
    }

    if (handle->device_manager.switch_device) {
		LOG_DEBUG("Device switch already in progress");
		return BM70_ERROR_BUSY;
	}

    if (handle->hid_state.mtu_received_time != 0) {
		uint32_t time_since_mtu = HAL_GetTick() - handle->hid_state.mtu_received_time;
		if (time_since_mtu < 1000) {  // Wait 1 second after MTU update
			LOG_DEBUG("Connection stabilizing, deferring device switch");
			return BM70_ERROR_BUSY;
		}
	}

    // Validate device manager state
	if (!handle->device_manager.devices_loaded || handle->device_manager.device_count == 0) {
		LOG_WARNING("No paired devices available for switching");
		return BM70_ERROR_INVALID_PARAM;
	}

    // Step 2: Cycle to next device
    bm70_error_t err = bm70_cycle_to_next_device(handle);
    if (err != BM70_OK) {
    	LOG_ERROR("Failed to cycle to next device: %s", bm70_error_to_string(err));
        return err;
    }

    // Step 3: Set directed advertising parameters for the new device
    err = bm70_set_directed_advertising_to_device(handle, handle->device_manager.current_device_slot);
    if (err != BM70_OK) {
    	LOG_ERROR("Failed to set directed advertising: %s", bm70_error_to_string(err));
        return err;
    }

    handle->device_manager.switch_device = true;
    // Step 1: Disconnect current device (if connected)
    if (handle->conn_state == BM70_CONN_STATE_CONNECTED) {
        err = bm70_disconnect_for_switch(handle);
        if (err != BM70_OK) {
        	LOG_ERROR("Failed to disconnect for switch: %s", bm70_error_to_string(err));
        	handle->device_manager.switch_device = true;
            return err;
        }
    }

    return BM70_OK;
}

bm70_error_t bm70_enter_pairing_mode(bm70_handle_t* handle, uint16_t discoverable_timeout)
{
	if (!handle || !handle->initialized) {
		return BM70_ERROR_INVALID_PARAM;
	}

	// Load current device list first
	bm70_error_t err = bm70_load_paired_devices(handle);
	if (err != BM70_OK) {
		LOG_WARNING("Failed to load paired devices: %s", bm70_error_to_string(err));
	}

	handle->device_manager.previous_device_slot = handle->device_manager.current_device_slot;
	// Check if array is full and handle it BEFORE pairing
	if (handle->device_manager.device_count >= BM70_MAX_PAIRED_DEVICES) {
		LOG_INFO("Device array full (%d devices), making room for new device...",
				 handle->device_manager.device_count);

		err = handle_device_array_full(handle);
		if (err != BM70_OK) {
			LOG_ERROR("Failed to make room for new device: %s", bm70_error_to_string(err));
			return err;
		}

		// Reload device list after deletion
		err = bm70_load_paired_devices(handle);
		if (err != BM70_OK) {
			LOG_WARNING("Failed to reload devices after deletion: %s", bm70_error_to_string(err));
		}

		LOG_INFO("Made room for new device, now have %d devices",
				 handle->device_manager.device_count);
	}

	LOG_INFO("Entering pairing mode (timeout: %d seconds)", discoverable_timeout);

	// If currently connected, disconnect first
	if (handle->conn_state == BM70_CONN_STATE_CONNECTED) {
		LOG_INFO("Disconnecting current device to enter pairing mode");
		err = bm70_disconnect(handle);
		if (err != BM70_OK) {
			LOG_WARNING("Failed to disconnect: %s", bm70_error_to_string(err));
		}
		HAL_Delay(200); // Wait for disconnect to complete
	}

	// Set to standard advertising for maximum discoverability
	err = bm70_set_standard_advertising(handle);
	if (err != BM70_OK) {
		LOG_ERROR("Failed to set standard advertising: %s", bm70_error_to_string(err));
		return err;
	}

	// Enable pairing mode flags
	handle->device_manager.pairing_mode_active = true;
	handle->device_manager.pairing_timeout = discoverable_timeout;
	handle->device_manager.pairing_start_time = HAL_GetTick();
	handle->device_manager.allow_new_connections = true;

	// Start advertising to make device discoverable
	err = bm70_start_advertising(handle);
	if (err != BM70_OK) {
		LOG_ERROR("Failed to start advertising for pairing: %s", bm70_error_to_string(err));
		handle->device_manager.pairing_mode_active = false;
		return err;
	}

	LOG_INFO("Pairing mode active - keyboard discoverable for %d seconds", discoverable_timeout);
	return BM70_OK;

}

bm70_error_t bm70_exit_pairing_mode(bm70_handle_t* handle)
{
    if (!handle || !handle->initialized) {
        return BM70_ERROR_INVALID_PARAM;
    }

    if (!handle->device_manager.pairing_mode_active) {
        LOG_DEBUG("Pairing mode not active");
        return BM70_OK;
    }

    LOG_INFO("Exiting pairing mode");

    // Clear pairing mode flags
    handle->device_manager.pairing_mode_active = false;
    handle->device_manager.allow_new_connections = false;
    handle->device_manager.pairing_timeout = 0;
    handle->device_manager.pairing_start_time = 0;

    // Reload paired device list (may have new device)
    bm70_error_t err = bm70_load_paired_devices(handle);
    if (err != BM70_OK) {
        LOG_WARNING("Failed to reload paired devices: %s", bm70_error_to_string(err));
    }

    err = bm70_stop_advertising(handle);
    if (err != BM70_OK) {
		LOG_WARNING("Failed to reload stop advertising: %s", bm70_error_to_string(err));
	}
    LOG_INFO("✓ Pairing mode exited");
    return BM70_OK;
}

bm70_error_t handle_device_array_full(bm70_handle_t* handle)
{
    if (!handle || handle->device_manager.device_count < BM70_MAX_PAIRED_DEVICES) {
        return BM70_OK;  // Array not full
    }

    LOG_INFO("Device array full (%d devices), removing last device",
             BM70_MAX_PAIRED_DEVICES);

    // Simple strategy: remove the last device in the array
    uint8_t last_slot = handle->device_manager.device_count - 1;
    bm70_paired_device_t* device_to_remove = &handle->device_manager.devices[last_slot];

    LOG_INFO("Removing last device: Idx=%d, Addr=%02X:%02X:%02X:%02X:%02X:%02X",
             device_to_remove->device_index,
             device_to_remove->bd_addr[0], device_to_remove->bd_addr[1],
             device_to_remove->bd_addr[2], device_to_remove->bd_addr[3],
             device_to_remove->bd_addr[4], device_to_remove->bd_addr[5]);

    // Delete from BM71 module
    bm70_error_t err = bm70_delete_paired_device(handle, device_to_remove->device_index);
    if (err != BM70_OK) {
        LOG_ERROR("Failed to delete last device: %s", bm70_error_to_string(err));
        return err;
    }

    LOG_INFO("Successfully removed last device, reloading device list...");
    return BM70_OK;
}

bool bm70_check_pairing_timeout(bm70_handle_t* handle)
{
    if (!handle || !handle->device_manager.pairing_mode_active) {
        return false;  // No pairing mode active, continue normal operations
    }

    // Check timeout (0 means no timeout)
    if (handle->device_manager.pairing_timeout == 0) {
        return false;  // No timeout set, continue normal operations
    }

    uint32_t elapsed = (HAL_GetTick() - handle->device_manager.pairing_start_time) / 1000;
    if (elapsed >= handle->device_manager.pairing_timeout) {
        LOG_INFO("Pairing timeout (%d seconds) - exiting pairing mode",
                 handle->device_manager.pairing_timeout);

        // Exit pairing mode
        handle->device_manager.pairing_mode_active = false;
        handle->device_manager.allow_new_connections = false;
        handle->device_manager.pairing_timeout = 0;
        handle->device_manager.pairing_start_time = 0;

        // Reload paired device list (may have new device)
        bm70_error_t err = bm70_load_paired_devices(handle);
        if (err != BM70_OK) {
            LOG_WARNING("Failed to reload paired devices: %s", bm70_error_to_string(err));
        }

        // *** RESTORE to the previous device slot (the one we were connected to before pairing) ***
        if (handle->device_manager.device_count > 0) {
            // Validate previous device slot is still valid
            if (handle->device_manager.previous_device_slot < handle->device_manager.device_count &&
                handle->device_manager.devices[handle->device_manager.previous_device_slot].is_valid) {

                handle->device_manager.current_device_slot = handle->device_manager.previous_device_slot;
                LOG_INFO("Restoring to previous device slot: %d", handle->device_manager.previous_device_slot);
            } else {
                LOG_WARNING("Previous device slot %d no longer valid, using slot 0",
                           handle->device_manager.previous_device_slot);
                handle->device_manager.current_device_slot = 0;
            }

            // *** USE bm70_start_directed_advertising for directed advertising ***
            LOG_INFO("Setting up directed advertising to reconnect to previous device");
            err = bm70_set_directed_advertising_to_device(handle,
                handle->device_manager.current_device_slot);
            if (err == BM70_OK) {
                // Use directed advertising start function
                err = bm70_start_directed_advertising(handle);
                if (err != BM70_OK) {
                    LOG_WARNING("Directed advertising failed, falling back to standard: %s",
                               bm70_error_to_string(err));
                    bm70_set_standard_advertising(handle);
                    bm70_start_advertising(handle);
                }
            } else {
                // Fallback to standard advertising
                LOG_WARNING("Failed to set directed advertising, using standard: %s",
                           bm70_error_to_string(err));
                bm70_set_standard_advertising(handle);
                bm70_start_advertising(handle);
            }
        } else {
            // No paired devices, use standard advertising
            LOG_INFO("No paired devices available, using standard advertising");
            bm70_set_standard_advertising(handle);
            bm70_start_advertising(handle);
        }

        return true;  // Timeout handled, skip other periodic operations
    }

    return false;  // Still in pairing mode but no timeout yet, continue normal operations
}
