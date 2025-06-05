/*
 * kb_ble_api.c
 *
 *  Created on: May 14, 2025
 *      Author: bettysidepiece
 */
#include "kb_driver.h"
#include "bm71_config.h"
#include "kb_ble_api.h"
#include "logger.h"
#include "main.h"

// External keyboard state
extern keyboard_state_t kb_state;

// BM70 handle
bm70_handle_t g_bm70;

// Connection state
extern volatile uint8_t ble_conn_flag;
uint32_t g_last_status_event_time = 0;

// UART interface implementation

static bool ble_data_ready_wrapper(void) {
    return (bool)ble_rx_message_ready();
}

static bool ble_rx_not_empty_wrapper(void) {
    return (bool)ble_rx_available();
}

static bool ble_transmit_ready_wrapper(void) {
    return (bool)ble_write_ready();
}

#if !BLE_CONFIG
static const bm70_uart_interface_t uart_interface = {
    .write = ble_uart_tx,
    .read = ble_rx_read,
    .data_ready = ble_data_ready_wrapper,
    .transmit_ready = ble_transmit_ready_wrapper,
    .flush_rxbuffer = ble_rx_flush,
    .rx_not_empty = ble_rx_not_empty_wrapper
};
#endif


static void on_connection_change(bm70_conn_state_t state, const bm70_conn_info_t* info) {
    switch (state) {
        case BM70_CONN_STATE_ADVERTISING:
            LOG_INFO("BLE advertising started");
            ble_conn_flag = false;
            break;

        case BM70_CONN_STATE_CONNECTED:
            if (info) {
                LOG_INFO("BLE connected to %02X:%02X:%02X:%02X:%02X:%02X (Handle: 0x%02X)",
                        info->peer_addr[5], info->peer_addr[4], info->peer_addr[3],
                        info->peer_addr[2], info->peer_addr[1], info->peer_addr[0],
                        info->handle);
            	}
				ble_conn_flag = true;
				g_bm70.conn_state = BM70_CONN_STATE_CONNECTED;
				g_bm70.status = BM70_STATUS_CONNECTED;  // Also sync status

				if (info && info->handle != 0) {
					memcpy(&g_bm70.conn_info, info, sizeof(bm70_conn_info_t));
					g_bm70.hid_state.service_ready = true;  // Mark HID ready
				}

            break;

        case BM70_CONN_STATE_DISCONNECTED:
            LOG_INFO("BLE disconnected");
            ble_conn_flag = false;
            break;

        default:
            LOG_DEBUG("BLE state change: %s", bm70_conn_state_to_string(state));
            break;
    }
}

// Status callback
static void on_status_change(bm70_status_t status) {
    LOG_DEBUG("BM70 status: 0x%02X", status);
}

// Data callback for HID output reports (LED status)
static void on_data_received(const uint8_t* data, uint16_t len) {
    if (len > 0) {
        // Update LED status in keyboard state
        kb_state.led_status = data[0];
        LOG_DEBUG("LED status: 0x%02X", kb_state.led_status);
    }
}

// Error callback
static void on_error(bm70_error_t error) {
    LOG_ERROR("BM70 error: %s", bm70_error_to_string(error));
}

// Initialize Bluetooth - called from kb_driver.c
void initBluetooth(void) {
    LOG_INFO("Initialising Bluetooth HID with comprehensive setup");

    // Initialize UART
    MX_LPUART1_UART_Init();
    /* Enable RX‐not‐empty and IDLE‐line interrupts */
    LPUART1->CR1 |= (USART_CR1_RXNEIE | USART_CR1_IDLEIE);

#if BLE_CONFIG
    HAL_GPIO_WritePin(BT_RESET_GPIO_Port, BT_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(BT_RESET_GPIO_Port, BT_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
    // Configuration mode - flash the module with HID configuration
    LOG_INFO("BLE_CONFIG mode - Configuring BM71 module");

    bm71_config_error_t err_cfg = bm71_config_init(&hlpuart1);
    if (err_cfg != BM71_CONFIG_OK) {
        LOG_ERROR("BM71 UART config failed");
        while (1);
    }

    HAL_Delay(100); // Give UART time to stabilize

    LOG_INFO("Flashing BM71 with HID keyboard configuration...");
    bm71_configure_yolk_keyboard();

    // Set mode pin HIGH to enter application mode after configuration
    UART_MODE_SW_GPIO_Port->BSRR = UART_MODE_SW_Pin;

    // Reset module to apply configuration
    if (BT_RESET_GPIO_Port && BT_RESET_Pin) {
        HAL_GPIO_WritePin(BT_RESET_GPIO_Port, BT_RESET_Pin, GPIO_PIN_RESET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(BT_RESET_GPIO_Port, BT_RESET_Pin, GPIO_PIN_SET);
    }

    LOG_INFO("BM71 configuration completed - system will halt");
    while(1);

#else
    // Runtime mode - normal operation
    LOG_INFO("Runtime mode - Initializing BM70/71 for HID operation");
    tim7_ble_init();
    g_last_status_event_time = HAL_GetTick();
	LOG_INFO("BLE periodic status checking enabled");

    // Configure BM70 with proper enum type
    bm70_config_t config = {
        .huart = &hlpuart1,
        .rst_port = BT_RESET_GPIO_Port,
        .rst_pin = BT_RESET_Pin,
        .mode_port = UART_MODE_SW_GPIO_Port,  // P2_0 pin for mode selection
        .mode_pin = UART_MODE_SW_Pin,
        .wake_port = BT_WAKEUP_GPIO_Port,     // BLE wake-up pin
        .wake_pin = BLE_WAKEUP_Pin,
        .device_name = "Yolk Keyboard",
        .adv_interval = 100,                  // 100 * 0.625ms = 62.5ms
        .adv_timeout = 30,                    // 30 seconds
        .auto_reconnect = true,
        .io_capability = BM70_IO_CAP_NO_INPUT_OUTPUT,  // Use proper enum
        .adv_type = BM70_ADV_CONNECTABLE_UNDIRECTED  // Use proper enum
    };

    // Initialize BM70 with comprehensive setup
    bm70_error_t err = bm70_init_with_uart(&g_bm70, &config, &uart_interface);
    if (err != BM70_OK) {
        LOG_ERROR("Failed to initialize BM70: %s", bm70_error_to_string(err));
        LOG_ERROR("Check connections and module configuration");
        return;
    }
    LOG_INFO("BM70/71 initialized successfully");

    // Set up callbacks for event handling
    bm70_set_connection_callback(&g_bm70, on_connection_change);
    bm70_set_status_callback(&g_bm70, on_status_change);
    bm70_set_data_callback(&g_bm70, on_data_received);
    bm70_set_error_callback(&g_bm70, on_error);

    LOG_INFO("Starting BLE advertising...");
    err = bm70_start_advertising(&g_bm70);
    if (err != BM70_OK) {
        LOG_ERROR("Failed to start advertising: %s", bm70_error_to_string(err));
        LOG_WARNING("Try restarting or check module configuration");
    } else {
        LOG_INFO("BLE advertising started - keyboard is discoverable");
        LOG_INFO("Device name: %s", config.device_name);
        LOG_INFO("Ready for BLE HID connections");
    }

#endif
}

// Send BLE report - called from kb_driver.c
// Modify your existing sendBLEReport() in kb_ble_api.c
void sendBLEReport(void)
{
    static boot_keyboard_report_t last_ble_keyboard = { 0 };
    static consumer_report_t last_ble_consumer = { 0 };
    static uint32_t last_failed_time = 0;
    static uint32_t last_send_time = 0;
    uint32_t current_time = getMicroseconds();

    // Pre-flight checks
    if (!ble_conn_flag || g_bm70.conn_state != BM70_CONN_STATE_CONNECTED) {
    	LOG_WARNING("Not in ideal connection state");
        return;
    }

    if ((HAL_GetTick() - last_send_time) < 5) {
		return;
	}

    // Skip sending if recent failures detected
    if ((HAL_GetTick() - last_failed_time) < 500) { // 500 millisecond cooldown
    	LOG_WARNING("Skipping send");
        return;
    }

    if (g_bm70.conn_info.handle == 0) {
        LOG_WARNING("Invalid connection handle: 0x%02X", g_bm70.conn_info.handle);
        return;
    }

    // Send keyboard report if changed
    if (memcmp(&kb_state.boot_report, &last_ble_keyboard, sizeof(boot_keyboard_report_t)) != 0) {
        bm70_error_t err = bm70_send_char_value(&g_bm70, HID_KB_INPUT_HANDLE,
            (uint8_t*)&kb_state.boot_report, sizeof(boot_keyboard_report_t));

        if (err == BM70_OK) {
            kb_state.last_report.ble_last_report = current_time;
            memcpy(&last_ble_keyboard, &kb_state.boot_report, sizeof(boot_keyboard_report_t));
            last_send_time = HAL_GetTick();

            LOG_DEBUG("BLE Keyboard Report Sent: Mod=0x%02X, Key[0]=0x%02X",
                kb_state.boot_report.modifiers, kb_state.boot_report.keys[0]);

        } else if (err == BM70_ERROR_TIMEOUT) {
            LOG_WARNING("BLE Keyboard Report Failed: %s", bm70_error_to_string(err));
            last_failed_time = HAL_GetTick();
            // The periodic health check will detect multiple failures
        }
    }

    // Send consumer report if changed (similar logic)
    if (memcmp(&kb_state.consumer_report, &last_ble_consumer, sizeof(consumer_report_t)) != 0) {
        bm70_error_t err = bm70_send_char_value(&g_bm70, HID_CONSUMER_INPUT_HANDLE,
            (uint8_t*)&kb_state.consumer_report, sizeof(consumer_report_t));

        if (err == BM70_OK) {

            memcpy(&last_ble_consumer, &kb_state.consumer_report, sizeof(consumer_report_t));
            last_send_time = HAL_GetTick();

            LOG_DEBUG("BLE Consumer Report Sent: Usage=0x%02X%02X",
                kb_state.consumer_report.buttons[0], kb_state.consumer_report.buttons[1]);

        } else if (err == BM70_ERROR_TIMEOUT) {
            LOG_WARNING("BLE Consumer Report Failed: %s", bm70_error_to_string(err));
            last_failed_time = HAL_GetTick();
        }
    }
}


// Additional BLE control functions
void ble_disconnect(void) {
    if (ble_conn_flag) {
        bm70_disconnect(&g_bm70);
    }
}

bool ble_is_connected(void) {
    return (g_bm70.initialized &&
            g_bm70.conn_state == BM70_CONN_STATE_CONNECTED &&
            g_bm70.conn_info.handle != 0 &&
            ble_conn_flag);
}

void ble_restart_advertising(void) {
    if (g_bm70.initialized && !ble_is_connected() && !ble_is_advertising()) {
        LOG_INFO("Restarting BLE advertising...");
        bm70_start_advertising(&g_bm70);
    }
}

bool ble_is_advertising(void) {
    return (g_bm70.initialized &&
            g_bm70.conn_state == BM70_CONN_STATE_ADVERTISING);
}

void ble_deinit(void) {
    tim7_ble_stop();
    bm70_deinit(&g_bm70);
    ble_conn_flag = false;
}

bool ble_is_responsive(void)
{
	if (!g_bm70.initialized)return false;
    uint32_t time_since_status = HAL_GetTick() - g_last_status_event_time;
    return (time_since_status < 60000);
}

uint32_t ble_get_last_status_time(void) {
    return g_last_status_event_time;
}

/* ============================================================================
 * Additional Helper Functions
 * ============================================================================ */

// Enhanced connection verification (replaces the problematic ble_verify_connection)
bool ble_verify_connection(void) {
    // Simple software-based connection verification
    if (!ble_conn_flag) {
        return false;
    }

    // Additional verification: check if BM70 handle indicates connected state
    if (g_bm70.initialized && g_bm70.conn_state == BM70_CONN_STATE_CONNECTED) {
        return true;
    }

    return ble_conn_flag; // Fallback to software flag
}

// Optional: Add a function to get detailed connection info
void ble_get_connection_info(void) {
    if (!g_bm70.initialized) {
        LOG_INFO("BM70 not initialized");
        return;
    }

    LOG_INFO("============ BLE Connection Status ============");
    LOG_INFO("Module Status: %s", bm70_status_to_string(g_bm70.status));
    LOG_INFO("Connection State: %s", bm70_conn_state_to_string(g_bm70.conn_state));
    LOG_INFO("HID Ready: %s", bm70_is_hid_ready(&g_bm70) ? "Yes" : "No");

    if (g_bm70.conn_state == BM70_CONN_STATE_CONNECTED) {
        LOG_INFO("Connected to: %02X:%02X:%02X:%02X:%02X:%02X",
                g_bm70.conn_info.peer_addr[0], g_bm70.conn_info.peer_addr[1],
                g_bm70.conn_info.peer_addr[2], g_bm70.conn_info.peer_addr[3],
                g_bm70.conn_info.peer_addr[4], g_bm70.conn_info.peer_addr[5]);
        LOG_INFO("Connection Handle: 0x%02X", g_bm70.conn_info.handle);
        LOG_INFO("Role: %s", g_bm70.conn_info.role ? "Slave" : "Master");
    }
    LOG_INFO("=============================================");
}

// Optional: Manual configuration verification function
void ble_verify_module_config(void) {
    if (!g_bm70.initialized) {
        LOG_WARNING("BM70 not initialized - cannot verify configuration");
        return;
    }

    LOG_INFO("Starting manual configuration verification...");
    bm70_error_t err = bm70_verify_configuration(&g_bm70);
    if (err == BM70_OK) {
        LOG_INFO("Configuration verification completed successfully");
    } else {
        LOG_WARNING("Configuration verification failed: %s", bm70_error_to_string(err));
    }
}

void ble_periodic_status_check(void) {
    static uint32_t last_advertising_attempt = 0;
    static uint32_t last_battery_update = 0;
    static uint32_t last_connection_test = 0;
	static uint8_t consecutive_failures = 0;
	static uint8_t idle_state_count = 0;
	uint32_t current_time = HAL_GetTick();

    if (!g_bm70.initialized) {
        return;
    }

    bm70_process_rx(&g_bm70);

    if (ble_conn_flag &&
		g_bm70.conn_state == BM70_CONN_STATE_CONNECTED &&
		(current_time - last_connection_test > 90000)) { // Every 90 seconds

		bm70_status_t status;
		bm70_error_t err = bm70_get_status(&g_bm70, &status);

		if (err == BM70_ERROR_TIMEOUT) {
			consecutive_failures++;
			LOG_WARNING("Connection test failed (%d/3) - %s", consecutive_failures, bm70_error_to_string(err));

			// After 3 consecutive failures, assume silent disconnect
			if (consecutive_failures >= 3) {
				LOG_ERROR("Silent disconnect detected - cleaning up connection state");

				// Force disconnect cleanup
				ble_conn_flag = false;
				g_bm70.conn_state = BM70_CONN_STATE_DISCONNECTED;
				memset(&g_bm70.conn_info, 0, sizeof(g_bm70.conn_info));
				g_bm70.hid_state.service_ready = false;
				g_bm70.hid_state.notifications_enabled = false;

				// Schedule restart
				g_bm70.hid_state.mtu_received_time = HAL_GetTick() + 2000;
				consecutive_failures = 0;
			}
		} else if (err == BM70_OK) {
			consecutive_failures = 0; // Reset on success
		}

		last_connection_test = current_time;
	}

    if (!ble_is_connected() &&
		!ble_is_advertising() &&
		g_bm70.hid_state.mtu_received_time > 0 &&
		current_time > g_bm70.hid_state.mtu_received_time &&
		(current_time - last_advertising_attempt > 5000)) {

		LOG_INFO("Restarting advertising after connection cleanup");
		bm70_start_advertising(&g_bm70);
		g_bm70.hid_state.mtu_received_time = 0; // Clear restart flag
		last_advertising_attempt = current_time;
	}

    // Handle post-MTU connection setup
    if (g_bm70.hid_state.waiting_for_conn_update &&
		g_bm70.hid_state.mtu_received_time > 0 &&  // Valid timestamp
		(current_time - g_bm70.hid_state.mtu_received_time > 10000)) { // 10 second timeout

        if (!g_bm70.hid_state.consumer_cccd_enabled) {
            LOG_INFO("Manually enabling consumer CCCD");
            bm70_enable_cccd_notifications(&g_bm70, HID_CONSUMER_INPUT_CCCD_HANDLE);
        }

        if (!g_bm70.hid_state.keyboard_cccd_enabled) {
            LOG_INFO("Manually enabling keyboard CCCD");
            bm70_enable_cccd_notifications(&g_bm70, HID_KB_INPUT_CCCD_HANDLE);
        }

        if (!g_bm70.hid_state.battery_cccd_enabled) {
            LOG_INFO("Manually enabling battery CCCD");
            bm70_enable_cccd_notifications(&g_bm70, BATTERY_LEVEL_CCCD);
        }

        g_bm70.hid_state.waiting_for_conn_update = false;
    }

    // Check if we've received any status events recently
    uint32_t time_since_last_status = current_time - g_last_status_event_time;

    // Update battery level periodically when connected (every 2 minutes)
    if (ble_conn_flag && (current_time - last_battery_update > 120000)) {
    	static uint8_t last_sent_level = 0xFF;
    	//TODO: Implement actual battery update code <--
    	g_bm70.hid_state.battery_level = 85; // Your actual battery reading

    	if(g_bm70.hid_state.battery_level > 0x64) {
    		g_bm70.hid_state.battery_level = 0x64;
    	}

        // Only send if level changed
        if (g_bm70.hid_state.battery_level != last_sent_level) {
            bm70_error_t err = bm70_update_battery_level(&g_bm70, g_bm70.hid_state.battery_level);
            if (err == BM70_OK) {
                last_sent_level = g_bm70.hid_state.battery_level;
                LOG_DEBUG("Battery level updated: %d%%", g_bm70.hid_state.battery_level);
            }
        }
        last_battery_update = HAL_GetTick();
    }

    // Track if module is stuck in idle without connection
    if (g_bm70.status == BM70_STATUS_IDLE && !ble_is_connected()) {
        idle_state_count++;

        // If stuck in idle for too long, try advertising
        if (idle_state_count >= 3 && // 3 cycles (6 seconds) in idle
            (current_time - last_advertising_attempt > 10000)) {

            LOG_INFO("Module stuck in idle - attempting advertising");
            bm70_start_advertising(&g_bm70);
            last_advertising_attempt = current_time;
            idle_state_count = 0; // Reset counter
        }
    } else {
        idle_state_count = 0; // Reset if not idle or if connected
    }

    // If we haven't seen a status event in a while and not connected, try advertising
    if (!ble_is_connected() && !ble_is_advertising() &&
        time_since_last_status > 30000 && // 30 seconds without status
        (current_time - last_advertising_attempt > 15000)) { // Don't try too often

        LOG_INFO("No recent status events - attempting advertising");
        bm70_start_advertising(&g_bm70);
        last_advertising_attempt = current_time;
    }

}
