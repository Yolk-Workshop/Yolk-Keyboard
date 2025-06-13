/*
 * kb_ble_api.c
 *
 *  Created on: May 14, 2025
 *      Author: bettysidepiece
 */
#include "effects.h"
#include "kb_driver.h"
#include "bm71_config.h"
#include "kb_ble_api.h"
#include "logger.h"
#include "main.h"

typedef enum {
    BLE_CHECK_ACTIVE_MODE = 0,     // Normal active operation
    BLE_CHECK_SLEEP_MODE = 1,      // Minimal checks for low power
    BLE_CHECK_MINIMAL_MODE = 2     // Absolute minimum (emergency only)
} ble_check_mode_t;

// External keyboard state
extern keyboard_state_t kb_state;

// BM70 handle
bm70_handle_t g_bm70;
uint8_t pmsm_mode;

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


static int8_t find_device_by_address(bm70_handle_t* handle, const uint8_t* peer_addr) {
    if (!handle || !peer_addr || !handle->device_manager.devices_loaded) {
        return -1;
    }

    for (uint8_t i = 0; i < handle->device_manager.device_count; i++) {
        if (handle->device_manager.devices[i].is_valid &&
            memcmp(handle->device_manager.devices[i].bd_addr, peer_addr, BM70_BT_ADDR_LEN) == 0) {
            return (int8_t)i;
        }
    }
    return -1; // Device not found
}

static void add_device_to_list(bm70_handle_t* handle, const bm70_conn_info_t* info) {
    if (!handle || !info) {
        LOG_ERROR("Invalid parameters for add_device_to_list");
        return;
    }

    if (handle->device_manager.device_count >= BM70_MAX_PAIRED_DEVICES) {
        LOG_WARNING("Device array is full (%d/%d) - cannot add new device",
                   handle->device_manager.device_count, BM70_MAX_PAIRED_DEVICES);
        return;
    }

    uint8_t slot = handle->device_manager.device_count;
    bm70_paired_device_t* device = &handle->device_manager.devices[slot];

    // Clear current flag from all other devices
    for (uint8_t i = 0; i < handle->device_manager.device_count; i++) {
        handle->device_manager.devices[i].is_current = false;
    }

    // Add basic info
    memcpy(device->bd_addr, info->peer_addr, BM70_BT_ADDR_LEN);
    device->addr_type = info->peer_addr_type;
    device->is_valid = true;
    device->is_current = true;
    device->last_connected = HAL_GetTick();
    device->device_index = 0xFF; // Unknown until we reload from BM71
    device->priority = 0; // Will be updated from BM71

    handle->device_manager.device_count++;
    handle->device_manager.current_device_slot = slot;

    LOG_INFO("Added new device to local list at slot %d (total: %d/%d)",
             slot, handle->device_manager.device_count, BM70_MAX_PAIRED_DEVICES);
}

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
			Effects_BLE_ConnectionStatus(ble_conn_flag);
			if (info && info->handle != 0) {
				memcpy(&g_bm70.conn_info, info, sizeof(bm70_conn_info_t));
				g_bm70.hid_state.service_ready = true;

				// Check if we know this device
				int8_t device_slot = find_device_by_address(&g_bm70, info->peer_addr);

				if (device_slot >= 0) {
					for (uint8_t i = 0; i < g_bm70.device_manager.device_count; i++) {
						g_bm70.device_manager.devices[i].is_current = false;
					}
					// Known device
					LOG_INFO("Connected to known device at slot %d", device_slot);
					g_bm70.device_manager.current_device_slot = (uint8_t)device_slot;
					g_bm70.device_manager.devices[device_slot].is_current = true;
				} else {
					// Unknown device
					LOG_INFO("Connected to unknown device - adding to list");
					add_device_to_list(&g_bm70, info);
				}
			}

			if (g_bm70.device_manager.pairing_mode_active) {
				LOG_INFO("New device connected - automatically exiting pairing mode");

				// Clear pairing mode flags
				g_bm70.device_manager.pairing_mode_active = false;
				g_bm70.device_manager.allow_new_connections = false;
				g_bm70.device_manager.pairing_timeout = 0;
				g_bm70.device_manager.pairing_start_time = 0;
				g_bm70.device_manager.pairing_mode_active = 0;

				// Reload device list (new device should now be in the list)
				bm70_error_t err = bm70_load_paired_devices(&g_bm70);
				if (err != BM70_OK) {
					LOG_WARNING("Failed to reload paired devices after new connection: %s",
								bm70_error_to_string(err));
				}

				LOG_INFO("Pairing mode automatically exited due to new connection");
			}

            break;

        case BM70_CONN_STATE_DISCONNECTED:
            LOG_INFO("BLE disconnected");
            ble_conn_flag = false;
            Effects_BLE_ConnectionStatus(ble_conn_flag);
            for (uint8_t i = 0; i < g_bm70.device_manager.device_count; i++) {
				g_bm70.device_manager.devices[i].is_current = false;
			}
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

    if ((HAL_GetTick() - last_send_time) < 2) {
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

   uint32_t battery_interval = 0;
   uint32_t health_check_interval  = 0;
   uint32_t advertising_cooldown = 0;

   static uint32_t last_advertising_attempt = 0;
   static uint32_t last_battery_update = 0;
   static uint32_t last_connection_test = 0;
   static uint8_t consecutive_failures = 0;
   static uint8_t idle_state_count = 0;
   static uint8_t advertising_failure_count = 0;

   uint32_t current_time = HAL_GetTick();

   if (!g_bm70.initialized) {
       return;
   }

   if (bm70_check_pairing_timeout(&g_bm70)) {
	  LOG_DEBUG("Pairing timeout handled, skipping other periodic operations");
	  bm70_process_rx(&g_bm70);
	  return;  // Pairing timeout was handled, don't do other BLE operations
  }

   switch (pmsm_mode) {
          case BLE_CHECK_ACTIVE_MODE:
              battery_interval = 45000;      // 45 seconds
              health_check_interval = 30000;  // 30 seconds
              advertising_cooldown = 1000;    // 1 seconds
              break;

          case BLE_CHECK_SLEEP_MODE:
              battery_interval = 900000;      // 15 minutes
              health_check_interval = 600000; // 10 minutes
              advertising_cooldown = 30000;   // 30 seconds
              break;
      }

   bm70_process_rx(&g_bm70);

   if (ble_conn_flag &&
   	g_bm70.conn_state == BM70_CONN_STATE_CONNECTED &&
   	(current_time - last_connection_test > health_check_interval)) {

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

   // Consolidated advertising logic
   if (!ble_is_connected() && !ble_is_advertising() &&
       (current_time - last_advertising_attempt > advertising_cooldown)) {

       uint8_t trigger = 0;

       // Check trigger conditions
       if (g_bm70.hid_state.mtu_received_time > 0 &&
           current_time > g_bm70.hid_state.mtu_received_time) {
           trigger = 1; // Scheduled restart
       }
       else if (g_bm70.status == BM70_STATUS_IDLE && !ble_is_connected() && idle_state_count >= 15) {
           trigger = 2; // Stuck in idle
           idle_state_count = 0;
       }
       else if ((current_time - g_last_status_event_time) > advertising_cooldown) {
           trigger = 3; // No recent status
       }

       if (trigger > 0) {
           LOG_INFO("Starting advertising (trigger %d)", trigger);
           bm70_error_t err = bm70_start_advertising(&g_bm70);

           if (err == BM70_OK) {
               advertising_failure_count = 0;
               g_bm70.hid_state.mtu_received_time = 0;
           } else {
               advertising_failure_count++;
               if (advertising_failure_count >= 10) {
                   bm70_hwreset_with_handle(&g_bm70);
                   advertising_failure_count = 0;
                   g_bm70.hid_state.mtu_received_time = 0;
               }
           }
           last_advertising_attempt = current_time;
       }
   }

   // Update battery level periodically when connected
   if (ble_conn_flag && (current_time - last_battery_update > battery_interval)) {
   	static uint8_t last_sent_level = 0xFF;
   	//TODO: Implement actual battery read for update
   	g_bm70.hid_state.battery_level = 85;

   	if(g_bm70.hid_state.battery_level > 0x64) {
   		g_bm70.hid_state.battery_level = 0x64;
   	}

       if (g_bm70.hid_state.battery_level != last_sent_level) {
           bm70_error_t err = bm70_update_battery_level(&g_bm70, g_bm70.hid_state.battery_level);
           if (err == BM70_OK) {
               last_sent_level = g_bm70.hid_state.battery_level;
               LOG_DEBUG("Battery level updated: %d%%", g_bm70.hid_state.battery_level);
           }
       }
       last_battery_update = HAL_GetTick();
   }

   // Track idle state
   if (g_bm70.status == BM70_STATUS_IDLE && !ble_is_connected()) {
       idle_state_count++;
   } else {
       idle_state_count = 0;
   }

}

bool ble_enter_pairing_mode(uint16_t timeout_seconds) {
    if (!g_bm70.initialized) {
        LOG_ERROR("BM70 not initialized");
        return false;
    }

    LOG_INFO("User requested pairing mode for %d seconds", timeout_seconds);
    bm70_error_t err = bm70_enter_pairing_mode(&g_bm70, timeout_seconds);
    return (err == BM70_OK);
}

bool ble_switch_to_next_device(void) {
    if (!g_bm70.initialized) {
        LOG_ERROR("BM70 not initialized");
        return false;
    }

    LOG_INFO("User requested device switch");
    bm70_error_t err = bm70_switch_to_next_device(&g_bm70);
    return (err == BM70_OK);
}

bool ble_is_pairing_mode(void) {
    return g_bm70.initialized && g_bm70.device_manager.pairing_mode_active;
}

uint32_t ble_get_pairing_time_remaining(void) {
    if (!g_bm70.initialized || !g_bm70.device_manager.pairing_mode_active) {
        return 0;
    }

    uint32_t elapsed = (HAL_GetTick() - g_bm70.device_manager.pairing_start_time) / 1000;
    if (elapsed >= g_bm70.device_manager.pairing_timeout) {
        return 0;
    }

    return g_bm70.device_manager.pairing_timeout - elapsed;
}

bool ble_exit_pairing_mode(void) {
    if (!g_bm70.initialized) {
        LOG_ERROR("BM70 not initialized");
        return false;
    }

    if (!g_bm70.device_manager.pairing_mode_active) {
        LOG_DEBUG("Not in pairing mode");
        return true;
    }

    LOG_INFO("User requested exit pairing mode");
    bm70_error_t err = bm70_exit_pairing_mode(&g_bm70);
    return (err == BM70_OK);
}

uint8_t ble_get_paired_device_count(void) {
    if (!g_bm70.initialized || !g_bm70.device_manager.devices_loaded) {
        return 0;
    }

    return g_bm70.device_manager.device_count;
}
