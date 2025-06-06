/*
 * auto_switch.c
 *
 *  Created on: Jun 6, 2025
 *      Author: bettysidepiece
 */
/* auto_switch.c - Complete implementation */

#include "auto_switch.h"
#include "kb_ble_api.h"
#include "usbd_core.h"

extern UART_HandleTypeDef hlpuart1;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern volatile uint8_t report_ready_flag;
extern volatile bool g_usb_suspended;

mode_switch_state_t switch_state = MODE_SWITCH_IDLE;
uint32_t switch_start_time = 0;

static void switch_connection_mode(connection_mode_t new_mode);

/* ============================================================================
 * Mode Shutdown Functions
 * ============================================================================ */

static void shutdown_ble_mode(void) {
    LOG_INFO("Shutting down BLE mode...");

    // Graceful BLE disconnect if connected
    if (ble_is_connected()) {
        LOG_DEBUG("Disconnecting BLE...");
        ble_disconnect();
        HAL_Delay(100); // Allow disconnect to complete
    }

    if (BT_RESET_GPIO_Port && BT_RESET_Pin) {
    	BT_RESET_GPIO_Port->BRR = BT_RESET_Pin;
		UART_MODE_SW_GPIO_Port->BRR = UART_MODE_SW_Pin;
	}

    HAL_UART_MspDeInit(&hlpuart1);

    // Use existing ble_deinit() - it handles everything properly
    ble_deinit(); // This calls tim7_ble_stop() and bm70_deinit()

    // Clear BLE-related state
    ble_conn_flag = false;
    memset(&g_bm70, 0, sizeof(g_bm70));

    LOG_INFO("BLE mode shutdown complete");
}

static void shutdown_usb_mode(void) {
    LOG_INFO("Shutting down USB mode...");

    // Deinitialize USB stack
    USBD_DeInit(&hUsbDeviceFS);

    // Disable USB peripheral clock
    __HAL_RCC_USB_CLK_DISABLE();

    NVIC_DisableIRQ(USB_IRQn);

    // Clear USB state
    g_usb_suspended = false;

    LOG_INFO("USB mode shutdown complete");
}

/* ============================================================================
 * Mode Initialization Functions
 * ============================================================================ */

static void init_ble_mode(void) {
    LOG_INFO("Initializing BLE mode...");

    // Initialize LPUART1 for BLE communication
    UART_MODE_SW_GPIO_Port->BSRR = UART_MODE_SW_Pin;
    HAL_UART_MspInit(&hlpuart1);

    // Small delay for UART to stabilize
    HAL_Delay(50);

    // Initialize BLE stack
    initBluetooth();

    LOG_INFO("BLE mode initialization complete");
}

static void init_usb_mode(void) {
    LOG_INFO("Initializing USB mode...");

    // Re-enable USB peripheral clock
    __HAL_RCC_USB_CLK_ENABLE();
    // Small delay for clock to stabilize
    HAL_Delay(10);
    // Initialize USB device
    MX_USB_DEVICE_Init();
    HAL_Delay(100);

    LOG_INFO("USB mode initialization complete");
}

/* ============================================================================
 * Mode Switching Logic
 * ============================================================================ */

static bool attempt_usb_enumeration(uint32_t timeout_ms) {
    LOG_INFO("Attempting USB enumeration...");

    // Initialize USB mode first
    init_usb_mode();

    uint32_t start_time = HAL_GetTick();

    // Block and wait for enumeration or timeout
    while ((HAL_GetTick() - start_time) < timeout_ms) {
        // Check if host assigned an address (enumeration success)
        uint8_t device_addr = USB->DADDR & 0x7F;
        if (device_addr > 0) {
            LOG_INFO("USB enumeration successful - address %d assigned", device_addr);
            return true;  // Real USB host detected
        }

        // Keep system alive while waiting
        WDG_Refresh();
        HAL_Delay(50);  // Small delay to prevent busy loop
    }

    LOG_INFO("USB enumeration timeout - no data host detected");
    return false;  // Just a charger
}

static void switch_connection_mode(connection_mode_t new_mode) {
    if (g_connection_mode == new_mode || switch_state == MODE_SWITCH_IN_PROGRESS) {
        return; // Already in mode or switch in progress
    }

    LOG_INFO("Mode switch: %s -> %s",
             g_connection_mode == CONNECTION_BLE ? "BLE" : "USB",
             new_mode == CONNECTION_BLE ? "BLE" : "USB");

    switch_state = MODE_SWITCH_IN_PROGRESS;
    switch_start_time = HAL_GetTick();

    // Record activity to prevent sleep during switch
    PM_RecordActivity();

    // Shutdown current mode
    if (g_connection_mode == CONNECTION_BLE) {
        shutdown_ble_mode();
    } else {
        shutdown_usb_mode();
    }

    // Handle USB mode with enumeration check
    if (new_mode == CONNECTION_USB) {
        // Test if it's a real USB host or just a charger
        if (attempt_usb_enumeration(2000)) {  // 2 second timeout
            // Real USB host - stay in USB mode
            g_connection_mode = CONNECTION_USB;
            kb_state.connection_mode = CONNECTION_USB;
            kb_state.output_mode = OUTPUT_USB;
            LOG_INFO("Validated USB host - staying in USB mode");
        } else {
            // Just a charger - switch to BLE instead
            shutdown_usb_mode();
            init_ble_mode();
            g_connection_mode = CONNECTION_BLE;
            kb_state.connection_mode = CONNECTION_BLE;
            kb_state.output_mode = OUTPUT_BLE;
            LOG_INFO("USB charger detected - switching to BLE mode");
        }
    } else {
        // BLE mode - no enumeration check needed
        init_ble_mode();
        g_connection_mode = CONNECTION_BLE;
        kb_state.connection_mode = CONNECTION_BLE;
        kb_state.output_mode = OUTPUT_BLE;
    }

    switch_state = MODE_SWITCH_COMPLETE;

    // Clear any pending reports since we switched modes
    memset(&kb_state.boot_report, 0, sizeof(kb_state.boot_report));
    memset(&kb_state.consumer_report, 0, sizeof(kb_state.consumer_report));
    report_ready_flag = 0;

    LOG_INFO("Mode switch completed successfully");
}

void check_vbus_and_switch_mode(void) {
    static uint32_t last_vbus_check = 0;
    uint32_t current_time = HAL_GetTick();

    // Check every 100ms
    if (current_time - last_vbus_check < 100) {
        return;
    }
    last_vbus_check = current_time;

    // Skip if switch in progress
    if (switch_state == MODE_SWITCH_IN_PROGRESS) {
        return;
    }

    bool usb_power_active = !(VBUS_DETECT_GPIO_Port->IDR & VBUS_DETECT_Pin); //XXX
    connection_mode_t desired_mode = usb_power_active ? CONNECTION_USB : CONNECTION_BLE;

    // Switch if needed - enumeration test will validate USB
    if (g_connection_mode != desired_mode) {
        switch_connection_mode(desired_mode);
    }
}

void check_connection(void) {
    // Reset switch state if it's been stuck too long
    if (switch_state == MODE_SWITCH_IN_PROGRESS) {
        if ((HAL_GetTick() - switch_start_time) > 3000) { // 3 second timeout
            LOG_ERROR("Mode switch timeout - resetting switch state");
            switch_state = MODE_SWITCH_IDLE;
            if (g_connection_mode == CONNECTION_BLE) {
					init_ble_mode();
				} else {
					init_usb_mode();
				}
        }
        return;
    }

    // Reset switch state after completion
    if (switch_state == MODE_SWITCH_COMPLETE) {
        switch_state = MODE_SWITCH_IDLE;
    }

    // Check for VBUS changes and switch if needed
    check_vbus_and_switch_mode();
}


