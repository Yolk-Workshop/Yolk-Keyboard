/*
 * kb_driver.c
 *
 *  Created on: Jan 7, 2025
 *      Author: bettysidepiece
 */
#include "kb_driver.h"
#include "usbd_core.h"
#include "pmsm.h"

// External declarations
extern TIM_HandleTypeDef htim21;
extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart2;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern keyboard_state_t kb_state;
extern StateHandler stateMachine[5];

volatile uint8_t ble_conn_flag = false;

const uint16_t row_pins[KEY_ROWS] =
		{ R0_Pin, R1_Pin, R2_Pin,
		R3_Pin, R4_Pin, R5_Pin };

GPIO_TypeDef *const row_ports[KEY_ROWS] =
	{ R0_GPIO_Port, R1_GPIO_Port, R2_GPIO_Port,
	R3_GPIO_Port, R4_GPIO_Port, R5_GPIO_Port };

GPIO_TypeDef *const col_ports[KEY_COLS] = {
	C0_GPIO_Port, C1_GPIO_Port, C2_GPIO_Port, C3_GPIO_Port,
	C4_GPIO_Port, C5_GPIO_Port, C6_GPIO_Port, C7_GPIO_Port,
	C8_GPIO_Port, C9_GPIO_Port, C10_GPIO_Port, C11_GPIO_Port,
	C12_GPIO_Port, C13_GPIO_Port
};

const uint16_t col_pins[KEY_COLS] = {
C0_Pin, C1_Pin, C2_Pin, C3_Pin, C4_Pin, C5_Pin, C6_Pin, C7_Pin,
C8_Pin, C9_Pin, C10_Pin, C11_Pin, C12_Pin, C13_Pin };

extern uint8_t USBD_HID_SendKeyboardReport(USBD_HandleTypeDef *pdev,
		uint8_t *report, uint16_t len);

extern uint8_t USBD_HID_SendNKROReport(USBD_HandleTypeDef *pdev,
		uint8_t *report, uint16_t len);

extern uint8_t USBD_HID_SendConsumerReport(USBD_HandleTypeDef *pdev,
		uint8_t *report, uint16_t len);

// Time-related functions
inline void delay_us(uint32_t us)
{
	if (us > 0xFFFF) {
		us = 0xFFFF;
	}
	volatile uint32_t start_time = TIM21->CNT;
	while (((TIM21->CNT - start_time) & 0xFFFF) < us) {
	}
}

inline uint32_t getMicroseconds(void)
{
	return TIM21->CNT;
}

uint32_t elapsedTime(uint32_t start_time)
{
	uint32_t current_time = getMicroseconds();
	if (current_time >= start_time) {
		return current_time - start_time;
	}
	else {
		return (0xFFFF - start_time + current_time);
	}
}


// Keyboard-related functions
inline void resetRows(void)
{
	for (int i = 0; i < KEY_ROWS; i++) {
		row_ports[i]->ODR &= ~row_pins[i];
	}
}

void scanKeyMatrix(void)
{
	static bool column_triggered[KEY_COLS] = {false};
	static uint32_t column_trigger_time[KEY_COLS] = {0};
	static uint8_t column_active_row[KEY_COLS] = {0xFF};

	uint32_t current_time = getMicroseconds();
	static int8_t locked_row = -1;
	static int8_t locked_col = -1;

	memset(column_triggered, false, sizeof(column_triggered));

	for (uint8_t row = 0; row < KEY_ROWS; row++) {
		row_ports[row]->ODR |= row_pins[row];
		delay_us(300);

		for (uint8_t col = 0; col < KEY_COLS; col++) {
			bool col_pressed = (col_ports[col]->IDR & col_pins[col]);
			bool should_process = true;

			// NEW: Enhanced column deduplication logic
			if (col_pressed) {
				// Check if this column already triggered in this scan cycle
				if (column_triggered[col]) {
					// This is definitely a phantom - same column already triggered
					LOG_DEBUG("SCAN PHANTOM BLOCKED: R%d C%d (col %d already active this scan)",
							 row, col, col);
					should_process = false;
				}
				// Check if a different row was recently active on this column
				else if (column_active_row[col] != 0xFF &&
						 column_active_row[col] != row &&
						 (current_time - column_trigger_time[col]) < 3000) {  // 2ms
					LOG_DEBUG("COLUMN SWITCH PHANTOM: R%d C%d (R%d was active %luµs ago)",
							 row, col, column_active_row[col], current_time - column_trigger_time[col]);
					should_process = false;
				}

				if (should_process) {
					// Mark this column as triggered in this scan
					column_triggered[col] = true;
					column_trigger_time[col] = current_time;
					column_active_row[col] = row;  // Remember which row is active
				}
			} else {
				// Key released - clear the active row tracking for this column
				if (column_active_row[col] == row) {
					column_active_row[col] = 0xFF;  // Mark as no active row
				}
			}

			// Only process if not filtered as phantom
			if (should_process) {
				key_states[row][col] = stateMachine[key_states[row][col]](row, col,
						col_pressed, current_time);

				if (key_states[row][col] == KEY_PRESSED) {
					LOG_DEBUG("R[%d]C[%d]-Pressed", row, col);
					locked_row = row;
					locked_col = col;
				}

				if (key_states[row][col] == KEY_RELEASED && row == locked_row
						&& col == locked_col) {
					LOG_DEBUG("R[%d]C[%d]-Released", row, col);
					locked_row = -1;
					locked_col = -1;
				}
			}
		}

		row_ports[row]->ODR &= ~row_pins[row];
		delay_us(200);
	}
}

void sendUSBReport(void)
{
	static boot_keyboard_report_t last_boot_report = { 0 };
	static nkro_keyboard_report_t last_nkro_report = { 0 };
	static consumer_report_t last_consumer_report = { 0 };
	uint32_t current_time = getMicroseconds();

	// Debug logging
	//LOG_DEBUG("Report Transmission:\n");
	// LOG_DEBUG("Key Count: %d\n", kb_state.key_count);
	if (kb_state.key_count <= 6) {
		if (memcmp(&kb_state.boot_report, &last_boot_report,
				sizeof(boot_keyboard_report_t)) != 0) {
			__disable_irq();
			uint8_t result = USBD_HID_SendKeyboardReport(&hUsbDeviceFS,
					(uint8_t*) &kb_state.boot_report,
					sizeof(boot_keyboard_report_t));
			__enable_irq();

			if (result == USBD_OK) {
				kb_state.last_report.usb_last_report = current_time;
				memcpy(&last_boot_report, &kb_state.boot_report,
						sizeof(boot_keyboard_report_t));

				// Additional debug logging
				LOG_DEBUG("Boot Report Sent:\n");
				LOG_DEBUG("Modifiers: 0x%02X\n",
						kb_state.boot_report.modifiers);
				LOG_DEBUG("Key: 0x%02X", kb_state.boot_report.keys[0]);
			}
			else {
				LOG_DEBUG("Boot Report Transmission Failed\n");
			}
		}
	}
	else {
		if (memcmp(&kb_state.nkro_report, &last_nkro_report,
				sizeof(nkro_keyboard_report_t)) != 0) {
			__disable_irq();
			uint8_t result = USBD_HID_SendNKROReport(&hUsbDeviceFS,
					(uint8_t*) &kb_state.nkro_report,
					sizeof(nkro_keyboard_report_t));
			__enable_irq();

			if (result == USBD_OK) {
				kb_state.last_report.usb_last_report = current_time;
				memcpy(&last_nkro_report, &kb_state.nkro_report,
						sizeof(nkro_keyboard_report_t));

				// Additional debug logging
				LOG_DEBUG("NKRO Report Sent\n");
			}
			else {
				LOG_DEBUG("NKRO Report Transmission Failed\n");
			}
		}
	}

	// Consumer Report Transmission
	if (memcmp(&kb_state.consumer_report, &last_consumer_report,
			sizeof(consumer_report_t)) != 0) {
		__disable_irq();
		uint8_t result = USBD_HID_SendConsumerReport(&hUsbDeviceFS,
				(uint8_t*) &kb_state.consumer_report,
				sizeof(consumer_report_t));
		__enable_irq();

		if (result == USBD_OK) {
			memcpy(&last_consumer_report, &kb_state.consumer_report,
					sizeof(consumer_report_t));

			// Additional debug logging
			LOG_DEBUG("Consumer Report Sent: 0x%X%X",
					kb_state.consumer_report.buttons[0],
					kb_state.consumer_report.buttons[1]);
		}
		else {
			LOG_DEBUG("Consumer Report Transmission Failed\n");
		}
	}
}

__weak void sendBLEReport(void)
{
	static boot_keyboard_report_t last_report = { 0 };
	//uint32_t current_time = getMicroseconds();

	// Only send if report has changed
	if (memcmp(&kb_state.boot_report, &last_report,
			sizeof(boot_keyboard_report_t)) != 0) {

	}
	//LOG_DEBUG("BLE Report Exit");
}

void checkConnection()
{
	bool usb_detected;
	uint8_t usb_state;
	connection_mode_t connection_mode;
	output_mode_t output_mode;

	__disable_irq();
	usb_detected = VBUS_DETECT_GPIO_Port->IDR & VBUS_DETECT_Pin;
	usb_state = hUsbDeviceFS.dev_state;
	__enable_irq();

	if (!usb_detected && (usb_state == USBD_STATE_CONFIGURED)) {
		connection_mode = CONNECTION_USB;
		output_mode = OUTPUT_USB;
	}
	else if (!usb_detected && !(usb_state == USBD_STATE_CONFIGURED)) {
		connection_mode = CONNECTION_BLE;
		output_mode = OUTPUT_BLE;
	}
	else {
		connection_mode = CONNECTION_BLE;
		output_mode = OUTPUT_BLE;
	}

	if (kb_state.connection_mode != connection_mode) {
		// Connection mode has changed
		if (connection_mode == CONNECTION_USB) {
			if (kb_state.connection_mode != CONNECTION_USB) {
				// Only initialise USB if we're switching to it
				MX_USB_DEVICE_Init();
				LOG_DEBUG("Initialising USB");
			}
		}
		else {
			if (kb_state.connection_mode != CONNECTION_BLE) {
				// Only initialise BLE if we're switching to it
				initBluetooth();
				LOG_DEBUG("Initialising Bluetooth");
			}
		}
		LOG_DEBUG("Switching to %s Mode",
				connection_mode == CONNECTION_USB ? "USB" : "BLE");
	}

	kb_state.connection_mode = connection_mode;
	kb_state.output_mode = output_mode;

}

void reportArbiter(void)
{
	PM_RecordActivity();
	//LOG_DEBUG("Report Routing: %d", kb_state.output_mode);
	if (kb_state.output_mode == OUTPUT_USB
			&& elapsedTime(kb_state.last_report.usb_last_report)
					>= HID_FS_BINTERVAL * 1000) {
		//LOG_DEBUG("USB Report");
		sendUSBReport();
	}
	else if (kb_state.output_mode == OUTPUT_BLE
			&& elapsedTime(kb_state.last_report.ble_last_report)
					>= 1000) {
		//LOG_DEBUG("BLE Report");
		sendBLEReport();
	}
}
