/*
 * kb_driver.c
 *
 *  Created on: Jan 7, 2025
 *      Author: bettysidepiece
 */
#include "kb_driver.h"
#include "usbd_core.h"

// External declarations
extern TIM_HandleTypeDef htim21;
extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart2;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern keyboard_state_t kb_state;
extern StateHandler stateMachine[5];
extern rnbd_interface_t RNBD;

volatile uint8_t ble_conn_flag = false;

const uint16_t row_pins[KEY_ROWS] = {R0_Pin, R1_Pin, R2_Pin, R3_Pin, R4_Pin, R5_Pin};

GPIO_TypeDef* const row_ports[KEY_ROWS] = {R0_GPIO_Port, R1_GPIO_Port, R2_GPIO_Port,
                                           R3_GPIO_Port, R4_GPIO_Port, R5_GPIO_Port};
GPIO_TypeDef* const col_ports[KEY_COLS] = {
    C0_GPIO_Port, C1_GPIO_Port, C2_GPIO_Port, C3_GPIO_Port,
    C4_GPIO_Port, C5_GPIO_Port, C6_GPIO_Port, C7_GPIO_Port,
    C8_GPIO_Port, C9_GPIO_Port, C10_GPIO_Port, C11_GPIO_Port,
    C12_GPIO_Port, C13_GPIO_Port
};

const uint16_t col_pins[KEY_COLS] = {
    C0_Pin, C1_Pin, C2_Pin, C3_Pin, C4_Pin, C5_Pin, C6_Pin, C7_Pin,
    C8_Pin, C9_Pin, C10_Pin, C11_Pin, C12_Pin, C13_Pin
};

extern uint8_t USBD_HID_SendReport(USBD_HandleTypeDef  *pdev,
	                                   uint8_t *report,
	                                   uint16_t len);

void checkConnection(void);

// Time-related functions
void delay_us(uint32_t us)
{
    if (us > 0xFFFF) {us = 0xFFFF;}
    volatile uint32_t start_time = TIM21->CNT;
    while (((TIM21->CNT - start_time) & 0xFFFF) < us) {}
}

static void non_blocking_delay_ms(uint32_t ms)
{
	if (ms > 0xFFFF) {ms = 0xFFFF;}
	volatile uint32_t start_time = HAL_GetTick();
	while(((HAL_GetTick() - start_time) & 0xFFFF) < ms);
}

uint32_t getMicroseconds(void)
{
    return TIM21->CNT;
}

uint32_t elapsedTime(uint32_t start_time)
{
    uint32_t current_time = getMicroseconds();
    if (current_time >= start_time) {
        return current_time - start_time;
    } else {
        return (0xFFFF - start_time + current_time);
    }
}


// Bluetooth-related functions
static void resetRNBD350(bool state)
{
    if (state) {
    	RNBD.gpio.reset_port->ODR &= ~RNBD.gpio.reset_pin ;
        LOG_DEBUG("RNBD350 Reset asserted (LOW)");
    } else {
    	RNBD.gpio.reset_port->ODR |= RNBD.gpio.reset_pin ;
        LOG_DEBUG("RNBD350 Reset released (HIGH)");

        if ((RNBD.gpio.reset_port->ODR & RNBD.gpio.reset_pin ) != RNBD.gpio.reset_pin ) {
            LOG_ERROR("ERR: Reset pin failed to release");
        }
    }
}


static void RxIndicate(bool value)
{
	if (value) {
		RNBD.gpio.wakeup_port->ODR |= RNBD.gpio.wakeup_pin ;
		LOG_DEBUG("RNBD350 Reset asserted (LOW)");
	} else {
		RNBD.gpio.wakeup_port->ODR &= ~RNBD.gpio.wakeup_pin ;
		LOG_DEBUG("RNBD350 Reset released (HIGH)");

		if ((RNBD.gpio.reset_port->ODR & RNBD.gpio.wakeup_pin ) != RNBD.gpio.wakeup_pin ) {
			LOG_ERROR("ERR: Reset pin failed to release");
		}
	}
}


static void setBLEMode(RNBD_sys_modes_t mode)
{
	LOG_DEBUG("FAULT DETECTED FUNCT PTR BUG");
}


static void initBLEhardware()
{
    RNBD.gpio.reset_pin 	=	 BT_RESET_Pin;
	RNBD.gpio.reset_port 	=	 BT_RESET_GPIO_Port;
	RNBD.gpio.wakeup_pin 	=	 BT_RX_INPUT_Pin;
	RNBD.gpio.wakeup_port 	=	 BT_RX_INPUT_GPIO_Port;
	RNBD.gpio.status1_pin 	=	 BLE_STAT1_Pin;
	RNBD.gpio.status1_port 	=	 BLE_STAT1_GPIO_Port;
	RNBD.gpio.status2_pin 	=	 BLE_STAT2_Pin;
	RNBD.gpio.status2_port 	=	 BLE_STAT2_GPIO_Port;
}


static void setBLECallbacks(void)
{
	RNBD.callback.delayMs = HAL_Delay;
	RNBD.callback.read = uart_read;
	RNBD.callback.write = uart_write;
	RNBD.callback.transmitReady = uart_transmit_ready;
	RNBD.callback.dataReady = uart_data_ready;
	RNBD.callback.resetModule = resetRNBD350;
	RNBD.callback.asyncHandler = asyncMessageHandler;
	RNBD.callback.rxIndicate = RxIndicate;
	RNBD.callback.systemModeset = setBLEMode;
	RNBD.callback.getConnStatus = RNBD_isConnected;
	RNBD.callback.nonBlockDelayMs = non_blocking_delay_ms;
}


static void setDevice_descriptor(void)
{
	RNBD.device.vendorName = (const uint8_t *)"Yolk-Workshop";
	RNBD.device.productName = (const uint8_t *)"Yolk-Keyboard";
	RNBD.device.hwVersion = (const uint8_t *)"0.1";
	RNBD.device.fwVersion = (const uint8_t *)"0.2";
	RNBD.device.driverVersion = (const uint8_t *)"2.0.0";
	RNBD.device.bleSdaHid = 961; // BLE HID
}


void initBluetooth(void)
{
	uint8_t retries = 3;
	uint8_t retry = 0;

    LOG_DEBUG("BLE initialisation started");

    USBD_DeInit(&hUsbDeviceFS);
    LOG_INFO("USB Disabled");

    MX_LPUART1_UART_Init();
    LOG_DEBUG("LPUART Initialised Successfully");

    initBLEhardware();
    setBLECallbacks();
    setDevice_descriptor();

    while(retry <= retries){
    	if(RNBD_Init()) {
        LOG_DEBUG("Setting Up Device Configuration");

        if(RNBD_EnterCmdMode()) {
        	LOG_DEBUG("Command Mode Entered");
        	//RNBD_FactoryReset(SF_2);

        	RNBD_StopAdvertising();
            RNBD_SetName(RNBD.device.productName, PRODUCT_NAME_LEN);

            //GATT Service Settings
            RNBD_SetAppearance(RNBD.device.bleSdaHid);

            // Switch to Transparent UART/ Device Info mode
            if(RNBD_SetServiceBitmap(0xC0)){
				LOG_DEBUG("BLE HID Mode set");
				//Verify the change
				RNBD_ServiceChangeIndicator();
				RNBD_SetHWVersion(RNBD.device.hwVersion);
				RNBD_SetFWVersion(RNBD.device.fwVersion);
				RNBD_SetModelName(RNBD.device.productName);
				RNBD_SetMakerName(RNBD.device.vendorName);
				LOG_DEBUG("Configuring HID GATT Profile");
			}

            RNBD.callback.delayMs(100);
			if(!GATT_ble_Init()){
				LOG_ERROR("Bluetooth GATT HID profile configuration failed");

				 if(!RNBD_FactoryReset(SF_2)){
					LOG_ERROR("Factory Reset Failed");
					LOG_WARNING("Hard Reset in 3 Seconds");
					RNBD.callback.delayMs(3000); // Allow module to reboot
					NVIC_SystemReset();
				}

				RNBD.callback.delayMs(2000); // Allow module to reboot
				LOG_INFO("Re-attempting Initialisation After Factory Reset");
				initBluetooth(); // Restart initialisation
				return;
			}

			if(RNBD_ServiceChangeIndicator()){
				LOG_DEBUG("BLE Service Changed");
			}



			RNBD.callback.resetModule(true);
			RNBD.callback.delayMs(RNBD_RESET_DELAY_TIME);
			RNBD.callback.resetModule(false);
			RNBD.callback.delayMs(RNBD_STARTUP_DELAY);


			while (RNBD.callback.dataReady()) RNBD.callback.read();
			LOG_INFO("RNBD3350 reboot successful, command mode entered");


			GATT_List_Services();
			//if(RNBD_SetServiceBitmap(0xC0)){
				//Verify the change
			//	RNBD_ServiceChangeIndicator();
			//}

			RNBD_EnableAdvertising();

            //RNBD_EnterDataMode();
            RNBD.device.connection_timer = BLE_DEFAULT_TIMER;
            ble_conn_flag = RESET;
            RNBD.connected = RESET;
            kb_state.output_mode = OUTPUT_BLE;
            LOG_DEBUG("BLE: Output - %d | Connected - %d",kb_state.output_mode, RNBD.connected);
            LOG_INFO("Bluetooth Initialisation Passed");

            return;
        	}
    	}

    	LOG_WARNING("Bluetooth Initialisation failed");
    	LOG_WARNING("[%s] retry to Initialise Bluetooth", retry);
    	retry++;
	}
}


// Keyboard-related functions
void resetRows(void)
{
    for(int i = 0; i < KEY_ROWS; i++) {
        row_ports[i]->ODR &= ~row_pins[i];
    }
}


void scanKeyMatrix(void)
{
    uint32_t current_time = getMicroseconds();
    static int8_t locked_row = -1;
    static int8_t locked_col = -1;

    for (uint8_t row = 0; row < KEY_ROWS; row++) {
        row_ports[row]->ODR |= row_pins[row];
        delay_us(25);

        for (uint8_t col = 0; col < KEY_COLS; col++) {
            bool col_pressed = (col_ports[col]->IDR & col_pins[col]);
            key_states[row][col] = stateMachine[key_states[row][col]](row, col, col_pressed, current_time);

            if (key_states[row][col] == KEY_PRESSED) {
                LOG_DEBUG("R[%d]C[%d]-Pressed", row, col);
                locked_row = row;
                locked_col = col;
            }

            if (key_states[row][col] == KEY_RELEASED && row == locked_row && col == locked_col) {
                locked_row = -1;
                locked_col = -1;
            }
        }

        row_ports[row]->ODR &= ~row_pins[row];
    }
}


void sendUSBReport(void)
{
    static hid_report_t last_report_usb = {0};
    uint32_t current_time = getMicroseconds();

    if (memcmp(&kb_state.current_report, &last_report_usb, sizeof(hid_report_t)) != 0) {
        __disable_irq();
        uint8_t result = USBD_HID_SendReport(&hUsbDeviceFS,
                            (uint8_t*)&kb_state.current_report,
                            sizeof(hid_report_t));
        __enable_irq();

        if (result == USBD_OK) {
            kb_state.last_report.usb_last_report = current_time;
            memcpy(&last_report_usb, &kb_state.current_report, sizeof(hid_report_t));
            clearReport();
        }
    }
}


void sendBLEReport(void)
{
    static hid_report_t last_report = {0};
    uint32_t current_time = getMicroseconds();

    // Only send if report has changed
    if (memcmp(&kb_state.current_report, &last_report, sizeof(hid_report_t)) != 0) {

        ble_report.report_id = 1;
        memcpy(&ble_report.report, &kb_state.current_report, sizeof(hid_report_t));

        if (GATT_sendHIDReport((uint8_t*)&ble_report, sizeof(ble_hid_report_t))) {
            // Update timing and save last report
            kb_state.last_report.ble_last_report = current_time;
            memcpy(&last_report, &kb_state.current_report, sizeof(hid_report_t));
            clearReport();
            LOG_DEBUG("BLE Report Successful");
        } else {
            LOG_DEBUG("BLE Report Failed");
        }
    }
    LOG_DEBUG("BLE Report Exit");
}


void checkConnection()
{
    if (!(VBUS_DETECT_GPIO_Port->IDR & VBUS_DETECT_Pin)) {
        // No USB connection detected
        if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
            // USB connection is active
            if (kb_state.output_mode != OUTPUT_USB) {
                LOG_DEBUG("Switching to USB Mode");
            }
            kb_state.connection_mode = CONNECTION_USB;
            kb_state.output_mode = OUTPUT_USB;
            LOG_DEBUG("USB Device State: 0x%02X", hUsbDeviceFS.dev_state);
        } else {
            // USB is not configured; default to BLE
            if (kb_state.output_mode != OUTPUT_BLE) {
                LOG_DEBUG("Switching to BLE Mode");
            }
            kb_state.connection_mode = CONNECTION_BLE;
            kb_state.output_mode = OUTPUT_BLE;
            LOG_DEBUG("USB State: 0x%02X", hUsbDeviceFS.dev_state);
        }
    } else {
        // VBUS is active, default to BLE mode
        if (kb_state.output_mode != OUTPUT_BLE) {
            LOG_DEBUG("Switching to BLE Mode");
        }
        kb_state.connection_mode = CONNECTION_BLE;
        kb_state.output_mode = OUTPUT_BLE;
    }
    LOG_DEBUG("Connection Mode: %s", kb_state.connection_mode == CONNECTION_USB ? "USB" : "BLE");
}


void checkBLEconnection(void)
{
    if (!ble_conn_flag) return;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    ble_conn_flag = false;
    __set_PRIMASK(primask);

    bool connectionStatus = RNBD.callback.getConnStatus();
    static bool prev_status = false;
    bool status_changed = false;
    static uint32_t last_change_time = 0;

    // Add debounce time after state changes
    if (HAL_GetTick() - last_change_time < 100) {
        return;  // Skip check if within debounce period
    }

    if (!RNBD.connected && connectionStatus) {
        RNBD.device.connection_timer = BLE_ALTERED_TIMER;
        status_changed = true;
        last_change_time = HAL_GetTick();
    }
    else if (RNBD.connected && !connectionStatus) {
        // Only disconnect if state has been stable
        if (HAL_GetTick() - last_change_time > 500) {
            RNBD.device.connection_timer = BLE_DEFAULT_TIMER;
            LOG_DEBUG("BLE Disconnected");
            RNBD.connected = false;
            status_changed = true;
            last_change_time = HAL_GetTick();
        }
    }

    if (prev_status != status_changed) {
        LOG_DEBUG("BLE connection check completed. Status: %s",
                 RNBD.connected ? "Connected" : "Disconnected");
    }
    prev_status = status_changed;
}



void reportArbiter(void)
{
	//LOG_DEBUG("Report Routing: %d", kb_state.output_mode);
    if (kb_state.output_mode == OUTPUT_USB &&
            elapsedTime(kb_state.last_report.usb_last_report) >= HID_FS_BINTERVAL * 1000) {
    	//LOG_DEBUG("USB Report");
        sendUSBReport();
    }
    else if (kb_state.output_mode == OUTPUT_BLE &&
		 	elapsedTime(kb_state.last_report.ble_last_report) >= BLE_STD_INTERVAL * 1000) {
    	//LOG_DEBUG("BLE Report");
        sendBLEReport();
    }
}
