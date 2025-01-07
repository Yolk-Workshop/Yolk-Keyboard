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

// UART helper functions
bool uart_transmit_ready(void)
{
    return ((LPUART1->ISR & UART_FLAG_TXE) == (UART_FLAG_TXE));
}

bool uart_data_ready(void)
{
    return ((LPUART1->ISR & UART_FLAG_RXNE) == (UART_FLAG_RXNE));
}

void uart_write_wrapper(void)
{
    // Implementation placeholder
}

void uart_read_wrapper(void)
{

}


// Bluetooth-related functions
void resetRNBD350(bool state)
{
    if (state) {
        GPIOC->ODR &= ~BT_RESET_Pin;
        LOG_DEBUG("RNBD350 Reset asserted (LOW)");
    } else {
        GPIOC->ODR |= BT_RESET_Pin;
        LOG_DEBUG("RNBD350 Reset released (HIGH)");

        if ((GPIOC->ODR & BT_RESET_Pin) != BT_RESET_Pin) {
            LOG_ERROR("ERR: Reset pin failed to release");
        }
    }
}


void initBluetooth(void)
{
    LOG_DEBUG("BLE initialization started");

    USBD_DeInit(&hUsbDeviceFS);
    LOG_INFO("USB Disabled");

    MX_LPUART1_UART_Init();
    LOG_DEBUG("LPUART Initialized Successfully");

    if(RNBD_Init()) {
        LOG_DEBUG("RNBD Initialized Successfully");

        if(RNBD_EnterCmdMode()) {
            RNBD_SetName((const uint8_t*)"Yolk_Keyboard", 12);
            RNBD_SetServiceBitmap(0x44);
            RNBD_EnterDataMode();

            kb_state.output_mode = OUTPUT_BLE;
            LOG_INFO("Bluetooth initialization successful");
            return;
        }
    }

    LOG_WARNING("Bluetooth Initialization failed");
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

    if (memcmp(&kb_state.current_report, &last_report, sizeof(hid_report_t)) != 0) {
        uint8_t uart_buffer[sizeof(ble_hid_report_t) + 2];

        uart_buffer[0] = 0xFD;
        ble_report.report_id = 1;
        memcpy(&ble_report.report, &kb_state.current_report, sizeof(hid_report_t));
        memcpy(&uart_buffer[1], &ble_report, sizeof(ble_hid_report_t));
        uart_buffer[sizeof(ble_hid_report_t) + 1] = 0xFE;

        if (HAL_UART_Transmit_DMA(&hlpuart1, uart_buffer,
                                 sizeof(ble_hid_report_t) + 2) == HAL_OK) {
            kb_state.last_report.ble_last_report = current_time;
            memcpy(&last_report, &kb_state.current_report, sizeof(hid_report_t));
            clearReport();
        }
    }
}


void checkConnection()
{
    if(!(VBUS_DETECT_GPIO_Port->IDR & VBUS_DETECT_Pin)) {
        if (hUsbDeviceFS.dev_state == USBD_STATE_DEFAULT) {
            kb_state.connection_mode = 0;
            LOG_DEBUG("USB Device State: 0x%02X", hUsbDeviceFS.dev_state);
            kb_state.output_mode = OUTPUT_USB;
            LOG_DEBUG("Connection Mode: USB");
            return;
        } else {
            kb_state.connection_mode = 1;
            kb_state.output_mode = OUTPUT_BLE;
            LOG_DEBUG("USB: 0x%02X", hUsbDeviceFS.dev_state);
            LOG_DEBUG("Connection Mode 1: BLE");
            return;
        }
    } else {
        kb_state.connection_mode = 1;
        kb_state.output_mode = OUTPUT_BLE;
        LOG_DEBUG("Connection Mode 2: BLE");
        return;
    }
}


void reportArbiter(void)
{
    if (kb_state.output_mode == OUTPUT_USB &&
            elapsedTime(kb_state.last_report.usb_last_report) >= HID_FS_BINTERVAL * 1000) {
        sendUSBReport();
    }
    else if (kb_state.output_mode == OUTPUT_BLE &&
             elapsedTime(kb_state.last_report.ble_last_report) >= HID_FS_BINTERVAL * 1000) {
        sendBLEReport();
    }
}
