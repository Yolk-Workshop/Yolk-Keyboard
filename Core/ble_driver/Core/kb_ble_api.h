/*
 * kb_ble_api.h
 *
 *  Created on: May 14, 2025
 *      Author: bettysidepiece
 */

#ifndef BLE_DRIVER_CORE_KB_BLE_API_H_
#define BLE_DRIVER_CORE_KB_BLE_API_H_

#include <bm7x_async.h>
#include <stdint.h>
#include <stdbool.h>

extern bm70_handle_t g_bm70;

// These functions are called from kb_driver.c
void initBluetooth(void);
void sendBLEReport(void);
void checkBLEconnection(void);
void ble_periodic_status_check(void);

// Additional BLE control functions
void ble_disconnect(void);
bool ble_is_connected(void);
bool ble_is_advertising(void);
void ble_restart_advertising(void);
void ble_deinit(void);

// Function to get battery percentage (implement based on your hardware)
uint8_t get_battery_percentage(void);

#endif /* BLE_DRIVER_CORE_KB_BLE_API_H_ */
