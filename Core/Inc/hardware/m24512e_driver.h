/*
 * m24512e_driver.h
 *
 *  Created on: Jun 13, 2025
 *      Author: bettysidepiece
 */

#ifndef INC_HARDWARE_M24512E_DRIVER_H_
#define INC_HARDWARE_M24512E_DRIVER_H_

#include "stm32l0xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Device Configuration */
#define M24512E_DEVICE_SIZE         65536U      // 64KB (512Kbit)
#define M24512E_PAGE_SIZE           128U        // 128 bytes per page
#define M24512E_ID_PAGE_SIZE        128U        // Identification page size
#define M24512E_MAX_PAGES           512U        // Total pages
#define M24512E_WRITE_CYCLE_TIME    4U          // 4ms max write cycle time

/* Device Select Codes */
#define M24512E_MEMORY_DEVICE_CODE      0xA0    // 1010 0000
#define M24512E_FEATURE_DEVICE_CODE     0xB0    // 1011 0000

/* Feature Address Codes (A15:A13) */
#define M24512E_DTI_ADDRESS_CODE        0xE0    // 111x xxxx
#define M24512E_CDA_ADDRESS_CODE        0xC0    // 110x xxxx
#define M24512E_SWP_ADDRESS_CODE        0xA0    // 101x xxxx
#define M24512E_ID_PAGE_ADDRESS_CODE    0x00    // 000x xxxx
#define M24512E_ID_LOCK_ADDRESS_CODE    0x60    // 011x xxxx

/* Register Default Values */
#define M24512E_DTI_DEFAULT_VALUE       0xB1    // 10110001
#define M24512E_CDA_DEFAULT_VALUE       0x00    // 00000000
#define M24512E_SWP_DEFAULT_VALUE       0x00    // 00000000

/* Timeout Values */
#define M24512E_I2C_TIMEOUT            100U     // HAL timeout in ms
#define M24512E_POLL_TIMEOUT           10U      // Polling timeout in ms
#define M24512E_WAKE_UP_DELAY          1U       // 5μs minimum, using 1ms for safety

/* CRC-16 CCITT Polynomial */
#define CRC16_POLYNOMIAL    0x1021UL

/* Status Codes */
typedef enum {
    M24512E_OK = 0,
    M24512E_ERROR,
    M24512E_TIMEOUT,
    M24512E_WRITE_PROTECTED,
    M24512E_INVALID_ADDRESS,
    M24512E_INVALID_SIZE,
    M24512E_CRC_ERROR
} m24512e_status_t;

/* Device Handle Structure */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t device_address;
    bool write_protection_enabled;
    uint32_t last_write_time;
} m24512e_handle_t;

/* Register Structures */
typedef struct {
    uint8_t dti3 : 1;
    uint8_t dti2 : 1;
    uint8_t dti1 : 1;
    uint8_t dti0 : 1;
    uint8_t reserved : 3;
    uint8_t dtil : 1;
} m24512e_dti_reg_t;

typedef struct {
    uint8_t dal : 1;
    uint8_t c0 : 1;
    uint8_t c1 : 1;
    uint8_t c2 : 1;
    uint8_t reserved : 4;
} m24512e_cda_reg_t;

typedef struct {
    uint8_t wpl : 1;
    uint8_t bp0 : 1;
    uint8_t bp1 : 1;
    uint8_t wpa : 1;
    uint8_t reserved : 4;
} m24512e_swp_reg_t;

/* Function Prototypes */

/* System Initialization */
void m24512e_system_init(void);

/* Core Functions */
m24512e_status_t m24512e_init(m24512e_handle_t *handle, I2C_HandleTypeDef *hi2c, uint8_t device_addr);
m24512e_status_t m24512e_deinit(m24512e_handle_t *handle);
bool m24512e_is_device_ready(m24512e_handle_t *handle);

/* Memory Operations */
m24512e_status_t m24512e_write_byte(m24512e_handle_t *handle, uint16_t address, uint8_t data);
m24512e_status_t m24512e_write_page(m24512e_handle_t *handle, uint16_t address, const uint8_t *data, uint16_t size);
m24512e_status_t m24512e_read_byte(m24512e_handle_t *handle, uint16_t address, uint8_t *data);
m24512e_status_t m24512e_read_sequential(m24512e_handle_t *handle, uint16_t address, uint8_t *data, uint16_t size);
m24512e_status_t m24512e_read_current(m24512e_handle_t *handle, uint8_t *data);

/* Memory Operations with CRC */
m24512e_status_t m24512e_write_page_with_crc(m24512e_handle_t *handle, uint16_t address, const uint8_t *data, uint16_t size);
m24512e_status_t m24512e_read_sequential_with_crc(m24512e_handle_t *handle, uint16_t address, uint8_t *data, uint16_t size);

/* Register Operations */
m24512e_status_t m24512e_read_dti_register(m24512e_handle_t *handle, m24512e_dti_reg_t *dti_reg);
m24512e_status_t m24512e_read_cda_register(m24512e_handle_t *handle, m24512e_cda_reg_t *cda_reg);
m24512e_status_t m24512e_write_cda_register(m24512e_handle_t *handle, const m24512e_cda_reg_t *cda_reg);
m24512e_status_t m24512e_read_swp_register(m24512e_handle_t *handle, m24512e_swp_reg_t *swp_reg);
m24512e_status_t m24512e_write_swp_register(m24512e_handle_t *handle, const m24512e_swp_reg_t *swp_reg);

/* Identification Page Operations */
m24512e_status_t m24512e_write_id_page(m24512e_handle_t *handle, uint8_t address, const uint8_t *data, uint8_t size);
m24512e_status_t m24512e_read_id_page(m24512e_handle_t *handle, uint8_t address, uint8_t *data, uint8_t size);
m24512e_status_t m24512e_lock_id_page(m24512e_handle_t *handle);
bool m24512e_is_id_page_locked(m24512e_handle_t *handle);

/* Utility Functions */
m24512e_status_t m24512e_wait_write_complete(m24512e_handle_t *handle);
uint16_t m24512e_calculate_crc16(const uint8_t *data, uint16_t size);
bool m24512e_verify_crc16(const uint8_t *data, uint16_t size, uint16_t expected_crc);

/* Advanced Operations */
m24512e_status_t m24512e_write_data_safe(m24512e_handle_t *handle, uint16_t address, const uint8_t *data, uint16_t size);

#endif /* INC_HARDWARE_M24512E_DRIVER_H_ */
