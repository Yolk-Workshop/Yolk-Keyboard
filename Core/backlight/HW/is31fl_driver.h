/**
 * @file IS31FL_Driver.h
 * @brief IS31FL3743A LED Matrix Driver Header
 * @version 1.0
 *
 * Optimized driver for IS31FL3743A 18x11 LED matrix controller
 * Memory efficient for constrained MCU environments
 */

#ifndef BACKLIGHT_CORE_IS31FL_DRIVER_H
#define BACKLIGHT_CORE_IS31FL_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32l072xx.h"

// Driver Configuration
#define IS31FL_MAX_DEVICES      2
#define IS31FL_MAX_LEDS         198  // 18x11 matrix
#define IS31FL_MASTER_LED_COUNT IS31FL_MAX_LEDS
#define IS31FL_SLAVE_LED_COUNT  46
#define IS31FL_I2C_TIMEOUT      100

// Register Definitions
#define IS31FL_REG_UNLOCK       0xFE
#define IS31FL_REG_COMMAND      0xFD
#define IS31FL_REG_ID           0xFC

// Pages
#define IS31FL_PAGE_PWM         0x00
#define IS31FL_PAGE_SCALING     0x01
#define IS31FL_PAGE_FUNCTION    0x02

// Function Registers
#define IS31FL_REG_CONFIG       0x00
#define IS31FL_REG_GLOBAL_CURRENT 0x01
#define IS31FL_REG_PULLUP       0x02
#define IS31FL_REG_RESET        0x2F

// Configuration bits
#define IS31FL_CONFIG_SSD       0x01  // Software shutdown disable
#define IS31FL_CONFIG_OSDE_OPEN 0x02  // Open detection
#define IS31FL_CONFIG_OSDE_SHORT 0x04 // Short detection

// I2C Addresses (7-bit)
#define IS31FL_ADDR_0  			0x40  // ADDR2=GND, ADDR1=GND
#define IS31FL_ADDR_1  			0x42  // ADDR2=GND, ADDR1=SCL
#define IS31FL_ADDR_2  			0x44  // ADDR2=GND, ADDR1=SDA
#define IS31FL_ADDR_3  			0x46  // ADDR2=GND, ADDR1=VCC
#define IS31FL_ADDR_U2  		0x2C  // ADDR2=GND, ADDR1=GND (U2 device)
#define IS31FL_ADDR_U4  		0x20  // ADDR2=VCC, ADDR1=GND (U4 device)

// Matrix Dimensions
#define IS31FL_CS_COUNT         18
#define IS31FL_SW_COUNT         11

// Error Codes
typedef enum {
    IS31FL_OK = 0,
    IS31FL_ERROR_I2C,
    IS31FL_ERROR_TIMEOUT,
    IS31FL_ERROR_PARAM,
    IS31FL_ERROR_DEVICE
} IS31FL_Error_t;

// I2C Interface Functions (implement these for your platform)
typedef struct {
	IS31FL_Error_t (*write_dma)(uint8_t addr, uint8_t *data, uint16_t len);
	IS31FL_Error_t (*write)(uint8_t addr, uint8_t* data, uint16_t len);
	IS31FL_Error_t (*read)(uint8_t addr, uint8_t* data, uint16_t len);
	IS31FL_Error_t (*write_read)(uint8_t addr, const uint8_t* tx_data, uint16_t tx_len,
								uint8_t* rx_data, uint16_t rx_len);
	IS31FL_Error_t (*write_reg_byte)(uint8_t addr, uint8_t reg, uint8_t value);
	IS31FL_Error_t (*read_reg_byte)(uint8_t addr, uint8_t reg, uint8_t* value);
	void (*clear_i2c_irq)(I2C_TypeDef* i2c);
    void (*delay_ms)(uint32_t ms);
} IS31FL_I2C_Interface_t;

// Device Handle
typedef struct {
    uint8_t i2c_addr;
    uint8_t current_page;
    bool initialized;
} IS31FL_Device_t;

// Driver Context
typedef struct {
    IS31FL_Device_t devices[IS31FL_MAX_DEVICES];
    uint8_t device_count;
    const IS31FL_I2C_Interface_t *i2c_if;
    bool initialized;
} IS31FL_Context_t;

// Public API Functions
IS31FL_Error_t IS31FL_Init(IS31FL_Context_t *ctx, const IS31FL_I2C_Interface_t *i2c_if);
IS31FL_Error_t IS31FL_AddDevice(IS31FL_Context_t *ctx, uint8_t i2c_addr);
IS31FL_Error_t IS31FL_DeviceInit(IS31FL_Context_t *ctx, uint8_t device_idx);
IS31FL_Error_t IS31FL_Reset(IS31FL_Context_t *ctx, uint8_t device_idx);
IS31FL_Error_t IS31FL_Shutdown(IS31FL_Context_t *ctx, uint8_t device_idx, bool enable);

// LED Control Functions
IS31FL_Error_t IS31FL_SetLED(IS31FL_Context_t *ctx, uint8_t device_idx,
                             uint8_t cs, uint8_t sw, uint8_t brightness);
IS31FL_Error_t IS31FL_SetAllLEDs(IS31FL_Context_t *ctx, uint8_t device_idx,
                                 uint8_t brightness);
IS31FL_Error_t IS31FL_UpdateMatrix(IS31FL_Context_t *ctx, uint8_t device_idx,
                                   const uint8_t *matrix_data, uint16_t size);

// Configuration Functions
IS31FL_Error_t IS31FL_SetGlobalCurrent(IS31FL_Context_t *ctx, uint8_t device_idx,
                                       uint8_t current);
IS31FL_Error_t IS31FL_SetScaling(IS31FL_Context_t *ctx, uint8_t device_idx,
                                 uint8_t cs, uint8_t sw, uint8_t scaling);

IS31FL_Error_t IS31FL_ConfigureSync(IS31FL_Context_t *ctx, uint8_t device_idx, bool is_master);
// Utility Functions
uint8_t IS31FL_GetRegisterAddr(uint8_t cs, uint8_t sw);
bool IS31FL_ValidatePosition(uint8_t cs, uint8_t sw);
IS31FL_Error_t SelectPageWrapper(const IS31FL_Context_t *ctx, uint8_t device_idx,
                                uint8_t page);
// Gamma Correction Table (32 steps)
extern const uint8_t IS31FL_GAMMA_TABLE[32];

// Convenience Macros
#define IS31FL_LED_ADDR(cs, sw)     IS31FL_GetRegisterAddr(cs, sw)
#define IS31FL_GAMMA(step)          IS31FL_GAMMA_TABLE[step]
#define IS31FL_BRIGHTNESS_MAX       255
#define IS31FL_BRIGHTNESS_OFF       0

#endif // IS31FL_DRIVER_H
