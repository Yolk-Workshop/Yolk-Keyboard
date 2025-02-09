#ifndef I2C_CORE_H
#define I2C_CORE_H
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <stddef.h>
#include <stm32l072xx.h>

// Structure definitions
typedef struct __attribute__((packed))
{
    uint8_t tx_buffer[16];
    uint8_t rx_buffer[16];
    uint8_t tx_count;
    uint8_t rx_count;
    uint8_t tx_size;
    uint8_t rx_size;
    atomic_bool busy;
} i2c_transfer;

typedef struct __attribute__((packed))
{
    uint8_t address;
    i2c_transfer tranfer;
} i2c_device;

// Global I2C instance
extern i2c_device i2c1_inst;

// Core I2C functions
bool initI2C1(void);
void enable_I2C1(void);
void disable_I2C1(void);
bool transmit_I2C(i2c_device *device);
bool receive_I2C(i2c_device *device);

// Recovery functions
void I2C_Reset(void);
void I2C_BusRecovery(void);
bool scan_I2C_bus(void);

#endif /* I2C_CORE_H */
