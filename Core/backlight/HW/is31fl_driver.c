/**
 * @file IS31FL_Driver.c
 * @brief IS31FL3743A LED Matrix Driver Implementation
 * @version 1.0
 *
 * Memory-optimized driver implementation for IS31FL3743A
 */

#include "is31fl_driver.h"
#include <string.h>
#include "logger.h"
#include "main.h"

// Gamma correction lookup table (stored in flash)
const uint8_t IS31FL_GAMMA_TABLE[32] = {
    0, 1, 2, 4, 6, 10, 13, 18, 22, 28, 33, 39, 46, 53, 61, 69,
    78, 86, 96, 106, 116, 126, 138, 149, 161, 173, 186, 199, 212, 226, 240, 255
};

// Private function prototypes
static IS31FL_Error_t WriteRegister(const IS31FL_Context_t *ctx, uint8_t device_idx,
                                   uint8_t reg, uint8_t value);
static IS31FL_Error_t ReadRegister(const IS31FL_Context_t *ctx, uint8_t device_idx,
                                  uint8_t reg, uint8_t *value);
static IS31FL_Error_t SelectPage(const IS31FL_Context_t *ctx, uint8_t device_idx,
                                uint8_t page);
static IS31FL_Error_t UnlockCommand(const IS31FL_Context_t *ctx, uint8_t device_idx);

/**
 * @brief Initialize the driver context
 */
IS31FL_Error_t IS31FL_Init(IS31FL_Context_t *ctx, const IS31FL_I2C_Interface_t *i2c_if)
{
    if (!ctx || !i2c_if || !i2c_if->write || !i2c_if->read || !i2c_if->delay_ms) {
        LOG_ERROR("IS31FL: Invalid parameters");
        return IS31FL_ERROR_PARAM;
    }

    LOG_INFO("IS31FL: Initializing driver");
    memset(ctx, 0, sizeof(IS31FL_Context_t));

    LMAT_SDB_GPIO_Port->BSRR = LMAT_SDB_Pin;
    HAL_Delay(1);

    ctx->i2c_if = i2c_if;
    ctx->initialized = true;

    LOG_INFO("IS31FL: Driver initialized successfully");
    return IS31FL_OK;
}

/**
 * @brief Add a device to the context
 */
IS31FL_Error_t IS31FL_AddDevice(IS31FL_Context_t *ctx, uint8_t i2c_addr)
{
    if (!ctx || !ctx->initialized || ctx->device_count >= IS31FL_MAX_DEVICES) {
        LOG_ERROR("IS31FL: Invalid parameters or too many devices");
        return IS31FL_ERROR_PARAM;
    }

    // Check if device already exists
    for (uint8_t i = 0; i < ctx->device_count; i++) {
        if (ctx->devices[i].i2c_addr == i2c_addr) {
            LOG_WARNING("IS31FL: Device 0x%02X already exists", i2c_addr);
            return IS31FL_ERROR_PARAM;
        }
    }

    LOG_DEBUG("IS31FL: Testing device 0x%02X", i2c_addr);

    // Test device presence using write-read transaction
    uint8_t reg_addr = IS31FL_REG_ID; // 0xFC
    uint8_t device_id;
    // Use combined write-read
    IS31FL_Error_t err = ctx->i2c_if->read_reg_byte(i2c_addr, reg_addr, &device_id);
    if (err != IS31FL_OK) {
        LOG_ERROR("IS31FL: Device 0x%02X ID read failed", i2c_addr);
        return IS31FL_ERROR_DEVICE;
    }

    device_id = device_id >> 1;
    // Validate the ID (should match the slave address)
    if (device_id != i2c_addr) {
        LOG_WARNING("IS31FL: Device 0x%02X returned unexpected ID: 0x%02X", i2c_addr, device_id);
    }

    LOG_INFO("IS31FL: Device 0x%02X detected (ID: 0x%02X)", i2c_addr, device_id);

    // Configure device for reliable 3.3V I2C operation
    LOG_DEBUG("IS31FL: Configuring device 0x%02X for 3.3V I2C", i2c_addr);
    // Step 1: Unlock command register
    err = ctx->i2c_if->write_reg_byte(i2c_addr, IS31FL_REG_UNLOCK, 0xC5);
    if (err != IS31FL_OK) {
        LOG_ERROR("IS31FL: Failed to unlock command register for device 0x%02X", i2c_addr);
        return IS31FL_ERROR_DEVICE;
    }
    // Step 2: Select Page 2 (function registers)
    err = ctx->i2c_if->write_reg_byte(i2c_addr, 0xFD, 0x02);
    if (err != IS31FL_OK) {
        LOG_ERROR("IS31FL: Failed to select page 2 for device 0x%02X", i2c_addr);
        return IS31FL_ERROR_DEVICE;
    }

    // Add device to context
    IS31FL_Device_t *device = &ctx->devices[ctx->device_count];
    device->i2c_addr = i2c_addr;
    device->current_page = 0xFF; // Invalid page (will be set when needed)
    device->initialized = true;   // Now properly initialized

    ctx->device_count++;
    LOG_INFO("IS31FL: Device 0x%02X added and configured for 3.3V I2C", i2c_addr);
    return IS31FL_OK;
}

/**
 * @brief Initialize a specific device
 */
IS31FL_Error_t IS31FL_DeviceInit(IS31FL_Context_t *ctx, uint8_t device_idx)
{
    if (!ctx || device_idx >= ctx->device_count) {
        LOG_ERROR("IS31FL: Invalid device index %d", device_idx);
        return IS31FL_ERROR_PARAM;
    }

    IS31FL_Device_t *device = &ctx->devices[device_idx];
    LOG_INFO("IS31FL: Initializing device %d (0x%02X)", device_idx, device->i2c_addr);

    IS31FL_Error_t err;

    // Reset device
    LOG_DEBUG("IS31FL: Resetting device %d", device_idx);
    err = IS31FL_Reset(ctx, device_idx);
    if (err != IS31FL_OK) {
        LOG_ERROR("IS31FL: Device %d reset failed", device_idx);
        return err;
    }

    ctx->i2c_if->delay_ms(10); // Wait for reset

    // Select function page
    err = SelectPage(ctx, device_idx, IS31FL_PAGE_FUNCTION);
    if (err != IS31FL_OK) {
        LOG_ERROR("IS31FL: Device %d function page select failed", device_idx);
        return err;
    }

    // Enable device (disable software shutdown)
    err = WriteRegister(ctx, device_idx, IS31FL_REG_CONFIG, IS31FL_CONFIG_SSD);
    if (err != IS31FL_OK) {
        LOG_ERROR("IS31FL: Device %d enable failed", device_idx);
        return err;
    }

    // Set maximum global current
    err = WriteRegister(ctx, device_idx, IS31FL_REG_GLOBAL_CURRENT, 0x40);
    if (err != IS31FL_OK) {
        LOG_ERROR("IS31FL: Device %d global current set failed", device_idx);
        return err;
    }
    if(device_idx == 0){
    	uint8_t thermal_config = (IS31FL_TS_140C << 2) | IS31FL_TROF_100_PERCENT;
		err = WriteRegister(ctx, device_idx, IS31FL_REG_THERMAL_CONFIG, thermal_config);
		if (err != IS31FL_OK) {
			LOG_ERROR("IS31FL: Device 0 Thermal protection threshold set failed");
			return err;
		}

		err = WriteRegister(ctx, device_idx, 0x25, 0x31);
		if (err != IS31FL_OK) {
			LOG_ERROR("IS31FL: Device 0 failed to set Spread Spectrum");
			return err;
		}
    }
    // Configure pull-up/down resistors for de-ghosting
    err = WriteRegister(ctx, device_idx, IS31FL_REG_PULLUP, 0x00);
    if (err != IS31FL_OK) {
        LOG_WARNING("IS31FL: Device %d pullup config failed", device_idx);
    }

    // Clear all PWM registers
    err = IS31FL_SetAllLEDs(ctx, device_idx, 0);
    if (err != IS31FL_OK) {
        LOG_ERROR("IS31FL: Device %d PWM clear failed", device_idx);
        return err;
    }

    // Set all scaling to maximum
    err = SelectPage(ctx, device_idx, IS31FL_PAGE_SCALING);
    if (err != IS31FL_OK) {
        LOG_ERROR("IS31FL: Device %d scaling page select failed", device_idx);
        return err;
    }

    for (uint16_t i = 1; i <= IS31FL_MAX_LEDS; i++) {
        err = WriteRegister(ctx, device_idx, i, 0xFF);
        if (err != IS31FL_OK) {
            LOG_ERROR("IS31FL: Device %d scaling setup failed at LED %d", device_idx, i);
            return err;
        }
    }

    device->initialized = true;
    LOG_INFO("IS31FL: Device %d initialization complete", device_idx);
    return IS31FL_OK;
}

/**
 * @brief Reset device to default state
 */
IS31FL_Error_t IS31FL_Reset(IS31FL_Context_t *ctx, uint8_t device_idx)
{
    if (!ctx || device_idx >= ctx->device_count) {
        return IS31FL_ERROR_PARAM;
    }

    IS31FL_Device_t *device = &ctx->devices[device_idx];
    IS31FL_Error_t err;

    err = UnlockCommand(ctx, device_idx);
    if (err != IS31FL_OK) return err;

    err = SelectPage(ctx, device_idx, IS31FL_PAGE_FUNCTION);
    if (err != IS31FL_OK) return err;

    err = WriteRegister(ctx, device_idx, IS31FL_REG_RESET, 0xAE);
    if (err != IS31FL_OK) return err;

    device->current_page = 0xFF;
    device->initialized = false;

    return IS31FL_OK;
}

/**
 * @brief Software shutdown control
 */
IS31FL_Error_t IS31FL_Shutdown(IS31FL_Context_t *ctx, uint8_t device_idx, bool enable)
{
    if (!ctx || device_idx >= ctx->device_count) {
        return IS31FL_ERROR_PARAM;
    }

    IS31FL_Error_t err = SelectPage(ctx, device_idx, IS31FL_PAGE_FUNCTION);
    if (err != IS31FL_OK) return err;

    uint8_t config = enable ? 0x00 : IS31FL_CONFIG_SSD;
    return WriteRegister(ctx, device_idx, IS31FL_REG_CONFIG, config);
}

/**
 * @brief Set individual LED brightness
 */
IS31FL_Error_t IS31FL_SetLED(IS31FL_Context_t *ctx, uint8_t device_idx,
                             uint8_t cs, uint8_t sw, uint8_t brightness)
{
    if (!ctx || device_idx >= ctx->device_count || !IS31FL_ValidatePosition(cs, sw)) {
        return IS31FL_ERROR_PARAM;
    }

    IS31FL_Error_t err = SelectPage(ctx, device_idx, IS31FL_PAGE_PWM);
    if (err != IS31FL_OK) return err;

    uint8_t reg_addr = IS31FL_GetRegisterAddr(cs, sw);
    return WriteRegister(ctx, device_idx, reg_addr, brightness);
}

/**
 * @brief Set all LEDs to same brightness
 */
IS31FL_Error_t IS31FL_SetAllLEDs(IS31FL_Context_t *ctx, uint8_t device_idx,
                                 uint8_t brightness)
{
    if (!ctx || device_idx >= ctx->device_count) {
        return IS31FL_ERROR_PARAM;
    }

    IS31FL_Error_t err = SelectPage(ctx, device_idx, IS31FL_PAGE_PWM);
    if (err != IS31FL_OK) return err;

    // Write to all PWM registers
    for (uint16_t i = 1; i <= IS31FL_MAX_LEDS; i++) {
        err = WriteRegister(ctx, device_idx, i, brightness);
        if (err != IS31FL_OK) return err;
    }

    return IS31FL_OK;
}

/**
 * @brief Update entire matrix with new data
 */
IS31FL_Error_t IS31FL_UpdateMatrix(IS31FL_Context_t *ctx, uint8_t device_idx,
                                   const uint8_t *matrix_data, uint16_t size)
{
    if (!ctx || device_idx >= ctx->device_count || !matrix_data || size > IS31FL_MAX_LEDS) {
        return IS31FL_ERROR_PARAM;
    }

    IS31FL_Error_t err = SelectPage(ctx, device_idx, IS31FL_PAGE_PWM);
    if (err != IS31FL_OK) return err;

    // Write matrix data using auto-increment
    const uint8_t *data_ptr = matrix_data;
    uint16_t bytes_written = 0;

    while (bytes_written < size) {
        // Write in chunks to avoid I2C buffer overflow
        uint16_t chunk_size = (size - bytes_written > 32) ? 32 : (size - bytes_written);

        for (uint16_t i = 0; i < chunk_size; i++) {
            err = WriteRegister(ctx, device_idx, 1 + bytes_written + i, data_ptr[i]);
            if (err != IS31FL_OK) return err;
        }

        data_ptr += chunk_size;
        bytes_written += chunk_size;
    }

    return IS31FL_OK;
}

/**
 * @brief Set global current control
 */
IS31FL_Error_t IS31FL_SetGlobalCurrent(IS31FL_Context_t *ctx, uint8_t device_idx,
                                       uint8_t current)
{
    if (!ctx || device_idx >= ctx->device_count) {
        return IS31FL_ERROR_PARAM;
    }

    IS31FL_Error_t err = SelectPage(ctx, device_idx, IS31FL_PAGE_FUNCTION);
    if (err != IS31FL_OK) return err;

    return WriteRegister(ctx, device_idx, IS31FL_REG_GLOBAL_CURRENT, current);
}

/**
 * @brief Set LED scaling value
 */
IS31FL_Error_t IS31FL_SetScaling(IS31FL_Context_t *ctx, uint8_t device_idx,
                                 uint8_t cs, uint8_t sw, uint8_t scaling)
{
    if (!ctx || device_idx >= ctx->device_count || !IS31FL_ValidatePosition(cs, sw)) {
        return IS31FL_ERROR_PARAM;
    }

    IS31FL_Error_t err = SelectPage(ctx, device_idx, IS31FL_PAGE_SCALING);
    if (err != IS31FL_OK) return err;

    uint8_t reg_addr = IS31FL_GetRegisterAddr(cs, sw);
    return WriteRegister(ctx, device_idx, reg_addr, scaling);
}

/**
 * @brief Get register address for LED position
 */
uint8_t IS31FL_GetRegisterAddr(uint8_t cs, uint8_t sw)
{
    if (!IS31FL_ValidatePosition(cs, sw)) {
        return 0;
    }

    // Convert CS(1-18) and SW(1-11) to register address
    uint16_t led_index = ((sw - 1) * 18) + (cs - 1);
    return (uint8_t)(0x01 + led_index);
}

/**
 * @brief Validate LED position
 */
bool IS31FL_ValidatePosition(uint8_t cs, uint8_t sw)
{
    return (cs >= 1 && cs <= IS31FL_CS_COUNT && sw >= 1 && sw <= IS31FL_SW_COUNT);
}


IS31FL_Error_t IS31FL_ConfigureSync(IS31FL_Context_t *ctx, uint8_t device_idx, bool is_master)
{
    if (!ctx || device_idx >= ctx->device_count) {
        LOG_ERROR("IS31FL: Invalid sync parameters");
        return IS31FL_ERROR_PARAM;
    }

    LOG_INFO("IS31FL: Configuring device %d as %s", device_idx, is_master ? "master" : "slave");

    IS31FL_Error_t err = SelectPage(ctx, device_idx, IS31FL_PAGE_FUNCTION);
    if (err != IS31FL_OK) {
        LOG_ERROR("IS31FL: Sync page select failed for device %d", device_idx);
        return err;
    }

    uint8_t sync_value = is_master ? 0xC0 : 0x80;  // Master or slave mode
    err = WriteRegister(ctx, device_idx, 0x25, sync_value);

    if (err == IS31FL_OK) {
        LOG_INFO("IS31FL: Device %d sync configured successfully", device_idx);
    } else {
        LOG_ERROR("IS31FL: Device %d sync configuration failed", device_idx);
    }

    return err;
}


// Private Functions

/**
 * @brief Write to device register
 */
static IS31FL_Error_t WriteRegister(const IS31FL_Context_t *ctx, uint8_t device_idx,
                                   uint8_t reg, uint8_t value)
{
    if (device_idx >= ctx->device_count) {
        LOG_ERROR("IS31FL: Invalid device index in write: %d", device_idx);
        return IS31FL_ERROR_PARAM;
    }

    uint8_t data[2] = {reg, value};
    IS31FL_Error_t result = ctx->i2c_if->write(ctx->devices[device_idx].i2c_addr, data, 2);

    if (result != IS31FL_OK) {
        LOG_DEBUG("IS31FL: Write failed - dev:%d reg:0x%02X val:0x%02X", device_idx, reg, value);
    }

    return result;
}

/**
 * @brief Read from device register
 */
__unused static IS31FL_Error_t ReadRegister(const IS31FL_Context_t *ctx, uint8_t device_idx,
                                 uint8_t reg, uint8_t *value)
{
   if (device_idx >= ctx->device_count || !value) {
       LOG_ERROR("IS31FL: Invalid parameters in read - dev:%d", device_idx);
       return IS31FL_ERROR_PARAM;
   }

   IS31FL_Error_t err = ctx->i2c_if->write(ctx->devices[device_idx].i2c_addr, &reg, 1);
   if (err != IS31FL_OK) {
       LOG_DEBUG("IS31FL: Read address write failed - dev:%d reg:0x%02X", device_idx, reg);
       return err;
   }

   err = ctx->i2c_if->read(ctx->devices[device_idx].i2c_addr, value, 1);
   if (err != IS31FL_OK) {
       LOG_DEBUG("IS31FL: Read data failed - dev:%d reg:0x%02X", device_idx, reg);
   }

   return err;
}

/**
 * @brief Select register page
 */
static IS31FL_Error_t SelectPage(const IS31FL_Context_t *ctx, uint8_t device_idx,
                                uint8_t page)
{
    if (device_idx >= ctx->device_count) {
        return IS31FL_ERROR_PARAM;
    }

    IS31FL_Device_t *device = &ctx->devices[device_idx];

    if (device->current_page == page) {
        return IS31FL_OK; // Already on correct page
    }

    IS31FL_Error_t err = UnlockCommand(ctx, device_idx);
    if (err != IS31FL_OK) return err;

    err = WriteRegister(ctx, device_idx, IS31FL_REG_COMMAND, page);
    if (err == IS31FL_OK) {
        device->current_page = page;
    }

    return err;
}

/**
 * @brief Unlock command register
 */
static IS31FL_Error_t UnlockCommand(const IS31FL_Context_t *ctx, uint8_t device_idx)
{
    return WriteRegister(ctx, device_idx, IS31FL_REG_UNLOCK, 0xC5);
}

/**
 * @brief Read device temperature register
 */
IS31FL_Error_t IS31FL_ReadTemperature(IS31FL_Context_t *ctx, uint8_t device_idx, uint8_t *temp_raw)
{
    if (!ctx || device_idx >= ctx->device_count || !temp_raw) {
        return IS31FL_ERROR_PARAM;
    }

    IS31FL_Error_t err = SelectPage(ctx, device_idx, IS31FL_PAGE_FUNCTION);
    if (err != IS31FL_OK) return err;

    uint8_t temp_reg;
    err = ReadRegister(ctx, device_idx, 0xDB, &temp_reg);
    if (err != IS31FL_OK) return err;

    // Extract temperature from upper 4 bits
    *temp_raw = (temp_reg >> 4) & 0x0F;

    return IS31FL_OK;
}

/**
 * @brief Convert raw temperature to estimated Celsius
 */
uint8_t IS31FL_TempToCelsius(uint8_t temp_raw)
{
    // Based on IS31FL3743A datasheet thermal rollback points
    switch (temp_raw & 0x0F) {
        case 0x00: return 85;   // Normal operation
        case 0x01: return 95;   // Approaching 90°C threshold
        case 0x02: return 105;  // 100°C threshold (30% reduction)
        case 0x03: return 115;  // 110°C threshold (55% reduction)
        default:   return 125;  // 120°C+ threshold (75% reduction)
    }
}

/**
 * @brief Check if device is in thermal protection mode
 */
IS31FL_Error_t IS31FL_IsThermalProtected(IS31FL_Context_t *ctx, uint8_t device_idx, bool *is_protected)
{
    if (!ctx || device_idx >= ctx->device_count || !is_protected) {
        return IS31FL_ERROR_PARAM;
    }

    uint8_t temp_raw;
    IS31FL_Error_t err = IS31FL_ReadTemperature(ctx, device_idx, &temp_raw);
    if (err != IS31FL_OK) return err;

    // Device is in thermal protection if temp_raw > 0
    *is_protected = (temp_raw > 0);

    return IS31FL_OK;
}

IS31FL_Error_t IS31FL_SetTROF(IS31FL_Context_t *ctx, uint8_t device_idx, IS31FL_TROF_t trof)
{
    if (!ctx || device_idx >= ctx->device_count) {
        return IS31FL_ERROR_PARAM;
    }

    IS31FL_Error_t err = SelectPage(ctx, device_idx, IS31FL_PAGE_FUNCTION);
    if (err != IS31FL_OK) return err;

    // Read current thermal config register
    uint8_t thermal_reg;
    err = ReadRegister(ctx, device_idx, IS31FL_REG_THERMAL_CONFIG, &thermal_reg);
    if (err != IS31FL_OK) return err;

    // Clear TROF bits (D1:D0) and set new value
    thermal_reg &= 0xFC;  // Clear lower 2 bits
    thermal_reg |= trof;  // Set new TROF value in bits 1:0

    err = WriteRegister(ctx, device_idx, IS31FL_REG_THERMAL_CONFIG, thermal_reg);

    LOG_INFO("Device %d TROF set to %d%%", device_idx,
             (trof == IS31FL_TROF_100_PERCENT) ? 100 :
             (trof == IS31FL_TROF_75_PERCENT) ? 75 :
             (trof == IS31FL_TROF_55_PERCENT) ? 55 : 30);

    return err;
}


/**
 * @brief Set thermal start point (TS)
 */
IS31FL_Error_t IS31FL_SetTS(IS31FL_Context_t *ctx, uint8_t device_idx, IS31FL_TS_t ts)
{
    if (!ctx || device_idx >= ctx->device_count) {
        return IS31FL_ERROR_PARAM;
    }

    IS31FL_Error_t err = SelectPage(ctx, device_idx, IS31FL_PAGE_FUNCTION);
    if (err != IS31FL_OK) return err;

    // Read current thermal config register
    uint8_t thermal_reg;
    err = ReadRegister(ctx, device_idx, IS31FL_REG_THERMAL_CONFIG, &thermal_reg);
    if (err != IS31FL_OK) return err;

    // Clear TS bits (D3:D2) and set new value
    thermal_reg &= 0xF3;  // Clear bits 3:2
    thermal_reg |= (ts << 2);    // Set new TS value in bits 3:2

    err = WriteRegister(ctx, device_idx, IS31FL_REG_THERMAL_CONFIG, thermal_reg);

    LOG_INFO("Device %d TS set to %d°C", device_idx,
             (ts == IS31FL_TS_90C) ? 90 :
             (ts == IS31FL_TS_100C) ? 100 :
             (ts == IS31FL_TS_120C) ? 120 : 140);

    return err;
}

/**
 * @brief Configure thermal protection settings
 */
IS31FL_Error_t IS31FL_ConfigureThermalProtection(IS31FL_Context_t *ctx, uint8_t device_idx,
                                                 IS31FL_TS_t ts, IS31FL_TROF_t trof)
{
    if (!ctx || device_idx >= ctx->device_count) {
        return IS31FL_ERROR_PARAM;
    }

    IS31FL_Error_t err = SelectPage(ctx, device_idx, IS31FL_PAGE_FUNCTION);
    if (err != IS31FL_OK) return err;

    // Combine TS and TROF into single register value
    // TS in bits 3:2, TROF in bits 1:0
    uint8_t thermal_config = (ts << 2) | trof;

    err = WriteRegister(ctx, device_idx, IS31FL_REG_THERMAL_CONFIG, thermal_config);

    if (err == IS31FL_OK) {
        LOG_INFO("Device %d thermal protection: %d°C start, %d%% reduction",
                 device_idx,
                 (ts == IS31FL_TS_90C) ? 90 : (ts == IS31FL_TS_100C) ? 100 :
                 (ts == IS31FL_TS_120C) ? 120 : 140,
                 (trof == IS31FL_TROF_100_PERCENT) ? 100 : (trof == IS31FL_TROF_75_PERCENT) ? 75 :
                 (trof == IS31FL_TROF_55_PERCENT) ? 55 : 30);
    }

    return err;
}


/**
 * @brief Read current TROF setting
 */
IS31FL_Error_t IS31FL_ReadTROF(IS31FL_Context_t *ctx, uint8_t device_idx, IS31FL_TROF_t *trof)
{
    if (!ctx || device_idx >= ctx->device_count || !trof) {
        return IS31FL_ERROR_PARAM;
    }

    IS31FL_Error_t err = SelectPage(ctx, device_idx, IS31FL_PAGE_FUNCTION);
    if (err != IS31FL_OK) return err;

    uint8_t thermal_reg;
    err = ReadRegister(ctx, device_idx, IS31FL_REG_THERMAL_CONFIG, &thermal_reg);
    if (err != IS31FL_OK) return err;

    // Extract TROF from lower 2 bits (D1:D0)
    *trof = (IS31FL_TROF_t)(thermal_reg & 0x03);

    return IS31FL_OK;
}

/**
 * @brief Read current TS setting
 */
IS31FL_Error_t IS31FL_ReadTS(IS31FL_Context_t *ctx, uint8_t device_idx, IS31FL_TS_t *ts)
{
    if (!ctx || device_idx >= ctx->device_count || !ts) {
        return IS31FL_ERROR_PARAM;
    }

    IS31FL_Error_t err = SelectPage(ctx, device_idx, IS31FL_PAGE_FUNCTION);
    if (err != IS31FL_OK) return err;

    uint8_t thermal_reg;
    err = ReadRegister(ctx, device_idx, IS31FL_REG_THERMAL_CONFIG, &thermal_reg);
    if (err != IS31FL_OK) return err;

    // Extract TS from bits 3:2
    *ts = (IS31FL_TS_t)((thermal_reg >> 2) & 0x03);

    return IS31FL_OK;
}

/**
 * @brief Read thermal configuration register raw value
 */
IS31FL_Error_t IS31FL_ReadThermalConfig(IS31FL_Context_t *ctx, uint8_t device_idx, uint8_t *thermal_config)
{
    if (!ctx || device_idx >= ctx->device_count || !thermal_config) {
        return IS31FL_ERROR_PARAM;
    }

    IS31FL_Error_t err = SelectPage(ctx, device_idx, IS31FL_PAGE_FUNCTION);
    if (err != IS31FL_OK) return err;

    return ReadRegister(ctx, device_idx, IS31FL_REG_THERMAL_CONFIG, thermal_config);
}
