/**
 * @file backlight.c
 * @brief Simplified Keyboard RGB Backlight Implementation
 * @version 1.0
 *
 * Simplified implementation using blocking I2C with timeouts
 * RGB-native interface with keyboard coordinate mapping
 * Optimized for STM32L072VZ (Cortex-M0+, no FPU)
 */

#include "backlight.h"
#include "led_mapping.h"
#include "is31fl_driver.h"
#include "stm32l0xx_hal.h"
#include "logger.h"
#include <string.h>

/* ========================================================================== */
/* Private Configuration */
/* ========================================================================== */

#define BACKLIGHT_I2C_TIMEOUT_MS        	50      // I2C operation timeout
#define BACKLIGHT_FADE_UPDATE_INTERVAL_MS  	16   // ~60 FPS fade updates
#define BACKLIGHT_MAX_FADE_STEPS        	32      // Maximum fade resolution
#define BACKLIGHT_RGB_MAX_VALUE          	255     // Maximum RGB value
#define BACKLIGHT_BRIGHTNESS_STEP        	10     // RGB step size for quick brightness adjustment
#define BACKLIGHT_RGB_MIN_VALUE			 	1
/* ========================================================================== */
/* Private Types */
/* ========================================================================== */

typedef struct {
    IS31FL_Context_t led_ctx;

    // Current state
    uint8_t global_brightness;
    uint8_t user_brightness_level;
    uint8_t current_max_brightness;
    uint8_t saved_brightness;
    Backlight_RGB_t current_colors[BACKLIGHT_MATRIX_ROWS][BACKLIGHT_MATRIX_COLS];

    uint8_t user_max_limit;
	uint8_t user_min_limit;

    // Saved color state for restore from off
	Backlight_RGB_t saved_colors[BACKLIGHT_MATRIX_ROWS][BACKLIGHT_MATRIX_COLS];
	bool has_saved_state;
	uint8_t saved_global_max;

    // Hardware status
    bool master_online;
    bool slave_online;
    bool initialized;

    // Error tracking
    uint32_t i2c_errors;
    uint32_t timeout_errors;
} backlight_context_t;

/* ========================================================================== */
/* Private Variables */
/* ========================================================================== */
static backlight_context_t g_ctx = {0};

static const uint8_t hue_brightness_correction[24] = {
    255, 255, 242, 230, 191, 191, 204, 230, 255, 255, 242, 204,
    191, 191, 217, 242, 255, 255, 255, 242, 242, 255, 255, 255
};

static const uint8_t hue_saturation_boost[24] = {
	100,105,110,115,120,115,110,105,100,100,100,105,
	110,105,100,100,100,100,105,110,115,110,105,100
};

static const int8_t hue_shift_correction[24] = {
	0,0,-3,-5,-8,-5,-3,0,0,0,0,-3,
	-5,-3,0,0,0,0,0,3,5,3,0,0
};

const uint8_t brightness_levels[11] = {
    0,   // Level 0 - Off
    20,  // Level 1 - Minimum visible
    31,  // Level 2 - Very dim
    51,  // Level 3 - Dim
    77,  // Level 4 - Low
    105, // Level 5 - Medium-low
    140, // Level 6 - Medium
    179, // Level 7 - Medium-high
    217, // Level 8 - High
    242, // Level 9 - Very high
    255  // Level 10 - Maximum
};

/* ========================================================================== */
/* Private Function Prototypes */
/* ========================================================================== */

static Backlight_Error_t InitDevices(void);
static Backlight_Error_t UpdateLEDHardware(uint8_t row, uint8_t col, Backlight_RGB_t color);
static void Backlight_ReadThermalConfig(uint8_t device_idx);
/* ========================================================================== */
/* Private Functions */
/* ========================================================================== */


static uint8_t LimitRGBComponent(uint8_t value){
    return (value > BACKLIGHT_RGB_MAX_VALUE) ? BACKLIGHT_RGB_MAX_VALUE : value;
}


static void SaveColorState(void)
{
    // Copy current colors to saved state
    memcpy(g_ctx.saved_colors, g_ctx.current_colors, sizeof(g_ctx.current_colors));

    // Find and save the current global maximum
    g_ctx.saved_global_max = 0;
    for (uint8_t row = 0; row < BACKLIGHT_MATRIX_ROWS; row++) {
        for (uint8_t col = 0; col < BACKLIGHT_MATRIX_COLS; col++) {
            if (Backlight_HasLED(row, col)) {
                Backlight_RGB_t current = g_ctx.current_colors[row][col];
                uint8_t key_max = current.r;
                if (current.g > key_max) key_max = current.g;
                if (current.b > key_max) key_max = current.b;
                if (key_max > g_ctx.saved_global_max) {
                    g_ctx.saved_global_max = key_max;
                }
            }
        }
    }

    g_ctx.has_saved_state = (g_ctx.saved_global_max > 0);

    if (g_ctx.has_saved_state) {
        LOG_INFO("Color state saved - Global max: %d", g_ctx.saved_global_max);
    }
}

/**
 * @brief Restore saved color state at minimum brightness
 */
static Backlight_Error_t RestoreColorState(void)
{
    if (!g_ctx.has_saved_state) {
        // No saved state, use default yellow
        LOG_INFO("No saved state - using default yellow");
        return Backlight_SetAll(BACKLIGHT_RGB_MIN_VALUE, 0, 0);
    }

    // Restore colors at minimum brightness level
    for (uint8_t row = 0; row < BACKLIGHT_MATRIX_ROWS; row++) {
        for (uint8_t col = 0; col < BACKLIGHT_MATRIX_COLS; col++) {
            if (Backlight_HasLED(row, col)) {
                Backlight_RGB_t saved = g_ctx.saved_colors[row][col];

                if (saved.r > 0 || saved.g > 0 || saved.b > 0) {
                    // Scale saved color to minimum brightness
                    Backlight_RGB_t restore_color = {
                        .r = LimitRGBComponent((saved.r * BACKLIGHT_RGB_MIN_VALUE) / g_ctx.saved_global_max),
                        .g = LimitRGBComponent((saved.g * BACKLIGHT_RGB_MIN_VALUE) / g_ctx.saved_global_max),
                        .b = LimitRGBComponent((saved.b * BACKLIGHT_RGB_MIN_VALUE) / g_ctx.saved_global_max)
                    };

                    UpdateLEDHardware(row, col, restore_color);
                    g_ctx.current_colors[row][col] = restore_color;
                }
            }
        }
    }

    LOG_INFO("Color state restored at minimum brightness");
    return BACKLIGHT_OK;
}

static Backlight_RGB_t LimitRGBColor(Backlight_RGB_t color)
{
    Backlight_RGB_t limited;
    limited.r = LimitRGBComponent(color.r);
    limited.g = LimitRGBComponent(color.g);
    limited.b = LimitRGBComponent(color.b);
    return limited;
}


bool Backlight_IsAllOff(void)
{
    if (!g_ctx.initialized) {
        return true;
    }

    for (uint8_t row = 0; row < BACKLIGHT_MATRIX_ROWS; row++) {
        for (uint8_t col = 0; col < BACKLIGHT_MATRIX_COLS; col++) {
            if (Backlight_HasLED(row, col)) {
                Backlight_RGB_t current = g_ctx.current_colors[row][col];
                if (current.r > 0 || current.g > 0 || current.b > 0) {
                    return false;
                }
            }
        }
    }

    return true;
}

// Gamma correction lookup table (stored in flash, no FPU needed)
static const uint8_t gamma_table[32] = {
    0, 1, 2, 4, 6, 10, 13, 18, 22, 28, 33, 39, 46, 53, 61, 69,
    78, 86, 96, 106, 116, 126, 138, 149, 161, 173, 186, 199, 212, 226, 240, 255
};



/* ========================================================================== */
/* External Dependencies */
/* ========================================================================== */

extern void HAL_Delay(uint32_t ms);
extern uint32_t HAL_GetTick(void);


/* ========================================================================== */
/* I2C Configuration */
/* ========================================================================== */

#define I2C_TIMING_400KHZ    0x00201D1DUL
#define I2C_TIMEOUT_MS       100

extern I2C_HandleTypeDef hi2c1;

/* ========================================================================== */
/* HAL I2C Interface Implementation */
/* ========================================================================== */

static HAL_StatusTypeDef I2C1_Init_Backlight(I2C_HandleTypeDef *inst)
{
	inst->Instance = I2C1;
	inst->Init.Timing = I2C_TIMING_400KHZ;  // Use your specified timing
	inst->Init.OwnAddress1 = 0;
	inst->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	inst->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	inst->Init.OwnAddress2 = 0;
	inst->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	inst->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	inst->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    HAL_StatusTypeDef status = HAL_I2C_Init(inst);
    if (status != HAL_OK) {
        LOG_ERROR("I2C2 HAL init failed: %d", status);
        return status;
    }

    status = HAL_I2CEx_ConfigAnalogFilter(inst, I2C_ANALOGFILTER_ENABLE);
    if (status != HAL_OK) {
        LOG_ERROR("I2C2 analog filter config failed: %d", status);
        return status;
    }

    status = HAL_I2CEx_ConfigDigitalFilter(inst, 0);
    if (status != HAL_OK) {
        LOG_ERROR("I2C2 digital filter config failed: %d", status);
        return status;
    }

    I2C1->CR1 = I2C_CR1_PE;

    return HAL_OK;
}

static IS31FL_Error_t hal_i2c_write(uint8_t addr, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, addr << 1, data, len, I2C_TIMEOUT_MS);

    switch(status) {
        case HAL_OK:
            return IS31FL_OK;
        case HAL_TIMEOUT:
            LOG_WARNING("I2C write timeout to 0x%02X", addr);
            return IS31FL_ERROR_TIMEOUT;
        case HAL_ERROR:
            LOG_ERROR("I2C write error to 0x%02X", addr);
            return IS31FL_ERROR_I2C;
        default:
            return IS31FL_ERROR_DEVICE;
    }
}

static IS31FL_Error_t hal_i2c_read(uint8_t addr, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&hi2c1, addr << 1, data, len, I2C_TIMEOUT_MS);

    switch(status) {
        case HAL_OK:
            return IS31FL_OK;
        case HAL_TIMEOUT:
            LOG_WARNING("I2C read timeout from 0x%02X", addr);
            return IS31FL_ERROR_TIMEOUT;
        case HAL_ERROR:
            LOG_ERROR("I2C read error from 0x%02X", addr);
            return IS31FL_ERROR_I2C;
        default:
            return IS31FL_ERROR_DEVICE;
    }
}

static IS31FL_Error_t hal_i2c_write_reg_byte(uint8_t addr, uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2C_TIMEOUT_MS);

    switch(status) {
        case HAL_OK:
            return IS31FL_OK;
        case HAL_TIMEOUT:
            LOG_WARNING("I2C reg write timeout to 0x%02X:0x%02X", addr, reg);
            return IS31FL_ERROR_TIMEOUT;
        case HAL_ERROR:
            LOG_ERROR("I2C reg write error to 0x%02X:0x%02X", addr, reg);
            return IS31FL_ERROR_I2C;
        default:
            return IS31FL_ERROR_DEVICE;
    }
}

static IS31FL_Error_t hal_i2c_read_reg_byte(uint8_t addr, uint8_t reg, uint8_t* value)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, value, 1, I2C_TIMEOUT_MS);

    switch(status) {
        case HAL_OK:
            return IS31FL_OK;
        case HAL_TIMEOUT:
            LOG_WARNING("I2C reg read timeout from 0x%02X:0x%02X", addr, reg);
            return IS31FL_ERROR_TIMEOUT;
        case HAL_ERROR:
            LOG_ERROR("I2C reg read error from 0x%02X:0x%02X", addr, reg);
            return IS31FL_ERROR_I2C;
        default:
            return IS31FL_ERROR_DEVICE;
    }
}

static IS31FL_Error_t hal_i2c_write_read(uint8_t addr, const uint8_t* tx_data, uint16_t tx_len,
                                         uint8_t* rx_data, uint16_t rx_len)
{
    // First write the command/register
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, addr << 1, (uint8_t*)tx_data, tx_len, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        LOG_ERROR("I2C write phase failed to 0x%02X", addr);
        return (status == HAL_TIMEOUT) ? IS31FL_ERROR_TIMEOUT : IS31FL_ERROR_I2C;
    }

    // Then read the response
    status = HAL_I2C_Master_Receive(&hi2c1, addr << 1, rx_data, rx_len, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        LOG_ERROR("I2C read phase failed from 0x%02X", addr);
        return (status == HAL_TIMEOUT) ? IS31FL_ERROR_TIMEOUT : IS31FL_ERROR_I2C;
    }

    return IS31FL_OK;
}

static void hal_clear_i2c_irq(I2C_TypeDef* i2c)
{
    // HAL manages interrupts internally, no manual clearing needed
    (void)i2c;  // Suppress unused parameter warning
}

static const IS31FL_I2C_Interface_t i2c_interface = {
    .write = hal_i2c_write,
    .read = hal_i2c_read,
	.write_read = hal_i2c_write_read,
	.write_reg_byte = hal_i2c_write_reg_byte,
	.read_reg_byte = hal_i2c_read_reg_byte,
	.clear_i2c_irq = hal_clear_i2c_irq,
    .delay_ms = HAL_Delay
};

/* ========================================================================== */
/* Core API Implementation */
/* ========================================================================== */
// Helper function to find LED mapping for a key

Backlight_Error_t Backlight_Init(void)
{
    if (g_ctx.initialized) {
        return BACKLIGHT_OK;
    }

    memset(&g_ctx, 0, sizeof(g_ctx));

    for (uint8_t row = 0; row < BACKLIGHT_MATRIX_ROWS; row++) {
		for (uint8_t col = 0; col < BACKLIGHT_MATRIX_COLS; col++) {
			g_ctx.current_colors[row][col] = (Backlight_RGB_t){0, 0, 0};
			g_ctx.saved_colors[row][col] = (Backlight_RGB_t){0, 0, 0};
		}
	}

	// Initialize state flags
	g_ctx.has_saved_state = false;
	g_ctx.saved_global_max = 0;
	g_ctx.global_brightness = BACKLIGHT_BRIGHTNESS_MAX;
	g_ctx.saved_brightness = BACKLIGHT_BRIGHTNESS_MAX;
	g_ctx.user_max_limit = 255;
	g_ctx.user_min_limit = 0;
	g_ctx.current_max_brightness = g_ctx.user_max_limit;

    if(I2C1_Init_Backlight(&hi2c1) != 0){
    	LOG_ERROR("I2C peripheral initialisation error");
    	return BACKLIGHT_ERROR_INIT;
    }
    // Initialize LED driver context
    IS31FL_Error_t err = IS31FL_Init(&g_ctx.led_ctx, &i2c_interface);
    if (err != IS31FL_OK) {
    	LOG_ERROR("Backlight Initialisation error");
        return BACKLIGHT_ERROR_INIT;
    }

    // Initialize devices
    Backlight_Error_t result = InitDevices();
    if (result != BACKLIGHT_OK) {
    	LOG_ERROR("Backlight LED Driver error");
        return result;
    }

    Backlight_ReadThermalConfig(0); // Read master thermal config
    Backlight_ReadThermalConfig(1); // Read slave thermal config

    Backlight_RGB_t color;
    color.b = 0;
    color.g = 0;
    color.r = 0;

    Backlight_SetAllRGB(color);

    g_ctx.initialized = true;

    LOG_INFO("Backlight Core and Hardware initialised");

    return BACKLIGHT_OK;
}

Backlight_Error_t Backlight_DeInit(void)
{
    if (!g_ctx.initialized) {
        return BACKLIGHT_OK;
    }

    Backlight_EmergencyOff();
    g_ctx.initialized = false;

    return BACKLIGHT_OK;
}


Backlight_Error_t Backlight_EmergencyOff(void)
{
    return Backlight_EnterShutdown();
}

/* ========================================================================== */
/* Global Brightness Control */
/* ========================================================================== */

Backlight_Error_t Backlight_SetCurrentLimit(uint8_t brightness)
{
    if (!g_ctx.initialized) {
        LOG_ERROR("Backlight not initialized");
        return BACKLIGHT_ERROR_INIT;
    }

    LOG_DEBUG("Setting brightness to %d", brightness);
    g_ctx.global_brightness = brightness;

    // Update hardware with error checking
    bool success = true;
    if (g_ctx.master_online) {
        if (IS31FL_SetGlobalCurrent(&g_ctx.led_ctx, 0, brightness) != IS31FL_OK) {
            LOG_ERROR("Failed to set master brightness");
            success = false;
        }
    }
    if (g_ctx.slave_online) {
        if (IS31FL_SetGlobalCurrent(&g_ctx.led_ctx, 1, brightness) != IS31FL_OK) {
            LOG_ERROR("Failed to set slave brightness");
            success = false;
        }
    }

    if (success) {
        LOG_INFO("Brightness set to %d", brightness);
    }

    return BACKLIGHT_OK;
}

Backlight_Error_t Backlight_GetBrightness(uint8_t *brightness)
{
    if (!brightness) {
        return BACKLIGHT_ERROR_PARAM;
    }

    *brightness = g_ctx.global_brightness;
    return BACKLIGHT_OK;
}

Backlight_Error_t Backlight_Off(void)
{
    return Backlight_EnterShutdown();
}

Backlight_Error_t Backlight_On(void)
{
    Backlight_Error_t result = Backlight_ExitShutdown();
    if (result == BACKLIGHT_OK) {
        HAL_Delay(1); // Small delay for device ready
        return RestoreColorState();
    }
    return result;
}

Backlight_Error_t Backlight_IncreaseBrightness(void)
{
    if (!g_ctx.initialized) {
        return BACKLIGHT_ERROR_INIT;
    }

    // If all LEDs are off, restore saved color state at minimum brightness
    if (Backlight_IsAllOff()) {
        LOG_INFO("LEDs off - exiting shutdown and restoring colors");

        // Exit software shutdown mode
        Backlight_Error_t result = Backlight_ExitShutdown();
        if (result != BACKLIGHT_OK) {
            return result;
        }

        // Small delay to ensure device is ready
        HAL_Delay(1);

        // Restore saved color state at minimum brightness (level 1)
        if (g_ctx.has_saved_state) {
            return RestoreColorState();
        } else {
            // No saved state, set to level 1 with default warm white
            return Backlight_SetBrightnessLevel(1);
        }
    }

    // Get current brightness level and increase by 1
    uint8_t current_level = Backlight_GetBrightnessLevel();
    if (current_level >= 10) {
        LOG_INFO("Already at maximum brightness (level 10)");
        return BACKLIGHT_OK;
    }

    // Update current max brightness tracking
    uint8_t new_level = current_level + 1;
    g_ctx.current_max_brightness = brightness_levels[new_level];

    LOG_INFO("Brightness increased - Level: %d → %d (RGB: %d)",
             current_level, new_level, brightness_levels[new_level]);

    return Backlight_SetBrightnessLevel(new_level);
}

/**
 * @brief Decrease RGB brightness - saves state when turning off
 */
Backlight_Error_t Backlight_DecreaseBrightness(void)
{
    if (!g_ctx.initialized) {
        return BACKLIGHT_ERROR_INIT;
    }

    // Check if all LEDs are already off
    if (Backlight_IsAllOff()) {
        LOG_INFO("All LEDs already off");
        return BACKLIGHT_OK;
    }

    // Get current brightness level and decrease by 1
    uint8_t current_level = Backlight_GetBrightnessLevel();
    if (current_level == 0) {
        LOG_INFO("Already at minimum brightness (off)");
        return BACKLIGHT_OK;
    }

    uint8_t new_level = current_level - 1;

    // If going to level 0, save state and enter shutdown
    if (new_level == 0) {
        // Save current color state before shutdown
        if (!Backlight_IsAllOff()) {
            SaveColorState();
        }
        LOG_INFO("Brightness decreased to off - entering shutdown");
        return Backlight_EnterShutdown();
    }

    // Update current max brightness tracking
    g_ctx.current_max_brightness = brightness_levels[new_level];

    // Don't go below user minimum (if configured)
    if (brightness_levels[new_level] < g_ctx.user_min_limit) {
        // Find the lowest level that meets user minimum
        for (uint8_t i = 1; i <= 10; i++) {
            if (brightness_levels[i] >= g_ctx.user_min_limit) {
                new_level = i;
                g_ctx.current_max_brightness = brightness_levels[new_level];
                break;
            }
        }
    }

    LOG_INFO("Brightness decreased - Level: %d → %d (RGB: %d)",
             current_level, new_level, brightness_levels[new_level]);

    return Backlight_SetBrightnessLevel(new_level);
}

/**
 * @brief Get current brightness level based on highest RGB component (0-8)
 * @return uint8_t Brightness level (0 = off, 8 = max)
 */
uint8_t Backlight_GetBrightnessLevel(void)
{
    if (!g_ctx.initialized || Backlight_IsAllOff()) {
        return 0;
    }

    // Find the highest component across all colors
    uint8_t global_max = 0;
    for (uint8_t row = 0; row < BACKLIGHT_MATRIX_ROWS; row++) {
        for (uint8_t col = 0; col < BACKLIGHT_MATRIX_COLS; col++) {
            if (Backlight_HasLED(row, col)) {
                Backlight_RGB_t current = g_ctx.current_colors[row][col];
                uint8_t key_max = current.r;
                if (current.g > key_max) key_max = current.g;
                if (current.b > key_max) key_max = current.b;
                if (key_max > global_max) global_max = key_max;
            }
        }
    }

    // Find which level we're closest to
    for (uint8_t i = 10; i > 0; i--) {
        if (global_max >= brightness_levels[i] - 3) {  // 3-point tolerance
            return i;
        }
    }
    return 0;
}

/**
 * @brief Set specific brightness level (0-8)
 * @param level Brightness level (0 = off, 8 = max brightness)
 * @return Backlight_Error_t
 */
Backlight_Error_t Backlight_SetBrightnessLevel(uint8_t level)
{
    if (!g_ctx.initialized) {
        return BACKLIGHT_ERROR_INIT;
    }

    if (level > 10) level = 10;
    if (level == 0) return Backlight_EnterShutdown();

    uint8_t target_brightness = brightness_levels[level];

    if (Backlight_IsAllOff()) {
        Backlight_ExitShutdown();
        HAL_Delay(1);
        // Restore saved colors at target brightness, not default
        if (g_ctx.has_saved_state) {
            RestoreColorState(); // This should restore at saved ratios
            // Then scale to target level
        } else {
            return Backlight_SetAll(target_brightness, target_brightness/2, 0);
        }
    }

    // Store current ratios by normalizing to max brightness
    uint8_t current_global_max = 0;

    // Find current max
    for (uint8_t row = 0; row < BACKLIGHT_MATRIX_ROWS; row++) {
        for (uint8_t col = 0; col < BACKLIGHT_MATRIX_COLS; col++) {
            if (Backlight_HasLED(row, col)) {
                Backlight_RGB_t current = g_ctx.current_colors[row][col];
                uint8_t key_max = current.r;
                if (current.g > key_max) key_max = current.g;
                if (current.b > key_max) key_max = current.b;
                if (key_max > current_global_max) current_global_max = key_max;
            }
        }
    }

    if (current_global_max == 0) return BACKLIGHT_OK;

    // Scale all colors maintaining ratios
    for (uint8_t row = 0; row < BACKLIGHT_MATRIX_ROWS; row++) {
        for (uint8_t col = 0; col < BACKLIGHT_MATRIX_COLS; col++) {
            if (Backlight_HasLED(row, col)) {
                Backlight_RGB_t current = g_ctx.current_colors[row][col];

                if (current.r > 0 || current.g > 0 || current.b > 0) {
                    // Scale maintaining exact ratios
                    Backlight_RGB_t new_color;
                    new_color.r = (current.r * target_brightness + current_global_max/2) / current_global_max;
                    new_color.g = (current.g * target_brightness + current_global_max/2) / current_global_max;
                    new_color.b = (current.b * target_brightness + current_global_max/2) / current_global_max;

                    // Preserve non-zero components
                    if (current.r > 0 && new_color.r == 0) new_color.r = 1;
                    if (current.g > 0 && new_color.g == 0) new_color.g = 1;
                    if (current.b > 0 && new_color.b == 0) new_color.b = 1;

                    UpdateLEDHardware(row, col, new_color);
                    g_ctx.current_colors[row][col] = new_color;
                }
            }
        }
    }

    g_ctx.current_max_brightness = target_brightness;
    return BACKLIGHT_OK;
}

/* ========================================================================== */
/* Individual Key Control */
/* ========================================================================== */

Backlight_Error_t Backlight_SetKeyRGB(uint8_t row, uint8_t col, Backlight_RGB_t color)
{
    if (!g_ctx.initialized || row >= BACKLIGHT_MATRIX_ROWS || col >= BACKLIGHT_MATRIX_COLS) {
        return BACKLIGHT_ERROR_PARAM;
    }

    if (!Backlight_HasLED(row, col)) {
        return BACKLIGHT_ERROR_KEY_NOT_FOUND;
    }

    // Limit RGB values to maximum allowed
    Backlight_RGB_t limited_color = LimitRGBColor(color);

    g_ctx.current_colors[row][col] = limited_color;
    return UpdateLEDHardware(row, col, limited_color);
}

Backlight_Error_t Backlight_SetKey(uint8_t row, uint8_t col, uint8_t r, uint8_t g, uint8_t b)
{
    Backlight_RGB_t color = {
        .r = LimitRGBComponent(r),
        .g = LimitRGBComponent(g),
        .b = LimitRGBComponent(b)
    };
    return Backlight_SetKeyRGB(row, col, color);
}

Backlight_Error_t Backlight_SetKeyBrightness(uint8_t row, uint8_t col, uint8_t brightness)
{
    uint8_t limited_brightness = LimitRGBComponent(brightness);
    Backlight_RGB_t color = {limited_brightness, limited_brightness, limited_brightness};
    return Backlight_SetKeyRGB(row, col, color);
}

Backlight_Error_t Backlight_SetKeyOff(uint8_t row, uint8_t col)
{
    Backlight_RGB_t color = {0, 0, 0};
    return Backlight_SetKeyRGB(row, col, color);
}

/* ========================================================================== */
/* All Keys Control */
/* ========================================================================== */

Backlight_Error_t Backlight_SetAllRGB(Backlight_RGB_t color)
{
    if (!g_ctx.initialized) {
        return BACKLIGHT_ERROR_INIT;
    }

    // Limit RGB values to maximum allowed
    Backlight_RGB_t limited_color = LimitRGBColor(color);

    // Update all keys in memory
    for (uint8_t row = 0; row < BACKLIGHT_MATRIX_ROWS; row++) {
        for (uint8_t col = 0; col < BACKLIGHT_MATRIX_COLS; col++) {
            if (Backlight_HasLED(row, col)) {
                g_ctx.current_colors[row][col] = limited_color;
                Backlight_Error_t result = UpdateLEDHardware(row, col, limited_color);
                if (result != BACKLIGHT_OK) {
                    return result;
                }
            }
        }
    }

    return BACKLIGHT_OK;
}

Backlight_Error_t Backlight_SetAll(uint8_t r, uint8_t g, uint8_t b)
{
    Backlight_RGB_t color = {
        .r = LimitRGBComponent(r),
        .g = LimitRGBComponent(g),
        .b = LimitRGBComponent(b)
    };
    return Backlight_SetAllRGB(color);
}

Backlight_Error_t Backlight_SetAllBrightness(uint8_t brightness)
{
    uint8_t limited_brightness = LimitRGBComponent(brightness);
    Backlight_RGB_t color = {limited_brightness, limited_brightness, limited_brightness};
    return Backlight_SetAllRGB(color);
}

/* ========================================================================== */
/* Status and Diagnostics */
/* ========================================================================== */

Backlight_Error_t Backlight_GetDeviceStatus(bool *master_online, bool *slave_online)
{
    if (!master_online || !slave_online) {
        return BACKLIGHT_ERROR_PARAM;
    }

    *master_online = g_ctx.master_online;
    *slave_online = g_ctx.slave_online;

    return BACKLIGHT_OK;
}

Backlight_Error_t Backlight_TestHardware(void)
{
    if (!g_ctx.initialized) {
        return BACKLIGHT_ERROR_INIT;
    }

    // Test devices using IS31FL driver - devices are tested during AddDevice
    // Just check if they were successfully added and initialized
    return (g_ctx.master_online || g_ctx.slave_online) ? BACKLIGHT_OK : BACKLIGHT_ERROR_HARDWARE;
}

Backlight_Error_t Backlight_GetErrorStats(uint32_t *i2c_errors, uint32_t *timeout_errors)
{
    if (!i2c_errors || !timeout_errors) {
        return BACKLIGHT_ERROR_PARAM;
    }

    *i2c_errors = g_ctx.i2c_errors;
    *timeout_errors = g_ctx.timeout_errors;

    return BACKLIGHT_OK;
}

/* ========================================================================== */
/* Utility Functions (No-FPU Optimized) */
/* ========================================================================== */

bool Backlight_HasLED(uint8_t row, uint8_t col)
{
    return get_led_for_key(row, col) != NULL;
}

Backlight_RGB_t Backlight_HSVtoRGB(uint16_t h, uint8_t s, uint8_t v)
{
    Backlight_RGB_t rgb = {0, 0, 0};

    if (s == 0) {
        uint8_t value = LimitRGBComponent((v * 255) / 100);
        rgb.r = rgb.g = rgb.b = value;
        return rgb;
    }

    // Apply perceptual corrections BEFORE conversion
    uint8_t hue_index = ((h / 15) % 24);

    // 1. Hue shift correction (if you have this table)
    int16_t corrected_h = h + hue_shift_correction[hue_index];
    if (corrected_h < 0) corrected_h += 360;
    if (corrected_h >= 360) corrected_h -= 360;
    h = (uint16_t)corrected_h;

    // 2. Saturation boost for washed-out hues (using percentage)
    uint16_t corrected_s = (s * hue_saturation_boost[hue_index]) / 100;
    if (corrected_s > 100) corrected_s = 100;
    s = (uint8_t)corrected_s;

    // Standard HSV to RGB conversion
    h = h % 360;
    uint8_t region = h / 60;
    uint8_t remainder = (h - (region * 60)) * 6;

    // Integer-only calculations to avoid FPU
    uint8_t p = (v * (100 - s)) / 100;
    uint8_t q = (v * (100 - ((s * remainder) / 360))) / 100;
    uint8_t t = (v * (100 - ((s * (360 - remainder)) / 360))) / 100;

    uint8_t v_scaled = LimitRGBComponent((v * 255) / 100);
    uint8_t p_scaled = LimitRGBComponent((p * 255) / 100);
    uint8_t q_scaled = LimitRGBComponent((q * 255) / 100);
    uint8_t t_scaled = LimitRGBComponent((t * 255) / 100);

    switch (region) {
        case 0: rgb.r = v_scaled; rgb.g = t_scaled; rgb.b = p_scaled; break;
        case 1: rgb.r = q_scaled; rgb.g = v_scaled; rgb.b = p_scaled; break;
        case 2: rgb.r = p_scaled; rgb.g = v_scaled; rgb.b = t_scaled; break;
        case 3: rgb.r = p_scaled; rgb.g = q_scaled; rgb.b = v_scaled; break;
        case 4: rgb.r = t_scaled; rgb.g = p_scaled; rgb.b = v_scaled; break;
        default: rgb.r = v_scaled; rgb.g = p_scaled; rgb.b = q_scaled; break;
    }

    // 3. Apply brightness correction AFTER conversion (using 255 scale)
    uint8_t brightness_correction = hue_brightness_correction[hue_index];
    rgb.r = (rgb.r * brightness_correction) / 255;
    rgb.g = (rgb.g * brightness_correction) / 255;
    rgb.b = (rgb.b * brightness_correction) / 255;

    // 4. Final warmth adjustment for more natural feel
    if (rgb.r < 240) rgb.r += (255 - rgb.r) / 10; // +10% red boost
    if (rgb.b > 20) rgb.b -= rgb.b / 12; // -8% blue reduction

    return rgb;
}

Backlight_RGB_t Backlight_RGB(uint8_t r, uint8_t g, uint8_t b)
{
    Backlight_RGB_t color = {r, g, b};
    return color;
}

uint8_t Backlight_ApplyGamma(uint8_t linear_value)
{
    // Use lookup table for gamma correction (no FPU needed)
    uint8_t gamma_idx = linear_value >> 3;  // Divide by 8 (256/32)
    if (gamma_idx >= 32) gamma_idx = 31;
    return gamma_table[gamma_idx];
}

uint8_t Backlight_PercentToBrightness(uint8_t percentage)
{
    if (percentage > 100) percentage = 100;
    return (percentage * 255) / 100;
}

/* ========================================================================== */
/* Private Functions Implementation */
/* ========================================================================== */

static Backlight_Error_t InitDevices(void)
{
    LOG_INFO("Initializing LED devices...");

    // Add and initialize master device
    IS31FL_Error_t err = IS31FL_AddDevice(&g_ctx.led_ctx, BACKLIGHT_MASTER_ADDR);
	if (err == IS31FL_OK) {
		// Add retry logic for DeviceInit
		const uint8_t MAX_INIT_RETRIES = 3;
		for (uint8_t retry = 0; retry < MAX_INIT_RETRIES; retry++) {
			if (retry > 0) {
				LOG_WARNING("Master device init retry %d/%d", retry, MAX_INIT_RETRIES - 1);
				HAL_Delay(100);
			}

			err = IS31FL_DeviceInit(&g_ctx.led_ctx, 0);
			if (err == IS31FL_OK) {
				break;
			}
			LOG_ERROR("Master device init failed (attempt %d): %d", retry + 1, err);
		}

		g_ctx.master_online = (err == IS31FL_OK);
		if (g_ctx.master_online) {
			LOG_INFO("Master device (0x%02X) initialized", BACKLIGHT_MASTER_ADDR);
		} else {
			LOG_ERROR("Master device (0x%02X) init failed after %d retries", BACKLIGHT_MASTER_ADDR, MAX_INIT_RETRIES);
		}
	} else {
		LOG_ERROR("Master device (0x%02X) not found", BACKLIGHT_MASTER_ADDR);
	}

	HAL_Delay(100);

    // Add and initialize slave device
    err = IS31FL_AddDevice(&g_ctx.led_ctx, BACKLIGHT_SLAVE_ADDR);
    if (err == IS31FL_OK) {
        err = IS31FL_DeviceInit(&g_ctx.led_ctx, 1);
        g_ctx.slave_online = (err == IS31FL_OK);
        if (g_ctx.slave_online) {
            LOG_INFO("Slave device (0x%02X) initialized", BACKLIGHT_SLAVE_ADDR);
        } else {
            LOG_ERROR("Slave device (0x%02X) init failed", BACKLIGHT_SLAVE_ADDR);
        }
    } else {
        LOG_ERROR("Slave device (0x%02X) not found", BACKLIGHT_SLAVE_ADDR);
    }

    if (!g_ctx.master_online && !g_ctx.slave_online) {
        LOG_ERROR("No LED devices online");
        return BACKLIGHT_ERROR_HARDWARE;
    }

    LOG_INFO("LED devices ready: Master=%s, Slave=%s",
             g_ctx.master_online ? "OK" : "FAIL",
             g_ctx.slave_online ? "OK" : "FAIL");

    return BACKLIGHT_OK;
}

static Backlight_Error_t UpdateLEDHardware(uint8_t row, uint8_t col, Backlight_RGB_t color)
{
    const rgb_led_mapping_t *mapping = get_led_for_key(row, col);
    if (!mapping) {
        LOG_DEBUG("No LED mapping for key (%d,%d)", row, col);
        return BACKLIGHT_ERROR_KEY_NOT_FOUND;
    }

    uint8_t device_idx = mapping->ic;
    bool device_online = (device_idx == 0) ? g_ctx.master_online : g_ctx.slave_online;

    if (!device_online) {
        LOG_WARNING("Device %d offline for key (%d,%d)", device_idx, row, col);
        return BACKLIGHT_ERROR_HARDWARE;
    }

    // Update RGB channels with error logging
    IS31FL_Error_t err;
    err = IS31FL_SetLED(&g_ctx.led_ctx, device_idx, mapping->red_col, mapping->led_row, color.r);
    if (err != IS31FL_OK) {
        g_ctx.i2c_errors++;
        LOG_ERROR("Red LED failed: dev=%d, cs=%d, sw=%d", device_idx, mapping->red_col, mapping->led_row);
        HAL_Delay(10);
        return BACKLIGHT_ERROR_HARDWARE;
    }

    err = IS31FL_SetLED(&g_ctx.led_ctx, device_idx, mapping->green_col, mapping->led_row, color.g);
    if (err != IS31FL_OK) {
        g_ctx.i2c_errors++;
        LOG_ERROR("Green LED failed: dev=%d, cs=%d, sw=%d", device_idx, mapping->green_col, mapping->led_row);
        HAL_Delay(10);
        return BACKLIGHT_ERROR_HARDWARE;
    }

    err = IS31FL_SetLED(&g_ctx.led_ctx, device_idx, mapping->blue_col, mapping->led_row, color.b);
    if (err != IS31FL_OK) {
        g_ctx.i2c_errors++;
        LOG_ERROR("Blue LED failed: dev=%d, cs=%d, sw=%d", device_idx, mapping->blue_col, mapping->led_row);
        HAL_Delay(10);
        return BACKLIGHT_ERROR_HARDWARE;
    }

    return BACKLIGHT_OK;
}

/**
 * @brief Clear saved color state (use when setting new colors)
 */
void Backlight_ClearSavedState(void)
{
    g_ctx.has_saved_state = false;
    g_ctx.saved_global_max = 0;
    memset(g_ctx.saved_colors, 0, sizeof(g_ctx.saved_colors));
    LOG_INFO("Saved color state cleared");
}

/**
 * @brief Check if there's a saved color state
 */
bool Backlight_HasSavedState(void)
{
    return g_ctx.has_saved_state;
}

/**
 * @brief Enter software shutdown mode (SSD bit = 0)
 * All current sources are switched off, typical current: 1.3μA
 */
Backlight_Error_t Backlight_EnterShutdown(void)
{
    if (!g_ctx.initialized) {
        return BACKLIGHT_ERROR_INIT;
    }

    // Save current color state before shutdown
    SaveColorState();

    // Enable shutdown (SSD bit = 0) for both devices
    bool success = true;

    if (g_ctx.master_online) {
        if (IS31FL_Shutdown(&g_ctx.led_ctx, 0, true) != IS31FL_OK) {
            LOG_ERROR("Failed to shutdown master device");
            success = false;
        } else {
            LOG_DEBUG("Master device entered software shutdown");
        }
    }

    if (g_ctx.slave_online) {
        if (IS31FL_Shutdown(&g_ctx.led_ctx, 1, true) != IS31FL_OK) {
            LOG_ERROR("Failed to shutdown slave device");
            success = false;
        } else {
            LOG_DEBUG("Slave device entered software shutdown");
        }
    }

    // Clear current colors in memory (but keep saved state)
    for (uint8_t row = 0; row < BACKLIGHT_MATRIX_ROWS; row++) {
        for (uint8_t col = 0; col < BACKLIGHT_MATRIX_COLS; col++) {
            g_ctx.current_colors[row][col] = (Backlight_RGB_t){0, 0, 0};
        }
    }

    return success ? BACKLIGHT_OK : BACKLIGHT_ERROR_HARDWARE;
}

/**
 * @brief Exit software shutdown mode (SSD bit = 1)
 * All current sources are re-enabled for normal operation
 */
Backlight_Error_t Backlight_ExitShutdown(void)
{
    if (!g_ctx.initialized) {
        return BACKLIGHT_ERROR_INIT;
    }

    // Disable shutdown (SSD bit = 1) for both devices
    bool success = true;

    if (g_ctx.master_online) {
        if (IS31FL_Shutdown(&g_ctx.led_ctx, 0, false) != IS31FL_OK) {
            LOG_ERROR("Failed to wake up master device");
            success = false;
        } else {
            LOG_DEBUG("Master device exited software shutdown");
            HAL_Delay(1);
        }
    }

    if (g_ctx.slave_online) {
        if (IS31FL_Shutdown(&g_ctx.led_ctx, 1, false) != IS31FL_OK) {
            LOG_ERROR("Failed to wake up slave device");
            success = false;
        } else {
            LOG_DEBUG("Slave device exited software shutdown");
        }
    }
    return success ? BACKLIGHT_OK : BACKLIGHT_ERROR_HARDWARE;
}



Backlight_Error_t Backlight_GetMaxRGBValues(uint8_t *max_r, uint8_t *max_g, uint8_t *max_b)
{
    if (!g_ctx.initialized || !max_r || !max_g || !max_b) {
        return BACKLIGHT_ERROR_PARAM;
    }

    *max_r = *max_g = *max_b = 0;

    for (uint8_t row = 0; row < BACKLIGHT_MATRIX_ROWS; row++) {
        for (uint8_t col = 0; col < BACKLIGHT_MATRIX_COLS; col++) {
            if (Backlight_HasLED(row, col)) {
                Backlight_RGB_t current = g_ctx.current_colors[row][col];

                if (current.r > BACKLIGHT_RGB_MAX_VALUE) current.r = BACKLIGHT_RGB_MAX_VALUE;
				if (current.g > BACKLIGHT_RGB_MAX_VALUE) current.g = BACKLIGHT_RGB_MAX_VALUE;
				if (current.b > BACKLIGHT_RGB_MAX_VALUE) current.b = BACKLIGHT_RGB_MAX_VALUE;

                if (current.r > *max_r) *max_r = current.r;
                if (current.g > *max_g) *max_g = current.g;
                if (current.b > *max_b) *max_b = current.b;
            }
        }
    }

    return BACKLIGHT_OK;
}

uint8_t Backlight_GetCurrentMaxBrightness(void)
{
    return g_ctx.current_max_brightness;
}

void Backlight_UpdateCurrentMaxBrightness(void)
{
    // Calculate current max from all LED values
    uint8_t max_r, max_g, max_b;
    if (Backlight_GetMaxRGBValues(&max_r, &max_g, &max_b) == BACKLIGHT_OK) {
        uint8_t global_max = max_r;
        if (max_g > global_max) global_max = max_g;
        if (max_b > global_max) global_max = max_b;
        g_ctx.current_max_brightness = global_max;
    }
}

/* ========================================================================== */
/* User Brightness Limit Functions */
/* ========================================================================== */

Backlight_Error_t Backlight_SetUserMaxLimit(uint8_t max_limit)
{
    if (!g_ctx.initialized) {
        return BACKLIGHT_ERROR_INIT;
    }

    // Clamp to hardware maximum
    if (max_limit > BACKLIGHT_RGB_MAX_VALUE) {
        max_limit = BACKLIGHT_RGB_MAX_VALUE;
    }

    // Ensure max is greater than min
    if (max_limit <= g_ctx.user_min_limit) {
        return BACKLIGHT_ERROR_PARAM;
    }

    g_ctx.user_max_limit = max_limit;

    // Update current max if it exceeds new limit
    if (g_ctx.current_max_brightness > max_limit) {
        g_ctx.current_max_brightness = max_limit;
    }

    LOG_INFO("User max brightness limit set to %d", max_limit);
    return BACKLIGHT_OK;
}

Backlight_Error_t Backlight_SetUserMinLimit(uint8_t min_limit)
{
    if (!g_ctx.initialized) {
        return BACKLIGHT_ERROR_INIT;
    }

    // Clamp to hardware minimum
    if (min_limit < 1) {
        min_limit = 1;
    }

    // Ensure min is less than max
    if (min_limit >= g_ctx.user_max_limit) {
        return BACKLIGHT_ERROR_PARAM;
    }

    g_ctx.user_min_limit = min_limit;

    // Update current max if it's below new minimum
    if (g_ctx.current_max_brightness < min_limit) {
        g_ctx.current_max_brightness = min_limit;
    }

    LOG_INFO("User min brightness limit set to %d", min_limit);
    return BACKLIGHT_OK;
}

uint8_t Backlight_GetUserMaxLimit(void)
{
    return g_ctx.user_max_limit;
}

uint8_t Backlight_GetUserMinLimit(void)
{
    return g_ctx.user_min_limit;
}

void Backlight_ReadTemp(void){
    uint8_t temp_raw = 0;
    if (IS31FL_ReadTemperature(&g_ctx.led_ctx, 0, &temp_raw) == IS31FL_OK) {
        uint8_t temp_celsius = IS31FL_TempToCelsius(temp_raw);
        LOG_INFO("Master IC: %d°C (raw: 0x%02X)", temp_celsius, temp_raw);

        if (temp_raw > 0) {
            LOG_WARNING("Master IC in thermal protection mode!");
        }
    }

    temp_raw = 0;
    if (IS31FL_ReadTemperature(&g_ctx.led_ctx, 1, &temp_raw) == IS31FL_OK) {
        uint8_t temp_celsius = IS31FL_TempToCelsius(temp_raw);
        LOG_INFO("Slave IC: %d°C (raw: 0x%02X)", temp_celsius, temp_raw);

        if (temp_raw > 0) {
            LOG_WARNING("Slave IC in thermal protection mode!");
        }
    }
}

static void Backlight_ReadThermalConfig(uint8_t device_idx)
{
    const char* device_name = (device_idx == 0) ? "Master" : "Slave";

    uint8_t thermal_reg;
    IS31FL_ReadThermalConfig(&g_ctx.led_ctx,device_idx,&thermal_reg);

    // Extract TS (bits 3:2) and TROF (bits 1:0)
    uint8_t ts_bits = (thermal_reg >> 2) & 0x03;
    uint8_t trof_bits = thermal_reg & 0x03;

    // Convert to human readable values
    uint8_t temp_c = (ts_bits == 0) ? 140 : (ts_bits == 1) ? 120 : (ts_bits == 2) ? 100 : 90;
    uint8_t current_percent = (trof_bits == 0) ? 100 : (trof_bits == 1) ? 75 : (trof_bits == 2) ? 55 : 30;

    LOG_INFO("%s thermal config: 0x%02X -> %d°C, %d%% current",
             device_name, thermal_reg, temp_c, current_percent);
}
