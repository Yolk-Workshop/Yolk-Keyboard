/**
 * @file backlight.h
 * @brief Simplified Keyboard RGB Backlight API
 * @version 1.0
 *
 * Simplified API for IS31FL3743A master-slave keyboard backlight control
 * - Master (IC1): 18×11 matrix covering main keyboard area
 * - Slave (IC2): 12×4 matrix covering function and modifier keys
 * - Hardware synchronized via SYNC line
 * - RGB-native interface using keyboard row/column coordinates
 * - Optimized for STM32L072VZ (Cortex-M0+, no FPU)
 */

#ifndef BACKLIGHT_CORE_BACKLIGHT_H
#define BACKLIGHT_CORE_BACKLIGHT_H

#include <stdint.h>
#include <stdbool.h>
#include "is31fl_driver.h"

/* ========================================================================== */
/* Hardware Configuration */
/* ========================================================================== */

// Device definitions based on your hardware
#define BACKLIGHT_MASTER_DEVICE_IDX     0       // U2 - Master device index
#define BACKLIGHT_SLAVE_DEVICE_IDX      1       // U4 - Slave device index

#define BACKLIGHT_MASTER_ADDR           0x20    // U1 I2C address
#define BACKLIGHT_SLAVE_ADDR            0x2C    // U2 I2C address

// Matrix dimensions per device
#define BACKLIGHT_MASTER_CS_COUNT       18      // Master CS lines
#define BACKLIGHT_MASTER_SW_COUNT       11      // Master SW lines
#define BACKLIGHT_MASTER_LED_COUNT      198     // 18×11

#define BACKLIGHT_SLAVE_CS_COUNT        12      // Slave CS lines
#define BACKLIGHT_SLAVE_SW_COUNT        4       // Slave SW lines
#define BACKLIGHT_SLAVE_LED_COUNT      (48 - 2)      // 12×4

#define BACKLIGHT_TOTAL_LEDS           (BACKLIGHT_MASTER_LED_COUNT + BACKLIGHT_SLAVE_LED_COUNT)

// Keyboard matrix dimensions
#define BACKLIGHT_MATRIX_ROWS           6       // Keyboard rows (0-5)
#define BACKLIGHT_MATRIX_COLS           14      // Keyboard columns (0-13)
#define BACKLIGHT_TOTAL_KEYS            82      // Total mappable keys

/* ========================================================================== */
/* STM32L072VZ Optimizations (No FPU) */
/* ========================================================================== */

#define BACKLIGHT_FADE_UPDATE_INTERVAL_MS   16      // ~60 FPS fade updates
#define BACKLIGHT_MAX_FADE_STEPS            32      // Reduced for no-FPU performance
#define BACKLIGHT_I2C_TIMEOUT_MS            50      // I2C operation timeout

extern const uint8_t brightness_levels[11];

/* ========================================================================== */
/* Effects Interface */
/* ========================================================================== */
typedef struct{

}Backlight_Effects_t;
/* ========================================================================== */
/* Error Codes */
/* ========================================================================== */

typedef enum {
    BACKLIGHT_OK = 0,                   /**< Operation successful */
    BACKLIGHT_ERROR_INIT,               /**< Initialization failed */
    BACKLIGHT_ERROR_PARAM,              /**< Invalid parameter */
    BACKLIGHT_ERROR_HARDWARE,           /**< Hardware communication error */
    BACKLIGHT_ERROR_KEY_NOT_FOUND,      /**< Key has no LED mapping */
    BACKLIGHT_ERROR_BUSY,               /**< System busy (fade in progress) */
    BACKLIGHT_ERROR_TIMEOUT             /**< Operation timeout */
} Backlight_Error_t;

/* ========================================================================== */
/* RGB Color Structure */
/* ========================================================================== */

typedef struct {
    uint8_t r;  /**< Red component (0-255) */
    uint8_t g;  /**< Green component (0-255) */
    uint8_t b;  /**< Blue component (0-255) */
} Backlight_RGB_t;

/* ========================================================================== */
/* Keyboard Zone Definitions */
/* ========================================================================== */

typedef enum {
    BACKLIGHT_ZONE_ALL = 0,             /**< All keys */
    BACKLIGHT_ZONE_FUNCTION_KEYS,       /**< F1-F12, ESC, DEL */
    BACKLIGHT_ZONE_NUMBER_ROW,          /**< 1-0, -, =, Backspace */
    BACKLIGHT_ZONE_QWERTY_ROW,          /**< Tab, Q-P, [, ], Enter */
    BACKLIGHT_ZONE_ASDF_ROW,            /**< Caps, A-L, ;, ', # */
    BACKLIGHT_ZONE_ZXCV_ROW,            /**< Shift, Z-M, ,, . */
    BACKLIGHT_ZONE_BOTTOM_ROW,          /**< Ctrl, Alt, Space, etc. */
    BACKLIGHT_ZONE_WASD,                /**< WASD gaming keys */
    BACKLIGHT_ZONE_ARROW_KEYS,          /**< Arrow key cluster */
    BACKLIGHT_ZONE_MODIFIERS,           /**< All modifier keys */
    BACKLIGHT_ZONE_ALPHAS,              /**< A-Z keys only */
    BACKLIGHT_ZONE_NUMBERS,             /**< 1-0 keys only */
    BACKLIGHT_ZONE_MAIN_TYPING,         /**< Main typing area: 1--, Q-P, A-L, Z-M */
    BACKLIGHT_ZONE_COUNT
} Backlight_Zone_t;

/* ========================================================================== */
/* Fade Configuration */
/* ========================================================================== */

typedef struct {
    Backlight_RGB_t target_color;       /**< Target RGB color */
    uint16_t duration_ms;               /**< Fade duration in milliseconds */
    uint8_t steps;                      /**< Number of fade steps (default: 32) */
} Backlight_Fade_Config_t;

/* ========================================================================== */
/* Core API */
/* ========================================================================== */

/**
 * @brief Initialize keyboard backlight system
 * @return BACKLIGHT_OK on success
 */
Backlight_Error_t Backlight_Init(void);

/**
 * @brief Deinitialize backlight system
 * @return BACKLIGHT_OK on success
 */
Backlight_Error_t Backlight_DeInit(void);


/**
 * @brief Emergency stop - turn off all LEDs immediately
 * @return BACKLIGHT_OK on success
 */
Backlight_Error_t Backlight_EmergencyOff(void);

/* ========================================================================== */
/* Global Brightness Control */
/* ========================================================================== */

/**
 * @brief Set global brightness for all LEDs
 * @param brightness Brightness level (0-255, 0=off)
 * @return BACKLIGHT_OK on success
 */
Backlight_Error_t Backlight_SetCurrentLimit(uint8_t brightness);

/**
 * @brief Get current global brightness
 * @param brightness Pointer to store current brightness
 * @return BACKLIGHT_OK on success
 */
Backlight_Error_t Backlight_GetBrightness(uint8_t *brightness);

/**
 * @brief Turn all LEDs off (brightness = 0)
 * @return BACKLIGHT_OK on success
 */
Backlight_Error_t Backlight_Off(void);

/**
 * @brief Turn all LEDs on with previous brightness
 * @return BACKLIGHT_OK on success
 */
Backlight_Error_t Backlight_On(void);

/* ========================================================================== */
/* Individual Key Control */
/* ========================================================================== */

/**
 * @brief Set RGB color for individual key
 * @param row Keyboard row (0-5)
 * @param col Keyboard column (0-13)
 * @param color RGB color
 * @return BACKLIGHT_OK on success, BACKLIGHT_ERROR_KEY_NOT_FOUND if no LED
 */
Backlight_Error_t Backlight_SetKeyRGB(uint8_t row, uint8_t col, Backlight_RGB_t color);

/**
 * @brief Set RGB color for individual key (separate components)
 * @param row Keyboard row (0-5)
 * @param col Keyboard column (0-13)
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 * @return BACKLIGHT_OK on success, BACKLIGHT_ERROR_KEY_NOT_FOUND if no LED
 */
Backlight_Error_t Backlight_SetKey(uint8_t row, uint8_t col, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Set white brightness for individual key
 * @param row Keyboard row (0-5)
 * @param col Keyboard column (0-13)
 * @param brightness White brightness (0-255)
 * @return BACKLIGHT_OK on success, BACKLIGHT_ERROR_KEY_NOT_FOUND if no LED
 */
Backlight_Error_t Backlight_SetKeyBrightness(uint8_t row, uint8_t col, uint8_t brightness);

/**
 * @brief Turn off individual key
 * @param row Keyboard row (0-5)
 * @param col Keyboard column (0-13)
 * @return BACKLIGHT_OK on success
 */
Backlight_Error_t Backlight_SetKeyOff(uint8_t row, uint8_t col);

/**
 * @brief Increase RGB brightness of all keys by step amount
 * @return Backlight_Error_t
 */
Backlight_Error_t Backlight_DecreaseBrightness(void);

/**
 * @brief Decrease RGB brightness of all keys by step amount
 * @return Backlight_Error_t
 */
Backlight_Error_t Backlight_IncreaseBrightness(void);

/**
 * @brief Get current brightness level (0-8 based on max RGB value)
 * @return uint8_t Brightness level (0 = off, 8 = max)
 */
uint8_t Backlight_GetBrightnessLevel(void);

/**
 * @brief Set specific brightness level (0-8)
 * @param level Brightness level (0 = off, 8 = max brightness)
 * @return Backlight_Error_t
 */
Backlight_Error_t Backlight_SetBrightnessLevel(uint8_t level);

/**
 * @brief Check if there's a saved color state
 */
bool Backlight_HasSavedState(void);

/**
 * @brief Clear saved color state (use when setting new colors)
 */
void Backlight_ClearSavedState(void);

/**
 * @brief Exit software shutdown mode (SSD bit = 1)
 * All current sources are re-enabled for normal operation
 */
Backlight_Error_t Backlight_ExitShutdown(void);

/**
 * @brief Enter software shutdown mode (SSD bit = 0)
 * All current sources are switched off, typical current: 1.3μA
 */
Backlight_Error_t Backlight_EnterShutdown(void);

/* ========================================================================== */
/* All Keys Control */
/* ========================================================================== */

/**
 * @brief Set all keys to same RGB color
 * @param color RGB color
 * @return BACKLIGHT_OK on success
 */
Backlight_Error_t Backlight_SetAllRGB(Backlight_RGB_t color);

/**
 * @brief Set all keys to same RGB color (separate components)
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 * @return BACKLIGHT_OK on success
 */
Backlight_Error_t Backlight_SetAll(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Set all keys to white brightness
 * @param brightness White brightness (0-255)
 * @return BACKLIGHT_OK on success
 */
Backlight_Error_t Backlight_SetAllBrightness(uint8_t brightness);

/* ========================================================================== */
/* Zone Control */
/* ========================================================================== */

/**
 * @brief Set zone to RGB color
 * @param zone Keyboard zone
 * @param color RGB color
 * @return BACKLIGHT_OK on success
 */
Backlight_Error_t Backlight_SetZoneRGB(Backlight_Zone_t zone, Backlight_RGB_t color);

/**
 * @brief Set zone to RGB color with intensity scaling
 * @param zone Keyboard zone
 * @param color RGB color
 * @param intensity Intensity scale (0-255, 255=full brightness)
 * @return BACKLIGHT_OK on success
 */
Backlight_Error_t Backlight_SetZoneRGBIntensity(Backlight_Zone_t zone, Backlight_RGB_t color, uint8_t intensity);

/**
 * @brief Set zone to RGB color (separate components)
 * @param zone Keyboard zone
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 * @return BACKLIGHT_OK on success
 */
Backlight_Error_t Backlight_SetZone(Backlight_Zone_t zone, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Set zone to RGB color (separate components) with intensity scaling
 * @param zone Keyboard zone
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 * @param intensity Intensity scale (0-255, 255=full brightness)
 * @return BACKLIGHT_OK on success
 */
Backlight_Error_t Backlight_SetZoneIntensity(Backlight_Zone_t zone, uint8_t r, uint8_t g, uint8_t b, uint8_t intensity);

/**
 * @brief Set zone to RGB color (separate components)
 * @param zone Keyboard zone
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 * @return BACKLIGHT_OK on success
 */
Backlight_Error_t Backlight_SetZone(Backlight_Zone_t zone, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Set zone to white brightness
 * @param zone Keyboard zone
 * @param brightness White brightness (0-255)
 * @return BACKLIGHT_OK on success
 */
Backlight_Error_t Backlight_SetZoneBrightness(Backlight_Zone_t zone, uint8_t brightness);

/**
 * @brief Turn off zone
 * @param zone Keyboard zone
 * @return BACKLIGHT_OK on success
 */
Backlight_Error_t Backlight_SetZoneOff(Backlight_Zone_t zone);

/* ========================================================================== */
/* Pattern Loading */
/* ========================================================================== */

/**
 * @brief Load RGB pattern from array
 * @param pattern Array of Backlight_RGB_t colors, indexed by (row * BACKLIGHT_MATRIX_COLS + col)
 * @param size Size of pattern array (max BACKLIGHT_MATRIX_ROWS * BACKLIGHT_MATRIX_COLS)
 * @return BACKLIGHT_OK on success
 */
Backlight_Error_t Backlight_LoadPattern(const Backlight_RGB_t *pattern, uint16_t size);

/**
 * @brief Save current colors to pattern array
 * @param pattern Array to store current colors
 * @param size Size of pattern array
 * @return BACKLIGHT_OK on success
 */
Backlight_Error_t Backlight_SavePattern(Backlight_RGB_t *pattern, uint16_t size);

/* ========================================================================== */
/* Status and Diagnostics */
/* ========================================================================== */

/**
 * @brief Check if both devices are online
 * @param master_online Pointer to store master status
 * @param slave_online Pointer to store slave status
 * @return BACKLIGHT_OK on success
 */
Backlight_Error_t Backlight_GetDeviceStatus(bool *master_online, bool *slave_online);

/**
 * @brief Test hardware connectivity
 * @return BACKLIGHT_OK if both devices respond
 */
Backlight_Error_t Backlight_TestHardware(void);

/**
 * @brief Get error statistics
 * @param i2c_errors Pointer to store I2C error count
 * @param timeout_errors Pointer to store timeout error count
 * @return BACKLIGHT_OK on success
 */
Backlight_Error_t Backlight_GetErrorStats(uint32_t *i2c_errors, uint32_t *timeout_errors);

/* ========================================================================== */
/* Utility Functions (No-FPU Optimized) */
/* ========================================================================== */

/**
 * @brief Check if key position has LED mapping
 * @param row Keyboard row (0-5)
 * @param col Keyboard column (0-13)
 * @return True if key has LED
 */
bool Backlight_HasLED(uint8_t row, uint8_t col);

/**
 * @brief Convert HSV to RGB (integer-only implementation)
 * @param h Hue (0-359)
 * @param s Saturation (0-100)
 * @param v Value/brightness (0-100)
 * @return RGB color
 */
Backlight_RGB_t Backlight_HSVtoRGB(uint16_t h, uint8_t s, uint8_t v);

/**
 * @brief Create RGB color from components
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 * @return RGB color
 */
Backlight_RGB_t Backlight_RGB(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Get number of keys in zone
 * @param zone Keyboard zone
 * @return Number of keys in zone
 */
uint8_t Backlight_GetZoneKeyCount(Backlight_Zone_t zone);

/**
 * @brief Apply gamma correction using lookup table (no FPU)
 * @param linear_value Linear brightness value (0-255)
 * @return Gamma corrected value
 */
uint8_t Backlight_ApplyGamma(uint8_t linear_value);

/**
 * @brief Convert percentage to brightness value
 * @param percentage Percentage (0-100)
 * @return Brightness value (0-255)
 */
uint8_t Backlight_PercentToBrightness(uint8_t percentage);

Backlight_Error_t Backlight_GetMaxRGBValues(uint8_t *max_r, uint8_t *max_g, uint8_t *max_b);

bool Backlight_IsAllOff(void);

uint8_t Backlight_GetCurrentMaxBrightness(void);

void Backlight_UpdateCurrentMaxBrightness(void);
uint8_t Backlight_GetUserMinLimit(void);
uint8_t Backlight_GetUserMaxLimit(void);
Backlight_Error_t Backlight_SetUserMinLimit(uint8_t min_limit);
Backlight_Error_t Backlight_SetUserMaxLimit(uint8_t max_limit);

/* ========================================================================== */
/* Convenience Macros */
/* ========================================================================== */

// Quick brightness levels
#define BACKLIGHT_BRIGHTNESS_OFF       0
#define BACKLIGHT_BRIGHTNESS_LOW       64
#define BACKLIGHT_BRIGHTNESS_MEDIUM    128
#define BACKLIGHT_BRIGHTNESS_HIGH      192
#define BACKLIGHT_BRIGHTNESS_MAX       255

// Common key shortcuts (using your mapping)
#define BACKLIGHT_KEY_ESC              0, 0
#define BACKLIGHT_KEY_SPACE            5, 4
#define BACKLIGHT_KEY_ENTER            2, 13
#define BACKLIGHT_KEY_WASD_W           2, 2
#define BACKLIGHT_KEY_WASD_A           3, 1
#define BACKLIGHT_KEY_WASD_S           3, 2
#define BACKLIGHT_KEY_WASD_D           3, 3

// Zone shortcuts
#define BACKLIGHT_SET_WASD(color)      Backlight_SetZoneRGB(BACKLIGHT_ZONE_WASD, color)
#define BACKLIGHT_SET_ARROWS(color)    Backlight_SetZoneRGB(BACKLIGHT_ZONE_ARROW_KEYS, color)
#define BACKLIGHT_SET_FUNCTION(color)  Backlight_SetZoneRGB(BACKLIGHT_ZONE_FUNCTION_KEYS, color)
#define BACKLIGHT_SET_MAIN_TYPING(color) Backlight_SetZoneRGB(BACKLIGHT_ZONE_MAIN_TYPING, color)

// Intensity shortcuts
#define BACKLIGHT_SET_MAIN_BRIGHT()    Backlight_SetZoneIntensity(BACKLIGHT_ZONE_MAIN_TYPING, 255, 255, 0, 255)
#define BACKLIGHT_SET_MAIN_DIM()       Backlight_SetZoneIntensity(BACKLIGHT_ZONE_MAIN_TYPING, 255, 255, 0, 128)
#define BACKLIGHT_SET_OTHERS_DIM()     Backlight_SetZoneIntensity(BACKLIGHT_ZONE_ALL, 255, 255, 0, 64); \
                                       Backlight_SetZoneIntensity(BACKLIGHT_ZONE_MAIN_TYPING, 255, 255, 0, 255)

// Animation shortcuts
#define BACKLIGHT_FADE_OUT(duration)   Backlight_FadeBrightness(0, duration)
#define BACKLIGHT_FADE_IN(duration)    Backlight_FadeBrightness(255, duration)

#endif /* BACKLIGHT_H */
