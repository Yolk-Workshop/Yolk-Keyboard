/*
* effects.h
*
* Created on: Jun 10, 2025
* Author: bettysidepiece
*/

#ifndef BACKLIGHT_CORE_EXTENSION_EFFECTS_H_
#define BACKLIGHT_CORE_EXTENSION_EFFECTS_H_

#include <stdint.h>
#include <stdbool.h>
#include "backlight.h"

/* ========================================================================== */
/* Effect Configuration */
/* ========================================================================== */

#define EFFECT_UPDATE_INTERVAL_MS       25      // 40 FPS - smoother fades
#define EFFECT_BREATHING_INTERVAL_MS    125     // 8 FPS - slower breathing
#define EFFECT_MAX_DURATION_MS          5000    // Allow longer effects
#define EFFECT_MIN_BRIGHTNESS           26       // Minimum effect brightness
#define EFFECT_MAX_BRIGHTNESS           90      // Maximum effect brightness

/* ========================================================================== */
/* Effect Types */
/* ========================================================================== */
typedef enum {
   EFFECT_NONE = 0,
   EFFECT_FADE_IN,           // Power on / wake transitions
   EFFECT_FADE_OUT,          // Power off / sleep transitions
   EFFECT_BREATHING,         // Continuous breathing effect
   EFFECT_COLOR_TRANSITION,   // Smooth color changes
   EFFECT_ZONE_BREATHING,
   EFFECT_ZONE_STATIC
} effect_type_t;

typedef enum {
   EFFECT_ERROR_NONE = 0,
   EFFECT_ERROR_INVALID_PARAM,
   EFFECT_ERROR_HARDWARE_FAIL,
   EFFECT_ERROR_TIMEOUT,
   EFFECT_ERROR_MEMORY
} effect_error_t;

typedef enum {
   EFFECT_TRIGGER_POWER_ON = 0,
   EFFECT_TRIGGER_POWER_OFF,
   EFFECT_TRIGGER_WAKE,
   EFFECT_TRIGGER_SLEEP,
   EFFECT_TRIGGER_IDLE,
   EFFECT_TRIGGER_LOW_BATTERY,
   EFFECT_TRIGGER_USER
} effect_trigger_t;

/* ========================================================================== */
/* Effect State Structure */
/* ========================================================================== */
typedef enum {
    EFFECT_MODE_STATIC = 0,          // Solid color, no animation
    EFFECT_MODE_BREATHING,           // Continuous breathing
    EFFECT_MODE_TRANSITION_BREATHING // Fade in → Color transition → Breathing
} effect_mode_t;

typedef enum {
    LMP_BEHAVIOR_OFF = 0,            // Turn backlight completely off
    LMP_BEHAVIOR_DIM,                // Dim to low brightness, keep runtime effect
} lmp_behavior_t;

typedef struct {
    // Global effect enable/disable
    bool effects_enabled;           // Master on/off

    // Runtime effect (what runs after startup)
    effect_mode_t runtime_mode;     // STATIC, BREATHING, TRANSITION_BREATHING
    uint8_t startup_color_index;    // For fade in (index into breathing_colors[])
    uint8_t runtime_color_index;    // Final runtime color (index into breathing_colors[])

    // User-configurable LPM behaviors
    lmp_behavior_t usb_lmp_behavior;    // What to do in USB LPM
    lmp_behavior_t ble_lmp_behavior;    // What to do in BLE LPM
    uint8_t lmp_backlight_mode;
    uint8_t lmp_dim_brightness;
    volatile bool lpm_backlight_off;
    volatile bool lpm_backlight_restore;

    // Effect timing configuration
    uint16_t startup_fade_duration_ms;  // Fade in duration
    uint16_t transition_duration_ms;    // Color transition duration
    uint16_t breathing_cycle_duration_ms; // Breathing cycle time
    uint8_t breathing_min_brightness;   // Breathing minimum
    uint8_t breathing_max_brightness;   // Breathing maximum

} user_effects_config_t;

typedef struct {
   effect_type_t type;
   uint32_t start_time;
   uint16_t duration_ms;
   uint8_t current_step;
   uint8_t total_steps;

   // Brightness control
   uint8_t start_brightness;
   uint8_t target_brightness;
   uint8_t min_brightness;
   uint8_t max_brightness;

   bool is_chained_effect;      // True if part of a chain
   effect_mode_t target_mode;   // Target mode for chaining
   uint8_t chain_step;          // 0=fade_in, 1=transition, 2=breathing
   bool is_color_cycling;
   uint8_t cycle_color_index;

   // Color control
   Backlight_RGB_t start_color;
   Backlight_RGB_t target_color;
   Backlight_RGB_t current_color;

   // State flags
   bool backlight_ready;
   bool active;
   bool breathing_direction_up;
   bool hardware_ready;

   // Zone-aware effects
   bool zone_effect_active;
   uint8_t active_zone_slot;
   zone_storage_t zone_backup; // Backup of original zone colors
   bool zone_backup_valid;

   // Error handling
   uint8_t error_count;
   uint8_t retry_count;

   user_effects_config_t config;

   // Completion callback
   void (*on_complete)(effect_error_t error);

} effect_state_t;

/* ========================================================================== */
/* Public API */
/* ========================================================================== */

extern effect_state_t g_effect;

/**
* @brief Initialize effects system
* @return effect_error_t Error code
*/
effect_error_t Effects_Init(void);

/**
* @brief Process active effects - call from main loop
* @return effect_error_t Error code
*/
effect_error_t Effects_Process(void);

/**
* @brief Stop any active effect
*/
void Effects_Stop(void);

/**
* @brief Check if any effect is currently active
* @return true if effect is running
*/
bool Effects_IsActive(void);

/**
* @brief Get current effect type
* @return effect_type_t Current effect type
*/
effect_type_t Effects_GetCurrentType(void);

/* ========================================================================== */
/* Fade Effects */
/* ========================================================================== */
/**
* @brief Start complete startup sequence (Fade In → Runtime Effect)
* @param callback Optional completion callback
* @return effect_error_t Error code
*/
effect_error_t Effects_StartupSequence(void (*callback)(effect_error_t));

/**
* @brief Start runtime effect based on user configuration
* @return effect_error_t Error code
*/
effect_error_t Effects_StartRuntimeEffect(void);

/**
* @brief Handle LPM entry based on connection type and user config
* @param is_usb_connected True if USB connected, false for BLE
* @return effect_error_t Error code
*/
effect_error_t Effects_EnterLPM(bool is_usb_connected);

/**
* @brief Handle LPM exit based on connection type and user config
* @param is_usb_connected True if USB connected, false for BLE
* @return effect_error_t Error code
*/
effect_error_t Effects_ExitLPM(bool is_usb_connected);

/**
* @brief Manual effect enable via Fn+Spacebar (works even when effects_enabled=false)
* @return effect_error_t Error code
*/
effect_error_t Effects_ManualEnable(void);

/**
* @brief Update user configuration at runtime
* @param config New user configuration
* @return effect_error_t Error code
*/
effect_error_t Effects_UpdateConfig(user_effects_config_t *config);
/**
* @brief Start fade in effect (power on, wake up)
* @param trigger What triggered this fade
* @param duration_ms Fade duration in milliseconds
* @param target_brightness Target brightness (0-80)
* @param callback Optional completion callback
* @return effect_error_t Error code
*/
effect_error_t Effects_StartFadeIn(effect_trigger_t trigger,
                                  uint16_t duration_ms,
                                  uint8_t target_brightness,
                                  void (*callback)(effect_error_t));

/**
* @brief Start fade out effect (power off, sleep)
* @param trigger What triggered this fade
* @param duration_ms Fade duration in milliseconds
* @param callback Optional completion callback
* @return effect_error_t Error code
*/
effect_error_t Effects_StartFadeOut(effect_trigger_t trigger,
                                   uint16_t duration_ms,
                                   void (*callback)(effect_error_t));

/* ========================================================================== */
/* Breathing Effects */
/* ========================================================================== */

/**
* @brief Start breathing effect
* @param min_brightness Minimum brightness (0-80)
* @param max_brightness Maximum brightness (0-80)
* @param cycle_duration_ms Full cycle duration in milliseconds
* @param color Base color for breathing
* @return effect_error_t Error code
*/
effect_error_t Effects_StartBreathing(uint8_t min_brightness,
                                     uint8_t max_brightness,
                                     uint16_t cycle_duration_ms,
                                     Backlight_RGB_t color);

/**
* @brief Start breathing with current colors
* @param min_brightness Minimum brightness (0-80)
* @param max_brightness Maximum brightness (0-80)
* @param cycle_duration_ms Full cycle duration in milliseconds
* @return effect_error_t Error code
*/
effect_error_t Effects_StartBreathingCurrent(uint8_t min_brightness,
                                            uint8_t max_brightness,
                                            uint16_t cycle_duration_ms);

/* ========================================================================== */
/* Color Transition Effects */
/* ========================================================================== */

/**
* @brief Start smooth color transition
* @param target_color Target color to transition to
* @param duration_ms Transition duration in milliseconds
* @param callback Optional completion callback
* @return effect_error_t Error code
*/
effect_error_t Effects_StartColorTransition(Backlight_RGB_t target_color,
                                           uint16_t duration_ms,
                                           void (*callback)(effect_error_t));

effect_error_t Effects_StartColorCyclingBreathing(uint8_t min_brightness,
                                                  uint8_t max_brightness,
                                                  uint16_t cycle_duration_ms);

/* ========================================================================== */
/* Convenience Functions */
/* ========================================================================== */

effect_error_t Effects_IncreaseBrightness(void);
effect_error_t Effects_DecreaseBrightness(void);
/**
* @brief Quick power-on fade with defaults
* @param callback Optional completion callback
* @return effect_error_t Error code
*/
effect_error_t Effects_PowerOn(void (*callback)(effect_error_t));

/**
* @brief Quick power-off fade with defaults
* @param callback Optional completion callback
* @return effect_error_t Error code
*/
effect_error_t Effects_PowerOff(void (*callback)(effect_error_t));

/**
* @brief Quick wake-up fade with defaults
* @return effect_error_t Error code
*/
effect_error_t Effects_WakeUp(void);

/**
* @brief Quick sleep fade with defaults
* @param callback Optional completion callback
* @return effect_error_t Error code
*/
effect_error_t Effects_Sleep(void (*callback)(effect_error_t));

/**
* @brief Start idle breathing effect
* @return effect_error_t Error code
*/
effect_error_t Effects_IdleBreathing(void);

/**
* @brief Start low battery breathing effect (red)
* @return effect_error_t Error code
*/
effect_error_t Effects_LowBatteryBreathing(void);

/* ========================================================================== */
/* Status and Diagnostics */
/* ========================================================================== */

/**
* @brief Get effect progress (0-100%)
* @return uint8_t Progress percentage
*/
uint8_t Effects_GetProgress(void);

/**
* @brief Get current effect error count
* @return uint8_t Number of errors encountered
*/
uint8_t Effects_GetErrorCount(void);

/**
* @brief Reset error counters
*/
void Effects_ResetErrors(void);

/**
* @brief Get time remaining for current effect (ms)
* @return uint32_t Time remaining in milliseconds
*/
uint32_t Effects_GetTimeRemaining(void);

/* ========================================================================== */
/* BLE Status Effects */
/* ========================================================================== */

/**
 * @brief Start BLE pairing mode effect (pulsing blue/cyan)
 * @return effect_error_t Error code
 */
effect_error_t Effects_BLE_PairingActive(void);

/**
 * @brief Start BLE pairing success effect (green flash)
 * @return effect_error_t Error code
 */
effect_error_t Effects_BLE_PairingSuccess(void);

/**
 * @brief Start BLE pairing failed effect (red flash)
 * @return effect_error_t Error code
 */
effect_error_t Effects_BLE_PairingFailed(void);

/**
 * @brief Start BLE device switch effect (cyan sweep)
 * @return effect_error_t Error code
 */
effect_error_t Effects_BLE_DeviceSwitch(void);

/**
 * @brief Start BLE connection status effect
 * @param connected true if connected, false if disconnected
 * @return effect_error_t Error code
 */
effect_error_t Effects_BLE_ConnectionStatus(bool connected);

/**
 * @brief Stop any active BLE effect
 * @return effect_error_t Error code
 */
effect_error_t Effects_BLE_Stop(void);

#endif /* BACKLIGHT_CORE_EFFECTS_H_ */
