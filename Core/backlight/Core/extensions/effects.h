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
#include "zones.h"
#include "backlight.h"
#include "system_config.h"

/* ========================================================================== */
/* Effect Configuration */
/* ========================================================================== */

#define EFFECT_UPDATE_INTERVAL_MS       25      // 40 FPS - smoother fades
#define EFFECT_BREATHING_INTERVAL_MS    125     // 8 FPS - slower breathing
#define EFFECT_MAX_DURATION_MS          5000    // Allow longer effects
#define EFFECT_MIN_BRIGHTNESS           77       // Minimum effect brightness
#define EFFECT_MAX_BRIGHTNESS           255      // Maximum effect brightness

/* ========================================================================== */
/* Effect Types */
/* ========================================================================== */
typedef enum {
   EFFECT_NONE = 0,
   EFFECT_FADE_IN,           // Power on / wake transitions
   EFFECT_FADE_OUT,          // Power off / sleep transitions
   EFFECT_BREATHING,         // Continuous breathing effect
   EFFECT_COLOR_TRANSITION   // Smooth color changes
} effect_type_t;

typedef enum {
   EFFECT_ERROR_NONE = 0,
   EFFECT_ERROR_INVALID_PARAM,
   EFFECT_ERROR_HARDWARE_FAIL,
   EFFECT_ERROR_TIMEOUT,
   EFFECT_ERROR_MEMORY,
   EFFECT_COMPLETE,
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
	LMP_BEHAVIOR_SYSTEM_OFF			//System Backlight off runtime and lpm
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

   // Error handling
   uint8_t error_count;
   uint8_t retry_count;

   user_effects_config_t config;

   //Zones
   bool zone_active;
   uint8_t active_zone_slot;

   // Completion callback
   void (*on_complete)(effect_error_t error);

} effect_state_t;

/* ========================================================================== */
/* Public API */
/* ========================================================================== */

extern effect_state_t g_effect;
effect_error_t Effects_Init(void);
effect_error_t Effects_Process(void);
void Effects_Stop(void);
void Effects_Start(void);
bool Effects_IsActive(void);
effect_type_t Effects_GetCurrentType(void);
void EffectsToggleBacklight(void);
/* ========================================================================== */
/* Fade Effects */
/* ========================================================================== */

effect_error_t Effects_StartupSequence(void (*callback)(effect_error_t));
effect_error_t Effects_StartRuntimeEffect(void);
effect_error_t Effects_EnterLPM(bool is_usb_connected);
effect_error_t Effects_ExitLPM(bool is_usb_connected);
effect_error_t Effects_ManualEnable(void);
effect_error_t Effects_UpdateConfig(user_effects_config_t *config);
effect_error_t Effects_StartFadeIn(effect_trigger_t trigger,
                                  uint16_t duration_ms,
                                  uint8_t target_brightness,
                                  void (*callback)(effect_error_t));
effect_error_t Effects_StartFadeOut(effect_trigger_t trigger,
                                   uint16_t duration_ms,
                                   void (*callback)(effect_error_t));

/* ========================================================================== */
/* Breathing Effects */
/* ========================================================================== */

effect_error_t Effects_StartBreathing(uint8_t min_brightness,
                                     uint8_t max_brightness,
                                     uint16_t cycle_duration_ms,
                                     Backlight_RGB_t color);

effect_error_t Effects_StartBreathingCurrent(uint8_t min_brightness,
                                            uint8_t max_brightness,
                                            uint16_t cycle_duration_ms);

/* ========================================================================== */
/* Color Transition Effects */
/* ========================================================================== */

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
effect_error_t Effects_PowerOn(void (*callback)(effect_error_t));
effect_error_t Effects_PowerOff(void (*callback)(effect_error_t));
effect_error_t Effects_WakeUp(void);
effect_error_t Effects_Sleep(void (*callback)(effect_error_t));
effect_error_t Effects_IdleBreathing(void);
effect_error_t Effects_LowBatteryBreathing(void);

/* ========================================================================== */
/* Status and Diagnostics */
/* ========================================================================== */
uint8_t Effects_GetProgress(void);
uint8_t Effects_GetErrorCount(void);
void Effects_ResetErrors(void);
uint32_t Effects_GetTimeRemaining(void);

/* ========================================================================== */
/* BLE Status Effects */
/* ========================================================================== */

effect_error_t Effects_BLE_PairingActive(void);
effect_error_t Effects_BLE_PairingSuccess(void);
effect_error_t Effects_BLE_PairingFailed(void);
effect_error_t Effects_BLE_DeviceSwitch(void);
effect_error_t Effects_BLE_ConnectionStatus(bool connected);
effect_error_t Effects_BLE_Stop(void);


/* ========================================================================== */
/* Zones */
/* ========================================================================== */
config_status_t Effects_ClearAllZones(void);
config_status_t Effects_DeactivateZone(uint8_t slot_id);
config_status_t Effects_ActivateZone(uint8_t slot_id);
effect_error_t Effects_ToggleRuntimeZones(bool enable);
effect_error_t Effects_SwitchRuntimeZone(uint8_t slot);
void Effects_GetRuntimeZoneStatus(bool *zones_enabled, uint8_t *active_count,
                                  bool *slot0_active, bool *slot1_active);
bool Effects_HasRuntimeZones(void);

/* ========================================================================
 * CONFIGURATION MODE VISUAL EFFECTS
 * ======================================================================== */
void Effects_ConfigModeInit(void);
void Effects_ConfigModeBreathing(void);
void Effects_ConfigModeMenu(void);
void Effects_ConfigModeZoneSlots(const bool slot_occupied[8]);
void Effects_ConfigModeZoneEdit(void);
void Effects_ConfigModeKeySelection(const uint8_t selected_keys[6][14]);
void Effects_ConfigModeColorPreview(const uint8_t selected_keys[6][14], Backlight_RGB_t color);
void Effects_ConfigModeEffectTypes(uint8_t current_type);
void Effects_ConfigModeEffectPreview(uint8_t effect_type, uint16_t cycle_time_ms);
void Effects_ConfigModeColorPresetPreview(uint8_t preset_index, Backlight_RGB_t color);
void Effects_ConfigModePowerOptions(uint8_t usb_mode, uint8_t battery_mode);
void Effects_ConfigModePowerPreview(uint8_t dim_percentage);
void Effects_ConfigModeBrightnessOptions(uint8_t startup_mode);
void Effects_ConfigModeBrightnessPreview(uint8_t test_brightness);
void Effects_ConfigModeFlash(Backlight_RGB_t color, uint16_t duration_ms);
void Effects_ConfigModeExit(void);

#endif /* BACKLIGHT_CORE_EFFECTS_H_ */
