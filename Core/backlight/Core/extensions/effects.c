/*
* effects.c
*
* Created on: Jun 10, 2025
* Author: bettysidepiece
*/

#include "effects.h"
#include "colors.h"
#include "logger.h"
#include "main.h"
#include <string.h>
#include "pmsm.h"

/* ========================================================================== */
/* Private Constants */
/* ========================================================================== */

#define EFFECT_MAX_RETRIES              3
#define EFFECT_RETRY_DELAY_MS           50
#define EFFECT_ERROR_THRESHOLD          5
#define EFFECT_SINE_TABLE_SIZE          48

// Default durations (ms)
#define DEFAULT_FADE_IN_DURATION        1500
#define DEFAULT_FADE_OUT_DURATION       1000
#define DEFAULT_WAKE_DURATION           1500
#define DEFAULT_SLEEP_DURATION          1000
#define DEFAULT_BREATHING_CYCLE         3000

/* ========================================================================== */
/* Lookup Tables */
/* ========================================================================== */

// Smooth sine wave for breathing (32 values, 0-255)
static const uint8_t sine_table[EFFECT_SINE_TABLE_SIZE] = {
   128, 140, 152, 164, 176, 188, 199, 210, 221, 231, 240, 249,
   255, 249, 240, 231, 221, 210, 199, 188, 176, 164, 152, 140,
   128, 115, 103, 91, 79, 67, 56, 45, 34, 24, 15, 6,
   0, 6, 15, 24, 34, 45, 56, 67, 79, 91, 103, 115
};

// Ease-in-out curve for fades (16 values, 0-255)
static const uint8_t ease_curve[32] = {
   0, 1, 2, 4, 6, 9, 12, 16, 20, 25, 30, 36, 42, 49, 56, 64,
   72, 81, 90, 100, 110, 121, 132, 144, 156, 169, 182, 196, 210, 225, 240, 255
};

/* ========================================================================== */
/* Private Variables */
/* ========================================================================== */

static struct {
    bool config_mode_active;
    uint32_t last_update;
    uint8_t breathing_phase;
    bool breathing_direction; // true = up, false = down
    Backlight_RGB_t saved_state[BACKLIGHT_MATRIX_ROWS][BACKLIGHT_MATRIX_COLS];
    bool state_saved;
} g_config_effects = {0};


effect_state_t g_effect = {0};
static bool g_initialized = false;
extern volatile pm_state_t current_pm_state;
extern zone_runtime_ctx_t g_zone_rt_ctx;
/* ========================================================================== */
/* Private Function Prototypes */
/* ========================================================================== */

static effect_error_t ProcessFadeIn(void);
static effect_error_t ProcessFadeOut(void);
static effect_error_t ProcessBreathing(void);
static effect_error_t ProcessColorTransition(void);
static effect_error_t ApplyEffect(Backlight_RGB_t color, uint8_t brightness);
static effect_error_t ApplyEffectWithZones(Backlight_RGB_t bg_color, uint8_t bg_brightness);
static uint8_t GetEaseCurveValue(uint8_t progress);
static void CompleteEffect(effect_error_t error);
static effect_error_t HandleError(effect_error_t error);
static bool IsValidBrightness(uint8_t brightness);
static void Effects_BLE_SuccessComplete(effect_error_t error);
static void Effects_BLE_FailedComplete(effect_error_t error);
static void Effects_BLE_SwitchComplete(effect_error_t error);
static void Effects_BLE_RestorePrevious(effect_error_t error);
static effect_error_t Effects_BlockingFadeOut(uint16_t duration_ms);
static effect_error_t Effects_BlockingFadeToDim(uint8_t target_dim_brightness,
		uint16_t duration_ms);
static bool GetZoneColorForKey(uint8_t row, uint8_t col, Backlight_RGB_t *zone_color);

static effect_error_t Effects_NonBlockingFadeToColor(Backlight_RGB_t target_color,
        uint8_t target_brightness, uint16_t duration_ms);
/* ========================================================================== */
/* Private Functions */
/* ========================================================================== */
/**
 * @brief Get zone color for a specific key (direct lookup in runtime context)
 */
static bool GetZoneColorForKey(uint8_t row, uint8_t col, Backlight_RGB_t *zone_color)
{
    if (!g_zone_rt_ctx.zones_enabled || row >= BACKLIGHT_MATRIX_ROWS || col >= BACKLIGHT_MATRIX_COLS) {
        return false;
    }

    // Search through active runtime zones (max 2 slots)
    for (uint8_t slot = 0; slot < 2; slot++) {
        if (!g_zone_rt_ctx.zones[slot].active) continue;

        // Linear search through keys in this zone
        for (uint8_t i = 0; i < g_zone_rt_ctx.zones[slot].key_count; i++) {
            zone_key_runtime_t *key = &g_zone_rt_ctx.zones[slot].keys[i];

            if (key->row == row && key->col == col) {
                *zone_color = key->color;
                return true;
            }
        }
    }

    return false;
}

static void Effects_StartupCallback(effect_error_t error)
{
    if (error != EFFECT_ERROR_NONE) {
        LOG_ERROR("Startup fade failed: %d", error);
        return;
    }
    // Fade in complete → start the configured runtime effect
    Effects_StartRuntimeEffect();
}


static void Effects_TransitionCallback(effect_error_t error)
{
    if (error != EFFECT_ERROR_NONE) {
        LOG_ERROR("Color transition failed: %d", error);
        return;
    }

    Effects_StartColorCyclingBreathing(
        g_effect.config.breathing_min_brightness,
        g_effect.config.breathing_max_brightness,
        g_effect.config.breathing_cycle_duration_ms
    );
}
/* ========================================================================== */
/* Public API Implementation */
/* ========================================================================== */

effect_error_t Effects_Init(void)
{
   if (g_initialized) {
       return EFFECT_ERROR_NONE;
   }

   memset(&g_effect, 0, sizeof(g_effect));
   g_effect.type = EFFECT_NONE;
   g_effect.active = false;
   g_effect.hardware_ready = true;
   g_effect.backlight_ready = false;
   g_effect.config.effects_enabled = true;
   g_effect.config.runtime_mode = EFFECT_MODE_BREATHING;
   g_effect.config.startup_color_index = 10;
   g_effect.config.runtime_color_index = 10;
   g_effect.config.usb_lmp_behavior = LMP_BEHAVIOR_OFF;
   g_effect.config.ble_lmp_behavior = LMP_BEHAVIOR_OFF;
   g_effect.config.lmp_backlight_mode = 0;
   g_effect.config.lmp_dim_brightness = EFFECT_MIN_BRIGHTNESS;
   g_effect.config.startup_fade_duration_ms = DEFAULT_FADE_IN_DURATION;
   g_effect.config.transition_duration_ms = 1000;
   g_effect.config.breathing_cycle_duration_ms = DEFAULT_BREATHING_CYCLE;
   g_effect.config.breathing_min_brightness = EFFECT_MIN_BRIGHTNESS;
   g_effect.config.breathing_max_brightness = Backlight_GetCurrentMaxBrightness();

   if (Zones_InitRuntime() != CONFIG_OK) {
           LOG_WARNING("Failed to initialize runtime zones - continuing without zones");
       }

   g_initialized = true;
   LOG_INFO("Effects system initialized");

   return EFFECT_ERROR_NONE;
}

effect_error_t Effects_Process(void)
{
    if (!g_initialized || !g_effect.active) {
        return EFFECT_ERROR_NONE;
    }

    if (current_pm_state == PM_STATE_STOP && !g_effect.config.effects_enabled) {
        return EFFECT_ERROR_NONE;
    }

    if (g_effect.config.lpm_backlight_off == true) {
        return EFFECT_ERROR_NONE;
    }

    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed = current_time - g_effect.start_time;

    uint32_t update_interval = (g_effect.type == EFFECT_BREATHING) ?
                              EFFECT_BREATHING_INTERVAL_MS : EFFECT_UPDATE_INTERVAL_MS;

    if (elapsed < (g_effect.current_step * update_interval)) {
        return EFFECT_ERROR_NONE;
    }

    effect_error_t result = EFFECT_ERROR_NONE;

    switch (g_effect.type) {
        case EFFECT_FADE_IN:
            result = ProcessFadeIn();
            break;
        case EFFECT_FADE_OUT:
            result = ProcessFadeOut();
            break;
        case EFFECT_BREATHING:
            result = ProcessBreathing();
            break;
        case EFFECT_COLOR_TRANSITION:
            result = ProcessColorTransition();
            break;
        default:
            result = EFFECT_ERROR_INVALID_PARAM;
            break;
    }

    if (result != EFFECT_ERROR_NONE) {
        return HandleError(result);
    }

    g_effect.current_step++;
    return EFFECT_ERROR_NONE;
}

void Effects_Stop(void)
{
   if (g_effect.active) {
       LOG_DEBUG("Stopping effect type %d", g_effect.type);
       g_effect.active = false;
       g_effect.type = EFFECT_NONE;
   }
}

void Effects_Start(void){
	if (!g_effect.active) {
	   LOG_DEBUG("Starting effect type %d", g_effect.type);
	   g_effect.active = true;
	   g_effect.type = EFFECT_NONE;
   }
}

bool Effects_IsActive(void)
{
   return g_effect.active;
}

effect_type_t Effects_GetCurrentType(void)
{
   return g_effect.type;
}

void EffectsToggleBacklight(void)
{
    if (g_effect.config.effects_enabled) {
        // Turn off backlight but keep effects system running
        Backlight_SetAll(0, 0, 0);
        if (!Backlight_IsAllOff()) {
            Backlight_SetAll(0, 0, 0);
        }
        Backlight_EnterShutdown();

        // Just disable the output, don't stop the engine
        g_effect.config.effects_enabled = false;
        g_effect.config.usb_lmp_behavior = LMP_BEHAVIOR_SYSTEM_OFF;
        g_effect.config.ble_lmp_behavior = LMP_BEHAVIOR_SYSTEM_OFF;

        LOG_INFO("Backlight disabled (effects engine still running)");

    } else {
        // Re-enable and restore
        g_effect.config.effects_enabled = true;
        g_effect.config.usb_lmp_behavior = LMP_BEHAVIOR_OFF;
        g_effect.config.ble_lmp_behavior = LMP_BEHAVIOR_OFF;
        Backlight_ExitShutdown();
		// Start fresh with wake sequence
		Backlight_RGB_t wake_color;
		if (g_effect.is_color_cycling) {
			wake_color = Color_GetTransition(g_effect.cycle_color_index);
		} else {
			wake_color = Color_GetBreathing(g_effect.config.runtime_color_index);
		}
		Effects_StartRuntimeEffect();
    }
}

/* ========================================================================== */
/* Fade Effects Implementation */
/* ========================================================================== */

effect_error_t Effects_StartFadeIn(effect_trigger_t trigger,
                                  uint16_t duration_ms,
                                  uint8_t target_brightness,
                                  void (*callback)(effect_error_t))
{
   if (!g_initialized) {
       return EFFECT_ERROR_INVALID_PARAM;
   }

   if (!IsValidBrightness(target_brightness) || duration_ms > EFFECT_MAX_DURATION_MS) {
       return EFFECT_ERROR_INVALID_PARAM;
   }

   // Stop any active effect
   Effects_Stop();

   // Wake up hardware if needed
   if (Backlight_IsAllOff()) {
       Backlight_ExitShutdown();
       HAL_Delay(1);
   }

   // Initialize fade in effect
   g_effect.type = EFFECT_FADE_IN;
   g_effect.start_time = HAL_GetTick();
   g_effect.duration_ms = duration_ms;
   g_effect.current_step = 0;
   g_effect.total_steps = duration_ms / EFFECT_UPDATE_INTERVAL_MS;
   g_effect.start_brightness = 0;
   g_effect.target_brightness = Backlight_GetCurrentMaxBrightness();
   g_effect.start_color = (Backlight_RGB_t){0, 0, 0};

   // Use saved colors if available, otherwise default
   if (Backlight_HasSavedState()) {
       // Will be restored during processing
       g_effect.target_color = Color_GetBreathing(g_effect.config.startup_color_index);
   } else {
       g_effect.target_color = Color_GetBreathing(g_effect.config.startup_color_index);
   }

   g_effect.active = true;
   g_effect.error_count = 0;
   g_effect.retry_count = 0;
   g_effect.on_complete = callback;

   LOG_INFO("Fade in started: %dms, target: %d", duration_ms, target_brightness);
   return EFFECT_ERROR_NONE;
}

effect_error_t Effects_StartFadeOut(effect_trigger_t trigger,
                                   uint16_t duration_ms,
                                   void (*callback)(effect_error_t))
{
   if (!g_initialized || duration_ms > EFFECT_MAX_DURATION_MS) {
       return EFFECT_ERROR_INVALID_PARAM;
   }

   // Stop any active effect
   Effects_Stop();

   // Don't fade if already off
   if (Backlight_IsAllOff()) {
       if (callback) callback(EFFECT_ERROR_NONE);
       return EFFECT_ERROR_NONE;
   }

   // Initialize fade out effect
   g_effect.type = EFFECT_FADE_OUT;
   g_effect.start_time = HAL_GetTick();
   g_effect.duration_ms = duration_ms;
   g_effect.current_step = 0;
   g_effect.total_steps = duration_ms / EFFECT_UPDATE_INTERVAL_MS;
   g_effect.start_brightness = EFFECT_MAX_BRIGHTNESS;
   g_effect.target_brightness = 0;
   g_effect.active = true;
   g_effect.error_count = 0;
   g_effect.retry_count = 0;
   g_effect.on_complete = callback;

   LOG_INFO("Fade out started: %dms", duration_ms);
   return EFFECT_ERROR_NONE;
}

/* ========================================================================== */
/* Breathing Effects Implementation */
/* ========================================================================== */

effect_error_t Effects_StartBreathing(uint8_t min_brightness,
                                     uint8_t max_brightness,
                                     uint16_t cycle_duration_ms,
                                     Backlight_RGB_t color)
{
   if (!g_initialized) {
       return EFFECT_ERROR_INVALID_PARAM;
   }

   if (!IsValidBrightness(min_brightness) || !IsValidBrightness(max_brightness) ||
       min_brightness >= max_brightness || cycle_duration_ms > EFFECT_MAX_DURATION_MS) {
       return EFFECT_ERROR_INVALID_PARAM;
   }

   // Stop any active effect
   Effects_Stop();

   // Wake up hardware if needed
   if (Backlight_IsAllOff()) {
       Backlight_ExitShutdown();
       HAL_Delay(1);
   }

   // Initialize breathing effect
   g_effect.type = EFFECT_BREATHING;
   g_effect.start_time = HAL_GetTick();
   g_effect.duration_ms = cycle_duration_ms;
   g_effect.current_step = 0;
   g_effect.total_steps = cycle_duration_ms / EFFECT_BREATHING_INTERVAL_MS;
   g_effect.min_brightness = min_brightness;
   g_effect.max_brightness = max_brightness;
   g_effect.target_color = color;
   g_effect.breathing_direction_up = true;
   g_effect.active = true;
   g_effect.error_count = 0;
   g_effect.retry_count = 0;
   g_effect.on_complete = NULL; // Breathing continues indefinitely

   LOG_INFO("Breathing started: min=%d, max=%d, cycle=%dms, color=(%d,%d,%d)",
            min_brightness, max_brightness, cycle_duration_ms,
            color.r, color.g, color.b);
   return EFFECT_ERROR_NONE;
}

effect_error_t Effects_StartBreathingCurrent(uint8_t min_brightness,
                                             uint8_t max_brightness,
                                             uint16_t cycle_duration_ms)
{
    // Get current average color from keyboard (simplified approach)
    uint8_t max_r = 0, max_g, max_b;
    if (Backlight_GetMaxRGBValues(&max_r, &max_g, &max_b) == BACKLIGHT_OK) {
        Backlight_RGB_t current_color = {max_r, max_g, max_b};
        return Effects_StartBreathing(min_brightness, max_brightness, cycle_duration_ms, current_color);
    }

    // Fallback to warm white if can't get current colors
    return Effects_StartBreathing(min_brightness, max_brightness, cycle_duration_ms, Color_GetBreathing(g_effect.config.runtime_color_index));
}


/* ========================================================================== */
/* Color Transition Effects Implementation */
/* ========================================================================== */

effect_error_t Effects_StartColorTransition(Backlight_RGB_t target_color,
                                           uint16_t duration_ms,
                                           void (*callback)(effect_error_t))
{
   if (!g_initialized || duration_ms > EFFECT_MAX_DURATION_MS) {
       return EFFECT_ERROR_INVALID_PARAM;
   }

   // Stop any active effect
   Effects_Stop();

   // Get current color (use warm white as fallback)
   if(g_effect.is_color_cycling == false){
	   g_effect.start_color = transition_colors[0];
   }else{
	   g_effect.start_color = transition_colors[g_effect.cycle_color_index];
   }

   // Initialize color transition effect
   g_effect.type = EFFECT_COLOR_TRANSITION;
   g_effect.start_time = HAL_GetTick();
   g_effect.duration_ms = duration_ms;
   g_effect.current_step = 0;
   g_effect.total_steps = duration_ms / EFFECT_UPDATE_INTERVAL_MS;
   g_effect.target_color = target_color;
   g_effect.active = true;
   g_effect.error_count = 0;
   g_effect.retry_count = 0;
   g_effect.on_complete = callback;

   LOG_INFO("Color transition started: %dms to (%d,%d,%d)",
            duration_ms, target_color.r, target_color.g, target_color.b);
   return EFFECT_ERROR_NONE;
}

effect_error_t Effects_StartColorCyclingBreathing(uint8_t min_brightness,
                                                  uint8_t max_brightness,
                                                  uint16_t cycle_duration_ms)
{
    if (!g_initialized) {
        return EFFECT_ERROR_INVALID_PARAM;
    }

    if (!IsValidBrightness(min_brightness) || !IsValidBrightness(max_brightness) ||
        min_brightness >= max_brightness || cycle_duration_ms > EFFECT_MAX_DURATION_MS) {
        return EFFECT_ERROR_INVALID_PARAM;
    }

    // Stop any active effect
    Effects_Stop();

    // Wake up hardware if needed
    if (Backlight_IsAllOff()) {
        Backlight_ExitShutdown();
        HAL_Delay(1);
    }

    // Initialize breathing effect with cycling flag
    g_effect.type = EFFECT_BREATHING;
    g_effect.start_time = HAL_GetTick();
    g_effect.duration_ms = cycle_duration_ms;
    g_effect.current_step = 0;
    g_effect.total_steps = cycle_duration_ms / EFFECT_BREATHING_INTERVAL_MS;
    g_effect.min_brightness = min_brightness;
    g_effect.max_brightness = max_brightness;
    g_effect.target_color = Color_GetTransition(g_effect.cycle_color_index);
    g_effect.breathing_direction_up = true;
    g_effect.is_color_cycling = true;
    g_effect.cycle_color_index = 0;
    g_effect.active = true;
    g_effect.error_count = 0;
    g_effect.retry_count = 0;
    g_effect.on_complete = NULL;

    LOG_INFO("Color cycling breathing started: min=%d, max=%d, cycle=%dms",
             min_brightness, max_brightness, cycle_duration_ms);
    return EFFECT_ERROR_NONE;
}
/* ========================================================================== */
/* Convenience Functions Implementation */
/* ========================================================================== */

effect_error_t Effects_PowerOn(void (*callback)(effect_error_t))
{
   return Effects_StartFadeIn(EFFECT_TRIGGER_POWER_ON, DEFAULT_FADE_IN_DURATION,
                              EFFECT_MAX_BRIGHTNESS, callback);
}

effect_error_t Effects_PowerOff(void (*callback)(effect_error_t))
{
   return Effects_StartFadeOut(EFFECT_TRIGGER_POWER_OFF, DEFAULT_FADE_OUT_DURATION, callback);
}

effect_error_t Effects_WakeUp(void)
{
   return Effects_StartFadeIn(EFFECT_TRIGGER_WAKE, DEFAULT_WAKE_DURATION,
                              EFFECT_MAX_BRIGHTNESS / 2, NULL);
}

effect_error_t Effects_Sleep(void (*callback)(effect_error_t))
{
   return Effects_StartFadeOut(EFFECT_TRIGGER_SLEEP, DEFAULT_SLEEP_DURATION, callback);
}

effect_error_t Effects_IdleBreathing(void)
{
   return Effects_StartBreathing(EFFECT_MIN_BRIGHTNESS, 25, DEFAULT_BREATHING_CYCLE,
		   Color_GetBreathing(g_effect.config.runtime_color_index)); // First breathing color (amber)
}

effect_error_t Effects_LowBatteryBreathing(void)
{
   return Effects_StartBreathing(EFFECT_MIN_BRIGHTNESS, 20, DEFAULT_BREATHING_CYCLE,
                                 COLOR_STATUS_LOW_BATT); // Red breathing
}

/* ========================================================================== */
/* Status and Diagnostics Implementation */
/* ========================================================================== */

uint8_t Effects_GetProgress(void)
{
   if (!g_effect.active || g_effect.total_steps == 0) {
       return 0;
   }

   uint8_t progress = (g_effect.current_step * 100) / g_effect.total_steps;
   return (progress > 100) ? 100 : progress;
}

uint8_t Effects_GetErrorCount(void)
{
   return g_effect.error_count;
}

void Effects_ResetErrors(void)
{
   g_effect.error_count = 0;
   g_effect.retry_count = 0;
}

uint32_t Effects_GetTimeRemaining(void)
{
   if (!g_effect.active) {
       return 0;
   }

   uint32_t elapsed = HAL_GetTick() - g_effect.start_time;
   if (elapsed >= g_effect.duration_ms) {
       return 0;
   }

   return g_effect.duration_ms - elapsed;
}

/* ========================================================================== */
/* Private Functions Implementation */
/* ========================================================================== */
static effect_error_t ProcessFadeIn(void)
{
    uint32_t elapsed = HAL_GetTick() - g_effect.start_time;

    // Check if effect complete
    if (elapsed >= g_effect.duration_ms) {
        effect_error_t result = EFFECT_ERROR_NONE;

        if (Backlight_HasSavedState()) {
            if (Backlight_On() != BACKLIGHT_OK) {
                result = EFFECT_ERROR_HARDWARE_FAIL;
            }
            // Apply zones if restored state
            if (g_zone_rt_ctx.zones_enabled && g_zone_rt_ctx.active_zone_count > 0) {
                ApplyEffectWithZones(g_effect.target_color, g_effect.target_brightness);
            }
        } else {
            result = (ApplyEffectWithZones(g_effect.target_color, g_effect.target_brightness) == EFFECT_ERROR_NONE) ?
                     EFFECT_ERROR_NONE : EFFECT_ERROR_HARDWARE_FAIL;
        }

        CompleteEffect(result);
        return result;
    }

    // Calculate progress and apply ease curve
    uint8_t progress = (elapsed * 100) / g_effect.duration_ms;
    if (progress > 99) progress = 99;

    uint8_t curve_value = GetEaseCurveValue(progress);
    uint8_t current_brightness = (g_effect.target_brightness * curve_value) / 255;

    // Apply gamma correction
    uint8_t current_max = Backlight_GetCurrentMaxBrightness();
    uint8_t gamma_brightness = Color_BrightnessToGamma((current_brightness * 100) / current_max, false);

    Backlight_RGB_t fade_color = {
        .r = (g_effect.target_color.r * gamma_brightness) / current_max,
        .g = (g_effect.target_color.g * gamma_brightness) / current_max,
        .b = (g_effect.target_color.b * gamma_brightness) / current_max
    };

    return ApplyEffectWithZones(fade_color, gamma_brightness);
}

static effect_error_t ProcessFadeOut(void)
{
    uint32_t elapsed = HAL_GetTick() - g_effect.start_time;

    // Check if effect complete
    if (elapsed >= g_effect.duration_ms) {
        if (Backlight_EnterShutdown() != BACKLIGHT_OK) {
            CompleteEffect(EFFECT_ERROR_HARDWARE_FAIL);
            return EFFECT_ERROR_HARDWARE_FAIL;
        }
        CompleteEffect(EFFECT_ERROR_NONE);
        return EFFECT_ERROR_NONE;
    }

    // Calculate progress and apply reverse ease curve
    uint8_t progress = (elapsed * 100) / g_effect.duration_ms;
    if (progress > 99) progress = 99;

    uint8_t curve_value = GetEaseCurveValue(99 - progress);
    uint8_t current_brightness = (g_effect.start_brightness * curve_value) / 255;

    uint8_t current_max = Backlight_GetCurrentMaxBrightness();
    uint8_t gamma_brightness = Color_BrightnessToGamma((current_brightness * 100) / current_max, false);

    // Get current colors for fade out
    uint8_t max_r, max_g, max_b;
    if (Backlight_GetMaxRGBValues(&max_r, &max_g, &max_b) == BACKLIGHT_OK) {
        uint8_t global_max = max_r;
        if (max_g > global_max) global_max = max_g;
        if (max_b > global_max) global_max = max_b;

        if (global_max > 0) {
            Backlight_RGB_t fade_color = {
                .r = (max_r * gamma_brightness) / global_max,
                .g = (max_g * gamma_brightness) / global_max,
                .b = (max_b * gamma_brightness) / global_max
            };
            return ApplyEffectWithZones(fade_color, gamma_brightness);
        }
    }

    return EFFECT_ERROR_HARDWARE_FAIL;
}


static effect_error_t ProcessBreathing(void)
{
    // Validate effect state
    if (g_effect.duration_ms == 0) {
        LOG_ERROR("ProcessBreathing: duration_ms is zero");
        return EFFECT_ERROR_INVALID_PARAM;
    }

    if (g_effect.min_brightness >= g_effect.max_brightness) {
        LOG_ERROR("ProcessBreathing: invalid brightness range");
        return EFFECT_ERROR_INVALID_PARAM;
    }

    // Calculate breathing phase
    uint32_t elapsed = HAL_GetTick() - g_effect.start_time;
    uint32_t cycle_position = elapsed % g_effect.duration_ms;
    uint8_t sine_progress = (cycle_position * (EFFECT_SINE_TABLE_SIZE - 1)) / g_effect.duration_ms;
    if (sine_progress >= EFFECT_SINE_TABLE_SIZE) {
        sine_progress = EFFECT_SINE_TABLE_SIZE - 1;
    }

    uint8_t sine_value = sine_table[sine_progress];

    // Handle color cycling
    static bool color_changed_this_cycle = false;
    if (g_effect.is_color_cycling && sine_value <= 10) {
        if (!color_changed_this_cycle) {
            g_effect.cycle_color_index = (g_effect.cycle_color_index + 1) % transition_color_count;
            g_effect.target_color = Color_GetTransition(g_effect.cycle_color_index);
            color_changed_this_cycle = true;
        }
    } else if (sine_value > 50) {
        color_changed_this_cycle = false;
    }

    // Calculate breathing intensity
    uint8_t brightness_range = g_effect.max_brightness - g_effect.min_brightness;
    uint8_t linear_brightness = g_effect.min_brightness + ((sine_value * brightness_range) / 255);

    uint8_t current_max = Backlight_GetCurrentMaxBrightness();
    uint8_t brightness_percent = (linear_brightness * 100) / current_max;
    if (brightness_percent > 100) brightness_percent = 100;

    uint8_t gamma_brightness = Color_BrightnessToGamma(brightness_percent, false);

    // Calculate background color with breathing intensity
    Backlight_RGB_t breathing_color = {
        .r = (g_effect.target_color.r * gamma_brightness) / current_max,
        .g = (g_effect.target_color.g * gamma_brightness) / current_max,
        .b = (g_effect.target_color.b * gamma_brightness) / current_max
    };

    // Use zone-aware rendering
    return ApplyEffectWithZones(breathing_color, gamma_brightness);
}

static effect_error_t ProcessColorTransition(void)
{
   uint32_t elapsed = HAL_GetTick() - g_effect.start_time;

   // Check if effect complete
   if (elapsed >= g_effect.duration_ms) {
       // Set final target color
       if (Backlight_SetAll(g_effect.target_color.r, g_effect.target_color.g,
                           g_effect.target_color.b) != BACKLIGHT_OK) {
           CompleteEffect(EFFECT_ERROR_HARDWARE_FAIL);
           return EFFECT_ERROR_HARDWARE_FAIL;
       }

       CompleteEffect(EFFECT_ERROR_NONE);
       return EFFECT_ERROR_NONE;
   }

   // Calculate progress and interpolate colors
   uint8_t progress = (elapsed * 100) / g_effect.duration_ms;
   if (progress > 99) progress = 99;

   uint8_t curve_value = GetEaseCurveValue(progress);

   Backlight_RGB_t current_color = {
       .r = g_effect.start_color.r + ((g_effect.target_color.r - g_effect.start_color.r) * curve_value) / 255,
       .g = g_effect.start_color.g + ((g_effect.target_color.g - g_effect.start_color.g) * curve_value) / 255,
       .b = g_effect.start_color.b + ((g_effect.target_color.b - g_effect.start_color.b) * curve_value) / 255
   };

   uint8_t current_max = Backlight_GetCurrentMaxBrightness();
   return ApplyEffect(current_color, current_max);
}

static effect_error_t ApplyEffectWithZones(Backlight_RGB_t bg_color, uint8_t bg_brightness)
{
    // If zones are disabled, use original behavior
    if (!g_zone_rt_ctx.zones_enabled || g_zone_rt_ctx.active_zone_count == 0) {
        return ApplyEffect(bg_color, bg_brightness);
    }

    // Zone-aware rendering: calculate final color for each key
    for (uint8_t row = 0; row < BACKLIGHT_MATRIX_ROWS; row++) {
        for (uint8_t col = 0; col < BACKLIGHT_MATRIX_COLS; col++) {
            if (!Backlight_HasLED(row, col)) continue;

            Backlight_RGB_t final_color;
            Backlight_RGB_t zone_color;

            // Check if this key has a zone override
            if (GetZoneColorForKey(row, col, &zone_color)) {
                // Use zone color but apply same intensity as background
                uint8_t current_max = Backlight_GetCurrentMaxBrightness();
                if (current_max == 0) current_max = 1; // Avoid division by zero

                final_color.r = (zone_color.r * bg_brightness) / current_max;
                final_color.g = (zone_color.g * bg_brightness) / current_max;
                final_color.b = (zone_color.b * bg_brightness) / current_max;
            } else {
                // Use background effect color
                final_color = bg_color;
            }

            // Apply to hardware
            if (Backlight_SetKeyRGB(row, col, final_color) != BACKLIGHT_OK) {
                return EFFECT_ERROR_HARDWARE_FAIL;
            }
        }
    }

    return EFFECT_ERROR_NONE;
}

static effect_error_t ApplyEffect(Backlight_RGB_t color, uint8_t brightness)
{
   if ((Backlight_SetAll(color.r, color.g, color.b) != BACKLIGHT_OK)
		   && (g_effect.backlight_ready == true)) {
       return EFFECT_ERROR_HARDWARE_FAIL;
   }

   g_effect.backlight_ready = true;
   return EFFECT_ERROR_NONE;
}

static uint8_t GetEaseCurveValue(uint8_t progress)
{
   if (progress >= 100) return 255;

   uint8_t index = (progress * 31) / 99; // Scale to 0-31
   if (index >= 32) index = 31;

   return ease_curve[index];
}

static void CompleteEffect(effect_error_t error)
{
    effect_type_t completed_type = g_effect.type;

    // Clear state first to prevent re-entry issues
    g_effect.active = false;
    g_effect.type = EFFECT_NONE;

    // Safely call callback if valid
    if (g_effect.on_complete != NULL) {
        void (*callback)(effect_error_t) = g_effect.on_complete;
        g_effect.on_complete = NULL;  // Clear before calling to prevent recursion
        callback(error);
    }

    // Log after callback to avoid issues if callback fails
    if (error == EFFECT_ERROR_NONE) {
        LOG_DEBUG("Effect %d completed successfully", completed_type);
    } else {
        LOG_ERROR("Effect %d completed with error %d", completed_type, error);
    }
}

static effect_error_t HandleError(effect_error_t error)
{
   g_effect.error_count++;

   // Try recovery for hardware failures
   if (error == EFFECT_ERROR_HARDWARE_FAIL && g_effect.retry_count < EFFECT_MAX_RETRIES) {
       g_effect.retry_count++;
       HAL_Delay(EFFECT_RETRY_DELAY_MS);
       LOG_WARNING("Effect retry %d/%d", g_effect.retry_count, EFFECT_MAX_RETRIES);
       return EFFECT_ERROR_NONE; // Continue trying
   }

   // Too many errors, abort effect
   if (g_effect.error_count >= EFFECT_ERROR_THRESHOLD) {
       LOG_ERROR("Effect aborted after %d errors", g_effect.error_count);
       CompleteEffect(error);
       return error;
   }

   return error;
}

static bool IsValidBrightness(uint8_t brightness)
{
   return brightness <= EFFECT_MAX_BRIGHTNESS;
}
/* ========================================================================== */
/* BLE Status Effects Implementation */
/* ========================================================================== */
effect_error_t Effects_BLE_PairingActive(void)
{
    // Cyan breathing effect for pairing mode
    return Effects_StartBreathing(8, 40, 2000, COLOR_CYAN);
}

effect_error_t Effects_BLE_PairingSuccess(void)
{
    // Green flash effect - quick fade in/out
    Effects_Stop(); // Stop any current effect

    LOG_INFO("BLE pairing success effect");

    // Set green immediately, then fade out
    Effects_StartColorTransition(COLOR_GREEN, 200, Effects_BLE_SuccessComplete);
    return EFFECT_ERROR_NONE;
}

effect_error_t Effects_BLE_PairingFailed(void)
{
    // Red flash effect - quick flash
    Effects_Stop(); // Stop any current effect

    LOG_INFO("BLE pairing failed effect");

    // Set red immediately, then fade out
    Effects_StartColorTransition(COLOR_RED, 300, Effects_BLE_FailedComplete);
    return EFFECT_ERROR_NONE;
}

effect_error_t Effects_BLE_DeviceSwitch(void)
{
    // Quick cyan flash for device switch
    Effects_Stop(); // Stop any current effect

    LOG_INFO("BLE device switch effect");

    Effects_StartColorTransition(COLOR_CYAN, 400, Effects_BLE_SwitchComplete);
    return EFFECT_ERROR_NONE;
}

effect_error_t Effects_BLE_ConnectionStatus(bool connected)
{
    Effects_Stop(); // Stop any current effect

    if (connected) {
        LOG_INFO("BLE connected effect");
        // Brief green pulse
        Effects_StartColorTransition(COLOR_GREEN, 500, Effects_BLE_RestorePrevious);
    } else {
        LOG_INFO("BLE disconnected effect");
        // Brief orange pulse
        Effects_StartColorTransition(COLOR_ORANGE, 500, Effects_BLE_RestorePrevious);
    }

    return EFFECT_ERROR_NONE;
}

effect_error_t Effects_BLE_Stop(void)
{
    Effects_Stop();
    return EFFECT_ERROR_NONE;
}

/* ========================================================================== */
/* Effects Chaining Implementation */
/* ========================================================================== */

effect_error_t Effects_StartupSequence(void (*callback)(effect_error_t))
{
    if (!g_effect.config.effects_enabled) {
        // Effects disabled, just turn on with saved state
        return Backlight_On();
    }

    // Always start with fade in using startup color, then chain to runtime effect
    return Effects_StartFadeIn(
        EFFECT_TRIGGER_POWER_ON,
        g_effect.config.startup_fade_duration_ms,
        g_effect.config.breathing_max_brightness,
        Effects_StartupCallback  // This will call Effects_StartRuntimeEffect()
    );
}

effect_error_t Effects_StartRuntimeEffect(void)
{
    if (!g_effect.config.effects_enabled) {
        return EFFECT_ERROR_NONE; // Do nothing if effects disabled
    }

    switch (g_effect.config.runtime_mode) {
        case EFFECT_MODE_STATIC:
            // Set static color and stop
        	return (ApplyEffectWithZones(Color_GetBreathing(g_effect.config.runtime_color_index),
				   Backlight_GetCurrentMaxBrightness()) == EFFECT_ERROR_NONE) ?
				   EFFECT_ERROR_NONE : EFFECT_ERROR_HARDWARE_FAIL;

        case EFFECT_MODE_BREATHING:
            // Start breathing effect
            return Effects_StartBreathing(
                g_effect.config.breathing_min_brightness,
                g_effect.config.breathing_max_brightness,
                g_effect.config.breathing_cycle_duration_ms,
                Color_GetBreathing(g_effect.config.runtime_color_index)
            );

        case EFFECT_MODE_TRANSITION_BREATHING:
            // Start color transition, then breathing via callback
            return Effects_StartColorTransition(
				Color_GetTransition(g_effect.cycle_color_index),
                g_effect.config.transition_duration_ms,
                Effects_TransitionCallback  // This will start breathing
            );

        default:
            return EFFECT_ERROR_INVALID_PARAM;
    }
}

effect_error_t Effects_EnterLPM(bool is_usb_connected)
{
    lmp_behavior_t behavior = is_usb_connected ?
        g_effect.config.usb_lmp_behavior :
        g_effect.config.ble_lmp_behavior;

    if (g_effect.is_color_cycling) {
		// Color cycling - use transition colors
		g_effect.target_color = Color_GetTransition(g_effect.cycle_color_index);
	} else {
		// Single color breathing - use breathing colors from config
		g_effect.target_color = Color_GetBreathing(g_effect.config.runtime_color_index);
	}

    switch (behavior) {
        case LMP_BEHAVIOR_OFF:
            //LOG_INFO("LPM: Blocking fade to off");
        	g_effect.config.lmp_backlight_mode = 1;
            return Effects_BlockingFadeOut(DEFAULT_SLEEP_DURATION);

        case LMP_BEHAVIOR_DIM:
            //LOG_INFO("LPM: Blocking fade to dim");
        	g_effect.config.lmp_backlight_mode = 0;
            return Effects_BlockingFadeToDim(g_effect.config.lmp_dim_brightness, DEFAULT_SLEEP_DURATION);
        case LMP_BEHAVIOR_SYSTEM_OFF:
        default:
            return EFFECT_ERROR_INVALID_PARAM;
    }
}

effect_error_t Effects_ExitLPM(bool is_usb_connected)
{
	lmp_behavior_t behavior = is_usb_connected ?
	        g_effect.config.usb_lmp_behavior :
	        g_effect.config.ble_lmp_behavior;

	if(behavior != LMP_BEHAVIOR_SYSTEM_OFF){
		g_effect.target_brightness = Backlight_GetCurrentMaxBrightness();

		Backlight_RGB_t wake_color;
		if (g_effect.is_color_cycling) {
			wake_color = Color_GetTransition(g_effect.cycle_color_index);
		} else {
			wake_color = Color_GetBreathing(g_effect.config.runtime_color_index);
		}

		return Effects_NonBlockingFadeToColor(wake_color, g_effect.target_brightness, DEFAULT_WAKE_DURATION);
	}else{
		return EFFECT_ERROR_NONE;
	}
}

effect_error_t Effects_ManualEnable(void)
{
    // Manual enable always works, even if effects_enabled=false
    // Use default breathing effect
    return Effects_StartBreathing(
        EFFECT_MIN_BRIGHTNESS,
        EFFECT_MAX_BRIGHTNESS / 2,
        DEFAULT_BREATHING_CYCLE,
		Color_GetBreathing(g_effect.config.runtime_color_index) // Default to first color
    );
}

effect_error_t Effects_UpdateConfig(user_effects_config_t *config)
{
    if (!config) {
        return EFFECT_ERROR_INVALID_PARAM;
    }

    // Copy new configuration
    g_effect.config = *config;

    // If currently running an effect and effects are now disabled, stop
    if (!config->effects_enabled && Effects_IsActive()) {
        Effects_Stop();
    }

    return EFFECT_ERROR_NONE;
}

/* ========================================================================== */
/* BLE Effect Completion Callbacks */
/* ========================================================================== */

static void Effects_BLE_SuccessComplete(effect_error_t error)
{
    // After green flash, fade out to previous state
    Effects_StartFadeOut(EFFECT_TRIGGER_USER, 500, Effects_BLE_RestorePrevious);
}

static void Effects_BLE_FailedComplete(effect_error_t error)
{
    // After red flash, fade out to previous state
    Effects_StartFadeOut(EFFECT_TRIGGER_USER, 500, Effects_BLE_RestorePrevious);
}

static void Effects_BLE_SwitchComplete(effect_error_t error)
{
    // After cyan flash, fade out to previous state
    Effects_StartFadeOut(EFFECT_TRIGGER_USER, 500, Effects_BLE_RestorePrevious);
}

static void Effects_BLE_RestorePrevious(effect_error_t error)
{
    // Restore to previous state or idle breathing
    if (Backlight_HasSavedState()) {
        Backlight_On(); // Restore saved colors
    } else {
        Effects_IdleBreathing(); // Default to idle breathing
    }
}

/* ========================================================================== */
/* Blocking Fade Implementation for LPM */
/* ========================================================================== */

static effect_error_t Effects_BlockingFadeOut(uint16_t duration_ms)
{
    if (!g_initialized) {
        return EFFECT_ERROR_INVALID_PARAM;
    }

    if (Backlight_IsAllOff()) {
        return EFFECT_ERROR_NONE;
    }

    // Get current colors
    uint8_t max_r, max_g, max_b;
    if (Backlight_GetMaxRGBValues(&max_r, &max_g, &max_b) != BACKLIGHT_OK) {
        return EFFECT_ERROR_HARDWARE_FAIL;
    }

    uint8_t start_max = max_r;
    if (max_g > start_max) start_max = max_g;
    if (max_b > start_max) start_max = max_b;

    if (start_max == 0) {
        return EFFECT_ERROR_NONE;
    }

    uint32_t start_time = HAL_GetTick();
    uint16_t total_steps = duration_ms / EFFECT_UPDATE_INTERVAL_MS;

    // Blocking fade loop
    for (uint16_t step = 0; step < total_steps; step++) {
        uint32_t step_start_time = HAL_GetTick();
        uint32_t elapsed = step_start_time - start_time;
        uint8_t progress = (elapsed * 100) / duration_ms;
        if (progress > 99) progress = 99;

        uint8_t curve_value = GetEaseCurveValue(99 - progress);
        uint8_t current_brightness = (start_max * curve_value) / 255;

        uint8_t current_max = Backlight_GetCurrentMaxBrightness();
        uint8_t brightness_percent = (current_brightness * 100) / current_max;
        if (brightness_percent > 100) brightness_percent = 100;

        uint8_t gamma_brightness = Color_BrightnessToGamma(brightness_percent, false);

        Backlight_RGB_t fade_color = {
            .r = (max_r * gamma_brightness) / current_max,
            .g = (max_g * gamma_brightness) / current_max,
            .b = (max_b * gamma_brightness) / current_max
        };

        // Use zone-aware rendering for blocking fade
        if (ApplyEffectWithZones(fade_color, gamma_brightness) != EFFECT_ERROR_NONE) {
            return EFFECT_ERROR_HARDWARE_FAIL;
        }

        while ((HAL_GetTick() - step_start_time) < EFFECT_UPDATE_INTERVAL_MS);
    }

    Backlight_SetAll(0, 0, 0);
    if (!Backlight_IsAllOff()) {
        Backlight_SetAll(0, 0, 0);
    }
    Backlight_EnterShutdown();
    LOG_INFO("Blocking fade out complete");
    return EFFECT_ERROR_NONE;
}

static effect_error_t Effects_BlockingFadeToDim(uint8_t target_dim_brightness, uint16_t duration_ms)
{
    if (!g_initialized || !IsValidBrightness(target_dim_brightness)) {
        return EFFECT_ERROR_INVALID_PARAM;
    }

    if (Backlight_IsAllOff()) {
        return EFFECT_ERROR_NONE;
    }

    // Get current colors
    uint8_t max_r, max_g, max_b;
    if (Backlight_GetMaxRGBValues(&max_r, &max_g, &max_b) != BACKLIGHT_OK) {
        return EFFECT_ERROR_HARDWARE_FAIL;
    }

    uint8_t start_max = max_r;
    if (max_g > start_max) start_max = max_g;
    if (max_b > start_max) start_max = max_b;

    if (start_max == 0) {
        return EFFECT_ERROR_NONE;
    }

    uint32_t start_time = HAL_GetTick();
    uint16_t total_steps = duration_ms / EFFECT_UPDATE_INTERVAL_MS;

    for (uint16_t step = 0; step < total_steps; step++) {
        uint32_t step_start_time = HAL_GetTick();
        uint32_t elapsed = step_start_time - start_time;
        uint8_t progress = (elapsed * 100) / duration_ms;
        if (progress > 99) progress = 99;

        uint8_t curve_value = GetEaseCurveValue(99 - progress);

        uint8_t brightness_range = start_max - target_dim_brightness;
        uint8_t current_brightness = start_max - ((brightness_range * (255 - curve_value)) / 255);

        uint8_t current_max = Backlight_GetCurrentMaxBrightness();
        uint8_t brightness_percent = (current_brightness * 100) / current_max;
        if (brightness_percent > 100) brightness_percent = 100;

        uint8_t gamma_brightness = Color_BrightnessToGamma(brightness_percent, false);

        Backlight_RGB_t fade_color = {
            .r = (max_r * gamma_brightness) / current_max,
            .g = (max_g * gamma_brightness) / current_max,
            .b = (max_b * gamma_brightness) / current_max
        };

        if (ApplyEffectWithZones(fade_color, gamma_brightness) != EFFECT_ERROR_NONE) {
            return EFFECT_ERROR_HARDWARE_FAIL;
        }

        while ((HAL_GetTick() - step_start_time) < EFFECT_UPDATE_INTERVAL_MS);
    }

    return EFFECT_ERROR_NONE;
}

static effect_error_t Effects_NonBlockingFadeToColor(Backlight_RGB_t target_color,
        uint8_t target_brightness, uint16_t duration_ms)
{
    // Static variables to track fade state
    static bool fade_initialized = false;
    static uint32_t fade_start_time = 0;
    static uint32_t last_update_time = 0;
    static uint16_t current_step = 0;
    static uint16_t total_steps = 0;
    static uint8_t start_r = 0, start_g = 0, start_b = 0;
    static Backlight_RGB_t safe_target_color = {0};
    static uint32_t current_fade_id = 0;
    static uint8_t target_final_brightness = 0;

    if (!g_initialized) {
        return EFFECT_ERROR_INVALID_PARAM;
    }

    // Generate unique fade ID for this call
    uint32_t new_fade_id = (target_color.r << 16) | (target_color.g << 8) | target_color.b;

    // First call OR new fade parameters - reinitialize
    if (!fade_initialized || current_fade_id != new_fade_id) {
        fade_initialized = false;
        current_fade_id = new_fade_id;

        // STOP all competing effects during fade
        Effects_Stop();

        // Exit shutdown if needed
        if (g_effect.config.lmp_backlight_mode == 1) {
            Backlight_ExitShutdown();
            HAL_Delay(1);
            g_effect.config.lmp_backlight_mode = 0;
        }

        // Get current state
        if (!Backlight_IsAllOff()) {
            Backlight_GetMaxRGBValues(&start_r, &start_g, &start_b);
            LOG_INFO("Starting NEW fade from current state: R=%d G=%d B=%d", start_r, start_g, start_b);
        } else {
            start_r = start_g = start_b = 0;
            LOG_INFO("Starting NEW fade from off state");
        }

        // Calculate safe target values
        uint8_t safe_target_brightness = target_brightness;
        if (safe_target_brightness > Backlight_GetCurrentMaxBrightness()) {
            safe_target_brightness = Backlight_GetCurrentMaxBrightness();
        }

        safe_target_color.r = (target_color.r > Backlight_GetCurrentMaxBrightness()) ?
                             Backlight_GetCurrentMaxBrightness() : target_color.r;
        safe_target_color.g = (target_color.g > Backlight_GetCurrentMaxBrightness()) ?
                             Backlight_GetCurrentMaxBrightness() : target_color.g;
        safe_target_color.b = (target_color.b > Backlight_GetCurrentMaxBrightness()) ?
                             Backlight_GetCurrentMaxBrightness() : target_color.b;

        // Scale target color if needed
        uint8_t target_max = safe_target_color.r;
        if (safe_target_color.g > target_max) target_max = safe_target_color.g;
        if (safe_target_color.b > target_max) target_max = safe_target_color.b;

        if (target_max > safe_target_brightness && target_max > 0) {
            safe_target_color.r = (safe_target_color.r * safe_target_brightness) / target_max;
            safe_target_color.g = (safe_target_color.g * safe_target_brightness) / target_max;
            safe_target_color.b = (safe_target_color.b * safe_target_brightness) / target_max;
        }

        // Store target brightness for final application
        target_final_brightness = safe_target_brightness;

        // Initialize fade parameters
        fade_start_time = HAL_GetTick();
        last_update_time = fade_start_time;
        current_step = 0;
        total_steps = duration_ms / 16;  // 16ms updates (60 FPS)
        if (total_steps == 0) total_steps = 1;
        fade_initialized = true;

        LOG_INFO("Non-blocking fade initialized - %d steps over %dms", total_steps, duration_ms);
        return EFFECT_ERROR_NONE;
    }

    // Check if it's time for next update
    uint32_t now = HAL_GetTick();
    if ((now - last_update_time) < 16) {  // 16ms intervals
        return EFFECT_ERROR_NONE;
    }

    last_update_time = now;
    current_step++;

    // Check if fade complete
    if (current_step >= total_steps) {
        // Fade complete - apply final values with correct target brightness
        uint8_t current_max = Backlight_GetCurrentMaxBrightness();
        uint8_t brightness_percent = (target_final_brightness * 100) / current_max;
        if (brightness_percent > 100) brightness_percent = 100;

        uint8_t final_gamma_brightness = Color_BrightnessToGamma(brightness_percent, false);

        // Apply final color with zone-aware rendering at correct brightness
        ApplyEffectWithZones(safe_target_color, final_gamma_brightness);

        // Reset static variables
        fade_initialized = false;
        current_step = 0;
        current_fade_id = 0;
        target_final_brightness = 0;

        // Start runtime effect
        Effects_StartRuntimeEffect();
        LOG_INFO("Non-blocking fade COMPLETE");
        return EFFECT_COMPLETE;
    }

    // Calculate progress and interpolate
    uint8_t step_progress = (current_step * 100) / total_steps;
    if (step_progress > 99) step_progress = 99;

    uint8_t curve_value = GetEaseCurveValue(step_progress);

    // Safe interpolation for color
    Backlight_RGB_t current_color;

    if (safe_target_color.r >= start_r) {
        current_color.r = start_r + (((safe_target_color.r - start_r) * curve_value) / 255);
    } else {
        current_color.r = start_r - (((start_r - safe_target_color.r) * curve_value) / 255);
    }

    if (safe_target_color.g >= start_g) {
        current_color.g = start_g + (((safe_target_color.g - start_g) * curve_value) / 255);
    } else {
        current_color.g = start_g - (((start_g - safe_target_color.g) * curve_value) / 255);
    }

    if (safe_target_color.b >= start_b) {
        current_color.b = start_b + (((safe_target_color.b - start_b) * curve_value) / 255);
    } else {
        current_color.b = start_b - (((start_b - safe_target_color.b) * curve_value) / 255);
    }

    // Bounds checking
    uint8_t max_brightness = Backlight_GetCurrentMaxBrightness();
    if (current_color.r > max_brightness) current_color.r = max_brightness;
    if (current_color.g > max_brightness) current_color.g = max_brightness;
    if (current_color.b > max_brightness) current_color.b = max_brightness;

    uint8_t current_brightness = current_color.r;
    if (current_color.g > current_brightness) current_brightness = current_color.g;
    if (current_color.b > current_brightness) current_brightness = current_color.b;

    // Apply current step using zone-aware rendering with background brightness
    effect_error_t result = ApplyEffectWithZones(current_color, current_brightness);
    if (result != EFFECT_ERROR_NONE) {
        LOG_WARNING("Zone-aware fade failed at step %d", current_step);
        current_step--; // Retry same step
        return EFFECT_ERROR_NONE;
    }

    return EFFECT_ERROR_NONE; // Still in progress
}

// In effects.c - Implement effect-aware brightness control:
effect_error_t Effects_IncreaseBrightness(void)
{
    // If breathing is active, adjust breathing range
    if (g_effect.active && g_effect.type == EFFECT_BREATHING) {
        // Get current max level
        uint8_t current_level = 0;
        for (uint8_t i = 1; i <= 10; i++) {
            if (g_effect.max_brightness <= brightness_levels[i]) {
                current_level = i;
                break;
            }
        }

        if (current_level >= 10) return EFFECT_ERROR_NONE;

        // Just increase max, keep min unchanged
        g_effect.max_brightness = brightness_levels[current_level + 1];
        return EFFECT_ERROR_NONE;
    }

    // No effect active, use backlight API
    return (Backlight_IncreaseBrightness() == BACKLIGHT_OK) ?
           EFFECT_ERROR_NONE : EFFECT_ERROR_HARDWARE_FAIL;
}

effect_error_t Effects_DecreaseBrightness(void)
{
    // If breathing is active, adjust breathing range
    if (g_effect.active && g_effect.type == EFFECT_BREATHING) {
        // Find current and min levels
        uint8_t current_level = 0, min_level = 0;
        for (uint8_t i = 1; i <= 10; i++) {
            if (g_effect.max_brightness <= brightness_levels[i]) {
                current_level = i;
                break;
            }
        }
        for (uint8_t i = 1; i <= 10; i++) {
            if (g_effect.min_brightness <= brightness_levels[i]) {
                min_level = i;
                break;
            }
        }

        // Don't go below min + 2 levels
        if (current_level <= min_level + 2) {
            return EFFECT_ERROR_NONE;
        }

        g_effect.max_brightness = brightness_levels[current_level - 1];
        return EFFECT_ERROR_NONE;
    }

    // No effect active, use backlight API
    return (Backlight_DecreaseBrightness() == BACKLIGHT_OK) ?
           EFFECT_ERROR_NONE : EFFECT_ERROR_HARDWARE_FAIL;
}

/* ========================================================================== */
/* Zone Effects Integration */
/* ========================================================================== */
config_status_t Effects_ActivateZone(uint8_t slot_id)
{
    if (slot_id >= ZONE_MAX_ZONES) {
        return CONFIG_INVALID_PARAMETER;
    }

    // Load EEPROM zone to runtime slot 0 (primary slot)
    config_status_t result = Zones_LoadToRuntimeSlot(slot_id, 0);

    if (result == CONFIG_OK) {
        // Enable zones if load was successful
        Zones_SetRuntimeEnabled(true);
        LOG_INFO("Zone %d activated in runtime slot 0", slot_id);

    } else {
        LOG_ERROR("Failed to activate zone %d: %d", slot_id, result);
    }

    return result;
}

config_status_t Effects_DeactivateZone(uint8_t slot_id)
{
    if (slot_id >= 2) {
        return CONFIG_INVALID_PARAMETER;
    }

    // Clear the specific runtime slot
    config_status_t result = Zones_ClearRuntimeSlot(slot_id);

    if (result == CONFIG_OK) {
        LOG_INFO("Zone slot %d deactivated", slot_id);
    }

    return result;
}

config_status_t Effects_ClearAllZones(void)
{
    // Use existing zones.c functions
    Zones_SetRuntimeEnabled(false);

    // Clear both runtime slots
    Zones_ClearRuntimeSlot(0);
    Zones_ClearRuntimeSlot(1);

    LOG_INFO("All runtime zones cleared from effects");
    return CONFIG_OK;
}

bool Effects_HasActiveZone(void)
{
    return g_effect.zone_active;
}

uint8_t Effects_GetActiveZone(void)
{
    return g_effect.zone_active ? g_effect.active_zone_slot : 0xFF;
}

/* ========================================================================== */
/* Public Zone Runtime API for Effects */
/* ========================================================================== */

/**
 * @brief Check if runtime zones are enabled
 */
bool Effects_HasRuntimeZones(void)
{
    return g_zone_rt_ctx.active_zone_count > 0;
}

/**
 * @brief Get runtime zone status
 */

void Effects_GetRuntimeZoneStatus(bool *zones_enabled, uint8_t *active_count,
                                  bool *slot0_active, bool *slot1_active)
{
    // Use existing zones.c function
    bool slot0_status, slot1_status;
    bool global_enabled = Zones_GetRuntimeStatus(&slot0_status, &slot1_status);

    if (zones_enabled) {
        *zones_enabled = global_enabled;
    }

    if (active_count) {
        *active_count = g_zone_rt_ctx.active_zone_count;
    }

    if (slot0_active) {
        *slot0_active = slot0_status;
    }

    if (slot1_active) {
        *slot1_active = slot1_status;
    }
}

/**
 * @brief Quick zone switch for runtime (called by user key press)
 * This provides instant switching between the 2 cached zones
 */
effect_error_t Effects_SwitchRuntimeZone(uint8_t slot)
{
    if (slot >= ZONE_MAX_ZONES) {
        return EFFECT_ERROR_INVALID_PARAM;
    }

    // Use existing zones.c function to switch to specific runtime slot
    config_status_t result = Zones_SwitchRuntimeSlot(slot % 2); // Map to runtime slot 0 or 1

    if (result == CONFIG_OK) {
        LOG_INFO("Switched to runtime zone slot %d", slot % 2);

        return EFFECT_COMPLETE;
    } else {
        LOG_ERROR("Failed to switch to zone slot %d", slot);
        return EFFECT_ERROR_INVALID_PARAM;
    }
}

/**
 * @brief Toggle runtime zones on/off (called by user key press)
 */
effect_error_t Effects_ToggleRuntimeZones(bool enable)
{
    // Use existing zones.c function
    Zones_SetRuntimeEnabled(enable);

    if (enable) {
        LOG_INFO("Runtime zones enabled (%d active)", g_zone_rt_ctx.active_zone_count);

    } else {
        LOG_INFO("Runtime zones disabled");
    }

    if(g_effect.config.runtime_mode == EFFECT_MODE_STATIC){
    	Effects_StartRuntimeEffect();
    }

    return EFFECT_COMPLETE;
}

/* ========================================================================
 * CONFIGURATION MODE EFFECTS IMPLEMENTATION
 * ======================================================================== */

void Effects_ConfigModeInit(void)
{
    if (!g_config_effects.state_saved) {
        // Save current lighting state
        for (uint8_t row = 0; row < BACKLIGHT_MATRIX_ROWS; row++) {
            for (uint8_t col = 0; col < BACKLIGHT_MATRIX_COLS; col++) {
                // Get current color from backlight system
                g_config_effects.saved_state[row][col] = (Backlight_RGB_t){0, 0, 0}; // Placeholder
            }
        }
        g_config_effects.state_saved = true;
    }

    g_config_effects.config_mode_active = true;
    g_config_effects.breathing_phase = 20;
    g_config_effects.breathing_direction = true;
    g_config_effects.last_update = HAL_GetTick();

    LOG_INFO("Config mode effects initialized");
}

void Effects_ConfigModeBreathing(void)
{
    if (!g_config_effects.config_mode_active) {
        Effects_ConfigModeInit();
    }

    uint32_t current_time = HAL_GetTick();

    // Update breathing every 50ms for smooth effect
    if (current_time - g_config_effects.last_update >= 50) {
        g_config_effects.last_update = current_time;

        // Simple breathing logic
        if (g_config_effects.breathing_direction) {
            g_config_effects.breathing_phase += 2;
            if (g_config_effects.breathing_phase >= 60) {
                g_config_effects.breathing_direction = false;
            }
        } else {
            g_config_effects.breathing_phase -= 2;
            if (g_config_effects.breathing_phase <= 20) {
                g_config_effects.breathing_direction = true;
            }
        }

        // Apply breathing to all non-highlighted keys
        Backlight_RGB_t breath_color = {
            g_config_effects.breathing_phase / 4,
            g_config_effects.breathing_phase / 4,
            g_config_effects.breathing_phase / 4
        };

        for (uint8_t row = 0; row < BACKLIGHT_MATRIX_ROWS; row++) {
            for (uint8_t col = 0; col < BACKLIGHT_MATRIX_COLS; col++) {
                if (Backlight_HasLED(row, col)) {
                    // Skip section keys (row 1, cols 1-4) - they'll be highlighted separately
                    if (!(row == 1 && col >= 1 && col <= 4)) {
                        Backlight_SetKeyRGB(row, col, breath_color);
                    }
                }
            }
        }
    }
}

void Effects_ConfigModeMenu(void)
{
    // Start breathing background
    Effects_ConfigModeBreathing();

    // Highlight section keys 1-4 on number row
    Backlight_RGB_t section_color = {0, 60, 0}; // Green
    for (uint8_t col = 1; col <= 4; col++) {
        if (Backlight_HasLED(1, col)) {
            Backlight_SetKeyRGB(1, col, section_color);
        }
    }

    LOG_DEBUG("Config menu display active");
}

void Effects_ConfigModeZoneSlots(const bool slot_occupied[8])
{
    // Dim background
    Backlight_RGB_t dim_color = {10, 10, 10};
    Backlight_SetAllRGB(dim_color);

    // Show zone slots 1-8 on number row
    for (uint8_t i = 1; i <= 8; i++) {
        if (Backlight_HasLED(1, i)) {
            Backlight_RGB_t slot_color = slot_occupied[i-1] ?
                (Backlight_RGB_t){0, 60, 0} :    // Green for occupied
                (Backlight_RGB_t){60, 0, 0};     // Red for empty
            Backlight_SetKeyRGB(1, i, slot_color);
        }
    }

    LOG_DEBUG("Zone slots display: %d occupied",
              slot_occupied[0] + slot_occupied[1] + slot_occupied[2] + slot_occupied[3] +
              slot_occupied[4] + slot_occupied[5] + slot_occupied[6] + slot_occupied[7]);
}

void Effects_ConfigModeZoneEdit(void)
{
    // Set dim background for key selection
    Backlight_RGB_t dim_color = {5, 5, 5};
    Backlight_SetAllRGB(dim_color);

    LOG_DEBUG("Zone edit mode active");
}

void Effects_ConfigModeKeySelection(const uint8_t selected_keys[6][14])
{
    // Update key selection visualization
    Backlight_RGB_t selected_color = {0, 40, 60};  // Cyan for selected
    Backlight_RGB_t dim_color = {5, 5, 5};         // Dim for unselected

    for (uint8_t row = 0; row < 6; row++) {
        for (uint8_t col = 0; col < 14; col++) {
            if (Backlight_HasLED(row, col)) {
                Backlight_RGB_t color = selected_keys[row][col] ? selected_color : dim_color;
                Backlight_SetKeyRGB(row, col, color);
            }
        }
    }
}

void Effects_ConfigModeColorPreview(const uint8_t selected_keys[6][14], Backlight_RGB_t color)
{
    // Apply color to selected keys only
    for (uint8_t row = 0; row < 6; row++) {
        for (uint8_t col = 0; col < 14; col++) {
            if (selected_keys[row][col] && Backlight_HasLED(row, col)) {
                Backlight_SetKeyRGB(row, col, color);
            }
        }
    }

    LOG_DEBUG("Color preview: RGB(%d,%d,%d)", color.r, color.g, color.b);
}

void Effects_ConfigModeEffectTypes(uint8_t current_type)
{
    // Dim background
    Backlight_RGB_t dim_color = {10, 10, 10};
    Backlight_SetAllRGB(dim_color);

    // Show effect options on number row
    Backlight_RGB_t option_color = {40, 40, 40};   // Gray for options
    Backlight_RGB_t active_color = {0, 60, 60};    // Cyan for selected

    for (uint8_t i = 1; i <= 3; i++) {
        if (Backlight_HasLED(1, i)) {
            Backlight_RGB_t color = (i == current_type) ? active_color : option_color;
            Backlight_SetKeyRGB(1, i, color);
        }
    }

    LOG_DEBUG("Effect types display: type %d selected", current_type);
}

void Effects_ConfigModeEffectPreview(uint8_t effect_type, uint16_t cycle_time_ms)
{
    switch (effect_type) {
        case 1: // Static
            {
                Backlight_RGB_t static_color = {60, 30, 0}; // Warm white
                Backlight_SetAllRGB(static_color);
            }
            break;
        case 2: // Breathing
            {
                // Start a simple breathing preview
                static uint32_t last_breath = 0;
                static uint8_t breath_phase = 0;
                static bool breath_up = true;

                uint32_t now = HAL_GetTick();
                uint32_t breath_interval = cycle_time_ms / 64; // 64 steps per cycle

                if (now - last_breath >= breath_interval) {
                    last_breath = now;

                    if (breath_up) {
                        breath_phase += 4;
                        if (breath_phase >= 80) breath_up = false;
                    } else {
                        breath_phase -= 4;
                        if (breath_phase <= 20) breath_up = true;
                    }

                    Backlight_RGB_t breath_color = {breath_phase, breath_phase/2, 0};
                    Backlight_SetAllRGB(breath_color);
                }
            }
            break;
        case 3: // Color cycling
            {
                // Simple color cycling preview
                static uint32_t last_cycle = 0;
                static uint16_t hue = 0;

                uint32_t now = HAL_GetTick();
                uint32_t cycle_interval = cycle_time_ms / 360; // 360 steps per cycle

                if (now - last_cycle >= cycle_interval) {
                    last_cycle = now;
                    hue = (hue + 10) % 360;

                    Backlight_RGB_t cycle_color = Backlight_HSVtoRGB(hue, 100, 70);
                    Backlight_SetAllRGB(cycle_color);
                }
            }
            break;
    }

    LOG_DEBUG("Effect preview: type %d, cycle %dms", effect_type, cycle_time_ms);
}

void Effects_ConfigModeColorPresetPreview(uint8_t preset_index, Backlight_RGB_t color)
{
    // Show color on specific key in number row
    if (preset_index < 6 && Backlight_HasLED(1, preset_index + 1)) {
        Backlight_SetKeyRGB(1, preset_index + 1, color);
    }
}

void Effects_ConfigModePowerOptions(uint8_t usb_mode, uint8_t battery_mode)
{
    // Dim background
    Backlight_RGB_t dim_color = {10, 10, 10};
    Backlight_SetAllRGB(dim_color);

    Backlight_RGB_t option_color = {40, 40, 40};   // Gray for options
    Backlight_RGB_t active_color = {0, 60, 60};    // Cyan for selected

    // USB mode indicators (top row)
    for (uint8_t i = 1; i <= 3; i++) {
        if (Backlight_HasLED(0, i)) {
            Backlight_RGB_t color = (i == usb_mode) ? active_color : option_color;
            Backlight_SetKeyRGB(0, i, color);
        }
    }

    // Battery mode indicators (number row)
    for (uint8_t i = 1; i <= 3; i++) {
        if (Backlight_HasLED(1, i)) {
            Backlight_RGB_t color = (i == battery_mode) ? active_color : option_color;
            Backlight_SetKeyRGB(1, i, color);
        }
    }

    LOG_DEBUG("Power options: USB=%d, Battery=%d", usb_mode, battery_mode);
}

void Effects_ConfigModePowerPreview(uint8_t dim_percentage)
{
    // Show dimming preview
    uint8_t preview_brightness = (80 * dim_percentage) / 100;
    Backlight_RGB_t preview_color = {preview_brightness, preview_brightness/2, 0};
    Backlight_SetAllRGB(preview_color);

    LOG_DEBUG("Power preview: %d%% dim", dim_percentage);
}

void Effects_ConfigModeBrightnessOptions(uint8_t startup_mode)
{
    // Dim background
    Backlight_RGB_t dim_color = {10, 10, 10};
    Backlight_SetAllRGB(dim_color);

    Backlight_RGB_t option_color = {40, 40, 40};   // Gray for options
    Backlight_RGB_t active_color = {0, 60, 60};    // Cyan for selected

    // Show startup mode options on number row
    for (uint8_t i = 1; i <= 3; i++) {
        if (Backlight_HasLED(1, i)) {
            Backlight_RGB_t color = (i == startup_mode) ? active_color : option_color;
            Backlight_SetKeyRGB(1, i, color);
        }
    }

    LOG_DEBUG("Brightness options: startup mode %d", startup_mode);
}

void Effects_ConfigModeBrightnessPreview(uint8_t test_brightness)
{
    // Apply test brightness to current limit
    Backlight_SetCurrentLimit(test_brightness);

    LOG_DEBUG("Brightness preview: %d", test_brightness);
}

void Effects_ConfigModeFlash(Backlight_RGB_t color, uint16_t duration_ms)
{
    // Simple flash implementation
    Backlight_SetAllRGB(color);
    HAL_Delay(duration_ms);

    // Don't restore here - let the calling state machine handle restoration
    LOG_DEBUG("Config flash: RGB(%d,%d,%d) for %dms", color.r, color.g, color.b, duration_ms);
}

void Effects_ConfigModeExit(void)
{
    g_config_effects.config_mode_active = false;

    // Restore saved state if available
    if (g_config_effects.state_saved) {
        for (uint8_t row = 0; row < BACKLIGHT_MATRIX_ROWS; row++) {
            for (uint8_t col = 0; col < BACKLIGHT_MATRIX_COLS; col++) {
                if (Backlight_HasLED(row, col)) {
                    Backlight_SetKeyRGB(row, col, g_config_effects.saved_state[row][col]);
                }
            }
        }
        g_config_effects.state_saved = false;
    }

    LOG_INFO("Config mode effects exited");
}
