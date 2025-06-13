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
#define DEFAULT_FADE_IN_DURATION        1200
#define DEFAULT_FADE_OUT_DURATION       1000
#define DEFAULT_WAKE_DURATION           1000
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

effect_state_t g_effect = {0};
static bool g_initialized = false;
extern volatile pm_state_t current_pm_state;

/* ========================================================================== */
/* Private Function Prototypes */
/* ========================================================================== */

static effect_error_t ProcessFadeIn(void);
static effect_error_t ProcessFadeOut(void);
static effect_error_t ProcessBreathing(void);
static effect_error_t ProcessColorTransition(void);
static effect_error_t ApplyEffect(Backlight_RGB_t color, uint8_t brightness);
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
static effect_error_t Effects_BlockingFadeToColor(Backlight_RGB_t target_color,
		uint8_t target_brightness, uint16_t duration_ms);
/* ========================================================================== */
/* Private Functions */
/* ========================================================================== */
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
   g_effect.config.runtime_mode = EFFECT_MODE_TRANSITION_BREATHING;
   g_effect.config.startup_color_index = 2;
   g_effect.config.runtime_color_index = 2;
   g_effect.config.usb_lmp_behavior = LMP_BEHAVIOR_OFF;
   g_effect.config.ble_lmp_behavior = LMP_BEHAVIOR_OFF;
   g_effect.config.lmp_backlight_mode = 0;
   g_effect.config.lmp_dim_brightness = EFFECT_MIN_BRIGHTNESS;
   g_effect.config.startup_fade_duration_ms = DEFAULT_FADE_IN_DURATION;
   g_effect.config.transition_duration_ms = 1000;
   g_effect.config.breathing_cycle_duration_ms = DEFAULT_BREATHING_CYCLE;
   g_effect.config.breathing_min_brightness = EFFECT_MIN_BRIGHTNESS;
   g_effect.config.breathing_max_brightness = Backlight_GetCurrentMaxBrightness();

   g_initialized = true;
   LOG_INFO("Effects system initialized");

   return EFFECT_ERROR_NONE;
}

effect_error_t Effects_Process(void)
{
   if (!g_initialized || !g_effect.active) {
       return EFFECT_ERROR_NONE;
   }

   // Skip processing in deep sleep
   if (current_pm_state == PM_STATE_STOP
		   && !g_effect.config.effects_enabled) {
       return EFFECT_ERROR_NONE;
   }

   if(g_effect.config.lpm_backlight_off == true){
	   return EFFECT_ERROR_NONE;
   }

   uint32_t current_time = HAL_GetTick();
   uint32_t elapsed = current_time - g_effect.start_time;

   // Determine update interval based on effect type
   uint32_t update_interval = (g_effect.type == EFFECT_BREATHING) ?
                              EFFECT_BREATHING_INTERVAL_MS : EFFECT_UPDATE_INTERVAL_MS;

   // Check if it's time for next update
   if (elapsed < (g_effect.current_step * update_interval)) {
       return EFFECT_ERROR_NONE;
   }

   effect_error_t result = EFFECT_ERROR_NONE;

   // Process based on effect type
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

   // Handle errors with retry logic
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

bool Effects_IsActive(void)
{
   return g_effect.active;
}

effect_type_t Effects_GetCurrentType(void)
{
   return g_effect.type;
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
    g_effect.is_color_cycling = true; // Add this flag to effect_state_t
    g_effect.cycle_color_index = 0;   // Add this field to effect_state_t
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
       // Restore saved state or set target color
       effect_error_t result = EFFECT_ERROR_NONE;

       if (Backlight_HasSavedState()) {
           if (Backlight_On() != BACKLIGHT_OK) {
               result = EFFECT_ERROR_HARDWARE_FAIL;
           }
       } else {
           if (Backlight_SetAll(g_effect.target_color.r, g_effect.target_color.g,
                               g_effect.target_color.b) != BACKLIGHT_OK) {
               result = EFFECT_ERROR_HARDWARE_FAIL;
           }
       }

       CompleteEffect(result);
       return result;
   }

   // Calculate progress and apply ease curve
   uint8_t progress = (elapsed * 100) / g_effect.duration_ms;
   if (progress > 99) progress = 99;

   uint8_t curve_value = GetEaseCurveValue(progress);
   uint8_t current_brightness = (g_effect.target_brightness * curve_value) / 255;

   // Apply fade using gamma correction for smoothness
   uint8_t current_max = Backlight_GetCurrentMaxBrightness();
   uint8_t gamma_brightness = Color_BrightnessToGamma((current_brightness * 100) / current_max, false);

   Backlight_RGB_t fade_color = {
       .r = (g_effect.target_color.r * gamma_brightness) / current_max,
       .g = (g_effect.target_color.g * gamma_brightness) / current_max,
       .b = (g_effect.target_color.b * gamma_brightness) / current_max
   };

   return ApplyEffect(fade_color, current_brightness);
}

static effect_error_t ProcessFadeOut(void)
{
   uint32_t elapsed = HAL_GetTick() - g_effect.start_time;

   // Check if effect complete
   if (elapsed >= g_effect.duration_ms) {
       // Enter shutdown
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

   uint8_t curve_value = GetEaseCurveValue(99 - progress); // Reverse curve
   uint8_t current_brightness = (g_effect.start_brightness * curve_value) / 255;

   // Apply gamma correction for smooth fade out
   uint8_t current_max = Backlight_GetCurrentMaxBrightness();
   uint8_t gamma_brightness = Color_BrightnessToGamma((current_brightness * 100) / current_max, false);

   // Scale current colors down
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
           return ApplyEffect(fade_color, current_brightness);
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

    // Safe time calculations
    uint32_t elapsed = HAL_GetTick() - g_effect.start_time;
    uint32_t cycle_position = elapsed % g_effect.duration_ms;

    // Safe step calculation with bounds checking (use 47 for 48-element table)
    uint8_t sine_progress = (cycle_position * (EFFECT_SINE_TABLE_SIZE - 1)) / g_effect.duration_ms;
    if (sine_progress >= EFFECT_SINE_TABLE_SIZE) {
        sine_progress = EFFECT_SINE_TABLE_SIZE - 1;
    }

    uint8_t sine_value = sine_table[sine_progress];

    static bool color_changed_this_cycle = false;
    if (g_effect.is_color_cycling && sine_value <= 10) { // Near minimum
        if (!color_changed_this_cycle) {
            // Change to next color
            g_effect.cycle_color_index = (g_effect.cycle_color_index + 1) % transition_color_count;
            g_effect.target_color = Color_GetTransition(g_effect.cycle_color_index);
            color_changed_this_cycle = true;
        }
    } else if (sine_value > 50) {
        color_changed_this_cycle = false; // Reset for next cycle
    }

    // Safe brightness range calculation
    uint8_t brightness_range = g_effect.max_brightness - g_effect.min_brightness;
    uint8_t linear_brightness = g_effect.min_brightness + ((sine_value * brightness_range) / 255);

    // Apply gamma correction with bounds checking
    uint8_t current_max = Backlight_GetCurrentMaxBrightness();
    uint8_t brightness_percent = (linear_brightness * 100) / current_max;
    if (brightness_percent > 100) brightness_percent = 100;

    uint8_t gamma_brightness = Color_BrightnessToGamma(brightness_percent, false);

    // Safe color scaling using current max brightness
    Backlight_RGB_t breathing_color = {
        .r = (g_effect.target_color.r * gamma_brightness) / current_max,
        .g = (g_effect.target_color.g * gamma_brightness) / current_max,
        .b = (g_effect.target_color.b * gamma_brightness) / current_max
    };

    return ApplyEffect(breathing_color, gamma_brightness);
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
            return Backlight_SetAllRGB(Color_GetBreathing(g_effect.config.runtime_color_index));

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

        default:
            return EFFECT_ERROR_INVALID_PARAM;
    }
}

effect_error_t Effects_ExitLPM(bool is_usb_connected)
{

	g_effect.target_brightness = Backlight_GetCurrentMaxBrightness();

	Backlight_RGB_t wake_color;
	if (g_effect.is_color_cycling) {
		wake_color = Color_GetTransition(g_effect.cycle_color_index);
	} else {
		wake_color = Color_GetBreathing(g_effect.config.runtime_color_index);
	}

	return Effects_BlockingFadeToColor(wake_color, g_effect.target_brightness, DEFAULT_WAKE_DURATION);
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

    // Don't fade if already off
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
        return EFFECT_ERROR_NONE; // Already off
    }

    uint32_t start_time = HAL_GetTick();
    uint16_t total_steps = duration_ms / EFFECT_UPDATE_INTERVAL_MS;

    // Blocking fade loop
    for (uint16_t step = 0; step < total_steps; step++) {
        uint32_t step_start_time = HAL_GetTick();
        uint32_t elapsed = step_start_time - start_time;
        uint8_t progress = (elapsed * 100) / duration_ms;
        if (progress > 99) progress = 99;

        uint8_t curve_value = GetEaseCurveValue(99 - progress); // Reverse curve
        uint8_t current_brightness = (start_max * curve_value) / 255;

        // Apply gamma correction
        uint8_t current_max = Backlight_GetCurrentMaxBrightness();
        uint8_t brightness_percent = (current_brightness * 100) / current_max;
        if (brightness_percent > 100) brightness_percent = 100;

        uint8_t gamma_brightness = Color_BrightnessToGamma(brightness_percent, false);

        // Scale colors down
        Backlight_RGB_t fade_color = {
            .r = (max_r * gamma_brightness) / current_max,
            .g = (max_g * gamma_brightness) / current_max,
            .b = (max_b * gamma_brightness) / current_max
        };

        if (Backlight_SetAll(fade_color.r, fade_color.g, fade_color.b) != BACKLIGHT_OK) {
            return EFFECT_ERROR_HARDWARE_FAIL;
        }

        // Wait for next step
        while ((HAL_GetTick() - step_start_time) < EFFECT_UPDATE_INTERVAL_MS);
    }

    Backlight_SetAll(0,0,0);

    if (!Backlight_IsAllOff()) {
    	Backlight_SetAll(0,0,0);
	}

    Backlight_EnterShutdown();
    LOG_INFO("Blocking fade out complete");
    return EFFECT_ERROR_NONE;
}

static effect_error_t Effects_BlockingFadeToDim(uint8_t target_dim_brightness, uint16_t duration_ms)
{
   if (!g_initialized) {
       return EFFECT_ERROR_INVALID_PARAM;
   }

   if (!IsValidBrightness(target_dim_brightness)) {
       return EFFECT_ERROR_INVALID_PARAM;
   }

   // Don't fade if already off
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
       return EFFECT_ERROR_NONE; // Already off
   }

   uint32_t start_time = HAL_GetTick();
   uint16_t total_steps = duration_ms / EFFECT_UPDATE_INTERVAL_MS;

   // Blocking fade loop
   for (uint16_t step = 0; step < total_steps; step++) {
       uint32_t step_start_time = HAL_GetTick();
       uint32_t elapsed = step_start_time - start_time;
       uint8_t progress = (elapsed * 100) / duration_ms;
       if (progress > 99) progress = 99;

       uint8_t curve_value = GetEaseCurveValue(99 - progress); // Reverse curve

       // Scale from start to target (not to 0)
       uint8_t brightness_range = start_max - target_dim_brightness;
       uint8_t current_brightness = start_max - ((brightness_range * (255 - curve_value)) / 255);

       // Apply gamma correction
       uint8_t current_max = Backlight_GetCurrentMaxBrightness();
       uint8_t brightness_percent = (current_brightness * 100) / current_max;
       if (brightness_percent > 100) brightness_percent = 100;

       uint8_t gamma_brightness = Color_BrightnessToGamma(brightness_percent, false);

       // Scale colors proportionally
       Backlight_RGB_t fade_color = {
           .r = (max_r * gamma_brightness) / current_max,
           .g = (max_g * gamma_brightness) / current_max,
           .b = (max_b * gamma_brightness) / current_max
       };

       if (Backlight_SetAll(fade_color.r, fade_color.g, fade_color.b) != BACKLIGHT_OK) {
           return EFFECT_ERROR_HARDWARE_FAIL;
       }

       // Wait for next step
       while ((HAL_GetTick() - step_start_time) < EFFECT_UPDATE_INTERVAL_MS);
   }

   LOG_INFO("Blocking fade to dim complete");
   return EFFECT_ERROR_NONE;
}

static effect_error_t Effects_BlockingFadeToColor(Backlight_RGB_t target_color,
        uint8_t target_brightness, uint16_t duration_ms)
{
    if (!g_initialized) {
        return EFFECT_ERROR_INVALID_PARAM;
    }

    // Exit shutdown if needed
    if (g_effect.config.lmp_backlight_mode == 1) {
        Backlight_ExitShutdown();
        HAL_Delay(1);
        g_effect.config.lmp_backlight_mode = 0;
    }

    // Get ACTUAL current colors and brightness from hardware state
    uint8_t start_r = 0, start_g = 0, start_b = 0;
    uint8_t current_brightness = 0;

    // If we're in dim mode, get current dim colors
    if (!Backlight_IsAllOff()) {
        Backlight_GetMaxRGBValues(&start_r, &start_g, &start_b);

        // Find current brightness level
        current_brightness = start_r;
        if (start_g > current_brightness) current_brightness = start_g;
        if (start_b > current_brightness) current_brightness = start_b;

        LOG_INFO("Starting fade from current state: R=%d G=%d B=%d (brightness=%d)",
                 start_r, start_g, start_b, current_brightness);
    } else {
        LOG_INFO("Starting fade from off state");
    }

    // Ensure target values don't exceed hardware limits
    uint8_t safe_target_brightness = target_brightness;
    if (safe_target_brightness > Backlight_GetCurrentMaxBrightness()) {
        safe_target_brightness = Backlight_GetCurrentMaxBrightness();
    }

    // Scale target color to safe brightness
    Backlight_RGB_t safe_target_color = {
        .r = (target_color.r > Backlight_GetCurrentMaxBrightness())
		? Backlight_GetCurrentMaxBrightness() : target_color.r,
        .g = (target_color.g > Backlight_GetCurrentMaxBrightness())
		? Backlight_GetCurrentMaxBrightness() : target_color.g,
        .b = (target_color.b > Backlight_GetCurrentMaxBrightness())
		? Backlight_GetCurrentMaxBrightness() : target_color.b
    };

    // Further scale target color if needed to stay within brightness limit
    uint8_t target_max = safe_target_color.r;
    if (safe_target_color.g > target_max) target_max = safe_target_color.g;
    if (safe_target_color.b > target_max) target_max = safe_target_color.b;

    if (target_max > safe_target_brightness && target_max > 0) {
        // Scale down proportionally
        safe_target_color.r = (safe_target_color.r * safe_target_brightness) / target_max;
        safe_target_color.g = (safe_target_color.g * safe_target_brightness) / target_max;
        safe_target_color.b = (safe_target_color.b * safe_target_brightness) / target_max;
    }

    uint32_t start_time = HAL_GetTick();
    uint16_t total_steps = duration_ms / EFFECT_UPDATE_INTERVAL_MS;
    if (total_steps == 0) total_steps = 1; // Minimum 1 step

    // Blocking fade loop with safe interpolation
    for (uint16_t step = 0; step < total_steps; step++) {
        uint32_t step_start_time = HAL_GetTick();
        uint32_t elapsed = step_start_time - start_time;
        uint8_t progress = (elapsed * 100) / duration_ms;
        if (progress > 99) progress = 99;

        uint8_t curve_value = GetEaseCurveValue(progress);

        // Safe interpolation - ensure no overflow
        Backlight_RGB_t current_color;

        // Use safe arithmetic to prevent overflow
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

        // Double-check bounds before sending to hardware
        if (current_color.r > Backlight_GetCurrentMaxBrightness()){
        	current_color.r = Backlight_GetCurrentMaxBrightness();
        }
        if (current_color.g > Backlight_GetCurrentMaxBrightness()){
        	current_color.g = Backlight_GetCurrentMaxBrightness();
        }
        if (current_color.b > Backlight_GetCurrentMaxBrightness()){
        	current_color.b = Backlight_GetCurrentMaxBrightness();
        }

        // Apply to hardware with error handling
        if (Backlight_SetAll(current_color.r, current_color.g, current_color.b) != BACKLIGHT_OK) {
            LOG_WARNING("I2C failed during fade to color step %d", step);
            // Don't abort on single failures - continue fade
        }

        // Wait for next step with precise timing
        while ((HAL_GetTick() - step_start_time) < EFFECT_UPDATE_INTERVAL_MS) {
            // Busy wait for precise timing
        }
    }

    // Set final target color with bounds checking
    Backlight_SetAll(safe_target_color.r, safe_target_color.g, safe_target_color.b);

    // Start runtime effect after fade
    return Effects_StartRuntimeEffect();
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
