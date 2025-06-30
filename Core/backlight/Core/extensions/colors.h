/*
* backlight_colors.h
*
* Created on: Jun 10, 2025
* Author: bettysidepiece
*/

#ifndef BACKLIGHT_CORE_EXTENSION_COLORS_H_
#define BACKLIGHT_CORE_EXTENSION_COLORS_H_

#include <stdint.h>
#include <stdbool.h>
#include "backlight.h"

/* ========================================================================== */
/* Color Configuration */
/* ========================================================================== */
// Full power values - no restrictions except blue whites
#define COLOR_MAX_COMPONENT 255 // Maximum safe RGB component value
#define COLOR_MIN_VISIBLE   20  // Minimum for visibility
#define COLOR_OFF           0   // Completely off
#define BACKLIGHT_OFF       ((Backlight_RGB_t){0, 0, 0})

// Pure colors at full intensity
#define COLOR_RED           ((Backlight_RGB_t){255, 0, 0})
#define COLOR_GREEN         ((Backlight_RGB_t){0, 255, 0})
#define COLOR_BLUE          ((Backlight_RGB_t){0, 0, 255})

// Secondary colors - full saturation
#define COLOR_YELLOW        ((Backlight_RGB_t){242, 51, 0})  // Pure yellow
#define COLOR_CYAN          ((Backlight_RGB_t){0, 255, 255}) // Pure cyan
#define COLOR_MAGENTA       ((Backlight_RGB_t){255, 0, 255}) // Pure magenta

// Vibrant mixed colors
#define COLOR_ORANGE        ((Backlight_RGB_t){255, 128, 0})  // Vibrant orange
#define COLOR_PURPLE        ((Backlight_RGB_t){168, 0, 255})  // Deep purple
#define COLOR_PINK          ((Backlight_RGB_t){255, 0, 87})   // Hot pink
#define COLOR_LIME          ((Backlight_RGB_t){168, 255, 0})  // Bright lime
#define COLOR_TEAL          ((Backlight_RGB_t){0, 255, 168})  // Rich teal
#define COLOR_CORAL         ((Backlight_RGB_t){255, 128, 87}) // Coral

// Warm whites - avoid blue-heavy combinations
#define COLOR_WARM_WHITE    ((Backlight_RGB_t){255, 217, 13}) // Incandescent warm
#define COLOR_SOFT_WHITE    ((Backlight_RGB_t){255, 217, 51}) // Soft warm
#define COLOR_CREAM_WHITE   ((Backlight_RGB_t){255, 217, 20}) // Creamy warm
#define COLOR_AMBER_WHITE   ((Backlight_RGB_t){255, 217, 0})  // Amber-tinted

// Specialty colors
#define COLOR_AMBER         ((Backlight_RGB_t){255, 87, 0})   // Pure amber
#define COLOR_GOLD          ((Backlight_RGB_t){255, 168, 0})  // Golden
#define COLOR_CRIMSON       ((Backlight_RGB_t){255, 5, 13})  // Deep red
#define COLOR_FOREST_GREEN  ((Backlight_RGB_t){0, 255, 41})   // Deep green
#define COLOR_ROYAL_BLUE    ((Backlight_RGB_t){41, 0, 255})   // Deep blue
#define COLOR_VIOLET        ((Backlight_RGB_t){214, 0, 168})  // True violet
#define COLOR_TURQUOISE     ((Backlight_RGB_t){0, 255, 255})  // Turquoise
#define COLOR_ROSE          ((Backlight_RGB_t){255, 41, 87})  // Rose pink
/* ========================================================================== */
/* Themed Color Sets */
/* ========================================================================== */

// Gaming theme (high contrast, energetic)
#define COLOR_GAMING_PRIMARY    COLOR_RED
#define COLOR_GAMING_ACCENT     COLOR_CYAN
#define COLOR_GAMING_HIGHLIGHT  COLOR_YELLOW
#define COLOR_GAMING_WARNING    COLOR_ORANGE

// Work theme (warm, professional)
#define COLOR_WORK_PRIMARY      COLOR_WARM_WHITE
#define COLOR_WORK_ACCENT       COLOR_AMBER
#define COLOR_WORK_HIGHLIGHT    COLOR_GOLD
#define COLOR_WORK_SUCCESS      COLOR_GREEN

// Evening theme (relaxing, low strain)
#define COLOR_EVENING_PRIMARY   COLOR_AMBER
#define COLOR_EVENING_ACCENT    COLOR_ORANGE
#define COLOR_EVENING_HIGHLIGHT COLOR_CORAL
#define COLOR_EVENING_DIM       COLOR_ROSE

// Status colors (clear system feedback)
#define COLOR_STATUS_SUCCESS    COLOR_GREEN
#define COLOR_STATUS_WARNING    COLOR_YELLOW
#define COLOR_STATUS_ERROR      COLOR_RED
#define COLOR_STATUS_INFO       COLOR_CYAN
#define COLOR_STATUS_CHARGING   COLOR_AMBER
#define COLOR_STATUS_LOW_BATT   COLOR_CRIMSON

/* ========================================================================== */
/* External Array Declarations */
/* ========================================================================== */

// Colors that look amazing when breathing (smooth transitions)
extern Backlight_RGB_t breathing_colors[];
extern const uint8_t breathing_color_count;

// High contrast colors for notifications/effects
extern const Backlight_RGB_t notification_colors[];
extern const uint8_t notification_color_count;

// Transition-friendly colors (gradual color changes)
extern Backlight_RGB_t transition_colors[];
extern const uint8_t transition_color_count;

// Gamma correction tables
extern const uint8_t gamma_32_steps[32];
extern const uint8_t gamma_64_steps[64];

/* ========================================================================== */
/* Color Utility Function Declarations */
/* ========================================================================== */

/**
* @brief Scale color brightness while maintaining ratios
* @param color Base color
* @param brightness_percent Brightness percentage (0-100)
* @return Scaled color
*/
Backlight_RGB_t Color_Scale(Backlight_RGB_t color, uint8_t brightness_percent);

/**
* @brief Get breathing color by index (cycles through palette)
* @param index Color index
* @return Color from breathing palette
*/
Backlight_RGB_t Color_GetBreathing(uint8_t index);

/**
* @brief Get notification color by type
* @param index Color index
* @return Color from notification palette
*/
Backlight_RGB_t Color_GetNotification(uint8_t index);

/**
* @brief Get transition color by index (for color cycling)
* @param index Color index
* @return Color from transition palette
*/
Backlight_RGB_t Color_GetTransition(uint8_t index);

/**
* @brief Get dimmed version of color (25% brightness)
* @param color Base color
* @return Dimmed color
*/
Backlight_RGB_t Color_Dim(Backlight_RGB_t color);

/**
* @brief Get bright version of color (75% brightness)
* @param color Base color
* @return Bright color
*/
Backlight_RGB_t Color_Bright(Backlight_RGB_t color);

/**
* @brief Mix two colors (50/50 blend)
* @param color1 First color
* @param color2 Second color
* @return Blended color
*/
Backlight_RGB_t Color_Mix(Backlight_RGB_t color1, Backlight_RGB_t color2);

/* ========================================================================== */
/* Gamma Correction Function Declarations */
/* ========================================================================== */

/**
* @brief Apply gamma correction using 32 steps (recommended for T=1s breathing)
* @param linear_value Linear brightness value (0-31)
* @return Gamma corrected PWM value (0-255)
*/
uint8_t Color_ApplyGamma32(uint8_t linear_value);

/**
* @brief Apply gamma correction using 64 steps (recommended for T=2s breathing)
* @param linear_value Linear brightness value (0-63)
* @return Gamma corrected PWM value (0-255)
*/
uint8_t Color_ApplyGamma64(uint8_t linear_value);

/**
* @brief Apply gamma correction to RGB color using 32 steps
* @param linear_color Linear RGB color (components 0-31)
* @return Gamma corrected RGB color (components 0-80)
*/
Backlight_RGB_t Color_ApplyGammaRGB32(Backlight_RGB_t linear_color);

/**
* @brief Convert brightness percentage to gamma-corrected value
* @param brightness_percent Brightness percentage (0-100)
* @param use_64_steps Use 64 steps if true, 32 steps if false
* @return Gamma corrected brightness (0-80)
*/
uint8_t Color_BrightnessToGamma(uint8_t brightness_percent, bool use_64_steps);

/**
* @brief Get breathing color with gamma-corrected brightness
* @param color_index Color index from breathing palette
* @param brightness_step Linear brightness step (0-31 for 32-step gamma)
* @return Gamma-corrected color
*/
Backlight_RGB_t Color_GetBreathingGamma(uint8_t color_index, uint8_t brightness_step);

#endif /* BACKLIGHT_CORE_COLORS_H_ */
