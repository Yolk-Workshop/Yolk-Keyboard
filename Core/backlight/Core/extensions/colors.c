/*
 * backlight_colors.c
 *
 * Created on: Jun 10, 2025
 * Author: bettysidepiece
 */

#include "colors.h"
#include "logger.h"

/* ========================================================================== */
/* Color Arrays Definitions */
/* ========================================================================== */

// Colors that look amazing when breathing (smooth transitions)
Backlight_RGB_t breathing_colors[] = {
    COLOR_AMBER,            // 0 - Warm, soothing
    COLOR_ORANGE,           // 1 - Energetic
    COLOR_CORAL,            // 2 - Soft, pleasant
    COLOR_PINK,             // 3 - Calming
    COLOR_PURPLE,           // 4 - Mysterious
    COLOR_VIOLET,           // 5 - Deep, rich
    COLOR_CYAN,             // 6 - Cool, modern
    COLOR_TEAL,             // 7 - Balanced
    COLOR_LIME,             // 8 - Fresh, vibrant
    COLOR_YELLOW,           // 9 - Bright, cheerful
	COLOR_CREAM_WHITE,       // 10 - Professional
    COLOR_ROSE              // 11 - Gentle, warm
};

const uint8_t breathing_color_count = sizeof(breathing_colors) / sizeof(breathing_colors[0]);

// High contrast colors for notifications/effects
const Backlight_RGB_t notification_colors[] = {
    COLOR_RED,              // 0 - Emergency/Error
    COLOR_GREEN,            // 1 - Success/OK
    COLOR_YELLOW,           // 2 - Warning/Attention
    COLOR_CYAN,             // 3 - Information
    COLOR_MAGENTA,          // 4 - Special/Custom
    COLOR_ORANGE            // 5 - Activity/Processing
};

const uint8_t notification_color_count = sizeof(notification_colors) / sizeof(notification_colors[0]);

// Transition-friendly colors (gradual color changes)
Backlight_RGB_t transition_colors[] = {
    COLOR_RED,              // 0
    COLOR_ORANGE,           // 1
    COLOR_YELLOW,           // 2
    COLOR_LIME,             // 3
    COLOR_GREEN,            // 4
    COLOR_TEAL,             // 5
    COLOR_CYAN,             // 6
    COLOR_BLUE,             // 7
    COLOR_PURPLE,           // 8
    COLOR_MAGENTA           // 9
};

const uint8_t transition_color_count = sizeof(transition_colors) / sizeof(transition_colors[0]);

/* ========================================================================== */
/* Gamma Correction Tables (From IS31FL3743A Datasheet) */
/* ========================================================================== */

// 32 Gamma Steps with 256 PWM Steps (from datasheet Table 15)
const uint8_t gamma_32_steps[32] = {
    0, 1, 2, 4, 6, 10, 13, 18, 22, 28, 33, 39, 46, 53, 61, 69,
    78, 86, 96, 106, 116, 126, 138, 149, 161, 173, 186, 199, 212, 226, 240, 255
};

// 64 Gamma Steps with 256 PWM Steps (from datasheet Table 16)
const uint8_t gamma_64_steps[64] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 14, 16, 18, 20, 22,
    24, 26, 29, 32, 35, 38, 41, 44, 47, 50, 53, 57, 61, 65, 69, 73,
    77, 81, 85, 89, 94, 99, 104, 109, 114, 119, 124, 129, 134, 140, 146, 152,
    158, 164, 170, 176, 182, 188, 195, 202, 209, 216, 223, 230, 237, 244, 251, 255
};

/* ========================================================================== */
/* Color Utility Functions Implementation */
/* ========================================================================== */

Backlight_RGB_t Color_Scale(Backlight_RGB_t color, uint8_t brightness_percent)
{
    if (brightness_percent > 100) brightness_percent = 100;

    Backlight_RGB_t scaled = {
        .r = (color.r * brightness_percent) / 100,
        .g = (color.g * brightness_percent) / 100,
        .b = (color.b * brightness_percent) / 100
    };
    return scaled;
}

Backlight_RGB_t Color_GetBreathing(uint8_t index)
{
    if (breathing_color_count == 0) {
        LOG_ERROR("Color_GetBreathing: breathing_color_count is zero");
        return COLOR_WARM_WHITE;  // Safe fallback
    }
    return breathing_colors[index % breathing_color_count];
}

Backlight_RGB_t Color_GetNotification(uint8_t index)
{
    if (notification_color_count == 0) {
        LOG_ERROR("Color_GetNotification: notification_color_count is zero");
        return COLOR_RED;  // Safe fallback
    }
    return notification_colors[index % notification_color_count];
}

Backlight_RGB_t Color_GetTransition(uint8_t index)
{
    return transition_colors[index % transition_color_count];
}

Backlight_RGB_t Color_Dim(Backlight_RGB_t color)
{
    return Color_Scale(color, 10);
}

Backlight_RGB_t Color_Bright(Backlight_RGB_t color)
{
    return Color_Scale(color, 75);
}

Backlight_RGB_t Color_Mix(Backlight_RGB_t color1, Backlight_RGB_t color2)
{
    Backlight_RGB_t mixed = {
        .r = (color1.r + color2.r) / 2,
        .g = (color1.g + color2.g) / 2,
        .b = (color1.b + color2.b) / 2
    };
    return mixed;
}

/* ========================================================================== */
/* Gamma Correction Functions Implementation */
/* ========================================================================== */

uint8_t Color_ApplyGamma32(uint8_t linear_value)
{
    if (linear_value >= 32) linear_value = 31;
    return gamma_32_steps[linear_value];
}

uint8_t Color_ApplyGamma64(uint8_t linear_value)
{
    if (linear_value >= 64) linear_value = 63;
    return gamma_64_steps[linear_value];
}

Backlight_RGB_t Color_ApplyGammaRGB32(Backlight_RGB_t linear_color)
{
    Backlight_RGB_t gamma_color = {
        .r = Color_ApplyGamma32(linear_color.r),
        .g = Color_ApplyGamma32(linear_color.g),
        .b = Color_ApplyGamma32(linear_color.b)
    };

    // Scale down to our 0-80 range
    gamma_color.r = (gamma_color.r * COLOR_MAX_COMPONENT) / 255;
    gamma_color.g = (gamma_color.g * COLOR_MAX_COMPONENT) / 255;
    gamma_color.b = (gamma_color.b * COLOR_MAX_COMPONENT) / 255;

    return gamma_color;
}

uint8_t Color_BrightnessToGamma(uint8_t brightness_percent, bool use_64_steps)
{
    if (brightness_percent > 100) brightness_percent = 100;

    uint8_t gamma_value;
    if (use_64_steps) {
        uint8_t linear_step = (brightness_percent * 63) / 100;
        if (linear_step >= 64) linear_step = 63;
        gamma_value = Color_ApplyGamma64(linear_step);
    } else {
        uint8_t linear_step = (brightness_percent * 31) / 100;
        if (linear_step >= 32) linear_step = 31;
        gamma_value = Color_ApplyGamma32(linear_step);
    }

    // Scale to our 0-80 range
    return (gamma_value * COLOR_MAX_COMPONENT) / 255;
}

Backlight_RGB_t Color_GetBreathingGamma(uint8_t color_index, uint8_t brightness_step)
{
    Backlight_RGB_t base_color = Color_GetBreathing(color_index);

    // Apply gamma correction to brightness
    uint8_t gamma_brightness = Color_ApplyGamma32(brightness_step);

    // Scale base color by gamma-corrected brightness
    Backlight_RGB_t result = {
        .r = (base_color.r * gamma_brightness) / 255,
        .g = (base_color.g * gamma_brightness) / 255,
        .b = (base_color.b * gamma_brightness) / 255
    };

    return result;
}
