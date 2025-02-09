#ifndef BACKLIGHT_DRIVER_H
#define BACKLIGHT_DRIVER_H

#include <stdbool.h>
#include <stdint.h>
#include "is31flxx.h"
// LED states
typedef struct {
    bool valid;          // LED is not shorted
    uint8_t pwm_value;   // Current PWM value
} led_status_t;

typedef struct {
    led_status_t matrix[12][12];
    uint8_t gcc_value;
} backlight_state_t;

extern i2c_device i2c1_inst;

// Function declarations
bool init_backlight(void);
bool set_backlight_brightness(uint8_t brightness);
bool set_led_brightness(uint8_t row, uint8_t col, uint8_t brightness);
void update_pulse_effect(void);
bool getLEDmatrixState(led_matrix_state *open_matrix,
		led_matrix_state *short_matrix);
bool checkLEDshortstate(void);

#endif
