/*
 * led_mapping.h
 *
 *  Created on: Jun 7, 2025
 *      Author: bettysidepiece
 */

#ifndef BACKLIGHT_CORE_LED_MAPPING_H_
#define BACKLIGHT_CORE_LED_MAPPING_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef struct {
    uint8_t ic;          // 0 = IC1, 1 = IC2
    uint8_t led_row;     // LED matrix row (1-11 for IC1, 1-4 for IC2)
    uint8_t red_col;     // Red channel column
    uint8_t green_col;   // Green channel column
    uint8_t blue_col;    // Blue channel column
    uint8_t key_row;     // Keyboard matrix row
    uint8_t key_col;     // Keyboard matrix column
} rgb_led_mapping_t;


extern const size_t RGB_LED_MAP_SIZE;
extern const rgb_led_mapping_t RGB_LED_MAP[];
const rgb_led_mapping_t* get_led_for_key(uint8_t row, uint8_t col);
bool get_led_index_for_key(uint8_t row, uint8_t col, uint8_t *led_index);
bool get_key_for_led_index(uint8_t led_index, uint8_t *row, uint8_t *col);

#endif /* BACKLIGHT_CORE_LED_MAPPING_H_ */
