/*
 * led_mapping.c
 *
 *  Created on: Jun 7, 2025
 *      Author: bettysidepiece
 */
#include "led_mapping.h"
#include "logger.h"


// [ic],[led_row],[red_col],[green_col],[blue_col],[key_row],[key_col]
const rgb_led_mapping_t RGB_LED_MAP[] = {
    // IC1 Coverage - 66 RGB LEDs
    // Row 0: F-key row (14 keys)
	// [ic],[led_row],[red_col],[green_col],[blue_col],[key_row],[key_col]
    {0, 11, 18, 16, 17, 0, 0},     // ESC
    {0, 10, 18, 16, 17, 0, 1},     // F1
    {0, 9, 18, 16, 17, 0, 2},      // F2
    {0, 8, 18, 16, 17, 0, 3},      // F3
    {0, 7, 18, 16, 17, 0, 4},      // F4
    {0, 6, 18, 16, 17, 0, 5},      // F5
    {0, 5, 18, 16, 17, 0, 6},      // F6
    {0, 4, 18, 16, 17, 0, 7},      // F7
    {0, 3, 18, 16, 17, 0, 8},      // F8
    {0, 2, 18, 16, 17, 0, 9},      // F9
    {0, 1, 18, 16, 17, 0, 10},     // F10
    {0, 11, 15, 13, 14, 0, 11},    // F11
    {0, 10, 15, 13, 14, 0, 12},    // F12
    {0, 9, 15, 13, 14, 0, 13},     // DEL

    // Row 1: Number row (14 keys)
	// [ic],[led_row],[red_col],[green_col],[blue_col],[key_row],[key_col]
    {0, 8, 15, 13, 14, 1, 0},      // GRAVE
    {0, 7, 15, 13, 14, 1, 1},      // 1
    {0, 6, 15, 13, 14, 1, 2},      // 2
    {0, 5, 15, 13, 14, 1, 3},      // 3
    {0, 4, 15, 13, 14, 1, 4},      // 4
    {0, 3, 15, 13, 14, 1, 5},      // 5
    {0, 2, 15, 13, 14, 1, 6},      // 6
    {0, 1, 15, 13, 14, 1, 7},      // 7
    {0, 11, 12, 10, 11, 1, 8},     // 8
    {0, 10, 12, 10, 11, 1, 9},     // 9
    {0, 9, 12, 10, 11, 1, 10},     // 0
    {0, 8, 12, 10, 11, 1, 11},     // MINUS
    {0, 7, 12, 10, 11, 1, 12},     // EQUAL
    {0, 6, 12, 10, 11, 1, 13},     // BSPC

    // Row 2: QWERTY row (14 keys)
	// [ic],[led_row],[red_col],[green_col],[blue_col],[key_row],[key_col]
    {0, 4, 9, 7, 8, 2, 0},      // TAB
    {0, 5, 9, 7, 8, 2, 1},      // Q
    {0, 6, 9, 7, 8, 2, 2},      // W
    {0, 7, 9, 7, 8, 2, 3},      // E
    {0, 8, 9, 7, 8, 2, 4},      // R
    {0, 9, 9, 7, 8, 2, 5},        // T
    {0, 10, 9, 7, 8, 2, 6},        // Y
    {0, 11, 9, 7, 8, 2, 7},         // U
    {0, 1, 12, 10, 11, 2, 8},         // I
    {0, 2, 12, 10, 11, 2, 9},         // O
    {0, 3, 12, 10, 11, 2, 10},        // P
    {0, 4, 12, 10, 11, 2, 11},        // LBRACKET
    {0, 5, 12, 10, 11, 2, 12},        // RBRACKET
    {0, 1, 6, 4, 5, 2, 13},        // ENTER

    // Row 3: ASDF row (13 keys - IC1)
	// [ic],[led_row],[red_col],[green_col],[blue_col],[key_row],[key_col]
    {0, 3, 9, 7, 8, 3, 0},         // CAPS
    {0, 2, 9, 7, 8, 3, 1},         // A
    {0, 1, 9, 7, 8, 3, 2},        // S
    {0, 11, 6, 4, 5, 3, 3},        // D
    {0, 10, 6, 4, 5, 3, 4},         // F
    {0, 9, 6, 4, 5, 3, 5},         // G
    {0, 8, 6, 4, 5, 3, 6},         // H
    {0, 7, 6, 4, 5, 3, 7},         // J
    {0, 6, 6, 4, 5, 3, 8},         // K
    {0, 5, 6, 4, 5, 3, 9},         // L
    {0, 4, 6, 4, 5, 3, 10},        // SCOLON
    {0, 3, 6, 4, 5, 3, 11},        // QUOTE
    {0, 2, 6, 4, 5, 3, 12},        // HASH

    // Row 4: ZXCV row (12 keys - IC1)
	// [ic],[led_row],[red_col],[green_col],[blue_col],[key_row],[key_col]
    {0, 11, 3, 1, 2, 4, 0},        // LSHIFT
    {0, 10, 3, 1, 2, 4, 1},        // BSLASH
    {0, 9, 3, 1, 2, 4, 2},         // Z
    {0, 8, 3, 1, 2, 4, 3},         // X
    {0, 7, 3, 1, 2, 4, 4},         // C
    {0, 6, 3, 1, 2, 4, 5},         // V
    {0, 5, 3, 1, 2, 4, 6},         // B
    {0, 4, 3, 1, 2, 4, 7},         // N
    {0, 3, 3, 1, 2, 4, 8},         // M
    {0, 2, 3, 1, 2, 4, 9},         // COMMA
    {0, 1, 3, 1, 2, 4, 10},        // DOT

    // IC2 Coverage - 16 RGB LEDs
	// [ic],[led_row],[red_col],[green_col],[blue_col],[key_row],[key_col]
    {1, 1, 12, 10, 11, 3, 13},     // SWAP
    {1, 2, 12, 10, 11, 4, 12},     // UP
    {1, 3, 12, 10, 11, 4, 13},     // RSHIFT
    {1, 4, 12, 10, 11, 4, 11},     // SLASH
    {1, 4, 9, 7, 8, 5, 0},         // FN
    {1, 3, 9, 7, 8, 5, 1},         // LCTRL
    {1, 2, 9, 7, 8, 5, 2},         // LGUI
    {1, 1, 9, 7, 8, 5, 3},         // LALT
    {1, 4, 6, 4, 5, 5, 4},         // SPACE
    {1, 3, 6, 4, 5, 5, 5},         // RALT
    {1, 2, 6, 4, 5, 5, 6},         // RCTRL
    {1, 1, 6, 4, 5, 5, 7},         // LEFT
    {1, 4, 3, 1, 2, 5, 8},         // DOWN
    {1, 3, 3, 1, 2, 5, 9},         // RIGHT
};

const size_t RGB_LED_MAP_SIZE = sizeof(RGB_LED_MAP) / sizeof(rgb_led_mapping_t);

const rgb_led_mapping_t* get_led_for_key(uint8_t row, uint8_t col) {
    for (int i = 0; i < RGB_LED_MAP_SIZE; i++) {
        if (RGB_LED_MAP[i].key_row == row && RGB_LED_MAP[i].key_col == col) {
            return &RGB_LED_MAP[i];
        }
    }
    return NULL;  // Key has no LED
}

bool get_key_for_led_index(uint8_t led_index, uint8_t *row, uint8_t *col)
{
    if (!row || !col || led_index >= RGB_LED_MAP_SIZE) {
        return false;
    }

    *row = RGB_LED_MAP[led_index].key_row;
    *col = RGB_LED_MAP[led_index].key_col;
    return true;
}

bool get_led_index_for_key(uint8_t row, uint8_t col, uint8_t *led_index)
{
    if (!led_index) return false;

    // Search through the array to find the matching key
    for (size_t i = 0; i < RGB_LED_MAP_SIZE; i++) {
        if (RGB_LED_MAP[i].key_row == row && RGB_LED_MAP[i].key_col == col) {
            *led_index = (uint8_t)i;  // Array index is the LED index
            return true;
        }
    }
    return false;
}
