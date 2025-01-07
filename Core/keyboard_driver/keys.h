/*
 * keys.h
 *
 * Created on: Dec 7, 2024
 * Author: bettysidepiece
 */

#ifndef KEYBOARD_DRIVER_KEYS_H_
#define KEYBOARD_DRIVER_KEYS_H_

#include <stdint.h>
#include <stdbool.h>
#include "keycodes.h"

// Matrix dimensions
#define KEY_ROWS 6
#define KEY_COLS 14
#define KEY_LAYERS 4

// NKRO Report Size
#define NKRO_SIZE 6
#define NKRO_EXT_SIZE 8

#define DEBOUNCE_TIME_US 650

typedef enum {
    KEY_IDLE,
    KEY_DEBOUNCE,
    KEY_PRESSED,
    KEY_HELD,
    KEY_RELEASED
} keystate_t;


typedef struct {
    uint8_t modifiers;
    uint8_t reserved;
    uint8_t keys[NKRO_SIZE];
    uint8_t ext_key[NKRO_EXT_SIZE];
} hid_report_t;

typedef struct {
    uint8_t report_id;   // BLE-specific report ID
    hid_report_t report;  // The actual NKRO report data
} ble_hid_report_t;

typedef enum {
	BASE_LAYER,
	FN_LAYER,
	MARCO_LAYER,
	CUSTOM_LAYER
} key_layers_t;

// Output modes
typedef enum {
    OUTPUT_USB,
    OUTPUT_BLE,
} output_mode_t;

// Last report timestamps
typedef struct {
    uint32_t ble_last_report;
    uint32_t usb_last_report;
} last_report_t;

// Keyboard state
typedef struct {
    bool fn_pressed;
    bool swap_active;
    uint8_t key_count;
    uint8_t current_layer;
    output_mode_t output_mode;
    uint8_t connection_mode;
    last_report_t last_report;
    hid_report_t current_report;
    uint8_t KRO_mode;
} keyboard_state_t;


// Layer definition
typedef uint16_t keymap_t[KEY_ROWS][KEY_COLS];
typedef keystate_t (*StateHandler)(uint8_t row, uint8_t col, bool col_pressed, uint32_t current_time);

// Global State
extern keyboard_state_t kb_state;
extern keystate_t key_states[KEY_ROWS][KEY_COLS];
extern keymap_t layers[KEY_LAYERS];
extern ble_hid_report_t ble_report;

// Function Prototypes
keystate_t handleIdle(uint8_t row, uint8_t col, bool col_pressed, uint32_t current_time);
keystate_t handleDebounce(uint8_t row, uint8_t col, bool col_pressed, uint32_t current_time);
keystate_t handlePressed(uint8_t row, uint8_t col, bool col_pressed, uint32_t current_time);
keystate_t handleHeld(uint8_t row, uint8_t col, bool col_pressed, uint32_t current_time);
keystate_t handleReleased(uint8_t row, uint8_t col, bool col_pressed, uint32_t current_time);
void loadKeymap(uint8_t layer, keymap_t *keymap);
void initKeyboard(void);
void processKey(uint8_t row, uint8_t col, bool pressed);
uint8_t getCurrentLayer(void);
uint8_t getModifierBit(uint8_t keycode);
bool isModifier(uint8_t keycode);
void clearReport(void);
void resetKeyboardState(void);

#endif /* INC_KEYS_H_ */
