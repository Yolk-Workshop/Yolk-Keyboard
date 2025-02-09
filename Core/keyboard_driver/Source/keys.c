/*
 * keys.c
 *
 * Created on: Dec 7, 2024
 * Author: bettysidepiece
 */

#include "keys.h"
#include "logger.h"
#include <string.h>

// Global state
ble_hid_report_t ble_report;
keyboard_state_t kb_state = {0};
keystate_t key_states[KEY_ROWS][KEY_COLS] = {0};
static uint32_t last_change_time[KEY_ROWS][KEY_COLS] = {0}; // Initialize to 0

//uint8_t debounce_buffers[KEY_ROWS][KEY_COLS] = {0};
keymap_t layers[KEY_LAYERS] = {0};

// Keymaps
keymap_t base_keymap = {
    {KC_ESCAPE, KC_F1,    KC_F2,    KC_F3,    KC_F4,    KC_F5,    KC_F6,    KC_F7,    KC_F8,    KC_F9,    KC_F10,   KC_F11,   KC_F12,   KC_DELETE},
    {KC_GRAVE,  KC_1,     KC_2,     KC_3,     KC_4,     KC_5,     KC_6,     KC_7,     KC_8,     KC_9,     KC_0,     KC_MINUS, KC_EQUAL, KC_BSPACE},
    {KC_TAB,    KC_Q,     KC_W,     KC_E,     KC_R,     KC_T,     KC_Y,     KC_U,     KC_I,     KC_O,     KC_P,     KC_LBRACKET, KC_RBRACKET, KC_ENTER},
    {KC_CAPS,   KC_A,     KC_S,     KC_D,     KC_F,     KC_G,     KC_H,     KC_J,     KC_K,     KC_L,     KC_SCOLON, KC_QUOTE, KC_NONUS_HASH, KC_SWAP},
    {KC_LSHIFT, KC_NONUS_BSLASH, KC_Z, KC_X,    KC_C,     KC_V,     KC_B,     KC_N,     KC_M,     KC_COMMA, KC_DOT,   KC_SLASH, KC_UP,    KC_RSHIFT},
    {KC_FN,     KC_LCTRL, KC_LGUI,  KC_LALT,  KC_SPACE, KC_RALT,  KC_RCTRL, KC_LEFT,  KC_DOWN,  KC_RIGHT, KC_NO,    KC_NO,    KC_NO,    KC_NO}
};

keymap_t fn_keymap = {
    {KC_LOWER_BRIGHT, KC_INCR_BRIGHT, KC_APPLICATION, KC_FIND, KC_MIC_TOGGLE, KC_BACKLIGHT_DIM, KC_BACKLIGHT_INCR,
     KC_PREV, KC_PLAY, KC_NEXT, KC_VOLDOWN, KC_VOLUP, KC_LOCK}
};

StateHandler stateMachine[5] = {
    [KEY_IDLE]      = handleIdle,
    [KEY_DEBOUNCE]  = handleDebounce,
    [KEY_PRESSED]   = handlePressed,
    [KEY_HELD]      = handleHeld,
    [KEY_RELEASED]  = handleReleased
};

extern volatile uint8_t report_ready_flag;

void initKeyboard(void) {
    memset(&kb_state, 0, sizeof(keyboard_state_t));
    memset(key_states, 0, sizeof(key_states));

    // Initialize keymaps
    kb_state.current_layer = BASE_LAYER;
    kb_state.key_count = 0;
    loadKeymap(BASE_LAYER, &base_keymap);
    loadKeymap(FN_LAYER, &fn_keymap);
}

void loadKeymap(uint8_t layer, keymap_t *keymap) {
    if (layer < KEY_LAYERS) {
        memcpy(&layers[layer], keymap, sizeof(keymap_t));
    }
}

bool isModifier(uint8_t keycode) {
    return keycode >= KC_LCTRL && keycode <= KC_RGUI;
}

uint8_t getModifierBit(uint8_t keycode) {
    switch (keycode) {
        case KC_LCTRL:  return 0x01;
        case KC_LSHIFT: return 0x02;
        case KC_LALT:   return 0x04;
        case KC_LGUI:   return 0x08;
        case KC_RCTRL:  return 0x10;
        case KC_RSHIFT: return 0x20;
        case KC_RALT:   return 0x40;
        case KC_RGUI:   return 0x80;
        default:        return 0;
    }
}

uint8_t getCurrentLayer(void) {
    if (kb_state.fn_pressed) {
        return FN_LAYER;
    }
    if (kb_state.swap_active) {
        return CUSTOM_LAYER;
    }
    return BASE_LAYER;
}

void clearReport(void) {
	kb_state.fn_pressed = false;
	kb_state.swap_active = false;
	kb_state.current_layer = 0;
}

void resetKeyboardState(void) {
    kb_state.fn_pressed = false;
    kb_state.swap_active = false;
}

static void handleKeyPress(uint8_t keycode) {
    if (kb_state.key_count < 6) {
        // Add key to 6KRO buffer
        kb_state.current_report.keys[kb_state.key_count++] = keycode;
        //LOG_DEBUG("6KRO-Press KC:0x%02X IDX:%u", keycode, kb_state.key_count - 1);
    } else {
        // Handle NKRO
        uint8_t byte = keycode / 8;
        uint8_t bit = keycode % 8;
        if (byte < NKRO_EXT_SIZE) {
            kb_state.current_report.ext_key[byte] |= (1 << bit);
            //LOG_DEBUG("NKRO-Press Bitset: %u Byte:%u KC:0x%02X", bit, byte, keycode);
        }
    }
}


static void handleKeyRelease(uint8_t keycode) {
    bool found = false;

    for (uint8_t i = 0; i < kb_state.key_count; i++) {
        // Compare keycode with current buffer entry
        if (kb_state.current_report.keys[i] == keycode) {
            found = true;
            // Shift the remaining keys to fill the gap
            for (uint8_t j = i; j < kb_state.key_count - 1; j++) {
                kb_state.current_report.keys[j] = kb_state.current_report.keys[j + 1];
            }
            // Decrease key count and clear the last position
            kb_state.key_count--;
            kb_state.current_report.keys[kb_state.key_count] = 0;
            break;
        }
    }

    // If not found in the 6KRO buffer, clear the NKRO bit
    if (!found) {
        uint8_t byte = keycode >> 3; // Divide by 8
        uint8_t bit = keycode & 0x07; // Modulo 8
        if (byte < NKRO_EXT_SIZE) {
            kb_state.current_report.ext_key[byte] &= ~(1 << bit);
        }
    }
}

static void handleModifiers(uint8_t keycode, bool pressed) {
    if (pressed) {
        kb_state.current_report.modifiers |= getModifierBit(keycode);
        //LOG_DEBUG("Modifier Pressed: KC:0x%02X", keycode);
    } else {
        kb_state.current_report.modifiers &= ~getModifierBit(keycode);
        //LOG_DEBUG("Modifier Released: KC:0x%02X", keycode);
    }

    report_ready_flag = 1;
}

static void handleNormalKeys(uint8_t keycode, bool pressed) {
    if (pressed) {
        handleKeyPress(keycode);
    } else {
        handleKeyRelease(keycode);
    }
}

void processKey(uint8_t row, uint8_t col, bool pressed) {
    if (row >= KEY_ROWS || col >= KEY_COLS) return;

    uint8_t keycode = layers[getCurrentLayer()][row][col];
    if (keycode == KC_NO) return;

    //LOG_DEBUG("KB:[%u][%u]-KC:0x%02X", row, col, keycode);

    if (isModifier(keycode)) {
        handleModifiers(keycode, pressed);
        return;
    } else {
        handleNormalKeys(keycode, pressed);
    }

    report_ready_flag = 1;
}

keystate_t handleIdle(uint8_t row, uint8_t col, bool col_pressed, uint32_t current_time) {
    if (col_pressed) {
        //LOG_DEBUG("Key [%u][%u] DEBOUNCED", row, col);
        last_change_time[row][col] = current_time;
        return KEY_DEBOUNCE;
    }
    return KEY_IDLE;
}

keystate_t handleDebounce(uint8_t row, uint8_t col, bool col_pressed, uint32_t current_time) {
    if (col_pressed && (current_time - last_change_time[row][col] > DEBOUNCE_TIME_US)) {
        //LOG_DEBUG("Key [%u][%u] PRESSED", row, col);
        processKey(row, col, true); // Key pressed
        return KEY_PRESSED;
    }
    if (!col_pressed) {
        return KEY_IDLE;
    }
    return KEY_DEBOUNCE;
}

keystate_t handlePressed(uint8_t row, uint8_t col, bool col_pressed, uint32_t current_time) {
    if (!col_pressed) {
        //LOG_DEBUG("Key [%u][%u] RELEASED", row, col);
        processKey(row, col, false); // Key released
        return KEY_RELEASED;
    }
    return KEY_HELD;
}

keystate_t handleHeld(uint8_t row, uint8_t col, bool col_pressed, uint32_t current_time) {
    if (!col_pressed) {
        //LOG_DEBUG("Key [%u][%u] RELEASED", row, col);
        processKey(row, col, false); // Key released
        return KEY_RELEASED;
    }
    return KEY_HELD;
}

keystate_t handleReleased(uint8_t row, uint8_t col, bool col_pressed, uint32_t current_time) {
    return KEY_IDLE;
}

