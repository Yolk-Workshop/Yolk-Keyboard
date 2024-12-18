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
uint8_t key_states[KEY_ROWS][KEY_COLS] = {0};
uint8_t debounce_buffers[KEY_ROWS][KEY_COLS] = {0};
keymap_t layers[KEY_LAYERS] = {0};

// Keymaps
keymap_t base_keymap = {
    {KC_ESCAPE, KC_F1,    KC_F2,    KC_F3,    KC_F4,    KC_F5,    KC_F6,    KC_F7,    KC_F8,    KC_F9,    KC_F10,   KC_F11,   KC_F12,   KC_DELETE},
    {KC_GRAVE,  KC_1,     KC_2,     KC_3,     KC_4,     KC_5,     KC_6,     KC_7,     KC_8,     KC_9,     KC_0,     KC_MINUS, KC_EQUAL, KC_BSPACE},
    {KC_TAB,    KC_Q,     KC_W,     KC_E,     KC_R,     KC_T,     KC_Y,     KC_U,     KC_I,     KC_O,     KC_P,     KC_LBRACKET, KC_RBRACKET, KC_ENTER},
    {KC_CAPS,   KC_A,     KC_S,     KC_D,     KC_F,     KC_G,     KC_H,     KC_J,     KC_K,     KC_L,     KC_SCOLON, KC_QUOTE, KC_NONUS_BSLASH, KC_SWAP},
    {KC_LSHIFT, KC_NONUS_HASH, KC_Z, KC_X,    KC_C,     KC_V,     KC_B,     KC_N,     KC_M,     KC_COMMA, KC_DOT,   KC_SLASH, KC_UP,    KC_RSHIFT},
    {KC_FN,     KC_LCTRL, KC_LGUI,  KC_LALT,  KC_SPACE, KC_RALT,  KC_RCTRL, KC_LEFT,  KC_DOWN,  KC_RIGHT, KC_NO,    KC_NO,    KC_NO,    KC_NO}
};

keymap_t fn_keymap = {
    {KC_LOWER_BRIGHT, KC_INCR_BRIGHT, KC_APPLICATION, KC_FIND, KC_MIC_TOGGLE, KC_BACKLIGHT_DIM, KC_BACKLIGHT_INCR,
     KC_PREV, KC_PLAY, KC_NEXT, KC_VOLDOWN, KC_VOLUP, KC_LOCK}
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
    switch (keycode) {
        case KC_LCTRL: case KC_LSHIFT: case KC_LALT: case KC_LGUI:
        case KC_RCTRL: case KC_RSHIFT: case KC_RALT: case KC_RGUI:
            return true;
        default:
            return false;
    }
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
	memset(&kb_state.current_report, 0, sizeof(hid_report_t));
	kb_state.current_report.modifiers = 0;
    memset(kb_state.current_report.keys, 0, sizeof(kb_state.current_report.keys));
    kb_state.key_count = 0;
}

void resetKeyboardState(void) {
    kb_state.fn_pressed = false;
    kb_state.swap_active = false;
}

static void handleKeyPress(uint8_t keycode) {
    if (kb_state.key_count < 6) {
        // Add key to 6KRO buffer
        kb_state.current_report.keys[kb_state.key_count++] = keycode;
        LOG_DEBUG("6KRO-Press KC:0x%02X IDX:%u", keycode, kb_state.key_count - 1);
    } else {
        // Handle NKRO
        uint8_t byte = keycode / 8;
        uint8_t bit = keycode % 8;
        if (byte < NKRO_EXT_SIZE) {
            kb_state.current_report.ext_key[byte] |= (1 << bit);
            LOG_DEBUG("NKRO-Press Bitset: %u Byte:%u KC:0x%02X", bit, byte, keycode);
        }
    }
}

static void handleKeyRelease(uint8_t keycode) {
    bool found = false;
    // Remove key from 6KRO buffer
    for (uint8_t i = 0; i < kb_state.key_count; i++) {
        if (kb_state.current_report.keys[i] == keycode) {
            found = true;
            // Shift keys in the buffer
            for (uint8_t j = i; j < kb_state.key_count - 1; j++) {
                kb_state.current_report.keys[j] = kb_state.current_report.keys[j + 1];
            }
            kb_state.current_report.keys[--kb_state.key_count] = 0;
            LOG_DEBUG("6KRO-Release Cleared KC:0x%02X Buffer: [%u, %u, %u, %u, %u, %u]",
                      keycode,
                      kb_state.current_report.keys[0], kb_state.current_report.keys[1],
                      kb_state.current_report.keys[2], kb_state.current_report.keys[3],
                      kb_state.current_report.keys[4], kb_state.current_report.keys[5]);
            break;
        }
    }

    // If not found in 6KRO, clear NKRO bit
    if (!found) {
        uint8_t byte = keycode / 8;
        uint8_t bit = keycode % 8;
        if (byte < NKRO_EXT_SIZE) {
            kb_state.current_report.ext_key[byte] &= ~(1 << bit);
            LOG_DEBUG("NKRO-Release Bitclr: %u Byte:%u KC:0x%02X", bit, byte, keycode);
        }
    }
}

void processKey(uint8_t row, uint8_t col, bool pressed) {
    if (row >= KEY_ROWS || col >= KEY_COLS) return;

    // Get keycode from the active layer
    uint8_t keycode = layers[getCurrentLayer()][row][col];
    if (keycode == KC_NO) return;  // Ignore undefined keys

    LOG_DEBUG("KB:[row:%u][col:%u] KC:0x%02X", row, col, keycode);

    // Update key state
    key_states[row][col] = pressed ? 1 : 0;

    // Handle special keys
    if (keycode == KC_FN) {
        kb_state.fn_pressed = pressed;
        LOG_DEBUG("FN key %s", pressed ? "pressed" : "released");
        return;
    }
    if (keycode == KC_SWAP) {
        kb_state.swap_active = pressed;
        LOG_DEBUG("SWAP key %s", pressed ? "pressed" : "released");
        return;
    }

    // Handle modifiers
    if (isModifier(keycode)) {
        if (pressed) {
            kb_state.current_report.modifiers |= getModifierBit(keycode);
            LOG_DEBUG("MOD Press KC:0x%02X", keycode);
        } else {
            kb_state.current_report.modifiers &= ~getModifierBit(keycode);
            LOG_DEBUG("MOD Release KC:0x%02X", keycode);
        }
        return;
    }

    // Process regular keys
    if (pressed) {
        handleKeyPress(keycode);
    } else {
        handleKeyRelease(keycode);
    }

    // Mark the report as ready
    report_ready_flag = 1;
}


void debounceKey(uint8_t row, uint8_t col, bool new_state) {

    // Shift the new sample into the buffer
    debounce_buffers[row][col] <<= 1;
    debounce_buffers[row][col] |= (new_state ? 1 : 0);

    // Check if the buffer is stable (all 1s or all 0s)
    if ((debounce_buffers[row][col] == DEBOUNCE_MASK) && !key_states[row][col]) {
        key_states[row][col] = 1;
        processKey(row, col, true);
        LOG_DEBUG("Key [%u][%u] %s", row, col, true ? "PRESSED" : "RELEASED");
    }
    else if ((debounce_buffers[row][col] == 0x00) && key_states[row][col]) {
        key_states[row][col] = 0;
        processKey(row, col, false);
        LOG_DEBUG("Key [%u][%u] %s", row, col, false ? "PRESSED" : "RELEASED");
    }
}


void processKeyBatch(void) {
    for (uint8_t row = 0; row < KEY_ROWS; row++) {
        for (uint8_t col = 0; col < KEY_COLS; col++) {
            if (key_states[row][col]) {
                processKey(row, col, true);
            } else {
                processKey(row, col, false);
            }
        }
    }
    report_ready_flag = 1;  // Batch processing complete
}
