/*
 * keycode.h
 *
 * Created on: Dec 15, 2024
 * Author: bettysidepiece
 *
 * Based on USB HID Usage Tables v1.12 (October 28, 2004)
 */

#ifndef KEYBOARD_DRIVER_KEYCODES_H_
#define KEYBOARD_DRIVER_KEYCODES_H_

typedef enum {
    // Reserved (0x00-0x03)
    KC_NO = 0x00,           // No event indicated
    KC_ERR_ROLL = 0x01,     // ErrorRollOver
    KC_POST_FAIL = 0x02,    // POSTFail
    KC_ERR_UNDEFINED = 0x03,// ErrorUndefined

    // Letters (0x04-0x1D)
    KC_A = 0x04,
    KC_B = 0x05,
    KC_C = 0x06,
    KC_D = 0x07,
    KC_E = 0x08,
    KC_F = 0x09,
    KC_G = 0x0A,
    KC_H = 0x0B,
    KC_I = 0x0C,
    KC_J = 0x0D,
    KC_K = 0x0E,
    KC_L = 0x0F,
    KC_M = 0x10,
    KC_N = 0x11,
    KC_O = 0x12,
    KC_P = 0x13,
    KC_Q = 0x14,
    KC_R = 0x15,
    KC_S = 0x16,
    KC_T = 0x17,
    KC_U = 0x18,
    KC_V = 0x19,
    KC_W = 0x1A,
    KC_X = 0x1B,
    KC_Y = 0x1C,
    KC_Z = 0x1D,

    // Numbers (0x1E-0x27)
    KC_1 = 0x1E,     // 1 and !
    KC_2 = 0x1F,     // 2 and @
    KC_3 = 0x20,     // 3 and #
    KC_4 = 0x21,     // 4 and $
    KC_5 = 0x22,     // 5 and %
    KC_6 = 0x23,     // 6 and ^
    KC_7 = 0x24,     // 7 and &
    KC_8 = 0x25,     // 8 and *
    KC_9 = 0x26,     // 9 and (
    KC_0 = 0x27,     // 0 and )

    // Basic keys (0x28-0x38)
    KC_ENTER = 0x28,      // Return (ENTER)
    KC_ESCAPE = 0x29,     // ESCAPE
    KC_BSPACE = 0x2A,     // DELETE (Backspace)
    KC_TAB = 0x2B,        // Tab
    KC_SPACE = 0x2C,      // Spacebar
    KC_MINUS = 0x2D,      // - and _
    KC_EQUAL = 0x2E,      // = and +
    KC_LBRACKET = 0x2F,   // [ and {
    KC_RBRACKET = 0x30,   // ] and }
    KC_BSLASH = 0x31,     // \ and |
    KC_NONUS_HASH = 0x32, // Non-US # and ~
    KC_SCOLON = 0x33,     // ; and :
    KC_QUOTE = 0x34,      // ' and "
    KC_GRAVE = 0x35,      // Grave Accent and Tilde
    KC_COMMA = 0x36,      // , and <
    KC_DOT = 0x37,        // . and >
    KC_SLASH = 0x38,      // / and ?
    KC_CAPS = 0x39,       // Caps Lock

    // Function keys (0x3A-0x45)
    KC_F1 = 0x3A,
    KC_F2 = 0x3B,
    KC_F3 = 0x3C,
    KC_F4 = 0x3D,
    KC_F5 = 0x3E,
    KC_F6 = 0x3F,
    KC_F7 = 0x40,
    KC_F8 = 0x41,
    KC_F9 = 0x42,
    KC_F10 = 0x43,
    KC_F11 = 0x44,
    KC_F12 = 0x45,

    // System keys (0x46-0x52)
    KC_PSCREEN = 0x46,    // PrintScreen
    KC_SCROLLLOCK = 0x47, // Scroll Lock
    KC_PAUSE = 0x48,      // Pause
    KC_INSERT = 0x49,     // Insert
    KC_HOME = 0x4A,       // Home
    KC_PGUP = 0x4B,       // PageUp
    KC_DELETE = 0x4C,     // Delete Forward
    KC_END = 0x4D,        // End
    KC_PGDOWN = 0x4E,     // PageDown
    KC_RIGHT = 0x4F,      // RightArrow
    KC_LEFT = 0x50,       // LeftArrow
    KC_DOWN = 0x51,       // DownArrow
    KC_UP = 0x52,         // UpArrow

    // Additional keys (0x64-0x67)
    KC_NONUS_BSLASH = 0x64, // Non-US \ and |
    KC_APPLICATION = 0x65,   // Application (Menu)
    KC_POWER = 0x66,        // Power
    KC_KP_EQUAL = 0x67,     // Keypad =

    // System control (0x74-0x7F)
    KC_EXECUTE = 0x74,
    KC_HELP = 0x75,
    KC_MENU = 0x76,
    KC_SELECT = 0x77,
    KC_STOP = 0x78,
    KC_AGAIN = 0x79,
    KC_UNDO = 0x7A,
    KC_CUT = 0x7B,
    KC_COPY = 0x7C,
    KC_PASTE = 0x7D,
    KC_FIND = 0x7E,
    KC_MUTE = 0x7F,
    KC_VOLUP = 0x80,
    KC_VOLDOWN = 0x81,

    // Modifiers (0xE0-0xE7) - Note: these are reported in byte 0
    KC_LCTRL = 0xE0,   // Keyboard LeftControl
    KC_LSHIFT = 0xE1,  // Keyboard LeftShift
    KC_LALT = 0xE2,    // Keyboard LeftAlt
    KC_LGUI = 0xE3,    // Keyboard Left GUI
    KC_RCTRL = 0xE4,   // Keyboard RightControl
    KC_RSHIFT = 0xE5,  // Keyboard RightShift
    KC_RALT = 0xE6,    // Keyboard RightAlt
    KC_RGUI = 0xE7,    // Keyboard Right GUI

} keycodes_t;

// Custom keycodes start after USB HID range
typedef enum {
    CUSTOM_KEY_START = 0x100,     // Start of custom keycodes

    // Custom Function Layer Keys
    KC_FN = CUSTOM_KEY_START,     // Function modifier
    KC_SWAP,                      // Swap key function
    KC_LOWER_BRIGHT,             // Lower brightness
    KC_INCR_BRIGHT,              // Increase brightness
    KC_MIC_TOGGLE,               // Microphone toggle
    KC_BACKLIGHT_DIM,            // Backlight dim
    KC_BACKLIGHT_INCR,           // Backlight increase
	KC_PREV,
	KC_PLAY,
	KC_NEXT,
	KC_LOCK,

} custom_keycode_t;

#endif /* INC_KEYCODE_H_ */
