/*
 * keys.c
 *
 * Created on: Dec 7, 2024
 * Author: bettysidepiece
 */

#include "keys.h"
#include "pmsm.h"
#include "logger.h"
#include <string.h>
#include "effects.h"
#include "config_ui.h"

// Global state
ble_hid_report_t ble_report;
keyboard_state_t kb_state = { 0 };
keystate_t key_states[KEY_ROWS][KEY_COLS] = { 0 };
static uint32_t last_change_time[KEY_ROWS][KEY_COLS] = { 0 };
volatile bool g_usb_suspended = false;
extern volatile uint8_t report_ready_flag;

keymap_t layers[KEY_LAYERS] = { 0 };


//Add phantom filtering variables
static uint32_t last_any_key_time = 0;
static uint8_t last_key_row = 0xFF;
static uint8_t last_key_col = 0xFF;
static uint32_t phantom_count = 0;
static uint32_t valid_key_count = 0;

static uint32_t fn_p_press_start_time = 0;
static bool fn_p_hold_active = false;
static bool fn_p_pairing_triggered = false;
static uint32_t last_device_switch_time = 0;
static bool device_switch_cooldown = false;

static uint32_t last_column_time[KEY_COLS] = {0};
static bool isPhantomKey(uint8_t row, uint8_t col, uint32_t current_time, bool pressed);
static void handleKeyPress(uint8_t keycode);
static void handleKeyRelease(uint8_t keycode);

// Keymaps
keymap_t base_keymap = {

		{ KC_ESCAPE, KC_F1, KC_F2, KC_F3, KC_F4, KC_F5, KC_F6,
		KC_F7, KC_F8, KC_F9, KC_F10, KC_F11, KC_F12, KC_DELETE },

		{ KC_GRAVE, KC_1, KC_2, KC_3, KC_4, KC_5, KC_6, KC_7,
		KC_8, KC_9, KC_0, KC_MINUS, KC_EQUAL, KC_BSPACE },

		{ KC_TAB, KC_Q, KC_W, KC_E, KC_R, KC_T, KC_Y,
		KC_U, KC_I, KC_O, KC_P, KC_LBRACKET, KC_RBRACKET, KC_ENTER },

		{ KC_CAPS, KC_A, KC_S, KC_D, KC_F, KC_G, KC_H, KC_J, KC_K,
		KC_L, KC_SCOLON, KC_QUOTE, KC_NONUS_HASH, KC_SWAP },

		{ KC_LSHIFT, KC_NONUS_BSLASH, KC_Z, KC_X, KC_C, KC_V, KC_B,
		KC_N, KC_M, KC_COMMA, KC_DOT, KC_SLASH, KC_UP, KC_RSHIFT },

		{ KC_FN, KC_LCTRL, KC_LGUI, KC_LALT, KC_SPACE, KC_RALT,
		KC_RCTRL, KC_LEFT, KC_DOWN, KC_RIGHT, KC_NO, KC_NO, KC_NO, KC_NO }
	};

keymap_t fn_keymap = {
    // Row 0: F1-F12, Delete row
    { KC_NO, KC_LOWER_BRIGHT, KC_INCR_BRIGHT, KC_APPLICATION,
      KC_FIND, KC_MIC_TOGGLE, KC_BACKLIGHT_DIM, KC_BACKLIGHT_INCR, KC_PREV,
      KC_PLAY, KC_NEXT, KC_VOLDOWN, KC_VOLUP, KC_LOCK },

    // Row 1: Number row (1-9, 0, -, =, Backspace)
    { KC_NO, KC_1, KC_2, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
      KC_NO, KC_NO, KC_NO, KC_BSPACE },

    // Row 2: QWERTY row (Q-P, [, ], Enter) - P is at position 10
    { KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
      KC_P, KC_NO, KC_NO, KC_ENTER },  // KC_P for pairing control

    // Row 3: ASDF row (A-L, ;, ', #, SWAP) - SWAP is at position 13
    { KC_NO, KC_NO, KC_S, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_L,
      KC_NO, KC_NO, KC_NONUS_HASH, KC_SWAP },  // KC_SWAP for device switching

    // Row 4: ZXCV row (Shift, \, Z-M, comma, period, /, Up, RShift)
    { KC_NO, KC_NO, KC_Z, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
      KC_NO, KC_NO, KC_NO, KC_NO },

    // Row 5: Bottom row (Fn, Ctrl, Win, Alt, Space, RAlt, RCtrl, arrows)
    { KC_NO, KC_NO, KC_NO, KC_LALT, KC_SPACE, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
      KC_NO, KC_NO, KC_NO, KC_NO }
};

StateHandler stateMachine[5] = { [KEY_IDLE] = handleIdle, [KEY_DEBOUNCE
		] = handleDebounce, [KEY_PRESSED ] = handlePressed, [KEY_HELD
		] = handleHeld, [KEY_RELEASED ] = handleReleased };


void initKeyboard(void)
{
	memset(&kb_state, 0, sizeof(keyboard_state_t));
	memset(key_states, 0, sizeof(key_states));

	kb_state.current_layer = BASE_LAYER;
	kb_state.key_count = 0;

	loadKeymap(BASE_LAYER, &base_keymap);
	loadKeymap(FN_LAYER, &fn_keymap);

	// Initialize phantom filtering
	last_any_key_time = 0;
	last_key_row = 0xFF;
	last_key_col = 0xFF;
	phantom_count = 0;
	valid_key_count = 0;

	// Initialize column timing
	memset(last_column_time, 0, sizeof(last_column_time));

}

void loadKeymap(uint8_t layer, keymap_t *keymap)
{
	if (layer < KEY_LAYERS) {
		memcpy(&layers[layer], keymap, sizeof(keymap_t));
	}
}

bool isModifier(uint8_t keycode)
{
	return keycode >= KC_LCTRL && keycode <= KC_RGUI;
}

uint8_t getModifierBit(uint8_t keycode)
{
	switch (keycode)
	{
		case KC_LCTRL:
			return 0x01;
		case KC_LSHIFT:
			return 0x02;
		case KC_LALT:
			return 0x04;
		case KC_LGUI:
			return 0x08;
		case KC_RCTRL:
			return 0x10;
		case KC_RSHIFT:
			return 0x20;
		case KC_RALT:
			return 0x40;
		case KC_RGUI:
			return 0x80;
		default:
			return 0;
	}
}

uint8_t getCurrentLayer(void)
{
	if (kb_state.fn_pressed) {
		return FN_LAYER;
	}
	if (kb_state.swap_active) {
		return CUSTOM_LAYER;
	}
	return BASE_LAYER;
}

void clearReport(void)
{
	kb_state.fn_pressed = false;
	kb_state.swap_active = false;
	kb_state.current_layer = 0;
}

void resetKeyboardState(void)
{
	kb_state.fn_pressed = false;
	kb_state.swap_active = false;
}

static uint16_t getStdFuncBit(uint8_t keycode)
{
    switch (keycode) {
        case KC_PREV:        return 0x00B6; // Scan Previous Track
        case KC_NEXT:        return 0x00B5; // Scan Next Track
        case KC_PLAY:        return 0x00CD; // Play/Pause
        case KC_MUTE:        return 0x00E2; // Mute
        case KC_VOLDOWN:     return 0x00EA; // Volume Down
        case KC_VOLUP:       return 0x00E9; // Volume Up
        case KC_STOP:        return 0x00B7; // Stop
        case KC_REWIND:      return 0x00B4; // Rewind

        // Second byte keys
        case KC_PAUSE:       return 0x00B1; // Pause
        case KC_FASTFORWARD: return 0x00B3; // Fast Forward
        case KC_RECORD:      return 0x00B2; // Record
        case KC_EJECTCD:     return 0x00B8; // Eject
        case KC_CALCULATOR:  return 0x0192; // Calculator

        default:             return 0;      // No mapping
    }
}

uint8_t getFuncBit(uint8_t keycode)
{
    switch (keycode)
    {
        // First byte of consumer report (buttons[0])
        case KC_PREV:
            return 0x01;  // Bit 0
        case KC_NEXT:
            return 0x02;  // Bit 1
        case KC_PLAY:
            return 0x04;  // Bit 2
        case KC_MUTE:
            return 0x08;  // Bit 3
        case KC_VOLDOWN:
            return 0x10;  // Bit 4
        case KC_VOLUP:
            return 0x20;  // Bit 5
        case KC_STOP:
            return 0x40;  // Bit 6
        case KC_REWIND:
            return 0x80;  // Bit 7

        // Second byte of consumer report (buttons[1])
        // These would need to be handled separately in handleFnKey
        case KC_PAUSE:
            return 0x01;  // Bit 0 of buttons[1]
        case KC_FASTFORWARD:
            return 0x02;  // Bit 1 of buttons[1]
        case KC_RECORD:
            return 0x08;  // Bit 3 of buttons[1]
        case KC_EJECTCD:
            return 0x20;  // Bit 5 of buttons[1]
        case KC_CALCULATOR:
            return 0x40;  // Bit 6 of buttons[1]

        default:
            return 0;
    }
}


static bool process_ble_pairing_key(bool pressed)
{
    extern volatile connection_mode_t g_connection_mode;

    if (g_connection_mode != CONNECTION_BLE) {
        return false; // Only process in BLE mode
    }

    uint32_t current_time = getMicroseconds();

    if (pressed) {
        // Key pressed - start timing
        fn_p_press_start_time = current_time;
        fn_p_hold_active = true;
        fn_p_pairing_triggered = false;
        LOG_DEBUG("Fn+P pressed, starting %dms timer for pairing mode", BLE_PAIRING_HOLD_TIME_MS);
        return true;

    } else {
        // Key released - check for short press action
        if (fn_p_hold_active && !fn_p_pairing_triggered) {
            uint32_t hold_duration = (current_time - fn_p_press_start_time) / 1000; // Convert to ms

            if (hold_duration < BLE_PAIRING_SHORT_PRESS_MS) {
                // Short press - exit pairing mode if active
                if (ble_is_pairing_mode()) {
                    LOG_INFO("Fn+P short press - exiting pairing mode");
                    if (ble_exit_pairing_mode()) {
                    	Effects_BLE_PairingSuccess();
                        LOG_INFO("Successfully exited pairing mode");
                    } else {
                        LOG_ERROR("Failed to exit pairing mode");
                        Effects_BLE_PairingFailed(); // Red flash
                    }
                } else {
                    LOG_DEBUG("Fn+P short press - not in pairing mode, no action");
                }
            }
            // Note: Long press (3+ seconds) is handled in check_ble_key_functions()
        }

        // Reset state on key release
        fn_p_hold_active = false;
        fn_p_pairing_triggered = false;
        return true;
    }
}

static bool process_ble_device_switch_key(bool pressed)
{
    extern volatile connection_mode_t g_connection_mode;

    if (g_connection_mode != CONNECTION_BLE) {
        return false; // Only process in BLE mode
    }

    if (!pressed) {
        return true; // We only act on key press, not release
    }

    if (device_switch_cooldown) {
        uint32_t current_time = getMicroseconds() / 1000; // Convert to ms
        uint32_t time_since_last = current_time - last_device_switch_time;

        if (time_since_last < BLE_DEVICE_SWITCH_COOLDOWN_MS) {
            LOG_DEBUG("Device switch on cooldown (%lu ms remaining)",
                     BLE_DEVICE_SWITCH_COOLDOWN_MS - time_since_last);
            return true; // Handled but ignored due to cooldown
        }
    }

    LOG_INFO("Fn+SWAP pressed - initiating device switch");

    // Check if we have multiple devices to switch between
    uint8_t device_count = ble_get_paired_device_count();
    if (device_count < 2) {
        LOG_WARNING("Cannot switch devices - only %d device(s) paired", device_count);
        Effects_BLE_PairingFailed();
        return true;
    }

    // Attempt device switch
    if (ble_switch_to_next_device()) {
        LOG_INFO("Device switch initiated successfully");

        // Set cooldown to prevent rapid switching
        last_device_switch_time = getMicroseconds() / 1000;
        device_switch_cooldown = true;

        // Provide visual feedback
        Effects_BLE_DeviceSwitch(); // Cyan flash

    } else {
        LOG_WARNING("Device switch failed - check BLE module status");
        Effects_BLE_PairingFailed();
    }

    return true;
}

static bool process_media_key(uint8_t keycode, bool pressed)
{
    extern volatile connection_mode_t g_connection_mode;

    if (g_connection_mode == CONNECTION_BLE) {
        // BLE mode: Use HID Usage codes for media keys
        uint16_t usage_code = getStdFuncBit(keycode);

        if (usage_code != 0) { // Valid consumer key
            if (pressed) {
                kb_state.consumer_report.buttons[0] = usage_code & 0xFF;
                kb_state.consumer_report.buttons[1] = (usage_code >> 8) & 0xFF;
                LOG_DEBUG("BLE media key pressed: 0x%04X", usage_code);
            } else {
                kb_state.consumer_report.buttons[0] = 0;
                kb_state.consumer_report.buttons[1] = 0;
                LOG_DEBUG("BLE media key released");
            }
            report_ready_flag = 1;
            return true;
        }
    } else {
        // USB mode: Use custom bit mapping
        uint8_t button_bit = getFuncBit(keycode);
        uint8_t button_byte = 0;

        // Determine which byte to use for certain keys
        if (keycode == KC_PAUSE || keycode == KC_FASTFORWARD || keycode == KC_RECORD ||
            keycode == KC_EJECTCD || keycode == KC_CALCULATOR) {
            button_byte = 1;
        }

        if (button_bit != 0) {
            if (pressed) {
                // Clear both bytes first
                kb_state.consumer_report.buttons[0] = 0;
                kb_state.consumer_report.buttons[1] = 0;
                // Set the appropriate bit in the correct byte
                kb_state.consumer_report.buttons[button_byte] |= button_bit;
                LOG_DEBUG("USB media key pressed: byte[%d] = 0x%02X", button_byte, button_bit);
            } else {
                // On key release, clear both bytes
                kb_state.consumer_report.buttons[0] = 0;
                kb_state.consumer_report.buttons[1] = 0;
                LOG_DEBUG("USB media key released");
            }
            report_ready_flag = 1;
            return true;
        }
    }

    return false; // Key not handled as media key
}

static bool process_system_function_key(uint8_t keycode, bool pressed)
{
    switch (keycode) {
        case KC_LOCK:
            if (pressed) {
                kb_state.boot_report.modifiers |= 0x08; // LGUI (Windows key)
                handleKeyPress(KC_L);
                LOG_DEBUG("Lock key (Win+L) pressed");
            } else {
                handleKeyRelease(KC_L);
                kb_state.boot_report.modifiers &= ~0x08; // Release LGUI
                LOG_DEBUG("Lock key (Win+L) released");
            }
            report_ready_flag = 1;
            return true;

        case KC_APPLICATION:
            // Application/Menu key
            if (pressed) {
                handleKeyPress(KC_APPLICATION);
                LOG_DEBUG("Application key pressed");
            } else {
                handleKeyRelease(KC_APPLICATION);
                LOG_DEBUG("Application key released");
            }
            report_ready_flag = 1;
            return true;

        case KC_FIND:
            if (pressed) {
                kb_state.boot_report.modifiers |= 0x01; // LCTRL
                handleKeyPress(KC_F);
                LOG_DEBUG("Find key (Ctrl+F) pressed");
            } else {
                handleKeyRelease(KC_F);
                kb_state.boot_report.modifiers &= ~0x01; // Release LCTRL
                LOG_DEBUG("Find key (Ctrl+F) released");
            }
            report_ready_flag = 1;
            return true;

        default:
            return false; // Key not handled
    }
}

static bool process_internal_function_key(uint8_t keycode, bool pressed)
{
    if (!pressed) {
        return true; // Internal functions only trigger on press
    }

    switch (keycode) {
        case KC_LOWER_BRIGHT:
        	// TODO: Implement device-specific brightness decrease
            LOG_DEBUG("Internal function: Decrease brightness");
            return true;

        case KC_INCR_BRIGHT:
        	// TODO: Implement device-specific brightness increase
            LOG_DEBUG("Internal function: Increase brightness");
            return true;

        case KC_MIC_TOGGLE:
        	// TODO: Implement device-specific microphone toggle
            LOG_DEBUG("Internal function: Toggle microphone");
            return true;

        case KC_BACKLIGHT_DIM:
        	Effects_DecreaseBrightness();
            return true;

        case KC_BACKLIGHT_INCR:
        	Effects_IncreaseBrightness();
            return true;

        default:
            return false; // Key not handled
    }
}

static void handleKeyPress(uint8_t keycode)
{
	if (kb_state.key_count < 6) {
		kb_state.boot_report.keys[kb_state.key_count] = keycode;
		kb_state.key_count++;
	}else{
		uint8_t byte = keycode / 8;
		uint8_t bit = keycode % 8;
		if (byte < sizeof(kb_state.nkro_report.bitmap)) {
			kb_state.nkro_report.bitmap[byte] |= (1 << bit);
		}
	}
}

static void handleKeyRelease(uint8_t keycode)
{
	for (uint8_t i = 0; i < kb_state.key_count; i++) {
		if (kb_state.boot_report.keys[i] == keycode) {
			// Shift the remaining keys
			for (uint8_t j = i; j < kb_state.key_count - 1; j++) {
				kb_state.boot_report.keys[j] = kb_state.boot_report.keys[j + 1];
			}
			kb_state.key_count--;
			kb_state.boot_report.keys[kb_state.key_count] = 0;
			break;
		}
	}

	// Always clear the bit in NKRO bitmap
	uint8_t byte = keycode / 8;
	uint8_t bit = keycode % 8;
	if (byte < sizeof(kb_state.nkro_report.bitmap)) {
		kb_state.nkro_report.bitmap[byte] &= ~(1 << bit);
	}
}

static void handleModifiers(uint8_t keycode, bool pressed)
{
	uint8_t modifier_bit = getModifierBit(keycode);

	if (pressed) {
		// Update both report formats
		kb_state.boot_report.modifiers |= modifier_bit;
		kb_state.nkro_report.modifiers |= modifier_bit;
	}
	else {
		kb_state.boot_report.modifiers &= ~modifier_bit;
		kb_state.nkro_report.modifiers &= ~modifier_bit;
	}

	report_ready_flag = 1;
}

static void handleNormalKeys(uint8_t keycode, bool pressed)
{
	if (pressed) {
		handleKeyPress(keycode);
	}
	else {
		handleKeyRelease(keycode);
	}
}

static void handleFnKey(uint8_t keycode, bool pressed)
{
    // First, try to handle BLE-specific keys
    if (keycode == KC_P) {
        if (process_ble_pairing_key(pressed)) {
            return; // Key handled
        }
    }

    if (keycode == KC_SWAP) {
        if (process_ble_device_switch_key(pressed)) {
            return; // Key handled
        }
    }

    if(keycode == KC_SPACE && pressed){
    	EffectsToggleBacklight();
    	return;
    }

    if(keycode == KC_Z && pressed){
    	bool current_status = Effects_HasRuntimeZones();
		Effects_ToggleRuntimeZones(!current_status);
    	return;
    }

    if(keycode == KC_1  && pressed){
    	Effects_SwitchRuntimeZone(0);
		return;
	}

    if(keycode == KC_2 && pressed){
		Effects_SwitchRuntimeZone(1);
		return;
	}

    if(config_ui_update_entry_sequence(keycode, pressed)){
    	return;
    }

    // Try to handle as media key
    if (process_media_key(keycode, pressed)) {
        return;
    }

    // Try to handle as system function key
    if (process_system_function_key(keycode, pressed)) {
        return;
    }

    // Try to handle as internal function key
    if (process_internal_function_key(keycode, pressed)) {
        return;
    }

    // If we reach here, the key was not handled
    LOG_DEBUG("Unhandled function key: 0x%02X (pressed: %d)", keycode, pressed);
}

static bool isPhantomKey(uint8_t row, uint8_t col, uint32_t current_time, bool pressed) {
    // Only keep global rapid key check - column filtering now happens in scanner
    uint32_t time_since_last = current_time - last_any_key_time;

    if (time_since_last < PHANTOM_FILTER_TIME_US) {
        // Same key? Allow it (for legitimate repeat/bounce)
        if (row == last_key_row && col == last_key_col) {
            return false;  // Same key is OK
        }

        // Different key too fast - definitely phantom
        LOG_DEBUG("GLOBAL PHANTOM: R%d C%d %s (%luµs after R%d C%d)",
                 row, col, pressed ? "PRESS" : "RELEASE",
                 time_since_last, last_key_row, last_key_col);
        phantom_count++;
        return true;
    }

    return false;
}

void processKey(uint8_t row, uint8_t col, bool pressed)
{
	if (row >= KEY_ROWS || col >= KEY_COLS) return;

	uint32_t current_time = getMicroseconds();

	//Filter phantoms on BOTH press and release
	if (isPhantomKey(row, col, current_time, pressed)) {
		return;  // Block phantom press OR release
	}

	// Record this valid key event (for both press and release)
	last_any_key_time = current_time;
	last_key_row = row;
	last_key_col = col;
	last_column_time[col] = current_time;

	uint8_t keycode = layers[BASE_LAYER][row][col];
	if (keycode == KC_FN) kb_state.fn_pressed = pressed;

	if(config_ui_is_active()){
		config_ui_handle_key_event(row,col,pressed);
		return;
	}

	// Get the keycode from the current layer (which may have changed if FN was pressed/released)
	uint8_t current_layer = getCurrentLayer();
	LOG_DEBUG("Key Event: row=%d, col=%d, pressed=%d", row, col, pressed);
	    LOG_DEBUG("Keycode: 0x%X", layers[BASE_LAYER][row][col]);
	keycode = layers[current_layer][row][col];

	if (keycode == KC_NO) return;

	// If in FN layer, check if it's a media key
	if (current_layer == FN_LAYER) {
		// Process function/media keys
		LOG_DEBUG("FN_LAYER set");
		handleFnKey(keycode, pressed);
		return;
	}

	if (isModifier(keycode)) {
		handleModifiers(keycode, pressed);
		return;
	}
	else {
		handleNormalKeys(keycode, pressed);
	}

	report_ready_flag = 1;
}

keystate_t handleIdle(uint8_t row, uint8_t col, bool col_pressed,
		uint32_t current_time)
{
	if (col_pressed) {
		last_change_time[row][col] = current_time;
		return KEY_DEBOUNCE;
	}
	return KEY_IDLE;
}

keystate_t handleDebounce(uint8_t row, uint8_t col, bool col_pressed,
		uint32_t current_time)
{
	if (col_pressed
			&& (current_time - last_change_time[row][col] > DEBOUNCE_TIME_US)) {
		//LOG_DEBUG("Key [%u][%u] PRESSED", row, col);
		processKey(row, col, true); // Key pressed
		return KEY_PRESSED;
	}
	if (!col_pressed) {
		return KEY_IDLE;
	}
	return KEY_DEBOUNCE;
}

keystate_t handlePressed(uint8_t row, uint8_t col, bool col_pressed,
		uint32_t current_time)
{
	if (!col_pressed) {
		//LOG_DEBUG("Key [%u][%u] RELEASED", row, col);
		processKey(row, col, false); // Key released
		return KEY_RELEASED;
	}
	return KEY_HELD;
}

keystate_t handleHeld(uint8_t row, uint8_t col, bool col_pressed,
		uint32_t current_time)
{
	if (!col_pressed) {
		//LOG_DEBUG("Key [%u][%u] RELEASED", row, col);
		processKey(row, col, false); // Key released
		return KEY_RELEASED;
	}
	return KEY_HELD;
}

keystate_t handleReleased(uint8_t row, uint8_t col, bool col_pressed,
		uint32_t current_time)
{
	if(!kb_state.fn_pressed){
		uint8_t keycode = layers[BASE_LAYER][row][col];
		if (keycode == KC_FN) kb_state.fn_pressed = col_pressed;
	}

	return KEY_IDLE;
}

void check_ble_key_functions(void)
{
    extern volatile connection_mode_t g_connection_mode;

    if (g_connection_mode != CONNECTION_BLE) {
        return; // Only process BLE functions in BLE mode
    }

    uint32_t current_time_ms = HAL_GetTick();

    // Check for 3-second Fn+P hold to enter/exit pairing mode
    if (fn_p_hold_active && !fn_p_pairing_triggered) {
        uint32_t hold_duration = current_time_ms - (fn_p_press_start_time / 1000);

        if (hold_duration >= BLE_PAIRING_HOLD_TIME_MS) {
            // 3 seconds reached - trigger pairing mode toggle
            fn_p_pairing_triggered = true;

            LOG_INFO("Fn+P held for %dms - toggling pairing mode", BLE_PAIRING_HOLD_TIME_MS);

            if (ble_is_pairing_mode()) {
                LOG_INFO("Currently in pairing mode - exiting pairing mode");
                if (ble_exit_pairing_mode()) {
                    LOG_INFO("Successfully exited pairing mode");
                    Effects_BLE_PairingSuccess();
                } else {
                    LOG_ERROR("Failed to exit pairing mode");
                    Effects_BLE_PairingFailed();
                }
            } else {
                LOG_INFO("Entering pairing mode for 180 seconds (3 minutes)");
                if (ble_enter_pairing_mode(180)) {  // 3 minutes timeout
                    LOG_INFO("Pairing mode activated successfully - device is discoverable");
                    Effects_BLE_PairingActive();

                    // Log current paired device count for user information
                    uint8_t device_count = ble_get_paired_device_count();
                    LOG_INFO("Currently have %d paired device(s)", device_count);
                } else {
                	Effects_BLE_PairingFailed();
                    LOG_ERROR("Failed to enter pairing mode - check BLE module");
                }
            }
        }
    }

    // Clear device switch cooldown after timeout
    if (device_switch_cooldown) {
        if ((current_time_ms - last_device_switch_time) >= BLE_DEVICE_SWITCH_COOLDOWN_MS) {
            device_switch_cooldown = false;
            LOG_DEBUG("Device switch cooldown cleared");
        }
    }
}

