/*
 * config_ui.c - Master Configuration System Controller
 *
 * Implements the complete Yolk keyboard configuration architecture:
 * - Master Entry: Fn+Space+Enter for 5 seconds
 * - Section Navigation: 1=Zones, 2=Effects, 3=Power, 4=Brightness
 * - Universal Controls: Fn+S (save all), Fn+L (cancel all), Fn+Esc (menu)
 * - Unified Session Management with atomic operations
 * - 10-minute auto-timeout with warning
 */

#include "config_ui.h"
#include "zones.h"
#include "effects.h"
#include "backlight.h"
#include "keys.h"
#include "logger.h"
#include "pmsm.h"
#include "colors.h"
#include <string.h>

/* ========================================================================
 * MASTER CONFIGURATION CONSTANTS
 * ======================================================================== */
#define FN_HOLD_TIME_MS    					1000
#define CONFIG_ENTRY_HOLD_TIME_MS           5000    // Fn+R_Alt+Enter hold time
#define CONFIG_SESSION_TIMEOUT_MS           600000  // 10 minutes
#define CONFIG_TIMEOUT_WARNING_MS           540000  // 9 minutes (1 min warning)
#define CONFIG_VISUAL_UPDATE_INTERVAL_MS    100
#define CONFIG_MAX_SESSION_ERRORS           5

// Visual feedback colors
#define CONFIG_SECTION_INDICATOR_COLOR      Color_Bright(COLOR_GREEN)
#define CONFIG_ACTIVE_SECTION_COLOR         Color_Bright(COLOR_CYAN)
#define CONFIG_SAVE_SUCCESS_COLOR           Color_Bright(COLOR_GREEN)
#define CONFIG_CANCEL_COLOR                 Color_Bright(COLOR_RED)
#define CONFIG_WARNING_COLOR                Color_Bright(COLOR_YELLOW)
#define CONFIG_BREATHING_COLOR              Color_Dim(COLOR_CREAM_WHITE)

// Section key mappings (number row)
#define SECTION_ZONES_KEY_COL               1       // Key "1"
#define SECTION_EFFECTS_KEY_COL             2       // Key "2"
#define SECTION_POWER_KEY_COL               3       // Key "3"
#define SECTION_BRIGHTNESS_KEY_COL          4       // Key "4"
#define SECTION_KEY_ROW                     1       // Number row

#define EFFECTS_COLOR_KEY_ROW           3       // Row for color selection (asdfghjkl;')
#define EFFECTS_HSV_TIMEOUT_MS          300000  // 5 minutes
#define EFFECTS_HSV_WARNING_MS          285000  // 4:45 warning



/* ========================================================================
 * MASTER CONFIGURATION STATE
 * ======================================================================== */

typedef enum {
    MASTER_STATE_IDLE = 0,              // Normal operation
    MASTER_STATE_ENTRY_SEQUENCE,        // Detecting Fn+Space+Enter
    MASTER_STATE_MAIN_MENU,             // Section selection (1-4)
    MASTER_STATE_ZONE_PROGRAMMING,      // Zone configuration active
    MASTER_STATE_EFFECTS_CONFIG,        // Effects configuration active
    MASTER_STATE_POWER_CONFIG,          // Power management active
    MASTER_STATE_BRIGHTNESS_CONFIG      // Brightness control active
} master_config_state_t;


typedef enum {
    EFFECTS_STATE_MODE_AND_COLOR,   // Normal mode/color selection
    EFFECTS_STATE_HSV_EDIT         // HSV editing active
} effects_config_state_t;

typedef struct {
    // Master state
    master_config_state_t state;
    master_config_state_t previous_state;

    // Entry sequence tracking
    uint32_t entry_sequence_start_time;
    bool fn_pressed;
    bool space_pressed;
    bool enter_pressed;
    bool entry_sequence_active;
    bool entry_sequence_complete;

    // Unified session management
    uint32_t session_start_time;
    uint32_t last_activity_time;
    bool session_active;
    bool has_unsaved_changes;
    uint8_t sections_modified_mask;     // Bitmask: bit0=zones, bit1=effects, bit2=power, bit3=brightness

    // Subsystem session states
    bool zones_session_active;
    bool effects_changes_pending;
    bool power_changes_pending;
    bool brightness_changes_pending;

    // Timeout and safety
    bool timeout_warning_shown;
    uint8_t error_count;
    config_status_t last_error;

    // Visual feedback
    uint32_t last_visual_update;
    uint8_t breathing_phase;
    bool breathing_direction_up;
    bool menu_indicators_active;

    // Saved state for restoration
	bool saved_effects_enabled;
	lmp_behavior_t saved_usb_lmp_behavior;
	lmp_behavior_t saved_ble_lmp_behavior;
	bool temp_enabled_for_config;

	// Effects configuration state
	effects_config_state_t effects_state;
	uint8_t effects_editing_color_index;    // Which color being edited
	uint32_t effects_hsv_start_time;        // HSV edit session timer

	bool right_alt_pressed;

	struct {
		uint16_t hue;           // 0-359
		uint8_t saturation;     // 0-100
		uint8_t value;          // 1-100
		uint8_t hue_step;       // Step size for hue
		uint8_t sv_step;        // Step size for sat/val
	} effects_hsv;

} master_config_context_t;

static struct {
    bool fn_blocking_active;        // In blocking state
    uint32_t fn_press_start_time;   // When Fn was first pressed
    bool fn_pressed;                // Current Fn state
    uint8_t fn_row, fn_col;        // Fn key position
} g_blocking_state = {0};


static master_config_context_t g_master_config = {0};
static bool g_master_initialized = false;

// Global flag to block HID reports during configuration
bool g_config_ui_active = false;
extern keyboard_state_t kb_state;
extern effect_state_t g_effect;



/* ========================================================================
 * PRIVATE FUNCTIONS
 * ======================================================================== */

static void blocking_state_enter(void);
static void blocking_state_exit(void);
static bool blocking_state_handle_key(uint8_t row, uint8_t col, bool pressed);
static void blocking_state_process(void);
static void blocking_state_update_visual(void);
static void update_power_config_backlight(void);

// Section-specific save/cancel operations
static bool config_ui_save_current_section(void);
static bool config_ui_cancel_current_section(void);

static config_status_t config_ui_save_zone_section(void);
static config_status_t config_ui_cancel_zone_section(void);
static config_status_t config_ui_save_effects_section(void);
static config_status_t config_ui_cancel_effects_section(void);
static config_status_t config_ui_save_power_section(void);
static config_status_t config_ui_cancel_power_section(void);
static config_status_t config_ui_save_brightness_section(void);
static config_status_t config_ui_cancel_brightness_section(void);

/* ========================================================================
 * EFFECTS CONFIGURATION - STATIC FUNCTION PROTOTYPES
 * ======================================================================== */

// Utility functions
static uint8_t effects_col_to_color_index(uint8_t col);
static bool effects_is_color_editable(uint8_t color_index);
static bool effects_is_color_valid(uint8_t color_index);

// Visual display functions
static void update_effects_config_display(void);
static void show_effects_mode_indicators(void);
static void show_effects_color_indicators(void);
static void show_effects_hsv_display(void);

// HSV editing functions
static config_status_t effects_enter_hsv_mode(uint8_t color_index);
static void effects_exit_hsv_mode(bool apply_changes);
static config_status_t effects_adjust_hue(int8_t direction);
static config_status_t effects_adjust_saturation(int8_t direction);
static config_status_t effects_adjust_value(int8_t direction);

// Key handler functions
static bool handle_effects_mode_selection(uint8_t col);
static bool handle_effects_color_selection(uint8_t col);
static bool handle_effects_hsv_keys(uint8_t row, uint8_t col);
static bool handle_effects_right_alt_combinations(uint8_t row, uint8_t col);
/* ========================================================================
 * BLOCKING STATE MANAGEMENT
 * ======================================================================== */

static void blocking_state_enter(void)
{
    g_blocking_state.fn_blocking_active = true;

    // Visual feedback - flash yellow then show commands
    Backlight_SetAllRGB(Color_Dim(COLOR_YELLOW));
    HAL_Delay(100);
    Backlight_SetAllRGB((Backlight_RGB_t){0, 0, 0});

    // Show available commands
    Backlight_SetKeyRGB(3, 2, COLOR_GREEN);    // S = Green (Save)
    Backlight_SetKeyRGB(3, 9, COLOR_CRIMSON);    // L = Red (Cancel)
}

static void blocking_state_exit(void)
{
    g_blocking_state.fn_blocking_active = false;

    Backlight_SetAllRGB((Backlight_RGB_t){0, 0, 0});
    switch (g_master_config.state) {
		case MASTER_STATE_ZONE_PROGRAMMING:
			Zone_Return_to_Section(g_blocking_state.fn_pressed);
			break;
		case MASTER_STATE_EFFECTS_CONFIG:
			config_ui_enter_effects();
			break;
		case MASTER_STATE_POWER_CONFIG:
			config_ui_enter_power();
			break;
		case MASTER_STATE_BRIGHTNESS_CONFIG:
			config_ui_enter_brightness();
			break;
		default:
			break;
	}
    LOG_INFO("Exiting Fn blocking state");
}

static bool blocking_state_handle_key(uint8_t row, uint8_t col, bool pressed)
{
    // Handle Fn key for zone programming mode only
    if (g_master_config.state == MASTER_STATE_MAIN_MENU) {
    	return false;
    }

    if(isZoneHSVMode()) return false;

    // Track Fn key presses/releases
    if (row == g_blocking_state.fn_row && col == g_blocking_state.fn_col) {
        if (pressed && !g_blocking_state.fn_pressed) {
            // Fn just pressed - start timing
            g_blocking_state.fn_pressed = true;
            g_blocking_state.fn_press_start_time = HAL_GetTick();
            LOG_DEBUG("Fn pressed - starting 2s timer");

        } else if (!pressed && g_blocking_state.fn_pressed) {
            // Fn released
            g_blocking_state.fn_pressed = false;

            if (g_blocking_state.fn_blocking_active) {
                blocking_state_exit();
                return true; // Consume the Fn release
            }
        }

        // If in blocking state, consume Fn events
        if (g_blocking_state.fn_blocking_active) {
            return true;
        }
        return false; // Let normal processing handle Fn
    }

    // If in blocking state, handle command keys or block others
    if (g_blocking_state.fn_blocking_active && pressed) {
        uint8_t keycode = layers[BASE_LAYER][row][col];

        switch (keycode) {
            case KC_S:
                LOG_INFO("Blocking state: Save command");
                return config_ui_save_current_section();
            case KC_L:
                LOG_INFO("Blocking state: Cancel command");
                return config_ui_cancel_current_section();

            default:
                // Block all other keys in blocking state
                LOG_DEBUG("Blocking state: Key 0x%02X blocked", keycode);
                return true;
        }
    }

    return false; // Not handled by blocking state
}

static void blocking_state_process(void)
{

	if(isZoneHSVMode() || g_master_config.state == MASTER_STATE_MAIN_MENU){
		g_blocking_state.fn_pressed = false;
		g_blocking_state.fn_blocking_active = false;
		return;
	}


    if(g_master_config.state != MASTER_STATE_MAIN_MENU &&
		!kb_state.fn_pressed &&
		g_blocking_state.fn_blocking_active){

		g_blocking_state.fn_pressed = false;
		blocking_state_exit();
	}


    // Check for Fn blocking state activation in zone programming
	if (g_master_config.state != MASTER_STATE_MAIN_MENU &&
		g_blocking_state.fn_pressed &&
		!g_blocking_state.fn_blocking_active) {

		uint32_t hold_duration = HAL_GetTick() - g_blocking_state.fn_press_start_time;

		if (hold_duration >= FN_HOLD_TIME_MS) {
			blocking_state_enter();
		}
	}


}

static void blocking_state_update_visual(void)
{
    // Maintain command indicators if in blocking state
    if (g_blocking_state.fn_blocking_active) {
        Backlight_SetKeyRGB(3, 2, (Backlight_RGB_t){0, 50, 0});    // S = Green
        Backlight_SetKeyRGB(3, 9, (Backlight_RGB_t){50, 0, 0});    // L = Red
    }
}

/* ========================================================================
 * MASTER SYSTEM INITIALIZATION
 * ======================================================================== */

config_status_t config_ui_init(void)
{
    if (g_master_initialized) {
        return CONFIG_OK;
    }

    LOG_INFO("Initializing master configuration system...");

    memset(&g_master_config, 0, sizeof(g_master_config));
    memset(&g_blocking_state, 0, sizeof(g_blocking_state));
    g_master_config.state = MASTER_STATE_IDLE;

    // Initialize all subsystems
    if (Zones_Init() != CONFIG_OK) {
        LOG_ERROR("Failed to initialize zones subsystem");
        return CONFIG_ERROR;
    }

    g_master_initialized = true;
    g_config_ui_active = false;
	g_blocking_state.fn_row = 5;
	g_blocking_state.fn_col = 0;

    LOG_INFO("Master configuration system initialized successfully");
    return CONFIG_OK;
}

/* ========================================================================
 * MASTER ENTRY SEQUENCE DETECTION
 * ======================================================================== */

bool config_ui_update_entry_sequence(uint8_t keycode, bool pressed)
{
    if (g_master_config.state != MASTER_STATE_IDLE) {
        return false;  // Only detect entry from normal mode
    }

    uint32_t current_time = HAL_GetTick();
    bool sequence_changed = false;

    // Track individual key states
    if (keycode == KC_ENTER) {
        g_master_config.space_pressed = pressed;
        sequence_changed = true;
        LOG_INFO("Space Key Pressed");
    }

    if (keycode == KC_LALT) {
        g_master_config.enter_pressed = pressed;
        sequence_changed = true;
        LOG_INFO("Enter Key Pressed");
    }

    // Fn state is tracked externally via kb_state.fn_pressed
    g_master_config.fn_pressed = kb_state.fn_pressed;

    // Check if complete sequence is active
    bool sequence_complete = g_master_config.fn_pressed &&
                           g_master_config.space_pressed &&
                           g_master_config.enter_pressed;

    if (sequence_complete && !g_master_config.entry_sequence_active) {
        // Sequence just started
        g_master_config.entry_sequence_start_time = current_time;
        g_master_config.entry_sequence_active = true;
        g_master_config.state = MASTER_STATE_ENTRY_SEQUENCE;
        LOG_DEBUG("Configuration entry sequence started");

    } else if (!sequence_complete && g_master_config.entry_sequence_active) {
        // Sequence broken
        g_master_config.entry_sequence_active = false;
        g_master_config.entry_sequence_complete = false;
        g_master_config.state = MASTER_STATE_IDLE;
        LOG_DEBUG("Configuration entry sequence cancelled");
    }

    return sequence_changed;
}

void check_config_ui_functions(void)
{
    if (!g_master_initialized) {
        return;
    }

    // Check entry sequence completion
    if (g_master_config.state == MASTER_STATE_ENTRY_SEQUENCE) {
        uint32_t current_time = HAL_GetTick();
        uint32_t hold_duration = current_time - g_master_config.entry_sequence_start_time;

        if (hold_duration >= (CONFIG_ENTRY_HOLD_TIME_MS)) {
            g_master_config.entry_sequence_complete = true;
            LOG_INFO("Configuration entry sequence complete - entering config mode");
            config_ui_enter_master_mode();
        }
    }
}

/* ========================================================================
 * MASTER MODE ENTRY/EXIT
 * ======================================================================== */

config_status_t config_ui_enter_master_mode(void)
{
    if (g_master_config.state != MASTER_STATE_ENTRY_SEQUENCE) {
        return CONFIG_INVALID_PARAMETER;
    }

    LOG_INFO("Entering master configuration mode");

    // Block HID reports
    g_config_ui_active = true;
	if(g_effect.config.ble_lmp_behavior == LMP_BEHAVIOR_SYSTEM_OFF
		&& g_effect.config.usb_lmp_behavior == LMP_BEHAVIOR_SYSTEM_OFF){
		Backlight_ExitShutdown();
	}
    // Stop normal effects
    Effects_Stop();



    // Initialize session
    g_master_config.session_start_time = HAL_GetTick();
    g_master_config.last_activity_time = HAL_GetTick();
    g_master_config.session_active = true;
    g_master_config.has_unsaved_changes = false;
    g_master_config.sections_modified_mask = 0;
    g_master_config.timeout_warning_shown = false;
    g_master_config.error_count = 0;

    // Reset entry sequence
    g_master_config.entry_sequence_active = false;
    g_master_config.entry_sequence_complete = false;
    g_master_config.fn_pressed = false;
    g_master_config.space_pressed = false;
    g_master_config.enter_pressed = false;

    kb_state.fn_pressed = false;
    // Enter main menu
    g_master_config.state = MASTER_STATE_MAIN_MENU;

    // Show visual confirmation and section indicators
    config_ui_flash_enter_confirmation();
    config_ui_show_section_menu();

    LOG_INFO("Master configuration mode active - showing section menu");
    return CONFIG_OK;
}

config_status_t config_ui_exit_master_mode(bool save_changes)
{
    if (!g_master_config.session_active) {
        return CONFIG_INVALID_PARAMETER;
    }

    g_blocking_state.fn_blocking_active = false;
    g_blocking_state.fn_pressed = false;

    LOG_INFO("Exiting master configuration mode (save=%d)", save_changes);

    config_status_t result = CONFIG_OK;

    // Handle any active sessions that haven't been explicitly saved/cancelled
    if (g_master_config.zones_session_active) {
        result = Zones_ExitProgramming(save_changes);
        g_master_config.zones_session_active = false;
    }

    // Clean up any other pending changes
    if (save_changes) {
        // Save any remaining pending section changes to EEPROM
        if (g_master_config.effects_changes_pending) {
            config_ui_save_effects_section();
        }
        if (g_master_config.power_changes_pending) {
            config_ui_save_power_section();
        }
        if (g_master_config.brightness_changes_pending) {
            config_ui_save_brightness_section();
        }
    } else {
        // Clear pending flags without saving
        g_master_config.effects_changes_pending = false;
        g_master_config.power_changes_pending = false;
        g_master_config.brightness_changes_pending = false;
    }

    // Reset master state and restore normal operation
    g_master_config.state = MASTER_STATE_IDLE;
    g_master_config.session_active = false;
    g_config_ui_active = false;
    Effects_StartRuntimeEffect();

    if (!g_effect.config.effects_enabled) {
        Backlight_SetAll(0, 0, 0);
        Backlight_EnterShutdown();
    }

    LOG_INFO("Master configuration mode exited");
    return result;
}

/* ========================================================================
 * MAIN MENU AND SECTION NAVIGATION
 * ======================================================================== */

void config_ui_show_section_menu(void)
{
    // Set breathing background
	Backlight_SetAllRGB(BACKLIGHT_OFF);

    // Show section indicators on number row (1-4)
    Backlight_SetKeyRGB(SECTION_KEY_ROW, SECTION_ZONES_KEY_COL, CONFIG_SECTION_INDICATOR_COLOR);
    Backlight_SetKeyRGB(SECTION_KEY_ROW, SECTION_EFFECTS_KEY_COL, CONFIG_SECTION_INDICATOR_COLOR);
    Backlight_SetKeyRGB(SECTION_KEY_ROW, SECTION_POWER_KEY_COL, CONFIG_SECTION_INDICATOR_COLOR);
    Backlight_SetKeyRGB(SECTION_KEY_ROW, SECTION_BRIGHTNESS_KEY_COL, CONFIG_SECTION_INDICATOR_COLOR);

    g_master_config.menu_indicators_active = true;

    LOG_DEBUG("Section menu displayed - sections 1-4 highlighted");
}

config_status_t config_ui_enter_section(uint8_t section_number)
{
    if (g_master_config.state != MASTER_STATE_MAIN_MENU) {
        return CONFIG_INVALID_PARAMETER;
    }

    config_ui_update_activity();

    LOG_INFO("Entering section %d", section_number);

    config_status_t result = CONFIG_OK;

    switch (section_number) {
        case 1: // Zone Programming
            result = config_ui_enter_zones();
            break;

        case 2: // Effects Configuration
            result = config_ui_enter_effects();
            break;

        case 3: // Power Management
            result = config_ui_enter_power();
            break;

        case 4: // Brightness Control
            result = config_ui_enter_brightness();
            break;

        default:
            LOG_WARNING("Invalid section number: %d", section_number);
            return CONFIG_INVALID_PARAMETER;
    }

    if (result != CONFIG_OK) {
        LOG_ERROR("Failed to enter section %d: %d", section_number, result);
        config_ui_handle_error(result);
    }

    return result;
}

/* ========================================================================
 * SECTION IMPLEMENTATIONS
 * ======================================================================== */

config_status_t config_ui_enter_zones(void)
{
    LOG_INFO("Entering zone programming section");

    // Start zones programming subsystem
    config_status_t result = Zones_EnterProgramming();
    if (result != CONFIG_OK) {
        return result;
    }

    g_master_config.state = MASTER_STATE_ZONE_PROGRAMMING;
    g_master_config.zones_session_active = true;
    g_master_config.menu_indicators_active = false;

    return CONFIG_OK;
}

config_status_t config_ui_enter_effects(void)
{
    LOG_INFO("Entering effects configuration section");
    Backlight_SetAllRGB(BACKLIGHT_OFF);
    Effects_ConfigModeInit();

    g_master_config.effects_state = EFFECTS_STATE_MODE_AND_COLOR;
    g_master_config.effects_editing_color_index = 0xFF;

    g_master_config.state = MASTER_STATE_EFFECTS_CONFIG;
    g_master_config.menu_indicators_active = false;

    update_effects_config_display();

    return CONFIG_OK;
}

config_status_t config_ui_enter_power(void)
{
    LOG_INFO("Entering power management section");
    Backlight_SetAllRGB(BACKLIGHT_OFF);
    // Use the centralized update function
    update_power_config_backlight();

    g_master_config.state = MASTER_STATE_POWER_CONFIG;
    g_master_config.menu_indicators_active = false;

    return CONFIG_OK;
}

config_status_t config_ui_enter_brightness(void)
{
    LOG_INFO("Entering brightness control section"); //TODO
    Backlight_SetAllRGB(BACKLIGHT_OFF);
    // Brightness mode options (number row)
    Backlight_SetKeyRGB(1, 1, CONFIG_SECTION_INDICATOR_COLOR);  // Preset Manual (default)
    Backlight_SetKeyRGB(1, 2, CONFIG_ACTIVE_SECTION_COLOR);     // Adaptive Battery Based Lighting

    g_master_config.state = MASTER_STATE_BRIGHTNESS_CONFIG;
    g_master_config.menu_indicators_active = false;

    return CONFIG_OK;
}

/* ========================================================================
 * UNIVERSAL COMMAND PROCESSING
 * ======================================================================== */

bool config_ui_handle_key_event(uint8_t row, uint8_t col, bool pressed)
{
    if (!g_config_ui_active) {
        return false;
    }

    if(g_master_config.state != MASTER_STATE_MAIN_MENU){
		config_ui_update_activity();
		// Check blocking state first
		if (blocking_state_handle_key(row, col, pressed)) {
			return true; // Event handled by blocking state
		}

		// Handle R_Alt key tracking first (both press and release)
		if (row == 5 && col == 3) { // R_Alt position
			g_master_config.right_alt_pressed = pressed;
			return true;
		}

		// Check R_Alt combinations ONLY on key press
		if (g_master_config.right_alt_pressed && pressed) {
			if (handle_effects_right_alt_combinations(row, col)) {
				return true;
			}
		}

    } else if(g_master_config.state == MASTER_STATE_MAIN_MENU){

    	uint8_t current_layer = g_master_config.fn_pressed?FN_LAYER:BASE_LAYER;

    	if(layers[BASE_LAYER][row][col] == KC_FN){
    		current_layer = FN_LAYER;
    		g_master_config.fn_pressed = pressed;
    	}

		// Check for universal commands first (Fn + key combinations)
		if (current_layer == FN_LAYER) {
			return config_ui_process_universal_command(row, col);
		}
    }

    // Route to appropriate section handler
    switch (g_master_config.state) {
        case MASTER_STATE_MAIN_MENU:
            return config_ui_handle_main_menu_key(row, col);

        case MASTER_STATE_ZONE_PROGRAMMING:
            return Zones_HandleKey(row, col, pressed);

        case MASTER_STATE_EFFECTS_CONFIG:
            return config_ui_handle_effects_key(row, col);

        case MASTER_STATE_POWER_CONFIG:
            return config_ui_handle_power_key(row, col);

        case MASTER_STATE_BRIGHTNESS_CONFIG:
            return config_ui_handle_brightness_key(row, col);

        default:
            return false;
    }
}

bool config_ui_process_universal_command(uint8_t row, uint8_t col)
{
    uint8_t keycode = layers[FN_LAYER][row][col];

    // Fn + Backspace = Exit System Configuration
    if (keycode == KC_BSPACE) {
        LOG_INFO("Exit system configuration command received");
        config_ui_exit_master_mode(true);  // Save any remaining changes
        return true;
    }

    return false;
}

config_status_t config_ui_return_to_menu(void)
{

	g_blocking_state.fn_blocking_active = false;
	g_blocking_state.fn_pressed = false;

    // Exit current section without saving section-specific changes
    switch (g_master_config.state) {
        case MASTER_STATE_ZONE_PROGRAMMING:
            if (g_master_config.zones_session_active) {
                // Note: Individual section changes are discarded, but session continues
                g_master_config.zones_session_active = false;
                Zones_ExitProgramming(false);
            }
            break;

        case MASTER_STATE_EFFECTS_CONFIG:
            Effects_ConfigModeExit();
            break;

        case MASTER_STATE_POWER_CONFIG:
        case MASTER_STATE_BRIGHTNESS_CONFIG:
            // No special cleanup needed
            break;

        default:
            break;
    }

    // Return to main menu
    g_master_config.state = MASTER_STATE_MAIN_MENU;
    config_ui_show_section_menu();

    return CONFIG_OK;
}

/* ========================================================================
 * SECTION-SPECIFIC KEY HANDLERS
 * ======================================================================== */

bool config_ui_handle_main_menu_key(uint8_t row, uint8_t col)
{
    // Only handle number row (1-4) for section selection
    if (row != SECTION_KEY_ROW || col < 1 || col > 4) {
        return false;
    }

    config_ui_enter_section(col);
    return true;
}

bool config_ui_handle_effects_key(uint8_t row, uint8_t col)
{

    // Handle HSV editing mode first
    if (g_master_config.effects_state == EFFECTS_STATE_HSV_EDIT) {
        return handle_effects_hsv_keys(row, col);
    }

    // Handle mode selection (row 1, cols 1-3)
    if (row == 1 && col >= 1 && col <= 3) {
        return handle_effects_mode_selection(col);
    }

    // Handle color selection (row 3, cols 1-12)
    if (row == EFFECTS_COLOR_KEY_ROW && col >= 1 && col <= 12) {
        return handle_effects_color_selection(col);
    }

    return false;
}

static void update_power_config_backlight(void)
{
    // Determine current USB mode column based on actual config
    uint8_t usb_col = 0; // default fallback
    switch(g_effect.config.usb_lmp_behavior) {
       case LMP_BEHAVIOR_OFF: usb_col = 1; break;
       case LMP_BEHAVIOR_DIM: usb_col = 2; break;
       case LMP_BEHAVIOR_SYSTEM_OFF: usb_col = 3; break;
    }

    // Determine current BLE mode column based on actual config
    uint8_t ble_col = 0; // default fallback
    switch(g_effect.config.ble_lmp_behavior) {
       case LMP_BEHAVIOR_OFF: ble_col = 1; break;
       case LMP_BEHAVIOR_DIM: ble_col = 2; break;
       case LMP_BEHAVIOR_SYSTEM_OFF: ble_col = 3; break;
    }

    // USB modes (function row) - only show available options when backlight enabled
    if (g_effect.config.effects_enabled) {
        Backlight_SetKeyRGB(0, 1, (usb_col == 1) ?
        		CONFIG_ACTIVE_SECTION_COLOR : CONFIG_SECTION_INDICATOR_COLOR); // Off
        Backlight_SetKeyRGB(0, 2, (usb_col == 2) ?
        		CONFIG_ACTIVE_SECTION_COLOR : CONFIG_SECTION_INDICATOR_COLOR); // Dim
        Backlight_SetKeyRGB(0, 3, Color_Dim(COLOR_RED)); // System Off (grayed out)
    } else {
        // Backlight disabled - only show SYSTEM_OFF as active
        Backlight_SetKeyRGB(0, 1, Color_Dim(COLOR_RED)); // Off (grayed)
        Backlight_SetKeyRGB(0, 2, Color_Dim(COLOR_RED)); // Dim (grayed)
        Backlight_SetKeyRGB(0, 3, CONFIG_ACTIVE_SECTION_COLOR); // System Off (forced active)
    }

    // BLE modes (number row)
    if (g_effect.config.effects_enabled) {
        Backlight_SetKeyRGB(1, 1, (ble_col == 1) ?
        		CONFIG_ACTIVE_SECTION_COLOR : CONFIG_SECTION_INDICATOR_COLOR);
        Backlight_SetKeyRGB(1, 2, (ble_col == 2) ?
        		CONFIG_ACTIVE_SECTION_COLOR : CONFIG_SECTION_INDICATOR_COLOR);
        Backlight_SetKeyRGB(1, 3, Color_Dim(COLOR_RED));
    } else {
        Backlight_SetKeyRGB(1, 1, Color_Dim(COLOR_RED));
        Backlight_SetKeyRGB(1, 2, Color_Dim(COLOR_RED));
        Backlight_SetKeyRGB(1, 3, CONFIG_ACTIVE_SECTION_COLOR);
    }

    // Master backlight control (row 2) - shows ACTUAL persistent setting
    Backlight_SetKeyRGB(2, 1, g_effect.config.effects_enabled ?
    		CONFIG_ACTIVE_SECTION_COLOR : CONFIG_SECTION_INDICATOR_COLOR); // Enable
    Backlight_SetKeyRGB(2, 2, !g_effect.config.effects_enabled ?
    		CONFIG_ACTIVE_SECTION_COLOR : CONFIG_SECTION_INDICATOR_COLOR); // Disable
}

bool config_ui_handle_power_key(uint8_t row, uint8_t col)
{
   // Master backlight enable/disable (row 2)
   if (row == 2 && col >= 1 && col <= 2) {
       bool new_backlight_enabled = (col == 1); // 1=Enable, 2=Disable

       // Update the PERSISTENT setting (not temporary config state)
       bool current_setting =  g_effect.config.effects_enabled;

       if (new_backlight_enabled != current_setting) {
           // Update the setting we'll save
           if (g_master_config.session_active) {
               g_effect.config.effects_enabled = new_backlight_enabled;
           }

           if (!new_backlight_enabled) {
               g_effect.config.usb_lmp_behavior = LMP_BEHAVIOR_SYSTEM_OFF;
               g_effect.config.ble_lmp_behavior = LMP_BEHAVIOR_SYSTEM_OFF;
               LOG_INFO("Backlight disabled - LMP behaviors set to SYSTEM_OFF");
           }else{
        	   g_effect.config.usb_lmp_behavior = LMP_BEHAVIOR_OFF;
        	   g_effect.config.ble_lmp_behavior = LMP_BEHAVIOR_OFF;
        	   LOG_INFO("Backlight re-enabled - LMP behaviors defaulted to OFF");
        	   // TODO: Later read from EEPROM to restore user's previous LMP settings
		  }

           g_master_config.power_changes_pending = true;
           g_master_config.has_unsaved_changes = true;
           g_master_config.sections_modified_mask |= (1 << 2);
       }

       // Update visual display
       update_power_config_backlight();

       LOG_DEBUG("Master backlight %s", new_backlight_enabled ? "ENABLED" : "DISABLED");
       return true;
   }

   // USB/BLE mode selection - check if backlight is enabled in PERSISTENT settings
   bool backlight_enabled = g_effect.config.effects_enabled;

   // USB mode selection (row 0)
   if (row == 0 && col >= 1 && col <= 2) {
       if (!backlight_enabled) {
           LOG_DEBUG("USB mode locked to SYSTEM_OFF (backlight disabled)");
           return true;
       }

       lmp_behavior_t new_usb_behavior;
       switch(col) {
           case 1: new_usb_behavior = LMP_BEHAVIOR_OFF; break;
           case 2: new_usb_behavior = LMP_BEHAVIOR_DIM; break;
           default: return false;
       }

       if (new_usb_behavior != g_effect.config.usb_lmp_behavior) {
           g_effect.config.usb_lmp_behavior = new_usb_behavior;
           g_master_config.power_changes_pending = true;
           g_master_config.has_unsaved_changes = true;
           g_master_config.sections_modified_mask |= (1 << 2);
       }

       // Update visual display
       update_power_config_backlight();

       LOG_DEBUG("USB power mode %d selected", new_usb_behavior);
       return true;
   }

   // BLE mode selection (row 1) - similar logic
   if (row == 1 && col >= 1 && col <= 2) {
       if (!backlight_enabled) {
           LOG_DEBUG("BLE mode locked to SYSTEM_OFF (backlight disabled)");
           return true;
       }

       lmp_behavior_t new_ble_behavior;
       switch(col) {
           case 1: new_ble_behavior = LMP_BEHAVIOR_OFF; break;
           case 2: new_ble_behavior = LMP_BEHAVIOR_DIM; break;
           default: return false;
       }

       if (new_ble_behavior != g_effect.config.ble_lmp_behavior) {
           g_effect.config.ble_lmp_behavior = new_ble_behavior;
           g_master_config.power_changes_pending = true;
           g_master_config.has_unsaved_changes = true;
           g_master_config.sections_modified_mask |= (1 << 2);
       }

       // Update visual display
       update_power_config_backlight();

       LOG_DEBUG("BLE power mode %d selected", new_ble_behavior);
       return true;
   }

   return false;
}

bool config_ui_handle_brightness_key(uint8_t row, uint8_t col)
{
    // Brightness control key handling
    static uint8_t brightness_mode = 1;  // Default: preset

    if (row == 1 && col >= 1 && col <= 2) {
        // Brightness mode selection
        brightness_mode = col;

        // Update mode indicators
        for (uint8_t i = 1; i <= 2; i++) {
            Backlight_RGB_t color = (i == brightness_mode) ? CONFIG_ACTIVE_SECTION_COLOR : CONFIG_SECTION_INDICATOR_COLOR;
            Backlight_SetKeyRGB(1, i, color);
        }

        g_master_config.brightness_changes_pending = true;
        g_master_config.has_unsaved_changes = true;
        g_master_config.sections_modified_mask |= (1 << 3);  // Brightness = bit 3

        LOG_DEBUG("Brightness mode %d selected", brightness_mode);
        return true;
    }

    return false;
}

/* ========================================================================
 * UNIFIED SESSION MANAGEMENT
 * ======================================================================== */

static bool config_ui_save_current_section(void)
{
    config_ui_flash_save_result(true);
    g_blocking_state.fn_blocking_active = false;

    switch (g_master_config.state) {
        case MASTER_STATE_ZONE_PROGRAMMING:
            config_ui_save_zone_section();
            break;

        case MASTER_STATE_EFFECTS_CONFIG:
            config_ui_save_effects_section();
            break;

        case MASTER_STATE_POWER_CONFIG:
            config_ui_save_power_section();
            break;

        case MASTER_STATE_BRIGHTNESS_CONFIG:
            config_ui_save_brightness_section();
            break;

        case MASTER_STATE_MAIN_MENU:
            LOG_INFO("Save command ignored in main menu");
            return true;

        default:
            LOG_WARNING("Save command in unknown state: %d", g_master_config.state);
            return false;
    }
    g_blocking_state.fn_pressed = false;
    return true;
}

static bool config_ui_cancel_current_section(void)
{
    config_ui_flash_save_result(false);
    g_blocking_state.fn_blocking_active = false;

    switch (g_master_config.state) {
        case MASTER_STATE_ZONE_PROGRAMMING:
            config_ui_cancel_zone_section();
            break;

        case MASTER_STATE_EFFECTS_CONFIG:
            config_ui_cancel_effects_section();
            break;

        case MASTER_STATE_POWER_CONFIG:
            config_ui_cancel_power_section();
            break;

        case MASTER_STATE_BRIGHTNESS_CONFIG:
            config_ui_cancel_brightness_section();
            break;

        case MASTER_STATE_MAIN_MENU:
            LOG_INFO("Cancel command ignored in main menu");
            return true;

        default:
            LOG_WARNING("Cancel command in unknown state: %d", g_master_config.state);
            return false;
    }

    return true;
}

static config_status_t config_ui_save_effects_section(void)
{
    config_status_t result = CONFIG_OK;

    if (g_master_config.effects_changes_pending) {
        // Save effects configuration directly to EEPROM
        // TODO: Replace with actual EEPROM save function
        // result = effects_save_config_to_eeprom(&g_effect.config);

        if (result == CONFIG_OK) {
            LOG_INFO("Effects configuration saved to EEPROM");
            g_master_config.effects_changes_pending = false;
        } else {
            LOG_ERROR("Failed to save effects configuration to EEPROM: %d", result);
        }
    }

    // Return to MAIN MENU - EFFECTS SECTION COMPLETE
    g_master_config.state = MASTER_STATE_MAIN_MENU;
    config_ui_show_section_menu();

    return result;
}

static config_status_t config_ui_cancel_effects_section(void)
{
    if (g_master_config.effects_changes_pending) {
        // Restore effects configuration from EEPROM
        // TODO: Replace with actual EEPROM load function
        // effects_load_config_from_eeprom(&g_effect.config);

        LOG_INFO("Effects configuration changes discarded - restored from EEPROM");
        g_master_config.effects_changes_pending = false;
    }

    // Return to MAIN MENU - EFFECTS SECTION COMPLETE
    g_master_config.state = MASTER_STATE_MAIN_MENU;
    config_ui_show_section_menu();

    return CONFIG_OK;
}

static config_status_t config_ui_save_zone_section(void)
{
    config_status_t result = CONFIG_OK;

    // Save if we have an active session
    if (g_master_config.zones_session_active) {
        result = Zones_ExitProgramming(true);  // Save to EEPROM
        if (result == CONFIG_OK) {
            LOG_INFO("Zone programming saved to EEPROM successfully");
        } else {
            LOG_ERROR("Failed to save zone programming to EEPROM: %d", result);
        }
        g_master_config.zones_session_active = false;
    }

    // Handle navigation based on zone state
    zone_programming_state_t zone_state = Zone_Return_to_Section(g_blocking_state.fn_pressed);
    if (zone_state == ZONE_PROG_INACTIVE) {
        // Zone wants to return to main menu
        g_master_config.state = MASTER_STATE_MAIN_MENU;
        config_ui_show_section_menu();
    }

    return result;
}

static config_status_t config_ui_cancel_zone_section(void)
{
    config_status_t result = CONFIG_OK;

    // Cancel if we have an active session
    if (g_master_config.zones_session_active) {
        result = Zones_ExitProgramming(false);  // Discard changes
        LOG_INFO("Zone programming changes discarded");
        g_master_config.zones_session_active = false;
    }

    // Handle navigation based on zone state
    zone_programming_state_t zone_state = Zone_Return_to_Section(g_blocking_state.fn_pressed);
    if (zone_state == ZONE_PROG_INACTIVE) {
        // Zone wants to return to main menu
        g_master_config.state = MASTER_STATE_MAIN_MENU;
        Backlight_SetAll(0,0,0);
        config_ui_show_section_menu();
    }

    return result;
}

static config_status_t config_ui_save_power_section(void)
{
    config_status_t result = CONFIG_OK;

    if (g_master_config.power_changes_pending) {
        // Save power management settings directly to EEPROM using system_config.h API
        // TODO: Replace with actual calls like:
        // result = user_set_power_management(&power_settings);
        // result |= effects_save_lmp_config(&g_effect.config);

        if (result == CONFIG_OK) {
            LOG_INFO("Power management configuration saved to EEPROM");
            g_master_config.power_changes_pending = false;
        } else {
            LOG_ERROR("Failed to save power management to EEPROM: %d", result);
        }
    }

    // Return to MAIN MENU - POWER SECTION COMPLETE
    g_master_config.state = MASTER_STATE_MAIN_MENU;
    config_ui_show_section_menu();

    return result;
}

static config_status_t config_ui_cancel_power_section(void)
{
    if (g_master_config.power_changes_pending) {
        // Restore power management settings from EEPROM
        // TODO: Replace with actual calls like:
        // user_get_power_management(&power_settings);
        // effects_load_lmp_config(&g_effect.config);

        LOG_INFO("Power management changes discarded - restored from EEPROM");
        g_master_config.power_changes_pending = false;
    }

    // Return to MAIN MENU - POWER SECTION COMPLETE
    g_master_config.state = MASTER_STATE_MAIN_MENU;
    config_ui_show_section_menu();

    return CONFIG_OK;
}

static config_status_t config_ui_save_brightness_section(void)
{
    config_status_t result = CONFIG_OK;

    if (g_master_config.brightness_changes_pending) {
        // Save brightness settings directly to EEPROM
        // TODO: Replace with actual EEPROM save functions
        // result = user_save_brightness_presets_to_eeprom(&brightness_config);

        if (result == CONFIG_OK) {
            LOG_INFO("Brightness configuration saved to EEPROM");
            g_master_config.brightness_changes_pending = false;
        } else {
            LOG_ERROR("Failed to save brightness configuration to EEPROM: %d", result);
        }
    }

    // Return to MAIN MENU - BRIGHTNESS SECTION COMPLETE
    g_master_config.state = MASTER_STATE_MAIN_MENU;
    config_ui_show_section_menu();

    return result;
}

static config_status_t config_ui_cancel_brightness_section(void)
{
    if (g_master_config.brightness_changes_pending) {
        // Restore brightness settings from EEPROM
        // TODO: Replace with actual EEPROM load functions
        // user_load_brightness_presets_from_eeprom(&brightness_config);

        LOG_INFO("Brightness configuration changes discarded - restored from EEPROM");
        g_master_config.brightness_changes_pending = false;
    }

    // Return to MAIN MENU - BRIGHTNESS SECTION COMPLETE
    g_master_config.state = MASTER_STATE_MAIN_MENU;
    config_ui_show_section_menu();

    return CONFIG_OK;
}



/* ========================================================================
 * TIMEOUT AND ERROR HANDLING
 * ======================================================================== */

void config_ui_process(void)
{
    if (!g_master_initialized || !g_master_config.session_active) {
        return;
    }

    // Check blocking state timer
	blocking_state_process();

    // Check session timeout
    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed = current_time - g_master_config.last_activity_time;
    PM_RecordActivity();

    // Show warning at 9 minutes
    if (elapsed >= CONFIG_TIMEOUT_WARNING_MS && !g_master_config.timeout_warning_shown) {
        config_ui_flash_timeout_warning();
        g_master_config.timeout_warning_shown = true;
        LOG_WARNING("Configuration timeout warning - 1 minute remaining");
    }

    // Auto-exit at 10 minutes
    if (elapsed >= CONFIG_SESSION_TIMEOUT_MS) {
        LOG_WARNING("Configuration session timeout - auto-exiting without saving");
        config_ui_handle_timeout();
        return;
    }

    if (g_master_config.state == MASTER_STATE_EFFECTS_CONFIG &&
        g_master_config.effects_state == EFFECTS_STATE_HSV_EDIT) {

        uint32_t current_time = HAL_GetTick();
        uint32_t hsv_elapsed = current_time - g_master_config.effects_hsv_start_time;

        if (hsv_elapsed >= EFFECTS_HSV_WARNING_MS && hsv_elapsed < (EFFECTS_HSV_WARNING_MS + 1000)) {
            config_ui_flash_timeout_warning();
        }

        if (hsv_elapsed >= EFFECTS_HSV_TIMEOUT_MS) {
            LOG_WARNING("Effects HSV mode timeout - discarding changes");
            effects_exit_hsv_mode(false);
        }
    }

    // Update visual feedback
    config_ui_update_visual_feedback();

    // Process zone programming if active
    if (g_master_config.state == MASTER_STATE_ZONE_PROGRAMMING) {
        Zones_Process();
    }
}

void config_ui_handle_timeout(void)
{
    config_ui_flash_timeout_warning();
    config_ui_exit_master_mode(false);  // Exit without saving
}

config_status_t config_ui_handle_error(config_status_t error)
{
    g_master_config.error_count++;
    g_master_config.last_error = error;

    LOG_ERROR("Configuration error %d (count: %d)", error, g_master_config.error_count);

    // Flash error indication
    Backlight_SetAllRGB(CONFIG_CANCEL_COLOR);
    HAL_Delay(200);

    // Return to appropriate state
    if (g_master_config.state != MASTER_STATE_MAIN_MENU) {
        config_ui_return_to_menu();
    } else {
        config_ui_show_section_menu();
    }

    // Force exit if too many errors
    if (g_master_config.error_count >= CONFIG_MAX_SESSION_ERRORS) {
        LOG_ERROR("Too many configuration errors - force exiting");
        config_ui_exit_master_mode(false);
    }

    return error;
}

/* ========================================================================
 * VISUAL FEEDBACK SYSTEM
 * ======================================================================== */

void config_ui_update_visual_feedback(void)
{
    uint32_t current_time = HAL_GetTick();

    if ((current_time - g_master_config.last_visual_update) < CONFIG_VISUAL_UPDATE_INTERVAL_MS) {
        return;
    }
    g_master_config.last_visual_update = current_time;
    blocking_state_update_visual();
}

void config_ui_flash_enter_confirmation(void)
{
    // Quick cyan flash to confirm entry
    Backlight_SetAllRGB(CONFIG_ACTIVE_SECTION_COLOR);
    HAL_Delay(200);
}

void config_ui_flash_save_result(bool success)
{
    Backlight_RGB_t flash_color = success ? CONFIG_SAVE_SUCCESS_COLOR : CONFIG_CANCEL_COLOR;
    uint8_t flash_count = success ? 1 : 2;

    for (uint8_t i = 0; i < flash_count; i++) {
        Backlight_SetAllRGB(flash_color);
        HAL_Delay(100);
        Backlight_SetAll(0, 0, 0);
        if (i < flash_count - 1) HAL_Delay(100);
    }
}

void config_ui_flash_timeout_warning(void)
{
    // Yellow flash for timeout warning
    for (uint8_t i = 0; i < 2; i++) {
        Backlight_SetAllRGB(CONFIG_WARNING_COLOR);
        HAL_Delay(200);
        Backlight_SetAll(0, 0, 0);
        HAL_Delay(200);
    }
}

/* ========================================================================
 * UTILITY AND STATUS FUNCTIONS
 * ======================================================================== */

void config_ui_update_activity(void)
{
    g_master_config.last_activity_time = HAL_GetTick();
    g_master_config.timeout_warning_shown = false;  // Reset warning
}

bool config_ui_is_active(void)
{
    return g_config_ui_active;
}

config_ui_state_t config_ui_get_state(void)
{
    // Convert master state to legacy state enum for compatibility
    switch (g_master_config.state) {
        case MASTER_STATE_IDLE:
            return CONFIG_UI_IDLE;
        case MASTER_STATE_MAIN_MENU:
            return CONFIG_UI_MENU;
        case MASTER_STATE_ZONE_PROGRAMMING:
            return CONFIG_UI_ZONE_CONFIG;
        case MASTER_STATE_EFFECTS_CONFIG:
            return CONFIG_UI_EFFECTS_CONFIG;
        case MASTER_STATE_POWER_CONFIG:
            return CONFIG_UI_POWER_CONFIG;
        case MASTER_STATE_BRIGHTNESS_CONFIG:
            return CONFIG_UI_BRIGHTNESS_CONFIG;
        default:
            return CONFIG_UI_IDLE;
    }
}

bool config_ui_has_pending_changes(void)
{
    return g_master_config.has_unsaved_changes;
}

uint32_t config_ui_get_session_duration(void)
{
    if (!g_master_config.session_active) return 0;
    return HAL_GetTick() - g_master_config.session_start_time;
}

uint32_t config_ui_get_timeout_remaining(void)
{
    if (!g_master_config.session_active) return 0;

    uint32_t elapsed = HAL_GetTick() - g_master_config.last_activity_time;
    if (elapsed >= CONFIG_SESSION_TIMEOUT_MS) return 0;

    return CONFIG_SESSION_TIMEOUT_MS - elapsed;
}

/* ========================================================================
 * LEGACY COMPATIBILITY FUNCTIONS
 * ======================================================================== */

// These functions provide compatibility with existing code

config_status_t config_ui_enter_system_mode(void)
{
    // Legacy function - redirect to master mode entry
    return config_ui_enter_master_mode();
}

config_status_t config_ui_exit_system_mode(bool save_changes)
{
    // Legacy function - redirect to master mode exit
    return config_ui_exit_master_mode(save_changes);
}

void config_ui_force_exit(void)
{
    LOG_WARNING("Force exiting configuration system");
    g_config_ui_active = false;
    g_master_config.state = MASTER_STATE_IDLE;
    g_master_config.session_active = false;
    Effects_StartRuntimeEffect();
}

const config_ui_context_t* config_ui_get_context(void)
{
    return NULL;
}

/* ========================================================================
 * EFFECTS COLOR SELECTION
 * ======================================================================== */
/**
 * @brief Convert column to color index for effects
 */
static uint8_t effects_col_to_color_index(uint8_t col)
{
    return (col >= 1 && col <= 12) ? (col - 1) : 0xFF;
}

/**
 * @brief Check if a color is editable in current mode
 */
static bool effects_is_color_editable(uint8_t color_index)
{
    if (g_effect.config.runtime_mode == EFFECT_MODE_TRANSITION_BREATHING) {
        return (color_index < transition_color_count);
    } else {
        return (color_index == 11);
    }
}

/**
 * @brief Check if a color index is valid for current mode
 */
static bool effects_is_color_valid(uint8_t color_index)
{
    if (g_effect.config.runtime_mode == EFFECT_MODE_TRANSITION_BREATHING) {
        return (color_index < transition_color_count);
    } else {
        return (color_index < breathing_color_count);
    }
}

/**
 * @brief Update the complete effects configuration display
 */
static void update_effects_config_display(void)
{
    show_effects_mode_indicators();
    show_effects_color_indicators();
}

/**
 * @brief Show effect mode indicators (Static/Breathing/Transition)
 */
static void show_effects_mode_indicators(void)
{
    for (uint8_t col = 1; col <= 3; col++) {
        effect_mode_t mode;
        switch(col) {
            case 1: mode = EFFECT_MODE_STATIC; break;
            case 2: mode = EFFECT_MODE_BREATHING; break;
            case 3: mode = EFFECT_MODE_TRANSITION_BREATHING; break;
            default: continue;
        }

        Backlight_RGB_t color = (mode == g_effect.config.runtime_mode) ?
            CONFIG_ACTIVE_SECTION_COLOR : CONFIG_SECTION_INDICATOR_COLOR;
        Backlight_SetKeyRGB(1, col, color);
    }
}

/**
 * @brief Show color selection indicators based on current mode
 */
static void show_effects_color_indicators(void)
{
    // Clear all color keys first
    for (uint8_t col = 1; col <= 12; col++) {
        Backlight_SetKeyRGB(EFFECTS_COLOR_KEY_ROW, col, BACKLIGHT_OFF);
    }

    if (g_effect.config.runtime_mode == EFFECT_MODE_TRANSITION_BREATHING) {
        // Transition mode: show transition_colors[0-9] on asdfghjkl (cols 1-10)
        for (uint8_t i = 0; i < transition_color_count && i < 10; i++) {
            Backlight_RGB_t color = Color_GetTransition(i);
            bool is_selected = (i == g_effect.config.runtime_color_index);

            Backlight_RGB_t display_color = is_selected ?
                Color_Bright(color) : Color_Scale(color, 40);

            Backlight_SetKeyRGB(EFFECTS_COLOR_KEY_ROW, i + 1, display_color);
        }
    } else {
        // Static/Breathing mode: show breathing_colors[0-11] on asdfghjkl;' (cols 1-12)
        for (uint8_t i = 0; i < breathing_color_count && i < 12; i++) {
            Backlight_RGB_t color = Color_GetBreathing(i);
            bool is_selected = (i == g_effect.config.runtime_color_index);

            Backlight_RGB_t display_color = is_selected ?
                Color_Bright(color) : Color_Scale(color, 60);

            // Highlight editable color (index 11) slightly brighter
            if (i == 11 && !is_selected) {
                display_color = Color_Scale(color, 100);
            }

            Backlight_SetKeyRGB(EFFECTS_COLOR_KEY_ROW, i + 1, display_color);
        }
    }
}

/**
 * @brief Show HSV editing display with current color
 */
static void show_effects_hsv_display(void)
{
    Backlight_RGB_t current_color = Backlight_HSVtoRGB(
        g_master_config.effects_hsv.hue,
        g_master_config.effects_hsv.saturation,
        g_master_config.effects_hsv.value
    );

    // Set dim background
    for (uint8_t row = 0; row < 6; row++) {
        for (uint8_t col = 0; col < 14; col++) {
            if (Backlight_HasLED(row, col)) {
                Backlight_SetKeyRGB(row, col, (Backlight_RGB_t){2, 2, 2});
            }
        }
    }

    // Show current color on target key
    uint8_t target_col = g_master_config.effects_editing_color_index + 1;
    Backlight_SetKeyRGB(EFFECTS_COLOR_KEY_ROW, target_col, current_color);

}

/**
 * @brief Enter HSV editing mode for a specific color
 */
static config_status_t effects_enter_hsv_mode(uint8_t color_index)
{
    if (!effects_is_color_editable(color_index)) {
        return CONFIG_INVALID_PARAMETER;
    }

    g_master_config.effects_state = EFFECTS_STATE_HSV_EDIT;
    g_master_config.effects_editing_color_index = color_index;
    g_master_config.effects_hsv_start_time = HAL_GetTick();
    g_master_config.right_alt_pressed = false;

    // Initialize HSV values from current color
    Backlight_RGB_t current_color;
    if (g_effect.config.runtime_mode == EFFECT_MODE_TRANSITION_BREATHING) {
        current_color = Color_GetTransition(color_index);
    } else {
        current_color = Color_GetBreathing(color_index);
    }

    // Start with reasonable HSV defaults
    g_master_config.effects_hsv.hue = 0;
    g_master_config.effects_hsv.saturation = 100;
    g_master_config.effects_hsv.value = 80;
    g_master_config.effects_hsv.hue_step = 5;
    g_master_config.effects_hsv.sv_step = 5;

    show_effects_hsv_display();

    LOG_INFO("Entered HSV edit mode for color index %d", color_index);
    return CONFIG_OK;
}

/**
 * @brief Exit HSV editing mode and apply/discard changes
 */
static void effects_exit_hsv_mode(bool apply_changes)
{
    if (g_master_config.effects_state != EFFECTS_STATE_HSV_EDIT) {
        return;
    }

    if (apply_changes) {
        Backlight_RGB_t new_color = Backlight_HSVtoRGB(
            g_master_config.effects_hsv.hue,
            g_master_config.effects_hsv.saturation,
            g_master_config.effects_hsv.value
        );

        // Direct array modification - simple and clean!
        if (g_effect.config.runtime_mode == EFFECT_MODE_TRANSITION_BREATHING) {
            transition_colors[g_master_config.effects_editing_color_index] = new_color;
        } else {
            breathing_colors[11] = new_color;
        }

        g_master_config.effects_changes_pending = true;
        g_master_config.has_unsaved_changes = true;
        g_master_config.sections_modified_mask |= (1 << 1);
    }
    g_master_config.right_alt_pressed = false;
    g_master_config.effects_state = EFFECTS_STATE_MODE_AND_COLOR;
    update_effects_config_display();
}

static config_status_t effects_adjust_hue(int8_t direction)
{
    if (g_master_config.effects_state != EFFECTS_STATE_HSV_EDIT) {
        return CONFIG_INVALID_PARAMETER;
    }

    int16_t new_hue = g_master_config.effects_hsv.hue + (direction * g_master_config.effects_hsv.hue_step);
    if (new_hue < 0) new_hue += 360;
    else if (new_hue >= 360) new_hue -= 360;

    g_master_config.effects_hsv.hue = (uint16_t)new_hue;
    show_effects_hsv_display();
    return CONFIG_OK;
}

static config_status_t effects_adjust_saturation(int8_t direction)
{
    if (g_master_config.effects_state != EFFECTS_STATE_HSV_EDIT) {
        return CONFIG_INVALID_PARAMETER;
    }

    int16_t new_sat = g_master_config.effects_hsv.saturation + (direction * g_master_config.effects_hsv.sv_step);
    if (new_sat < 0) new_sat = 0;
    else if (new_sat > 100) new_sat = 100;

    g_master_config.effects_hsv.saturation = (uint8_t)new_sat;
    show_effects_hsv_display();
    return CONFIG_OK;
}

static config_status_t effects_adjust_value(int8_t direction)
{
    if (g_master_config.effects_state != EFFECTS_STATE_HSV_EDIT) {
        return CONFIG_INVALID_PARAMETER;
    }

    int16_t new_val = g_master_config.effects_hsv.value + (direction * g_master_config.effects_hsv.sv_step);
    if (new_val < 1) new_val = 1;
    else if (new_val > 100) new_val = 100;

    g_master_config.effects_hsv.value = (uint8_t)new_val;
    show_effects_hsv_display();
    return CONFIG_OK;
}

static bool handle_effects_mode_selection(uint8_t col)
{
    effect_mode_t new_mode;
    switch(col) {
        case 1: new_mode = EFFECT_MODE_STATIC; break;
        case 2: new_mode = EFFECT_MODE_BREATHING; break;
        case 3: new_mode = EFFECT_MODE_TRANSITION_BREATHING; break;
        default: return false;
    }

    if (new_mode != g_effect.config.runtime_mode) {
        g_effect.config.runtime_mode = new_mode;
        g_effect.config.runtime_color_index = 0; // Reset to first color

        g_master_config.effects_changes_pending = true;
        g_master_config.has_unsaved_changes = true;
        g_master_config.sections_modified_mask |= (1 << 1);
    }

    update_effects_config_display();
    return true;
}

static bool handle_effects_color_selection(uint8_t col)
{
    uint8_t color_index = effects_col_to_color_index(col);

    if (!effects_is_color_valid(color_index)) {
        return false;
    }

    if (color_index != g_effect.config.runtime_color_index) {
        g_effect.config.runtime_color_index = color_index;
        g_master_config.effects_changes_pending = true;
        g_master_config.has_unsaved_changes = true;
        g_master_config.sections_modified_mask |= (1 << 1);
    }

    show_effects_color_indicators();
    return true;
}

static bool handle_effects_hsv_keys(uint8_t row, uint8_t col)
{
    uint8_t keycode = layers[BASE_LAYER][row][col];

    switch (keycode) {
        case KC_W: effects_adjust_value(1); return true;
        case KC_S: effects_adjust_value(-1); return true;
        case KC_A: effects_adjust_saturation(-1); return true;
        case KC_D: effects_adjust_saturation(1); return true;
        case KC_Q: effects_adjust_hue(-1); return true;
        case KC_E: effects_adjust_hue(1); return true;
        default: return false;
    }
}

static bool handle_effects_right_alt_combinations(uint8_t row, uint8_t col)
{
    if (g_master_config.state != MASTER_STATE_EFFECTS_CONFIG) {
        return false;
    }

    // R_Alt + Enter = Apply HSV changes
    if (layers[BASE_LAYER][row][col] == KC_ENTER &&
        g_master_config.effects_state == EFFECTS_STATE_HSV_EDIT) {
        effects_exit_hsv_mode(true);
        return true;
    }

    // R_Alt + Backspace = Cancel HSV changes
    if (layers[BASE_LAYER][row][col] == KC_BSPACE &&
        g_master_config.effects_state == EFFECTS_STATE_HSV_EDIT) {
        effects_exit_hsv_mode(false);
        return true;
    }

    // R_Alt + color keys = Enter HSV edit mode
    if (row == EFFECTS_COLOR_KEY_ROW && col >= 1 && col <= 12) {
        uint8_t color_index = effects_col_to_color_index(col);
        if (effects_is_color_editable(color_index)) {
            return (effects_enter_hsv_mode(color_index) == CONFIG_OK);
        }
    }

    return false;
}
