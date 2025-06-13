/*
 * config_ui.c - Complete Configuration UI Implementation
 *
 *  Created on: Jun 13, 2025
 *      Author: bettysidepiece
 */

#include "config_ui.h"
#include "colors.h"
#include "led_mapping.h"
#include "backlight.h"
#include "effects.h"
#include "zones.h"
#include "logger.h"
#include "pmsm.h"
#include "keys.h"
#include <string.h>

/* ========================================================================
 * PRIVATE CONSTANTS
 * ======================================================================== */

#define CONFIG_SECTION_COLOR        COLOR_GREEN
#define CONFIG_ACTIVE_COLOR         COLOR_CYAN
#define CONFIG_SAVE_COLOR           COLOR_GREEN
#define CONFIG_CANCEL_COLOR         COLOR_RED
#define CONFIG_WARNING_COLOR        COLOR_YELLOW
#define CONFIG_DIM_COLOR            Color_Dim(COLOR_WARM_WHITE)

#define CONFIG_FLASH_DURATION_MS    300
#define CONFIG_BREATH_MIN           10
#define CONFIG_BREATH_MAX           60

#define CONFIG_ENTRY_HOLD_TIME_MS   5000
#define CONFIG_AUTO_TIMEOUT_MS      600000  // 10 minutes
#define CONFIG_TIMEOUT_WARNING_MS   540000  // 9 minutes
#define CONFIG_VISUAL_UPDATE_MS     100
#define CONFIG_MAX_ERRORS           5

#define CONFIG_SECTION_ROW          1       // Number row

/* ========================================================================
 * GLOBAL STATE
 * ======================================================================== */

static config_ui_context_t g_config_ctx = {0};
static bool g_initialized = false;

// Simple global flag for blocking HID reports
bool g_config_ui_active = false;

/* ========================================================================
 * PRIVATE FUNCTION PROTOTYPES
 * ======================================================================== */

// Core system functions
static config_status_t setup_config_mode_environment(void);
static config_status_t restore_normal_environment(void);
static void update_activity_time(void);
static bool check_timeout_conditions(void);

// Entry sequence
static void reset_entry_sequence(void);
static bool update_entry_sequence(uint8_t keycode, bool pressed);
static bool is_entry_sequence_complete(void);

// Key processing
static bool is_universal_command(uint8_t row, uint8_t col, bool fn_pressed);
static universal_command_t detect_universal_command(uint8_t row, uint8_t col, bool fn_pressed);
static config_status_t process_universal_command(universal_command_t cmd);
static bool is_section_key(uint8_t row, uint8_t col);
static uint8_t get_section_from_key(uint8_t row, uint8_t col);

// State management
static config_status_t enter_state(config_ui_state_t new_state);
static config_status_t exit_current_state(void);

// Section FSM implementations
static config_status_t process_menu_state(uint8_t row, uint8_t col, bool pressed);
static config_status_t process_zone_config_fsm(uint8_t row, uint8_t col, bool pressed);
static config_status_t process_effects_config_fsm(uint8_t row, uint8_t col, bool pressed);
static config_status_t process_power_config_fsm(uint8_t row, uint8_t col, bool pressed);
static config_status_t process_brightness_config_fsm(uint8_t row, uint8_t col, bool pressed);

// Visual feedback
static void setup_config_mode_lighting(void);
static void update_breathing_effect(void);
static void show_section_indicators(void);
static void flash_confirmation(Backlight_RGB_t color, uint16_t duration_ms);

// Error handling
static config_status_t handle_error(config_status_t error);

/* ========================================================================
 * PUBLIC API IMPLEMENTATION
 * ======================================================================== */

config_status_t config_ui_init(void)
{
    if (g_initialized) {
        return CONFIG_OK;
    }

    memset(&g_config_ctx, 0, sizeof(g_config_ctx));

    g_config_ctx.current_state = CONFIG_UI_IDLE;
    g_config_ctx.session.timeout_duration_ms = CONFIG_AUTO_TIMEOUT_MS;
    g_config_ctx.session.auto_timeout_enabled = true;

    g_config_ui_active = false;
    reset_entry_sequence();

    g_initialized = true;
    LOG_INFO("Configuration UI initialized");

    return CONFIG_OK;
}

void config_ui_process(void)
{
    if (!g_initialized || g_config_ctx.current_state == CONFIG_UI_IDLE) {
        return;
    }

    // Check timeout
    if (check_timeout_conditions()) {
        LOG_INFO("Configuration timeout - auto-exiting");
        config_ui_handle_timeout();
        return;
    }

    // Process pending commands
    if (g_config_ctx.pending_command != UNIVERSAL_CMD_NONE) {
        config_status_t result = process_universal_command(g_config_ctx.pending_command);
        g_config_ctx.pending_command = UNIVERSAL_CMD_NONE;

        if (result != CONFIG_OK) {
            handle_error(result);
        }
    }

    // Update visual feedback
    config_ui_update_visual_feedback();
}

void check_config_ui_functions(void)
{
    if (g_config_ctx.current_state != CONFIG_UI_IDLE) {
        return;
    }

    if (g_config_ctx.entry_sequence.sequence_complete && is_entry_sequence_complete()) {
        LOG_INFO("Configuration entry sequence complete");
        config_status_t result = config_ui_enter_system_mode();
        if (result != CONFIG_OK) {
            LOG_ERROR("Failed to enter config mode: %d", result);
            reset_entry_sequence();
        }
    }
}

bool config_ui_handle_key_event(uint8_t row, uint8_t col, bool pressed)
{
    if (!g_config_ui_active || !pressed) {
        return false;
    }

    update_activity_time();

    // Check universal commands first
    extern kb_state_t kb_state;
    bool fn_pressed = kb_state.fn_pressed;

    if (is_universal_command(row, col, fn_pressed)) {
        g_config_ctx.pending_command = detect_universal_command(row, col, fn_pressed);
        return true;
    }

    // Route to state handlers
    config_status_t result = CONFIG_OK;
    switch (g_config_ctx.current_state) {
        case CONFIG_UI_MENU:
            result = process_menu_state(row, col, pressed);
            break;
        case CONFIG_UI_ZONE_CONFIG:
            result = process_zone_config_fsm(row, col, pressed);
            break;
        case CONFIG_UI_EFFECTS_CONFIG:
            result = process_effects_config_fsm(row, col, pressed);
            break;
        case CONFIG_UI_POWER_CONFIG:
            result = process_power_config_fsm(row, col, pressed);
            break;
        case CONFIG_UI_BRIGHTNESS_CONFIG:
            result = process_brightness_config_fsm(row, col, pressed);
            break;
        default:
            result = CONFIG_ERROR;
            break;
    }

    if (result != CONFIG_OK) {
        handle_error(result);
    }

    return true;
}

bool config_ui_is_active(void)
{
    return g_config_ui_active;
}

config_ui_state_t config_ui_get_state(void)
{
    return g_initialized ? g_config_ctx.current_state : CONFIG_UI_IDLE;
}

config_status_t config_ui_enter_system_mode(void)
{
    if (!g_initialized || g_config_ui_active) {
        return CONFIG_INVALID_PARAMETER;
    }

    LOG_INFO("Entering configuration mode");

    g_config_ui_active = true;

    config_status_t result = setup_config_mode_environment();
    if (result != CONFIG_OK) {
        g_config_ui_active = false;
        return result;
    }

    // Initialize session
    g_config_ctx.session.session_start_time = HAL_GetTick();
    g_config_ctx.session.last_activity_time = g_config_ctx.session.session_start_time;
    g_config_ctx.session.session_active = true;
    g_config_ctx.session.changes_pending = false;
    g_config_ctx.session.error_count = 0;

    result = enter_state(CONFIG_UI_MENU);
    if (result != CONFIG_OK) {
        restore_normal_environment();
        g_config_ui_active = false;
        return result;
    }

    flash_confirmation(CONFIG_ACTIVE_COLOR, 200);
    LOG_INFO("Configuration mode entered");

    return CONFIG_OK;
}

config_status_t config_ui_exit_system_mode(bool save_changes)
{
    if (!g_config_ui_active) {
        return CONFIG_INVALID_PARAMETER;
    }

    LOG_INFO("Exiting configuration mode (save=%d)", save_changes);

    if (save_changes && g_config_ctx.session.changes_pending) {
        // TODO: Implement actual save logic
        flash_confirmation(CONFIG_SAVE_COLOR, CONFIG_FLASH_DURATION_MS);
        LOG_INFO("Configuration saved");
    } else if (g_config_ctx.session.changes_pending) {
        flash_confirmation(CONFIG_CANCEL_COLOR, CONFIG_FLASH_DURATION_MS);
        LOG_INFO("Changes discarded");
    }

    exit_current_state();
    restore_normal_environment();

    g_config_ui_active = false;
    g_config_ctx.current_state = CONFIG_UI_IDLE;
    g_config_ctx.session.session_active = false;
    g_config_ctx.session.changes_pending = false;

    reset_entry_sequence();

    return CONFIG_OK;
}

void config_ui_force_exit(void)
{
    LOG_WARNING("Force exiting configuration mode");
    g_config_ui_active = false;
    g_config_ctx.current_state = CONFIG_UI_IDLE;
    Effects_StartRuntimeEffect();
}

/* ========================================================================
 * SECTION ENTRY FUNCTIONS
 * ======================================================================== */

config_status_t config_ui_enter_zone_config(void)
{
    LOG_INFO("Entering zone configuration");

    // Initialize zone FSM
    g_config_ctx.zone_fsm.state = SECTION_FSM_ENTRY;
    g_config_ctx.zone_fsm.selected_slot = 0;
    g_config_ctx.zone_fsm.selection_count = 0;

    // Set default HSV
    g_config_ctx.zone_fsm.hsv_current.hue = 30;
    g_config_ctx.zone_fsm.hsv_current.saturation = 100;
    g_config_ctx.zone_fsm.hsv_current.value = 80;

    // Read zone flags
    for (uint8_t i = 0; i < 8; i++) {
        g_config_ctx.zone_fsm.slot_occupied[i] = Zones_IsSlotOccupied(i);
        g_config_ctx.zone_fsm.slot_key_counts[i] = Zones_GetKeyCount(i);
    }

    return enter_state(CONFIG_UI_ZONE_CONFIG);
}

config_status_t config_ui_enter_effects_config(void)
{
    LOG_INFO("Entering effects configuration");

    // Initialize effects FSM
    g_config_ctx.effects_fsm.state = SECTION_FSM_ENTRY;
    g_config_ctx.effects_fsm.effect_type = 2; // breathing
    g_config_ctx.effects_fsm.cycle_time_ms = 3000;
    g_config_ctx.effects_fsm.brightness_min = 10;
    g_config_ctx.effects_fsm.brightness_max = 80;
    g_config_ctx.effects_fsm.transition_duration_ms = 1000;

    // Initialize color presets (6 colors for transitions)
    g_config_ctx.effects_fsm.color_presets[0] = 0;   // Red
    g_config_ctx.effects_fsm.color_presets[1] = 60;  // Yellow
    g_config_ctx.effects_fsm.color_presets[2] = 120; // Green
    g_config_ctx.effects_fsm.color_presets[3] = 180; // Cyan
    g_config_ctx.effects_fsm.color_presets[4] = 240; // Blue
    g_config_ctx.effects_fsm.color_presets[5] = 300; // Magenta

    g_config_ctx.effects_fsm.preview_active = false;

    return enter_state(CONFIG_UI_EFFECTS_CONFIG);
}

config_status_t config_ui_enter_power_config(void)
{
    LOG_INFO("Entering power management");

    // Initialize power FSM - unified modes
    g_config_ctx.power_fsm.state = SECTION_FSM_ENTRY;
    g_config_ctx.power_fsm.usb_mode = 2;         // dim
    g_config_ctx.power_fsm.battery_mode = 2;     // 2min (same as usb_mode values)
    g_config_ctx.power_fsm.usb_timeout_s = 300;  // 5 minutes
    g_config_ctx.power_fsm.battery_timeout_s = 120; // 2 minutes
    g_config_ctx.power_fsm.dim_percentage = 20;
    g_config_ctx.power_fsm.wake_on_keypress = true;
    g_config_ctx.power_fsm.preview_active = false;

    return enter_state(CONFIG_UI_POWER_CONFIG);
}

config_status_t config_ui_enter_brightness_config(void)
{
    LOG_INFO("Entering brightness control");

    // Initialize brightness FSM - simplified
    g_config_ctx.brightness_fsm.state = SECTION_FSM_ENTRY;
    g_config_ctx.brightness_fsm.global_max = Backlight_GetCurrentMaxBrightness();
    g_config_ctx.brightness_fsm.global_min = 10;
    g_config_ctx.brightness_fsm.startup_mode = 2; // preset

    return enter_state(CONFIG_UI_BRIGHTNESS_CONFIG);
}

/* ========================================================================
 * UNIVERSAL COMMANDS
 * ======================================================================== */

config_status_t config_ui_execute_save(void)
{
    LOG_INFO("Executing save command");

    // TODO: Implement section-specific save logic
    config_status_t result = CONFIG_OK;

    if (result == CONFIG_OK) {
        g_config_ctx.session.changes_pending = false;
        flash_confirmation(CONFIG_SAVE_COLOR, CONFIG_FLASH_DURATION_MS);
        return config_ui_exit_system_mode(true);
    } else {
        flash_confirmation(CONFIG_CANCEL_COLOR, CONFIG_FLASH_DURATION_MS);
        return result;
    }
}

config_status_t config_ui_execute_cancel(void)
{
    LOG_INFO("Executing cancel command");

    g_config_ctx.session.changes_pending = false;
    flash_confirmation(CONFIG_CANCEL_COLOR, CONFIG_FLASH_DURATION_MS);

    return config_ui_exit_system_mode(false);
}

config_status_t config_ui_execute_menu(void)
{
    LOG_DEBUG("Returning to main menu");
    return enter_state(CONFIG_UI_MENU);
}

void config_ui_handle_timeout(void)
{
    LOG_WARNING("Configuration timeout - auto-exiting");
    flash_confirmation(CONFIG_WARNING_COLOR, CONFIG_FLASH_DURATION_MS);
    config_ui_exit_system_mode(false);
}

/* ========================================================================
 * VISUAL FEEDBACK
 * ======================================================================== */

void config_ui_update_visual_feedback(void)
{
    uint32_t current_time = HAL_GetTick();

    if ((current_time - g_config_ctx.visual.last_visual_update) < CONFIG_VISUAL_UPDATE_MS) {
        return;
    }
    g_config_ctx.visual.last_visual_update = current_time;

    if (g_config_ctx.visual.config_mode_breathing) {
        update_breathing_effect();
    }

    if (g_config_ctx.visual.section_indicators_active) {
        show_section_indicators();
    }
}

void config_ui_show_menu_indicators(void)
{
    // Dim all keys
    Backlight_RGB_t dim_color = CONFIG_DIM_COLOR;
    Backlight_SetAllRGB(dim_color);

    // Highlight section keys 1-4
    for (uint8_t col = 1; col <= 4; col++) {
        const rgb_led_mapping_t* led = get_led_for_key(CONFIG_SECTION_ROW, col);
        if (led) {
            Backlight_SetKeyRGB(CONFIG_SECTION_ROW, col, CONFIG_SECTION_COLOR);
        }
    }

    g_config_ctx.visual.section_indicators_active = true;
}

/* ========================================================================
 * DIAGNOSTICS
 * ======================================================================== */

const config_ui_context_t* config_ui_get_context(void)
{
    return &g_config_ctx;
}

uint32_t config_ui_get_session_duration(void)
{
    if (!g_config_ctx.session.session_active) return 0;
    return HAL_GetTick() - g_config_ctx.session.session_start_time;
}

uint32_t config_ui_get_timeout_remaining(void)
{
    if (!g_config_ctx.session.session_active) return 0;

    uint32_t elapsed = HAL_GetTick() - g_config_ctx.session.last_activity_time;
    if (elapsed >= CONFIG_AUTO_TIMEOUT_MS) return 0;

    return CONFIG_AUTO_TIMEOUT_MS - elapsed;
}

bool config_ui_has_pending_changes(void)
{
    return g_config_ctx.session.changes_pending;
}

/* ========================================================================
 * ENTRY SEQUENCE INTEGRATION
 * ======================================================================== */

bool config_ui_update_entry_sequence(uint8_t keycode, bool pressed)
{
    if (g_config_ctx.current_state != CONFIG_UI_IDLE) {
        return false;
    }

    uint32_t current_time = getMicroseconds();
    config_entry_sequence_t *seq = &g_config_ctx.entry_sequence;

    // Track Fn key state (handled externally)
    extern kb_state_t kb_state;
    seq->fn_pressed = kb_state.fn_pressed;

    // Handle sequence keys
    bool handled = update_entry_sequence(keycode, pressed);

    // Check for sequence completion
    if (seq->fn_pressed && seq->space_pressed && seq->enter_pressed) {
        if (seq->sequence_start_time == 0) {
            seq->sequence_start_time = current_time;
        }

        uint32_t hold_duration = current_time - seq->sequence_start_time;
        if (hold_duration >= (CONFIG_ENTRY_HOLD_TIME_MS * 1000)) {
            seq->sequence_complete = true;
            LOG_INFO("Config entry sequence complete");
        }
    } else if (seq->sequence_start_time != 0) {
        reset_entry_sequence();
    }

    return handled;
}

/* ========================================================================
 * PRIVATE IMPLEMENTATIONS
 * ======================================================================== */

static config_status_t setup_config_mode_environment(void)
{
    LOG_DEBUG("Setting up config mode environment");

    // Setup visual environment
    setup_config_mode_lighting();

    return CONFIG_OK;
}

static config_status_t restore_normal_environment(void)
{
    LOG_DEBUG("Restoring normal environment");

    // Restore normal effects
    Effects_StartRuntimeEffect();

    // Reset visual state
    g_config_ctx.visual.config_mode_breathing = false;
    g_config_ctx.visual.section_indicators_active = false;

    return CONFIG_OK;
}

static void update_activity_time(void)
{
    g_config_ctx.session.last_activity_time = HAL_GetTick();
}

static bool check_timeout_conditions(void)
{
    if (!g_config_ctx.session.auto_timeout_enabled || !g_config_ctx.session.session_active) {
        return false;
    }

    uint32_t elapsed = HAL_GetTick() - g_config_ctx.session.last_activity_time;

    // Warning at 9 minutes
    if (elapsed >= CONFIG_TIMEOUT_WARNING_MS && elapsed < CONFIG_AUTO_TIMEOUT_MS) {
        static bool warning_shown = false;
        if (!warning_shown) {
            config_ui_flash_timeout_warning();
            LOG_WARNING("Timeout warning - 1 minute remaining");
            warning_shown = true;
        }
    }

    return (elapsed >= CONFIG_AUTO_TIMEOUT_MS);
}

static void reset_entry_sequence(void)
{
    memset(&g_config_ctx.entry_sequence, 0, sizeof(g_config_ctx.entry_sequence));
}

static bool update_entry_sequence(uint8_t keycode, bool pressed)
{
    config_entry_sequence_t *seq = &g_config_ctx.entry_sequence;

    if (keycode == KC_SPACE) {
        seq->space_pressed = pressed;
        return true;
    }

    if (keycode == KC_ENTER) {
        seq->enter_pressed = pressed;
        return true;
    }

    return false;
}

static bool is_entry_sequence_complete(void)
{
    config_entry_sequence_t *seq = &g_config_ctx.entry_sequence;
    return (seq->sequence_complete && !seq->fn_pressed && !seq->space_pressed && !seq->enter_pressed);
}

static bool is_universal_command(uint8_t row, uint8_t col, bool fn_pressed)
{
    if (!fn_pressed) return false;

    // Fn + S (Save)
    if (row == 3 && col == 2) return true;
    // Fn + L (Cancel)
    if (row == 3 && col == 9) return true;
    // Fn + Esc (Menu)
    if (row == 0 && col == 0) return true;

    return false;
}

static universal_command_t detect_universal_command(uint8_t row, uint8_t col, bool fn_pressed)
{
    if (!fn_pressed) return UNIVERSAL_CMD_NONE;

    if (row == 3 && col == 2) return UNIVERSAL_CMD_SAVE;
    if (row == 3 && col == 9) return UNIVERSAL_CMD_CANCEL;
    if (row == 0 && col == 0) return UNIVERSAL_CMD_MENU;

    return UNIVERSAL_CMD_NONE;
}

static config_status_t process_universal_command(universal_command_t cmd)
{
    switch (cmd) {
        case UNIVERSAL_CMD_SAVE:
            return config_ui_execute_save();
        case UNIVERSAL_CMD_CANCEL:
            return config_ui_execute_cancel();
        case UNIVERSAL_CMD_MENU:
            return config_ui_execute_menu();
        case UNIVERSAL_CMD_TIMEOUT:
            config_ui_handle_timeout();
            return CONFIG_OK;
        default:
            return CONFIG_INVALID_PARAMETER;
    }
}

static bool is_section_key(uint8_t row, uint8_t col)
{
    return (row == CONFIG_SECTION_ROW && col >= 1 && col <= 4);
}

static uint8_t get_section_from_key(uint8_t row, uint8_t col)
{
    if (row != CONFIG_SECTION_ROW || col < 1 || col > 4) return 0;
    return col;
}

static config_status_t enter_state(config_ui_state_t new_state)
{
    config_ui_state_t old_state = g_config_ctx.current_state;

    if (old_state != CONFIG_UI_IDLE) {
        exit_current_state();
    }

    g_config_ctx.previous_state = old_state;
    g_config_ctx.current_state = new_state;

    LOG_DEBUG("Config state: %d -> %d", old_state, new_state);

    // State entry actions
    switch (new_state) {
        case CONFIG_UI_MENU:
            config_ui_show_menu_indicators();
            g_config_ctx.visual.config_mode_breathing = true;
            break;
        case CONFIG_UI_ZONE_CONFIG:
        case CONFIG_UI_EFFECTS_CONFIG:
        case CONFIG_UI_POWER_CONFIG:
        case CONFIG_UI_BRIGHTNESS_CONFIG:
            g_config_ctx.visual.section_indicators_active = false;
            break;
        default:
            break;
    }

    update_activity_time();
    return CONFIG_OK;
}

static config_status_t exit_current_state(void)
{
    switch (g_config_ctx.current_state) {
        case CONFIG_UI_MENU:
            g_config_ctx.visual.section_indicators_active = false;
            break;
        default:
            break;
    }
    return CONFIG_OK;
}

static config_status_t process_menu_state(uint8_t row, uint8_t col, bool pressed)
{
    if (!is_section_key(row, col)) {
        return CONFIG_OK;
    }

    uint8_t section = get_section_from_key(row, col);
    LOG_INFO("Section %d selected", section);

    switch (section) {
        case 1: return config_ui_enter_zone_config();
        case 2: return config_ui_enter_effects_config();
        case 3: return config_ui_enter_power_config();
        case 4: return config_ui_enter_brightness_config();
        default: return CONFIG_INVALID_PARAMETER;
    }
}

static config_status_t process_zone_config_fsm(uint8_t row, uint8_t col, bool pressed)
{
    // Zone configuration FSM implementation
    zone_config_fsm_t *fsm = &g_config_ctx.zone_fsm;

    switch (fsm->state) {
        case SECTION_FSM_ENTRY:
            // Show zone slots 1-8 (red=empty, green=occupied)
            for (uint8_t i = 1; i <= 8; i++) {
                Backlight_RGB_t slot_color = fsm->slot_occupied[i-1] ? CONFIG_SECTION_COLOR : CONFIG_CANCEL_COLOR;
                Backlight_SetKeyRGB(CONFIG_SECTION_ROW, i, slot_color);
            }
            fsm->state = SECTION_FSM_BROWSE;
            break;

        case SECTION_FSM_BROWSE:
            // Handle slot selection (keys 1-8)
            if (row == CONFIG_SECTION_ROW && col >= 1 && col <= 8) {
                fsm->selected_slot = col - 1;
                LOG_INFO("Zone slot %d selected", fsm->selected_slot);

                // Load zone if occupied, or start with blank
                if (fsm->slot_occupied[fsm->selected_slot]) {
                    // TODO: Load zone from EEPROM
                    LOG_INFO("Loading existing zone");
                } else {
                    LOG_INFO("Creating new zone");
                }

                // Enter edit mode
                fsm->state = SECTION_FSM_EDIT;

                // Clear display and start key selection
                Backlight_SetAllRGB(CONFIG_DIM_COLOR);
                fsm->selection_count = 0;
                memset(fsm->selected_keys, 0, sizeof(fsm->selected_keys));
            }
            break;

        case SECTION_FSM_EDIT:
            // Handle key selection and HSV adjustment
            extern kb_state_t kb_state;

            if (kb_state.fn_pressed) {
                // HSV adjustment with Fn + Arrow keys
                switch (col) {
                    case 11: // Left arrow - decrease hue
                        if (row == 4) {
                            fsm->hsv_current.hue = (fsm->hsv_current.hue + 350) % 360;
                            LOG_DEBUG("Hue: %d", fsm->hsv_current.hue);
                        }
                        break;
                    case 13: // Right arrow - increase hue
                        if (row == 4) {
                            fsm->hsv_current.hue = (fsm->hsv_current.hue + 10) % 360;
                            LOG_DEBUG("Hue: %d", fsm->hsv_current.hue);
                        }
                        break;
                    case 12: // Up arrow - increase brightness
                        if (row == 3) {
                            fsm->hsv_current.value = (fsm->hsv_current.value < 95) ?
                                                    fsm->hsv_current.value + 5 : 100;
                            LOG_DEBUG("Value: %d", fsm->hsv_current.value);
                        }
                        break;
                    case 12: // Down arrow - decrease brightness
                        if (row == 5) {
                            fsm->hsv_current.value = (fsm->hsv_current.value > 5) ?
                                                    fsm->hsv_current.value - 5 : 0;
                            LOG_DEBUG("Value: %d", fsm->hsv_current.value);
                        }
                        break;
                }

                // Apply color to selected keys in real-time
                Backlight_RGB_t current_color = HSV_to_RGB(fsm->hsv_current.hue,
                                                          fsm->hsv_current.saturation,
                                                          fsm->hsv_current.value);
                for (uint8_t r = 0; r < 10; r++) {
                    for (uint8_t c = 0; c < 14; c++) {
                        if (fsm->selected_keys[r][c]) {
                            Backlight_SetKeyRGB(r, c, current_color);
                        }
                    }
                }
            } else {
                // Key selection mode
                const rgb_led_mapping_t* led = get_led_for_key(row, col);
                if (led) {
                    // Toggle key selection
                    if (fsm->selected_keys[row][col]) {
                        // Deselect key
                        fsm->selected_keys[row][col] = 0;
                        fsm->selection_count--;
                        Backlight_SetKeyRGB(row, col, CONFIG_DIM_COLOR);
                        LOG_DEBUG("Key R%dC%d deselected", row, col);
                    } else {
                        // Select key
                        fsm->selected_keys[row][col] = 1;
                        fsm->selection_count++;
                        Backlight_SetKeyRGB(row, col, CONFIG_ACTIVE_COLOR);
                        LOG_DEBUG("Key R%dC%d selected", row, col);
                    }
                }
            }
            break;

        default:
            break;
    }

    g_config_ctx.session.changes_pending = true;
    return CONFIG_OK;
}

static config_status_t process_effects_config_fsm(uint8_t row, uint8_t col, bool pressed)
{
    // Effects configuration FSM implementation
    effects_config_fsm_t *fsm = &g_config_ctx.effects_fsm;

    switch (fsm->state) {
        case SECTION_FSM_ENTRY:
            // Show effect type options
            Backlight_SetAllRGB(CONFIG_DIM_COLOR);

            // Show effect types on number row
            Backlight_SetKeyRGB(CONFIG_SECTION_ROW, 1, CONFIG_SECTION_COLOR); // Static
            Backlight_SetKeyRGB(CONFIG_SECTION_ROW, 2, CONFIG_SECTION_COLOR); // Breathing
            Backlight_SetKeyRGB(CONFIG_SECTION_ROW, 3, CONFIG_SECTION_COLOR); // Color cycling

            // Highlight current selection
            Backlight_SetKeyRGB(CONFIG_SECTION_ROW, fsm->effect_type, CONFIG_ACTIVE_COLOR);

            fsm->state = SECTION_FSM_BROWSE;
            break;

        case SECTION_FSM_BROWSE:
            // Handle effect type selection
            if (row == CONFIG_SECTION_ROW && col >= 1 && col <= 3) {
                fsm->effect_type = col;
                LOG_INFO("Effect type %d selected", fsm->effect_type);

                // Update highlight
                for (uint8_t i = 1; i <= 3; i++) {
                    Backlight_RGB_t color = (i == fsm->effect_type) ? CONFIG_ACTIVE_COLOR : CONFIG_SECTION_COLOR;
                    Backlight_SetKeyRGB(CONFIG_SECTION_ROW, i, color);
                }

                fsm->state = SECTION_FSM_EDIT;
            }
            break;

        case SECTION_FSM_EDIT:
            // Handle parameter adjustment based on effect type
            extern kb_state_t kb_state;

            if (kb_state.fn_pressed) {
                // Parameter adjustment with Fn + Arrow keys
                switch (col) {
                    case 11: // Left arrow - decrease parameter
                        if (row == 4) {
                            if (fsm->effect_type == 2 || fsm->effect_type == 3) { // breathing or cycling
                                fsm->cycle_time_ms = (fsm->cycle_time_ms > 1000) ?
                                                    fsm->cycle_time_ms - 500 : 1000;
                                LOG_DEBUG("Cycle time: %dms", fsm->cycle_time_ms);
                            }
                        }
                        break;
                    case 13: // Right arrow - increase parameter
                        if (row == 4) {
                            if (fsm->effect_type == 2 || fsm->effect_type == 3) {
                                fsm->cycle_time_ms = (fsm->cycle_time_ms < 10000) ?
                                                    fsm->cycle_time_ms + 500 : 10000;
                                LOG_DEBUG("Cycle time: %dms", fsm->cycle_time_ms);
                            }
                        }
                        break;
                    case 12: // Up arrow - increase brightness max
                        if (row == 3) {
                            fsm->brightness_max = (fsm->brightness_max < 100) ?
                                                 fsm->brightness_max + 5 : 100;
                            LOG_DEBUG("Max brightness: %d", fsm->brightness_max);
                        }
                        break;
                    case 12: // Down arrow - decrease brightness max
                        if (row == 5) {
                            fsm->brightness_max = (fsm->brightness_max > fsm->brightness_min + 5) ?
                                                 fsm->brightness_max - 5 : fsm->brightness_min + 5;
                            LOG_DEBUG("Max brightness: %d", fsm->brightness_max);
                        }
                        break;
                }

                // Start preview if not active
                if (!fsm->preview_active) {
                    fsm->preview_active = true;
                    LOG_INFO("Effect preview started");
                    // TODO: Start preview effect with current parameters
                }
            } else {
                // Handle color preset modification for cycling effect
                if (fsm->effect_type == 3 && row == CONFIG_SECTION_ROW && col >= 1 && col <= 6) {
                    uint8_t preset_idx = col - 1;
                    // Cycle through hue values for this preset
                    fsm->color_presets[preset_idx] = (fsm->color_presets[preset_idx] + 30) % 360;

                    // Show color preview
                    Backlight_RGB_t preview_color = HSV_to_RGB(fsm->color_presets[preset_idx], 100, 80);
                    Backlight_SetKeyRGB(row, col, preview_color);

                    LOG_DEBUG("Color preset %d: hue %d", preset_idx, fsm->color_presets[preset_idx]);
                }
            }
            break;

        default:
            break;
    }

    g_config_ctx.session.changes_pending = true;
    return CONFIG_OK;
}

static config_status_t process_power_config_fsm(uint8_t row, uint8_t col, bool pressed)
{
    // Power management FSM implementation
    power_config_fsm_t *fsm = &g_config_ctx.power_fsm;

    switch (fsm->state) {
        case SECTION_FSM_ENTRY:
            // Show power mode options
            Backlight_SetAllRGB(CONFIG_DIM_COLOR);

            // USB mode indicators (top row)
            Backlight_SetKeyRGB(0, 1, CONFIG_SECTION_COLOR); // Always on
            Backlight_SetKeyRGB(0, 2, CONFIG_SECTION_COLOR); // Dim
            Backlight_SetKeyRGB(0, 3, CONFIG_SECTION_COLOR); // Off

            // Battery mode indicators (number row)
            Backlight_SetKeyRGB(CONFIG_SECTION_ROW, 1, CONFIG_SECTION_COLOR); // Always on
            Backlight_SetKeyRGB(CONFIG_SECTION_ROW, 2, CONFIG_SECTION_COLOR); // Dim
            Backlight_SetKeyRGB(CONFIG_SECTION_ROW, 3, CONFIG_SECTION_COLOR); // Off

            // Highlight current selections
            Backlight_SetKeyRGB(0, fsm->usb_mode, CONFIG_ACTIVE_COLOR);
            Backlight_SetKeyRGB(CONFIG_SECTION_ROW, fsm->battery_mode, CONFIG_ACTIVE_COLOR);

            fsm->state = SECTION_FSM_BROWSE;
            break;

        case SECTION_FSM_BROWSE:
            // Handle mode selection
            if (row == 0 && col >= 1 && col <= 3) {
                // USB mode selection
                fsm->usb_mode = col;
                LOG_INFO("USB mode %d selected", fsm->usb_mode);

                // Update USB mode highlights
                for (uint8_t i = 1; i <= 3; i++) {
                    Backlight_RGB_t color = (i == fsm->usb_mode) ? CONFIG_ACTIVE_COLOR : CONFIG_SECTION_COLOR;
                    Backlight_SetKeyRGB(0, i, color);
                }

                fsm->state = SECTION_FSM_EDIT;
            } else if (row == CONFIG_SECTION_ROW && col >= 1 && col <= 3) {
                // Battery mode selection
                fsm->battery_mode = col;
                LOG_INFO("Battery mode %d selected", fsm->battery_mode);

                // Update battery mode highlights
                for (uint8_t i = 1; i <= 3; i++) {
                    Backlight_RGB_t color = (i == fsm->battery_mode) ? CONFIG_ACTIVE_COLOR : CONFIG_SECTION_COLOR;
                    Backlight_SetKeyRGB(CONFIG_SECTION_ROW, i, color);
                }

                fsm->state = SECTION_FSM_EDIT;
            }
            break;

        case SECTION_FSM_EDIT:
            // Handle timeout adjustment
            extern kb_state_t kb_state;

            if (kb_state.fn_pressed) {
                switch (col) {
                    case 11: // Left arrow - decrease timeout
                        if (row == 4) {
                            if (fsm->usb_mode == 2) { // dim mode
                                fsm->usb_timeout_s = (fsm->usb_timeout_s > 60) ?
                                                    fsm->usb_timeout_s - 30 : 60;
                                LOG_DEBUG("USB timeout: %ds", fsm->usb_timeout_s);
                            }
                            if (fsm->battery_mode == 2) { // dim mode
                                fsm->battery_timeout_s = (fsm->battery_timeout_s > 30) ?
                                                        fsm->battery_timeout_s - 30 : 30;
                                LOG_DEBUG("Battery timeout: %ds", fsm->battery_timeout_s);
                            }
                        }
                        break;
                    case 13: // Right arrow - increase timeout
                        if (row == 4) {
                            if (fsm->usb_mode == 2) {
                                fsm->usb_timeout_s = (fsm->usb_timeout_s < 600) ?
                                                    fsm->usb_timeout_s + 30 : 600;
                                LOG_DEBUG("USB timeout: %ds", fsm->usb_timeout_s);
                            }
                            if (fsm->battery_mode == 2) {
                                fsm->battery_timeout_s = (fsm->battery_timeout_s < 300) ?
                                                        fsm->battery_timeout_s + 30 : 300;
                                LOG_DEBUG("Battery timeout: %ds", fsm->battery_timeout_s);
                            }
                        }
                        break;
                    case 12: // Up arrow - increase dim percentage
                        if (row == 3) {
                            fsm->dim_percentage = (fsm->dim_percentage < 50) ?
                                                 fsm->dim_percentage + 5 : 50;
                            LOG_DEBUG("Dim percentage: %d%%", fsm->dim_percentage);
                        }
                        break;
                    case 12: // Down arrow - decrease dim percentage
                        if (row == 5) {
                            fsm->dim_percentage = (fsm->dim_percentage > 5) ?
                                                 fsm->dim_percentage - 5 : 5;
                            LOG_DEBUG("Dim percentage: %d%%", fsm->dim_percentage);
                        }
                        break;
                }

                // Start power preview if not active
                if (!fsm->preview_active) {
                    fsm->preview_active = true;
                    LOG_INFO("Power preview started");
                    // TODO: Show current power behavior
                }
            }
            break;

        default:
            break;
    }

    g_config_ctx.session.changes_pending = true;
    return CONFIG_OK;
}

static config_status_t process_brightness_config_fsm(uint8_t row, uint8_t col, bool pressed)
{
    // Brightness configuration FSM implementation
    brightness_config_fsm_t *fsm = &g_config_ctx.brightness_fsm;

    switch (fsm->state) {
        case SECTION_FSM_ENTRY:
            // Show brightness configuration interface
            Backlight_SetAllRGB(CONFIG_DIM_COLOR);

            // Show startup mode options
            Backlight_SetKeyRGB(CONFIG_SECTION_ROW, 1, CONFIG_SECTION_COLOR); // Last used
            Backlight_SetKeyRGB(CONFIG_SECTION_ROW, 2, CONFIG_SECTION_COLOR); // Preset
            Backlight_SetKeyRGB(CONFIG_SECTION_ROW, 3, CONFIG_SECTION_COLOR); // Adaptive

            // Highlight current selection
            Backlight_SetKeyRGB(CONFIG_SECTION_ROW, fsm->startup_mode, CONFIG_ACTIVE_COLOR);

            fsm->state = SECTION_FSM_BROWSE;
            break;

        case SECTION_FSM_BROWSE:
            // Handle startup mode selection
            if (row == CONFIG_SECTION_ROW && col >= 1 && col <= 3) {
                fsm->startup_mode = col;
                LOG_INFO("Startup mode %d selected", fsm->startup_mode);

                // Update highlights
                for (uint8_t i = 1; i <= 3; i++) {
                    Backlight_RGB_t color = (i == fsm->startup_mode) ? CONFIG_ACTIVE_COLOR : CONFIG_SECTION_COLOR;
                    Backlight_SetKeyRGB(CONFIG_SECTION_ROW, i, color);
                }

                fsm->state = SECTION_FSM_EDIT;
            }
            break;

        case SECTION_FSM_EDIT:
            // Handle brightness range adjustment
            extern kb_state_t kb_state;

            if (kb_state.fn_pressed) {
                switch (col) {
                    case 11: // Left arrow - decrease max brightness
                        if (row == 4) {
                            fsm->global_max = (fsm->global_max > fsm->global_min + 10) ?
                                             fsm->global_max - 5 : fsm->global_min + 10;
                            LOG_DEBUG("Max brightness: %d", fsm->global_max);
                        }
                        break;
                    case 13: // Right arrow - increase max brightness
                        if (row == 4) {
                            fsm->global_max = (fsm->global_max < 127) ?
                                             fsm->global_max + 5 : 127;
                            LOG_DEBUG("Max brightness: %d", fsm->global_max);
                        }
                        break;
                    case 12: // Up arrow - increase min brightness
                        if (row == 3) {
                            fsm->global_min = (fsm->global_min < fsm->global_max - 10) ?
                                             fsm->global_min + 5 : fsm->global_max - 10;
                            LOG_DEBUG("Min brightness: %d", fsm->global_min);
                        }
                        break;
                    case 12: // Down arrow - decrease min brightness
                        if (row == 5) {
                            fsm->global_min = (fsm->global_min > 5) ?
                                             fsm->global_min - 5 : 5;
                            LOG_DEBUG("Min brightness: %d", fsm->global_min);
                        }
                        break;
                }

                // Apply brightness change for immediate feedback
                uint8_t test_brightness = (fsm->global_min + fsm->global_max) / 2;
                Backlight_SetGlobalBrightness(test_brightness);
                LOG_DEBUG("Test brightness: %d", test_brightness);
            }
            break;

        default:
            break;
    }

    g_config_ctx.session.changes_pending = true;
    return CONFIG_OK;
}

static void setup_config_mode_lighting(void)
{
    g_config_ctx.visual.config_mode_breathing = true;
    g_config_ctx.visual.current_brightness = CONFIG_BREATH_MIN;
    LOG_DEBUG("Config mode lighting setup complete");
}

static void update_breathing_effect(void)
{
    static uint32_t last_breath_update = 0;
    static bool breathing_up = true;

    uint32_t current_time = HAL_GetTick();

    if ((current_time - last_breath_update) < 50) {
        return;
    }
    last_breath_update = current_time;

    // Simple breathing logic
    if (breathing_up) {
        g_config_ctx.visual.current_brightness += 2;
        if (g_config_ctx.visual.current_brightness >= CONFIG_BREATH_MAX) {
            breathing_up = false;
        }
    } else {
        g_config_ctx.visual.current_brightness -= 2;
        if (g_config_ctx.visual.current_brightness <= CONFIG_BREATH_MIN) {
            breathing_up = true;
        }
    }

    // Apply breathing to background
    Backlight_RGB_t breath_color = Color_Scale(CONFIG_DIM_COLOR, g_config_ctx.visual.current_brightness);

    // Set background breathing (section indicators will override)
    for (uint8_t row = 0; row < 6; row++) {
        for (uint8_t col = 0; col < 14; col++) {
            const rgb_led_mapping_t* led = get_led_for_key(row, col);
            if (led && !(row == CONFIG_SECTION_ROW && col >= 1 && col <= 4)) {
                Backlight_SetKeyRGB(row, col, breath_color);
            }
        }
    }
}

static void show_section_indicators(void)
{
    // Keep section keys highlighted
    for (uint8_t col = 1; col <= 4; col++) {
        const rgb_led_mapping_t* led = get_led_for_key(CONFIG_SECTION_ROW, col);
        if (led) {
            Backlight_SetKeyRGB(CONFIG_SECTION_ROW, col, CONFIG_SECTION_COLOR);
        }
    }
}

static void flash_confirmation(Backlight_RGB_t color, uint16_t duration_ms)
{
    // Simple flash implementation
    Backlight_SetAllRGB(color);
    HAL_Delay(duration_ms);

    // Restore appropriate lighting
    if (g_config_ctx.current_state == CONFIG_UI_MENU) {
        config_ui_show_menu_indicators();
    }
}

static config_status_t handle_error(config_status_t error)
{
    g_config_ctx.session.error_count++;
    g_config_ctx.session.last_error = error;

    LOG_ERROR("Config UI error %d (count: %d)", error, g_config_ctx.session.error_count);

    flash_confirmation(CONFIG_CANCEL_COLOR, 200);

    if (g_config_ctx.session.error_count >= CONFIG_MAX_ERRORS) {
        LOG_ERROR("Too many errors - force exiting");
        config_ui_force_exit();
    }

    return error;
}

/* ========================================================================
 * VISUAL FEEDBACK FUNCTIONS
 * ======================================================================== */

void config_ui_flash_save_success(void)
{
    flash_confirmation(CONFIG_SAVE_COLOR, CONFIG_FLASH_DURATION_MS);
}

void config_ui_flash_save_error(void)
{
    flash_confirmation(CONFIG_CANCEL_COLOR, CONFIG_FLASH_DURATION_MS);
}

void config_ui_flash_cancel(void)
{
    flash_confirmation(CONFIG_CANCEL_COLOR, CONFIG_FLASH_DURATION_MS);
}

void config_ui_flash_timeout_warning(void)
{
    flash_confirmation(CONFIG_WARNING_COLOR, CONFIG_FLASH_DURATION_MS);
}
