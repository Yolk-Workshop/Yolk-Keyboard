/*
 * config_ui.h
 *
 *  Created on: Jun 13, 2025
 *      Author: bettysidepiece
 */

#ifndef INC_SYSTEM_CONFIG_CONFIG_UI_H_
#define INC_SYSTEM_CONFIG_CONFIG_UI_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "system_config.h"

/* ========================================================================
 * CONFIGURATION UI SYSTEM STATES
 * ======================================================================== */

typedef enum {
    CONFIG_UI_IDLE = 0,           // Normal operation (not in config mode)
    CONFIG_UI_MENU,               // Main configuration menu (show sections 1-4)
    CONFIG_UI_ZONE_CONFIG,        // Section 1: Zone programming
    CONFIG_UI_EFFECTS_CONFIG,     // Section 2: Effects configuration
    CONFIG_UI_POWER_CONFIG,       // Section 3: Power management
    CONFIG_UI_BRIGHTNESS_CONFIG,  // Section 4: Brightness control
    CONFIG_UI_SAVE_CONFIRM,       // Universal save confirmation
    CONFIG_UI_CANCEL_CONFIRM,     // Universal cancel confirmation
    CONFIG_UI_ERROR               // Error state - force exit
} config_ui_state_t;

/* Section-specific FSM states */
typedef enum {
    SECTION_FSM_ENTRY = 0,        // Section initialization
    SECTION_FSM_BROWSE,           // Main section interface
    SECTION_FSM_EDIT,             // Parameter modification
    SECTION_FSM_PREVIEW,          // Live effect testing
    SECTION_FSM_CONFIRM           // Save/cancel confirmation
} section_fsm_state_t;

/* Universal command types */
typedef enum {
    UNIVERSAL_CMD_NONE = 0,
    UNIVERSAL_CMD_SAVE,           // Fn + S
    UNIVERSAL_CMD_CANCEL,         // Fn + L
    UNIVERSAL_CMD_MENU,           // Fn + Esc
    UNIVERSAL_CMD_TIMEOUT         // Auto-timeout
} universal_command_t;

/* ========================================================================
 * ENTRY SEQUENCE TRACKING
 * ======================================================================== */

typedef struct {
    bool fn_pressed;
    bool space_pressed;
    bool enter_pressed;
    uint32_t sequence_start_time;
    uint32_t hold_duration;
    bool sequence_complete;
    bool triggered;
} config_entry_sequence_t;

/* ========================================================================
 * SESSION MANAGEMENT
 * ======================================================================== */

typedef struct {
    uint32_t session_start_time;
    uint32_t last_activity_time;
    uint32_t timeout_duration_ms;
    bool auto_timeout_enabled;
    bool changes_pending;
    bool session_active;
    uint8_t error_count;
    config_status_t last_error;
} config_session_t;

/* ========================================================================
 * HID SYSTEM CONTROL - SIMPLIFIED
 * ======================================================================== */

// Simple flag - no complex struct needed
extern bool g_config_ui_active;

/* ========================================================================
 * SECTION FSM INTERFACES - SIMPLIFIED
 * ======================================================================== */

/* Zone Configuration Section */
typedef struct {
    section_fsm_state_t state;
    uint8_t selected_slot;
    bool slot_occupied[8];
    uint8_t slot_key_counts[8];
    uint32_t slot_crc[8];
    uint8_t selected_keys[10][14];   // Key selection matrix
    uint8_t selection_count;
    struct {
        uint16_t hue;
        uint8_t saturation;
        uint8_t value;
    } hsv_current;
} zone_config_fsm_t;

/* Effects Configuration Section - Simplified */
typedef struct {
    section_fsm_state_t state;
    uint8_t effect_type;          // 1=static, 2=breathing, 3=color_cycling
    uint16_t cycle_time_ms;
    uint8_t brightness_min;
    uint8_t brightness_max;
    uint16_t transition_duration_ms;
    uint8_t color_presets[6];     // Allow modification of default transition colors
    bool preview_active;
} effects_config_fsm_t;

/* Power Management Section - Unified modes */
typedef struct {
    section_fsm_state_t state;
    uint8_t usb_mode;             // 1=always_on, 2=dim, 3=off
    uint8_t battery_mode;         // 1=always_on, 2=dim, 3=off
    uint16_t usb_timeout_s;
    uint16_t battery_timeout_s;
    uint8_t dim_percentage;
    bool wake_on_keypress;
    bool preview_active;
} power_config_fsm_t;

/* Brightness Control Section - Simplified */
typedef struct {
    section_fsm_state_t state;
    uint8_t global_max;
    uint8_t global_min;
    uint8_t startup_mode;         // 1=last, 2=default, 3=adaptive
} brightness_config_fsm_t;

/* ========================================================================
 * VISUAL FEEDBACK SYSTEM
 * ======================================================================== */

typedef struct {
    bool config_mode_breathing;   // Special breathing for config mode
    bool section_indicators_active;
    bool preview_active;
    uint32_t last_visual_update;
    uint8_t current_brightness;
    uint8_t target_brightness;
} config_visual_state_t;

/* ========================================================================
 * SECTION FSM INTERFACES
 * ======================================================================== */

/* Zone Configuration Section */
typedef struct {
    section_fsm_state_t state;
    uint8_t selected_slot;
    bool slot_occupied[8];
    uint8_t slot_key_counts[8];
    uint32_t slot_crc[8];
    uint8_t selected_keys[10][14];   // Key selection matrix
    uint8_t selection_count;
    struct {
        uint16_t hue;
        uint8_t saturation;
        uint8_t value;
    } hsv_current;
} zone_config_fsm_t;

/* Effects Configuration Section */
typedef struct {
    section_fsm_state_t state;
    uint8_t effect_type;          // 1=static, 2=breathing, 3=cycling, 4=transition
    uint16_t cycle_time_ms;
    uint8_t brightness_min;
    uint8_t brightness_max;
    uint16_t transition_duration_ms;
    uint8_t color_preset;
    bool preview_active;
} effects_config_fsm_t;

/* Power Management Section */
typedef struct {
    section_fsm_state_t state;
    uint8_t usb_mode;             // 1=always_on, 2=dim, 3=off
    uint8_t battery_mode;         // 1=30s, 2=2min, 3=custom
    uint16_t usb_timeout_s;
    uint16_t battery_timeout_s;
    uint8_t dim_percentage;
    bool wake_on_keypress;
    bool preview_active;
} power_config_fsm_t;

/* Brightness Control Section */
typedef struct {
    section_fsm_state_t state;
    uint8_t global_max;
    uint8_t global_min;
    uint8_t curve_type;           // 1=linear, 2=perceptual, 3=custom
    uint8_t presets[4];           // Q/W/E/R presets
    uint8_t startup_mode;         // 1=last, 2=preset, 3=adaptive
    uint8_t test_brightness;
} brightness_config_fsm_t;

/* ========================================================================
 * MAIN CONFIGURATION CONTEXT
 * ======================================================================== */

typedef struct {
    // System state
    config_ui_state_t current_state;
    config_ui_state_t previous_state;

    // Entry sequence
    config_entry_sequence_t entry_sequence;

    // Session management
    config_session_t session;

    // Visual feedback
    config_visual_state_t visual;

    // Section FSMs
    zone_config_fsm_t zone_fsm;
    effects_config_fsm_t effects_fsm;
    power_config_fsm_t power_fsm;
    brightness_config_fsm_t brightness_fsm;

    // Universal command detection
    universal_command_t pending_command;

} config_ui_context_t;

/* ========================================================================
 * CONFIGURATION CONSTANTS
 * ======================================================================== */

#define CONFIG_ENTRY_HOLD_TIME_MS       5000   // 5 seconds for Fn+Space+Enter
#define CONFIG_AUTO_TIMEOUT_MS          600000 // 10 minutes
#define CONFIG_TIMEOUT_WARNING_MS       540000 // 9 minutes (warning)
#define CONFIG_VISUAL_UPDATE_MS         100    // Visual feedback update rate
#define CONFIG_BREATHING_CYCLE_MS       3000   // Config mode breathing cycle
#define CONFIG_MAX_ERRORS               5      // Max errors before force exit

/* Key positions for sections (number row) */
#define CONFIG_SECTION_ROW              1      // Number row
#define CONFIG_SECTION_1_COL            1      // Zone Programming
#define CONFIG_SECTION_2_COL            2      // Effects Configuration
#define CONFIG_SECTION_3_COL            3      // Power Management
#define CONFIG_SECTION_4_COL            4      // Brightness Control

/* Universal command key positions */
#define CONFIG_SAVE_KEY                 KC_S   // Fn + S
#define CONFIG_CANCEL_KEY               KC_L   // Fn + L
#define CONFIG_MENU_KEY                 KC_ESCAPE // Fn + Esc

/* ========================================================================
 * PUBLIC API - SYSTEM INTEGRATION
 * ======================================================================== */

/**
 * @brief Initialize the configuration UI system
 * @return CONFIG_OK on success, error code on failure
 */
config_status_t config_ui_init(void);

/**
 * @brief Main processing function - call from main loop
 * Must be called every loop iteration when config mode is active
 */
void config_ui_process(void);

/**
 * @brief Check for configuration entry sequence
 * Call from main loop to detect Fn+Space+Enter
 */
void check_config_ui_functions(void);

/**
 * @brief Handle key events when in configuration mode
 * @param row Key matrix row
 * @param col Key matrix column
 * @param pressed True if pressed, false if released
 * @return True if key was handled, false if should be processed normally
 */
bool config_ui_handle_key_event(uint8_t row, uint8_t col, bool pressed);

/**
 * @brief Check if configuration UI should intercept key processing
 * @param row Key matrix row
 * @param col Key matrix column
 * @return True if config UI should handle this key
 */
bool config_ui_should_intercept_key(uint8_t row, uint8_t col);

/* ========================================================================
 * PUBLIC API - STATE MANAGEMENT
 * ======================================================================== */

/**
 * @brief Check if configuration UI is currently active
 * @return True if in config mode, false if normal operation
 */
bool config_ui_is_active(void);

/**
 * @brief Get current configuration UI state
 * @return Current state
 */
config_ui_state_t config_ui_get_state(void);

/**
 * @brief Enter configuration mode (system state change)
 * @return CONFIG_OK on success, error code on failure
 */
config_status_t config_ui_enter_system_mode(void);

/**
 * @brief Exit configuration mode (restore normal operation)
 * @param save_changes True to commit changes, false to discard
 * @return CONFIG_OK on success, error code on failure
 */
config_status_t config_ui_exit_system_mode(bool save_changes);

/**
 * @brief Force immediate exit from configuration mode
 * Used for error recovery and timeout handling
 */
void config_ui_force_exit(void);

/* ========================================================================
 * PUBLIC API - HID SYSTEM CONTROL
 * ======================================================================== */

/**
 * @brief Check if HID reports should be suspended
 * @return True if reports should be blocked
 */
bool config_ui_hid_reports_suspended(void);

/**
 * @brief Check if USB reports are suspended
 * @return True if USB HID reports are blocked
 */
bool config_ui_usb_suspended(void);

/**
 * @brief Check if BLE reports are suspended
 * @return True if BLE HID reports are blocked
 */
bool config_ui_ble_suspended(void);

/**
 * @brief Suspend HID report generation
 * Blocks both USB and BLE keyboard reports
 */
void config_ui_suspend_hid_reports(void);

/**
 * @brief Resume HID report generation
 * Re-enables USB and BLE keyboard reports
 */
void config_ui_resume_hid_reports(void);

/* ========================================================================
 * PUBLIC API - SECTION INTERFACES
 * ======================================================================== */

/**
 * @brief Enter zone configuration section
 * @return CONFIG_OK on success, error code on failure
 */
config_status_t config_ui_enter_zone_config(void);

/**
 * @brief Enter effects configuration section
 * @return CONFIG_OK on success, error code on failure
 */
config_status_t config_ui_enter_effects_config(void);

/**
 * @brief Enter power management section
 * @return CONFIG_OK on success, error code on failure
 */
config_status_t config_ui_enter_power_config(void);

/**
 * @brief Enter brightness control section
 * @return CONFIG_OK on success, error code on failure
 */
config_status_t config_ui_enter_brightness_config(void);

/**
 * @brief Return to main configuration menu
 * @return CONFIG_OK on success, error code on failure
 */
config_status_t config_ui_return_to_menu(void);

/* ========================================================================
 * PUBLIC API - UNIVERSAL COMMANDS
 * ======================================================================== */

/**
 * @brief Execute save command (Fn + S)
 * Commits all pending changes to EEPROM
 * @return CONFIG_OK on success, error code on failure
 */
config_status_t config_ui_execute_save(void);

/**
 * @brief Execute cancel command (Fn + L)
 * Discards all pending changes
 * @return CONFIG_OK on success, error code on failure
 */
config_status_t config_ui_execute_cancel(void);

/**
 * @brief Execute menu command (Fn + Esc)
 * Returns to main configuration menu
 * @return CONFIG_OK on success, error code on failure
 */
config_status_t config_ui_execute_menu(void);

/**
 * @brief Handle auto-timeout
 * Called when 10-minute timeout expires
 */
void config_ui_handle_timeout(void);

/* ========================================================================
 * PUBLIC API - VISUAL FEEDBACK
 * ======================================================================== */

/**
 * @brief Update configuration mode visual feedback
 * Called automatically by config_ui_process()
 */
void config_ui_update_visual_feedback(void);

/**
 * @brief Show main menu section indicators
 * Highlights keys 1-4 for available sections
 */
void config_ui_show_menu_indicators(void);

/**
 * @brief Flash save success confirmation
 * Brief green flash effect
 */
void config_ui_flash_save_success(void);

/**
 * @brief Flash save error indication
 * Brief red flash effect
 */
void config_ui_flash_save_error(void);

/**
 * @brief Flash cancel confirmation
 * Brief red flash effect
 */
void config_ui_flash_cancel(void);

/**
 * @brief Flash timeout warning
 * Brief yellow flash at 9 minutes
 */
void config_ui_flash_timeout_warning(void);

/* ========================================================================
 * PUBLIC API - DIAGNOSTICS AND STATUS
 * ======================================================================== */

/**
 * @brief Get configuration context for debugging
 * @return Pointer to internal context (read-only)
 */
const config_ui_context_t* config_ui_get_context(void);

/**
 * @brief Get session duration in milliseconds
 * @return Time since config mode was entered
 */
uint32_t config_ui_get_session_duration(void);

/**
 * @brief Get time until auto-timeout in milliseconds
 * @return Remaining time before auto-exit
 */
uint32_t config_ui_get_timeout_remaining(void);

/**
 * @brief Check if changes are pending
 * @return True if there are unsaved changes
 */
bool config_ui_has_pending_changes(void);

/**
 * @brief Get error count for current session
 * @return Number of errors encountered
 */
uint8_t config_ui_get_error_count(void);

/**
 * @brief Reset error count
 * Used after successful recovery
 */
void config_ui_reset_error_count(void);

/**
 * @brief Get last error code
 * @return Last error that occurred
 */
config_status_t config_ui_get_last_error(void);

/* ========================================================================
 * INTEGRATION MACROS
 * ======================================================================== */

/**
 * @brief Check if system is in configuration mode
 * Use this in main loop and other system code
 */
#define IS_CONFIG_MODE_ACTIVE()     config_ui_is_active()

/**
 * @brief Check if key should be routed to config UI
 * Use this in processKey() function
 */
#define SHOULD_ROUTE_TO_CONFIG_UI(row, col) \
    (config_ui_is_active() && config_ui_should_intercept_key(row, col))

/**
 * @brief Check if HID reports should be blocked
 * Use this in reportArbiter() and HID functions
 */
#define SHOULD_BLOCK_HID_REPORTS()  config_ui_hid_reports_suspended()

#endif /* INC_SYSTEM_CONFIG_CONFIG_UI_H_ */
