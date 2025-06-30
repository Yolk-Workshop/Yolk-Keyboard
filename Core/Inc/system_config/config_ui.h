/*
 * config_ui.h - Master Configuration System Interface
 *
 * Implements the complete Yolk keyboard configuration architecture:
 * - Master Entry: Fn+Space+Enter for 5 seconds
 * - Section Navigation: 1=Zones, 2=Effects, 3=Power, 4=Brightness
 * - Universal Controls: Fn+S (save all), Fn+L (cancel all), Fn+Esc (menu)
 * - Unified Session Management with atomic operations
 * - 10-minute auto-timeout with warning
 */

#ifndef CONFIG_UI_H_
#define CONFIG_UI_H_

#include <stdint.h>
#include <stdbool.h>
#include "system_config.h"

/* ========================================================================
 * MASTER CONFIGURATION CONSTANTS
 * ======================================================================== */

#define CONFIG_UI_ENTRY_HOLD_TIME_MS        5000    // Fn+Space+Enter hold time
#define CONFIG_UI_SESSION_TIMEOUT_MS        600000  // 10 minutes total session
#define CONFIG_UI_TIMEOUT_WARNING_MS        540000  // 9 minutes (1 min warning)
#define CONFIG_UI_MAX_SESSION_ERRORS        5       // Force exit after errors

/* ========================================================================
 * MASTER CONFIGURATION TYPES
 * ======================================================================== */

/**
 * @brief Master configuration system states
 */
typedef enum {
    CONFIG_UI_IDLE = 0,                     // Normal keyboard operation
    CONFIG_UI_ENTRY_SEQUENCE,               // Detecting Fn+Space+Enter sequence
    CONFIG_UI_MENU,                         // Main menu (section selection 1-4)
    CONFIG_UI_ZONE_CONFIG,                  // Zone programming subsystem active
    CONFIG_UI_EFFECTS_CONFIG,               // Effects configuration subsystem active
    CONFIG_UI_POWER_CONFIG,                 // Power management subsystem active
    CONFIG_UI_BRIGHTNESS_CONFIG,            // Brightness control subsystem active
    CONFIG_UI_ERROR_STATE                   // Error recovery state
} config_ui_state_t;

/**
 * @brief Configuration sections available
 */
typedef enum {
    CONFIG_SECTION_NONE = 0,
    CONFIG_SECTION_ZONES = 1,               // Zone programming (key 1)
    CONFIG_SECTION_EFFECTS = 2,             // Effects configuration (key 2)
    CONFIG_SECTION_POWER = 3,               // Power management (key 3)
    CONFIG_SECTION_BRIGHTNESS = 4           // Brightness control (key 4)
} config_section_t;

/**
 * @brief Universal commands available from any section
 */
typedef enum {
    UNIVERSAL_CMD_NONE = 0,
    UNIVERSAL_CMD_SAVE_ALL,                 // Fn+S: Save all changes and exit
    UNIVERSAL_CMD_CANCEL_ALL,               // Fn+L: Cancel all changes and exit
    UNIVERSAL_CMD_RETURN_TO_MENU,           // Fn+Esc: Return to main menu
    UNIVERSAL_CMD_TIMEOUT                   // System timeout occurred
} universal_command_t;

/**
 * @brief Configuration UI context (legacy compatibility)
 */
typedef struct {
    config_ui_state_t current_state;
    bool session_active;
    bool has_unsaved_changes;
    uint32_t session_duration_ms;
    uint32_t timeout_remaining_ms;
    uint8_t error_count;
    config_status_t last_error;
} config_ui_context_t;

/* ========================================================================
 * MASTER SYSTEM LIFECYCLE
 * ======================================================================== */

/**
 * @brief Initialize the master configuration system
 * @return CONFIG_OK on success, error code on failure
 * @note Must be called during system initialization
 */
config_status_t config_ui_init(void);

/**
 * @brief Process configuration system (call from main loop)
 * @note Handles timeouts, visual updates, and subsystem processing
 */
void config_ui_process(void);

/* ========================================================================
 * ENTRY SEQUENCE DETECTION
 * ======================================================================== */

/**
 * @brief Update entry sequence detection
 * @param keycode Key that was pressed/released
 * @param pressed True if pressed, false if released
 * @return True if key was part of entry sequence
 * @note Call from key processing for Space and Enter keys
 */
bool config_ui_update_entry_sequence(uint8_t keycode, bool pressed);

/**
 * @brief Check for entry sequence completion
 * @note Call from main loop to detect 5-second hold completion
 */
void check_config_ui_functions(void);

/* ========================================================================
 * MASTER MODE CONTROL
 * ======================================================================== */

/**
 * @brief Enter master configuration mode
 * @return CONFIG_OK on success
 * @note Called automatically when entry sequence completes
 */
config_status_t config_ui_enter_master_mode(void);

/**
 * @brief Exit master configuration mode
 * @param save_changes True to save all changes, false to discard
 * @return CONFIG_OK on success
 * @note Handles atomic save/cancel operations across all sections
 */
config_status_t config_ui_exit_master_mode(bool save_changes);

/**
 * @brief Force immediate exit from configuration mode
 * @note Emergency exit function for error recovery
 */
void config_ui_force_exit(void);

/* ========================================================================
 * SECTION NAVIGATION
 * ======================================================================== */

/**
 * @brief Show main section menu (1-4 indicators)
 * @note Displays breathing section indicators on number row
 */
void config_ui_show_section_menu(void);

/**
 * @brief Enter specific configuration section
 * @param section_number Section to enter (1=Zones, 2=Effects, 3=Power, 4=Brightness)
 * @return CONFIG_OK on success
 */
config_status_t config_ui_enter_section(uint8_t section_number);

/**
 * @brief Return to main menu from current section
 * @return CONFIG_OK on success
 * @note Discards section-specific changes but preserves session
 */
config_status_t config_ui_return_to_menu(void);

/* ========================================================================
 * SECTION ENTRY POINTS
 * ======================================================================== */

/**
 * @brief Enter zone programming section
 * @return CONFIG_OK on success
 * @note Activates zones.c subsystem
 */
config_status_t config_ui_enter_zones(void);

/**
 * @brief Enter effects configuration section
 * @return CONFIG_OK on success
 * @note Activates effects configuration interface
 */
config_status_t config_ui_enter_effects(void);

/**
 * @brief Enter power management section
 * @return CONFIG_OK on success
 * @note Configures USB/battery power behaviors
 */
config_status_t config_ui_enter_power(void);

/**
 * @brief Enter brightness control section
 * @return CONFIG_OK on success
 * @note Configures global brightness settings
 */
config_status_t config_ui_enter_brightness(void);

/* ========================================================================
 * KEY EVENT PROCESSING
 * ======================================================================== */

/**
 * @brief Process key event in configuration mode
 * @param row Key row (0-5)
 * @param col Key column (0-13)
 * @param pressed True if pressed, false if released
 * @return True if key was handled by configuration system
 * @note Routes keys to appropriate section or processes universal commands
 */
bool config_ui_handle_key_event(uint8_t row, uint8_t col, bool pressed);

/**
 * @brief Process universal command (Fn+key combinations)
 * @param row Key row
 * @param col Key column
 * @return True if universal command was processed
 */
bool config_ui_process_universal_command(uint8_t row, uint8_t col);

/* ========================================================================
 * SECTION-SPECIFIC KEY HANDLERS
 * ======================================================================== */

/**
 * @brief Handle key in main menu state
 * @param row Key row
 * @param col Key column
 * @return True if key was handled
 */
bool config_ui_handle_main_menu_key(uint8_t row, uint8_t col);

/**
 * @brief Handle key in effects configuration
 * @param row Key row
 * @param col Key column
 * @return True if key was handled
 */
bool config_ui_handle_effects_key(uint8_t row, uint8_t col);

/**
 * @brief Handle key in power management
 * @param row Key row
 * @param col Key column
 * @return True if key was handled
 */
bool config_ui_handle_power_key(uint8_t row, uint8_t col);

/**
 * @brief Handle key in brightness control
 * @param row Key row
 * @param col Key column
 * @return True if key was handled
 */
bool config_ui_handle_brightness_key(uint8_t row, uint8_t col);

/* ========================================================================
 * UNIFIED SESSION MANAGEMENT
 * ======================================================================== */

/**
 * @brief Save all pending changes across all sections
 * @return CONFIG_OK if all saves successful
 * @note Atomic operation - all sections saved together
 */
config_status_t config_ui_save_all_changes(void);

/**
 * @brief Cancel all pending changes across all sections
 * @return CONFIG_OK on success
 * @note Discards changes from all modified sections
 */
config_status_t config_ui_cancel_all_changes(void);

/**
 * @brief Check if any section has pending changes
 * @return True if changes need to be saved
 */
bool config_ui_has_pending_changes(void);

/* ========================================================================
 * TIMEOUT AND ERROR HANDLING
 * ======================================================================== */

/**
 * @brief Handle session timeout
 * @note Called automatically when 10-minute timeout expires
 */
void config_ui_handle_timeout(void);

/**
 * @brief Handle configuration error
 * @param error Error code that occurred
 * @return Same error code for chaining
 * @note Provides visual feedback and error recovery
 */
config_status_t config_ui_handle_error(config_status_t error);

/**
 * @brief Update activity timestamp
 * @note Call whenever user interacts with configuration
 */
void config_ui_update_activity(void);

/* ========================================================================
 * VISUAL FEEDBACK SYSTEM
 * ======================================================================== */

/**
 * @brief Update visual feedback effects
 * @note Call from main loop for breathing effects and indicators
 */
void config_ui_update_visual_feedback(void);

/**
 * @brief Update breathing background effect
 * @note Creates power-efficient breathing for menu backgrounds
 */
void config_ui_update_breathing_background(void);

/**
 * @brief Flash configuration entry confirmation
 * @note Cyan flash when entering configuration mode
 */
void config_ui_flash_enter_confirmation(void);

/**
 * @brief Flash save operation result
 * @param success True for success (green), false for error (red)
 */
void config_ui_flash_save_result(bool success);

/**
 * @brief Flash cancellation confirmation
 * @note Red flash when cancelling changes
 */
void config_ui_flash_cancel_confirmation(void);

/**
 * @brief Flash timeout warning
 * @note Yellow flash at 9-minute mark
 */
void config_ui_flash_timeout_warning(void);

/* ========================================================================
 * STATUS AND DIAGNOSTICS
 * ======================================================================== */

/**
 * @brief Check if configuration mode is active
 * @return True if in configuration mode
 */
bool config_ui_is_active(void);

/**
 * @brief Get current configuration state
 * @return Current state enum value
 */
config_ui_state_t config_ui_get_state(void);

/**
 * @brief Get session duration in milliseconds
 * @return Duration since session started, 0 if no active session
 */
uint32_t config_ui_get_session_duration(void);

/**
 * @brief Get remaining time before timeout
 * @return Milliseconds until timeout, 0 if expired
 */
uint32_t config_ui_get_timeout_remaining(void);

/* ========================================================================
 * LEGACY COMPATIBILITY
 * ======================================================================== */

/**
 * @brief Legacy function - redirects to config_ui_enter_master_mode()
 * @return CONFIG_OK on success
 * @deprecated Use config_ui_enter_master_mode() instead
 */
config_status_t config_ui_enter_system_mode(void);

/**
 * @brief Legacy function - redirects to config_ui_exit_master_mode()
 * @param save_changes True to save, false to discard
 * @return CONFIG_OK on success
 * @deprecated Use config_ui_exit_master_mode() instead
 */
config_status_t config_ui_exit_system_mode(bool save_changes);

/**
 * @brief Legacy function - returns NULL
 * @return NULL (use specific status functions instead)
 * @deprecated Use config_ui_get_session_duration(), config_ui_has_pending_changes(), etc.
 */
const config_ui_context_t* config_ui_get_context(void);

/* ========================================================================
 * GLOBAL FLAGS
 * ======================================================================== */

/**
 * @brief Global flag indicating configuration mode is active
 * @note Used to block HID reports during configuration
 */
extern bool g_config_ui_active;

#endif /* CONFIG_UI_H_ */
