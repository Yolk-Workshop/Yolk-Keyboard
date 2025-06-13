/*
* zones.h - Visual Zone Programming API
*
* Created on: Jun 10, 2025
* Author: bettysidepiece
*/

#ifndef BACKLIGHT_CORE_EXTENSION_ZONES_H_
#define BACKLIGHT_CORE_EXTENSION_ZONES_H_

#include <stdint.h>
#include <stdbool.h>
#include "backlight.h"

/* ========================================================================== */
/* Zone Programming Configuration */
/* ========================================================================== */

#define ZONE_MAX_SLOTS                  8       // Maximum zone slots (0-7)
#define ZONE_PROGRAMMING_TIMEOUT_MS     600000  // 10 minute auto-exit timeout
#define ZONE_PREVIEW_DELAY_MS           55      // Preview display duration
#define ZONE_STORAGE_SIZE_BYTES         240     // 240 bytes per zone (6x14x3 RGB + metadata)
#define ZONE_MAX_KEYS                   84      // Maximum keys per zone (6 rows × 14 cols)

// Visual feedback colors
#define ZONE_COLOR_UNSET               (Backlight_RGB_t){64, 0, 0}    // Red for empty slots
#define ZONE_COLOR_SET                 (Backlight_RGB_t){0, 64, 0}    // Green for occupied slots
#define ZONE_COLOR_SELECTED            (Backlight_RGB_t){0, 64, 64}   // Cyan for selected keys
#define ZONE_COLOR_EDITING             (Backlight_RGB_t){64, 32, 0}   // Orange for edit mode
#define ZONE_COLOR_DIM                 (Backlight_RGB_t){16, 16, 16}  // Dim white for inactive

/* ========================================================================== */
/* Zone Programming States */
/* ========================================================================== */

typedef enum {
    ZONE_STATE_INACTIVE = 0,        // Normal keyboard operation
    ZONE_STATE_BROWSE,              // Browse available zone slots
    ZONE_STATE_EDIT,                // Edit selected zone
    ZONE_STATE_KEY_EDIT,            // Individual key color editing
    ZONE_STATE_GROUP_EDIT,          // Group/zone color editing
    ZONE_STATE_HSV_ADJUST,          // HSV wheel color selection
    ZONE_STATE_SAVE_CONFIRM,        // Save confirmation
    ZONE_STATE_ERROR                // Error state
} zone_state_t;

typedef enum {
    ZONE_EDIT_MODE_INDIVIDUAL = 0,  // Edit individual keys
    ZONE_EDIT_MODE_GROUP,           // Edit selected group as zone
    ZONE_EDIT_MODE_ZONE_WIDE        // Edit entire zone uniformly
} zone_edit_mode_t;

typedef enum {
    ZONE_ERROR_NONE = 0,
    ZONE_ERROR_INVALID_SLOT,
    ZONE_ERROR_STORAGE_FULL,
    ZONE_ERROR_EEPROM_FAIL,
    ZONE_ERROR_CRC_MISMATCH,
    ZONE_ERROR_TIMEOUT,
    ZONE_ERROR_INVALID_STATE,
    ZONE_ERROR_MEMORY,
	ZONE_ERROR_INVALID_PARAM
} zone_error_t;

/* ========================================================================== */
/* Zone Data Structures */
/* ========================================================================== */

// Individual key color data
typedef struct __attribute__((packed)) {
    uint8_t row;                    // Key row (0-5)
    uint8_t col;                    // Key column (0-13)
    Backlight_RGB_t color;          // RGB color for this key
    uint8_t active;                 // Key is part of this zone (changed from bool to uint8_t)
} zone_key_data_t;

// Zone metadata (stored with each zone) - Aligned for efficient access
typedef struct __attribute__((packed)) {
    uint8_t version;                // Zone format version
    uint8_t slot_id;                // Zone slot number (0-7)
    uint8_t key_count;              // Number of active keys in zone
    uint8_t brightness_level;       // Default brightness level (1-10)
    uint32_t created_timestamp;     // Creation timestamp (moved for 4-byte alignment)
    uint32_t modified_timestamp;    // Last modification timestamp (4-byte aligned)
    uint16_t crc16;                 // CRC16 checksum for validation (moved after 32-bit fields)
    uint8_t name[16];               // Zone name (null-terminated)
    uint8_t reserved[6];            // Reserved for future use (reduced to maintain 40 bytes total)
} zone_metadata_t;

// Complete zone storage structure (240 bytes total) - EEPROM DATA
typedef struct __attribute__((packed)) {
    zone_metadata_t metadata;       // 40 bytes - Zone information
    zone_key_data_t keys[ZONE_MAX_KEYS]; // 200 bytes - Key data (84 × 2.38 ≈ 200)
} zone_storage_t;

// Runtime zone management
typedef struct {
    bool slot_occupied[ZONE_MAX_SLOTS];     // Which slots have saved zones
    uint8_t active_slot;                    // Currently active zone slot
    uint8_t editing_slot;                   // Zone slot being edited
    zone_storage_t active_zone;             // Current active zone data
    zone_storage_t staging_zone;            // Zone being edited
    bool has_changes;                       // Staging zone has unsaved changes
} zone_manager_t;

/* ========================================================================== */
/* Zone Programming Context */
/* ========================================================================== */

// HSV color selection state
typedef struct {
    uint16_t hue;                   // Current hue (0-359)
    uint8_t saturation;             // Current saturation (0-100)
    uint8_t value;                  // Current value/brightness (0-100)
    uint8_t hue_step;              // Hue adjustment step size
    uint8_t sv_step;               // Saturation/Value step size
} hsv_state_t;

// Key selection tracking
typedef struct {
    bool selected[BACKLIGHT_MATRIX_ROWS][BACKLIGHT_MATRIX_COLS]; // Selected keys
    uint8_t selection_count;        // Number of selected keys
    uint8_t last_row, last_col;     // Last selected key position
    bool group_mode;                // Group selection active
} key_selection_t;

// Complete zone programming context
typedef struct {
    // State management
    zone_state_t current_state;     // Current programming state
    zone_state_t previous_state;    // Previous state for back navigation
    zone_edit_mode_t edit_mode;     // Current editing mode
    uint32_t state_start_time;      // When current state started
    uint32_t last_activity_time;    // Last user interaction time

    // Zone management
    zone_manager_t manager;         // Zone storage and management

    // User interaction state
    key_selection_t selection;      // Key selection state
    hsv_state_t hsv;               // HSV color picker state

    // Visual feedback
    bool preview_active;            // Preview mode active
    uint8_t preview_slot;           // Slot being previewed
    uint32_t preview_start_time;    // Preview start time

    // Error handling
    zone_error_t last_error;        // Last error encountered
    uint8_t error_count;            // Total error count

    // Configuration
    bool auto_exit_enabled;         // Auto-exit on timeout
    uint16_t preview_duration_ms;   // Preview display duration
    uint8_t default_brightness;     // Default brightness for new zones

} zone_programming_context_t;

/* ========================================================================== */
/* Zone Programming API */
/* ========================================================================== */

/**
 * @brief Initialize zone programming system
 * @return zone_error_t Error code
 */
zone_error_t Zones_Init(void);

/**
 * @brief Process zone programming state machine - call from main loop
 * @return zone_error_t Error code
 */
zone_error_t Zones_Process(void);

/**
 * @brief Check if zone programming mode is active
 * @return bool True if in programming mode
 */
bool Zones_IsActive(void);

/**
 * @brief Get current programming state
 * @return zone_state_t Current state
 */
zone_state_t Zones_GetState(void);

/**
 * @brief Check if zones should override effects system
 * Called by effects system before applying any LED updates
 * @return bool True if zones has control, false if effects can proceed
 */
bool Zones_HasLEDControl(void);

/* ========================================================================== */
/* Zone Programming Entry/Exit */
/* ========================================================================== */

/**
 * @brief Enter zone programming mode (Fn + Space + Enter)
 * Automatically suspends any active effects
 * @return zone_error_t Error code
 */
zone_error_t Zones_EnterProgramming(void);

/**
 * @brief Exit zone programming mode
 * Resumes previous effects state
 * @param save_changes True to save staging changes, false to discard
 * @return zone_error_t Error code
 */
zone_error_t Zones_ExitProgramming(bool save_changes);

/**
 * @brief Handle programming timeout (auto-exit after 10 minutes)
 * @return zone_error_t Error code
 */
zone_error_t Zones_HandleTimeout(void);

/* ========================================================================== */
/* Zone Browse Mode */
/* ========================================================================== */

/**
 * @brief Enter zone browse mode (show 8 slots with red/green indicators)
 * @return zone_error_t Error code
 */
zone_error_t Zones_EnterBrowseMode(void);

/**
 * @brief Select zone slot for editing or activation
 * @param slot_id Zone slot (0-7)
 * @return zone_error_t Error code
 */
zone_error_t Zones_SelectSlot(uint8_t slot_id);

/**
 * @brief Preview zone slot briefly (55ms)
 * @param slot_id Zone slot to preview (0-7)
 * @return zone_error_t Error code
 */
zone_error_t Zones_PreviewSlot(uint8_t slot_id);

/**
 * @brief Quick switch between first 2 saved zones (daily use)
 * @return zone_error_t Error code
 */
zone_error_t Zones_QuickSwitch(void);

/* ========================================================================== */
/* Zone Edit Mode */
/* ========================================================================== */

/**
 * @brief Enter zone edit mode for selected slot
 * @param slot_id Zone slot to edit (0-7)
 * @return zone_error_t Error code
 */
zone_error_t Zones_EnterEditMode(uint8_t slot_id);

/**
 * @brief Toggle edit mode between individual and group editing
 * @return zone_error_t Error code
 */
zone_error_t Zones_ToggleEditMode(void);

/**
 * @brief Add/remove key from selection (press to select/deselect)
 * @param row Key row (0-5)
 * @param col Key column (0-13)
 * @return zone_error_t Error code
 */
zone_error_t Zones_ToggleKeySelection(uint8_t row, uint8_t col);

/**
 * @brief Clear all key selections
 * @return zone_error_t Error code
 */
zone_error_t Zones_ClearSelection(void);

/**
 * @brief Select all keys in current zone
 * @return zone_error_t Error code
 */
zone_error_t Zones_SelectAllKeys(void);

/* ========================================================================== */
/* HSV Color Selection */
/* ========================================================================== */

/**
 * @brief Enter HSV color adjustment mode
 * @return zone_error_t Error code
 */
zone_error_t Zones_EnterHSVMode(void);

/**
 * @brief Adjust hue using arrow keys (left/right)
 * @param direction -1 for left (decrease), +1 for right (increase)
 * @return zone_error_t Error code
 */
zone_error_t Zones_AdjustHue(int8_t direction);

/**
 * @brief Adjust brightness using arrow keys (up/down)
 * @param direction -1 for down (decrease), +1 for up (increase)
 * @return zone_error_t Error code
 */
zone_error_t Zones_AdjustBrightness(int8_t direction);

/**
 * @brief Apply current HSV color to selected keys
 * @return zone_error_t Error code
 */
zone_error_t Zones_ApplyHSVColor(void);

/**
 * @brief Get current HSV values
 * @param h Pointer to store hue (0-359)
 * @param s Pointer to store saturation (0-100)
 * @param v Pointer to store value (0-100)
 * @return zone_error_t Error code
 */
zone_error_t Zones_GetHSV(uint16_t *h, uint8_t *s, uint8_t *v);

/* ========================================================================== */
/* Zone Save/Load Operations */
/* ========================================================================== */

/**
 * @brief Save staging zone to EEPROM slot (Ctrl+S)
 * @param slot_id Target slot (0-7)
 * @return zone_error_t Error code
 */
zone_error_t Zones_SaveZone(uint8_t slot_id);

/**
 * @brief Load zone from EEPROM slot to staging area
 * @param slot_id Source slot (0-7)
 * @return zone_error_t Error code
 */
zone_error_t Zones_LoadZone(uint8_t slot_id);

/**
 * @brief Delete zone from EEPROM slot
 * @param slot_id Slot to delete (0-7)
 * @return zone_error_t Error code
 */
zone_error_t Zones_DeleteZone(uint8_t slot_id);

/**
 * @brief Activate saved zone (apply to keyboard)
 * @param slot_id Zone slot to activate (0-7)
 * @return zone_error_t Error code
 */
zone_error_t Zones_ActivateZone(uint8_t slot_id);

/* ========================================================================== */
/* Zone Validation and Utilities */
/* ========================================================================== */

/**
 * @brief Validate zone data integrity
 * @param zone Pointer to zone data
 * @return zone_error_t Error code
 */
zone_error_t Zones_ValidateZone(const zone_storage_t *zone);

/**
 * @brief Calculate CRC16 for zone data
 * @param zone Pointer to zone data
 * @return uint16_t CRC16 checksum
 */
uint16_t Zones_CalculateCRC16(const zone_storage_t *zone);

/**
 * @brief Check if zone slot is occupied
 * @param slot_id Zone slot (0-7)
 * @return bool True if slot has saved zone
 */
bool Zones_IsSlotOccupied(uint8_t slot_id);

/**
 * @brief Get zone key count
 * @param slot_id Zone slot (0-7)
 * @return uint8_t Number of active keys in zone
 */
uint8_t Zones_GetKeyCount(uint8_t slot_id);

/**
 * @brief Copy zone data between slots
 * @param src_slot Source slot (0-7)
 * @param dst_slot Destination slot (0-7)
 * @return zone_error_t Error code
 */
zone_error_t Zones_CopyZone(uint8_t src_slot, uint8_t dst_slot);

/* ========================================================================== */
/* Visual Feedback Functions */
/* ========================================================================== */

/**
 * @brief Show zone browse interface (red/green slot indicators)
 * @return zone_error_t Error code
 */
zone_error_t Zones_ShowBrowseInterface(void);

/**
 * @brief Show key selection interface (selected keys = green)
 * @return zone_error_t Error code
 */
zone_error_t Zones_ShowSelectionInterface(void);

/**
 * @brief Show HSV color adjustment feedback
 * @return zone_error_t Error code
 */
zone_error_t Zones_ShowHSVFeedback(void);

/**
 * @brief Flash save success (green flash)
 * @return zone_error_t Error code
 */
zone_error_t Zones_FlashSaveSuccess(void);

/**
 * @brief Flash save error (red flash 3x for 3 seconds)
 * @return zone_error_t Error code
 */
zone_error_t Zones_FlashSaveError(void);

/**
 * @brief Restore previous keyboard state
 * Resumes effects that were active before zone programming
 * @return zone_error_t Error code
 */
zone_error_t Zones_RestoreKeyboardState(void);

/* ========================================================================== */
/* Configuration and Status */
/* ========================================================================== */

/**
 * @brief Get programming context (for debugging)
 * @return const zone_programming_context_t* Read-only context
 */
const zone_programming_context_t* Zones_GetContext(void);

/**
 * @brief Get last error information
 * @return zone_error_t Last error code
 */
zone_error_t Zones_GetLastError(void);

/**
 * @brief Reset error counters
 */
void Zones_ResetErrors(void);

/**
 * @brief Set zone programming configuration
 * @param auto_exit Enable auto-exit timeout
 * @param preview_duration_ms Preview display duration
 * @param default_brightness Default brightness for new zones
 * @return zone_error_t Error code
 */
zone_error_t Zones_SetConfig(bool auto_exit, uint16_t preview_duration_ms, uint8_t default_brightness);

/* ========================================================================== */
/* EEPROM Storage Interface (for future implementation) */
/* ========================================================================== */

/**
 * @brief EEPROM storage interface - to be implemented
 */
typedef struct {
    zone_error_t (*write_zone)(uint8_t slot_id, const zone_storage_t *zone);
    zone_error_t (*read_zone)(uint8_t slot_id, zone_storage_t *zone);
    zone_error_t (*erase_zone)(uint8_t slot_id);
    zone_error_t (*format_storage)(void);
    bool (*is_slot_valid)(uint8_t slot_id);
} zone_storage_interface_t;

/**
 * @brief Register EEPROM storage interface
 * @param interface Pointer to storage interface functions
 * @return zone_error_t Error code
 */
zone_error_t Zones_RegisterStorageInterface(const zone_storage_interface_t *interface);

/* ========================================================================== */
/* Factory Reset and Defaults */
/* ========================================================================== */

/**
 * @brief Factory reset all zones to defaults
 * @return zone_error_t Error code
 */
zone_error_t Zones_FactoryReset(void);

/**
 * @brief Create default zone configurations
 * @return zone_error_t Error code
 */
zone_error_t Zones_CreateDefaults(void);

/* ========================================================================== */
/* Convenience Macros */
/* ========================================================================== */

// Zone slot shortcuts
#define ZONE_SLOT_1     0
#define ZONE_SLOT_2     1
#define ZONE_SLOT_3     2
#define ZONE_SLOT_4     3
#define ZONE_SLOT_5     4
#define ZONE_SLOT_6     5
#define ZONE_SLOT_7     6
#define ZONE_SLOT_8     7

// HSV adjustment steps
#define HSV_HUE_STEP_FINE       5       // Fine hue adjustment
#define HSV_HUE_STEP_COARSE     30      // Coarse hue adjustment
#define HSV_SV_STEP_FINE        2       // Fine saturation/value adjustment
#define HSV_SV_STEP_COARSE      10      // Coarse saturation/value adjustment

// Visual feedback durations
#define FLASH_SUCCESS_DURATION_MS       500     // Green success flash
#define FLASH_ERROR_DURATION_MS         3000    // Red error flash (3 seconds)
#define FLASH_ERROR_COUNT               3       // Number of error flashes

// Validation macros
#define ZONE_VALID_SLOT(slot)           ((slot) < ZONE_MAX_SLOTS)
#define ZONE_VALID_KEY(row, col)        ((row) < BACKLIGHT_MATRIX_ROWS && (col) < BACKLIGHT_MATRIX_COLS)
#define ZONE_VALID_HSV_HUE(h)          ((h) < 360)
#define ZONE_VALID_HSV_SV(sv)          ((sv) <= 100)

#endif /* BACKLIGHT_CORE_EXTENSION_ZONES_H_ */
