/*
* zones.c - Visual Zone Programming Implementation
*
* Created on: Jun 10, 2025
* Author: bettysidepiece
*/

#include "zones.h"
#include "effects.h"
#include "colors.h"
#include "logger.h"
#include "main.h"
#include "led_mapping.h"
#include <string.h>

/* ========================================================================== */
/* Private Constants */
/* ========================================================================== */

#define ZONE_CRC_POLYNOMIAL         0x1021      // CRC-16-CCITT polynomial
#define ZONE_VERSION_CURRENT        1           // Current zone format version
#define ZONE_MAGIC_BYTE             0xA5        // Magic byte for validation
#define ZONE_MAX_RETRIES            3           // Maximum operation retries
#define ZONE_DEBOUNCE_MS            50          // Key debounce time

// HSV adjustment parameters
#define HSV_HUE_STEP_DEFAULT        15          // Default hue step
#define HSV_SV_STEP_DEFAULT         10          // Default saturation/value step
#define HSV_HUE_FAST_STEP           45          // Fast hue adjustment
#define HSV_SV_FAST_STEP            25          // Fast saturation/value adjustment

// Visual feedback timings
#define FLASH_PULSE_DURATION_MS     200         // Duration of each flash pulse
#define PREVIEW_FADE_IN_MS          25          // Preview fade in time
#define PREVIEW_FADE_OUT_MS         30          // Preview fade out time

// Zone slot to key mapping (using number row keys 1-8)
#define ZONE_SLOT_ROW               1           // Number row
#define ZONE_SLOT_START_COL         1           // Start at key "1"

/* ========================================================================== */
/* Private Variables */
/* ========================================================================== */

static zone_programming_context_t g_zone_ctx = {0};
static bool g_initialized = false;
static bool g_led_control_active = false;
static const zone_storage_interface_t *g_storage_interface = NULL;

// Saved effects state for restoration
static bool g_effects_were_active = false;
static effect_type_t g_previous_effect_type = EFFECT_NONE;



/* ========================================================================== */
/* Private Function Prototypes */
/* ========================================================================== */

static zone_error_t ProcessStateMachine(void);
static zone_error_t ProcessBrowseState(void);
static zone_error_t ProcessEditState(void);
static zone_error_t ProcessHSVState(void);
static zone_error_t ProcessSaveConfirmState(void);

static zone_error_t InitializeZoneSlots(void);
static zone_error_t ValidateZoneIntegrity(const zone_storage_t *zone);
static uint16_t CalculateZoneCRC16(const zone_storage_t *zone);
static zone_error_t CreateDefaultZone(zone_storage_t *zone, uint8_t slot_id);

static zone_error_t UpdateVisualFeedback(void);
static zone_error_t ShowSlotIndicators(void);
static zone_error_t ShowKeySelection(void);
static zone_error_t ShowHSVPreview(void);
static zone_error_t FlashConfirmation(bool success);

static zone_error_t SaveEffectsState(void);
static zone_error_t RestoreEffectsState(void);
static zone_error_t SuspendEffects(void);
static zone_error_t ResumeEffects(void);

static bool IsValidSlot(uint8_t slot_id);
static bool IsValidKey(uint8_t row, uint8_t col);
static bool HasUserActivity(void);
static void UpdateActivityTime(void);
static zone_error_t HandleTimeout(void);
static zone_error_t AddKeyToZone(zone_storage_t *zone, uint8_t row, uint8_t col, Backlight_RGB_t color);
static zone_error_t RemoveKeyFromZone(zone_storage_t *zone, uint8_t row, uint8_t col);
static zone_key_data_t* FindKeyInZone(zone_storage_t *zone, uint8_t row, uint8_t col);

/* ========================================================================== */
/* Public API Implementation */
/* ========================================================================== */

zone_error_t Zones_Init(void)
{
    if (g_initialized) {
        return ZONE_ERROR_NONE;
    }

    // Initialize context
    memset(&g_zone_ctx, 0, sizeof(g_zone_ctx));

    // Set initial state
    g_zone_ctx.current_state = ZONE_STATE_INACTIVE;
    g_zone_ctx.edit_mode = ZONE_EDIT_MODE_INDIVIDUAL;
    g_zone_ctx.last_activity_time = HAL_GetTick();

    // Initialize HSV defaults
    g_zone_ctx.hsv.hue = 30;             // Orange/amber
    g_zone_ctx.hsv.saturation = 100;     // Full saturation
    g_zone_ctx.hsv.value = 80;           // 80% brightness
    g_zone_ctx.hsv.hue_step = HSV_HUE_STEP_DEFAULT;
    g_zone_ctx.hsv.sv_step = HSV_SV_STEP_DEFAULT;

    // Initialize configuration
    g_zone_ctx.auto_exit_enabled = true;
    g_zone_ctx.preview_duration_ms = ZONE_PREVIEW_DELAY_MS;
    g_zone_ctx.default_brightness = 5;   // Mid-range brightness level

    // Initialize zone slots
    zone_error_t result = InitializeZoneSlots();
    if (result != ZONE_ERROR_NONE) {
        LOG_ERROR("Zone slots initialization failed: %d", result);
        return result;
    }

    g_led_control_active = false;
    g_initialized = true;

    LOG_INFO("Zone programming system initialized");
    return ZONE_ERROR_NONE;
}

zone_error_t Zones_Process(void)
{
    if (!g_initialized) {
        return ZONE_ERROR_NONE;
    }

    // Check for timeout if in programming mode
    if (g_zone_ctx.current_state != ZONE_STATE_INACTIVE) {
        zone_error_t timeout_result = HandleTimeout();
        if (timeout_result != ZONE_ERROR_NONE) {
            return timeout_result;
        }
    }

    // Process state machine
    zone_error_t result = ProcessStateMachine();
    if (result != ZONE_ERROR_NONE) {
        g_zone_ctx.last_error = result;
        g_zone_ctx.error_count++;
        LOG_ERROR("Zone state machine error: %d", result);
    }

    return result;
}

bool Zones_IsActive(void)
{
    return g_initialized && (g_zone_ctx.current_state != ZONE_STATE_INACTIVE);
}

zone_state_t Zones_GetState(void)
{
    return g_initialized ? g_zone_ctx.current_state : ZONE_STATE_INACTIVE;
}

bool Zones_HasLEDControl(void)
{
    return g_led_control_active;
}

/* ========================================================================== */
/* Zone Programming Entry/Exit */
/* ========================================================================== */

zone_error_t Zones_EnterProgramming(void)
{
    if (!g_initialized) {
        return ZONE_ERROR_INVALID_STATE;
    }

    if (g_zone_ctx.current_state != ZONE_STATE_INACTIVE) {
        LOG_WARNING("Already in programming mode");
        return ZONE_ERROR_NONE;
    }

    LOG_INFO("Entering zone programming mode");

    // Save and suspend effects
    zone_error_t result = SaveEffectsState();
    if (result != ZONE_ERROR_NONE) {
        LOG_ERROR("Failed to save effects state: %d", result);
        return result;
    }

    result = SuspendEffects();
    if (result != ZONE_ERROR_NONE) {
        LOG_ERROR("Failed to suspend effects: %d", result);
        return result;
    }

    // Take LED control
    g_led_control_active = true;

    // Initialize programming session
    g_zone_ctx.current_state = ZONE_STATE_BROWSE;
    g_zone_ctx.state_start_time = HAL_GetTick();
    g_zone_ctx.last_activity_time = HAL_GetTick();

    // Clear any previous selection
    memset(&g_zone_ctx.selection, 0, sizeof(g_zone_ctx.selection));

    // Enter browse mode to show slot selection
    return Zones_EnterBrowseMode();
}

zone_error_t Zones_ExitProgramming(bool save_changes)
{
    if (!g_initialized || g_zone_ctx.current_state == ZONE_STATE_INACTIVE) {
        return ZONE_ERROR_INVALID_STATE;
    }

    LOG_INFO("Exiting zone programming mode (save=%d)", save_changes);

    zone_error_t result = ZONE_ERROR_NONE;

    // Save changes if requested and there are changes to save
    if (save_changes && g_zone_ctx.manager.has_changes) {
        result = Zones_SaveZone(g_zone_ctx.manager.editing_slot);
        if (result == ZONE_ERROR_NONE) {
            // Flash success confirmation
            FlashConfirmation(true);

            // Apply the zone immediately
            Zones_ActivateZone(g_zone_ctx.manager.editing_slot);
        } else {
            // Flash error confirmation
            FlashConfirmation(false);
            LOG_ERROR("Failed to save zone changes: %d", result);
        }
    }

    // Release LED control
    g_led_control_active = false;

    // Reset state
    g_zone_ctx.current_state = ZONE_STATE_INACTIVE;
    g_zone_ctx.manager.has_changes = false;

    // Restore effects if no zone was applied or save failed
    if (!save_changes || result != ZONE_ERROR_NONE) {
        zone_error_t restore_result = RestoreEffectsState();
        if (restore_result != ZONE_ERROR_NONE) {
            LOG_ERROR("Failed to restore effects state: %d", restore_result);
        }
    }

    return result;
}

zone_error_t Zones_HandleTimeout(void)
{
    return HandleTimeout();
}

/* ========================================================================== */
/* Zone Browse Mode */
/* ========================================================================== */

zone_error_t Zones_EnterBrowseMode(void)
{
    if (!g_initialized) {
        return ZONE_ERROR_INVALID_STATE;
    }

    LOG_DEBUG("Entering zone browse mode");

    g_zone_ctx.current_state = ZONE_STATE_BROWSE;
    g_zone_ctx.state_start_time = HAL_GetTick();
    UpdateActivityTime();

    // Show slot indicators (red/green for empty/occupied)
    return ShowSlotIndicators();
}

zone_error_t Zones_SelectSlot(uint8_t slot_id)
{
    if (!IsValidSlot(slot_id)) {
        return ZONE_ERROR_INVALID_SLOT;
    }

    UpdateActivityTime();

    LOG_DEBUG("Selected slot %d", slot_id);

    g_zone_ctx.manager.editing_slot = slot_id;

    // Load zone data if slot is occupied
    if (g_zone_ctx.manager.slot_occupied[slot_id]) {
        zone_error_t result = Zones_LoadZone(slot_id);
        if (result != ZONE_ERROR_NONE) {
            LOG_ERROR("Failed to load zone %d: %d", slot_id, result);
            return result;
        }
        LOG_INFO("Loaded existing zone %d with %d keys", slot_id,
                 g_zone_ctx.manager.staging_zone.metadata.key_count);
    } else {
        // Create new zone
        zone_error_t result = CreateDefaultZone(&g_zone_ctx.manager.staging_zone, slot_id);
        if (result != ZONE_ERROR_NONE) {
            LOG_ERROR("Failed to create default zone: %d", result);
            return result;
        }
        LOG_INFO("Created new zone %d", slot_id);
    }

    // Enter edit mode
    return Zones_EnterEditMode(slot_id);
}

zone_error_t Zones_PreviewSlot(uint8_t slot_id)
{
    if (!IsValidSlot(slot_id) || !g_zone_ctx.manager.slot_occupied[slot_id]) {
        return ZONE_ERROR_INVALID_SLOT;
    }

    UpdateActivityTime();

    LOG_DEBUG("Previewing slot %d", slot_id);

    g_zone_ctx.preview_active = true;
    g_zone_ctx.preview_slot = slot_id;
    g_zone_ctx.preview_start_time = HAL_GetTick();

    // Temporarily load zone for preview
    zone_storage_t preview_zone;
    zone_error_t result = ZONE_ERROR_NONE;

    if (g_storage_interface && g_storage_interface->read_zone) {
        result = g_storage_interface->read_zone(slot_id, &preview_zone);
        if (result != ZONE_ERROR_NONE) {
            g_zone_ctx.preview_active = false;
            return result;
        }

        // Apply preview zone colors
        for (uint8_t i = 0; i < preview_zone.metadata.key_count; i++) {
            zone_key_data_t *key = &preview_zone.keys[i];
            if (key->active && IsValidKey(key->row, key->col)) {
                Backlight_SetKeyRGB(key->row, key->col, key->color);
            }
        }
    }

    return ZONE_ERROR_NONE;
}

zone_error_t Zones_QuickSwitch(void)
{
    if (!g_initialized) {
        return ZONE_ERROR_INVALID_STATE;
    }

    // Find first two occupied slots
    uint8_t first_slot = 255, second_slot = 255;
    for (uint8_t i = 0; i < ZONE_MAX_SLOTS; i++) {
        if (g_zone_ctx.manager.slot_occupied[i]) {
            if (first_slot == 255) {
                first_slot = i;
            } else if (second_slot == 255) {
                second_slot = i;
                break;
            }
        }
    }

    if (first_slot == 255) {
        LOG_WARNING("No zones available for quick switch");
        return ZONE_ERROR_INVALID_SLOT;
    }

    // Switch between first and second, or just activate first if only one exists
    uint8_t target_slot;
    if (second_slot == 255 || g_zone_ctx.manager.active_slot != first_slot) {
        target_slot = first_slot;
    } else {
        target_slot = second_slot;
    }

    LOG_INFO("Quick switching to zone slot %d", target_slot);
    return Zones_ActivateZone(target_slot);
}

/* ========================================================================== */
/* Zone Edit Mode */
/* ========================================================================== */

zone_error_t Zones_EnterEditMode(uint8_t slot_id)
{
    if (!IsValidSlot(slot_id)) {
        return ZONE_ERROR_INVALID_SLOT;
    }

    LOG_DEBUG("Entering edit mode for slot %d", slot_id);

    g_zone_ctx.current_state = ZONE_STATE_EDIT;
    g_zone_ctx.state_start_time = HAL_GetTick();
    g_zone_ctx.manager.editing_slot = slot_id;
    UpdateActivityTime();

    // Clear previous selection
    memset(&g_zone_ctx.selection, 0, sizeof(g_zone_ctx.selection));

    // Show current zone keys and selection interface
    return ShowKeySelection();
}

zone_error_t Zones_ToggleEditMode(void)
{
    if (g_zone_ctx.current_state != ZONE_STATE_EDIT) {
        return ZONE_ERROR_INVALID_STATE;
    }

    UpdateActivityTime();

    // Cycle through edit modes
    switch (g_zone_ctx.edit_mode) {
        case ZONE_EDIT_MODE_INDIVIDUAL:
            g_zone_ctx.edit_mode = ZONE_EDIT_MODE_GROUP;
            LOG_DEBUG("Switched to group edit mode");
            break;
        case ZONE_EDIT_MODE_GROUP:
            g_zone_ctx.edit_mode = ZONE_EDIT_MODE_ZONE_WIDE;
            LOG_DEBUG("Switched to zone-wide edit mode");
            break;
        case ZONE_EDIT_MODE_ZONE_WIDE:
            g_zone_ctx.edit_mode = ZONE_EDIT_MODE_INDIVIDUAL;
            LOG_DEBUG("Switched to individual edit mode");
            break;
    }

    // Update visual feedback based on new mode
    return ShowKeySelection();
}

zone_error_t Zones_ToggleKeySelection(uint8_t row, uint8_t col)
{
    if (!IsValidKey(row, col)) {
        return ZONE_ERROR_INVALID_STATE;
    }

    if (g_zone_ctx.current_state != ZONE_STATE_EDIT) {
        return ZONE_ERROR_INVALID_STATE;
    }

    UpdateActivityTime();

    bool was_selected = g_zone_ctx.selection.selected[row][col];
    g_zone_ctx.selection.selected[row][col] = !was_selected;

    if (was_selected) {
        g_zone_ctx.selection.selection_count--;
        LOG_DEBUG("Deselected key (%d,%d)", row, col);
    } else {
        g_zone_ctx.selection.selection_count++;
        g_zone_ctx.selection.last_row = row;
        g_zone_ctx.selection.last_col = col;
        LOG_DEBUG("Selected key (%d,%d)", row, col);
    }

    // Update visual feedback
    return ShowKeySelection();
}

zone_error_t Zones_ClearSelection(void)
{
    if (g_zone_ctx.current_state != ZONE_STATE_EDIT) {
        return ZONE_ERROR_INVALID_STATE;
    }

    UpdateActivityTime();

    memset(&g_zone_ctx.selection.selected, 0, sizeof(g_zone_ctx.selection.selected));
    g_zone_ctx.selection.selection_count = 0;

    LOG_DEBUG("Cleared key selection");

    // Update visual feedback
    return ShowKeySelection();
}

zone_error_t Zones_SelectAllKeys(void)
{
    if (g_zone_ctx.current_state != ZONE_STATE_EDIT) {
        return ZONE_ERROR_INVALID_STATE;
    }

    UpdateActivityTime();

    // Select all keys that have LEDs based on the LED mapping
    g_zone_ctx.selection.selection_count = 0;
    for (uint8_t row = 0; row < BACKLIGHT_MATRIX_ROWS; row++) {
        for (uint8_t col = 0; col < BACKLIGHT_MATRIX_COLS; col++) {
            if (get_led_for_key(row, col) != NULL) {
                g_zone_ctx.selection.selected[row][col] = true;
                g_zone_ctx.selection.selection_count++;
            }
        }
    }

    LOG_DEBUG("Selected all keys (%d total)", g_zone_ctx.selection.selection_count);

    // Update visual feedback
    return ShowKeySelection();
}

/* ========================================================================== */
/* HSV Color Selection */
/* ========================================================================== */

zone_error_t Zones_EnterHSVMode(void)
{
    if (g_zone_ctx.current_state != ZONE_STATE_EDIT) {
        return ZONE_ERROR_INVALID_STATE;
    }

    if (g_zone_ctx.selection.selection_count == 0) {
        LOG_WARNING("No keys selected for HSV editing");
        return ZONE_ERROR_INVALID_STATE;
    }

    LOG_DEBUG("Entering HSV adjustment mode with %d keys selected",
              g_zone_ctx.selection.selection_count);

    g_zone_ctx.previous_state = g_zone_ctx.current_state;
    g_zone_ctx.current_state = ZONE_STATE_HSV_ADJUST;
    g_zone_ctx.state_start_time = HAL_GetTick();
    UpdateActivityTime();

    // Show HSV preview
    return ShowHSVPreview();
}

zone_error_t Zones_AdjustHue(int8_t direction)
{
    if (g_zone_ctx.current_state != ZONE_STATE_HSV_ADJUST) {
        return ZONE_ERROR_INVALID_STATE;
    }

    UpdateActivityTime();

    int16_t new_hue = g_zone_ctx.hsv.hue + (direction * g_zone_ctx.hsv.hue_step);

    // Wrap around hue (0-359)
    if (new_hue < 0) {
        new_hue += 360;
    } else if (new_hue >= 360) {
        new_hue -= 360;
    }

    g_zone_ctx.hsv.hue = (uint16_t)new_hue;

    LOG_DEBUG("Adjusted hue to %d", g_zone_ctx.hsv.hue);

    // Update preview
    return ShowHSVPreview();
}

zone_error_t Zones_AdjustBrightness(int8_t direction)
{
    if (g_zone_ctx.current_state != ZONE_STATE_HSV_ADJUST) {
        return ZONE_ERROR_INVALID_STATE;
    }

    UpdateActivityTime();

    int16_t new_value = g_zone_ctx.hsv.value + (direction * g_zone_ctx.hsv.sv_step);

    // Clamp value (0-100)
    if (new_value < 0) {
        new_value = 0;
    } else if (new_value > 100) {
        new_value = 100;
    }

    g_zone_ctx.hsv.value = (uint8_t)new_value;

    LOG_DEBUG("Adjusted brightness to %d", g_zone_ctx.hsv.value);

    // Update preview
    return ShowHSVPreview();
}

zone_error_t Zones_ApplyHSVColor(void)
{
    if (g_zone_ctx.current_state != ZONE_STATE_HSV_ADJUST) {
        return ZONE_ERROR_INVALID_STATE;
    }

    if (g_zone_ctx.selection.selection_count == 0) {
        return ZONE_ERROR_INVALID_STATE;
    }

    UpdateActivityTime();

    // Convert HSV to RGB
    Backlight_RGB_t rgb_color = Backlight_HSVtoRGB(g_zone_ctx.hsv.hue,
                                                   g_zone_ctx.hsv.saturation,
                                                   g_zone_ctx.hsv.value);

    LOG_DEBUG("Applying HSV(%d,%d,%d) -> RGB(%d,%d,%d) to %d keys",
              g_zone_ctx.hsv.hue, g_zone_ctx.hsv.saturation, g_zone_ctx.hsv.value,
              rgb_color.r, rgb_color.g, rgb_color.b, g_zone_ctx.selection.selection_count);

    // Apply color to selected keys in staging zone
    for (uint8_t row = 0; row < BACKLIGHT_MATRIX_ROWS; row++) {
        for (uint8_t col = 0; col < BACKLIGHT_MATRIX_COLS; col++) {
            if (g_zone_ctx.selection.selected[row][col]) {
                zone_error_t result = AddKeyToZone(&g_zone_ctx.manager.staging_zone, row, col, rgb_color);
                if (result != ZONE_ERROR_NONE) {
                    LOG_ERROR("Failed to add key (%d,%d) to zone: %d", row, col, result);
                }

                // Apply to hardware immediately for visual feedback
                Backlight_SetKeyRGB(row, col, rgb_color);
            }
        }
    }

    // Mark as changed
    g_zone_ctx.manager.has_changes = true;

    // Return to edit mode
    g_zone_ctx.current_state = g_zone_ctx.previous_state;

    LOG_INFO("Applied color to %d keys, zone now has %d total keys",
             g_zone_ctx.selection.selection_count,
             g_zone_ctx.manager.staging_zone.metadata.key_count);

    return ZONE_ERROR_NONE;
}

zone_error_t Zones_GetHSV(uint16_t *h, uint8_t *s, uint8_t *v)
{
    if (!h || !s || !v) {
        return ZONE_ERROR_INVALID_PARAM;
    }

    *h = g_zone_ctx.hsv.hue;
    *s = g_zone_ctx.hsv.saturation;
    *v = g_zone_ctx.hsv.value;

    return ZONE_ERROR_NONE;
}

/* ========================================================================== */
/* Zone Save/Load Operations */
/* ========================================================================== */

zone_error_t Zones_SaveZone(uint8_t slot_id)
{
    if (!IsValidSlot(slot_id)) {
        return ZONE_ERROR_INVALID_SLOT;
    }

    if (!g_storage_interface || !g_storage_interface->write_zone) {
        LOG_ERROR("No storage interface registered");
        return ZONE_ERROR_MEMORY;
    }

    LOG_INFO("Saving zone to slot %d (%d keys)", slot_id,
             g_zone_ctx.manager.staging_zone.metadata.key_count);

    // Update metadata
    g_zone_ctx.manager.staging_zone.metadata.slot_id = slot_id;
    g_zone_ctx.manager.staging_zone.metadata.version = ZONE_VERSION_CURRENT;
    g_zone_ctx.manager.staging_zone.metadata.modified_timestamp = HAL_GetTick();

    // Set creation timestamp if this is a new zone
    if (!g_zone_ctx.manager.slot_occupied[slot_id]) {
        g_zone_ctx.manager.staging_zone.metadata.created_timestamp = HAL_GetTick();
    }

    // Calculate CRC
    g_zone_ctx.manager.staging_zone.metadata.crc16 = CalculateZoneCRC16(&g_zone_ctx.manager.staging_zone);

    // Write to storage
    zone_error_t result = g_storage_interface->write_zone(slot_id, &g_zone_ctx.manager.staging_zone);
    if (result != ZONE_ERROR_NONE) {
        LOG_ERROR("Failed to write zone %d to storage: %d", slot_id, result);
        return result;
    }

    // Mark slot as occupied
    g_zone_ctx.manager.slot_occupied[slot_id] = true;
    g_zone_ctx.manager.has_changes = false;

    LOG_INFO("Zone %d saved successfully", slot_id);
    return ZONE_ERROR_NONE;
}

zone_error_t Zones_LoadZone(uint8_t slot_id)
{
    if (!IsValidSlot(slot_id)) {
        return ZONE_ERROR_INVALID_SLOT;
    }

    if (!g_storage_interface || !g_storage_interface->read_zone) {
        LOG_ERROR("No storage interface registered");
        return ZONE_ERROR_MEMORY;
    }

    LOG_DEBUG("Loading zone from slot %d", slot_id);

    zone_error_t result = g_storage_interface->read_zone(slot_id, &g_zone_ctx.manager.staging_zone);
    if (result != ZONE_ERROR_NONE) {
        LOG_ERROR("Failed to read zone %d from storage: %d", slot_id, result);
        return result;
    }

    // Validate zone integrity
    result = ValidateZoneIntegrity(&g_zone_ctx.manager.staging_zone);
    if (result != ZONE_ERROR_NONE) {
        LOG_ERROR("Zone %d failed integrity check: %d", slot_id, result);
        return result;
    }

    LOG_DEBUG("Zone %d loaded successfully (%d keys)", slot_id,
              g_zone_ctx.manager.staging_zone.metadata.key_count);
    return ZONE_ERROR_NONE;
}

zone_error_t Zones_DeleteZone(uint8_t slot_id)
{
    if (!IsValidSlot(slot_id)) {
        return ZONE_ERROR_INVALID_SLOT;
    }

    if (!g_storage_interface || !g_storage_interface->erase_zone) {
        LOG_ERROR("No storage interface registered");
        return ZONE_ERROR_MEMORY;
    }

    LOG_INFO("Deleting zone from slot %d", slot_id);

    zone_error_t result = g_storage_interface->erase_zone(slot_id);
    if (result != ZONE_ERROR_NONE) {
        LOG_ERROR("Failed to erase zone %d: %d", slot_id, result);
        return result;
    }

    // Mark slot as unoccupied
    g_zone_ctx.manager.slot_occupied[slot_id] = false;

    LOG_INFO("Zone %d deleted successfully", slot_id);
    return ZONE_ERROR_NONE;
}

zone_error_t Zones_ActivateZone(uint8_t slot_id)
{
    if (!IsValidSlot(slot_id) || !g_zone_ctx.manager.slot_occupied[slot_id]) {
        return ZONE_ERROR_INVALID_SLOT;
    }

    LOG_INFO("Activating zone %d", slot_id);

    // Load zone to staging area first
    zone_error_t result = Zones_LoadZone(slot_id);
    if (result != ZONE_ERROR_NONE) {
        return result;
    }

    // Copy to active zone
    g_zone_ctx.manager.active_zone = g_zone_ctx.manager.staging_zone;
    g_zone_ctx.manager.active_slot = slot_id;

    // Clear all LEDs first
    Backlight_SetAll(0, 0, 0);

    // Apply zone colors to keyboard
    for (uint8_t i = 0; i < g_zone_ctx.manager.active_zone.metadata.key_count; i++) {
        zone_key_data_t *key = &g_zone_ctx.manager.active_zone.keys[i];
        if (key->active && IsValidKey(key->row, key->col)) {
            Backlight_SetKeyRGB(key->row, key->col, key->color);
        }
    }

    // Clear saved backlight state since we're applying a zone
    Backlight_ClearSavedState();

    LOG_INFO("Zone %d activated successfully (%d keys)", slot_id,
             g_zone_ctx.manager.active_zone.metadata.key_count);
    return ZONE_ERROR_NONE;
}

/* ========================================================================== */
/* Zone Validation and Utilities */
/* ========================================================================== */

zone_error_t Zones_ValidateZone(const zone_storage_t *zone)
{
    return ValidateZoneIntegrity(zone);
}

uint16_t Zones_CalculateCRC16(const zone_storage_t *zone)
{
    return CalculateZoneCRC16(zone);
}

bool Zones_IsSlotOccupied(uint8_t slot_id)
{
    if (!IsValidSlot(slot_id)) {
        return false;
    }
    return g_zone_ctx.manager.slot_occupied[slot_id];
}

uint8_t Zones_GetKeyCount(uint8_t slot_id)
{
    if (!IsValidSlot(slot_id) || !g_zone_ctx.manager.slot_occupied[slot_id]) {
        return 0;
    }

    // If this is the currently loaded slot, return from staging zone
    if (slot_id == g_zone_ctx.manager.editing_slot) {
        return g_zone_ctx.manager.staging_zone.metadata.key_count;
    }

    // For other slots, would need to load from storage (simplified for now)
    return 0;
}

zone_error_t Zones_CopyZone(uint8_t src_slot, uint8_t dst_slot)
{
    if (!IsValidSlot(src_slot) || !IsValidSlot(dst_slot)) {
        return ZONE_ERROR_INVALID_SLOT;
    }

    if (!g_zone_ctx.manager.slot_occupied[src_slot]) {
        return ZONE_ERROR_INVALID_SLOT;
    }

    LOG_INFO("Copying zone from slot %d to slot %d", src_slot, dst_slot);

    // Load source zone
    zone_storage_t temp_zone;
    if (!g_storage_interface || !g_storage_interface->read_zone) {
        return ZONE_ERROR_MEMORY;
    }

    zone_error_t result = g_storage_interface->read_zone(src_slot, &temp_zone);
    if (result != ZONE_ERROR_NONE) {
        return result;
    }

    // Update metadata for destination
    temp_zone.metadata.slot_id = dst_slot;
    temp_zone.metadata.created_timestamp = HAL_GetTick();
    temp_zone.metadata.modified_timestamp = HAL_GetTick();
    temp_zone.metadata.crc16 = CalculateZoneCRC16(&temp_zone);

    // Write to destination
    result = g_storage_interface->write_zone(dst_slot, &temp_zone);
    if (result != ZONE_ERROR_NONE) {
        return result;
    }

    g_zone_ctx.manager.slot_occupied[dst_slot] = true;

    LOG_INFO("Zone copied successfully");
    return ZONE_ERROR_NONE;
}

/* ========================================================================== */
/* Visual Feedback Functions */
/* ========================================================================== */

zone_error_t Zones_ShowBrowseInterface(void)
{
    return ShowSlotIndicators();
}

zone_error_t Zones_ShowSelectionInterface(void)
{
    return ShowKeySelection();
}

zone_error_t Zones_ShowHSVFeedback(void)
{
    return ShowHSVPreview();
}

zone_error_t Zones_FlashSaveSuccess(void)
{
    return FlashConfirmation(true);
}

zone_error_t Zones_FlashSaveError(void)
{
    return FlashConfirmation(false);
}

zone_error_t Zones_RestoreKeyboardState(void)
{
    return RestoreEffectsState();
}

/* ========================================================================== */
/* Configuration and Status */
/* ========================================================================== */

const zone_programming_context_t* Zones_GetContext(void)
{
    return &g_zone_ctx;
}

zone_error_t Zones_GetLastError(void)
{
    return g_zone_ctx.last_error;
}

void Zones_ResetErrors(void)
{
    g_zone_ctx.last_error = ZONE_ERROR_NONE;
    g_zone_ctx.error_count = 0;
}

zone_error_t Zones_SetConfig(bool auto_exit, uint16_t preview_duration_ms, uint8_t default_brightness)
{
    if (preview_duration_ms == 0 || default_brightness == 0 || default_brightness > 10) {
        return ZONE_ERROR_INVALID_PARAM;
    }

    g_zone_ctx.auto_exit_enabled = auto_exit;
    g_zone_ctx.preview_duration_ms = preview_duration_ms;
    g_zone_ctx.default_brightness = default_brightness;

    LOG_INFO("Zone config updated: auto_exit=%d, preview=%dms, brightness=%d",
             auto_exit, preview_duration_ms, default_brightness);

    return ZONE_ERROR_NONE;
}

/* ========================================================================== */
/* EEPROM Storage Interface */
/* ========================================================================== */

zone_error_t Zones_RegisterStorageInterface(const zone_storage_interface_t *interface)
{
    if (!interface) {
        return ZONE_ERROR_INVALID_PARAM;
    }

    // Validate required function pointers
    if (!interface->write_zone || !interface->read_zone || !interface->erase_zone) {
        LOG_ERROR("Storage interface missing required functions");
        return ZONE_ERROR_INVALID_PARAM;
    }

    g_storage_interface = interface;
    LOG_INFO("Storage interface registered successfully");

    return ZONE_ERROR_NONE;
}

/* ========================================================================== */
/* Factory Reset and Defaults */
/* ========================================================================== */

zone_error_t Zones_FactoryReset(void)
{
    if (!g_storage_interface || !g_storage_interface->format_storage) {
        LOG_ERROR("No storage interface for factory reset");
        return ZONE_ERROR_MEMORY;
    }

    LOG_INFO("Performing factory reset of all zones");

    zone_error_t result = g_storage_interface->format_storage();
    if (result != ZONE_ERROR_NONE) {
        LOG_ERROR("Factory reset failed: %d", result);
        return result;
    }

    // Clear all slot occupied flags
    memset(g_zone_ctx.manager.slot_occupied, 0, sizeof(g_zone_ctx.manager.slot_occupied));

    // Create defaults
    result = Zones_CreateDefaults();
    if (result != ZONE_ERROR_NONE) {
        LOG_ERROR("Failed to create default zones: %d", result);
        return result;
    }

    LOG_INFO("Factory reset completed successfully");
    return ZONE_ERROR_NONE;
}

zone_error_t Zones_CreateDefaults(void)
{
    LOG_INFO("Creating default zone configurations");

    // Create a default zone in slot 0 - main typing area
    zone_storage_t default_zone;
    zone_error_t result = CreateDefaultZone(&default_zone, 0);
    if (result != ZONE_ERROR_NONE) {
        return result;
    }

    // Set warm amber color for main typing area
    Backlight_RGB_t warm_amber = {80, 40, 10};  // Warm amber/orange

    // Add main typing keys to default zone using the LED mapping
    uint8_t key_count = 0;

    // Add number row (1-0, -, =)
    for (uint8_t col = 1; col <= 12; col++) {
        if (get_led_for_key(1, col) != NULL && key_count < ZONE_MAX_KEYS) {
            default_zone.keys[key_count].row = 1;
            default_zone.keys[key_count].col = col;
            default_zone.keys[key_count].color = warm_amber;
            default_zone.keys[key_count].active = 1;
            key_count++;
        }
    }

    // Add QWERTY row (Q-P, [, ])
    for (uint8_t col = 1; col <= 12; col++) {
        if (get_led_for_key(2, col) != NULL && key_count < ZONE_MAX_KEYS) {
            default_zone.keys[key_count].row = 2;
            default_zone.keys[key_count].col = col;
            default_zone.keys[key_count].color = warm_amber;
            default_zone.keys[key_count].active = 1;
            key_count++;
        }
    }

    // Add ASDF row (A-L, ;, ')
    for (uint8_t col = 1; col <= 11; col++) {
        if (get_led_for_key(3, col) != NULL && key_count < ZONE_MAX_KEYS) {
            default_zone.keys[key_count].row = 3;
            default_zone.keys[key_count].col = col;
            default_zone.keys[key_count].color = warm_amber;
            default_zone.keys[key_count].active = 1;
            key_count++;
        }
    }

    // Add ZXCV row (Z-M, ,, .)
    for (uint8_t col = 2; col <= 10; col++) {  // Skip left shift
        if (get_led_for_key(4, col) != NULL && key_count < ZONE_MAX_KEYS) {
            default_zone.keys[key_count].row = 4;
            default_zone.keys[key_count].col = col;
            default_zone.keys[key_count].color = warm_amber;
            default_zone.keys[key_count].active = 1;
            key_count++;
        }
    }

    default_zone.metadata.key_count = key_count;
    strcpy((char*)default_zone.metadata.name, "Main Typing");

    // Save default zone
    if (g_storage_interface && g_storage_interface->write_zone) {
        result = g_storage_interface->write_zone(0, &default_zone);
        if (result == ZONE_ERROR_NONE) {
            g_zone_ctx.manager.slot_occupied[0] = true;
            LOG_INFO("Default main typing zone created in slot 0 with %d keys", key_count);
        }
    }

    // Create a WASD gaming zone in slot 1
    zone_storage_t gaming_zone;
    result = CreateDefaultZone(&gaming_zone, 1);
    if (result == ZONE_ERROR_NONE) {
        Backlight_RGB_t gaming_red = {120, 0, 0};  // Bright red for gaming

        // Add WASD keys
        const uint8_t wasd_keys[4][2] = {{2, 2}, {3, 1}, {3, 2}, {3, 3}};  // W, A, S, D
        key_count = 0;

        for (uint8_t i = 0; i < 4; i++) {
            uint8_t row = wasd_keys[i][0];
            uint8_t col = wasd_keys[i][1];
            if (get_led_for_key(row, col) != NULL) {
                gaming_zone.keys[key_count].row = row;
                gaming_zone.keys[key_count].col = col;
                gaming_zone.keys[key_count].color = gaming_red;
                gaming_zone.keys[key_count].active = 1;
                key_count++;
            }
        }

        gaming_zone.metadata.key_count = key_count;
        strcpy((char*)gaming_zone.metadata.name, "WASD Gaming");

        if (g_storage_interface && g_storage_interface->write_zone) {
            result = g_storage_interface->write_zone(1, &gaming_zone);
            if (result == ZONE_ERROR_NONE) {
                g_zone_ctx.manager.slot_occupied[1] = true;
                LOG_INFO("Default WASD gaming zone created in slot 1 with %d keys", key_count);
            }
        }
    }

    return ZONE_ERROR_NONE;
}

/* ========================================================================== */
/* Private Functions Implementation */
/* ========================================================================== */

static zone_error_t ProcessStateMachine(void)
{
    switch (g_zone_ctx.current_state) {
        case ZONE_STATE_INACTIVE:
            // Nothing to process
            return ZONE_ERROR_NONE;

        case ZONE_STATE_BROWSE:
            return ProcessBrowseState();

        case ZONE_STATE_EDIT:
            return ProcessEditState();

        case ZONE_STATE_HSV_ADJUST:
            return ProcessHSVState();

        case ZONE_STATE_SAVE_CONFIRM:
            return ProcessSaveConfirmState();

        case ZONE_STATE_ERROR:
            // Handle error state recovery
            LOG_WARNING("Recovering from error state");
            g_zone_ctx.current_state = ZONE_STATE_INACTIVE;
            return ZONE_ERROR_NONE;

        default:
            LOG_ERROR("Unknown zone state: %d", g_zone_ctx.current_state);
            g_zone_ctx.current_state = ZONE_STATE_ERROR;
            return ZONE_ERROR_INVALID_STATE;
    }
}

static zone_error_t ProcessBrowseState(void)
{
    // Handle preview timeout
    if (g_zone_ctx.preview_active) {
        uint32_t preview_elapsed = HAL_GetTick() - g_zone_ctx.preview_start_time;
        if (preview_elapsed >= g_zone_ctx.preview_duration_ms) {
            g_zone_ctx.preview_active = false;
            // Restore browse interface
            return ShowSlotIndicators();
        }
    }

    return ZONE_ERROR_NONE;
}

static zone_error_t ProcessEditState(void)
{
    // Periodically refresh key selection display
    uint32_t state_elapsed = HAL_GetTick() - g_zone_ctx.state_start_time;
    if (state_elapsed % 2000 == 0) {  // Every 2 seconds
        return ShowKeySelection();
    }

    return ZONE_ERROR_NONE;
}

static zone_error_t ProcessHSVState(void)
{
    // Update HSV preview continuously
    return ShowHSVPreview();
}

static zone_error_t ProcessSaveConfirmState(void)
{
    // Handle save confirmation timeout
    uint32_t state_elapsed = HAL_GetTick() - g_zone_ctx.state_start_time;
    if (state_elapsed >= 3000) {  // 3 second timeout
        g_zone_ctx.current_state = ZONE_STATE_EDIT;
    }

    return ZONE_ERROR_NONE;
}

static zone_error_t InitializeZoneSlots(void)
{
    // Initialize slot occupied flags
    memset(g_zone_ctx.manager.slot_occupied, 0, sizeof(g_zone_ctx.manager.slot_occupied));

    if (!g_storage_interface || !g_storage_interface->is_slot_valid) {
        LOG_WARNING("No storage interface - slots marked as empty");
        return ZONE_ERROR_NONE;
    }

    // Check which slots are occupied
    uint8_t occupied_count = 0;
    for (uint8_t i = 0; i < ZONE_MAX_SLOTS; i++) {
        bool slot_valid = g_storage_interface->is_slot_valid(i);
        g_zone_ctx.manager.slot_occupied[i] = slot_valid;
        if (slot_valid) {
            occupied_count++;
        }
    }

    LOG_INFO("Zone slots initialized: %d/%d occupied", occupied_count, ZONE_MAX_SLOTS);
    return ZONE_ERROR_NONE;
}

static zone_error_t ValidateZoneIntegrity(const zone_storage_t *zone)
{
    if (!zone) {
        return ZONE_ERROR_INVALID_PARAM;
    }

    // Check version
    if (zone->metadata.version != ZONE_VERSION_CURRENT) {
        LOG_ERROR("Invalid zone version: %d (expected %d)",
                  zone->metadata.version, ZONE_VERSION_CURRENT);
        return ZONE_ERROR_CRC_MISMATCH;
    }

    // Check key count
    if (zone->metadata.key_count > ZONE_MAX_KEYS) {
        LOG_ERROR("Invalid key count: %d (max %d)",
                  zone->metadata.key_count, ZONE_MAX_KEYS);
        return ZONE_ERROR_CRC_MISMATCH;
    }

    // Validate key positions
    for (uint8_t i = 0; i < zone->metadata.key_count; i++) {
        const zone_key_data_t *key = &zone->keys[i];
        if (!IsValidKey(key->row, key->col)) {
            LOG_ERROR("Invalid key position in zone: (%d,%d)", key->row, key->col);
            return ZONE_ERROR_CRC_MISMATCH;
        }
    }

    // Validate CRC (skip for now - will be implemented with hardware CRC)
    uint16_t calculated_crc = CalculateZoneCRC16(zone);
    if (calculated_crc != zone->metadata.crc16) {
        LOG_ERROR("CRC mismatch: expected 0x%04X, got 0x%04X",
                  zone->metadata.crc16, calculated_crc);
        return ZONE_ERROR_CRC_MISMATCH;
    }

    return ZONE_ERROR_NONE;
}

static uint16_t CalculateZoneCRC16(const zone_storage_t *zone)
{
    if (!zone) {
        return 0;
    }

    // Simple CRC-16 calculation for now
    // TODO: Replace with STM32L072 hardware CRC peripheral
    uint16_t crc = 0xFFFF;
    const uint8_t *data = (const uint8_t*)zone;

    // Calculate CRC for all data except the CRC field itself
    size_t crc_offset = offsetof(zone_metadata_t, crc16);
    size_t data_len = sizeof(zone_storage_t);

    // Process data before CRC field
    for (size_t i = 0; i < crc_offset; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ ZONE_CRC_POLYNOMIAL;
            } else {
                crc >>= 1;
            }
        }
    }

    // Skip CRC field and process remaining data
    for (size_t i = crc_offset + sizeof(uint16_t); i < data_len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ ZONE_CRC_POLYNOMIAL;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

static zone_error_t CreateDefaultZone(zone_storage_t *zone, uint8_t slot_id)
{
    if (!zone) {
        return ZONE_ERROR_INVALID_PARAM;
    }

    memset(zone, 0, sizeof(zone_storage_t));

    // Initialize metadata
    zone->metadata.version = ZONE_VERSION_CURRENT;
    zone->metadata.slot_id = slot_id;
    zone->metadata.key_count = 0;
    zone->metadata.brightness_level = g_zone_ctx.default_brightness;
    zone->metadata.created_timestamp = HAL_GetTick();
    zone->metadata.modified_timestamp = HAL_GetTick();

    snprintf((char*)zone->metadata.name, sizeof(zone->metadata.name), "Zone %d", slot_id + 1);

    // CRC will be calculated when saving

    return ZONE_ERROR_NONE;
}

static zone_error_t ShowSlotIndicators(void)
{
    // Clear all LEDs first
    Backlight_SetAll(0, 0, 0);

    // Show slot indicators on number row keys 1-8
    for (uint8_t i = 0; i < ZONE_MAX_SLOTS; i++) {
        uint8_t row = ZONE_SLOT_ROW;
        uint8_t col = ZONE_SLOT_START_COL + i;

        // Check if this key position has an LED using the mapping
        if (get_led_for_key(row, col) != NULL) {
            if (g_zone_ctx.manager.slot_occupied[i]) {
                Backlight_SetKeyRGB(row, col, ZONE_COLOR_SET);     // Green for occupied
            } else {
                Backlight_SetKeyRGB(row, col, ZONE_COLOR_UNSET);   // Red for empty
            }
        }
    }

    return ZONE_ERROR_NONE;
}

static zone_error_t ShowKeySelection(void)
{
    // Clear all LEDs
    Backlight_SetAll(0, 0, 0);

    // Show current zone keys with dimmed colors
    for (uint8_t i = 0; i < g_zone_ctx.manager.staging_zone.metadata.key_count; i++) {
        zone_key_data_t *key = &g_zone_ctx.manager.staging_zone.keys[i];
        if (key->active && IsValidKey(key->row, key->col)) {
            // Show zone key with dimmed color (1/4 brightness)
            Backlight_RGB_t dim_color = {
                key->color.r / 4,
                key->color.g / 4,
                key->color.b / 4
            };
            Backlight_SetKeyRGB(key->row, key->col, dim_color);
        }
    }

    // Highlight selected keys with cyan
    for (uint8_t row = 0; row < BACKLIGHT_MATRIX_ROWS; row++) {
        for (uint8_t col = 0; col < BACKLIGHT_MATRIX_COLS; col++) {
            if (g_zone_ctx.selection.selected[row][col] && get_led_for_key(row, col) != NULL) {
                Backlight_SetKeyRGB(row, col, ZONE_COLOR_SELECTED);  // Cyan for selected
            }
        }
    }

    return ZONE_ERROR_NONE;
}

static zone_error_t ShowHSVPreview(void)
{
    // Convert current HSV to RGB
    Backlight_RGB_t preview_color = Backlight_HSVtoRGB(g_zone_ctx.hsv.hue,
                                                       g_zone_ctx.hsv.saturation,
                                                       g_zone_ctx.hsv.value);

    // Apply preview color to selected keys
    for (uint8_t row = 0; row < BACKLIGHT_MATRIX_ROWS; row++) {
        for (uint8_t col = 0; col < BACKLIGHT_MATRIX_COLS; col++) {
            if (g_zone_ctx.selection.selected[row][col] && get_led_for_key(row, col) != NULL) {
                Backlight_SetKeyRGB(row, col, preview_color);
            }
        }
    }

    return ZONE_ERROR_NONE;
}

static zone_error_t FlashConfirmation(bool success)
{
    Backlight_RGB_t flash_color = success ? ZONE_COLOR_SET : ZONE_COLOR_UNSET;
    uint8_t flash_count = success ? 1 : 3;

    for (uint8_t i = 0; i < flash_count; i++) {
        // Flash on
        Backlight_SetAllRGB(flash_color);
        HAL_Delay(FLASH_PULSE_DURATION_MS);

        // Flash off
        Backlight_SetAll(0, 0, 0);
        if (i < flash_count - 1) {
            HAL_Delay(FLASH_PULSE_DURATION_MS);
        }
    }

    return ZONE_ERROR_NONE;
}

static zone_error_t SaveEffectsState(void)
{
    g_effects_were_active = Effects_IsActive();
    g_previous_effect_type = Effects_GetCurrentType();

    LOG_DEBUG("Saved effects state: active=%d, type=%d",
              g_effects_were_active, g_previous_effect_type);

    return ZONE_ERROR_NONE;
}

static zone_error_t RestoreEffectsState(void)
{
    if (g_effects_were_active) {
        LOG_DEBUG("Restoring effects: type=%d", g_previous_effect_type);
        Effects_StartRuntimeEffect();
    }

    return ZONE_ERROR_NONE;
}

static zone_error_t SuspendEffects(void)
{
    if (Effects_IsActive()) {
        Effects_Stop();
        LOG_DEBUG("Effects suspended for zone programming");
    }

    return ZONE_ERROR_NONE;
}

static zone_error_t ResumeEffects(void)
{
    return RestoreEffectsState();
}

static bool IsValidSlot(uint8_t slot_id)
{
    return slot_id < ZONE_MAX_SLOTS;
}

static bool IsValidKey(uint8_t row, uint8_t col)
{
    return (row < BACKLIGHT_MATRIX_ROWS) &&
           (col < BACKLIGHT_MATRIX_COLS) &&
           (get_led_for_key(row, col) != NULL);
}

static bool HasUserActivity(void)
{
    // This would be called by keyboard interrupt handlers
    // For now, just check if we're actively processing
    return (g_zone_ctx.current_state != ZONE_STATE_INACTIVE);
}

static void UpdateActivityTime(void)
{
    g_zone_ctx.last_activity_time = HAL_GetTick();
}

static zone_error_t HandleTimeout(void)
{
    if (!g_zone_ctx.auto_exit_enabled || g_zone_ctx.current_state == ZONE_STATE_INACTIVE) {
        return ZONE_ERROR_NONE;
    }

    uint32_t elapsed = HAL_GetTick() - g_zone_ctx.last_activity_time;
    if (elapsed >= ZONE_PROGRAMMING_TIMEOUT_MS) {
        LOG_INFO("Zone programming timeout - auto-exiting without saving");
        return Zones_ExitProgramming(false);  // Exit without saving
    }

    return ZONE_ERROR_NONE;
}

static zone_error_t AddKeyToZone(zone_storage_t *zone, uint8_t row, uint8_t col, Backlight_RGB_t color)
{
    if (!zone || !IsValidKey(row, col)) {
        return ZONE_ERROR_INVALID_PARAM;
    }

    // Look for existing key first
    zone_key_data_t *existing_key = FindKeyInZone(zone, row, col);
    if (existing_key) {
        // Update existing key
        existing_key->color = color;
        existing_key->active = 1;
        return ZONE_ERROR_NONE;
    }

    // Add new key if space available
    if (zone->metadata.key_count >= ZONE_MAX_KEYS) {
        LOG_ERROR("Zone full - cannot add key (%d,%d)", row, col);
        return ZONE_ERROR_STORAGE_FULL;
    }

    zone_key_data_t *new_key = &zone->keys[zone->metadata.key_count];
    new_key->row = row;
    new_key->col = col;
    new_key->color = color;
    new_key->active = 1;
    zone->metadata.key_count++;

    LOG_DEBUG("Added key (%d,%d) to zone (now %d keys)", row, col, zone->metadata.key_count);
    return ZONE_ERROR_NONE;
}

static zone_error_t RemoveKeyFromZone(zone_storage_t *zone, uint8_t row, uint8_t col)
{
    if (!zone) {
        return ZONE_ERROR_INVALID_PARAM;
    }

    // Find key to remove
    for (uint8_t i = 0; i < zone->metadata.key_count; i++) {
        zone_key_data_t *key = &zone->keys[i];
        if (key->row == row && key->col == col) {
            // Move last key to this position to avoid gaps
            if (i < zone->metadata.key_count - 1) {
                zone->keys[i] = zone->keys[zone->metadata.key_count - 1];
            }
            zone->metadata.key_count--;
            LOG_DEBUG("Removed key (%d,%d) from zone (now %d keys)", row, col, zone->metadata.key_count);
            return ZONE_ERROR_NONE;
        }
    }

    return ZONE_ERROR_INVALID_PARAM;  // Key not found
}

static zone_key_data_t* FindKeyInZone(zone_storage_t *zone, uint8_t row, uint8_t col)
{
    if (!zone) {
        return NULL;
    }

    for (uint8_t i = 0; i < zone->metadata.key_count; i++) {
        zone_key_data_t *key = &zone->keys[i];
        if (key->row == row && key->col == col) {
            return key;
        }
    }

    return NULL;  // Key not found
}
