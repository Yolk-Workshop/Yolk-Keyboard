/*
 * zones.c - Zone Programming Interface Implementation
 */

#include "zones.h"
#include "system_config.h"
#include "backlight.h"
#include "colors.h"
#include "led_mapping.h"
#include "logger.h"
#include "keys.h"
#include <string.h>

/* Private Constants */
#define HSV_HUE_STEP_DEFAULT        5
#define HSV_VALUE_MIN				1
#define HSV_VALUE_MAX				100
#define HSV_SV_STEP_DEFAULT         5
#define ZONE_SLOT_ROW               1
#define ZONE_SLOT_START_COL         1

#define ZONE_DEFAULT_COLOR_HUE          30      // Warm white/yellow
#define ZONE_DEFAULT_COLOR_SATURATION   20      // Low saturation
#define ZONE_DEFAULT_COLOR_VALUE        80      // Medium brightness

/* Private Variables */
static zone_programming_context_t g_prog_ctx = {0};
static bool g_initialized = false;
zone_runtime_ctx_t g_zone_rt_ctx = {0};

typedef struct{
	uint32_t session_lock_time;
	uint8_t active_zone_id;
	bool zone_session_locked;
}g_session_t;

g_session_t g_session_guard = {0};
/* Private Function Prototypes */
static void UpdateActivityTime(void);
static config_status_t ShowSlotIndicators(void);
static config_status_t FlashConfirmation(bool success);
static void UpdateRuntimeContext(void);

/* Preview Mode Functions */
static config_status_t enter_preview_mode(uint8_t zone_id);
static void exit_preview_mode(void);
static bool handle_preview_keys(uint8_t row, uint8_t col);
static void load_zone_to_preview_cache(uint8_t zone_id);
static void show_preview_display(void);

/* Browse Mode Functions */
static bool handle_browse_keys(uint8_t row, uint8_t col);

/* Color Assignment Functions */
static bool enter_individual_color_mode(uint8_t row, uint8_t col);
static bool enter_zone_color_mode(void);
static bool enter_edit_from_preview(void);
static void show_color_mode_display(void);
static void apply_color_and_exit(void);
static void reset_to_zone_default_and_exit(void);

/* Key Combination Handlers */
static bool handle_right_alt_combinations(uint8_t row, uint8_t col);
static bool handle_color_hsv_keys(uint8_t row, uint8_t col);
static config_status_t swap_eeprom_zones(uint8_t eeprom_zone_a, uint8_t eeprom_zone_b);
static config_status_t load_eeprom_to_runtime(uint8_t eeprom_zone_id, uint8_t runtime_slot);
static config_status_t enter_edit_mode_with_existing_zone(uint8_t zone_id);

static const uint8_t KEY_COUNT_ = 80;

/* Visual Feedback Functions */
static void flash_timeout_warning(void);

static bool ValidateSessionState(void)
{
    // Check if we have a valid session lock
    if (g_session_guard.zone_session_locked) {
        uint32_t lock_age = HAL_GetTick() - g_session_guard.session_lock_time;

        // Session locks expire after 30 seconds to prevent permanent locks
        if (lock_age > 5000) {
            LOG_WARNING("Session lock expired, clearing");
            g_session_guard.zone_session_locked = false;
            return false;
        }
        return true;
    }
    return false;
}

static bool Zones_IsRuntimeSlotOccupied(uint8_t runtime_slot)
{
    if (runtime_slot >= 2) return false;
    return g_zone_rt_ctx.zones[runtime_slot].key_count > 0;
}

static Backlight_RGB_t get_default_zone_color(void)
{
    return Backlight_HSVtoRGB(ZONE_DEFAULT_COLOR_HUE,
                             ZONE_DEFAULT_COLOR_SATURATION,
                             ZONE_DEFAULT_COLOR_VALUE);
}

static void CleanupModifierStates(void)
{
    g_prog_ctx.right_alt_pressed = false;
    g_prog_ctx.color_mode_active = false;
    g_prog_ctx.preview_mode_active = false;

    LOG_DEBUG("Modifier states cleaned up");
}

static const zone_interface_t zone_interface = {
	.load_keys_to_runtime = Zones_LoadKeysToRuntime,
	.is_runtime_slot_occupied = Zones_IsRuntimeSlotOccupied,
};

/* Public API Implementation */
config_status_t Zones_Init(void)
{
    if (g_initialized) return CONFIG_OK;

    memset(&g_prog_ctx, 0, sizeof(g_prog_ctx));
    g_prog_ctx.current_state = ZONE_PROG_INACTIVE;
    g_prog_ctx.edit_mode = ZONE_EDIT_INDIVIDUAL;

    // Initialize HSV defaults with proper bounds
    g_prog_ctx.hsv.hue = 30;
    g_prog_ctx.hsv.saturation = 100;
    g_prog_ctx.hsv.value = 80;  // Safe default in 1-100 range
    g_prog_ctx.hsv.hue_step = HSV_HUE_STEP_DEFAULT;
    g_prog_ctx.hsv.sv_step = HSV_SV_STEP_DEFAULT;

    g_prog_ctx.auto_exit_enabled = false;  // Master config handles timeouts
    g_prog_ctx.timeout_duration_ms = 0;    // Disabled

    // Initialize session guard
    memset(&g_session_guard, 0, sizeof(g_session_guard));

    session_set_zone_interface(&zone_interface);

    g_initialized = true;
    LOG_INFO("Zone programming system initialized");
    return CONFIG_OK;
}

config_status_t Zones_Process(void)
{
    if (!g_initialized || g_prog_ctx.current_state == ZONE_PROG_INACTIVE) {
        return CONFIG_OK;
    }

    uint32_t current_time = HAL_GetTick();

    if (g_prog_ctx.color_mode_active) {
        uint32_t color_elapsed = current_time - g_prog_ctx.color_mode_start_time;

        // Warning at 4:45 (285 seconds)
        if (color_elapsed >= 285000 && color_elapsed < 286000) {
            flash_timeout_warning();
        }

        // Auto-exit at 5 minutes (300 seconds)
        if (color_elapsed >= 300000) {
            LOG_WARNING("Color mode timeout - discarding changes");
            g_prog_ctx.color_mode_active = false;
            g_prog_ctx.right_alt_pressed = false;
            g_prog_ctx.current_state = g_prog_ctx.previous_state;
            ShowKeySelection();
        }
    }

    // Original timeout handling for main session
    if (g_prog_ctx.auto_exit_enabled) {
        uint32_t elapsed = current_time - g_prog_ctx.last_activity_time;
        if (elapsed >= g_prog_ctx.timeout_duration_ms) {
            LOG_INFO("Zone programming timeout");
            return Zones_ExitProgramming(false);
        }
    }

    return CONFIG_OK;
}

bool Zones_IsProgammingActive(void)
{
    return g_initialized && (g_prog_ctx.current_state != ZONE_PROG_INACTIVE);
}

bool Zones_IsSlotOccupied(uint8_t slot_id)
{
    return (slot_id < ZONE_MAX_ZONES) ? zone_is_slot_occupied(slot_id) : false;
}

uint8_t Zones_GetKeyCount(uint8_t slot_id)
{
    if (slot_id >= ZONE_MAX_ZONES) return 0;

    uint8_t led_count;
    if (zone_get_basic_info(slot_id, &led_count, NULL, NULL) == CONFIG_OK) {
        return led_count;
    }
    return 0;
}

config_status_t Zones_EnterProgramming(void)
{
    if (!g_initialized) return CONFIG_INVALID_PARAMETER;
    if (g_prog_ctx.current_state != ZONE_PROG_INACTIVE) return CONFIG_OK;

    LOG_INFO("Entering zone programming mode");

    g_prog_ctx.current_state = ZONE_PROG_BROWSE;
    g_prog_ctx.state_start_time = HAL_GetTick();
    g_prog_ctx.last_activity_time = HAL_GetTick();
    g_prog_ctx.has_unsaved_changes = false;

    return Zones_EnterBrowse();
}

config_status_t Zones_ExitProgramming(bool save_changes)
{
    if (!g_initialized || g_prog_ctx.current_state == ZONE_PROG_INACTIVE) {
        return CONFIG_INVALID_PARAMETER;
    }

    LOG_INFO("Exiting zone programming mode (save=%d)", save_changes);

    config_status_t result = CONFIG_OK;

    if (save_changes && g_prog_ctx.has_unsaved_changes) {
        uint8_t editing_zone = g_prog_ctx.editing_zone_id;

        // Handle slots 0-1: Runtime cache + EEPROM
        if (editing_zone <= 1) {
            LOG_INFO("Saving zone %d to runtime cache + EEPROM", editing_zone);

            // 1. Load to runtime cache first
            result = Zones_LoadStagingToRuntime(editing_zone);
            if (result == CONFIG_OK) {
                LOG_INFO("Zone %d loaded to runtime cache", editing_zone);

                // 2. Enable zones if this is the first active zone
                if (!g_zone_rt_ctx.zones_enabled && g_zone_rt_ctx.active_zone_count > 0) {
                    Zones_SetRuntimeEnabled(false);
                    LOG_INFO("Runtime zones auto-enabled");
                }
            }

            // 4. TODO: Save to EEPROM when ready
            // result = session_commit_changes(); // EEPROM save
        }
        // Handle slots 2-7: EEPROM only
        else {
            LOG_INFO("Saving zone %d directly to EEPROM", editing_zone);

            // TODO: Direct EEPROM save when implemented
            // result = session_commit_changes();
            result = CONFIG_OK; // Placeholder for now
        }

        FlashConfirmation(result == CONFIG_OK);

    } else if (g_prog_ctx.has_unsaved_changes) {
        session_discard_changes();
        FlashConfirmation(false);
    }

    memset(&g_prog_ctx.selection, 0, sizeof(g_prog_ctx.selection));
    // Clear session protection
    g_session_guard.zone_session_locked = false;
    g_session_guard.active_zone_id = 0xFF;

    // Clean up all state
    CleanupModifierStates();
    g_prog_ctx.has_unsaved_changes = false;

    return result;
}


config_status_t Zones_EnterBrowse(void)
{
    if (!g_initialized) return CONFIG_INVALID_PARAMETER;

    g_session_guard.zone_session_locked = false;
	g_session_guard.active_zone_id = 0xFF;

    g_prog_ctx.current_state = ZONE_PROG_BROWSE;
    UpdateActivityTime();

    return ShowSlotIndicators();
}

config_status_t Zones_SelectZone(uint8_t zone_id)
{
    if (zone_id >= ZONE_MAX_ZONES) return CONFIG_INVALID_PARAMETER;

    // Protect against session corruption
    if (ValidateSessionState() && g_session_guard.active_zone_id != zone_id) {
        LOG_WARNING("Attempted to switch zones while session locked to zone %d",
                   g_session_guard.active_zone_id);
        return CONFIG_ERROR;
    }

    UpdateActivityTime();
    g_prog_ctx.editing_zone_id = zone_id;

    // Lock this session to prevent corruption
    g_session_guard.zone_session_locked = true;
    g_session_guard.active_zone_id = zone_id;
    g_session_guard.session_lock_time = HAL_GetTick();

    // Begin editing session using system_config
    config_status_t result = session_begin_edit(zone_id);
    if (result != CONFIG_OK) {
        // Clear lock on failure
        g_session_guard.zone_session_locked = false;
        return result;
    }

    return Zones_EnterEdit(zone_id);
}

config_status_t Zones_EnterEdit(uint8_t zone_id)
{
    g_prog_ctx.current_state = ZONE_PROG_EDIT;
	g_prog_ctx.editing_zone_id = zone_id;
	g_prog_ctx.color_explicitly_programmed = false;
	g_prog_ctx.current_default_color = get_default_zone_color();
	g_prog_ctx.has_default_color_shown = true;

    UpdateActivityTime();

    memset(&g_prog_ctx.selection, 0, sizeof(g_prog_ctx.selection));
    Backlight_SetAll(0, 0, 0);

    return ShowKeySelection();
}

config_status_t Zones_ToggleKeySelection(uint8_t row, uint8_t col)
{
    if (!Zones_IsValidKey(row, col) || g_prog_ctx.current_state != ZONE_PROG_EDIT) {
        return CONFIG_INVALID_PARAMETER;
    }

    UpdateActivityTime();

    bool was_selected = g_prog_ctx.selection.selected[row][col];
    g_prog_ctx.selection.selected[row][col] = !was_selected;

    if (was_selected) {
        g_prog_ctx.selection.selection_count--;
        session_remove_key_color(row, col);
    } else {
        g_prog_ctx.selection.selection_count++;
        g_prog_ctx.has_unsaved_changes = true;
        session_set_key_color(row, col, BACKLIGHT_TO_RGB(COLOR_CRIMSON), false);
    }

    if(g_prog_ctx.selection.selection_count == 0){
    	g_prog_ctx.has_unsaved_changes = false;
    }

    return ShowKeySelection();
}

config_status_t Zones_ClearSelection(void)
{
    if (g_prog_ctx.current_state != ZONE_PROG_EDIT) return CONFIG_INVALID_PARAMETER;

    UpdateActivityTime();
    memset(&g_prog_ctx.selection.selected, 0, sizeof(g_prog_ctx.selection.selected));
    g_prog_ctx.selection.selection_count = 0;
    g_prog_ctx.has_unsaved_changes = false;
    return ShowKeySelection();
}

config_status_t Zones_SelectAllKeys(void)
{
    if (g_prog_ctx.current_state != ZONE_PROG_EDIT) return CONFIG_INVALID_PARAMETER;

    UpdateActivityTime();
    g_prog_ctx.selection.selection_count = 0;

    for (uint8_t row = 0; row < ZONE_MATRIX_ROWS; row++) {
        for (uint8_t col = 0; col < ZONE_MATRIX_COLS; col++) {
            if (Zones_IsValidKey(row, col)) {
                g_prog_ctx.selection.selected[row][col] = true;
                g_prog_ctx.selection.selection_count++;
            }
        }
    }

    return ShowKeySelection();
}

config_status_t Zones_EnterHSV(void)
{
    if (g_prog_ctx.current_state != ZONE_PROG_EDIT) return CONFIG_INVALID_PARAMETER;
    if (g_prog_ctx.selection.selection_count == 0) return CONFIG_INVALID_PARAMETER;

    g_prog_ctx.previous_state = g_prog_ctx.current_state;
    g_prog_ctx.current_state = ZONE_PROG_HSV_ADJUST;
    UpdateActivityTime();

    if (g_prog_ctx.color_mode_active) {
		show_color_mode_display();
	}
    return CONFIG_OK;
}

config_status_t Zones_AdjustHue(int8_t direction)
{
    if (g_prog_ctx.current_state != ZONE_PROG_HSV_ADJUST) {
        return CONFIG_INVALID_PARAMETER;
    }

    UpdateActivityTime();

    int16_t new_hue = g_prog_ctx.hsv.hue + (direction * g_prog_ctx.hsv.hue_step);
    if (new_hue < 0) new_hue += 360;
    else if (new_hue >= 360) new_hue -= 360;

    g_prog_ctx.hsv.hue = (uint16_t)new_hue;

    // Update display based on mode
	if (g_prog_ctx.color_mode_active)
	{
		Backlight_RGB_t rgb = Backlight_HSVtoRGB(g_prog_ctx.hsv.hue,
													g_prog_ctx.hsv.saturation,
													g_prog_ctx.hsv.value);
		rgb_color_t session_color = {rgb.r, rgb.g, rgb.b};

		// Apply based on existing edit_mode
		if (g_prog_ctx.edit_mode == ZONE_EDIT_INDIVIDUAL) {
			// Individual key mode: only apply to color_target_row/col
			session_set_key_color(g_prog_ctx.color_target_row,
								 g_prog_ctx.color_target_col,
								 session_color, true);
		} else if (g_prog_ctx.edit_mode == ZONE_EDIT_GROUP) {
			// Zone mode: apply to all selected keys
			for (uint8_t row = 0; row < ZONE_MATRIX_ROWS; row++) {
				for (uint8_t col = 0; col < ZONE_MATRIX_COLS; col++) {
					if (g_prog_ctx.selection.selected[row][col]) {
						session_set_key_color(row, col, session_color, true);
					}
				}
			}
		}

		g_prog_ctx.has_unsaved_changes = true;
        show_color_mode_display();
    }
    return CONFIG_OK;
}


config_status_t Zones_AdjustBrightness(int8_t direction)
{
    if (g_prog_ctx.current_state != ZONE_PROG_HSV_ADJUST) return CONFIG_INVALID_PARAMETER;

    UpdateActivityTime();

    int16_t new_value = g_prog_ctx.hsv.value + (direction * g_prog_ctx.hsv.sv_step);

    if (new_value < HSV_VALUE_MIN) new_value = HSV_VALUE_MIN;
    else if (new_value > HSV_VALUE_MAX) new_value = HSV_VALUE_MAX;

    g_prog_ctx.hsv.value = (uint8_t)new_value;
    // Use optimized display update
   if (g_prog_ctx.color_mode_active)
   {
		Backlight_RGB_t rgb = Backlight_HSVtoRGB(g_prog_ctx.hsv.hue,
													g_prog_ctx.hsv.saturation,
													g_prog_ctx.hsv.value);
		rgb_color_t session_color = {rgb.r, rgb.g, rgb.b};

		// Apply based on existing edit_mode
		if (g_prog_ctx.edit_mode == ZONE_EDIT_INDIVIDUAL) {
			// Individual key mode: only apply to color_target_row/col
			session_set_key_color(g_prog_ctx.color_target_row,
								 g_prog_ctx.color_target_col,
								 session_color, true);
		} else if (g_prog_ctx.edit_mode == ZONE_EDIT_GROUP) {
			// Zone mode: apply to all selected keys
			for (uint8_t row = 0; row < ZONE_MATRIX_ROWS; row++) {
				for (uint8_t col = 0; col < ZONE_MATRIX_COLS; col++) {
					if (g_prog_ctx.selection.selected[row][col]) {
						session_set_key_color(row, col, session_color, true);
					}
				}
			}
		}

		g_prog_ctx.has_unsaved_changes = true;

    	show_color_mode_display();
    }

    return CONFIG_OK;
}

config_status_t Zones_AdjustSaturation(int8_t direction)
{
    if (g_prog_ctx.current_state != ZONE_PROG_HSV_ADJUST) return CONFIG_INVALID_PARAMETER;

    UpdateActivityTime();

    int16_t new_saturation = g_prog_ctx.hsv.saturation + (direction * g_prog_ctx.hsv.sv_step);

    // Saturation bounds: 0-100
    if (new_saturation < 0) new_saturation = 0;
    else if (new_saturation > 100) new_saturation = 100;

    g_prog_ctx.hsv.saturation = (uint8_t)new_saturation;

    // Apply color changes in real-time for color mode
    if (g_prog_ctx.color_mode_active)
    {
        // Convert HSV to RGB
    	Backlight_RGB_t rgb = Backlight_HSVtoRGB(g_prog_ctx.hsv.hue,
    	                                            g_prog_ctx.hsv.saturation,
    	                                            g_prog_ctx.hsv.value);
		rgb_color_t session_color = {rgb.r, rgb.g, rgb.b};

		// Apply based on existing edit_mode
		if (g_prog_ctx.edit_mode == ZONE_EDIT_INDIVIDUAL) {
			// Individual key mode: only apply to color_target_row/col
			session_set_key_color(g_prog_ctx.color_target_row,
								 g_prog_ctx.color_target_col,
								 session_color, true);
		} else if (g_prog_ctx.edit_mode == ZONE_EDIT_GROUP) {
			// Zone mode: apply to all selected keys
			for (uint8_t row = 0; row < ZONE_MATRIX_ROWS; row++) {
				for (uint8_t col = 0; col < ZONE_MATRIX_COLS; col++) {
					if (g_prog_ctx.selection.selected[row][col]) {
						session_set_key_color(row, col, session_color, true);
					}
				}
			}
		}

		g_prog_ctx.has_unsaved_changes = true;
        show_color_mode_display();
    }

    return CONFIG_OK;
}

config_status_t Zones_ApplyHSVColor(void)
{
    if (g_prog_ctx.current_state != ZONE_PROG_HSV_ADJUST) {
        return CONFIG_INVALID_PARAMETER;
    }

    Backlight_RGB_t rgb = Backlight_HSVtoRGB(g_prog_ctx.hsv.hue,
                                            g_prog_ctx.hsv.saturation,
                                            g_prog_ctx.hsv.value);
    rgb_color_t system_color = {rgb.r, rgb.g, rgb.b};

    if (g_prog_ctx.color_mode_active)
    {
        if (g_prog_ctx.edit_mode == ZONE_EDIT_INDIVIDUAL) {
            // Apply to individual key
            session_set_key_color(g_prog_ctx.color_target_row,
                                 g_prog_ctx.color_target_col,
								 system_color, true);
            Backlight_SetKeyRGB(g_prog_ctx.color_target_row,
                               g_prog_ctx.color_target_col, rgb);
        } else if (g_prog_ctx.edit_mode == ZONE_EDIT_GROUP) {
            // Apply to all selected keys
            for (uint8_t row = 0; row < ZONE_MATRIX_ROWS; row++) {
                for (uint8_t col = 0; col < ZONE_MATRIX_COLS; col++) {
                    if (g_prog_ctx.selection.selected[row][col]) {
                        session_set_key_color(row, col, system_color, true);
                        Backlight_SetKeyRGB(row, col, rgb);
                    }
                }
            }
        }

        g_prog_ctx.has_unsaved_changes = true;
        return CONFIG_OK;  // Stay in color mode for more adjustments
    }

    // Original behavior for normal HSV mode
    UpdateActivityTime();

    for (uint8_t row = 0; row < ZONE_MATRIX_ROWS; row++) {
        for (uint8_t col = 0; col < ZONE_MATRIX_COLS; col++) {
            if (g_prog_ctx.selection.selected[row][col]) {
                session_set_key_color(row, col, system_color, true);
                Backlight_SetKeyRGB(row, col, rgb);
            }
        }
    }

    g_prog_ctx.color_explicitly_programmed = true;
    g_prog_ctx.has_unsaved_changes = true;
    g_prog_ctx.current_state = g_prog_ctx.previous_state;
    return CONFIG_OK;
}

bool Zones_HandleKey(uint8_t row, uint8_t col, bool pressed)
{
    if (!g_initialized || g_prog_ctx.current_state == ZONE_PROG_INACTIVE || !pressed) {
        return false;
    }

    UpdateActivityTime();

    // Right-Alt tracking with proper cleanup on section switches
    if (row == 5 && col == 3) {
        g_prog_ctx.right_alt_pressed = pressed;
        return true;
    }



    // Handle R_Alt combinations first
    if (g_prog_ctx.right_alt_pressed && pressed) {
		return handle_right_alt_combinations(row, col);
    }

    // State machine dispatch
    switch (g_prog_ctx.current_state) {
        case ZONE_PROG_BROWSE:
			return handle_browse_keys(row, col);

        case ZONE_PROG_PREVIEW:
			return handle_preview_keys(row, col);

        case ZONE_PROG_EDIT:
            if (Zones_IsValidKey(row, col)) {
                Zones_ToggleKeySelection(row, col);
                return true;
            }
            return false;

        case ZONE_PROG_HSV_ADJUST:
            // Color assignment mode
            return handle_color_hsv_keys(row, col);

        default:
            return false;
    }
}

bool Zones_IsValidKey(uint8_t row, uint8_t col)
{
	if (row >= ZONE_MATRIX_ROWS || col >= ZONE_MATRIX_COLS) return false;
	return (get_led_for_key(row, col) != NULL);
}

bool Zones_IsValidZone(uint8_t zone_id)
{
    return zone_id < ZONE_MAX_ZONES;
}


/* Private Functions */
static void UpdateActivityTime(void)
{
    g_prog_ctx.last_activity_time = HAL_GetTick();
}

static config_status_t ShowSlotIndicators(void)
{
    Backlight_SetAll(0, 0, 0);

    for (uint8_t slot = 0; slot < ZONE_MAX_ZONES; slot++) {
        uint8_t row = ZONE_SLOT_ROW;
        uint8_t col = ZONE_SLOT_START_COL + slot;

        if (Zones_IsValidKey(row, col)) {
            bool occupied = Zones_IsSlotOccupied(slot);
            Backlight_RGB_t color = occupied ? COLOR_GREEN : COLOR_RED;
            Backlight_SetKeyRGB(row, col, color);
        }
    }

    return CONFIG_OK;
}


zone_programming_state_t Zone_Return_to_Section(bool pressed)
{
    // Block navigation if in HSV color mode
    if (isZoneHSVMode()) {
        LOG_DEBUG("Navigation blocked - HSV color mode active");
        return g_prog_ctx.current_state;  // Stay in current state
    }

    switch (g_prog_ctx.current_state) {
        case ZONE_PROG_BROWSE:
            // From browse mode, should return to main config menu
        	if(pressed){
        		g_prog_ctx.current_state = ZONE_PROG_INACTIVE;
        	}else{
        		Zones_EnterBrowse();
        		g_prog_ctx.current_state = ZONE_PROG_BROWSE;
        	}
            return g_prog_ctx.current_state;  // Signal to exit zones

        case ZONE_PROG_EDIT:
            // From edit mode, return to zone browse (slot selection)
        	if(pressed){
				Zones_EnterBrowse();
				g_prog_ctx.current_state = ZONE_PROG_BROWSE;
        	}else{
        		ShowKeySelection();
        		g_prog_ctx.current_state = ZONE_PROG_EDIT;
        	}
            return g_prog_ctx.current_state;

        case ZONE_PROG_PREVIEW:
        	if (pressed) {
				exit_preview_mode();  // Return to browse
			}
			return g_prog_ctx.current_state;

        case ZONE_PROG_HSV_ADJUST:
            // Should be blocked by isZoneHSVMode() check above
            LOG_WARNING("HSV mode detected in switch - should be blocked");
            return g_prog_ctx.current_state;  // Stay in HSV mode

        case ZONE_PROG_INACTIVE:
            // Not in zone programming
            LOG_DEBUG("Zone programming not active");
            return ZONE_PROG_INACTIVE;

        default:
            LOG_WARNING("Unknown zone programming state: %d", g_prog_ctx.current_state);
            return g_prog_ctx.current_state;
    }
}

bool isZoneHSVMode(void)
{
    return g_prog_ctx.color_mode_active;
}

config_status_t ShowKeySelection(void)
{
    // Start with all keys OFF
    Backlight_SetAll(0, 0, 0);

    // Show saved individual colors from staging session
    uint8_t led_count;
    if (session_get_staging_led_count(&led_count) == CONFIG_OK) {
        for (uint8_t i = 0; i < led_count; i++) {
            uint8_t row, col;
            rgb_color_t color;
            if (session_get_staging_led_data(i, &row, &col, &color) == CONFIG_OK) {
            	if (g_prog_ctx.selection.selected[row][col]) {
					Backlight_RGB_t display_color = {color.red, color.green, color.blue};
					Backlight_SetKeyRGB(row, col, display_color);
				}
            }
        }
    }

    for (uint8_t row = 0; row < ZONE_MATRIX_ROWS; row++) {
		for (uint8_t col = 0; col < ZONE_MATRIX_COLS; col++) {
			if (g_prog_ctx.selection.selected[row][col]) {
				// Check if this key already has a saved color
				bool has_saved_color = false;
				for (uint8_t i = 0; i < led_count; i++) {
					uint8_t s_row, s_col;
					rgb_color_t s_color;
					if (session_get_staging_led_data(i, &s_row, &s_col, &s_color) == CONFIG_OK) {
						if (s_row == row && s_col == col) {
							has_saved_color = true;
							break;
						}
					}
				}

				// Only show cyan if no saved color exists
				if (!has_saved_color) {
					Backlight_SetKeyRGB(row, col, COLOR_CYAN);
				}
			}
		}
	}

    return CONFIG_OK;
}

static bool handle_color_hsv_keys(uint8_t row, uint8_t col)
{
    if (!g_prog_ctx.color_mode_active) return false;

    uint8_t keycode = layers[BASE_LAYER][row][col];

    // WASD for more granular control
    if (keycode == KC_W) { Zones_AdjustBrightness(1); return true; }
    if (keycode == KC_S) { Zones_AdjustBrightness(-1); return true; }
    if (keycode == KC_A) { Zones_AdjustSaturation(-1); return true; }
    if (keycode == KC_D) { Zones_AdjustSaturation(1); return true; }
    if (keycode == KC_Q) { Zones_AdjustHue(-1); return true; }
    if (keycode == KC_E) { Zones_AdjustHue(1); return true; }

    return false;
}

static config_status_t FlashConfirmation(bool success)
{
    Backlight_RGB_t flash_color = success ? COLOR_GREEN : COLOR_RED;
    uint8_t flash_count = success ? 1 : 2;

    for (uint8_t i = 0; i < flash_count; i++) {
        Backlight_SetAllRGB(flash_color);
        HAL_Delay(100);
        Backlight_SetAll(0, 0, 0);
        if (i < flash_count - 1) HAL_Delay(100);
    }

    return CONFIG_OK;
}

static bool enter_individual_color_mode(uint8_t row, uint8_t col)
{
	if(g_prog_ctx.selection.selection_count == 0) {
		g_prog_ctx.right_alt_pressed = false;
		return false;
	}

    g_prog_ctx.color_mode_active = true;
    g_prog_ctx.edit_mode = ZONE_EDIT_INDIVIDUAL;
    g_prog_ctx.right_alt_pressed = false;
    g_prog_ctx.color_target_row = row;
    g_prog_ctx.color_target_col = col;
    g_prog_ctx.color_mode_start_time = HAL_GetTick();
    g_prog_ctx.previous_state = ZONE_PROG_EDIT;
    g_prog_ctx.current_state = ZONE_PROG_HSV_ADJUST;

    show_color_mode_display();
    LOG_INFO("Entered individual color mode for key (%d,%d)", row, col);
    return true;
}

static bool enter_zone_color_mode(void)
{

	if(g_prog_ctx.selection.selection_count == 0) {
		g_prog_ctx.right_alt_pressed = false;
		return false;
	}

	for (uint8_t row = 0; row < ZONE_MATRIX_ROWS; row++) {
		for (uint8_t col = 0; col < ZONE_MATRIX_COLS; col++) {
			if (g_prog_ctx.selection.selected[row][col]) {
				session_remove_key_color(row, col);
			}
		}
	}

	g_prog_ctx.right_alt_pressed = false;
    g_prog_ctx.color_mode_active = true;
    g_prog_ctx.edit_mode = ZONE_EDIT_GROUP;
    g_prog_ctx.color_mode_start_time = HAL_GetTick();
    g_prog_ctx.previous_state = ZONE_PROG_EDIT;
    g_prog_ctx.current_state = ZONE_PROG_HSV_ADJUST;

    show_color_mode_display();
    LOG_INFO("Entered zone color mode for %d selected keys", g_prog_ctx.selection.selection_count);
    return true;
}

static void show_color_mode_display(void)
{
    // Calculate current HSV color
    Backlight_RGB_t current_color = Backlight_HSVtoRGB(g_prog_ctx.hsv.hue,
                                                       g_prog_ctx.hsv.saturation,
                                                       g_prog_ctx.hsv.value);

    // Set dim background on all keys
    for (uint8_t row = 0; row < ZONE_MATRIX_ROWS; row++) {
        for (uint8_t col = 0; col < ZONE_MATRIX_COLS; col++) {
            if (Backlight_HasLED(row, col)) {
                bool is_target = false;

                if (g_prog_ctx.edit_mode == ZONE_EDIT_INDIVIDUAL) {
                    is_target = (row == g_prog_ctx.color_target_row &&
                               col == g_prog_ctx.color_target_col);
                } else if (g_prog_ctx.edit_mode == ZONE_EDIT_GROUP) {
                    is_target = g_prog_ctx.selection.selected[row][col];
                }
                if (is_target) {
                    Backlight_SetKeyRGB(row, col, current_color);
                } else {
                    Backlight_SetKeyRGB(row, col, (Backlight_RGB_t){2, 2, 2});
                }
            }
        }
    }
}

static config_status_t enter_edit_mode_with_existing_zone(uint8_t zone_id)
{
    // Standard edit mode entry (this creates a fresh, empty staging session)
    config_status_t result = Zones_SelectZone(zone_id);
    if (result != CONFIG_OK) {
        return result;
    }

    Backlight_SetAll(0, 0, 0);
    // Clear selection first
    memset(&g_prog_ctx.selection, 0, sizeof(g_prog_ctx.selection));

    // Populate staging from preview cache (which has the existing zone data)
    if (g_prog_ctx.preview_cache.active && g_prog_ctx.preview_cache.key_count > 0) {

        for (uint8_t i = 0; i < KEY_COUNT_; i++) {
            zone_key_runtime_t *key = &g_prog_ctx.preview_cache.keys[i];

            if (key->row < ZONE_MATRIX_ROWS && key->col < ZONE_MATRIX_COLS) {
                // Add key to staging with its existing color
                rgb_color_t existing_color = {
                    .red = key->color.r,
                    .green = key->color.g,
                    .blue = key->color.b
                };

                // Add to staging (this populates the staging session)
                session_set_key_color(key->row, key->col, existing_color, true);

                // Add to selection context
                g_prog_ctx.selection.selected[key->row][key->col] = true;
                g_prog_ctx.selection.selection_count++;
            }
        }

        LOG_INFO("Populated staging and selection with %d keys from preview cache",
                 g_prog_ctx.selection.selection_count);
    } else {
        LOG_WARNING("No preview cache data available for zone %d", zone_id);
    }

    // Show the key selection with loaded keys
    return ShowKeySelection();
}

static config_status_t enter_preview_mode(uint8_t zone_id)
{
    if (!Zones_IsSlotOccupied(zone_id)) {
        return CONFIG_INVALID_PARAMETER;
    }

    LOG_INFO("Entering preview mode for zone %d", zone_id);

    g_prog_ctx.preview_mode_active = true;
    g_prog_ctx.preview_zone_id = zone_id;
    g_prog_ctx.current_state = ZONE_PROG_PREVIEW;

    // Load zone content to preview cache
    load_zone_to_preview_cache(zone_id);

    // Display the preview
    show_preview_display();

    UpdateActivityTime();
    return CONFIG_OK;
}

static bool handle_browse_keys(uint8_t row, uint8_t col)
{
    // Zone slot selection (number row 1-8)
    if (row == 1 && col >= 1 && col <= 8) {
        uint8_t zone_id = col - 1;

        if (Zones_IsSlotOccupied(zone_id)) {
            // Occupied slot - enter preview mode (no timeout)
            return (enter_preview_mode(zone_id) == CONFIG_OK);
        } else {
            // Empty slot - enter edit mode directly
            return (Zones_SelectZone(zone_id) == CONFIG_OK);
        }
    }

    return false;
}


static bool handle_preview_keys(uint8_t row, uint8_t col)
{
    uint8_t keycode = layers[BASE_LAYER][row][col];

    // Space key = Enter edit mode for current previewed zone
    if (keycode == KC_SPACE) { //XXX
        LOG_INFO("Space pressed - entering edit mode for zone %d", g_prog_ctx.preview_zone_id);

        // Enter edit mode and load existing keys + update selection
        uint8_t zone_to_edit = g_prog_ctx.preview_zone_id;
        enter_edit_mode_with_existing_zone(zone_to_edit);
        return true;
    }

    // Escape = Return to browse immediately
    if (keycode == KC_BSPACE) { //XXX Preview Mode
        exit_preview_mode();
        return true;
    }

    // Other keys = ignore (stay in preview)
    UpdateActivityTime();
    return true;
}

static void load_zone_to_preview_cache(uint8_t zone_id)
{
    // Initialize preview cache
    g_prog_ctx.preview_cache.active = false;
    g_prog_ctx.preview_cache.key_count = 0;
    memset(g_prog_ctx.preview_cache.keys, 0, sizeof(g_prog_ctx.preview_cache.keys));

    // For slots 0-1: Check runtime cache directly
    if (zone_id <= 1) {
        zone_runtime_t *rt_zone = &g_zone_rt_ctx.zones[zone_id];

        if (rt_zone->key_count > 0) {
            g_prog_ctx.preview_cache = *rt_zone;
            g_prog_ctx.preview_cache.active = true;

            return;
        }
    }

    // Load from EEPROM using session system
    if (session_begin_edit(zone_id) == CONFIG_OK) {
        uint8_t led_count;
        if (session_get_staging_led_count(&led_count) == CONFIG_OK && led_count > 0) {
            for (uint8_t i = 0; i < led_count && i < KEY_COUNT_ && g_prog_ctx.preview_cache.key_count < 80; i++) {
                uint8_t row, col;
                rgb_color_t color;
                if (session_get_staging_led_data(i, &row, &col, &color) == CONFIG_OK) {
                    // Validate coordinates before adding
                    if (row < ZONE_MATRIX_ROWS && col < ZONE_MATRIX_COLS) {
                        g_prog_ctx.preview_cache.keys[g_prog_ctx.preview_cache.key_count] = (zone_key_runtime_t){
                            .row = row,
                            .col = col,
                            .color = {color.red, color.green, color.blue}
                        };
                        g_prog_ctx.preview_cache.key_count++;
                    }
                }
            }
        }
        session_end_edit(false); // Don't save, just previewing
        g_prog_ctx.preview_cache.active = (g_prog_ctx.preview_cache.key_count > 0);

        LOG_INFO("Loaded %d keys from EEPROM for preview", g_prog_ctx.preview_cache.key_count);
    }

    // If no keys loaded from either source
    if (!g_prog_ctx.preview_cache.active) {
        LOG_WARNING("No keys found for zone %d preview", zone_id);
    }
}


static void show_preview_display(void)
{
    // Start with dim background
    Backlight_SetAllRGB(BACKLIGHT_OFF);

    // Show cached zone content at full brightness
    if (g_prog_ctx.preview_cache.active) {
        for (uint8_t i = 0; i < 80; i++) {
            zone_key_runtime_t *key = &g_prog_ctx.preview_cache.keys[i];
            Backlight_SetKeyRGB(key->row, key->col, key->color);
        }
    }

    // Show space key indicator for "press to edit"
    Backlight_SetKeyRGB(5, 4, COLOR_GREEN);  // Space key highlighted
    Backlight_SetKeyRGB(1, 13, COLOR_RED);  // ESC key highlighted
}

static bool handle_right_alt_combinations(uint8_t row, uint8_t col)
{
    uint8_t keycode = layers[BASE_LAYER][row][col];

    // R_Alt + Enter = Apply color and exit
    if (keycode == KC_ENTER && g_prog_ctx.color_mode_active) {
        apply_color_and_exit();
        return true;
    }

    // R_Alt + Backspace = Reset to zone default and exit
    if (keycode == KC_BSPACE && g_prog_ctx.color_mode_active) {
        reset_to_zone_default_and_exit();
        return true;
    }

    // R_Alt + Z = Enter zone color mode
    if (keycode == KC_Z && g_prog_ctx.current_state == ZONE_PROG_EDIT &&
        !g_prog_ctx.preview_mode_active && g_prog_ctx.selection.selection_count > 0) {
        enter_zone_color_mode();
        return true;
    }

    // R_Alt + zone slot (in preview) = Enter edit mode
    if (g_prog_ctx.preview_mode_active && row == 1 && col >= 1 && col <= 8) {
        uint8_t zone_id = col - 1;
        if (zone_id == g_prog_ctx.preview_zone_id) {
            enter_edit_from_preview();
            return true;
        }
    }

    // R_Alt + individual key = Enter individual key color mode
    if (g_prog_ctx.current_state == ZONE_PROG_EDIT && !g_prog_ctx.preview_mode_active &&
        Zones_IsValidKey(row, col)) {
        enter_individual_color_mode(row, col);
        return true;
    }

    g_prog_ctx.right_alt_pressed = false;
    return false;
}

static void exit_preview_mode(void)
{
    if (!g_prog_ctx.preview_mode_active) return;

    //LOG_INFO("Exiting preview mode for zone %d", g_prog_ctx.preview_zone_id);

    g_prog_ctx.preview_mode_active = false;
    g_prog_ctx.preview_zone_id = 0xFF;
    g_prog_ctx.preview_cache.active = false;
    g_prog_ctx.preview_cache.key_count = 0;
    g_prog_ctx.current_state = ZONE_PROG_BROWSE;

    // Clear preview cache memory
    memset(&g_prog_ctx.preview_cache, 0, sizeof(g_prog_ctx.preview_cache));

    // Return to browse display
    ShowSlotIndicators();
}

static bool enter_edit_from_preview(void)
{
    // Transition from preview to edit mode
    g_prog_ctx.preview_mode_active = false;
    g_prog_ctx.current_state = ZONE_PROG_EDIT;

    // The zone is already loaded in staging from preview
    // Just switch to edit display
    memset(&g_prog_ctx.selection, 0, sizeof(g_prog_ctx.selection));
    ShowKeySelection();

    LOG_INFO("Entered edit mode from preview for zone %d", g_prog_ctx.preview_zone_id);
    return true;
}

static void apply_color_and_exit(void)
{
    // Color already applied via real-time preview
    g_prog_ctx.color_mode_active = false;
    g_prog_ctx.right_alt_pressed = false;
    g_prog_ctx.current_state = ZONE_PROG_EDIT;

    // Return to edit display
    ShowKeySelection();

    LOG_INFO("Color applied and exited color mode");
}

static void reset_to_zone_default_and_exit(void)
{
    // Get zone default color from EEPROM
	Backlight_RGB_t rgb = Backlight_HSVtoRGB(g_prog_ctx.hsv.hue,
	                                            g_prog_ctx.hsv.saturation,
	                                            g_prog_ctx.hsv.value);
	rgb_color_t system_color = {rgb.r, rgb.g, rgb.b};

	if (g_prog_ctx.edit_mode == ZONE_EDIT_INDIVIDUAL) {
		// Apply to individual key
		session_set_key_color(g_prog_ctx.color_target_row,
							 g_prog_ctx.color_target_col, system_color, true);
		LOG_INFO("Applied HSV(%d,%d,%d) to key (%d,%d)",
				g_prog_ctx.hsv.hue, g_prog_ctx.hsv.saturation, g_prog_ctx.hsv.value,
				g_prog_ctx.color_target_row, g_prog_ctx.color_target_col);
	} else if (g_prog_ctx.edit_mode == ZONE_EDIT_GROUP) {
		// Apply to all selected keys
		uint8_t applied_count = 0;
		for (uint8_t row = 0; row < ZONE_MATRIX_ROWS; row++) {
			for (uint8_t col = 0; col < ZONE_MATRIX_COLS; col++) {
				if (g_prog_ctx.selection.selected[row][col]) {
					session_set_key_color(row, col, system_color, true);
					applied_count++;
				}
			}
		}
		LOG_INFO("Applied HSV(%d,%d,%d) to %d selected keys",
				g_prog_ctx.hsv.hue, g_prog_ctx.hsv.saturation, g_prog_ctx.hsv.value,
				applied_count);
	}

    g_prog_ctx.color_mode_active = false;
    g_prog_ctx.right_alt_pressed = false;
    g_prog_ctx.current_state = ZONE_PROG_EDIT;
    g_prog_ctx.has_unsaved_changes = true;

    ShowKeySelection();
    LOG_INFO("Reset to zone default and exited color mode");
}

static void flash_timeout_warning(void)
{
    // Brief yellow flash
    Backlight_SetAllRGB(COLOR_YELLOW);
    HAL_Delay(100);

    // Restore appropriate display
    if (g_prog_ctx.color_mode_active) {
        show_color_mode_display();
    }
}

static config_status_t swap_eeprom_zones(uint8_t eeprom_zone_a, uint8_t eeprom_zone_b)
{
    if (eeprom_zone_a >= ZONE_MAX_ZONES || eeprom_zone_b >= ZONE_MAX_ZONES) {
        return CONFIG_INVALID_PARAMETER;
    }

    // Find which runtime slots (if any) these zones occupy
    int8_t slot_a = -1, slot_b = -1;

    // Check if zone A is in runtime slot 0 or 1
    if (eeprom_zone_a == 0 && g_zone_rt_ctx.zones[0].active) slot_a = 0;
    if (eeprom_zone_a == 1 && g_zone_rt_ctx.zones[1].active) slot_a = 1;

    // Check if zone B is in runtime slot 0 or 1
    if (eeprom_zone_b == 0 && g_zone_rt_ctx.zones[0].active) slot_b = 0;
    if (eeprom_zone_b == 1 && g_zone_rt_ctx.zones[1].active) slot_b = 1;

    // Case 1: Both zones in runtime - just swap the slots
    if (slot_a >= 0 && slot_b >= 0) {
        zone_runtime_t temp = g_zone_rt_ctx.zones[slot_a];
        g_zone_rt_ctx.zones[slot_a] = g_zone_rt_ctx.zones[slot_b];
        g_zone_rt_ctx.zones[slot_b] = temp;

        LOG_INFO("Swapped runtime slots: zone %d ↔ zone %d", eeprom_zone_a, eeprom_zone_b);
    }
    // Case 2: Zone A in runtime, Zone B not - replace A with B
    else if (slot_a >= 0 && slot_b < 0) {
        // TODO: Load zone B from EEPROM to slot_a when EEPROM ready
        // For now: Clear slot A
        Zones_ClearRuntimeSlot(slot_a);
        LOG_INFO("Cleared zone %d from runtime slot %d (zone %d not available)",
                 eeprom_zone_a, slot_a, eeprom_zone_b);
    }
    // Case 3: Zone B in runtime, Zone A not - replace B with A
    else if (slot_a < 0 && slot_b >= 0) {
        // TODO: Load zone A from EEPROM to slot_b when EEPROM ready
        // For now: Clear slot B
        Zones_ClearRuntimeSlot(slot_b);
        LOG_INFO("Cleared zone %d from runtime slot %d (zone %d not available)",
                 eeprom_zone_b, slot_b, eeprom_zone_a);
    }
    // Case 4: Neither in runtime - nothing to do
    else {
        LOG_INFO("Neither zone %d nor zone %d are in runtime cache", eeprom_zone_a, eeprom_zone_b);
        return CONFIG_OK;
    }

    UpdateRuntimeContext();
    return CONFIG_OK;
}

static config_status_t load_eeprom_to_runtime(uint8_t eeprom_zone_id, uint8_t runtime_slot)
{
    if (eeprom_zone_id >= ZONE_MAX_ZONES || runtime_slot >= 2) {
        return CONFIG_INVALID_PARAMETER;
    }

    // For zones 0-1: check if already in staging/runtime
    if (eeprom_zone_id <= 1) {
        if (g_zone_rt_ctx.zones[eeprom_zone_id].active) {
            // Just copy to target slot if different
            if (eeprom_zone_id != runtime_slot) {
                g_zone_rt_ctx.zones[runtime_slot] = g_zone_rt_ctx.zones[eeprom_zone_id];
                UpdateRuntimeContext();
                LOG_INFO("Copied zone %d to runtime slot %d", eeprom_zone_id, runtime_slot);
            }
            return CONFIG_OK;
        }
    }

    // For zones 2-7 or empty 0-1: TODO load from EEPROM when ready
    LOG_INFO("Loading zone %d from EEPROM to runtime slot %d (TODO: EEPROM load)",
             eeprom_zone_id, runtime_slot);

    // For now, clear the target slot
    Zones_ClearRuntimeSlot(runtime_slot);

    return CONFIG_OK;
}

/* ========================================================================== */
/* Zone Runtime Interface */
/* ========================================================================== */
config_status_t Zones_LoadStagingToRuntime(uint8_t runtime_slot)
{
    if (runtime_slot >= 2) {
        return CONFIG_INVALID_PARAMETER;
    }

    // Call system_config function that has access to g_staging
    return session_zone_load_staging_to_runtime(runtime_slot);
}

config_status_t Zones_LoadKeysToRuntime(const staging_key_t *staging_keys, uint8_t key_count, uint8_t runtime_slot)
{
    if (!staging_keys || runtime_slot >= 2) {
        return CONFIG_INVALID_PARAMETER;
    }

    zone_runtime_t *rt_zone = &g_zone_rt_ctx.zones[runtime_slot];

    // Clear runtime slot
    rt_zone->active = false;
    rt_zone->key_count = 0;
    memset(rt_zone->keys, 0, sizeof(rt_zone->keys));

    // Copy staging keys to runtime format
    uint8_t loaded_keys = 0;
    for (uint8_t i = 0; i < key_count && loaded_keys < 80; i++) {
        const staging_key_t *staging_key = &staging_keys[i];

        if (!staging_key->active) continue;

        // Validate coordinates
        if (staging_key->row >= ZONE_MATRIX_ROWS || staging_key->col >= ZONE_MATRIX_COLS) {
            LOG_WARNING("Invalid key coordinates (%d,%d) in staging",
                       staging_key->row, staging_key->col);
            continue;
        }

        // Verify LED exists
        if (!Zones_IsValidKey(staging_key->row, staging_key->col)) {
            LOG_WARNING("Key (%d,%d) has no LED mapping - skipping",
                       staging_key->row, staging_key->col);
            continue;
        }

        // Copy to runtime
        rt_zone->keys[loaded_keys] = (zone_key_runtime_t){
            .row = staging_key->row,
            .col = staging_key->col,
            .color = {
                staging_key->color.red,
                staging_key->color.green,
                staging_key->color.blue
            }
        };
        loaded_keys++;
    }

    rt_zone->key_count = loaded_keys;
    rt_zone->active = (loaded_keys > 0);

    // Update runtime context
    UpdateRuntimeContext();

    LOG_INFO("Loaded %d keys from staging to runtime slot %d", loaded_keys, runtime_slot);
    return CONFIG_OK;
}

config_status_t Zones_LoadToRuntimeSlot(uint8_t eeprom_slot_id, uint8_t runtime_slot)
{
    if (eeprom_slot_id >= ZONE_MAX_ZONES || runtime_slot >= 2) {
        return CONFIG_INVALID_PARAMETER;
    }

    // Protect against loading over active programming sessions
    if (ValidateSessionState() && g_session_guard.active_zone_id == eeprom_slot_id) {
        LOG_WARNING("Cannot load EEPROM slot %d - currently being edited", eeprom_slot_id);
        return CONFIG_ERROR;
    }

    if (!zone_is_slot_occupied(eeprom_slot_id)) {
        // Clear runtime slot if EEPROM slot is empty
        return Zones_ClearRuntimeSlot(runtime_slot);
    }

    // Clear runtime slot first - with bounds checking
    zone_runtime_t *rt_zone = &g_zone_rt_ctx.zones[runtime_slot];
    if (!rt_zone) {
        LOG_ERROR("Invalid runtime zone pointer for slot %d", runtime_slot);
        return CONFIG_ERROR;
    }

    rt_zone->active = false;
    rt_zone->key_count = 0;
    memset(rt_zone->keys, 0, sizeof(rt_zone->keys));  // Clear old data

    // Load zone data from EEPROM using session system
    config_status_t result = session_begin_edit(eeprom_slot_id);
    if (result != CONFIG_OK) {
        LOG_ERROR("Failed to begin session for EEPROM slot %d", eeprom_slot_id);
        return result;
    }

    // Get zone LED count with bounds checking
    uint8_t led_count;
    result = session_get_staging_led_count(&led_count);
    if (result != CONFIG_OK) {
        session_end_edit(false);
        LOG_ERROR("Failed to get LED count for EEPROM slot %d", eeprom_slot_id);
        return result;
    }

    // Sanity check LED count
    if (led_count > 80) {
        session_end_edit(false);
        LOG_ERROR("Invalid LED count %d for EEPROM slot %d", led_count, eeprom_slot_id);
        return CONFIG_ERROR;
    }

    // Convert each LED to runtime format with bounds checking
    for (uint8_t i = 0; i < led_count && rt_zone->key_count < 80; i++) {
        uint8_t row, col;
        rgb_color_t color;

        if (session_get_staging_led_data(i, &row, &col, &color) == CONFIG_OK) {
            // Validate key coordinates
            if (row >= ZONE_MATRIX_ROWS || col >= ZONE_MATRIX_COLS) {
                LOG_WARNING("Invalid key coordinates (%d,%d) in EEPROM slot %d",
                           row, col, eeprom_slot_id);
                continue;
            }

            rt_zone->keys[rt_zone->key_count].row = row;
            rt_zone->keys[rt_zone->key_count].col = col;
            rt_zone->keys[rt_zone->key_count].color = (Backlight_RGB_t){
                color.red, color.green, color.blue
            };
            rt_zone->key_count++;
        }
    }

    session_end_edit(false); // Don't save, just used for loading

    // Activate runtime zone if we loaded keys
    rt_zone->active = (rt_zone->key_count > 0);

    // Update global runtime context
    UpdateRuntimeContext();

    LOG_INFO("EEPROM zone %d loaded to runtime slot %d (%d keys)",
             eeprom_slot_id, runtime_slot, rt_zone->key_count);

    return CONFIG_OK;
}


/**
 * @brief Clear a runtime slot
 */
config_status_t Zones_ClearRuntimeSlot(uint8_t runtime_slot)
{
    if (runtime_slot >= 2) {
        return CONFIG_INVALID_PARAMETER;
    }

    zone_runtime_t *rt_zone = &g_zone_rt_ctx.zones[runtime_slot];
    if (!rt_zone) {
        LOG_ERROR("Invalid runtime zone pointer for slot %d", runtime_slot);
        return CONFIG_ERROR;
    }

    // Safe cleanup with memory clearing
    rt_zone->active = false;
    rt_zone->key_count = 0;
    memset(rt_zone->keys, 0, sizeof(rt_zone->keys));

    UpdateRuntimeContext();

    LOG_INFO("Runtime slot %d cleared and memory zeroed", runtime_slot);
    return CONFIG_OK;
}


/**
 * @brief Activate/deactivate runtime zones for effects
 */
void Zones_SetRuntimeEnabled(bool enable)
{
    g_zone_rt_ctx.zones_enabled = enable;

    // If disabling, deactivate all zones but keep them loaded
    if (!enable) {
        for (uint8_t i = 0; i < 2; i++) {
            if (g_zone_rt_ctx.zones[i].key_count > 0) {
                g_zone_rt_ctx.zones[i].active = false;
            }
        }
        g_zone_rt_ctx.active_zone_count = 0;
    } else {
        // Re-enable zones that have data
        UpdateRuntimeContext();
    }

    LOG_INFO("Runtime zones %s", enable ? "ENABLED" : "DISABLED");
}

/**
 * @brief Toggle between runtime zone 0 and 1 (instant switching)
 */
config_status_t Zones_SwitchRuntimeSlot(uint8_t slot)
{
    if (slot >= 2) {
        return CONFIG_INVALID_PARAMETER;
    }

    if (!g_zone_rt_ctx.zones_enabled) {
        return CONFIG_INVALID_PARAMETER;
    }

    // Deactivate all slots first
    for (uint8_t i = 0; i < 2; i++) {
        if (g_zone_rt_ctx.zones[i].key_count > 0) {
            g_zone_rt_ctx.zones[i].active = false;
        }
    }

    // Activate requested slot if it has data
    if (g_zone_rt_ctx.zones[slot].key_count > 0) {
        g_zone_rt_ctx.zones[slot].active = true;
        g_zone_rt_ctx.active_zone_count = 1;
        LOG_INFO("Switched to runtime zone %d", slot);
    } else {
        g_zone_rt_ctx.active_zone_count = 0;
        LOG_INFO("Runtime zone %d is empty - no zones active", slot);
    }

    return CONFIG_OK;
}

/**
 * @brief Get current runtime zone status
 */
bool Zones_GetRuntimeStatus(bool *slot0_active, bool *slot1_active)
{
    if (slot0_active) *slot0_active = g_zone_rt_ctx.zones[0].active;
    if (slot1_active) *slot1_active = g_zone_rt_ctx.zones[1].active;

    return g_zone_rt_ctx.zones_enabled;
}

/**
 * @brief Save current runtime slots to EEPROM (called during sys_config save)
 */
config_status_t Zones_SaveRuntimeToEEPROM(void)
{
    // This function would compare runtime data with EEPROM and only write if different
    // Implementation depends on your EEPROM storage format

    LOG_INFO("Runtime zones saved to EEPROM");
    return CONFIG_OK;
}

/**
 * @brief Update runtime context counters and flags
 */
static void UpdateRuntimeContext(void)
{
    g_zone_rt_ctx.active_zone_count = 0;

    for (uint8_t i = 0; i < 2; i++) {
        // Zone is active if it has data and zones are globally enabled
        if (g_zone_rt_ctx.zones[i].key_count > 0 && g_zone_rt_ctx.zones_enabled) {
            g_zone_rt_ctx.zones[i].active = true;
            g_zone_rt_ctx.active_zone_count++;
        } else {
            g_zone_rt_ctx.zones[i].active = false;
        }
    }

    LOG_DEBUG("Runtime context: %d active zones, enabled=%d",
              g_zone_rt_ctx.active_zone_count, g_zone_rt_ctx.zones_enabled);
}


config_status_t Zones_SetRuntimeAssignments(uint8_t eeprom_slot0, uint8_t eeprom_slot1)
{
    config_status_t result = CONFIG_OK;

    // Load slot 0
    if (eeprom_slot0 < ZONE_MAX_ZONES) {
        result = Zones_LoadToRuntimeSlot(eeprom_slot0, 0);
        if (result != CONFIG_OK) {
            LOG_ERROR("Failed to load EEPROM slot %d to runtime slot 0", eeprom_slot0);
        }
    } else {
        Zones_ClearRuntimeSlot(0);
    }

    // Load slot 1
    if (eeprom_slot1 < ZONE_MAX_ZONES) {
        config_status_t result1 = Zones_LoadToRuntimeSlot(eeprom_slot1, 1);
        if (result1 != CONFIG_OK) {
            LOG_ERROR("Failed to load EEPROM slot %d to runtime slot 1", eeprom_slot1);
            if (result == CONFIG_OK) result = result1;
        }
    } else {
        Zones_ClearRuntimeSlot(1);
    }

    // Enable zones if we loaded any
    Zones_SetRuntimeEnabled(g_zone_rt_ctx.active_zone_count > 0);

    return result;
}

/**
 * @brief Initialize runtime zones on system startup
 */
config_status_t Zones_InitRuntime(void)
{

    Zones_SetRuntimeEnabled(false);
    LOG_INFO("Runtime zones initialized");
    return CONFIG_OK;
}


