/*
 * zones.h - Zone Programming Interface (Programming Interface Only)
 */

#ifndef BACKLIGHT_CORE_EXTENSION_ZONES_H_
#define BACKLIGHT_CORE_EXTENSION_ZONES_H_

#include <stdint.h>
#include <stdbool.h>
#include "system_config.h"
#include "backlight.h"

/* Zone Programming States */
typedef enum {
    ZONE_PROG_INACTIVE = 0,
    ZONE_PROG_BROWSE,
    ZONE_PROG_EDIT,
    ZONE_PROG_HSV_ADJUST,
	ZONE_PROG_PREVIEW
} zone_programming_state_t;

typedef enum {
    ZONE_EDIT_INDIVIDUAL = 0,
    ZONE_EDIT_GROUP
} zone_edit_mode_t;

/* HSV Color System */
typedef struct {
    uint16_t hue;           // 0-359
    uint8_t saturation;     // 0-100
    uint8_t value;          // 0-100
    uint8_t hue_step;
    uint8_t sv_step;
} zone_hsv_t;

typedef struct {
    uint8_t row, col;
    Backlight_RGB_t color;
} zone_key_runtime_t;

typedef struct {
    bool active;
    uint8_t key_count;
    zone_key_runtime_t keys[80];
} zone_runtime_t;

typedef struct {
    zone_runtime_t zones[2];
    uint8_t active_zone_count;
    bool zones_enabled;
} zone_runtime_ctx_t;


extern zone_runtime_ctx_t g_zone_rt_ctx;
/* Key Selection System */
#define ZONE_MATRIX_ROWS    6
#define ZONE_MATRIX_COLS    14
#define RUNTIME_ZONES	2

typedef struct {
    bool selected[ZONE_MATRIX_ROWS][ZONE_MATRIX_COLS];
    uint8_t selection_count;
} zone_key_selection_t;

/* Zone Programming Context */
typedef struct {
    zone_programming_state_t current_state;
    zone_programming_state_t previous_state;
    zone_edit_mode_t edit_mode;

    uint32_t state_start_time;
    uint32_t last_activity_time;
    uint32_t timeout_duration_ms;
    bool auto_exit_enabled;

    uint8_t editing_zone_id;
    bool has_unsaved_changes;

    zone_key_selection_t selection;
    zone_hsv_t hsv;

    uint8_t error_count;
    config_status_t last_error;

    bool has_default_color_shown;
	Backlight_RGB_t current_default_color;
	bool color_explicitly_programmed;

    bool preview_mode_active;
    uint8_t preview_zone_id;
    zone_runtime_t preview_cache;

    bool color_mode_active;
    uint8_t color_target_row, color_target_col;
    uint32_t color_mode_start_time;
    bool right_alt_pressed;

} zone_programming_context_t;

/* Constants */
#define ZONE_PROGRAMMING_TIMEOUT_MS     600000ULL  // 10 minutes
#define ZONE_MAX_ZONES                  8

/* Zone Programming API */
config_status_t Zones_Init(void);
config_status_t Zones_Process(void);
bool Zones_IsProgammingActive(void);

/* Zone Data Management */
bool Zones_IsSlotOccupied(uint8_t slot_id);
uint8_t Zones_GetKeyCount(uint8_t slot_id);

/* Zone Programming Entry/Exit */
config_status_t Zones_EnterProgramming(void);
config_status_t Zones_ExitProgramming(bool save_changes);

/* Zone Browse/Edit Interface */
config_status_t Zones_EnterBrowse(void);
config_status_t Zones_SelectZone(uint8_t zone_id);
config_status_t Zones_EnterEdit(uint8_t zone_id);
bool isZoneHSVMode(void);
zone_programming_state_t Zone_Return_to_Section(bool pressed);
/* Key Selection Interface */
config_status_t Zones_ToggleKeySelection(uint8_t row, uint8_t col);
config_status_t Zones_ClearSelection(void);
config_status_t Zones_SelectAllKeys(void);

/* HSV Color Selection */
config_status_t Zones_EnterHSV(void);
config_status_t Zones_AdjustHue(int8_t direction);
config_status_t Zones_AdjustBrightness(int8_t direction);
config_status_t Zones_ApplyHSVColor(void);
config_status_t ShowKeySelection(void);
config_status_t Zones_AdjustSaturation(int8_t direction);

/* Key Event Handling */
bool Zones_HandleKey(uint8_t row, uint8_t col, bool pressed);

/* Utility Functions */
bool Zones_IsValidKey(uint8_t row, uint8_t col);
bool Zones_IsValidZone(uint8_t zone_id);

//Runtime Zone Support
config_status_t Zones_LoadToRuntimeSlot(uint8_t eeprom_slot_id, uint8_t runtime_slot);
config_status_t Zones_ClearRuntimeSlot(uint8_t runtime_slot);
void Zones_SetRuntimeEnabled(bool enable);
config_status_t Zones_SwitchRuntimeSlot(uint8_t slot);
bool Zones_GetRuntimeStatus(bool *slot0_active, bool *slot1_active);
config_status_t Zones_SaveRuntimeToEEPROM(void);
config_status_t Zones_SetRuntimeAssignments(uint8_t eeprom_slot0, uint8_t eeprom_slot1);
config_status_t Zones_InitRuntime(void);
config_status_t Zones_LoadStagingToRuntime(uint8_t runtime_slot);
config_status_t Zones_LoadKeysToRuntime(const staging_key_t *staging_keys, uint8_t key_count, uint8_t runtime_slot);

#endif /* ZONES_H_ */
