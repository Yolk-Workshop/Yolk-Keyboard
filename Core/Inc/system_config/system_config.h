/*
 * system_config.h
 *
 *  Created on: Jun 13, 2025
 *      Author: bettysidepiece
 */

#ifndef SYSTEM_CONFIG_SYSTEM_CONFIG_H_
#define SYSTEM_CONFIG_SYSTEM_CONFIG_H_

#include "m24512e_driver.h"
#include "eeprom_memory_map.h"
#include <stdint.h>
#include <stdbool.h>
#include "led_mapping.h"

#define MAX_STAGING_KEYS 80


/* Status Codes */
typedef enum {
   CONFIG_OK = 0,
   CONFIG_ERROR,
   CONFIG_CRC_ERROR,
   CONFIG_INVALID_PARAMETER,
   CONFIG_EEPROM_ERROR,
   CONFIG_NOT_FOUND,
   CONFIG_TIMEOUT,
   CONFIG_MEMORY_ERROR
} config_status_t;


/* Data Structures */
typedef struct {
   uint8_t red;
   uint8_t green;
   uint8_t blue;
} rgb_color_t;

typedef struct {
   uint8_t pattern_type;
   uint8_t speed;
   uint8_t intensity;
   uint8_t direction;
} pattern_config_t;

typedef struct {
   bool first_boot;
   bool safe_mode;
   bool hardware_detected;
   bool update_pending;
} boot_flags_t;

typedef struct {
   uint16_t auto_dim_timeout_s;
   uint16_t auto_off_timeout_s;
   uint8_t dim_percentage;
   bool wake_on_keypress;
} power_management_t;

typedef struct {
	uint8_t row : 3;        // 3 bits (0-5 rows)
	uint8_t col : 4;        // 4 bits (0-13 cols)
    rgb_color_t color;
    bool active;
    bool color_modified;
} staging_key_t;


typedef struct {
    config_status_t (*load_keys_to_runtime)(const staging_key_t *keys, uint8_t count, uint8_t slot);
    bool (*is_runtime_slot_occupied)(uint8_t runtime_slot);
} zone_interface_t;

// Function to set the callback
void session_set_zone_interface(const zone_interface_t *interface);

/* System Data API */
config_status_t system_get_device_id(uint32_t *device_id);
config_status_t system_get_firmware_version(uint32_t *version);
config_status_t system_get_boot_flags(boot_flags_t *flags);
config_status_t system_set_boot_flags(const boot_flags_t *flags);
config_status_t system_get_last_modified(uint32_t *timestamp);
config_status_t system_update_section_crc(uint8_t section_id);
config_status_t system_verify_section_crc(uint8_t section_id, bool *valid);

/* Hardware Detection API */
config_status_t hardware_get_keyboard_version(uint8_t *board_version);
config_status_t hardware_get_led_count(uint8_t *led_count);
config_status_t hardware_get_switch_count(uint8_t *switch_count);
config_status_t hardware_get_serial_number(char *serial_number, uint8_t max_len);
config_status_t hardware_is_proximity_present(bool *present);
config_status_t hardware_get_proximity_threshold(uint8_t *threshold_mm);
config_status_t hardware_is_bm71_present(bool *present);
config_status_t hardware_get_bm71_mac(uint8_t mac_address[6]);
config_status_t hardware_is_haptic_present(bool *present);
config_status_t hardware_get_haptic_type(uint8_t *driver_type);

/* Zone Programming API */
config_status_t zone_get_count(uint8_t *zone_count);
config_status_t zone_get_name(uint8_t zone_id, char *name, uint8_t max_len);
config_status_t zone_set_name(uint8_t zone_id, const char *name);
config_status_t zone_get_color(uint8_t zone_id, rgb_color_t *color);
config_status_t zone_set_color(uint8_t zone_id, const rgb_color_t *color);
config_status_t zone_get_pattern(uint8_t zone_id, pattern_config_t *pattern);
config_status_t zone_set_pattern(uint8_t zone_id, const pattern_config_t *pattern);
config_status_t zone_is_enabled(uint8_t zone_id, bool *enabled);
config_status_t zone_set_enabled(uint8_t zone_id, bool enabled);
config_status_t zone_get_led_count(uint8_t zone_id, uint8_t *led_count);
config_status_t zone_get_led_indices(uint8_t zone_id, uint8_t *indices, uint8_t max_count);
config_status_t zone_set_led_indices(uint8_t zone_id, const uint8_t *indices, uint8_t count);

/* Zone Assignment API */

config_status_t zone_assign_get_key_zone(uint8_t key_id, uint8_t *zone_id);
config_status_t zone_assign_set_key_zone(uint8_t key_id, uint8_t zone_id);
config_status_t zone_assign_get_priority(uint8_t zone_id, uint8_t *priority);
config_status_t zone_assign_set_priority(uint8_t zone_id, uint8_t priority);
config_status_t zone_assign_is_global_enabled(bool *enabled);
config_status_t zone_assign_set_global_enabled(bool enabled);
bool zone_is_slot_occupied(uint8_t slot_id);
config_status_t zone_get_basic_info(uint8_t slot_id, uint8_t *led_count,
                                   rgb_color_t *primary_color, bool *enabled);
config_status_t zone_get_led_indices_raw(uint8_t slot_id, uint8_t *indices,
                                         uint8_t max_indices, uint8_t *actual_count);
config_status_t zone_save_basic_data(uint8_t slot_id, uint8_t led_count,
                                     const staging_key_t *keys);

/* Zone Effects API */
config_status_t effects_get_fade_time(uint16_t *fade_time_ms);
config_status_t effects_set_fade_time(uint16_t fade_time_ms);
config_status_t effects_is_breathing_enabled(bool *enabled);
config_status_t effects_set_breathing_enabled(bool enabled);
config_status_t effects_get_breathing_period(uint16_t *period_ms);
config_status_t effects_set_breathing_period(uint16_t period_ms);
config_status_t effects_is_reactive_enabled(bool *enabled);
config_status_t effects_set_reactive_enabled(bool enabled);
config_status_t effects_get_reactive_fade(uint16_t *fade_ms);
config_status_t effects_set_reactive_fade(uint16_t fade_ms);

/* User Configuration API */
config_status_t user_get_global_brightness(uint8_t *brightness);
config_status_t user_set_global_brightness(uint8_t brightness);
config_status_t user_get_brightness_preset(uint8_t preset_id, uint8_t *brightness);
config_status_t user_set_brightness_preset(uint8_t preset_id, uint8_t brightness);
config_status_t user_get_active_preset(uint8_t *preset_id);
config_status_t user_set_active_preset(uint8_t preset_id);
config_status_t user_get_power_management(power_management_t *power_mgmt);
config_status_t user_set_power_management(const power_management_t *power_mgmt);
config_status_t user_get_animation_speed(uint8_t *speed);
config_status_t user_set_animation_speed(uint8_t speed);
config_status_t user_get_key_repeat_delay(uint16_t *delay_ms);
config_status_t user_set_key_repeat_delay(uint16_t delay_ms);
config_status_t user_get_debounce_time(uint8_t *debounce_ms);
config_status_t user_set_debounce_time(uint8_t debounce_ms);

/* Custom Keybinding API */
config_status_t keybind_get_function_key(uint8_t fn_key, uint8_t *action);
config_status_t keybind_set_function_key(uint8_t fn_key, uint8_t action);
config_status_t keybind_get_media_key(uint8_t media_function, uint8_t *key_id);
config_status_t keybind_set_media_key(uint8_t media_function, uint8_t key_id);
config_status_t keybind_get_lighting_key(uint8_t lighting_function, uint8_t *key_id);
config_status_t keybind_set_lighting_key(uint8_t lighting_function, uint8_t key_id);

/* Operational Data API (Read-Only) */
config_status_t operational_get_charge_cycles(uint32_t *cycles);
config_status_t operational_get_battery_capacity(uint8_t *percentage);
config_status_t operational_get_total_keystrokes(uint64_t *keystrokes);
config_status_t operational_get_uptime_hours(uint32_t *hours);
config_status_t operational_get_error_count(uint32_t *errors);
config_status_t operational_get_last_error(uint16_t *error_code, uint32_t *timestamp);

/* Operational Data Update API (Internal Use) */
config_status_t operational_update_battery_health(uint32_t charge_cycles, uint8_t capacity_pct);
config_status_t operational_update_usage_stats(uint64_t keystrokes, uint32_t uptime_hours);
config_status_t operational_log_error(uint16_t error_code);
config_status_t operational_increment_power_cycles(void);

/* Session Management API */
config_status_t session_begin_edit(uint8_t zone_id);
config_status_t session_has_unsaved_changes(bool *has_changes);
config_status_t session_commit_changes(void);
config_status_t session_discard_changes(void);
config_status_t session_backup_current_state(void);
config_status_t session_restore_backup_state(void);
config_status_t session_get_staging_led_count(uint8_t *led_count);
config_status_t session_get_staging_led_data(uint8_t index, uint8_t *row,
                                            uint8_t *col, rgb_color_t *color);
config_status_t session_set_key_color(uint8_t row, uint8_t col, rgb_color_t color, bool type);
config_status_t session_remove_key_color(uint8_t row, uint8_t col);
config_status_t session_end_edit(bool save_changes);
config_status_t session_zone_load_staging_to_runtime(uint8_t runtime_slot);

/* Maintenance & Lifecycle API */
config_status_t config_init(m24512e_handle_t *eeprom_handle);
config_status_t config_factory_reset(void);
config_status_t config_verify_integrity(void);
config_status_t config_create_backup(void);
config_status_t config_restore_backup(void);
config_status_t config_perform_maintenance_update(void);  // Called from LPTIM handler
uint32_t config_calculate_adaptive_interval(void);       // Returns next update interval in hours

#define BACKLIGHT_TO_RGB(bl_color) \
    ((rgb_color_t){(bl_color).r, (bl_color).g, (bl_color).b})

#define RGB_TO_BACKLIGHT(rgb_color) \
    ((Backlight_RGB_t){(rgb_color).red, (rgb_color).green, (rgb_color).blue})

#endif /* SYSTEM_CONFIG_SYSTEM_CONFIG_H_ */
