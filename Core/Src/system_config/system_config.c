/**
 * @file system_config.c
 * @brief Complete System Configuration API Implementation for Keyboard EEPROM Access
 * @author Electronics Engineer
 * @date 2025
 * @version 5.1 - Complete implementation with hardware CRC
 */

#include "system_config.h"
#include "eeprom_memory_map.h"
#include <string.h>

/* External dependencies */
extern uint32_t HAL_GetTick(void);

/* Global EEPROM handle */
static m24512e_handle_t *g_eeprom_handle = NULL;

/* Private function prototypes */
static config_status_t read_eeprom_field(uint16_t address, void *data,
		uint16_t size);
static config_status_t write_eeprom_field(uint16_t address, const void *data,
		uint16_t size);
static config_status_t calculate_section_crc(uint16_t base_address,
		uint16_t size, uint16_t *crc);
static uint32_t get_current_timestamp(void);

/* ========================================================================
 * SYSTEM DATA API
 * ======================================================================== */
config_status_t system_get_device_id(uint32_t *device_id)
{
	if (!device_id || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}
	return read_eeprom_field(DEVICE_ID, device_id, sizeof(uint32_t));
}

config_status_t system_get_firmware_version(uint32_t *version)
{
	if (!version || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}
	return read_eeprom_field(FW_VERSION, version, sizeof(uint32_t));
}

config_status_t system_get_boot_flags(boot_flags_t *flags)
{
	if (!flags || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint8_t boot_flags_byte;
	config_status_t status = read_eeprom_field(BOOT_FLAGS, &boot_flags_byte, 1);

	if (status == CONFIG_OK) {
		flags->first_boot = (boot_flags_byte & 0x01) != 0;
		flags->safe_mode = (boot_flags_byte & 0x02) != 0;
		flags->hardware_detected = (boot_flags_byte & 0x04) != 0;
		flags->update_pending = (boot_flags_byte & 0x08) != 0;
	}

	return status;
}

config_status_t system_set_boot_flags(const boot_flags_t *flags)
{
	if (!flags || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint8_t boot_flags_byte = 0;
	boot_flags_byte |= flags->first_boot ? 0x01 : 0x00;
	boot_flags_byte |= flags->safe_mode ? 0x02 : 0x00;
	boot_flags_byte |= flags->hardware_detected ? 0x04 : 0x00;
	boot_flags_byte |= flags->update_pending ? 0x08 : 0x00;

	config_status_t status = write_eeprom_field(BOOT_FLAGS, &boot_flags_byte,
			1);

	if (status == CONFIG_OK) {
		uint32_t timestamp = get_current_timestamp();
		write_eeprom_field(LAST_MODIFIED, &timestamp, sizeof(uint32_t));
	}

	return status;
}

config_status_t system_get_last_modified(uint32_t *timestamp)
{
	if (!timestamp || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}
	return read_eeprom_field(LAST_MODIFIED, timestamp, sizeof(uint32_t));
}

config_status_t system_update_section_crc(uint8_t section_id)
{
	if (!g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t crc;
	uint16_t crc_address;
	config_status_t status = CONFIG_ERROR;

	switch (section_id)
	{
		case 0: // Hardware config CRC
			crc_address = HW_CONFIG_CRC;
			status = calculate_section_crc(KEYBOARD_PCB_BASE, 0x600, &crc);
			break;
		case 1: // Zone data CRC
			crc_address = ZONE_DATA_CRC;
			status = calculate_section_crc(ZONE_PROGRAMMING_BASE, 0x2800, &crc);
			break;
		case 2: // User config CRC
			crc_address = USER_CONFIG_CRC;
			status = calculate_section_crc(USER_CONFIG_BASE, 0x800, &crc);
			break;
		case 3: // Operational data CRC
			crc_address = OPERATIONAL_CRC;
			status = calculate_section_crc(OPERATIONAL_DATA_BASE, 0x1000, &crc);
			break;
		default:
			return CONFIG_INVALID_PARAMETER;
	}

	if (status == CONFIG_OK) {
		status = write_eeprom_field(crc_address, &crc, sizeof(uint16_t));
	}

	return status;
}

config_status_t system_verify_section_crc(uint8_t section_id, bool *valid)
{
	if (!valid || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t stored_crc, calculated_crc;
	uint16_t crc_address;
	config_status_t status;

	switch (section_id)
	{
		case 0: // Hardware config CRC
			crc_address = HW_CONFIG_CRC;
			status = read_eeprom_field(crc_address, &stored_crc,
					sizeof(uint16_t));
			if (status == CONFIG_OK) {
				status = calculate_section_crc(KEYBOARD_PCB_BASE, 0x600,
						&calculated_crc);
			}
			break;
		case 1: // Zone data CRC
			crc_address = ZONE_DATA_CRC;
			status = read_eeprom_field(crc_address, &stored_crc,
					sizeof(uint16_t));
			if (status == CONFIG_OK) {
				status = calculate_section_crc(ZONE_PROGRAMMING_BASE, 0x2800,
						&calculated_crc);
			}
			break;
		case 2: // User config CRC
			crc_address = USER_CONFIG_CRC;
			status = read_eeprom_field(crc_address, &stored_crc,
					sizeof(uint16_t));
			if (status == CONFIG_OK) {
				status = calculate_section_crc(USER_CONFIG_BASE, 0x800,
						&calculated_crc);
			}
			break;
		case 3: // Operational data CRC
			crc_address = OPERATIONAL_CRC;
			status = read_eeprom_field(crc_address, &stored_crc,
					sizeof(uint16_t));
			if (status == CONFIG_OK) {
				status = calculate_section_crc(OPERATIONAL_DATA_BASE, 0x1000,
						&calculated_crc);
			}
			break;
		default:
			return CONFIG_INVALID_PARAMETER;
	}

	if (status == CONFIG_OK) {
		*valid = (stored_crc == calculated_crc);
	}

	return status;
}

/* ========================================================================
 * HARDWARE DETECTION API
 * ======================================================================== */

config_status_t hardware_get_keyboard_version(uint8_t *board_version)
{
	if (!board_version || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}
	return read_eeprom_field(BOARD_VERSION, board_version, 1);
}

config_status_t hardware_get_led_count(uint8_t *led_count)
{
	if (!led_count || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	// Calculate total LED count from all zones
	uint8_t zone_count;
	config_status_t status = read_eeprom_field(ZONE_COUNT, &zone_count, 1);
	if (status != CONFIG_OK) return status;

	uint8_t total_leds = 0;
	for (uint8_t i = 0; i < zone_count && i < 16; i++) {
		uint8_t zone_led_count;
		uint16_t zone_address = ZONE_ADDRESS(i, ZONE_LED_COUNT_OFFSET);
		if (read_eeprom_field(zone_address, &zone_led_count, 1) == CONFIG_OK) {
			total_leds += zone_led_count;
		}
	}

	*led_count = total_leds;
	return CONFIG_OK;
}

config_status_t hardware_get_switch_count(uint8_t *switch_count)
{
	if (!switch_count || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	*switch_count = 80; // Standard keyboard switch count
	return CONFIG_OK;
}

config_status_t hardware_get_serial_number(char *serial_number, uint8_t max_len)
{
	if (!serial_number || max_len < 16 || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}
	return read_eeprom_field(MFG_SERIAL_NUMBER, serial_number, 16);
}

config_status_t hardware_is_proximity_present(bool *present)
{
	if (!present || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint8_t present_flag;
	config_status_t status = read_eeprom_field(PX_SENSOR_PRESENT, &present_flag,
			1);

	if (status == CONFIG_OK) {
		*present = (present_flag != 0);
	}

	return status;
}

config_status_t hardware_get_proximity_threshold(uint8_t *threshold_mm)
{
	if (!threshold_mm || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}
	return read_eeprom_field(PX_THRESHOLD_MM, threshold_mm, 1);
}

config_status_t hardware_is_bm71_present(bool *present)
{
	if (!present || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint8_t present_flag;
	config_status_t status = read_eeprom_field(BM71_MODULE_PRESENT,
			&present_flag, 1);

	if (status == CONFIG_OK) {
		*present = (present_flag != 0);
	}

	return status;
}

config_status_t hardware_get_bm71_mac(uint8_t mac_address[6])
{
	if (!mac_address || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}
	return read_eeprom_field(BM71_MAC_ADDRESS, mac_address, 6);
}

config_status_t hardware_is_haptic_present(bool *present)
{
	if (!present || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint8_t present_flag;
	config_status_t status = read_eeprom_field(HAPTIC_ENGINE_PRESENT,
			&present_flag, 1);

	if (status == CONFIG_OK) {
		*present = (present_flag != 0);
	}

	return status;
}

config_status_t hardware_get_haptic_type(uint8_t *driver_type)
{
	if (!driver_type || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}
	return read_eeprom_field(HAPTIC_I2C_ADDRESS, driver_type, 1);
}

/* ========================================================================
 * ZONE PROGRAMMING API
 * ======================================================================== */

config_status_t zone_get_count(uint8_t *zone_count)
{
	if (!zone_count || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}
	return read_eeprom_field(ZONE_COUNT, zone_count, 1);
}

config_status_t zone_get_color(uint8_t zone_id, rgb_color_t *color)
{
	if (!color || !IS_VALID_ZONE_ID(zone_id) || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t zone_address = ZONE_ADDRESS(zone_id, ZONE_COLOR_OFFSET);
	return read_eeprom_field(zone_address, color, sizeof(rgb_color_t));
}

config_status_t zone_set_color(uint8_t zone_id, const rgb_color_t *color)
{
	if (!color || !IS_VALID_ZONE_ID(zone_id) || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t zone_address = ZONE_ADDRESS(zone_id, ZONE_COLOR_OFFSET);
	config_status_t status = write_eeprom_field(zone_address, color,
			sizeof(rgb_color_t));

	if (status == CONFIG_OK) {
		system_update_section_crc(1);
	}

	return status;
}

config_status_t zone_get_pattern(uint8_t zone_id, pattern_config_t *pattern)
{
	if (!pattern || !IS_VALID_ZONE_ID(zone_id) || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t zone_address = ZONE_ADDRESS(zone_id, ZONE_PATTERN_OFFSET);
	return read_eeprom_field(zone_address, pattern, sizeof(pattern_config_t));
}

config_status_t zone_set_pattern(uint8_t zone_id,
		const pattern_config_t *pattern)
{
	if (!pattern || !IS_VALID_ZONE_ID(zone_id) || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t zone_address = ZONE_ADDRESS(zone_id, ZONE_PATTERN_OFFSET);
	config_status_t status = write_eeprom_field(zone_address, pattern,
			sizeof(pattern_config_t));

	if (status == CONFIG_OK) {
		system_update_section_crc(1);
	}

	return status;
}

config_status_t zone_is_enabled(uint8_t zone_id, bool *enabled)
{
	if (!enabled || !IS_VALID_ZONE_ID(zone_id) || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t zone_address = ZONE_ADDRESS(zone_id, ZONE_ENABLED_OFFSET);
	uint8_t enabled_flag;
	config_status_t status = read_eeprom_field(zone_address, &enabled_flag, 1);

	if (status == CONFIG_OK) {
		*enabled = (enabled_flag != 0);
	}

	return status;
}

config_status_t zone_set_enabled(uint8_t zone_id, bool enabled)
{
	if (!IS_VALID_ZONE_ID(zone_id) || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t zone_address = ZONE_ADDRESS(zone_id, ZONE_ENABLED_OFFSET);
	uint8_t enabled_flag = enabled ? 1 : 0;
	config_status_t status = write_eeprom_field(zone_address, &enabled_flag, 1);

	if (status == CONFIG_OK) {
		system_update_section_crc(1);
	}

	return status;
}

config_status_t zone_get_led_count(uint8_t zone_id, uint8_t *led_count)
{
	if (!led_count || !IS_VALID_ZONE_ID(zone_id) || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t zone_address = ZONE_ADDRESS(zone_id, ZONE_LED_COUNT_OFFSET);
	return read_eeprom_field(zone_address, led_count, 1);
}

config_status_t zone_get_led_indices(uint8_t zone_id, uint8_t *indices,
		uint8_t max_count)
{
	if (!indices || !IS_VALID_ZONE_ID(zone_id) || max_count < 32
			|| !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t zone_address = ZONE_ADDRESS(zone_id, ZONE_LED_INDICES_OFFSET);
	return read_eeprom_field(zone_address, indices, 32);
}

config_status_t zone_set_led_indices(uint8_t zone_id, const uint8_t *indices,
		uint8_t count)
{
	if (!indices || !IS_VALID_ZONE_ID(zone_id) || count > 32
			|| !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t zone_address = ZONE_ADDRESS(zone_id, ZONE_LED_INDICES_OFFSET);
	config_status_t status = write_eeprom_field(zone_address, indices, 32);

	if (status == CONFIG_OK) {
		// Update LED count
		uint16_t led_count_address = ZONE_ADDRESS(zone_id,
				ZONE_LED_COUNT_OFFSET);
		write_eeprom_field(led_count_address, &count, 1);
		system_update_section_crc(1);
	}

	return status;
}

/* ========================================================================
 * ZONE ASSIGNMENT API
 * ======================================================================== */

config_status_t zone_assign_get_key_zone(uint8_t key_id, uint8_t *zone_id)
{
	if (!zone_id || !IS_VALID_KEY_ID(key_id) || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t key_address = KEY_ZONE_ADDRESS(key_id);
	return read_eeprom_field(key_address, zone_id, 1);
}

config_status_t zone_assign_set_key_zone(uint8_t key_id, uint8_t zone_id)
{
	if (!IS_VALID_KEY_ID(key_id) || !IS_VALID_ZONE_ID(zone_id)
			|| !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t key_address = KEY_ZONE_ADDRESS(key_id);
	config_status_t status = write_eeprom_field(key_address, &zone_id, 1);

	if (status == CONFIG_OK) {
		system_update_section_crc(1);
	}

	return status;
}

config_status_t zone_assign_get_priority(uint8_t zone_id, uint8_t *priority)
{
	if (!priority || !IS_VALID_ZONE_ID(zone_id) || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t priority_address = ZONE_PRIORITY + zone_id;
	return read_eeprom_field(priority_address, priority, 1);
}

config_status_t zone_assign_set_priority(uint8_t zone_id, uint8_t priority)
{
	if (!IS_VALID_ZONE_ID(zone_id) || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t priority_address = ZONE_PRIORITY + zone_id;
	config_status_t status = write_eeprom_field(priority_address, &priority, 1);

	if (status == CONFIG_OK) {
		system_update_section_crc(1);
	}

	return status;
}

config_status_t zone_assign_is_global_enabled(bool *enabled)
{
	if (!enabled || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint8_t enabled_flag;
	config_status_t status = read_eeprom_field(GLOBAL_ZONE_ENABLED,
			&enabled_flag, 1);

	if (status == CONFIG_OK) {
		*enabled = (enabled_flag != 0);
	}

	return status;
}

config_status_t zone_assign_set_global_enabled(bool enabled)
{
	if (!g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint8_t enabled_flag = enabled ? 1 : 0;
	config_status_t status = write_eeprom_field(GLOBAL_ZONE_ENABLED,
			&enabled_flag, 1);

	if (status == CONFIG_OK) {
		system_update_section_crc(1);
	}

	return status;
}

/* ========================================================================
 * ZONE EFFECTS API
 * ======================================================================== */

config_status_t effects_get_fade_time(uint16_t *fade_time_ms)
{
	if (!fade_time_ms || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}
	return read_eeprom_field(FADE_TIME_MS, fade_time_ms, sizeof(uint16_t));
}

config_status_t effects_set_fade_time(uint16_t fade_time_ms)
{
	if (!g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	config_status_t status = write_eeprom_field(FADE_TIME_MS, &fade_time_ms,
			sizeof(uint16_t));

	if (status == CONFIG_OK) {
		system_update_section_crc(1);
	}

	return status;
}

config_status_t effects_is_breathing_enabled(bool *enabled)
{
	if (!enabled || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint8_t enabled_flag;
	config_status_t status = read_eeprom_field(BREATHING_ENABLED, &enabled_flag,
			1);

	if (status == CONFIG_OK) {
		*enabled = (enabled_flag != 0);
	}

	return status;
}

config_status_t effects_set_breathing_enabled(bool enabled)
{
	if (!g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint8_t enabled_flag = enabled ? 1 : 0;
	config_status_t status = write_eeprom_field(BREATHING_ENABLED,
			&enabled_flag, 1);

	if (status == CONFIG_OK) {
		system_update_section_crc(1);
	}

	return status;
}

config_status_t effects_get_breathing_period(uint16_t *period_ms)
{
	if (!period_ms || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}
	return read_eeprom_field(BREATHING_PERIOD_MS, period_ms, sizeof(uint16_t));
}

config_status_t effects_set_breathing_period(uint16_t period_ms)
{
	if (!g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	config_status_t status = write_eeprom_field(BREATHING_PERIOD_MS, &period_ms,
			sizeof(uint16_t));

	if (status == CONFIG_OK) {
		system_update_section_crc(1);
	}

	return status;
}

config_status_t effects_is_reactive_enabled(bool *enabled)
{
	if (!enabled || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint8_t enabled_flag;
	config_status_t status = read_eeprom_field(REACTIVE_ENABLED, &enabled_flag,
			1);

	if (status == CONFIG_OK) {
		*enabled = (enabled_flag != 0);
	}

	return status;
}

config_status_t effects_set_reactive_enabled(bool enabled)
{
	if (!g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint8_t enabled_flag = enabled ? 1 : 0;
	config_status_t status = write_eeprom_field(REACTIVE_ENABLED, &enabled_flag,
			1);

	if (status == CONFIG_OK) {
		system_update_section_crc(1);
	}

	return status;
}

config_status_t effects_get_reactive_fade(uint16_t *fade_ms)
{
	if (!fade_ms || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}
	return read_eeprom_field(REACTIVE_FADE_MS, fade_ms, sizeof(uint16_t));
}

config_status_t effects_set_reactive_fade(uint16_t fade_ms)
{
	if (!g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	config_status_t status = write_eeprom_field(REACTIVE_FADE_MS, &fade_ms,
			sizeof(uint16_t));

	if (status == CONFIG_OK) {
		system_update_section_crc(1);
	}

	return status;
}

/* ========================================================================
 * USER CONFIGURATION API
 * ======================================================================== */

config_status_t user_get_global_brightness(uint8_t *brightness)
{
	if (!brightness || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}
	return read_eeprom_field(GLOBAL_BRIGHTNESS, brightness, 1);
}

config_status_t user_set_global_brightness(uint8_t brightness)
{
	if (!IS_VALID_BRIGHTNESS(brightness) || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	config_status_t status = write_eeprom_field(GLOBAL_BRIGHTNESS, &brightness,
			1);

	if (status == CONFIG_OK) {
		system_update_section_crc(2);
	}

	return status;
}

config_status_t user_get_brightness_preset(uint8_t preset_id,
		uint8_t *brightness)
{
	if (!brightness || !IS_VALID_PRESET_ID(preset_id) || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t preset_address = BRIGHTNESS_PRESET_ADDRESS(preset_id);
	return read_eeprom_field(preset_address, brightness, 1);
}

config_status_t user_set_brightness_preset(uint8_t preset_id,
		uint8_t brightness)
{
	if (!IS_VALID_PRESET_ID(preset_id) || !IS_VALID_BRIGHTNESS(brightness)
			|| !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t preset_address = BRIGHTNESS_PRESET_ADDRESS(preset_id);
	config_status_t status = write_eeprom_field(preset_address, &brightness, 1);

	if (status == CONFIG_OK) {
		system_update_section_crc(2);
	}

	return status;
}

config_status_t user_get_active_preset(uint8_t *preset_id)
{
	if (!preset_id || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}
	return read_eeprom_field(ACTIVE_PRESET, preset_id, 1);
}

config_status_t user_set_active_preset(uint8_t preset_id)
{
	if (!IS_VALID_PRESET_ID(preset_id) || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	config_status_t status = write_eeprom_field(ACTIVE_PRESET, &preset_id, 1);

	if (status == CONFIG_OK) {
		system_update_section_crc(2);
	}

	return status;
}

config_status_t user_get_power_management(power_management_t *power_mgmt)
{
	if (!power_mgmt || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	config_status_t status;

	status = read_eeprom_field(AUTO_DIM_TIMEOUT_S,
			&power_mgmt->auto_dim_timeout_s, sizeof(uint16_t));
	if (status != CONFIG_OK) return status;

	status = read_eeprom_field(AUTO_OFF_TIMEOUT_S,
			&power_mgmt->auto_off_timeout_s, sizeof(uint16_t));
	if (status != CONFIG_OK) return status;

	status = read_eeprom_field(DIM_PERCENTAGE, &power_mgmt->dim_percentage, 1);
	if (status != CONFIG_OK) return status;

	uint8_t wake_flag;
	status = read_eeprom_field(WAKE_ON_KEYPRESS, &wake_flag, 1);
	if (status == CONFIG_OK) {
		power_mgmt->wake_on_keypress = (wake_flag != 0);
	}

	return status;
}

config_status_t user_set_power_management(const power_management_t *power_mgmt)
{
	if (!power_mgmt || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	config_status_t status;

	status = write_eeprom_field(AUTO_DIM_TIMEOUT_S,
			&power_mgmt->auto_dim_timeout_s, sizeof(uint16_t));
	if (status != CONFIG_OK) return status;

	status = write_eeprom_field(AUTO_OFF_TIMEOUT_S,
			&power_mgmt->auto_off_timeout_s, sizeof(uint16_t));
	if (status != CONFIG_OK) return status;

	status = write_eeprom_field(DIM_PERCENTAGE, &power_mgmt->dim_percentage, 1);
	if (status != CONFIG_OK) return status;

	uint8_t wake_flag = power_mgmt->wake_on_keypress ? 1 : 0;
	status = write_eeprom_field(WAKE_ON_KEYPRESS, &wake_flag, 1);
	if (status != CONFIG_OK) return status;

	system_update_section_crc(2);

	return CONFIG_OK;
}

config_status_t user_get_animation_speed(uint8_t *speed)
{
	if (!speed || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}
	return read_eeprom_field(ANIMATION_SPEED, speed, 1);
}

config_status_t user_set_animation_speed(uint8_t speed)
{
	if (!g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	config_status_t status = write_eeprom_field(ANIMATION_SPEED, &speed, 1);

	if (status == CONFIG_OK) {
		system_update_section_crc(2);
	}

	return status;
}

config_status_t user_get_key_repeat_delay(uint16_t *delay_ms)
{
	if (!delay_ms || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}
	return read_eeprom_field(KEY_REPEAT_DELAY_MS, delay_ms, sizeof(uint16_t));
}

config_status_t user_set_key_repeat_delay(uint16_t delay_ms)
{
	if (!g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	config_status_t status = write_eeprom_field(KEY_REPEAT_DELAY_MS, &delay_ms,
			sizeof(uint16_t));

	if (status == CONFIG_OK) {
		system_update_section_crc(2);
	}

	return status;
}

config_status_t user_get_debounce_time(uint8_t *debounce_ms)
{
	if (!debounce_ms || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}
	return read_eeprom_field(DEBOUNCE_TIME_MS, debounce_ms, 1);
}

config_status_t user_set_debounce_time(uint8_t debounce_ms)
{
	if (!g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	config_status_t status = write_eeprom_field(DEBOUNCE_TIME_MS, &debounce_ms,
			1);

	if (status == CONFIG_OK) {
		system_update_section_crc(2);
	}

	return status;
}

/* ========================================================================
 * CUSTOM KEYBINDING API
 * ======================================================================== */

config_status_t keybind_get_function_key(uint8_t fn_key, uint8_t *action)
{
	if (!action || !IS_VALID_FN_KEY(fn_key) || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t key_address = FUNCTION_KEY_ADDRESS(fn_key);
	return read_eeprom_field(key_address, action, 1);
}

config_status_t keybind_set_function_key(uint8_t fn_key, uint8_t action)
{
	if (!IS_VALID_FN_KEY(fn_key) || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t key_address = FUNCTION_KEY_ADDRESS(fn_key);
	config_status_t status = write_eeprom_field(key_address, &action, 1);

	if (status == CONFIG_OK) {
		system_update_section_crc(2);
	}

	return status;
}

config_status_t keybind_get_media_key(uint8_t media_function, uint8_t *key_id)
{
	if (!key_id || media_function >= 4 || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t key_address = MEDIA_CONTROLS_BASE + media_function;
	return read_eeprom_field(key_address, key_id, 1);
}

config_status_t keybind_set_media_key(uint8_t media_function, uint8_t key_id)
{
	if (media_function >= 4 || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t key_address = MEDIA_CONTROLS_BASE + media_function;
	config_status_t status = write_eeprom_field(key_address, &key_id, 1);

	if (status == CONFIG_OK) {
		system_update_section_crc(2);
	}

	return status;
}

config_status_t keybind_get_lighting_key(uint8_t lighting_function,
		uint8_t *key_id)
{
	if (!key_id || lighting_function >= 4 || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t key_address = LIGHTING_CONTROLS_BASE + lighting_function;
	return read_eeprom_field(key_address, key_id, 1);
}

config_status_t keybind_set_lighting_key(uint8_t lighting_function,
		uint8_t key_id)
{
	if (lighting_function >= 4 || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t key_address = LIGHTING_CONTROLS_BASE + lighting_function;
	config_status_t status = write_eeprom_field(key_address, &key_id, 1);

	if (status == CONFIG_OK) {
		system_update_section_crc(2);
	}

	return status;
}

/* ========================================================================
 * OPERATIONAL DATA API
 * ======================================================================== */

config_status_t operational_get_charge_cycles(uint32_t *cycles)
{
	if (!cycles || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}
	return read_eeprom_field(CHARGE_CYCLES, cycles, sizeof(uint32_t));
}

config_status_t operational_get_battery_capacity(uint8_t *percentage)
{
	if (!percentage || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}
	return read_eeprom_field(CAPACITY_PERCENTAGE, percentage, 1);
}

config_status_t operational_get_total_keystrokes(uint64_t *keystrokes)
{
	if (!keystrokes || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}
	return read_eeprom_field(TOTAL_KEYSTROKES, keystrokes, sizeof(uint64_t));
}

config_status_t operational_get_uptime_hours(uint32_t *hours)
{
	if (!hours || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}
	return read_eeprom_field(TOTAL_UPTIME_HOURS, hours, sizeof(uint32_t));
}

config_status_t operational_get_error_count(uint32_t *errors)
{
	if (!errors || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}
	return read_eeprom_field(TOTAL_ERRORS, errors, sizeof(uint32_t));
}

config_status_t operational_get_last_error(uint16_t *error_code,
		uint32_t *timestamp)
{
	if (!error_code || !timestamp || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	config_status_t status;

	status = read_eeprom_field(LAST_ERROR_CODE, error_code, sizeof(uint16_t));
	if (status != CONFIG_OK) return status;

	status = read_eeprom_field(LAST_ERROR_TIMESTAMP, timestamp,
			sizeof(uint32_t));

	return status;
}

config_status_t operational_update_battery_health(uint32_t charge_cycles,
		uint8_t capacity_pct)
{
	if (!g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	config_status_t status;

	status = write_eeprom_field(CHARGE_CYCLES, &charge_cycles,
			sizeof(uint32_t));
	if (status != CONFIG_OK) {
		return status;
	}

	status = write_eeprom_field(CAPACITY_PERCENTAGE, &capacity_pct, 1);
	if (status != CONFIG_OK) {
		return status;
	}

	uint32_t timestamp = get_current_timestamp();
	write_eeprom_field(LAST_CALIBRATION, &timestamp, sizeof(uint32_t));

	system_update_section_crc(3);

	return CONFIG_OK;
}

config_status_t operational_update_usage_stats(uint64_t keystrokes,
		uint32_t uptime_hours)
{
	if (!g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	config_status_t status;

	status = write_eeprom_field(TOTAL_KEYSTROKES, &keystrokes,
			sizeof(uint64_t));
	if (status != CONFIG_OK) {
		return status;
	}

	status = write_eeprom_field(TOTAL_UPTIME_HOURS, &uptime_hours,
			sizeof(uint32_t));
	if (status != CONFIG_OK) {
		return status;
	}

	system_update_section_crc(3);

	return CONFIG_OK;
}

config_status_t operational_log_error(uint16_t error_code)
{
	if (!g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint32_t timestamp = get_current_timestamp();
	config_status_t status;

	status = write_eeprom_field(LAST_ERROR_CODE, &error_code, sizeof(uint16_t));
	if (status != CONFIG_OK) {
		return status;
	}

	status = write_eeprom_field(LAST_ERROR_TIMESTAMP, &timestamp,
			sizeof(uint32_t));
	if (status != CONFIG_OK) {
		return status;
	}

	uint32_t error_count;
	status = read_eeprom_field(TOTAL_ERRORS, &error_count, sizeof(uint32_t));
	if (status == CONFIG_OK) {
		error_count++;
		status = write_eeprom_field(TOTAL_ERRORS, &error_count,
				sizeof(uint32_t));
	}

	if (status == CONFIG_OK) {
		system_update_section_crc(3);
	}

	return status;
}

config_status_t operational_increment_power_cycles(void)
{
	if (!g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint32_t power_cycles;
	config_status_t status = read_eeprom_field(POWER_CYCLES, &power_cycles,
			sizeof(uint32_t));

	if (status == CONFIG_OK) {
		power_cycles++;
		status = write_eeprom_field(POWER_CYCLES, &power_cycles,
				sizeof(uint32_t));
	}

	if (status == CONFIG_OK) {
		system_update_section_crc(3);
	}

	return status;
}

/* ========================================================================
 * SESSION MANAGEMENT API
 * ======================================================================== */

config_status_t session_begin_edit(uint8_t zone_id)
{
	if (!IS_VALID_ZONE_ID(zone_id) || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	config_status_t status;

	status = write_eeprom_field(CURRENT_EDIT_ZONE, &zone_id, 1);
	if (status != CONFIG_OK) return status;

	uint32_t timestamp = get_current_timestamp();
	status = write_eeprom_field(LAST_EDIT_TIMESTAMP, &timestamp,
			sizeof(uint32_t));
	if (status != CONFIG_OK) return status;

	uint8_t unsaved = 0;
	status = write_eeprom_field(UNSAVED_CHANGES, &unsaved, 1);

	return status;
}

config_status_t session_has_unsaved_changes(bool *has_changes)
{
	if (!has_changes || !g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint8_t unsaved_flag;
	config_status_t status = read_eeprom_field(UNSAVED_CHANGES, &unsaved_flag,
			1);

	if (status == CONFIG_OK) {
		*has_changes = (unsaved_flag != 0);
	}

	return status;
}

config_status_t session_commit_changes(void)
{
	if (!g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	system_update_section_crc(1); // Zone data
	system_update_section_crc(2); // User config

	uint8_t unsaved = 0;
	return write_eeprom_field(UNSAVED_CHANGES, &unsaved, 1);
}

config_status_t session_discard_changes(void)
{
	if (!g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint8_t unsaved = 0;
	return write_eeprom_field(UNSAVED_CHANGES, &unsaved, 1);
}

config_status_t session_backup_current_state(void)
{
	if (!g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint8_t valid = 1;
	return write_eeprom_field(SESSION_BACKUP_VALID, &valid, 1);
}

config_status_t session_restore_backup_state(void)
{
	if (!g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint8_t valid;
	config_status_t status = read_eeprom_field(SESSION_BACKUP_VALID, &valid, 1);

	if (status == CONFIG_OK && valid) {
		uint8_t unsaved = 0;
		status = write_eeprom_field(UNSAVED_CHANGES, &unsaved, 1);
	}

	return status;
}

/* ========================================================================
 * MAINTENANCE & LIFECYCLE API
 * ======================================================================== */

config_status_t config_init(m24512e_handle_t *eeprom_handle)
{
	if (!eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	g_eeprom_handle = eeprom_handle;

	if (!m24512e_is_device_ready(g_eeprom_handle)) {
		return CONFIG_EEPROM_ERROR;
	}

	boot_flags_t flags;
	config_status_t status = system_get_boot_flags(&flags);

	if (status == CONFIG_OK && flags.first_boot) {
		status = config_factory_reset();

		if (status == CONFIG_OK) {
			flags.first_boot = false;
			system_set_boot_flags(&flags);
		}
	}

	operational_increment_power_cycles();

	return status;
}

config_status_t config_factory_reset(void)
{
	if (!g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	config_status_t status;

	// Initialize system header
	uint32_t device_id = 0x12345678;
	uint32_t firmware_version = 0x01000000;

	status = write_eeprom_field(DEVICE_ID, &device_id, sizeof(uint32_t));
	if (status != CONFIG_OK) return status;

	status = write_eeprom_field(FW_VERSION, &firmware_version,
			sizeof(uint32_t));
	if (status != CONFIG_OK) return status;

	// Initialize zones
	uint8_t zone_count = 4;
	status = write_eeprom_field(ZONE_COUNT, &zone_count, 1);
	if (status != CONFIG_OK) return status;

	for (uint8_t i = 0; i < zone_count; i++) {
		rgb_color_t zone_color = { 255, 255, 255 }; // White
		pattern_config_t pattern = { 0, 50, 100, 0 }; // Static pattern
		bool zone_enabled = true;
		uint8_t priority = i;
		uint8_t led_count = 8;

		status = zone_set_color(i, &zone_color);
		if (status != CONFIG_OK) return status;

		status = zone_set_pattern(i, &pattern);
		if (status != CONFIG_OK) return status;

		status = zone_set_enabled(i, zone_enabled);
		if (status != CONFIG_OK) return status;

		status = zone_assign_set_priority(i, priority);
		if (status != CONFIG_OK) return status;

		uint16_t zone_address = ZONE_ADDRESS(i, ZONE_LED_COUNT_OFFSET);
		write_eeprom_field(zone_address, &led_count, 1);
	}

	// Initialize user preferences
	uint8_t default_brightness = 50;
	status = user_set_global_brightness(default_brightness);
	if (status != CONFIG_OK) return status;

	for (uint8_t i = 0; i < 4; i++) {
		uint8_t preset_brightness = 25 + (i * 25);
		status = user_set_brightness_preset(i, preset_brightness);
		if (status != CONFIG_OK) return status;
	}

	power_management_t power_mgmt = { .auto_dim_timeout_s = 300,
			.auto_off_timeout_s = 1800, .dim_percentage = 20,
			.wake_on_keypress = true };
	status = user_set_power_management(&power_mgmt);
	if (status != CONFIG_OK) return status;

	status = user_set_key_repeat_delay(500);
	if (status != CONFIG_OK) return status;

	status = user_set_debounce_time(20);
	if (status != CONFIG_OK) return status;

	// Initialize operational data with zeros
	uint32_t zero_32 = 0;
	uint64_t zero_64 = 0;
	uint8_t zero_8 = 0;

	write_eeprom_field(CHARGE_CYCLES, &zero_32, sizeof(uint32_t));
	write_eeprom_field(CAPACITY_PERCENTAGE, &zero_8, 1);
	write_eeprom_field(TOTAL_KEYSTROKES, &zero_64, sizeof(uint64_t));
	write_eeprom_field(TOTAL_UPTIME_HOURS, &zero_32, sizeof(uint32_t));
	write_eeprom_field(POWER_CYCLES, &zero_32, sizeof(uint32_t));
	write_eeprom_field(TOTAL_ERRORS, &zero_32, sizeof(uint32_t));

	// Update all section CRCs
	system_update_section_crc(0);
	system_update_section_crc(1);
	system_update_section_crc(2);
	system_update_section_crc(3);

	return CONFIG_OK;
}

config_status_t config_verify_integrity(void)
{
	if (!g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	bool valid;

	for (uint8_t i = 0; i < 4; i++) {
		config_status_t status = system_verify_section_crc(i, &valid);
		if (status != CONFIG_OK || !valid) {
			return CONFIG_CRC_ERROR;
		}
	}

	return CONFIG_OK;
}

config_status_t config_create_backup(void)
{
	if (!g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint8_t backup_buffer[512];
	config_status_t status;

	// Read and write with CRC verification
	status = read_eeprom_field(ZONE_PROGRAMMING_BASE, backup_buffer, 512);
	if (status != CONFIG_OK) return status;

	status = write_eeprom_field(BACKUP_ZONE_DATA, backup_buffer, 512);
	if (status != CONFIG_OK) return status;

	status = read_eeprom_field(USER_CONFIG_BASE, backup_buffer, 512);
	if (status != CONFIG_OK) return status;

	status = write_eeprom_field(BACKUP_USER_PREFS, backup_buffer, 512);
	if (status != CONFIG_OK) return status;

	uint8_t header_buffer[256];
	status = read_eeprom_field(SYSTEM_HEADER_BASE, header_buffer, 256);
	if (status != CONFIG_OK) return status;

	status = write_eeprom_field(BACKUP_SYSTEM_HEADER, header_buffer, 256);
	if (status != CONFIG_OK) return status;

	// Calculate backup validation CRC using driver
	uint16_t backup_crc;
	status = calculate_section_crc(CRITICAL_BACKUP_BASE, 0x500, &backup_crc);
	if (status != CONFIG_OK) return status;

	status = write_eeprom_field(BACKUP_VALIDATION_CRC, &backup_crc,
			sizeof(uint16_t));

	return status;
}

config_status_t config_restore_backup(void)
{
	if (!g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	uint16_t stored_crc, calculated_crc;
	config_status_t status = read_eeprom_field(BACKUP_VALIDATION_CRC,
			&stored_crc, sizeof(uint16_t));
	if (status != CONFIG_OK) return status;

	// Verify backup integrity using driver CRC
	status = calculate_section_crc(CRITICAL_BACKUP_BASE, 0x500,
			&calculated_crc);
	if (status != CONFIG_OK) return status;

	if (stored_crc != calculated_crc) {
		return CONFIG_CRC_ERROR;
	}

	uint8_t restore_buffer[512];

	// Restore with CRC verification
	status = read_eeprom_field(BACKUP_ZONE_DATA, restore_buffer, 512);
	if (status != CONFIG_OK) return status;

	status = write_eeprom_field(ZONE_PROGRAMMING_BASE, restore_buffer, 512);
	if (status != CONFIG_OK) return status;

	status = read_eeprom_field(BACKUP_USER_PREFS, restore_buffer, 512);
	if (status != CONFIG_OK) return status;

	status = write_eeprom_field(USER_CONFIG_BASE, restore_buffer, 512);
	if (status != CONFIG_OK) return status;

	uint8_t header_buffer[256];
	status = read_eeprom_field(BACKUP_SYSTEM_HEADER, header_buffer, 256);
	if (status != CONFIG_OK) return status;

	status = write_eeprom_field(SYSTEM_HEADER_BASE, header_buffer, 256);
	if (status != CONFIG_OK) return status;

	// Update section CRCs after restore
	system_update_section_crc(0);
	system_update_section_crc(1);
	system_update_section_crc(2);

	return CONFIG_OK;
}

config_status_t config_perform_maintenance_update(void)
{
	if (!g_eeprom_handle) {
		return CONFIG_INVALID_PARAMETER;
	}

	// Perform routine maintenance tasks
	config_status_t status = config_verify_integrity();
	if (status != CONFIG_OK) {
		// Log integrity failure and attempt backup restore
		operational_log_error(0x1001); // Integrity check failed
		status = config_restore_backup();
		if (status != CONFIG_OK) {
			operational_log_error(0x1002); // Backup restore failed
			return status;
		}
	}

	// Update system timestamp
	uint32_t timestamp = get_current_timestamp();
	write_eeprom_field(LAST_MODIFIED, &timestamp, sizeof(uint32_t));

	// Create fresh backup of critical data
	status = config_create_backup();
	if (status != CONFIG_OK) {
		operational_log_error(0x1003); // Backup creation failed
	}

	return CONFIG_OK;
}

uint32_t config_calculate_adaptive_interval(void)
{
	uint32_t power_cycles;
	if (read_eeprom_field(POWER_CYCLES, &power_cycles, sizeof(uint32_t))
			!= CONFIG_OK) {
		return 7; // Default 7 hours
	}

	static uint32_t last_power_cycles = 0;
	uint32_t recent_cycles = power_cycles - last_power_cycles;
	last_power_cycles = power_cycles;

	uint32_t maintenance_interval;
	if (recent_cycles > 10) {
		maintenance_interval = 5;  // High activity
	}
	else if (recent_cycles > 5) {
		maintenance_interval = 7;  // Normal activity
	}
	else {
		maintenance_interval = 12; // Low activity
	}

	return maintenance_interval;
}

/* ========================================================================
 * PRIVATE HELPER FUNCTIONS
 * ======================================================================== */

static config_status_t read_eeprom_field(uint16_t address, void *data,
		uint16_t size)
{
	if (!data || size == 0) {
		return CONFIG_INVALID_PARAMETER;
	}

	m24512e_status_t status;

	// Use CRC verification for critical data reads
	if (size > 1) {
		status = m24512e_read_sequential_with_crc(g_eeprom_handle, address,
				(uint8_t*) data, size);
	}
	else {
		status = m24512e_read_byte(g_eeprom_handle, address, (uint8_t*) data);
	}

	switch (status)
	{
		case M24512E_OK:
			return CONFIG_OK;
		case M24512E_TIMEOUT:
		case M24512E_ERROR:
			return CONFIG_EEPROM_ERROR;
		case M24512E_INVALID_ADDRESS:
		case M24512E_INVALID_SIZE:
			return CONFIG_INVALID_PARAMETER;
		case M24512E_CRC_ERROR:
			return CONFIG_CRC_ERROR;
		default:
			return CONFIG_ERROR;
	}
}

static config_status_t write_eeprom_field(uint16_t address, const void *data,
		uint16_t size)
{
	if (!data || size == 0) return CONFIG_INVALID_PARAMETER;

	m24512e_status_t status;

	if (size == 1) {
		status = m24512e_write_byte(g_eeprom_handle, address, *(uint8_t*) data);
	}
	else if (size <= M24512E_PAGE_SIZE) {
		// Use CRC verification for page writes
		status = m24512e_write_page_with_crc(g_eeprom_handle, address,
				(const uint8_t*) data, size);
	}
	else {
		// Use safe write with automatic page handling and CRC
		status = m24512e_write_data_safe(g_eeprom_handle, address,
				(const uint8_t*) data, size);
	}

	if (status == M24512E_OK) {
		status = m24512e_wait_write_complete(g_eeprom_handle);
	}

	switch (status)
	{
		case M24512E_OK:
			return CONFIG_OK;
		case M24512E_TIMEOUT:
		case M24512E_ERROR:
			return CONFIG_EEPROM_ERROR;
		case M24512E_INVALID_ADDRESS:
		case M24512E_INVALID_SIZE:
			return CONFIG_INVALID_PARAMETER;
		case M24512E_CRC_ERROR:
			return CONFIG_CRC_ERROR;
		default:
			return CONFIG_ERROR;
	}
}

static config_status_t calculate_section_crc(uint16_t base_address,
		uint16_t size, uint16_t *crc)
{
	if (!crc || size == 0) return CONFIG_INVALID_PARAMETER;

	uint8_t buffer[256]; // Larger buffer for efficiency
	uint16_t bytes_remaining = size;
	uint16_t current_address = base_address;
	uint16_t total_crc = 0;

	while (bytes_remaining > 0) {
		uint16_t chunk_size = (bytes_remaining > 256) ? 256 : bytes_remaining;

		m24512e_status_t status = m24512e_read_sequential(g_eeprom_handle,
				current_address, buffer, chunk_size);
		if (status != M24512E_OK) return CONFIG_EEPROM_ERROR;

		// Use driver's CRC calculation
		uint16_t chunk_crc = m24512e_calculate_crc16(buffer, chunk_size);
		total_crc ^= chunk_crc; // XOR for combining CRCs

		current_address += chunk_size;
		bytes_remaining -= chunk_size;
	}

	*crc = total_crc;
	return CONFIG_OK;
}

static uint32_t get_current_timestamp(void)
{
	return HAL_GetTick() / 1000; // Convert milliseconds to seconds
}
