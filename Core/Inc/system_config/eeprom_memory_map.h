/**
 * @file eeprom_memory_map.h
 * @brief EEPROM Memory Map for Keyboard Configuration System
 * @author Electronics Engineer
 * @date 2025
 * @version 4.0 - Exact alignment with specification
 */

#ifndef EEPROM_MEMORY_MAP_H_
#define EEPROM_MEMORY_MAP_H_

/* EEPROM Section Base Addresses */
#define SYSTEM_DATA_BASE        0x0000  // 2KB - System configuration and hardware detection [INTERNAL]
#define ZONE_PROGRAMMING_BASE   0x0800  // 10KB - Zone definitions and LED assignments [USER EXPOSED]
#define USER_CONFIG_BASE        0x3000  // 2KB - User preferences and settings [USER EXPOSED]
#define OPERATIONAL_DATA_BASE   0x3800  // 4KB - Runtime data and statistics [INTERNAL]
#define CRITICAL_BACKUP_BASE    0x4800  // 2KB - Backup of critical settings [REDUNDANCY]
#define FUTURE_EXPANSION_BASE   0x5000  // 46KB - Reserved for future use [RESERVED]

/* ========================================================================
 * SYSTEM DATA SECTION (0x0000 - 0x07FF) - 2KB
 * ========================================================================
 */

/* System Header (0x0000 - 0x00FF) - 256B */
#define SYSTEM_HEADER_BASE      (SYSTEM_DATA_BASE + 0x000)
#define DEVICE_ID               (SYSTEM_HEADER_BASE + 0x00)     // 4 bytes - Unique device identifier
#define FW_VERSION              (SYSTEM_HEADER_BASE + 0x04)     // 4 bytes - Firmware version
#define BOOT_FLAGS              (SYSTEM_HEADER_BASE + 0x08)     // 1 byte  - Boot state flags
#define SYSTEM_CRC_BASE         (SYSTEM_HEADER_BASE + 0x09)     // 8 bytes - Section CRCs
#define HW_CONFIG_CRC           (SYSTEM_CRC_BASE + 0x00)        // 2 bytes - Hardware config CRC
#define ZONE_DATA_CRC           (SYSTEM_CRC_BASE + 0x02)        // 2 bytes - Zone data CRC
#define USER_CONFIG_CRC         (SYSTEM_CRC_BASE + 0x04)        // 2 bytes - User config CRC
#define OPERATIONAL_CRC         (SYSTEM_CRC_BASE + 0x06)        // 2 bytes - Operational data CRC
#define LAST_MODIFIED           (SYSTEM_HEADER_BASE + 0x11)     // 4 bytes - Last modification timestamp
#define SYSTEM_HEADER_RESERVED  (SYSTEM_HEADER_BASE + 0x15)     // 235 bytes - Reserved

/* Keyboard PCB (0x0100 - 0x02FF) - 512B */
#define KEYBOARD_PCB_BASE       (SYSTEM_DATA_BASE + 0x100)
#define BOARD_VERSION           (KEYBOARD_PCB_BASE + 0x00)      // 1 byte  - PCB version
#define MFG_SERIAL_NUMBER       (KEYBOARD_PCB_BASE + 0x01)      // 16 bytes - Manufacturing serial
#define MFG_DATE                (KEYBOARD_PCB_BASE + 0x11)      // 4 bytes  - Manufacturing date
#define FACTORY_TEST_PASSED     (KEYBOARD_PCB_BASE + 0x15)      // 1 byte   - Factory test result
#define VERSION_LOOKUP_TABLE    (KEYBOARD_PCB_BASE + 0x16)      // 16 bytes - Version lookup table (8x uint16_t)
#define KEYBOARD_PCB_RESERVED   (KEYBOARD_PCB_BASE + 0x26)      // 474 bytes - Reserved

/* Proximity Sensor (0x0300 - 0x03FF) - 256B */
#define PROXIMITY_BASE          (SYSTEM_DATA_BASE + 0x300)
#define PX_SENSOR_PRESENT       (PROXIMITY_BASE + 0x00)         // 1 byte  - Sensor present flag
#define PX_SENSOR_VERSION       (PROXIMITY_BASE + 0x01)         // 1 byte  - Sensor version
#define PX_I2C_ADDRESS          (PROXIMITY_BASE + 0x02)         // 1 byte  - I2C address
#define PX_THRESHOLD_MM         (PROXIMITY_BASE + 0x03)         // 1 byte  - Detection threshold
#define PX_VERSION_LOOKUP       (PROXIMITY_BASE + 0x04)         // 8 bytes - Version lookup table (4x uint16_t)
#define PROXIMITY_RESERVED      (PROXIMITY_BASE + 0x0C)         // 244 bytes - Reserved

/* BM71 Module (0x0400 - 0x04FF) - 256B */
#define BM71_BASE               (SYSTEM_DATA_BASE + 0x400)
#define BM71_MODULE_PRESENT     (BM71_BASE + 0x00)              // 1 byte  - Module present flag
#define BM71_COMM_ACTIVE        (BM71_BASE + 0x01)              // 1 byte  - Communication active
#define BM71_FW_VERSION         (BM71_BASE + 0x02)              // 4 bytes - Firmware version
#define BM71_MAC_ADDRESS        (BM71_BASE + 0x06)              // 6 bytes - MAC address
#define BM71_ERROR_COUNT        (BM71_BASE + 0x0C)              // 2 bytes - Error count
#define BM71_RESERVED           (BM71_BASE + 0x0E)              // 242 bytes - Reserved

/* Haptic Engine (0x0500 - 0x05FF) - 256B */
#define HAPTIC_BASE             (SYSTEM_DATA_BASE + 0x500)
#define HAPTIC_ENGINE_PRESENT   (HAPTIC_BASE + 0x00)            // 1 byte  - Engine present flag
#define HAPTIC_I2C_ADDRESS      (HAPTIC_BASE + 0x01)            // 1 byte  - I2C address
#define HAPTIC_CALIBRATION_DONE (HAPTIC_BASE + 0x02)            // 1 byte  - Calibration complete flag
#define HAPTIC_ERROR_COUNT      (HAPTIC_BASE + 0x03)            // 2 bytes - Error count
#define HAPTIC_RESERVED         (HAPTIC_BASE + 0x05)            // 251 bytes - Reserved

/* System Reserved (0x0600 - 0x07FF) - 512B */
#define SYSTEM_RESERVED_BASE    (SYSTEM_DATA_BASE + 0x600)      // 512 bytes - Future system use

/* ========================================================================
 * ZONE PROGRAMMING SECTION (0x0800 - 0x2FFF) - 10KB
 * ========================================================================
 */

/* LED Zone Data (0x0800 - 0x27FF) - 8KB */
#define LED_ZONE_DATA_BASE      (ZONE_PROGRAMMING_BASE + 0x000)
#define ZONE_COUNT              (LED_ZONE_DATA_BASE + 0x00)     // 1 byte - Number of zones

/* Zone Definition Layout (per zone) */
#define ZONE_DEFINITION_SIZE    64
#define ZONE_LED_COUNT_OFFSET   0x00    // 1 byte  - LED count in zone
#define ZONE_LED_INDICES_OFFSET 0x01    // 32 bytes - LED indices array
#define ZONE_COLOR_OFFSET       0x21    // 3 bytes - RGB base color (red, green, blue)
#define ZONE_PATTERN_OFFSET     0x24    // 4 bytes - Pattern configuration (type, speed, intensity, direction)
#define ZONE_ENABLED_OFFSET     0x28    // 1 byte  - Zone enabled flag
#define ZONE_RESERVED_OFFSET    0x29    // 23 bytes - Reserved per zone

/* Zone Assignments (0x2800 - 0x29FF) - 512B */
#define ZONE_ASSIGNMENTS_BASE   (ZONE_PROGRAMMING_BASE + 0x2000)
#define KEY_TO_ZONE_MAP         (ZONE_ASSIGNMENTS_BASE + 0x00)  // 80 bytes - Key to zone mapping
#define ZONE_PRIORITY           (ZONE_ASSIGNMENTS_BASE + 0x50)  // 16 bytes - Zone priority array
#define GLOBAL_ZONE_ENABLED     (ZONE_ASSIGNMENTS_BASE + 0x60)  // 1 byte  - Global zones enable
#define ZONE_ASSIGN_RESERVED    (ZONE_ASSIGNMENTS_BASE + 0x61)  // 415 bytes - Reserved

/* Zone Effects (0x2A00 - 0x2BFF) - 512B */
#define ZONE_EFFECTS_BASE       (ZONE_PROGRAMMING_BASE + 0x2200)
#define FADE_TIME_MS            (ZONE_EFFECTS_BASE + 0x00)      // 2 bytes - Fade transition time
#define CROSS_FADE_ENABLED      (ZONE_EFFECTS_BASE + 0x02)      // 1 byte  - Cross fade enable
#define TRANSITION_CURVE        (ZONE_EFFECTS_BASE + 0x03)      // 1 byte  - Transition curve type
#define BREATHING_ENABLED       (ZONE_EFFECTS_BASE + 0x04)      // 1 byte  - Breathing effect enable
#define BREATHING_PERIOD_MS     (ZONE_EFFECTS_BASE + 0x05)      // 2 bytes - Breathing period
#define REACTIVE_ENABLED        (ZONE_EFFECTS_BASE + 0x07)      // 1 byte  - Reactive effect enable
#define REACTIVE_FADE_MS        (ZONE_EFFECTS_BASE + 0x08)      // 2 bytes - Reactive fade time
#define SYNC_ALL_ZONES          (ZONE_EFFECTS_BASE + 0x0A)      // 1 byte  - Sync all zones flag
#define ZONE_EFFECTS_RESERVED   (ZONE_EFFECTS_BASE + 0x0B)      // 501 bytes - Reserved

/* Programming State (0x2C00 - 0x2DFF) - 512B */
#define PROGRAMMING_STATE_BASE  (ZONE_PROGRAMMING_BASE + 0x2400)
#define CURRENT_EDIT_ZONE       (PROGRAMMING_STATE_BASE + 0x00) // 1 byte  - Currently editing zone
#define UNSAVED_CHANGES         (PROGRAMMING_STATE_BASE + 0x01) // 1 byte  - Unsaved changes flag
#define LAST_EDIT_TIMESTAMP     (PROGRAMMING_STATE_BASE + 0x02) // 4 bytes - Last edit timestamp
#define SESSION_BACKUP_VALID    (PROGRAMMING_STATE_BASE + 0x06) // 1 byte  - Session backup valid
#define PROG_STATE_RESERVED     (PROGRAMMING_STATE_BASE + 0x07) // 505 bytes - Reserved

/* Zone Reserved (0x2E00 - 0x2FFF) - 512B */
#define ZONE_RESERVED_BASE      (ZONE_PROGRAMMING_BASE + 0x2600) // 512 bytes - Future zone features

/* ========================================================================
 * USER CONFIGURATION SECTION (0x3000 - 0x37FF) - 2KB
 * ========================================================================
 */

/* Backlight Settings (0x3000 - 0x31FF) - 512B */
#define BACKLIGHT_SETTINGS_BASE (USER_CONFIG_BASE + 0x000)
#define GLOBAL_BRIGHTNESS       (BACKLIGHT_SETTINGS_BASE + 0x00) // 1 byte  - Global brightness
#define BRIGHTNESS_PRESETS      (BACKLIGHT_SETTINGS_BASE + 0x01) // 4 bytes - Brightness presets array
#define ACTIVE_PRESET           (BACKLIGHT_SETTINGS_BASE + 0x05) // 1 byte  - Active preset
#define AUTO_DIM_TIMEOUT_S      (BACKLIGHT_SETTINGS_BASE + 0x06) // 2 bytes - Auto dim timeout
#define AUTO_OFF_TIMEOUT_S      (BACKLIGHT_SETTINGS_BASE + 0x08) // 2 bytes - Auto off timeout
#define DIM_PERCENTAGE          (BACKLIGHT_SETTINGS_BASE + 0x0A) // 1 byte  - Dim percentage
#define WAKE_ON_KEYPRESS        (BACKLIGHT_SETTINGS_BASE + 0x0B) // 1 byte  - Wake on keypress
#define BATTERY_SAVE_MODE       (BACKLIGHT_SETTINGS_BASE + 0x0C) // 1 byte  - Battery save mode
#define BACKLIGHT_RESERVED      (BACKLIGHT_SETTINGS_BASE + 0x0D) // 499 bytes - Reserved

/* User Preferences (0x3200 - 0x33FF) - 512B */
#define USER_PREFS_BASE         (USER_CONFIG_BASE + 0x200)
#define ANIMATION_SPEED         (USER_PREFS_BASE + 0x00)        // 1 byte  - Animation speed
#define DISABLE_ANIMATIONS      (USER_PREFS_BASE + 0x01)        // 1 byte  - Disable animations
#define HIGH_CONTRAST_MODE      (USER_PREFS_BASE + 0x02)        // 1 byte  - High contrast mode
#define KEY_REPEAT_DELAY_MS     (USER_PREFS_BASE + 0x03)        // 2 bytes - Key repeat delay
#define KEY_REPEAT_RATE_HZ      (USER_PREFS_BASE + 0x05)        // 1 byte  - Key repeat rate
#define DEBOUNCE_TIME_MS        (USER_PREFS_BASE + 0x06)        // 1 byte  - Debounce time
#define CONFIG_TIMEOUT_S        (USER_PREFS_BASE + 0x07)        // 2 bytes - Config timeout
#define SLEEP_TIMEOUT_S         (USER_PREFS_BASE + 0x09)        // 2 bytes - Sleep timeout
#define USER_PREFS_RESERVED     (USER_PREFS_BASE + 0x0B)        // 501 bytes - Reserved

/* Custom Keybindings (0x3400 - 0x35FF) - 512B */
#define KEYBINDINGS_BASE        (USER_CONFIG_BASE + 0x400)
#define FUNCTION_KEYS_BASE      (KEYBINDINGS_BASE + 0x00)       // Function key actions (fn_f1_action, fn_f2_action, etc.)
#define MEDIA_CONTROLS_BASE     (KEYBINDINGS_BASE + 0x0C)       // Media control keys (volume_up_key, volume_down_key, play_pause_key)
#define LIGHTING_CONTROLS_BASE  (KEYBINDINGS_BASE + 0x10)       // Lighting control keys (brightness_up_key, brightness_down_key, effect_cycle_key)
#define KEYBINDINGS_RESERVED    (KEYBINDINGS_BASE + 0x14)       // Reserved

/* User Reserved (0x3600 - 0x37FF) - 512B */
#define USER_RESERVED_BASE      (USER_CONFIG_BASE + 0x600)      // 512 bytes - Future user features

/* ========================================================================
 * OPERATIONAL DATA SECTION (0x3800 - 0x47FF) - 4KB
 * ========================================================================
 */

/* Battery Health (0x3800 - 0x3BFF) - 1KB */
#define BATTERY_HEALTH_BASE     (OPERATIONAL_DATA_BASE + 0x000)
#define CHARGE_CYCLES           (BATTERY_HEALTH_BASE + 0x00)    // 4 bytes - Battery charge cycles
#define CAPACITY_PERCENTAGE     (BATTERY_HEALTH_BASE + 0x04)    // 1 byte  - Battery capacity %
#define VOLTAGE_FULL            (BATTERY_HEALTH_BASE + 0x05)    // 2 bytes - Full charge voltage
#define VOLTAGE_EMPTY           (BATTERY_HEALTH_BASE + 0x07)    // 2 bytes - Empty voltage
#define LAST_CALIBRATION        (BATTERY_HEALTH_BASE + 0x09)    // 4 bytes - Last calibration time
#define MAX_TEMP                (BATTERY_HEALTH_BASE + 0x0D)    // 1 byte  - Maximum temperature (int8_t)
#define MIN_TEMP                (BATTERY_HEALTH_BASE + 0x0E)    // 1 byte  - Minimum temperature (int8_t)
#define BATTERY_HEALTH_RESERVED (BATTERY_HEALTH_BASE + 0x0F)    // 1009 bytes - Reserved

/* Usage Statistics (0x3C00 - 0x3FFF) - 1KB */
#define USAGE_STATS_BASE        (OPERATIONAL_DATA_BASE + 0x400)
#define TOTAL_KEYSTROKES        (USAGE_STATS_BASE + 0x00)       // 8 bytes - Total keystrokes
#define TOTAL_UPTIME_HOURS      (USAGE_STATS_BASE + 0x08)       // 4 bytes - Total uptime
#define POWER_CYCLES            (USAGE_STATS_BASE + 0x0C)       // 4 bytes - Power cycles
#define MOST_USED_KEYS          (USAGE_STATS_BASE + 0x10)       // 10 bytes - Most used keys array
#define LIGHTING_ON_PERCENTAGE  (USAGE_STATS_BASE + 0x1A)       // 1 byte  - Lighting on percentage
#define CONFIG_MODE_ENTRIES     (USAGE_STATS_BASE + 0x1B)       // 4 bytes - Config mode entries
#define MEMORY_ERRORS           (USAGE_STATS_BASE + 0x1F)       // 2 bytes - Memory errors
#define COMM_ERRORS             (USAGE_STATS_BASE + 0x21)       // 2 bytes - Communication errors
#define USAGE_STATS_RESERVED    (USAGE_STATS_BASE + 0x23)       // 989 bytes - Reserved

/* Fault Logging (0x4000 - 0x43FF) - 1KB */
#define FAULT_LOGGING_BASE      (OPERATIONAL_DATA_BASE + 0x800)
#define TOTAL_ERRORS            (FAULT_LOGGING_BASE + 0x00)     // 4 bytes - Total error count
#define LAST_ERROR_CODE         (FAULT_LOGGING_BASE + 0x04)     // 2 bytes - Last error code
#define LAST_ERROR_TIMESTAMP    (FAULT_LOGGING_BASE + 0x06)     // 4 bytes - Last error timestamp
#define CRITICAL_ERROR_COUNT    (FAULT_LOGGING_BASE + 0x0A)     // 2 bytes - Critical errors
#define WATCHDOG_RESETS         (FAULT_LOGGING_BASE + 0x0C)     // 2 bytes - Watchdog resets
#define HARD_FAULT_COUNT        (FAULT_LOGGING_BASE + 0x0E)     // 2 bytes - Hard fault count
#define STACK_OVERFLOW_COUNT    (FAULT_LOGGING_BASE + 0x10)     // 2 bytes - Stack overflow count
#define FAULT_LOGGING_RESERVED  (FAULT_LOGGING_BASE + 0x12)     // 1006 bytes - Reserved

/* Operational Reserved (0x4400 - 0x47FF) - 1KB */
#define OPERATIONAL_RESERVED    (OPERATIONAL_DATA_BASE + 0xC00) // 1KB - Future operational data

/* ========================================================================
 * CRITICAL BACKUP SECTION (0x4800 - 0x4FFF) - 2KB
 * ========================================================================
 */

#define BACKUP_ZONE_DATA        (CRITICAL_BACKUP_BASE + 0x000)  // Backup zone data
#define BACKUP_USER_PREFS       (CRITICAL_BACKUP_BASE + 0x200)  // Backup user preferences
#define BACKUP_SYSTEM_HEADER    (CRITICAL_BACKUP_BASE + 0x400)  // Backup system header
#define BACKUP_VALIDATION_CRC   (CRITICAL_BACKUP_BASE + 0x500)  // 2 bytes - Backup validation CRC
#define BACKUP_RESERVED         (CRITICAL_BACKUP_BASE + 0x502)  // Backup reserved space

/* ========================================================================
 * HELPER MACROS
 * ========================================================================
 */

/* Zone address calculation macro */
#define ZONE_ADDRESS(zone_id, offset) \
    (LED_ZONE_DATA_BASE + 1 + ((zone_id) * ZONE_DEFINITION_SIZE) + (offset))

/* Key assignment address calculation macro */
#define KEY_ZONE_ADDRESS(key_id) \
    (KEY_TO_ZONE_MAP + (key_id))

/* Brightness preset address calculation */
#define BRIGHTNESS_PRESET_ADDRESS(preset_id) \
    (BRIGHTNESS_PRESETS + (preset_id))

/* Function key address calculation */
#define FUNCTION_KEY_ADDRESS(fn_key) \
    (FUNCTION_KEYS_BASE + (fn_key))

/* Validation macros */
#define IS_VALID_ZONE_ID(id)        ((id) < 16)
#define IS_VALID_KEY_ID(id)         ((id) < 80)
#define IS_VALID_PRESET_ID(id)      ((id) < 4)
#define IS_VALID_BRIGHTNESS(val)    ((val) <= 100)
#define IS_VALID_FN_KEY(id)         ((id) < 12)

#endif /* EEPROM_MEMORY_MAP_H_ */
