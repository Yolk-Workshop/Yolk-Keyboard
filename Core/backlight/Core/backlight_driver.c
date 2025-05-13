#include "backlight_driver.h"
#include <string.h>
#include "logger.h"
#include "is31flxx.h"

#define EFFECT_UPDATE_INTERVAL_MS 50

static led_matrix_state open_state;
static led_matrix_state short_state;

// LED matrix layout (0xFF indicates no LED present)
static const uint8_t led_matrix[7][12] =
{
{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 },      // ROW1
		{ 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23 }, // ROW2
		{ 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35 }, // ROW3
		{ 36, 37, 38, 39, 40, 41, 42, 0xFF, 44, 45, 46, 47 }, // ROW4
		{ 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 0xFF, 59 }, // ROW5
		{ 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71 },   // ROW6
		{ 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83 }    // ROW7
};

typedef struct
{
	uint32_t last_update;
	uint8_t pulse_level;
	bool pulse_rising;
} effect_state_t;

static effect_state_t effect_state =
{ 0 };
static backlight_state_t bl_state;

bool checkLEDshortstate(void)
{
	uint8_t short_status[12];
	bool status = true;

	LOG_DEBUG("Short Status Matrix:");
	// Read all short detection registers (12 x CS lines)
	for (uint8_t i = 0; i < 12; i++)
	{
		status &= is31fl_read_reg(LED_CTRL_REG, 0x04 + i, &short_status[i]);
		if (!status)
		{
			LOG_ERROR("Failed to read short status register %d", i);
			return false;
		}

		while (atomic_load(&i2c1_inst.tranfer.busy))
		{
		}

		// Print the raw register value
		//LOG_DEBUG("CS%2d: 0x%02X", i, short_status[i]);

		// Update matrix state for this CS line
		for (uint8_t j = 0; j < 12; j++)
		{
			// Extract bit for each SW line
			bool is_shorted = (short_status[i] & (1 << j)) != 0;
			short_state.state[i][j] = is_shorted;
			if (is_shorted)
			{
				LOG_ERROR("Short detected at CS%d SW%d", i, j);
			}
		}
	}
	LOG_DEBUG("No Short detected");

	return status;
}

bool getLEDmatrixState(led_matrix_state *open_matrix,
		led_matrix_state *short_matrix)
{
	// Return current states
	if (open_matrix)
	{
		memcpy(open_matrix, &open_state, sizeof(led_matrix_state));
	}
	if (short_matrix)
	{
		memcpy(short_matrix, &short_state, sizeof(led_matrix_state));
	}
	return true;
}

bool init_backlight(void)
{
	// Initialize LED driver
	if (!Init_IS31FL3737())
	{
		LOG_ERROR("LED driver initialization failed");
		return false;
	}

	// Check for shorts
	led_matrix_state open_matrix, short_matrix;
	if (!checkLEDshortstate())
	{
		LOG_ERROR("LED short check failed");
		return false;
	}

	if (!getLEDmatrixState(&open_matrix, &short_matrix))
	{
		LOG_ERROR("Failed to get LED matrix state");
		return false;
	}

	// Set initial brightness
	bl_state.gcc_value = 0xFF;
	if (!setGlobalCurrent(bl_state.gcc_value))
	{
		LOG_ERROR("Failed to set initial brightness");
		return false;
	}

	setAllLEDsPWM(0xFF);

	return true;
}

bool set_backlight_brightness(uint8_t brightness)
{
	bool success = setGlobalCurrent(brightness);
	if (success)
	{
		bl_state.gcc_value = brightness;
	}
	return success;
}

bool set_led_brightness(uint8_t row, uint8_t col, uint8_t brightness)
{
	if (row >= 7 || col >= 12 || led_matrix[row][col] == 0xFF)
	{
		return false;
	}

	if (!bl_state.matrix[row][col].valid)
	{
		return false;
	}

	bool success = setLEDPWM(row, col, brightness);
	if (success)
	{
		bl_state.matrix[row][col].pwm_value = brightness;
	}
	return success;
}

void update_pulse_effect(void)
{
	uint32_t now = HAL_GetTick();
	if ((now - effect_state.last_update) >= EFFECT_UPDATE_INTERVAL_MS)
	{
		if (effect_state.pulse_rising)
		{
			effect_state.pulse_level =
					(effect_state.pulse_level + 16 > 255) ?
							255 : effect_state.pulse_level + 16;
			if (effect_state.pulse_level >= 255)
			{
				effect_state.pulse_rising = false;
			}
		}
		else
		{
			effect_state.pulse_level =
					(effect_state.pulse_level < 16) ?
							0 : effect_state.pulse_level - 16;
			if (effect_state.pulse_level == 0)
			{
				effect_state.pulse_rising = true;
			}
		}

		set_backlight_brightness(effect_state.pulse_level);
		effect_state.last_update = now;
	}
}
