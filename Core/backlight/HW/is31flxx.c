#include "is31flxx.h"
#include <stdbool.h>
#include <string.h>
#include "logger.h"

static bool cmd_write_locked = true;

static bool init_is31fl_transfer(void)
{
	i2c1_inst.address = 0x50;  // Default I2C address
	i2c1_inst.tranfer.tx_size = 0;
	i2c1_inst.tranfer.rx_size = 0;
	i2c1_inst.tranfer.tx_count = 0;
	i2c1_inst.tranfer.rx_count = 0;
	atomic_store(&i2c1_inst.tranfer.busy, false);
	return true;
}

static bool IS31FL_UNLOCK(void)
{
	if (i2c1_inst.tranfer.busy)
	{
		return false;
	}

	i2c1_inst.tranfer.tx_buffer[0] = CMD_LOCK_REG;
	i2c1_inst.tranfer.tx_buffer[1] = CMD_WRITE_EN;
	i2c1_inst.tranfer.tx_size = 2;

	LOG_DEBUG("Unlock attempt: CMD_LOCK_REG=0x%02X, CMD_WRITE_EN=0x%02X", CMD_LOCK_REG, CMD_WRITE_EN);
	bool state = transmit_I2C(&i2c1_inst);
	if (state)
	{

		cmd_write_locked = false;
		return true;
	}

	LOG_ERROR("I2C transmission failed during unlock");
	LOG_DEBUG("Failed after sending %d bytes", i2c1_inst.tranfer.tx_count);
	cmd_write_locked = true;
	return false;
}

bool is31fl_write_reg(uint8_t page, uint8_t reg, uint8_t data)
{
	if (i2c1_inst.tranfer.busy)
	{
		return false;
	}

	if (cmd_write_locked)
	{
		if (!IS31FL_UNLOCK())
			return false;
	}

	// Select page first
	i2c1_inst.tranfer.tx_buffer[0] = CMD_REG;
	i2c1_inst.tranfer.tx_buffer[1] = page;
	i2c1_inst.tranfer.tx_size = 2;

	if (!transmit_I2C(&i2c1_inst))
	{
		return false;
	}

	if (i2c1_inst.tranfer.busy)
	{

	}

	// Write data to register
	i2c1_inst.tranfer.tx_buffer[0] = reg;
	i2c1_inst.tranfer.tx_buffer[1] = data;
	i2c1_inst.tranfer.tx_size = 2;

	return transmit_I2C(&i2c1_inst);
}

bool is31fl_read_reg(uint8_t page, uint8_t reg, uint8_t *data)
{
	if (i2c1_inst.tranfer.busy)
	{
		return false;
	}

	// Select page first
	i2c1_inst.tranfer.tx_buffer[0] = CMD_REG;
	i2c1_inst.tranfer.tx_buffer[1] = page;
	i2c1_inst.tranfer.tx_size = 2;

	if (!transmit_I2C(&i2c1_inst))
	{
		return false;
	}

	while (atomic_load(&i2c1_inst.tranfer.busy))
	{
	}

	// Write register to read
	i2c1_inst.tranfer.tx_buffer[0] = reg;
	i2c1_inst.tranfer.tx_size = 1;

	if (!transmit_I2C(&i2c1_inst))
	{
	}

	if (i2c1_inst.tranfer.busy)
	{
	}

	// Read data
	i2c1_inst.tranfer.rx_size = 1;
	if (!receive_I2C(&i2c1_inst))
	{
		return false;
	}

	if (i2c1_inst.tranfer.busy)
	{
	}

	*data = i2c1_inst.tranfer.rx_buffer[0];
	return true;
}

bool Init_IS31FL3737(void)
{
	// Initialize transfer structure
	if (!init_is31fl_transfer())
	{
		LOG_ERROR("Failed to initialise ISSI I2C structure");
		return false;
	}

	IS31FL3737_STB_Enable();

	// Unlock the device
	if (cmd_write_locked)
	{
		if (!IS31FL_UNLOCK())
		{
			LOG_ERROR("Failed to unlock LED driver");
			return false;
		}
		LOG_DEBUG("LED driver unlocked");
	}
	// Configure device settings
	cfg_reg_t config =
	{ .SSD = SSD_NORM_OP, .B_EN = AUTO_BREATH_OFF, .OSD = CFG_OSD_OFF, .SYNC =
	CFG_SYNC_HIZ };

	return is31fl_write_reg(FUNC_REG, CONFIG_REGISTER, *(uint8_t*) &config);
}

bool set_LED_state(led_sw led_matrix_state)
{
	if (atomic_load(&i2c1_inst.tranfer.busy))
	{
		return false;
	}

	bool status = true;

	// Write on/off states
	status &= is31fl_write_reg(LED_CTRL_REG, 0x00,
			*(uint8_t*) &led_matrix_state.on_off[0]);
	status &= is31fl_write_reg(LED_CTRL_REG, 0x01,
			*(uint8_t*) &led_matrix_state.on_off[1]);

	return status;
}

bool enableLED_Matrix(led_sw led_matrix_state)
{
	// Set LED states first
	if (!set_LED_state(led_matrix_state))
	{
		return false;
	}

	// Enable normal operation
	cfg_reg_t config =
	{ .SSD = SSD_NORM_OP, .B_EN = AUTO_BREATH_OFF, .OSD = CFG_OSD_OFF, .SYNC =
	CFG_SYNC_HIZ };

	return is31fl_write_reg(FUNC_REG, CONFIG_REGISTER, *(uint8_t*) &config);
}

bool disableLED_Matrix(void)
{
	cfg_reg_t config =
	{ .SSD = SSD_SW_SD, .B_EN = AUTO_BREATH_OFF, .OSD = CFG_OSD_OFF, .SYNC =
	CFG_SYNC_HIZ };

	return is31fl_write_reg(FUNC_REG, CONFIG_REGISTER, *(uint8_t*) &config);
}

bool setLEDPWM(uint8_t row, uint8_t col, uint8_t pwm_value)
{
	if (row >= 12 || col >= 12)
		return false;

	uint8_t reg_addr = row * 12 + col;
	return is31fl_write_reg(LED_PWM_REG, reg_addr, pwm_value);
}

bool setAllLEDsPWM(uint8_t pwm_value)
{
	bool status = true;

	for (uint8_t i = 0; i < 144; i++)
	{
		status &= is31fl_write_reg(LED_PWM_REG, i, pwm_value);
		if (!status)
			return false;
	}

	return true;
}

bool getLEDPWM(uint8_t row, uint8_t col, uint8_t *pwm_value)
{
	if (row >= 12 || col >= 12)
		return false;

	uint8_t reg_addr = row * 12 + col;
	return is31fl_read_reg(LED_PWM_REG, reg_addr, pwm_value);
}

bool setGlobalCurrent(uint8_t gcc_value)
{
	return is31fl_write_reg(FUNC_REG, GGC_REGISTER, gcc_value);
}

bool setPullUpDownResistors(is31fl3737_PUR pur_value, is31fl3737_PDR pdr_value)
{
	bool status = true;
	status &= is31fl_write_reg(FUNC_REG, SWy_PUP_SEL_REG, pur_value);
	status &= is31fl_write_reg(FUNC_REG, CSx_PDWN_SEL_REG, pdr_value);
	return status;
}

bool configABM(void)
{
	bool status = true;

	// Example configuration for ABM1
	status &= is31fl_write_reg(FUNC_REG, ABM_1_FADE_IN, T1_0_84s);
	status &= is31fl_write_reg(FUNC_REG, ABM_1_FADE_OUT, T2_0_84s);
	status &= is31fl_write_reg(FUNC_REG, ABM_1_LOOP, 0);  // Infinite loop

	if (status)
	{
		// Enable ABM in configuration
		cfg_reg_t config =
		{ .SSD = SSD_NORM_OP, .B_EN = AUTO_BREATH_ON, .OSD = CFG_OSD_OFF,
				.SYNC = CFG_SYNC_HIZ };
		status &= is31fl_write_reg(FUNC_REG, CONFIG_REGISTER,
				*(uint8_t*) &config);
	}

	return status;
}

bool setDeviceInterrupts(is31fl3737_imr int_mask)
{
	return is31fl_write_reg(FUNC_REG, INT_MSK_REG, *(uint8_t*) &int_mask);
}

bool clearInterruptFlags(void)
{
	uint8_t dummy;
	return is31fl_read_reg(FUNC_REG, INT_STATUS_REG, &dummy);
}

bool getInterruptStatus(is31fl3737_isr *status)
{
	uint8_t int_status;
	if (!is31fl_read_reg(FUNC_REG, INT_STATUS_REG, &int_status))
	{
		return false;
	}
	*status = *(is31fl3737_isr*) &int_status;
	return true;
}

bool is31fl_write_page(uint8_t page, uint8_t start_reg, uint8_t *data,
		uint8_t len)
{
	if (atomic_load(&i2c1_inst.tranfer.busy))
	{
		return false;
	}

	// Select page first
	if (!is31fl_write_reg(CMD_REG, page, 0))
	{
		return false;
	}

	// Write data sequentially
	bool status = true;
	for (uint8_t i = 0; i < len; i++)
	{
		status &= is31fl_write_reg(start_reg + i, data[i], 0);
		if (!status)
			break;
	}

	return status;
}

void IS31FL3737_STB_Enable(void)
{
	GPIOD->BSRR = GPIO_PIN_10;

}

void IS31FL3737_STB_Exit(void)
{
	GPIOD->BRR = GPIO_PIN_10;
}
