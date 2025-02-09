#include "i2c_core.h"
#include "logger.h"
#include "stm32l0xx_hal.h"

i2c_device i2c1_inst;

// Pin definitions
#define SCL_PIN GPIO_PIN_9
#define SDA_PIN GPIO_PIN_10
#define SCL_PORT GPIOA
#define SDA_PORT GPIOA

// I2C state machine states
typedef enum
{
	I2C_IDLE,
	I2C_START,
	I2C_ADDRESS,
	I2C_DATA_TX,
	I2C_DATA_RX,
	I2C_ACK,
	I2C_STOP
} i2c_state_t;

// Timer configuration for TIM6
#define TIM6_FREQ 100000  // 400kHz for Fast I2C mode
#define TIM6_PERIOD (SystemCoreClock / TIM6_FREQ - 1)

static volatile i2c_state_t current_state = I2C_IDLE;
static volatile bool half_cycle_complete = false;
static volatile uint8_t bit_counter = 0;
static volatile uint8_t current_byte = 0;
static volatile bool operation_complete = false;

static void init_timer6(void)
{
	// Enable TIM6 clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

	// Configure TIM6 for 400kHz operation
	TIM6->PSC = 0;  // No prescaler
	TIM6->ARR = TIM6_PERIOD;

	// Enable update interrupt
	TIM6->DIER |= TIM_DIER_UIE;

	// Enable timer interrupt in NVIC
	NVIC_SetPriority(TIM6_DAC_IRQn, 1);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);

	// Enable timer
	TIM6->CR1 |= TIM_CR1_CEN;
}

void TIM6_DAC_IRQHandler(void)
{
	if (TIM6->SR & TIM_SR_UIF)
	{
		TIM6->SR = ~TIM_SR_UIF;  // Clear interrupt flag
		half_cycle_complete = true;
	}
}

static void delay_half_cycle(void)
{
	half_cycle_complete = false;
	while (!half_cycle_complete)
	{
		// Just wait for the interrupt
	}
}

static void SCL_HIGH(void)
{
	SCL_PORT->BSRR = SCL_PIN;
	delay_half_cycle();
}

static void SCL_LOW(void)
{
	SCL_PORT->BRR = SCL_PIN;
	delay_half_cycle();
}

static void SDA_HIGH(void)
{
	SDA_PORT->BSRR = SDA_PIN;
	delay_half_cycle();
}

static void SDA_LOW(void)
{
	SDA_PORT->BRR = SDA_PIN;
	delay_half_cycle();
}

static uint8_t SDA_READ(void)
{
	return (SDA_PORT->IDR & SDA_PIN) ? 1 : 0;
}

static void i2c_start_condition(void)
{
	SDA_HIGH();
	SCL_HIGH();
	SDA_LOW();  // Start condition: SDA goes low while SCL is high
	SCL_LOW();
}

static void i2c_stop_condition(void)
{
	SDA_LOW();
	SCL_HIGH();
	SDA_HIGH();  // Stop condition: SDA goes high while SCL is high
}

static bool i2c_write_byte(uint8_t byte)
{
	for (int i = 7; i >= 0; i--)
	{
		if (byte & (1 << i))
		{
			SDA_HIGH();
		}
		else
		{
			SDA_LOW();
		}
		SCL_HIGH();
		SCL_LOW();
	}

	// Release SDA for ACK
	SDA_HIGH();
	SCL_HIGH();

	// Read ACK
	uint8_t ack = SDA_READ();
	SCL_LOW();

	return (ack == 0);  // ACK = 0, NACK = 1
}

static uint8_t i2c_read_byte(bool send_ack)
{
	uint8_t byte = 0;
	SDA_HIGH();  // Release SDA for reading

	for (int i = 7; i >= 0; i--)
	{
		SCL_HIGH();
		if (SDA_READ())
		{
			byte |= (1 << i);
		}
		SCL_LOW();
	}

	// Send ACK/NACK
	if (send_ack)
	{
		SDA_LOW();
	}
	else
	{
		SDA_HIGH();
	}
	SCL_HIGH();
	SCL_LOW();

	return byte;
}

bool initI2C1(void)
{
	// Enable GPIOA clock
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

	// Configure SCL and SDA pins as open-drain outputs
	GPIOA->MODER &= ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
	GPIOA->MODER |= (GPIO_MODER_MODE9_0 | GPIO_MODER_MODE10_0);  // Output mode
	GPIOA->OTYPER |= (GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_10);     // Open-drain
	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEED9_0 | GPIO_OSPEEDER_OSPEED10_0); // High speed
	GPIOA->PUPDR |= (GPIO_PUPDR_PUPD9_1|GPIO_PUPDR_PUPD10_1); // pull ups

	// Initialize timer for I2C timing
	init_timer6();

	// Set initial pin states
	SDA_HIGH();
	SCL_HIGH();

	return true;
}

bool transmit_I2C(i2c_device* device)
{
	if (device->tranfer.busy)
	{
		LOG_ERROR("I2C busy - transmission rejected");
		return false;
	}

	device->tranfer.busy = true;
	operation_complete = false;
	current_state = I2C_START;

	while (!operation_complete)
	{
		if (!half_cycle_complete)
			continue;

		half_cycle_complete = false;

		switch (current_state)
		{
		case I2C_START:
			i2c_start_condition();
			current_state = I2C_ADDRESS;
			bit_counter = 0;
			current_byte = (device->address << 1) & 0xFE;
			break;

		case I2C_ADDRESS:
			if (!i2c_write_byte(current_byte))
			{
				LOG_ERROR("Address 0x%02X NACK", current_byte);
				i2c_stop_condition();
				device->tranfer.busy = false;
				operation_complete = true;
				return false;
			}
			current_state = I2C_DATA_TX;
			bit_counter = 0;
			break;

		case I2C_DATA_TX:
			if (bit_counter < device->tranfer.tx_size)
			{
				if (!i2c_write_byte(device->tranfer.tx_buffer[bit_counter]))
				{
					LOG_ERROR("Data byte NACK: 0x%02X",
							device->tranfer.tx_buffer[bit_counter]);
					i2c_stop_condition();
					device->tranfer.busy = false;
					operation_complete = true;
					return false;
				}
				bit_counter++;
			}
			else
			{
				current_state = I2C_STOP;
			}
			break;

		case I2C_STOP:
			i2c_stop_condition();
			device->tranfer.busy = false;
			operation_complete = true;
			return true;

		default:
			LOG_ERROR("Invalid I2C state");
			device->tranfer.busy = false;
			operation_complete = true;
			return false;
		}
	}

	return true;
}

bool receive_I2C(i2c_device* device)
{
	if (device->tranfer.busy)
	{
		return false;
	}

	device->tranfer.busy = true;
	operation_complete = false;
	current_state = I2C_START;

	// Non-blocking state machine for I2C reception
	while (!operation_complete)
	{
		switch (current_state)
		{
		case I2C_START:
			i2c_start_condition();
			current_state = I2C_ADDRESS;
			bit_counter = 0;
			current_byte = (device->address << 1) | 0x01;
			break;

		case I2C_ADDRESS:
			if (!i2c_write_byte(current_byte))
			{
				i2c_stop_condition();
				device->tranfer.busy = false;
				operation_complete = true;
				return false;
			}
			current_state = I2C_DATA_RX;
			bit_counter = 0;
			break;

		case I2C_DATA_RX:
			if (bit_counter < device->tranfer.rx_size)
			{
				device->tranfer.rx_buffer[bit_counter] = i2c_read_byte(
						bit_counter < (device->tranfer.rx_size - 1));
				bit_counter++;
			}
			else
			{
				current_state = I2C_STOP;
			}
			break;

		case I2C_STOP:
			i2c_stop_condition();
			device->tranfer.busy = false;
			operation_complete = true;
			return true;

		default:
			device->tranfer.busy = false;
			operation_complete = true;
			return false;
		}

		__WFI();  // Wait for next timer interrupt
	}

	return true;
}

void disable_I2C1(void)
{
	// Disable timer
	TIM6->CR1 &= ~TIM_CR1_CEN;
	RCC->APB1ENR &= ~RCC_APB1ENR_TIM6EN;

	// Set pins to analog mode
	GPIOA->MODER |= (GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
}

void enable_I2C1(void)
{
	initI2C1();
}

bool scan_I2C_bus(void)
{
	LOG_DEBUG("Starting I2C bus scan...");
	uint8_t addresses_found = 0;

	for (uint8_t addr = 0x08; addr < 0x78; addr++)
	{
		i2c_start_condition();
		bool ack = i2c_write_byte((addr << 1) & 0xFE);
		i2c_stop_condition();

		if (ack)
		{
			LOG_DEBUG("Device found at address: 0x%02X", addr);
			addresses_found++;
		}
	}

	if (addresses_found == 0)
	{
		LOG_DEBUG("No I2C devices found");
		return false;
	}

	LOG_DEBUG("Found %d I2C device(s)", addresses_found);
	return true;
}
