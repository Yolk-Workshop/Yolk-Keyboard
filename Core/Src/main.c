/*
 * main.c
 *
 * Created on: Jan 7, 2025
 * Author: bettysidepiece
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <assert.h>
#include <stdatomic.h>
#include "bm71_config.h"
#include "config_ui.h"

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void Init_TIM3(void);
static void Init_TIM21(void);
static void initWatchdog(void);
static void Backlight_Processes(void);
static void BLE_Process(void);
static void KB_HID_Processes(void);
static void System_Init_Sequence(void);

/* Private variables --------------------------------------------------------*/
// Global state flags
volatile uint8_t scan_flag = 0;
volatile uint8_t report_ready_flag = 0;
volatile ITStatus uart2_tc_flag = SET;
volatile ITStatus sys_ready = RESET;
volatile ITStatus pmsm_update_flag = SET;
volatile uint8_t startup_complete = 0;
volatile bool bm7x_timer_flag = false;


// Hardware interface handlers
TIM_HandleTypeDef htim2; //Haptic
TIM_HandleTypeDef htim3; //Scan Interrupt
TIM_HandleTypeDef htim21; //Non blocking timer
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_usart2_tx;
//XXX DMA_HandleTypeDef hdma_lpuart1_rx;
//XXX DMA_HandleTypeDef hdma_lpuart1_tx;


extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t dma_rx_buffer[128];
extern volatile connection_mode_t g_connection_mode;
extern volatile bool ble_initialized;
extern bm70_handle_t g_bm70;
extern mode_switch_state_t switch_state;
extern uint32_t switch_start_time;

static log_buffer_t log_queue = { 0 };
void logger_output(const char *message);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	System_Init_Sequence();

	while (1)
	{
		//check_connection();
		//if(switch_state == MODE_SWITCH_IN_PROGRESS) continue; XXX

		Backlight_Processes();
		KB_HID_Processes();
		BLE_Process();
		PM_Update();
	}
}

static void Backlight_Processes(void){
	Effects_Process();
	config_ui_process();
	check_config_ui_functions();
}

static void KB_HID_Processes(void){

	if (scan_flag) {
		scanKeyMatrix();
		if (report_ready_flag == SET && !config_ui_is_active()) {
			reportArbiter();
			report_ready_flag = 0;
		}

		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		scan_flag = 0;
		__set_PRIMASK(primask);
	}

}

static void BLE_Process(void){

	if (g_connection_mode == CONNECTION_BLE) {
		static uint32_t last_main_rx_check = 0;
		uint32_t current_time = HAL_GetTick();

		if (current_time - last_main_rx_check > 50) {
			bm70_process_rx(&g_bm70);
			last_main_rx_check = current_time;
		}

		if (bm7x_timer_flag) {
			bm7x_timer_flag = false;

			uint32_t ble_start = HAL_GetTick();
			ble_periodic_status_check();
			uint32_t ble_duration = HAL_GetTick() - ble_start;

			if (ble_duration > 500) {
				LOG_WARNING("BLE operation took %lu ms", ble_duration);
			}
		}
	}

	check_ble_key_functions();
}

static void initWatchdog(void)
{
    // Disable watchdog during debugging sessions
    WDG_DisableInDebug();
    // Initialize watchdog with 4-second timeout
    WDG_Init(WDG_TIMEOUT);
    LOG_INFO("Watchdog initialized with %d second timeout", WDG_TIMEOUT/1000);
}

static void System_Init_Sequence(void){
	/* MCU Configuration--------------------------------------------------------*/
  	HAL_Init();
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	LOG_INFO("System Reset");
	LOG_INFO("Core System Hardware Initialised");

	MX_I2C2_Init();
	LOG_INFO("I2C Peripheral Initialised");

	/* Initialize Keyboard Hardware */
	initKeyboard();
	resetRows();

	/* Initialize Timers */
	MX_TIM2_Init();
	Init_TIM21();
	Init_TIM3();
	LOG_INFO("TIMER 2, 3 & 21 Initialised");
	HAL_Delay(100);

	HAL_TIM_Base_Start(&htim21);
	LOG_DEBUG("Timer 21 Started");
	HAL_Delay(100);

	HAL_TIM_Base_Start_IT(&htim3);
	LOG_DEBUG("Timer 3 Interrupt Started");

	Backlight_Init();
	config_ui_init();
	if (Effects_Init() != EFFECT_ERROR_NONE) {
		LOG_ERROR("Effects system initialization failed");
	} else {
		LOG_INFO("Effects system initialized");

		// Start beautiful power-on fade effect
		Effects_StartupSequence(NULL);
		LOG_INFO("Power-on fade effect started");
	}

	PM_Init();
	/* Initialize USB and Bluetooth */
	if (g_connection_mode == CONNECTION_BLE) {
		initBluetooth();
	}
	else {
		MX_USB_DEVICE_Init();
	}

	initWatchdog();
	startup_complete = 1;
	LOG_INFO("System ready and running");
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit =
	{ 0 };

	/* Configure the main internal regulator output voltage */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Initialize RCC Oscillators */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_8;
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/* Initialize CPU, AHB and APB buses clocks */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}

	/* Configure peripheral clock sources */
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_LPUART1 | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_USB;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/* Timer Initialization Functions --------------------------------------------*/
static void Init_TIM21(void)
{
	__HAL_RCC_TIM21_CLK_ENABLE();

	htim21.Instance = TIM21;
	htim21.Init.Prescaler = (HAL_RCC_GetHCLKFreq() / 1000000) - 1; // 1 µs resolution
	htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim21.Init.Period = 0xFFFF;
	htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
	{
		Error_Handler();
	}
}

void tim7_ble_init(void)
{
    // Enable TIM7 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

    // Wait for clock to stabilize
    __DSB();

    // Reset TIM7
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM7RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM7RST;

    // Configure TIM7 for 2Hz (every 500ms) - good balance for BLE status checking
    // Assuming 32MHz system clock
    TIM7->PSC = 31999;   // Prescaler: 32MHz / 32000 = 1kHz
    TIM7->ARR = 1999;     // Auto-reload: 1kHz / 500 = 2Hz (500ms)

    // Enable update interrupt
    TIM7->DIER |= TIM_DIER_UIE;

    // Configure NVIC - Priority 2 (lower than critical keyboard scanning)
    NVIC_SetPriority(TIM7_IRQn, 2);
    NVIC_EnableIRQ(TIM7_IRQn);

    // Generate an update event to load the prescaler
    TIM7->EGR |= TIM_EGR_UG;

    // Clear any pending interrupt flags
    TIM7->SR = 0;

    // Start timer
    TIM7->CR1 |= TIM_CR1_CEN;

    LOG_INFO("Timer 7 initialized for BLE status checking (500ms interval)");
}

void tim7_ble_stop(void)
{
    TIM7->CR1 &= ~TIM_CR1_CEN;
    TIM7->DIER &= ~TIM_DIER_UIE;
    NVIC_DisableIRQ(TIM7_IRQn);
    RCC->APB1ENR &= ~RCC_APB1ENR_TIM7EN;

    LOG_INFO("Timer 7 stopped");
}





static void Init_TIM3(void)
{
	__HAL_RCC_TIM3_CLK_ENABLE();

	TIM_ClockConfigTypeDef sClockSourceConfig =
	{ 0 };
	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 32000 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	HAL_TIM_Base_Init(&htim3);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

	// Set high priority interrupt
	NVIC_SetPriority(TIM3_IRQn, 0);
	NVIC_EnableIRQ(TIM3_IRQn);
}

static void MX_TIM2_Init(void)
{
	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };
	TIM_OC_InitTypeDef sConfigOC =
	{ 0 };

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim2);
}

/* UART/USART Initialization Functions -------------------------------------*/
void MX_LPUART1_UART_Init(void)
{
    hlpuart1.Instance = LPUART1;
    hlpuart1.Init.BaudRate = 115200;
    hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
    hlpuart1.Init.StopBits = UART_STOPBITS_1;
    hlpuart1.Init.Parity = UART_PARITY_NONE;
    hlpuart1.Init.Mode = UART_MODE_TX_RX;
    hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&hlpuart1) != HAL_OK)
    {
        Error_Handler();
    }

}

static void MX_USART2_UART_Init(void)
{
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 230400;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
}

/* I2C Initialization Functions -------------------------------------------*/

static void MX_I2C2_Init(void)
{
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x00B07CB4;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
	{
		Error_Handler();
	}
}

/* DMA Initialization Function --------------------------------------------*/
static void MX_DMA_Init(void)
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1);
	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

	NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 2);
	NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
}

/* GPIO Initialization Function ------------------------------------------*/
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Enable GPIO Ports Clock */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/* Configure Default Output Levels */
	HAL_GPIO_WritePin(LMAT_RESET_GPIO_Port, LMAT_RESET_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LMAT_SDB_GPIO_Port, LMAT_SDB_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, EEPROM_WC_Pin | BATT_CHRG_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, R0_Pin | R1_Pin | R2_Pin | R3_Pin | R4_Pin | R5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HAPTIC_PWM_EN_GPIO_Port, HAPTIC_PWM_EN_Pin | HAPTIC_EN_PWM_Pin, GPIO_PIN_RESET);

	/* Configure Column Pins (C8-C13) on GPIOE */
	GPIO_InitStruct.Pin = C8_Pin | C9_Pin | C10_Pin | C11_Pin | C12_Pin | C13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/* Column Pin C0 */
	GPIO_InitStruct.Pin = C0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(C0_GPIO_Port, &GPIO_InitStruct);

	/* Column Pins C1-C7 */
	GPIO_InitStruct.Pin = C1_Pin | C2_Pin | C3_Pin | C4_Pin | C5_Pin | C6_Pin | C7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Row Pins */
	GPIO_InitStruct.Pin = R0_Pin | R1_Pin | R2_Pin | R3_Pin | R4_Pin | R5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* Interrupt Pins */
	GPIO_InitStruct.Pin = TOUCH_INT_Pin | BMS_ALERT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = PROX_SENSE_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(PROX_SENSE_INT_GPIO_Port, &GPIO_InitStruct);

	/* Version Detection Pins */
	GPIO_InitStruct.Pin = HAPTIC_BRD_VERSION_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(HAPTIC_BRD_VERSION_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = KB_BRD_VERSION_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(KB_BRD_VERSION_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = PS_BRD_VERSION_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(PS_BRD_VERSION_Port, &GPIO_InitStruct);

	/* LMAT Control Pins */
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LMAT_RESET_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LMAT_SDB_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(LMAT_SDB_GPIO_Port, &GPIO_InitStruct);

	/* Bluetooth Status Pins */
	GPIO_InitStruct.Pin = BLE_STAT1_Pin | BLE_STAT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	//RX_IND_Pin
	GPIO_InitStruct.Pin = BLE_WAKEUP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(BT_WAKEUP_GPIO_Port, &GPIO_InitStruct);

	/* BT_RESET */
	GPIO_InitStruct.Pin = BT_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BT_RESET_GPIO_Port, &GPIO_InitStruct);


	/* UART Mode Switch (BM70 P2_0) */
	GPIO_InitStruct.Pin = UART_MODE_SW_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(UART_MODE_SW_GPIO_Port, &GPIO_InitStruct);

	#if BLE_CONFIG
		// Programming/config mode: Pull P2_0 LOW
		UART_MODE_SW_GPIO_Port->BRR = UART_MODE_SW_Pin;   // Assert LOW
	#else
		// Application/run mode: Pull P2_0 HIGH
		UART_MODE_SW_GPIO_Port->BSRR = UART_MODE_SW_Pin;  // Assert HIGH
	#endif
	/* BLE TX Alert */
	GPIO_InitStruct.Pin = BLE_TXALERT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BT_TXALERT_GPIO_Port, &GPIO_InitStruct);

	/* Battery Management */
	GPIO_InitStruct.Pin = BATT_PG_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BATT_PG_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = BATT_STAT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BATT_STAT_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = BATT_CHRG_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BATT_CHRG_EN_GPIO_Port, &GPIO_InitStruct);

	/* USB Detect */
	GPIO_InitStruct.Pin = VBUS_DETECT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(VBUS_DETECT_GPIO_Port, &GPIO_InitStruct);

	/* EEPROM + UART Switch */
	GPIO_InitStruct.Pin = UART_MODE_SW_Pin | EEPROM_WC_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* Haptic Pins */
	GPIO_InitStruct.Pin = HAPTIC_PWM_EN_Pin | HAPTIC_EN_PWM_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(HAPTIC_PWM_EN_GPIO_Port, &GPIO_InitStruct);

	/* Interrupt Priorities (disabled initially) */
	NVIC_SetPriority(EXTI0_1_IRQn, 5);
	NVIC_SetPriority(EXTI4_15_IRQn, 5);

	/* Set unused pins to analog */
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;

	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_8 |
						  GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_15;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_12;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
						  GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
						  GPIO_PIN_14 | GPIO_PIN_15;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_11 |
						  GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_All;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
}


void logger_output(const char *message)
{
	uint32_t primask;
	uint16_t len = strlen(message);

	primask = __get_PRIMASK();
	__disable_irq();

	// Check if we have space in the queue
	if (log_queue.buffer_count < LOG_BUFFER_COUNT)
	{
		// Copy message to next available buffer
		strncpy(log_queue.buffers[log_queue.write_index], message,
		LOG_BUFFER_SIZE - 1);
		log_queue.buffers[log_queue.write_index][LOG_BUFFER_SIZE - 1] = '\0';

		log_queue.write_index = (log_queue.write_index + 1) % LOG_BUFFER_COUNT;
		log_queue.buffer_count++;

		// If DMA not busy, start transmission
		if (!log_queue.dma_busy && uart2_tc_flag == SET)
		{
			len = strlen(log_queue.buffers[log_queue.read_index]);
			if (HAL_UART_Transmit_DMA(&huart2,
					(uint8_t*) log_queue.buffers[log_queue.read_index], len)
					== HAL_OK)
			{
				log_queue.dma_busy = 1;
				uart2_tc_flag = RESET;
			}
		}
	}

	__set_PRIMASK(primask);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2)
	{
		uart2_tc_flag = SET;
		log_queue.dma_busy = 0;
		uint16_t len;

		// Critical section
		__disable_irq();

		if (log_queue.buffer_count > 0)
		{
			log_queue.buffer_count--;
			log_queue.read_index =
					(log_queue.read_index + 1) % LOG_BUFFER_COUNT;

			if (log_queue.buffer_count > 0)
			{
				len = strlen(log_queue.buffers[log_queue.read_index]);
				if (HAL_UART_Transmit_DMA(&huart2,
						(uint8_t*) log_queue.buffers[log_queue.read_index], len)
						== HAL_OK)
				{
					log_queue.dma_busy = 1;
					uart2_tc_flag = RESET;
				}
			}
		}
		__enable_irq();

	}
	else if (huart == &hlpuart1)
	{

	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &hlpuart1)
	{
		rx_irq_handler();
	}
}
/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	LOG_ERROR("error occurred!\n");
	while (1)
	{

	}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add implementation to report the file name and line number */
    LOG_ERROR("Wrong parameters value: file %s on line %d\r\n", file, line);
}
#endif
