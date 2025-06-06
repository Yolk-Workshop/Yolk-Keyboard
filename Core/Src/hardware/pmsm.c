/**
 * @file    pmsm.c
 * @brief   Power Management State Machine Implementation with STOP Mode
 * @author  Yolk Workshop
 * @version 6.1
 * @date    2025
 */

#include "pmsm.h"
#include "logger.h"
#include "watchdog.h"
#include "usbd_core.h"
#include "kb_ble_api.h"

extern bm70_handle_t g_bm70;
extern volatile uint8_t ble_conn_flag;

// Constants for battery check and IWDG feeding
#define BATTERY_CHECK_PERIOD 45000      // Check battery every minute
#define IWDG_FEED_PERIOD_MS  1500       // Feed IWDG every 1.5 seconds (safety margin for 4s timeout)
#define LSI_CLOCK_FREQ       40000      // LSI frequency (typical) - matches watchdog.c

// Global variables
volatile uint32_t last_activity_time = 0;
volatile pm_state_t current_pm_state = PM_STATE_ACTIVE;
volatile connection_mode_t g_connection_mode = CONNECTION_USB;
static uint8_t battery_level = 100;
static bool lpm_flag = false;

// Debugging flag for power management events
#ifdef DEBUG_POWER
volatile uint8_t dbg_pm_events = 0;
#endif

// External variables and functions
extern volatile uint8_t scan_flag;
extern volatile uint8_t startup_complete;
extern void resetRows(void);
extern keyboard_state_t kb_state;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern bm70_handle_t g_bm70;
extern uint8_t pmsm_mode;

// External row/column definitions from kb_driver.c
extern const uint16_t row_pins[KEY_ROWS];
extern GPIO_TypeDef *const row_ports[KEY_ROWS];
extern GPIO_TypeDef *const col_ports[KEY_COLS];
extern const uint16_t col_pins[KEY_COLS];

// Function to configure LPTIM for IWDG feeding in STOP mode
static void PM_ConfigureLPTIM(void) {
    // Enable LPTIM clock
    RCC->APB1ENR |= RCC_APB1ENR_LPTIM1EN;

    // Configure LPTIM to use LSI clock source
    RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_LPTIM1SEL) | (0x01 << RCC_CCIPR_LPTIM1SEL_Pos);

    // Enable LSI if not already enabled
    if (!(RCC->CSR & RCC_CSR_LSION)) {
        RCC->CSR |= RCC_CSR_LSION;
        // Wait for LSI to be ready
        while (!(RCC->CSR & RCC_CSR_LSIRDY));
    }

    // Configure LPTIM
    LPTIM1->CFGR = 0;  // Reset configuration
    LPTIM1->CFGR |= LPTIM_CFGR_PRESC_2 | LPTIM_CFGR_PRESC_1 | LPTIM_CFGR_PRESC_0; // Prescaler /128

    // Enable LPTIM
    LPTIM1->CR |= LPTIM_CR_ENABLE;

    // Configure interrupt
    NVIC_SetPriority(LPTIM1_IRQn, 1);
    NVIC_EnableIRQ(LPTIM1_IRQn);
}

// Modified EXTI row functions
static void EXTI_Rows_Init(void) {
    // Enable GPIO clocks and SYSCFG
    RCC->IOPENR |= RCC_IOPENR_GPIODEN;  // All rows are on GPIOD

    // Enable SYSCFG clock for EXTI configuration
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Clear all EXTI configurations first
    SYSCFG->EXTICR[0] = 0;
    SYSCFG->EXTICR[1] = 0;

    // Configure EXTI lines for each row:
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PD | SYSCFG_EXTICR1_EXTI2_PD | SYSCFG_EXTICR1_EXTI3_PD;
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PD | SYSCFG_EXTICR2_EXTI5_PD | SYSCFG_EXTICR2_EXTI6_PD;
}

static inline void EXTI_Rows_Enable(void) {
    const uint32_t mask = 0x007E;  // Lines 1-6 (binary: 0111 1110)

    // Configure falling edge trigger for keyboard matrix
    EXTI->FTSR |= mask;
    EXTI->RTSR &= ~mask;

    // Clear pending flags
    EXTI->PR = mask;

    // Enable interrupt mask
    EXTI->IMR |= mask;

    // Enable NVIC interrupts
    NVIC_EnableIRQ(EXTI0_1_IRQn);    // For EXTI1 (R0)
    NVIC_SetPriority(EXTI0_1_IRQn, 0);
    NVIC_EnableIRQ(EXTI2_3_IRQn);    // For EXTI2-3 (R1-R2)
    NVIC_SetPriority(EXTI2_3_IRQn, 0);
    NVIC_EnableIRQ(EXTI4_15_IRQn);   // For EXTI4-6 (R3-R5)
    NVIC_SetPriority(EXTI4_15_IRQn, 0);
}

static inline void EXTI_Rows_Disable(void) {
    const uint32_t mask = 0x007E;  // Lines 1-6

    // Disable interrupt mask
    EXTI->IMR &= ~mask;

    // Clear pending flags
    EXTI->PR = mask;

    // Disable NVIC interrupts
    NVIC_DisableIRQ(EXTI0_1_IRQn);
    NVIC_DisableIRQ(EXTI2_3_IRQn);
    NVIC_DisableIRQ(EXTI4_15_IRQn);
}


// 2. I2C-only peripheral management (simplified for LED drivers)
static void PM_SuspendI2C(void) {

}

static void PM_ResumeI2C(void) {

}

/**
 * @brief Initialize the power management system
 * @retval None
 */
void PM_Init(void) {
    // Record initial time
    last_activity_time = HAL_GetTick();
    current_pm_state = PM_STATE_ACTIVE;

    // Check if USB is connected at startup
    if (!(VBUS_DETECT_GPIO_Port->IDR & VBUS_DETECT_Pin)) {  //XXX VBUS detect
        g_connection_mode = CONNECTION_USB;
        kb_state.output_mode = OUTPUT_USB;
        LOG_INFO("Power management initialized in USB mode");
    } else {
        g_connection_mode = CONNECTION_BLE;
        kb_state.output_mode = OUTPUT_BLE;
        LOG_INFO("Power management initialized in BLE mode");
    }

    // Initialize battery level
    g_bm70.hid_state.battery_level = PM_GetBatteryLevel();
    pmsm_mode = 0;

    // Initialize EXTI for column detection
    EXTI_Rows_Init();

    // Initialize LPTIM for IWDG feeding in STOP mode
    PM_ConfigureLPTIM();

    LOG_INFO("Power management initialized - Battery: %d%%", battery_level);
}

/**
 * @brief Enter STOP mode with IWDG feeding support
 * @retval None
 */
static void PM_EnterStopMode(void) {
    // Suspend USB if connected
    if (g_connection_mode == CONNECTION_USB) {
    	USBD_Stop(&hUsbDeviceFS);
    }

    PM_SuspendI2C();

    // Configure STOPWUCK to use HSI16 for faster wakeup
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_STOPWUCK) | RCC_CFGR_STOPWUCK;

    // Configure wake-up from STOP mode
    PM_ConfigureKeyEXTI(true);

	NVIC->ICPR[0] = 0xFFFFFFFF;
	EXTI->PR = 0xFFFFFFFF;

	// Clear LPUART flags and flush RX buffer
	LPUART1->ICR = USART_ICR_WUCF | USART_ICR_IDLECF | USART_ICR_ORECF |
				   USART_ICR_NECF | USART_ICR_FECF | USART_ICR_PECF;

	// Read and discard any pending LPUART data
	while (LPUART1->ISR & USART_ISR_RXNE) {
		volatile uint8_t dummy = LPUART1->RDR;
		(void)dummy;
	}

    // Calculate timer period for IWDG feeding (LSI/128)
    uint32_t lptim_period = (LSI_CLOCK_FREQ / 128) * IWDG_FEED_PERIOD_MS / 1000;
    if (lptim_period > 0xFFFF) lptim_period = 0xFFFF;

    // Start LPTIM for periodic IWDG feeding
    LPTIM1->ICR = LPTIM_ICR_ARRMCF;  // Clear any pending interrupts
    LPTIM1->IER |= LPTIM_IER_ARRMIE; // Enable auto-reload match interrupt
    LPTIM1->ARR = lptim_period;      // Set auto-reload value
    LPTIM1->CR |= LPTIM_CR_CNTSTRT;  // Start counter

    // Stop TIM3 (scanning timer)
    TIM3->CR1 &= ~TIM_CR1_CEN;

    // Disable systick interrupt during STOP
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;

    // Enter STOP mode with regulator in main mode
    PWR->CR = (PWR->CR & ~PWR_CR_LPSDSR) | PWR_CR_CWUF;
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    __DSB();
    __WFI();

    // Execution resumes here after wake-up
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    // Re-enable systick
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
}

/**
 * @brief Exit STOP mode and restore system
 * @retval None
 */
static void PM_ExitStopMode(void) {
    // Stop LPTIM
    LPTIM1->CR &= ~LPTIM_CR_CNTSTRT;
    LPTIM1->IER &= ~LPTIM_IER_ARRMIE;

    // Restart system clock to PLL (32 MHz)
    SystemClock_Config();

    // Restart TIM3 (scanning timer)
    TIM3->CR1 |= TIM_CR1_CEN;

    // Resume I2C peripherals
	PM_ResumeI2C();

    // Resume USB if connected
    if (g_connection_mode == CONNECTION_USB) {
    	USBD_Start(&hUsbDeviceFS);
    }

    // Disable EXTI for normal operation
    PM_ConfigureKeyEXTI(false);
}



/**
 * @brief Handle state transitions
 * @param new_state The state to enter
 * @retval None
 */
void PM_EnterState(pm_state_t new_state) {
    if (new_state == current_pm_state) {
        return;
    }

    if (new_state == PM_STATE_ACTIVE) {
        if (current_pm_state == PM_STATE_STOP) {
        	pmsm_mode = 0; // Active mode
            PM_ExitStopMode();
        }
        current_pm_state = PM_STATE_ACTIVE;
    }
    else if (new_state == PM_STATE_STOP) {
        current_pm_state = PM_STATE_STOP;
        pmsm_mode = 1; // Sleep mode
        PM_EnterStopMode();
        PM_EnterState(PM_STATE_ACTIVE);
    }

    // Always refresh the watchdog on any transition
    WDG_Refresh();
}

/**
 * @brief Record user activity to reset timeouts
 * @retval None
 */
void PM_RecordActivity(void) {
    // Update activity timestamp
    last_activity_time = HAL_GetTick();

    // If in STOP state, force return to ACTIVE
    if (current_pm_state != PM_STATE_ACTIVE) {
        PM_EnterState(PM_STATE_ACTIVE);
    }
}

/**
 * @brief Handle key interrupt for wakeup
 * @param GPIO_Pin The pin that triggered the interrupt
 * @retval None
 */
void PM_HandleWakeup(void) {
    #ifdef DEBUG_POWER
    dbg_pm_events++;
    #endif
    WDG_Refresh();
    // Record activity to reset timeouts
    last_activity_time = HAL_GetTick();

    // Set scan flag for matrix scanning
    scan_flag = 1;
    if (g_connection_mode == CONNECTION_BLE && g_bm70.initialized) {
		lpm_flag = true;
	}

    // The MCU has already woken from STOP mode when this interrupt fires
    // Force active state if we were in STOP
    if (current_pm_state == PM_STATE_STOP) {
        current_pm_state = PM_STATE_ACTIVE;
        PM_ExitStopMode();
    }
}

/**
 * @brief Update power management state based on activity timeout
 * Called periodically from main loop
 * @retval None
 */
void PM_Update(void) {
    static uint32_t last_battery_check = 0;
    uint32_t current_time = HAL_GetTick();

    // Check battery level periodically
    if (current_time - last_battery_check > BATTERY_CHECK_PERIOD) {
        battery_level = PM_GetBatteryLevel();
        last_battery_check = current_time;
    }

    // Calculate idle time
    uint32_t idle_time = current_time - last_activity_time;

    // State transition logic based on idle time
    if (idle_time > PM_IDLE_TIMEOUT) {
        if (current_pm_state == PM_STATE_ACTIVE) {
            PM_EnterState(PM_STATE_STOP);
        }
    }

    if (lpm_flag){
    	bm70_error_t err = bm70_read_device_name(&g_bm70);
		if (err != BM70_OK) {

			// Hardware says disconnected - sync software state
			ble_conn_flag = false;
			g_bm70.conn_state = BM70_CONN_STATE_DISCONNECTED;
			g_bm70.status = BM70_STATUS_IDLE;// Also sync status
			memset(&g_bm70.conn_info, 0, sizeof(g_bm70.conn_info));
			g_bm70.hid_state.service_ready = false;
			g_bm70.hid_state.notifications_enabled = false;

			// Start advertising
			LOG_INFO("Connection lost during sleep - restarting advertising");
			bm70_start_advertising(&g_bm70);
		}

		lpm_flag = false;
    }

    // Always refresh watchdog in active mode
    if (current_pm_state == PM_STATE_ACTIVE) {
        WDG_Refresh();
    }
}

/**
 * @brief Configure key EXTI for wakeup
 * @param enable Enable or disable EXTI for keys
 * @retval None
 */
void PM_ConfigureKeyEXTI(bool enable) {
    if (enable) {
        // First, configure all rows as inputs with pull-up
        for (uint8_t row = 0; row < KEY_ROWS; row++) {
            uint32_t pin = row_pins[row];
            GPIO_TypeDef* port = row_ports[row];
            uint32_t pin_pos = 0;

            // Find pin position
            while ((pin >> pin_pos) != 1) pin_pos++;

            // Configure as input (MODE = 00)
            port->MODER &= ~(3UL << (pin_pos * 2));

            // Configure pull-up (PUPDR = 01)
            port->PUPDR &= ~(3UL << (pin_pos * 2));
            port->PUPDR |= (1UL << (pin_pos * 2));

            // Ensure no output driver is active
            port->OTYPER &= ~(1UL << pin_pos);
        }

        // Configure all columns as outputs driven low
        for (uint8_t col = 0; col < KEY_COLS; col++) {
            uint32_t pin = col_pins[col];
            GPIO_TypeDef* port = col_ports[col];
            uint32_t pin_pos = 0;

            // Find pin position
            while ((pin >> pin_pos) != 1) pin_pos++;

            // Configure as output (MODE = 01)
            port->MODER &= ~(3UL << (pin_pos * 2));
            port->MODER |= (1UL << (pin_pos * 2));

            // Configure as push-pull (OTYPER = 0)
            port->OTYPER &= ~(1UL << pin_pos);

            // Drive low
            port->ODR &= ~pin;
        }

        // Wait for signals to stabilize
        delay_us(100);

        // Enable EXTI for rows
        EXTI_Rows_Enable();

    } else {
        // Disable EXTI first
        EXTI_Rows_Disable();

        // Configure rows back to outputs
        for (uint8_t row = 0; row < KEY_ROWS; row++) {
            uint32_t pin = row_pins[row];
            GPIO_TypeDef* port = row_ports[row];
            uint32_t pin_pos = 0;

            // Find pin position
            while ((pin >> pin_pos) != 1) pin_pos++;

            // Configure as output (MODE = 01)
            port->MODER &= ~(3UL << (pin_pos * 2));
            port->MODER |= (1UL << (pin_pos * 2));

            // Configure as push-pull
            port->OTYPER &= ~(1UL << pin_pos);

            // Start with row low (normal scanning state)
            port->ODR &= ~pin;
        }

        // Configure columns back to inputs with pull-down
        for (uint8_t col = 0; col < KEY_COLS; col++) {
            uint32_t pin = col_pins[col];
            GPIO_TypeDef* port = col_ports[col];
            uint32_t pin_pos = 0;

            // Find pin position
            while ((pin >> pin_pos) != 1) pin_pos++;

            // Configure as input (MODE = 00)
            port->MODER &= ~(3UL << (pin_pos * 2));

            // Configure pull-down (PUPDR = 10)
            port->PUPDR &= ~(3UL << (pin_pos * 2));
            port->PUPDR |= (2UL << (pin_pos * 2));
        }
    }
}


/**
 * @brief Set the connection mode (USB or BLE)
 * @param mode The connection mode to set
 * @retval None
 */
void PM_SetConnectionMode(connection_mode_t mode) {
    if (g_connection_mode != mode) {
        g_connection_mode = mode;
        // Sync with keyboard state
        kb_state.connection_mode = mode;
    }
}

/**
 * @brief Get current battery level
 * @retval Battery level percentage (0-100)
 */
uint8_t PM_GetBatteryLevel(void) {
    // Simplified implementation - just return stored value
    return battery_level;
}


/**
 * @brief System clock configuration
 * @retval None
 */
__weak void SystemClock_Config(void) {
    // Enable HSE
    RCC->CR |= RCC_CR_HSEON;
    while(!(RCC->CR & RCC_CR_HSERDY));

    // Configure PLL (HSE as source, PLLMUL x8, PLLDIV /3)
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL | RCC_CFGR_PLLDIV);
    RCC->CFGR |= RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMUL8 | RCC_CFGR_PLLDIV3;

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));

    // Set Flash latency
    FLASH->ACR |= FLASH_ACR_LATENCY;

    // Select PLL as system clock
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // Update SystemCoreClock variable
    SystemCoreClock = 32000000;
}
