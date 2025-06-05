
#include "kb_ble_api.h"
#include "main.h"
#include "stm32l0xx_it.h"
#include "pmsm.h"
#include "logger.h"

extern PCD_HandleTypeDef hpcd_USB_FS;
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_lpuart1_rx;
extern DMA_HandleTypeDef hdma_lpuart1_tx;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef hlpuart1;
extern volatile uint8_t scan_flag;
extern volatile ITStatus pmsm_update_flag;
extern volatile pm_state_t current_pm_state;
extern keyboard_state_t kb_state;
extern volatile bool bm7x_timer_flag;
extern volatile uint8_t startup_complete;

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
	while (1) {
	}
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
	while (1) {
	}
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{

}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{

}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
	HAL_IncTick();

}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles EXTI line 0 and line 1 interrupts.
 */
/**
 * @brief This function handles EXTI line 0 and line 1 interrupts.
 * Note: Only EXTI1 is used for R0
 */
void EXTI0_1_IRQHandler(void)
{
    // Check for EXTI1 (R0)
    if (EXTI->PR & EXTI_PR_PR1) {
        // Clear the flag
        EXTI->PR = EXTI_PR_PR1;
        // Handle the interrupt
        PM_HandleWakeup();
    }
}

/**
 * @brief This function handles EXTI line 2 and line 3 interrupts.
 * EXTI2 = R1, EXTI3 = R2
 */
void EXTI2_3_IRQHandler(void)
{
    uint32_t pending = EXTI->PR & (EXTI_PR_PR2 | EXTI_PR_PR3);

    if (pending) {
        // Clear all pending flags at once
        EXTI->PR = pending;

        // Handle any pending interrupt (we don't need to know which row)
        PM_HandleWakeup();
    }
}

/**
 * @brief This function handles EXTI line[4:15] interrupts.
 * EXTI4 = R3, EXTI5 = R4, EXTI6 = R5
 */
void EXTI4_15_IRQHandler(void)
{
    uint32_t pending = EXTI->PR & (EXTI_PR_PR4 | EXTI_PR_PR5 | EXTI_PR_PR6);

    if (pending) {
        // Clear the pending flags
        EXTI->PR = pending;

        // Handle the interrupt (we don't need to know which row)
        PM_HandleWakeup();
    }
}

void LPTIM1_IRQHandler(void) {
    if (LPTIM1->ISR & LPTIM_ISR_ARRM) {
        // Clear interrupt flag
        LPTIM1->ICR |= LPTIM_ICR_ARRMCF;

        // Feed IWDG immediately in interrupt context
        IWDG->KR = 0xAAAA;

    }
}

/**
 * @brief This function handles DMA1 channel 2 and channel 3 interrupts.
 */
void DMA1_Channel2_3_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_lpuart1_tx);
	HAL_DMA_IRQHandler(&hdma_lpuart1_rx);
}

/**
 * @brief This function handles DMA1 channel 4, channel 5, channel 6 and channel 7 interrupts.
 */
void DMA1_Channel4_5_6_7_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

/**
 * @brief This function handles TIM3 global interrupt.
 */
void TIM3_IRQHandler(void)
{
	/* USER CODE BEGIN TIM3_IRQn 0 */
	static uint32_t start_time = 0;
	uint32_t primask = __get_PRIMASK();
	__disable_irq();

	// Set scan flag if it's not already set
	if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET &&
	__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE) != RESET) {
		__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);

		if (!scan_flag) {
			scan_flag = SET;
		}
	}

	// Check BLE connection every 10 seconds
	if ((kb_state.output_mode == OUTPUT_BLE)
			&& ((uint32_t) (HAL_GetTick() - start_time)) >= 10000) {
		start_time = HAL_GetTick();
		// ble_conn_flag = SET; // Signal main loop to check BLE connection
	}

	// Update power management state every second
	if (((uint32_t) (HAL_GetTick() - start_time)) >= 1000) {
		pmsm_update_flag = SET;
	}

	__set_PRIMASK(primask);
	/* USER CODE END TIM3_IRQn 1 */
}

void TIM7_IRQHandler(void)
{
    if (TIM7->SR & TIM_SR_UIF) {
        TIM7->SR &= ~TIM_SR_UIF; // Clear interrupt flag
        bm7x_timer_flag = true;
    }
}


/**
 * @brief This function handles USART2 global interrupt.
 */
void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart2);
}

/**
 * @brief This function handles RNG and LPUART1 interrupts / LPUART1 wake-up interrupt through EXTI line 28.
 */
void RNG_LPUART1_IRQHandler(void)
{
    uint32_t isr = LPUART1->ISR;

    // Handle wake-up flag
    if (isr & USART_ISR_WUF) {
        LPUART1->ICR = USART_ICR_WUCF;
        PM_HandleWakeup();
    }

    // Handle received data
    if (isr & USART_ISR_RXNE) {
        PM_RecordActivity();
    }

    // Clear error flags that could prevent proper operation
    if (isr & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE)) {
        LPUART1->ICR = USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF | USART_ICR_PECF;
    }

    // Call your existing handlers
    rx_irq_handler();
    HAL_UART_IRQHandler(&hlpuart1);
}

/**
 * @brief This function handles USB event interrupt / USB wake-up interrupt through EXTI line 18.
 */
void USB_IRQHandler(void)
{
	HAL_PCD_IRQHandler(&hpcd_USB_FS);

}

