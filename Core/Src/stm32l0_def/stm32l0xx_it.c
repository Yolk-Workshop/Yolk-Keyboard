/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32l0xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 Yolk Workshop
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_it.h"
#include "pmsm.h"
#include "logger.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
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

extern volatile uint8_t startup_complete;

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */
	while (1) {
	}
	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
	/* USER CODE BEGIN SVC_IRQn 0 */

	/* USER CODE END SVC_IRQn 0 */
	/* USER CODE BEGIN SVC_IRQn 1 */

	/* USER CODE END SVC_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
	/* USER CODE BEGIN PendSV_IRQn 0 */

	/* USER CODE END PendSV_IRQn 0 */
	/* USER CODE BEGIN PendSV_IRQn 1 */

	/* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
	/* USER CODE BEGIN SysTick_IRQn 0 */

	/* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
	/* USER CODE BEGIN SysTick_IRQn 1 */

	/* USER CODE END SysTick_IRQn 1 */
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
void EXTI0_1_IRQHandler(void)
{
    uint32_t pending = EXTI->PR & 0x0003;
    EXTI->PR = pending;  // Clear all at once

    // Since we only wake from sleep, we don't need to identify specific pin
    if (pending) {
        PM_HandleKeyInterrupt(pending);
    }
}

/**
 * @brief This function handles EXTI line 2 and line 3 interrupts.
 */
void EXTI2_3_IRQHandler(void)
{
    uint32_t pending = EXTI->PR & 0x000C;
    EXTI->PR = pending;  // Clear all at once

    if (pending) {
        PM_HandleKeyInterrupt(pending);
    }
}

/**
 * @brief This function handles EXTI line[4:15] interrupts.
 */
void EXTI4_15_IRQHandler(void)
{
    uint32_t pending = EXTI->PR & 0xFFF0;
    EXTI->PR = pending;  // Clear all at once

    if (pending) {
        PM_HandleKeyInterrupt(pending);
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
	/* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */

	/* USER CODE END DMA1_Channel2_3_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_lpuart1_tx);
	HAL_DMA_IRQHandler(&hdma_lpuart1_rx);
	/* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */

	/* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

/**
 * @brief This function handles DMA1 channel 4, channel 5, channel 6 and channel 7 interrupts.
 */
void DMA1_Channel4_5_6_7_IRQHandler(void)
{
	/* USER CODE BEGIN DMA1_Channel4_5_6_7_IRQn 0 */

	/* USER CODE END DMA1_Channel4_5_6_7_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart2_tx);
	/* USER CODE BEGIN DMA1_Channel4_5_6_7_IRQn 1 */

	/* USER CODE END DMA1_Channel4_5_6_7_IRQn 1 */
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

		/* Refresh watchdog every 10 timer ticks (adjust based on timer frequency)
		wdg_counter++;
		if (wdg_counter >= 10) {
			WDG_Refresh();
			wdg_counter = 0;
		}*/


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

/**
 * @brief This function handles USART2 global interrupt.
 */
void USART2_IRQHandler(void)
{
	/* USER CODE BEGIN USART2_IRQn 0 */

	/* USER CODE END USART2_IRQn 0 */
	HAL_UART_IRQHandler(&huart2);
	/* USER CODE BEGIN USART2_IRQn 1 */

	/* USER CODE END USART2_IRQn 1 */
}

/**
 * @brief This function handles RNG and LPUART1 interrupts / LPUART1 wake-up interrupt through EXTI line 28.
 */
void RNG_LPUART1_IRQHandler(void)
{
	/* USER CODE BEGIN RNG_LPUART1_IRQn 0 */
	if (__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_IDLE)) {
		__HAL_UART_CLEAR_IDLEFLAG(&hlpuart1);
		rx_irq_handler();
	}
	/* USER CODE END RNG_LPUART1_IRQn 0 */
	HAL_UART_IRQHandler(&hlpuart1);
	/* USER CODE BEGIN RNG_LPUART1_IRQn 1 */

	/* USER CODE END RNG_LPUART1_IRQn 1 */
}

/**
 * @brief This function handles USB event interrupt / USB wake-up interrupt through EXTI line 18.
 */
void USB_IRQHandler(void)
{
	/* USER CODE BEGIN USB_IRQn 0 */

	/* USER CODE END USB_IRQn 0 */
	HAL_PCD_IRQHandler(&hpcd_USB_FS);
	/* USER CODE BEGIN USB_IRQn 1 */

	/* USER CODE END USB_IRQn 1 */
}

