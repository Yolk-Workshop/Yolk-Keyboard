/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "logger.h"
#include "stm32l0xx_it.h"


/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN EV */

extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart2;

extern DMA_HandleTypeDef hdma_lpuart1_rx;
extern DMA_HandleTypeDef hdma_lpuart1_tx;
extern UART_HandleTypeDef hlpuart1;

extern TIM_HandleTypeDef htim3;

extern volatile uint8_t scan_flag;
extern volatile uint8_t ble_conn_flag;
extern keyboard_state_t kb_state;
extern rnbd_interface_t RNBD;

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable Interrupt.
  */
void NMI_Handler(void)
{
   while (1)
  {
  }
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  while (1)
  {
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
  * @brief This function handles DMA1 channel 2 and channel 3 interrupts.
  */

void DMA1_Channel2_3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_lpuart1_tx);
  HAL_DMA_IRQHandler(&hdma_lpuart1_rx);
}

void DMA1_Channel4_5_6_7_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart2);
}

void RNG_LPUART1_IRQHandler(void)
{
	if(__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_IDLE)) {
	        __HAL_UART_CLEAR_IDLEFLAG(&hlpuart1);
	        rx_irq_handler();
	    }

	HAL_UART_IRQHandler(&hlpuart1);
}

void TIM3_IRQHandler(void) {
	// 500us Timer Interrupt
	static uint32_t ble_start_time = 0;

	uint32_t primask = __get_PRIMASK();
	__disable_irq();

    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET &&
        __HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);

		// Set scan flag
		if (!scan_flag) {
			scan_flag = SET;
		}
    }

    // Check BLE connection every timer interval
    if ((kb_state.output_mode == OUTPUT_BLE) &&
		((uint32_t)(HAL_GetTick() - ble_start_time)) >= RNBD.device.connection_timer) {
    	ble_start_time = HAL_GetTick();
    	ble_conn_flag = SET; // Signal main loop to check BLE connection
    }

    __set_PRIMASK(primask);
}

__weak void I2C1_IRQHandler(void) {
	LOG_DEBUG("I2C1 IRQ: %d",__NVIC_GetEnableIRQ(I2C1_IRQn));
}

/**
  * @brief This function handles USB event interrupt / USB wake-up interrupt through EXTI line 18.
  */
void USB_IRQHandler(void)
{

  HAL_PCD_IRQHandler(&hpcd_USB_FS);

}

/**
  * @brief This function handles EXTI line [9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{

}

/**
  * @brief This function handles EXTI line [15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{

}
