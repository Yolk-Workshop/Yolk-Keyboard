/*
 * ble_uart.c
 *
 *  Created on: Jan 7, 2025
 *      Author: bettysidepiece
 */
#include "stm32l0xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include "logger.h"
#include <main.h>


#define UART_RX_BUFFER_SIZE 128
#define UART_TX_BUFFER_SIZE 128
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1)
#define UART_TX_BUFFER_MASK (UART_TX_BUFFER_SIZE - 1)

// Ensure buffer sizes are powers of 2
//_static_assert((UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK) == 0, "RX buffer size must be a power of 2");
//_Static_assert((UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK) == 0, "TX buffer size must be a power of 2");

// UART Buffers
static volatile uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];

// Buffer Pointers
static volatile uint16_t uart_rx_head = 0;
static volatile uint16_t uart_rx_tail = 0;
static volatile uint16_t uart_tx_head = 0;
static volatile uint16_t uart_tx_tail = 0;

// UART State Variables
static volatile uint16_t uart_rx_count = 0;
static volatile uint16_t uart_tx_count = UART_TX_BUFFER_SIZE;

// UART Instance (LPUART1)
extern UART_HandleTypeDef hlpuart1;
extern DMA_HandleTypeDef hdma_lpuart1_rx;
extern DMA_HandleTypeDef hdma_lpuart1_tx;

// DMA Buffers
uint8_t dma_rx_buffer[UART_RX_BUFFER_SIZE];
uint8_t dma_tx_buffer[UART_TX_BUFFER_SIZE];


void tx_irq_handler(void) {
    __disable_irq();
    //ATOMIC_CLEAR_BIT(hlpuart1.Instance->CR1, USART_CR1_IDLEIE);

    if(uart_tx_tail != uart_tx_head) {
        uint16_t next_tail = (uart_tx_tail + 1) & UART_TX_BUFFER_MASK;
        dma_tx_buffer[0] = uart_tx_buffer[uart_tx_tail];
        HAL_UART_Transmit_DMA(&hlpuart1, dma_tx_buffer, 1);
        uart_tx_tail = next_tail;
    }

    __enable_irq();
}


void rx_irq_handler(void) {
    __disable_irq();
    ATOMIC_CLEAR_BIT(hlpuart1.Instance->CR1, USART_CR1_IDLEIE);

    // Get actual bytes received
    uint16_t dma_count = __HAL_DMA_GET_COUNTER(&hdma_lpuart1_rx);
    uint16_t bytes_received = UART_RX_BUFFER_SIZE - dma_count;

    for(uint16_t i = 0; i < bytes_received; i++) {
        uint16_t next_head = (uart_rx_head + 1) & UART_RX_BUFFER_MASK;

        if(next_head != uart_rx_tail) {
            uart_rx_buffer[uart_rx_head] = dma_rx_buffer[i];
            uart_rx_head = next_head;
            uart_rx_count++;
        }
        else {
            LOG_WARNING("RX Buffer overflow");
            break;
        }
    }

    // Restart DMA reception
    HAL_UART_AbortReceive(&hlpuart1);
    if(HAL_UART_Receive_DMA(&hlpuart1, dma_rx_buffer, UART_RX_BUFFER_SIZE) != HAL_OK) {
        LOG_ERROR("UART DMA RX restart failed");
    }

    ATOMIC_SET_BIT(hlpuart1.Instance->CR1, USART_CR1_IDLEIE);
    __enable_irq();
}



// UART helper functions
bool uart_transmit_ready(void)
{
	LOG_DEBUG("DMA TX State: %d", HAL_DMA_GetState(&hdma_lpuart1_tx));
	return (HAL_DMA_GetState(&hdma_lpuart1_tx) == HAL_DMA_STATE_READY);
}


bool uart_data_ready(void) {
    static bool last_state = false;
    bool current_state = (uart_rx_count > 0);

    if(current_state != last_state) {
        LOG_DEBUG("UART RX state changed: %d", uart_rx_count);
        last_state = current_state;
    }

    return current_state;
}


void uart_write(uint8_t data) {
    //LOG_DEBUG("UART WRITE ENTER");
    __disable_irq();

    // Store in ring buffer
    uint16_t next_head = (uart_tx_head + 1) & UART_TX_BUFFER_MASK;
    if(next_head != uart_tx_tail) {
        uart_tx_buffer[uart_tx_head] = data;
        uart_tx_head = next_head;

        // If DMA not busy, start transmission
        if(HAL_DMA_GetState(&hdma_lpuart1_tx) == HAL_DMA_STATE_READY) {
            dma_tx_buffer[0] = uart_tx_buffer[uart_tx_tail];
            uart_tx_tail = (uart_tx_tail + 1) & UART_TX_BUFFER_MASK;
            HAL_UART_Transmit_DMA(&hlpuart1, dma_tx_buffer, 1);
        }
    }

    __enable_irq();
    //LOG_DEBUG("UART WRITE EXIT");
}


uint8_t uart_read(void) {
    uint8_t read = 0;

    // Disable IDLE line interrupt during read
    ATOMIC_CLEAR_BIT(hlpuart1.Instance->CR1, USART_CR1_IDLEIE);

    if(uart_rx_count > 0) {
        read = uart_rx_buffer[uart_rx_tail];
        uart_rx_tail = (uart_rx_tail + 1) & UART_RX_BUFFER_MASK;
        uart_rx_count--;
    }
    else {
        LOG_WARNING("RX buffer empty");
    }

    // Re-enable IDLE line after read complete
    ATOMIC_SET_BIT(hlpuart1.Instance->CR1, USART_CR1_IDLEIE);
    //LOG_DEBUG("R:%c", read);
    return read;
}

