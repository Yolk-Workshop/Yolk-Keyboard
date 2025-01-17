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

#define UART_RX_BUFFER_SIZE 256
#define UART_TX_BUFFER_SIZE 128
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1)
#define UART_TX_BUFFER_MASK (UART_TX_BUFFER_SIZE - 1)

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

extern volatile ITStatus sys_ready;


void tx_irq_handler(void) {
    __disable_irq();

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

    RNBD.async.msg_in_progress = false;
    uint16_t dma_count = __HAL_DMA_GET_COUNTER(&hdma_lpuart1_rx);
    uint16_t bytes_received = UART_RX_BUFFER_SIZE - dma_count;

    for(uint16_t i = 0; i < bytes_received; i++) {
        uint16_t next_head = (uart_rx_head + 1) & UART_RX_BUFFER_MASK;

        if(next_head != uart_rx_tail) {
            uint8_t rx_byte = dma_rx_buffer[i];

            // Store all data in main UART buffer
            uart_rx_buffer[uart_rx_head] = rx_byte;
            uart_rx_head = next_head;
            uart_rx_count++;

            // Handle async messages
            if (rx_byte == '%') {
                if (!RNBD.async.msg_in_progress) {
                    // Start of async message
                	RNBD.async.msg_in_progress = true;
                    RNBD.async.async_pHead = RNBD.async.async_buffer;
                    *RNBD.async.async_pHead++ = rx_byte;
                } else {
                    // End of async message
                    *RNBD.async.async_pHead++ = rx_byte;
                    *RNBD.async.async_pHead = '\0';
                    if(sys_ready){
                    	RNBD.callback.asyncHandler((char*)RNBD.async.async_buffer); //XXX
                    }
                    RNBD.async.msg_in_progress = false;
                }
            }
            else if (RNBD.async.msg_in_progress) {
                // Store byte in async buffer if we're between % characters
                if (RNBD.async.async_pHead < RNBD.async.async_buffer + RNBD.async.asyncBufferSize - 1) {
                    *RNBD.async.async_pHead++ = rx_byte;
                }
            }
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
	printf("DMA TX State: %d", HAL_DMA_GetState(&hdma_lpuart1_tx));
	return (HAL_DMA_GetState(&hdma_lpuart1_tx) == HAL_DMA_STATE_READY);
}


bool uart_data_ready(void) {
    static bool last_state = false;
    bool current_state = (uart_rx_count > 0);

    if(current_state != last_state) {
        printf("UART RX state changed: %d", uart_rx_count);
        last_state = current_state;
    }

    return current_state;
}


void uart_write(uint8_t data) {
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

    ATOMIC_SET_BIT(hlpuart1.Instance->CR1, USART_CR1_IDLEIE);
    return read;
}

