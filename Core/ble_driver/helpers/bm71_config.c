/**
 * @file bm71_config.c
 * @brief BM71 configuration mode functions
 * @date May 18, 2025
 */

#include "bm71_config.h"
#include "logger.h"
#include <string.h>

#if BLE_CONFIG
// Static variables
static UART_HandleTypeDef *config_huart;

#define BM71_CONFIG_ADDR          0x00033800  /* Starting address of configuration section */
#define BM71_NAME_FRAGMENT_OFFSET 0x0006      /* Offset for device name in configuration */
#define BM71_SYSTEM_OPT2_OFFSET   0x2B        /* Offset for system option 2 (operation mode) */
#define BM71_FLASH_ERASE_SIZE     0x800       /* 2KB - minimum erase size for flash */

#define FLASH_START      0x30000U
#define FLASH_END_BM71   0x36000U
#define BLOCK_SIZE       128U
#define CHUNK_SIZE       64U  // ACTUAL_DATA_SIZE
#define READ_SIZE        (HEADER_SIZE + CHUNK_SIZE)


static uint8_t calculate_checksum(uint8_t *data, uint16_t length);
static void uart_transmit(uint8_t *data, uint16_t size);
static bm71_config_error_t unlock_flash(void);
static bool wait_for_isdap_response(uint16_t expected_opcode, uint32_t timeout);
static bool wait_for_event(uint8_t expected_event, uint32_t timeout);
static bm71_config_error_t unlock_flash(void);
static void send_hci_command(uint8_t *data, uint16_t length);


typedef struct Memory128 {
    uint16_t address;
    uint8_t data[128];
} Flash128_t;

const Flash128_t memoryBlock[] = {
    {
        .address = 0x0000,
        .data = {
            0x0A, 0x00, 0x00, 0x00, 0x02, 0x28, 0x00, 0x00,
            0x00, 0x18, 0x0A, 0x0B, 0x00, 0x00, 0x00, 0x02,
            0x28, 0x03, 0x02, 0x00, 0x00, 0x2A, 0x29, 0x14,
            0x00, 0x00, 0x00, 0x02, 0x2A, 0x29, 0x0D, 0x59,
            0x6F, 0x6C, 0x6B, 0x2D, 0x57, 0x6F, 0x72, 0x6B,
            0x73, 0x68, 0x6F, 0x70, 0x0B, 0x00, 0x00, 0x00,
            0x02, 0x28, 0x03, 0x02, 0x00, 0x00, 0x2A, 0x24,
            0x14, 0x00, 0x00, 0x00, 0x02, 0x2A, 0x24, 0x0D,
            0x59, 0x6F, 0x6C, 0x6B, 0x2D, 0x4B, 0x65, 0x79,
            0x62, 0x6F, 0x61, 0x72, 0x64, 0x0B, 0x00, 0x00,
            0x00, 0x02, 0x28, 0x03, 0x02, 0x00, 0x00, 0x2A,
            0x25, 0x0B, 0x00, 0x00, 0x00, 0x02, 0x2A, 0x25,
            0x04, 0x30, 0x30, 0x30, 0x31, 0x0B, 0x00, 0x00,
            0x00, 0x02, 0x28, 0x03, 0x02, 0x00, 0x00, 0x2A,
            0x27, 0x10, 0x00, 0x00, 0x00, 0x02, 0x2A, 0x27,
            0x09, 0x59, 0x57, 0x32, 0x30, 0x32, 0x35, 0x59
        }
    },
    {
        .address = 0x0080,
        .data = {
            0x42, 0x31, 0x0B, 0x00, 0x00, 0x00, 0x02, 0x28,
            0x03, 0x02, 0x00, 0x00, 0x2A, 0x26, 0x0D, 0x00,
            0x00, 0x00, 0x02, 0x2A, 0x26, 0x06, 0x31, 0x30,
            0x30, 0x30, 0x37, 0x30, 0x0B, 0x00, 0x00, 0x00,
            0x02, 0x28, 0x03, 0x02, 0x00, 0x00, 0x2A, 0x28,
            0x0B, 0x00, 0x00, 0x00, 0x02, 0x2A, 0x28, 0x04,
            0x30, 0x30, 0x30, 0x31, 0x0B, 0x00, 0x00, 0x00,
            0x02, 0x28, 0x03, 0x02, 0x00, 0x00, 0x2A, 0x23,
            0x0F, 0x00, 0x00, 0x00, 0x02, 0x2A, 0x23, 0x08,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x0B, 0x00, 0x00, 0x00, 0x02, 0x28, 0x03, 0x02,
            0x00, 0x00, 0x2A, 0x2A, 0x0F, 0x00, 0x00, 0x00,
            0x02, 0x2A, 0x2A, 0x08, 0x00, 0x00, 0x00, 0x00,
            0x01, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x00, 0x00,
            0x02, 0x28, 0x03, 0x02, 0x00, 0x00, 0x2A, 0x01,
            0x09, 0x00, 0x00, 0x00, 0x02, 0x2A, 0x01, 0x02
        }
    },
    {
        .address = 0x0100,
        .data = {
            0xC1, 0x03, 0x0A, 0x00, 0x00, 0x00, 0x02, 0x28,
            0x00, 0x00, 0x00, 0x18, 0x12, 0x0B, 0x04, 0x00,
            0x00, 0x02, 0x28, 0x03, 0x02, 0x00, 0x00, 0x2A,
            0x4B, 0x4F, 0x04, 0x00, 0x00, 0x02, 0x2A, 0x4B,
            0x48, 0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x85,
            0x01, 0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15,
            0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81,
            0x02, 0x95, 0x01, 0x75, 0x08, 0x81, 0x01, 0x95,
            0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0x65, 0x05,
            0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00, 0xC0,
            0x05, 0x0C, 0x09, 0x01, 0xA1, 0x01, 0x85, 0x02,
            0x15, 0x00, 0x26, 0xFF, 0x03, 0x19, 0x00, 0x2A,
            0xFF, 0x03, 0x75, 0x10, 0x95, 0x01, 0x81, 0x00,
            0xC0, 0x0B, 0x04, 0x00, 0x00, 0x02, 0x28, 0x03,
            0x02, 0x00, 0x00, 0x2A, 0x4A, 0x0B, 0x04, 0x00,
            0x00, 0x02, 0x2A, 0x4A, 0x04, 0x01, 0x01, 0x00
        }
    },
    {
        .address = 0x0180,
        .data = {
            0x03, 0x0B, 0x00, 0x00, 0x00, 0x02, 0x28, 0x03,
            0x04, 0x00, 0x00, 0x2A, 0x4C, 0x08, 0x00, 0x00,
            0x00, 0x02, 0x2A, 0x4C, 0x01, 0x00, 0x0B, 0x04,
            0x00, 0x00, 0x02, 0x28, 0x03, 0x06, 0x00, 0x00,
            0x2A, 0x4E, 0x08, 0x04, 0x00, 0x00, 0x02, 0x2A,
            0x4E, 0x01, 0x01, 0x0B, 0x0C, 0x00, 0x00, 0x02,
            0x28, 0x03, 0x1A, 0x00, 0x00, 0x2A, 0x4D, 0x0F,
            0x0C, 0x00, 0x00, 0x02, 0x2A, 0x4D, 0x08, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09,
            0x0C, 0x00, 0x00, 0x02, 0x29, 0x08, 0x02, 0x01,
            0x01, 0x08, 0x0C, 0x00, 0x00, 0x02, 0x29, 0x02,
            0x00, 0x00, 0x0B, 0x04, 0x00, 0x00, 0x02, 0x28,
            0x03, 0x12, 0x00, 0x00, 0x2A, 0x22, 0x0F, 0x04,
            0x00, 0x00, 0x02, 0x2A, 0x22, 0x08, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x04,
            0x00, 0x00, 0x02, 0x29, 0x02, 0x00, 0x00, 0x0B
        }
    },
    {
        .address = 0x0200,
        .data = {
            0x0C, 0x00, 0x00, 0x02, 0x28, 0x03, 0x0E, 0x00,
            0x00, 0x2A, 0x32, 0x08, 0x0C, 0x00, 0x00, 0x02,
            0x2A, 0x32, 0x01, 0x00, 0x0B, 0x0C, 0x00, 0x00,
            0x02, 0x28, 0x03, 0x1A, 0x00, 0x00, 0x2A, 0x4D,
            0x09, 0x0C, 0x00, 0x00, 0x02, 0x2A, 0x4D, 0x02,
            0x00, 0x00, 0x09, 0x0C, 0x00, 0x00, 0x02, 0x29,
            0x08, 0x02, 0x02, 0x01, 0x08, 0x0C, 0x00, 0x00,
            0x02, 0x29, 0x02, 0x00, 0x00, 0x0A, 0x00, 0x00,
            0x00, 0x02, 0x28, 0x00, 0x00, 0x00, 0x18, 0x0F,
            0x0B, 0x00, 0x00, 0x00, 0x02, 0x28, 0x03, 0x12,
            0x00, 0x00, 0x2A, 0x19, 0x08, 0x00, 0x00, 0x00,
            0x02, 0x2A, 0x19, 0x01, 0x64, 0x08, 0x00, 0x00,
            0x00, 0x02, 0x29, 0x02, 0x00, 0x00, 0x18, 0x00,
            0x00, 0x00, 0x02, 0x28, 0x00, 0x00, 0x00, 0x49,
            0x53, 0x53, 0x43, 0xFE, 0x7D, 0x4A, 0xE5, 0x8F,
            0xA9, 0x9F, 0xAF, 0xD2, 0x05, 0xE4, 0x55, 0x19
        }
    },
    {
        .address = 0x0280,
        .data = {
            0x00, 0x00, 0x00, 0x02, 0x28, 0x03, 0x18, 0x00,
            0x00, 0x49, 0x53, 0x53, 0x43, 0x1E, 0x4D, 0x4B,
            0xD9, 0xBA, 0x61, 0x23, 0xC6, 0x47, 0x24, 0x96,
            0x16, 0x16, 0x00, 0x00, 0x00, 0x10, 0x49, 0x53,
            0x53, 0x43, 0x1E, 0x4D, 0x4B, 0xD9, 0xBA, 0x61,
            0x23, 0xC6, 0x47, 0x24, 0x96, 0x16, 0x01, 0x00,
            0x08, 0x00, 0x00, 0x00, 0x02, 0x29, 0x02, 0x00,
            0x00, 0x19, 0x00, 0x00, 0x00, 0x02, 0x28, 0x03,
            0x0C, 0x00, 0x00, 0x49, 0x53, 0x53, 0x43, 0x88,
            0x41, 0x43, 0xF4, 0xA8, 0xD4, 0xEC, 0xBE, 0x34,
            0x72, 0x9B, 0xB3, 0x16, 0x00, 0x00, 0x00, 0x10,
            0x49, 0x53, 0x53, 0x43, 0x88, 0x41, 0x43, 0xF4,
            0xA8, 0xD4, 0xEC, 0xBE, 0x34, 0x72, 0x9B, 0xB3,
            0x01, 0x00, 0x19, 0x00, 0x00, 0x00, 0x02, 0x28,
            0x03, 0x1C, 0x00, 0x00, 0x49, 0x53, 0x53, 0x43,
            0x4C, 0x8A, 0x39, 0xB3, 0x2F, 0x49, 0x51, 0x1C
        }
    },
    {
        .address = 0x0300,
        .data = {
            0xFF, 0x07, 0x3B, 0x7E, 0x16, 0x00, 0x00, 0x00,
            0x10, 0x49, 0x53, 0x53, 0x43, 0x4C, 0x8A, 0x39,
            0xB3, 0x2F, 0x49, 0x51, 0x1C, 0xFF, 0x07, 0x3B,
            0x7E, 0x01, 0x00, 0x08, 0x00, 0x00, 0x00, 0x02,
            0x29, 0x02, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
        }
    },
    {
        .address = 0x0800,
        .data = {
            0x59, 0x6F, 0x6C, 0x6B, 0x20, 0x4B, 0x65, 0x79,
            0x62, 0x6F, 0x61, 0x72, 0x64, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x03, 0x02, 0x50, 0x04, 0x02, 0x0C, 0x0C, 0x02,
            0x05, 0x00, 0xBA, 0xB0, 0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xA0, 0x06, 0x01,
            0x03, 0x00, 0x3E, 0x00, 0x84, 0x6C, 0x58, 0x50,
            0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x01, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x3C, 0x1E, 0x1E, 0x00, 0x00, 0x00, 0x06, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00
        }
    },
    {
        .address = 0x0880,
        .data = {
            0x1A, 0x0E, 0x09, 0x59, 0x6F, 0x6C, 0x6B, 0x20,
            0x4B, 0x65, 0x79, 0x62, 0x6F, 0x61, 0x72, 0x64,
            0x03, 0x03, 0x12, 0x18, 0x02, 0x01, 0x06, 0x03,
            0x19, 0xC1, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x08, 0x02, 0x0A, 0x00, 0x04, 0x16, 0x0F, 0x18,
            0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x03, 0x02, 0x01, 0x06, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x01, 0x59,
            0x4F, 0x4C, 0x4B, 0x00, 0x02, 0x00, 0xAA, 0x55,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
        }
    },
};


const size_t memoryBlockCount = sizeof(memoryBlock) / sizeof(memoryBlock[0]);


// UART transmit helper
static void uart_transmit(uint8_t *data, uint16_t size)
{
    HAL_UART_Transmit(config_huart, data, size, BM71_CONFIG_UART_TIMEOUT);
    HAL_Delay(5);
}

static bm71_config_error_t unlock_flash(void)
{
    LOG_INFO("Unlocking BM71 flash memory...");

    uint8_t unlock_cmd[] = {
        0x02, 0xFF, 0x0F,   /* Start, Handle */
        0x07, 0x00,         /* ISDAP Data Length */
        0x00, 0x01,         /* ISDAP Opcode - 0x0100 */
        0x03, 0x00,         /* Data Length */
        0x03, 0x00, 0x00    /* Memory type, sub-type, unlock (0) */
    };

    /* Calculate and add checksum */
    unlock_cmd[sizeof(unlock_cmd)-1] = calculate_checksum(&unlock_cmd[1], sizeof(unlock_cmd)-2);
    send_hci_command(unlock_cmd, sizeof(unlock_cmd));
    if (!wait_for_isdap_response(0x0100, 1000)) {
        LOG_ERROR("Flash unlock failed");
        return BM71_CONFIG_ERROR_UNLOCK_FAILED;
    }

    LOG_INFO("BM71 flash memory unlocked successfully");
    return BM71_CONFIG_OK;
}

// UART receive helper with IRQ
static bool uart_receive(uint8_t *data, uint16_t size, uint32_t timeout) {
    uint32_t start_time = HAL_GetTick();

    // Wait for enough data to be available
    while (ble_rx_available() < size) {
        if ((HAL_GetTick() - start_time) >= timeout) {
            LOG_WARNING("BM71 receive timeout waiting for %u bytes", size);
            return false;
        }
        HAL_Delay(1);  // Small delay to prevent tight loop
    }

    // Read the data
    ble_rx_read_buffer(data, size);
    return true;
}

static uint8_t calculate_checksum(uint8_t *data, uint16_t length) {
    uint16_t sum = 0;
    for (uint16_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return (uint8_t)(0x100 - (sum & 0xFF));
}

static void send_hci_command(uint8_t *data, uint16_t length) {
    //LOG_DEBUG("Sending HCI command: 0x%02X", data[2]);
    uart_transmit(data, length);
}

static bool wait_for_event(uint8_t expected_event, uint32_t timeout)
{
    uint8_t buf[64];
    if (!uart_receive(buf, 3, timeout)) return false;
    if (buf[0] != 0x04 || buf[1] != expected_event) return false;
    uint8_t len = buf[2];
    return len == 0 || uart_receive(buf + 3, len, timeout);
}

static bool wait_for_isdap_response(uint16_t expected_opcode, uint32_t timeout) {
    uint8_t hdr[7];

    /* First wait for Number of Completed Packets Event (0x13) */
    if (!wait_for_event(0x13, timeout)) {
        LOG_ERROR("Did not receive Number of Completed Packets Event");
        return false;
    }
    /* Receive ISDAP response header */
    if (!uart_receive(hdr, sizeof(hdr), timeout)) {
        LOG_ERROR("Failed to receive ISDAP response header");
        return false;
    }
    /* Validate header (0x02) */
    if (hdr[0] != 0x02) {
        LOG_ERROR("Invalid ISDAP response header: %02X", hdr[0]);
        return false;
    }
    /* Extract length and opcode */
    uint16_t len = hdr[3] | (hdr[4] << 8);
    uint16_t opcode = hdr[5] | (hdr[6] << 8);

    if (opcode != expected_opcode) {
        LOG_ERROR("Unexpected ISDAP opcode: %04X (expected %04X)", opcode, expected_opcode);
        return false;
    }

    if (len < 2) {
        LOG_ERROR("ISDAP response too short");
        return false;
    }
    /* Discard remaining data */
    uint8_t discard[512];
    return uart_receive(discard, len - 2, timeout);
}


static bm71_config_error_t connect_to_flash(void) {
    uint8_t read_buffer_cmd[] = {0x01, 0x05, 0x10, 0x00};
    send_hci_command(read_buffer_cmd, sizeof(read_buffer_cmd));
    if (!wait_for_event(0x0E, 1000)) return BM71_CONFIG_ERROR_TIMEOUT;

    uint8_t host_buffer_cmd[] = {0x01, 0x33, 0x0C, 0x07, 0,0,0,0,0,0,0};
    send_hci_command(host_buffer_cmd, sizeof(host_buffer_cmd));
    if (!wait_for_event(0x0E, 1000)) return BM71_CONFIG_ERROR_TIMEOUT;

    uint8_t create_conn_cmd[] = {0x01, 0x05, 0x04, 0x0D, 0,0,0,0,0,0,0,0,0,0,0,0,0};
    send_hci_command(create_conn_cmd, sizeof(create_conn_cmd));
    if (!wait_for_event(0x0F, 1000)) return BM71_CONFIG_ERROR_TIMEOUT;
    if (!wait_for_event(0x03, 1000)) return BM71_CONFIG_ERROR_TIMEOUT;

    LOG_DEBUG("Flash Connected");
    return BM71_CONFIG_OK;
}

static bm71_config_error_t disconnect_from_flash(void) {
    uint8_t cmd[] = {0x01, 0x06, 0x04, 0x03, 0xFF, 0x0F, 0x00};
    send_hci_command(cmd, sizeof(cmd));
    if (!wait_for_event(0x0F, 1000)) return BM71_CONFIG_ERROR_TIMEOUT;
    if (!wait_for_event(0x05, 1000)) return BM71_CONFIG_ERROR_TIMEOUT;
    LOG_DEBUG("Flash Disconnected");
    return BM71_CONFIG_OK;
}

/**
 * Erase flash memory starting at specified address
 * @param start_addr: Must be aligned to 2KB boundary (0x800)
 * @param size_kb: Size in KB to erase (must be multiple of 2)
 * @return BM71_CONFIG_OK on success, error code on failure
 */
static bm71_config_error_t erase_flash_at_address(uint32_t start_addr, uint16_t size_kb) {
    // Validate 2KB alignment
    if (start_addr % 0x800 != 0) {
        LOG_ERROR("Start address 0x%08X not aligned to 2KB boundary", start_addr);
        return BM71_CONFIG_ERROR_INVALID_PARAM;
    }

    // Validate size is multiple of 2KB
    if (size_kb % 2 != 0) {
        LOG_ERROR("Size %dKB not multiple of 2KB", size_kb);
        return BM71_CONFIG_ERROR_INVALID_PARAM;
    }

    uint32_t erase_size = size_kb * 1024; // Convert KB to bytes

    LOG_INFO("Erasing BM71 flash: %dKB from 0x%08X...", size_kb, start_addr);

    uint8_t erase_cmd[] = {
        0x02, 0xFF, 0x0F,           // HCI ACL data
        0x0E, 0x00,                 // Length = 14 bytes
        0x12,                       // HCI payload length
        0x01,                       // Flags
        0x0A, 0x00,                 // Handle
        0x03,                       // Memory type (Flash)
        0x00,                       // Subtype
        (start_addr >> 0) & 0xFF,   // Start address byte 0
        (start_addr >> 8) & 0xFF,   // Start address byte 1
        (start_addr >> 16) & 0xFF,  // Start address byte 2
        (start_addr >> 24) & 0xFF,  // Start address byte 3
        (erase_size >> 0) & 0xFF,   // Erase size byte 0
        (erase_size >> 8) & 0xFF,   // Erase size byte 1
        (erase_size >> 16) & 0xFF,  // Erase size byte 2
        (erase_size >> 24) & 0xFF   // Erase size byte 3
    };

    ble_rx_flush();

    send_hci_command(erase_cmd, sizeof(erase_cmd));
    if (!wait_for_event(0x13, 1000)) {
        LOG_ERROR("Erase command failed - event 0x13 not received");
        return BM71_CONFIG_ERROR_ERASE_FAILED;
    }

    LOG_INFO("Successfully erased %dKB at 0x%08X", size_kb, start_addr);
    HAL_Delay(100); // Delay after erase

    return BM71_CONFIG_OK;
}

static bm71_config_error_t read_config_block(uint16_t offset, uint8_t *data, uint16_t len) {
    //LOG_INFO("Reading %d bytes from configuration offset 0x%04X", len, offset);

    uint32_t addr = BM71_CONFIG_ADDR + offset;

    uint8_t read_cmd[] = {
		0x02, 0xFF, 0x0F,           /* Start, Handle */
		0x0E, 0x00,                 /* ISDAP Data Length */
		0x10, 0x01,                 /* ISDAP Opcode - 0x0110 (Read) */
		0x0A, 0x00,                 /* Data Length */
		0x03, 0x00,                 /* Memory type, sub-type */
		(addr & 0xFF),              /* Address - Little Endian */
		(addr >> 8) & 0xFF,
		(addr >> 16) & 0xFF,
		(addr >> 24) & 0xFF,
		(len & 0xFF),               /* Size - Little Endian */
		(len >> 8) & 0xFF,
		0x00, 0x00
	};

    send_hci_command(read_cmd, sizeof(read_cmd));

    /* Wait for Number of Completed Packets Event (0x13) */
    wait_for_event(0x13, 1000);

    /* Read data */
    if (!uart_receive(data, len, 1000)) {
        LOG_ERROR("Read configuration - failed to receive data");
        return BM71_CONFIG_ERROR_READ_FAILED;
    }

    //LOG_DEBUG("Read configuration successful");
    return BM71_CONFIG_OK;
}


static bm71_config_error_t write_flash_block_128(uint16_t offset, const uint8_t *data)
{
    LOG_INFO("Writing 128 bytes to configuration offset 0x%04X", offset);

    uint32_t addr = BM71_CONFIG_ADDR + offset;

    static uint8_t packet[256];
    memset(packet, 0, sizeof(packet));

    // Header
    packet[0] = 0x02;       // Start byte
    packet[1] = 0xFF;       // Handle LSB
    packet[2] = 0x0F;       // Handle MSB

    // ISDAP Data Length = 14 + 128
    packet[3] = (14 + 128) & 0xFF;
    packet[4] = ((14 + 128) >> 8) & 0xFF;

    // ISDAP Opcode = 0x0111
    packet[5] = 0x11;
    packet[6] = 0x01;

    // ISDAP Payload Length = 10 + 128
    packet[7] = (10 + 128) & 0xFF;
    packet[8] = ((10 + 128) >> 8) & 0xFF;

    // Memory type / sub-type
    packet[9]  = 0x03;  // Flash
    packet[10] = 0x00;  // EFLASH

    // Address (little-endian)
    packet[11] = (addr >> 0)  & 0xFF;
    packet[12] = (addr >> 8)  & 0xFF;
    packet[13] = (addr >> 16) & 0xFF;
    packet[14] = (addr >> 24) & 0xFF;

    // Total write length (little-endian)
    packet[15] = 128 & 0xFF;
    packet[16] = (128 >> 8) & 0xFF;
    packet[17] = 0x00;
    packet[18] = 0x00;

    // Copy payload
    memcpy(&packet[19], data, 128);

    uint16_t packet_size = 19 + 128;

    // Optional checksum
#if BM71_USE_CHECKSUM
    packet[packet_size] = calculate_checksum(&packet[1], packet_size - 1);
    packet_size += 1;
#endif

    ble_rx_flush();  // Clear RX before command
    send_hci_command(packet, packet_size);

    // Wait for 0x13 event
    if (!wait_for_event(0x13, 1000)) {
        LOG_ERROR("Block write - event 0x13 not received");
        return BM71_CONFIG_ERROR_WRITE_FAILED;
    }

    // Wait for ISDAP response
    uint8_t resp[10];
    if (!uart_receive(resp, sizeof(resp), 1000)) {
        LOG_ERROR("Block write - failed to receive ISDAP response");
        return BM71_CONFIG_ERROR_WRITE_FAILED;
    }

    // Validate response
    if (resp[0] != 0x02 || resp[5] != 0x11 || resp[6] != 0x01 || resp[9] != 0x00) {
        LOG_ERROR("Block write - invalid response: %02X %02X %02X %02X",
                  resp[0], resp[5], resp[6], resp[9]);
        return BM71_CONFIG_ERROR_WRITE_FAILED;
    }

    LOG_INFO("Block write successful at offset 0x%04X", offset);
    HAL_Delay(50);  // Give time for BM71 to settle
    return BM71_CONFIG_OK;
}

bm71_config_error_t bm71_config_init(UART_HandleTypeDef *huart) {
    if (!huart) return BM71_CONFIG_ERROR_INVALID_PARAM;
    config_huart = huart;
    ble_uart_rx_init();
    __HAL_UART_ENABLE_IT(config_huart, UART_IT_RXNE);
    return BM71_CONFIG_OK;
}

static bool verify_blocks(bool log_mismatches) {
    const uint8_t READ_REQUEST_SIZE = 80;
    const uint8_t HEADER_SIZE = 11;
    const uint8_t ACTUAL_DATA_SIZE = 64;

    static uint8_t current_config[128];

    for (int i = 0; i < memoryBlockCount; i++) {
        uint16_t base_address = memoryBlock[i].address;

        for (int chunk = 0; chunk < 2; chunk++) {
            uint16_t address = base_address + (chunk * ACTUAL_DATA_SIZE);

            ble_rx_flush();
            HAL_Delay(10);

            bm71_config_error_t err = read_config_block(address, current_config, READ_REQUEST_SIZE);
            if (err != BM71_CONFIG_OK) {
                LOG_ERROR("Failed to read chunk %d at 0x%04X", chunk, address);
                return false;
            }

            uint8_t compare_offset = HEADER_SIZE;
            uint8_t skip = 0;
            uint8_t compare_size = ACTUAL_DATA_SIZE;
            uint16_t data_offset = chunk * ACTUAL_DATA_SIZE;

            if (i == 0 && chunk == 0) {
                skip = 11;
                compare_size -= skip;
            }

            const uint8_t *expected = memoryBlock[i].data + data_offset + skip;
            const uint8_t *actual   = current_config + compare_offset + skip;

            if (memcmp(actual, expected, compare_size) != 0) {
                if (log_mismatches) {
                    LOG_INFO("Mismatch at block 0x%04X, chunk %d", base_address, chunk);
                    for (int j = 0; j < compare_size && j < 3; j++) {
                        if (actual[j] != expected[j]) {
                            LOG_INFO("  Offset 0x%02X: actual=0x%02X, expected=0x%02X",
                                     data_offset + skip + j, actual[j], expected[j]);
                        }
                    }
                }
                return false;
            }
        }
    }

    return true;
}


bm71_config_error_t bm71_configure_yolk_keyboard(void) {
    LOG_INFO("Configuring BM71 for Yolk Keyboard...");
    bm71_config_error_t err;

    // 1. Connect to Flash
    err = connect_to_flash();
    if (err != BM71_CONFIG_OK) return err;

    // 2. Unlock Flash
    err = unlock_flash();
    if (err != BM71_CONFIG_OK) {
        disconnect_from_flash();
        return err;
    }

    // 3. Compare config
    LOG_INFO("Checking existing configuration...");
    if (verify_blocks(false)) {
        LOG_INFO("Configuration already matches, no update needed");
        disconnect_from_flash();
        return BM71_CONFIG_OK;
    }

    // 4. Erase 8KB
    LOG_INFO("Erasing flash region for config...");
    err = erase_flash_at_address(BM71_CONFIG_ADDR, 8);
    if (err != BM71_CONFIG_OK) {
        LOG_ERROR("Failed to erase config region");
        disconnect_from_flash();
        return err;
    }

    // 5. Write config blocks
    LOG_INFO("Writing configuration blocks...");
    for (int i = 0; i < memoryBlockCount; i++) {
        err = write_flash_block_128(memoryBlock[i].address, memoryBlock[i].data);
        if (err != BM71_CONFIG_OK) {
            LOG_ERROR("Failed to write block at 0x%04X", memoryBlock[i].address);
            disconnect_from_flash();
            return err;
        }
        LOG_INFO("Written block %2d/%d → 0x%04X", i + 1, memoryBlockCount, memoryBlock[i].address);
    }

    // 6. Verify again after write
    LOG_INFO("Verifying configuration after write...");
    bool verified = verify_blocks(true);

    ble_rx_flush();
    HAL_Delay(10);

    disconnect_from_flash();

    if (verified) {
        LOG_INFO("Final verification passed — config successfully written.");
        return BM71_CONFIG_OK;
    } else {
        LOG_ERROR("Final verification failed after writing config.");
        return BM71_CONFIG_ERROR_VERIFY_FAILED;
    }
}
#endif


//------------------------------------------------------------------------------------------------//

/* BLE UART Buffer and Control */

#define BLE_RX_BUFFER_SIZE 256
static uint8_t ble_rx_buffer[BLE_RX_BUFFER_SIZE];
static volatile uint16_t ble_rx_head = 0;
static volatile uint16_t ble_rx_tail = 0;
static volatile uint8_t ble_rx_data_ready = 0;
extern volatile bool bm7x_timer_flag;

/**
 * @brief RX interrupt handler for BLE UART
 * Called when data is received on LPUART1 or when IDLE line is detected
 */
void rx_irq_handler(void)
{
    /* Check if data was received */
    if (LPUART1->ISR & UART_FLAG_RXNE){
        /* Read the received byte from the proper register */
        uint8_t rx_data;
        rx_data = (uint8_t)(LPUART1->RDR & 0xFF);
        uint16_t next_head = (ble_rx_head + 1) % BLE_RX_BUFFER_SIZE;

        if (next_head != ble_rx_tail) {
            ble_rx_buffer[ble_rx_head] = rx_data;
            ble_rx_head = next_head;

            #ifdef BM71_DEBUG_RX
            LOG_DEBUG("BM71 RX: 0x%02X", rx_data);
            #endif
        } else {
            /* Buffer overflow */
            LOG_WARNING("BM71 RX buffer overflow");
        }

        /* RXNE flag is cleared by reading the data register */
    }

    /* If IDLE line detected, it means a complete message was received */
    if (LPUART1->ISR & UART_FLAG_IDLE) {
        /* For STM32L0 LPUART, properly clear the IDLE flag */
        LPUART1->ICR = UART_CLEAR_IDLEF;

        /* Signal that a complete message is ready for processing */
        if (ble_rx_head != ble_rx_tail) {
            ble_rx_data_ready = 1;
            bm7x_timer_flag = true;
            #ifdef BM71_DEBUG_RX
            LOG_DEBUG("BM71 message complete, %u bytes available", bm71_rx_available());
            #endif
        }
    }
}

void ble_rx_flush(void) {
    __disable_irq();  // Prevent ISR from writing during reset

    memset(ble_rx_buffer, 0, BLE_RX_BUFFER_SIZE);  // Optional but useful in debug
    ble_rx_head = 0;
    ble_rx_tail = 0;
    ble_rx_data_ready = 0;

    __enable_irq();   // Re-enable ISR

    // Also clear hardware FIFO just in case
    while (LPUART1->ISR & USART_ISR_RXNE) {
        volatile uint8_t dummy = LPUART1->RDR;
        (void)dummy;
    }
    //LOG_DEBUG("RX buffer and hardware register flushed");
}

#if BLE_CONFIG
/**
 * @brief Check if data is available in the BLE RX buffer
 * @return Number of bytes available in buffer
 */
uint16_t ble_rx_available(void)
{
    if (ble_rx_head >= ble_rx_tail) {
        return ble_rx_head - ble_rx_tail;
    } else {
        return BLE_RX_BUFFER_SIZE - ble_rx_tail + ble_rx_head;
    }
}


/**
 * @brief Read a byte from the BLE RX buffer
 * @return The next byte in the buffer, or -1 if buffer is empty
 */
int16_t ble_rx_read(void)
{
    if (ble_rx_head == ble_rx_tail) {
        return -1; /* Buffer empty */
    }

    uint8_t data = ble_rx_buffer[ble_rx_tail];
    ble_rx_tail = (ble_rx_tail + 1) % BLE_RX_BUFFER_SIZE;

    return data;
}
#else
uint8_t ble_rx_read(void)
{
    if (ble_rx_head == ble_rx_tail) {
#if !BLE_CONFIG
    	ble_rx_clear_ready();
#endif
        return 0; /* Buffer empty */
    }

    uint8_t data = ble_rx_buffer[ble_rx_tail];
    ble_rx_tail = (ble_rx_tail + 1) % BLE_RX_BUFFER_SIZE;

    return data;
}

uint8_t ble_rx_available(void)
{
    if (ble_rx_head >= ble_rx_tail) {
        return (uint8_t)(ble_rx_head - ble_rx_tail);
    } else {
        return (uint8_t)(BLE_RX_BUFFER_SIZE - ble_rx_tail + ble_rx_head);
    }
}
#endif
/**
 * @brief Read multiple bytes from the BLE RX buffer
 * @param buf Buffer to store read bytes
 * @param len Maximum number of bytes to read
 * @return Number of bytes actually read
 */
uint16_t ble_rx_read_buffer(uint8_t *buf, uint16_t len)
{
    uint16_t count = 0;

    while (count < len && ble_rx_head != ble_rx_tail) {
        buf[count++] = ble_rx_buffer[ble_rx_tail];
        ble_rx_tail = (ble_rx_tail + 1) % BLE_RX_BUFFER_SIZE;
    }

    return count;
}

/**
 * @brief Check if a complete message is ready to be processed
 * @return 1 if a message is ready, 0 otherwise
 */
uint8_t ble_rx_message_ready(void)
{
    return ble_rx_data_ready;
}

/**
 * @brief Clear the message ready flag after processing
 */
void ble_rx_clear_ready(void)
{
    ble_rx_data_ready = 0;
}

/**
 * @brief Initialize BLE UART reception
 * Call this during BLE initialization
 */
void ble_uart_rx_init(void)
{
    /* Clear buffers and flags */
    ble_rx_head = 0;
    ble_rx_tail = 0;
    ble_rx_data_ready = 0;
}

void ble_uart_tx(uint8_t* data, uint16_t size) {

	uint8_t wakeup_rst = 0;
    // Check if size is valid
    if (data == NULL || size == 0) {
        return;
    }
    // Make sure transmitter is enabled
    if (!(LPUART1->CR1 & USART_CR1_TE)) {
        // Enable transmitter if it's not already enabled
        LPUART1->CR1 |= USART_CR1_TE;
        // Wait a bit for transmitter to become ready
        uint32_t timeout = 1000;
        while (!(LPUART1->ISR & USART_ISR_TEACK) && timeout--) {}
    }

    uint32_t primask = __get_PRIMASK();
	__disable_irq();



    // Transmit all bytes in the buffer
    for (uint16_t i = 0; i < size; i++) {
        // Wait until TDR is empty (TXE flag is set)
        uint32_t timeout = 1000; // Timeout to prevent hanging
        while (!(LPUART1->ISR & USART_ISR_TXE) && timeout--) {
            if (timeout == 0) {
                // Release wake-up pin if timeout occurs
                BT_WAKEUP_GPIO_Port->BSRR = BLE_WAKEUP_Pin;
                return;
            }
        }
        if(!wakeup_rst){
        	// Wake up BM70/71 by pulling RX_IND pin low
			BT_WAKEUP_GPIO_Port->BRR = BLE_WAKEUP_Pin;
			//delay_us(2000);
			wakeup_rst = 1;
        }
        // Write data to transmit data register
        LPUART1->TDR = data[i];
    }

    // Wait until transmission complete (TC flag is set)
    // This ensures all data is sent before releasing the wake-up pin
    uint32_t timeout = 10000;
    while (!(LPUART1->ISR & USART_ISR_TC) && timeout--) {
        if (timeout == 0) {
            break;
        }
    }
    delay_us(100);
    // Release wake-up pin (set it high)
    BT_WAKEUP_GPIO_Port->BSRR = BLE_WAKEUP_Pin;

    // Restore previous interrupt state
	__set_PRIMASK(primask);
}

bool ble_write_ready(void) {
    /* Check if transmission complete (TC) or transmit data register empty (TXE) */
    return (LPUART1->ISR & (USART_ISR_TC | USART_ISR_TXE)) != 0;
}
