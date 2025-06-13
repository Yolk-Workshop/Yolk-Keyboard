#include "m24512e_driver.h"

/* Private function prototypes */
static m24512e_status_t m24512e_write_register(m24512e_handle_t *handle, uint8_t addr_high, uint8_t data);
static m24512e_status_t m24512e_read_register(m24512e_handle_t *handle, uint8_t addr_high, uint8_t *data);
static uint8_t m24512e_get_device_select_code(uint8_t base_code, uint8_t chip_addr, bool read_operation);
static void m24512e_crc_init(void);



/*Initialize system with CRC peripheral enabled
 * @note Call this function during system initialization
 */
void m24512e_system_init(void)
{
    /* Enable CRC peripheral clock */
    __HAL_RCC_CRC_CLK_ENABLE();

    /* Configure CRC peripheral for CRC-16 calculations */
    m24512e_crc_init();
}

/**
 * @brief Initialize M24512E-F EEPROM driver
 * @param handle Pointer to device handle
 * @param hi2c Pointer to I2C handle (I2C2)
 * @param device_addr Device chip enable address (0-7)
 * @return Status code
 */
m24512e_status_t m24512e_init(m24512e_handle_t *handle, I2C_HandleTypeDef *hi2c, uint8_t device_addr)
{
    if (!handle || !hi2c || device_addr > 7) {
        return M24512E_ERROR;
    }

    handle->hi2c = hi2c;
    handle->device_address = device_addr;
    handle->write_protection_enabled = false;
    handle->last_write_time = 0;

    /* Wait for power-up wake time */
    HAL_Delay(M24512E_WAKE_UP_DELAY);

    /* Verify device is present and responding */
    if (!m24512e_is_device_ready(handle)) {
        return M24512E_ERROR;
    }

    return M24512E_OK;
}

/**
 * @brief Deinitialize M24512E-F EEPROM driver
 * @param handle Pointer to device handle
 * @return Status code
 */
m24512e_status_t m24512e_deinit(m24512e_handle_t *handle)
{
    if (!handle) {
        return M24512E_ERROR;
    }

    handle->hi2c = NULL;
    handle->device_address = 0;
    handle->write_protection_enabled = false;
    handle->last_write_time = 0;

    return M24512E_OK;
}

/**
 * @brief Check if device is ready for communication
 * @param handle Pointer to device handle
 * @return true if device is ready, false otherwise
 */
bool m24512e_is_device_ready(m24512e_handle_t *handle)
{
    if (!handle || !handle->hi2c) {
        return false;
    }

    uint8_t device_select = m24512e_get_device_select_code(M24512E_MEMORY_DEVICE_CODE,
                                                          handle->device_address, false);

    return (HAL_I2C_IsDeviceReady(handle->hi2c, device_select, 3, M24512E_I2C_TIMEOUT) == HAL_OK);
}

/**
 * @brief Write single byte to memory
 * @param handle Pointer to device handle
 * @param address Memory address (0-65535)
 * @param data Data byte to write
 * @return Status code
 */
m24512e_status_t m24512e_write_byte(m24512e_handle_t *handle, uint16_t address, uint8_t data)
{
    if (!handle || !handle->hi2c || address >= M24512E_DEVICE_SIZE) {
        return M24512E_INVALID_ADDRESS;
    }

    uint8_t device_select = m24512e_get_device_select_code(M24512E_MEMORY_DEVICE_CODE,
                                                          handle->device_address, false);
    uint8_t buffer[3] = {
        (uint8_t)(address >> 8),    // Address high byte
        (uint8_t)(address & 0xFF),  // Address low byte
        data                        // Data byte
    };

    HAL_StatusTypeDef hal_status = HAL_I2C_Master_Transmit(handle->hi2c, device_select,
                                                          buffer, 3, M24512E_I2C_TIMEOUT);

    if (hal_status != HAL_OK) {
        return (hal_status == HAL_TIMEOUT) ? M24512E_TIMEOUT : M24512E_ERROR;
    }

    handle->last_write_time = HAL_GetTick();
    return M24512E_OK;
}

/**
 * @brief Write page to memory
 * @param handle Pointer to device handle
 * @param address Starting memory address
 * @param data Pointer to data buffer
 * @param size Number of bytes to write (max 128 for same page)
 * @return Status code
 */
m24512e_status_t m24512e_write_page(m24512e_handle_t *handle, uint16_t address,
                                   const uint8_t *data, uint16_t size)
{
    if (!handle || !handle->hi2c || !data || address >= M24512E_DEVICE_SIZE || size == 0) {
        return M24512E_ERROR;
    }

    /* Check if write crosses page boundary */
    uint16_t page_start = address & ~(M24512E_PAGE_SIZE - 1);
    uint16_t page_end = page_start + M24512E_PAGE_SIZE - 1;

    if (address + size - 1 > page_end) {
        return M24512E_INVALID_SIZE;
    }

    if (size > M24512E_PAGE_SIZE) {
        return M24512E_INVALID_SIZE;
    }

    uint8_t device_select = m24512e_get_device_select_code(M24512E_MEMORY_DEVICE_CODE,
                                                          handle->device_address, false);

    /* Prepare transmission buffer */
    uint8_t tx_buffer[M24512E_PAGE_SIZE + 2];
    tx_buffer[0] = (uint8_t)(address >> 8);
    tx_buffer[1] = (uint8_t)(address & 0xFF);

    for (uint16_t i = 0; i < size; i++) {
        tx_buffer[i + 2] = data[i];
    }

    HAL_StatusTypeDef hal_status = HAL_I2C_Master_Transmit(handle->hi2c, device_select,
                                                          tx_buffer, size + 2, M24512E_I2C_TIMEOUT);

    if (hal_status != HAL_OK) {
        return (hal_status == HAL_TIMEOUT) ? M24512E_TIMEOUT : M24512E_ERROR;
    }

    handle->last_write_time = HAL_GetTick();
    return M24512E_OK;
}

/**
 * @brief Write page with CRC16 verification
 * @param handle Pointer to device handle
 * @param address Starting memory address
 * @param data Pointer to data buffer
 * @param size Number of bytes to write
 * @return Status code
 */
m24512e_status_t m24512e_write_page_with_crc(m24512e_handle_t *handle, uint16_t address,
                                            const uint8_t *data, uint16_t size)
{
    m24512e_status_t status = m24512e_write_page(handle, address, data, size);
    if (status != M24512E_OK) {
        return status;
    }

    /* Wait for write completion */
    status = m24512e_wait_write_complete(handle);
    if (status != M24512E_OK) {
        return status;
    }

    /* Verify with CRC */
    uint8_t read_buffer[M24512E_PAGE_SIZE];
    status = m24512e_read_sequential(handle, address, read_buffer, size);
    if (status != M24512E_OK) {
        return status;
    }

    uint16_t original_crc = m24512e_calculate_crc16(data, size);
    uint16_t read_crc = m24512e_calculate_crc16(read_buffer, size);

    return (original_crc == read_crc) ? M24512E_OK : M24512E_CRC_ERROR;
}

/**
 * @brief Read single byte from memory
 * @param handle Pointer to device handle
 * @param address Memory address
 * @param data Pointer to store read data
 * @return Status code
 */
m24512e_status_t m24512e_read_byte(m24512e_handle_t *handle, uint16_t address, uint8_t *data)
{
    if (!handle || !handle->hi2c || !data || address >= M24512E_DEVICE_SIZE) {
        return M24512E_ERROR;
    }

    uint8_t device_select_write = m24512e_get_device_select_code(M24512E_MEMORY_DEVICE_CODE,
                                                               handle->device_address, false);
    uint8_t device_select_read = m24512e_get_device_select_code(M24512E_MEMORY_DEVICE_CODE,
                                                              handle->device_address, true);

    uint8_t addr_buffer[2] = {
        (uint8_t)(address >> 8),
        (uint8_t)(address & 0xFF)
    };

    /* Dummy write to set address */
    HAL_StatusTypeDef hal_status = HAL_I2C_Master_Transmit(handle->hi2c, device_select_write,
                                                          addr_buffer, 2, M24512E_I2C_TIMEOUT);
    if (hal_status != HAL_OK) {
        return (hal_status == HAL_TIMEOUT) ? M24512E_TIMEOUT : M24512E_ERROR;
    }

    /* Read data */
    hal_status = HAL_I2C_Master_Receive(handle->hi2c, device_select_read, data, 1, M24512E_I2C_TIMEOUT);

    if (hal_status != HAL_OK) {
        return (hal_status == HAL_TIMEOUT) ? M24512E_TIMEOUT : M24512E_ERROR;
    }

    return M24512E_OK;
}

/**
 * @brief Read sequential bytes from memory
 * @param handle Pointer to device handle
 * @param address Starting memory address
 * @param data Pointer to data buffer
 * @param size Number of bytes to read
 * @return Status code
 */
m24512e_status_t m24512e_read_sequential(m24512e_handle_t *handle, uint16_t address,
                                        uint8_t *data, uint16_t size)
{
    if (!handle || !handle->hi2c || !data || address >= M24512E_DEVICE_SIZE || size == 0) {
        return M24512E_ERROR;
    }

    if (address + size > M24512E_DEVICE_SIZE) {
        return M24512E_INVALID_SIZE;
    }

    uint8_t device_select_write = m24512e_get_device_select_code(M24512E_MEMORY_DEVICE_CODE,
                                                               handle->device_address, false);
    uint8_t device_select_read = m24512e_get_device_select_code(M24512E_MEMORY_DEVICE_CODE,
                                                              handle->device_address, true);

    uint8_t addr_buffer[2] = {
        (uint8_t)(address >> 8),
        (uint8_t)(address & 0xFF)
    };

    /* Dummy write to set address */
    HAL_StatusTypeDef hal_status = HAL_I2C_Master_Transmit(handle->hi2c, device_select_write,
                                                          addr_buffer, 2, M24512E_I2C_TIMEOUT);
    if (hal_status != HAL_OK) {
        return (hal_status == HAL_TIMEOUT) ? M24512E_TIMEOUT : M24512E_ERROR;
    }

    /* Sequential read */
    hal_status = HAL_I2C_Master_Receive(handle->hi2c, device_select_read, data, size, M24512E_I2C_TIMEOUT);

    if (hal_status != HAL_OK) {
        return (hal_status == HAL_TIMEOUT) ? M24512E_TIMEOUT : M24512E_ERROR;
    }

    return M24512E_OK;
}

/**
 * @brief Read sequential bytes with CRC16 verification
 * @param handle Pointer to device handle
 * @param address Starting memory address
 * @param data Pointer to data buffer
 * @param size Number of bytes to read
 * @return Status code
 */
m24512e_status_t m24512e_read_sequential_with_crc(m24512e_handle_t *handle, uint16_t address,
                                                 uint8_t *data, uint16_t size)
{
    m24512e_status_t status = m24512e_read_sequential(handle, address, data, size);
    if (status != M24512E_OK) {
        return status;
    }

    /* Read again for verification */
    uint8_t verify_buffer[size];
    status = m24512e_read_sequential(handle, address, verify_buffer, size);
    if (status != M24512E_OK) {
        return status;
    }

    /* Compare CRCs */
    uint16_t first_crc = m24512e_calculate_crc16(data, size);
    uint16_t second_crc = m24512e_calculate_crc16(verify_buffer, size);

    return (first_crc == second_crc) ? M24512E_OK : M24512E_CRC_ERROR;
}

/**
 * @brief Read current address (using internal address counter)
 * @param handle Pointer to device handle
 * @param data Pointer to store read data
 * @return Status code
 */
m24512e_status_t m24512e_read_current(m24512e_handle_t *handle, uint8_t *data)
{
    if (!handle || !handle->hi2c || !data) {
        return M24512E_ERROR;
    }

    uint8_t device_select_read = m24512e_get_device_select_code(M24512E_MEMORY_DEVICE_CODE,
                                                              handle->device_address, true);

    HAL_StatusTypeDef hal_status = HAL_I2C_Master_Receive(handle->hi2c, device_select_read,
                                                         data, 1, M24512E_I2C_TIMEOUT);

    if (hal_status != HAL_OK) {
        return (hal_status == HAL_TIMEOUT) ? M24512E_TIMEOUT : M24512E_ERROR;
    }

    return M24512E_OK;
}

/**
 * @brief Read Device Type Identifier register
 * @param handle Pointer to device handle
 * @param dti_reg Pointer to DTI register structure
 * @return Status code
 */
m24512e_status_t m24512e_read_dti_register(m24512e_handle_t *handle, m24512e_dti_reg_t *dti_reg)
{
    if (!handle || !dti_reg) {
        return M24512E_ERROR;
    }

    uint8_t reg_value;
    m24512e_status_t status = m24512e_read_register(handle, M24512E_DTI_ADDRESS_CODE, &reg_value);

    if (status == M24512E_OK) {
        dti_reg->dti3 = (reg_value >> 7) & 0x01;
        dti_reg->dti2 = (reg_value >> 6) & 0x01;
        dti_reg->dti1 = (reg_value >> 5) & 0x01;
        dti_reg->dti0 = (reg_value >> 4) & 0x01;
        dti_reg->reserved = (reg_value >> 1) & 0x07;
        dti_reg->dtil = reg_value & 0x01;
    }

    return status;
}

/**
 * @brief Read Configurable Device Address register
 * @param handle Pointer to device handle
 * @param cda_reg Pointer to CDA register structure
 * @return Status code
 */
m24512e_status_t m24512e_read_cda_register(m24512e_handle_t *handle, m24512e_cda_reg_t *cda_reg)
{
    if (!handle || !cda_reg) {
        return M24512E_ERROR;
    }

    uint8_t reg_value;
    m24512e_status_t status = m24512e_read_register(handle, M24512E_CDA_ADDRESS_CODE, &reg_value);

    if (status == M24512E_OK) {
        cda_reg->dal = reg_value & 0x01;
        cda_reg->c0 = (reg_value >> 1) & 0x01;
        cda_reg->c1 = (reg_value >> 2) & 0x01;
        cda_reg->c2 = (reg_value >> 3) & 0x01;
        cda_reg->reserved = (reg_value >> 4) & 0x0F;
    }

    return status;
}

/**
 * @brief Write Configurable Device Address register
 * @param handle Pointer to device handle
 * @param cda_reg Pointer to CDA register structure
 * @return Status code
 */
m24512e_status_t m24512e_write_cda_register(m24512e_handle_t *handle, const m24512e_cda_reg_t *cda_reg)
{
    if (!handle || !cda_reg) {
        return M24512E_ERROR;
    }

    uint8_t reg_value = (cda_reg->dal & 0x01) |
                       ((cda_reg->c0 & 0x01) << 1) |
                       ((cda_reg->c1 & 0x01) << 2) |
                       ((cda_reg->c2 & 0x01) << 3);

    return m24512e_write_register(handle, M24512E_CDA_ADDRESS_CODE, reg_value);
}

/**
 * @brief Read Software Write Protection register
 * @param handle Pointer to device handle
 * @param swp_reg Pointer to SWP register structure
 * @return Status code
 */
m24512e_status_t m24512e_read_swp_register(m24512e_handle_t *handle, m24512e_swp_reg_t *swp_reg)
{
    if (!handle || !swp_reg) {
        return M24512E_ERROR;
    }

    uint8_t reg_value;
    m24512e_status_t status = m24512e_read_register(handle, M24512E_SWP_ADDRESS_CODE, &reg_value);

    if (status == M24512E_OK) {
        swp_reg->wpl = reg_value & 0x01;
        swp_reg->bp0 = (reg_value >> 1) & 0x01;
        swp_reg->bp1 = (reg_value >> 2) & 0x01;
        swp_reg->wpa = (reg_value >> 3) & 0x01;
        swp_reg->reserved = (reg_value >> 4) & 0x0F;
    }

    return status;
}

/**
 * @brief Write Software Write Protection register
 * @param handle Pointer to device handle
 * @param swp_reg Pointer to SWP register structure
 * @return Status code
 */
m24512e_status_t m24512e_write_swp_register(m24512e_handle_t *handle, const m24512e_swp_reg_t *swp_reg)
{
    if (!handle || !swp_reg) {
        return M24512E_ERROR;
    }

    uint8_t reg_value = (swp_reg->wpl & 0x01) |
                       ((swp_reg->bp0 & 0x01) << 1) |
                       ((swp_reg->bp1 & 0x01) << 2) |
                       ((swp_reg->wpa & 0x01) << 3);

    return m24512e_write_register(handle, M24512E_SWP_ADDRESS_CODE, reg_value);
}

/**
 * @brief Write to identification page
 * @param handle Pointer to device handle
 * @param address Address within ID page (0-127)
 * @param data Pointer to data buffer
 * @param size Number of bytes to write
 * @return Status code
 */
m24512e_status_t m24512e_write_id_page(m24512e_handle_t *handle, uint8_t address,
                                      const uint8_t *data, uint8_t size)
{
    if (!handle || !handle->hi2c || !data || address >= M24512E_ID_PAGE_SIZE) {
        return M24512E_ERROR;
    }

    if (address + size > M24512E_ID_PAGE_SIZE) {
        return M24512E_INVALID_SIZE;
    }

    uint8_t device_select = m24512e_get_device_select_code(M24512E_FEATURE_DEVICE_CODE,
                                                          handle->device_address, false);

    uint8_t tx_buffer[M24512E_ID_PAGE_SIZE + 2];
    tx_buffer[0] = M24512E_ID_PAGE_ADDRESS_CODE;
    tx_buffer[1] = address;

    for (uint8_t i = 0; i < size; i++) {
        tx_buffer[i + 2] = data[i];
    }

    HAL_StatusTypeDef hal_status = HAL_I2C_Master_Transmit(handle->hi2c, device_select,
                                                          tx_buffer, size + 2, M24512E_I2C_TIMEOUT);

    if (hal_status != HAL_OK) {
        return (hal_status == HAL_TIMEOUT) ? M24512E_TIMEOUT : M24512E_ERROR;
    }

    handle->last_write_time = HAL_GetTick();
    return M24512E_OK;
}

/**
 * @brief Read from identification page
 * @param handle Pointer to device handle
 * @param address Address within ID page (0-127)
 * @param data Pointer to data buffer
 * @param size Number of bytes to read
 * @return Status code
 */
m24512e_status_t m24512e_read_id_page(m24512e_handle_t *handle, uint8_t address,
                                     uint8_t *data, uint8_t size)
{
    if (!handle || !handle->hi2c || !data || address >= M24512E_ID_PAGE_SIZE) {
        return M24512E_ERROR;
    }

    if (address + size > M24512E_ID_PAGE_SIZE) {
        return M24512E_INVALID_SIZE;
    }

    uint8_t device_select_write = m24512e_get_device_select_code(M24512E_FEATURE_DEVICE_CODE,
                                                               handle->device_address, false);
    uint8_t device_select_read = m24512e_get_device_select_code(M24512E_FEATURE_DEVICE_CODE,
                                                              handle->device_address, true);

    uint8_t addr_buffer[2] = {
        M24512E_ID_PAGE_ADDRESS_CODE,
        address
    };

    /* Dummy write to set address */
    HAL_StatusTypeDef hal_status = HAL_I2C_Master_Transmit(handle->hi2c, device_select_write,
                                                          addr_buffer, 2, M24512E_I2C_TIMEOUT);
    if (hal_status != HAL_OK) {
        return (hal_status == HAL_TIMEOUT) ? M24512E_TIMEOUT : M24512E_ERROR;
    }

    /* Read data */
    hal_status = HAL_I2C_Master_Receive(handle->hi2c, device_select_read, data, size, M24512E_I2C_TIMEOUT);

    if (hal_status != HAL_OK) {
        return (hal_status == HAL_TIMEOUT) ? M24512E_TIMEOUT : M24512E_ERROR;
    }

    return M24512E_OK;
}

/**
 * @brief Lock identification page permanently
 * @param handle Pointer to device handle
 * @return Status code
 */
m24512e_status_t m24512e_lock_id_page(m24512e_handle_t *handle)
{
    if (!handle || !handle->hi2c) {
        return M24512E_ERROR;
    }

    uint8_t device_select = m24512e_get_device_select_code(M24512E_FEATURE_DEVICE_CODE,
                                                          handle->device_address, false);

    uint8_t lock_command[3] = {
        M24512E_ID_LOCK_ADDRESS_CODE,
        0x00,  // Don't care address
        0x02   // Lock bit pattern (xxxx xx1x)
    };

    HAL_StatusTypeDef hal_status = HAL_I2C_Master_Transmit(handle->hi2c, device_select,
                                                          lock_command, 3, M24512E_I2C_TIMEOUT);

    if (hal_status != HAL_OK) {
        return (hal_status == HAL_TIMEOUT) ? M24512E_TIMEOUT : M24512E_ERROR;
    }

    handle->last_write_time = HAL_GetTick();
    return M24512E_OK;
}

/**
 * @brief Check if identification page is locked
 * @param handle Pointer to device handle
 * @return true if locked, false if unlocked or error
 */
bool m24512e_is_id_page_locked(m24512e_handle_t *handle)
{
    if (!handle || !handle->hi2c) {
        return false;
    }

    uint8_t device_select = m24512e_get_device_select_code(M24512E_FEATURE_DEVICE_CODE,
                                                          handle->device_address, false);

    uint8_t test_command[3] = {
        M24512E_ID_LOCK_ADDRESS_CODE,
        0x00,  // Don't care address
        0xFF   // Test data
    };

    /* Send test write command - if locked, will get NO ACK */
    HAL_StatusTypeDef hal_status = HAL_I2C_Master_Transmit(handle->hi2c, device_select,
                                                          test_command, 3, M24512E_I2C_TIMEOUT);

    /* Send start-stop sequence to reset device state */
    HAL_I2C_Master_Transmit(handle->hi2c, device_select, NULL, 0, M24512E_I2C_TIMEOUT);

    /* If command was NACKed, page is locked */
    return (hal_status != HAL_OK);
}

/**
 * @brief Wait for write cycle completion using ACK polling
 * @param handle Pointer to device handle
 * @return Status code
 */
m24512e_status_t m24512e_wait_write_complete(m24512e_handle_t *handle)
{
    if (!handle || !handle->hi2c) {
        return M24512E_ERROR;
    }

    uint32_t start_time = HAL_GetTick();
    uint8_t device_select = m24512e_get_device_select_code(M24512E_MEMORY_DEVICE_CODE,
                                                          handle->device_address, false);

    /* Poll for ACK response */
    while ((HAL_GetTick() - start_time) < M24512E_WRITE_CYCLE_TIME) {
        if (HAL_I2C_IsDeviceReady(handle->hi2c, device_select, 1, 1) == HAL_OK) {
            return M24512E_OK;
        }
        HAL_Delay(1);
    }

    return M24512E_TIMEOUT;
}

/**
 * @brief Initialize CRC peripheral for CRC-16 calculation
 * @note Configure STM32L072 CRC peripheral for 16-bit polynomial
 */
static void m24512e_crc_init(void)
{
    /* Configure CRC for CRC-16 CCITT polynomial */
    CRC->CR = CRC_CR_POLYSIZE_0;               // 16-bit polynomial (POLYSIZE = 01)
    CRC->POL = CRC16_POLYNOMIAL;               // Set CRC-16 polynomial (0x1021)
    CRC->INIT = 0xFFFFUL;                      // Set initial value

    /* Reset CRC calculation unit */
    CRC->CR |= CRC_CR_RESET;
}

/**
 * @brief Calculate CRC16 using STM32L072 hardware CRC unit
 * @param data Pointer to data buffer
 * @param size Number of bytes
 * @return CRC16 value
 */
uint16_t m24512e_calculate_crc16(const uint8_t *data, uint16_t size)
{
    if (!data || size == 0) {
        return 0;
    }

    /* Ensure CRC is initialized */
    static bool crc_initialized = false;
    if (!crc_initialized) {
        m24512e_crc_init();
        crc_initialized = true;
    }

    /* Reset CRC calculation */
    CRC->CR |= CRC_CR_RESET;

    /* Process data byte by byte for CRC-16 */
    volatile uint8_t *crc_dr_8 = (volatile uint8_t *)&CRC->DR;

    for (uint16_t i = 0; i < size; i++) {
        *crc_dr_8 = data[i];
    }

    /* Return 16-bit CRC result (lower 16 bits of DR register) */
    return (uint16_t)(CRC->DR & 0xFFFF);
}

/**
 * @brief Verify CRC16 of data
 * @param data Pointer to data buffer
 * @param size Number of bytes
 * @param expected_crc Expected CRC16 value
 * @return true if CRC matches, false otherwise
 */
bool m24512e_verify_crc16(const uint8_t *data, uint16_t size, uint16_t expected_crc)
{
    uint16_t calculated_crc = m24512e_calculate_crc16(data, size);
    return (calculated_crc == expected_crc);
}

/**
 * @brief Write and verify data with automatic page handling
 * @param handle Pointer to device handle
 * @param address Starting address
 * @param data Pointer to data buffer
 * @param size Total number of bytes to write
 * @return Status code
 */
m24512e_status_t m24512e_write_data_safe(m24512e_handle_t *handle, uint16_t address,
                                        const uint8_t *data, uint16_t size)
{
    if (!handle || !data || size == 0) {
        return M24512E_ERROR;
    }

    uint16_t bytes_written = 0;
    m24512e_status_t status;

    while (bytes_written < size) {
        /* Calculate current page boundaries */
        uint16_t current_addr = address + bytes_written;
        uint16_t page_start = current_addr & ~(M24512E_PAGE_SIZE - 1);
        uint16_t page_end = page_start + M24512E_PAGE_SIZE - 1;
        uint16_t bytes_to_page_end = page_end - current_addr + 1;

        /* Determine how many bytes to write in this operation */
        uint16_t bytes_to_write = (size - bytes_written > bytes_to_page_end) ?
                                 bytes_to_page_end : (size - bytes_written);

        /* Write page with CRC verification */
        status = m24512e_write_page_with_crc(handle, current_addr,
                                           &data[bytes_written], bytes_to_write);
        if (status != M24512E_OK) {
            return status;
        }

        bytes_written += bytes_to_write;

        /* Wait for write completion before next page */
        if (bytes_written < size) {
            status = m24512e_wait_write_complete(handle);
            if (status != M24512E_OK) {
                return status;
            }
        }
    }

    return M24512E_OK;
}

/* Private Functions */

/**
 * @brief Write to feature register
 * @param handle Pointer to device handle
 * @param addr_high High address byte for register selection
 * @param data Data byte to write
 * @return Status code
 */
static m24512e_status_t m24512e_write_register(m24512e_handle_t *handle, uint8_t addr_high, uint8_t data)
{
    uint8_t device_select = m24512e_get_device_select_code(M24512E_FEATURE_DEVICE_CODE,
                                                          handle->device_address, false);

    uint8_t buffer[3] = {
        addr_high,
        0x00,  // Don't care low address
        data
    };

    HAL_StatusTypeDef hal_status = HAL_I2C_Master_Transmit(handle->hi2c, device_select,
                                                          buffer, 3, M24512E_I2C_TIMEOUT);

    if (hal_status != HAL_OK) {
        return (hal_status == HAL_TIMEOUT) ? M24512E_TIMEOUT : M24512E_ERROR;
    }

    handle->last_write_time = HAL_GetTick();
    return M24512E_OK;
}

/**
 * @brief Read from feature register
 * @param handle Pointer to device handle
 * @param addr_high High address byte for register selection
 * @param data Pointer to store read data
 * @return Status code
 */
static m24512e_status_t m24512e_read_register(m24512e_handle_t *handle, uint8_t addr_high, uint8_t *data)
{
    uint8_t device_select_write = m24512e_get_device_select_code(M24512E_FEATURE_DEVICE_CODE,
                                                               handle->device_address, false);
    uint8_t device_select_read = m24512e_get_device_select_code(M24512E_FEATURE_DEVICE_CODE,
                                                              handle->device_address, true);

    uint8_t addr_buffer[2] = {
        addr_high,
        0x00  // Don't care low address
    };

    /* Dummy write to set register address */
    HAL_StatusTypeDef hal_status = HAL_I2C_Master_Transmit(handle->hi2c, device_select_write,
                                                          addr_buffer, 2, M24512E_I2C_TIMEOUT);
    if (hal_status != HAL_OK) {
        return (hal_status == HAL_TIMEOUT) ? M24512E_TIMEOUT : M24512E_ERROR;
    }

    /* Read register data */
    hal_status = HAL_I2C_Master_Receive(handle->hi2c, device_select_read, data, 1, M24512E_I2C_TIMEOUT);

    if (hal_status != HAL_OK) {
        return (hal_status == HAL_TIMEOUT) ? M24512E_TIMEOUT : M24512E_ERROR;
    }

    return M24512E_OK;
}

/**
 * @brief Generate device select code
 * @param base_code Base device code (0xA0 for memory, 0xB0 for features)
 * @param chip_addr Chip enable address (0-7)
 * @param read_operation true for read, false for write
 * @return Device select code
 */
static uint8_t m24512e_get_device_select_code(uint8_t base_code, uint8_t chip_addr, bool read_operation)
{
    uint8_t device_select = base_code | ((chip_addr & 0x07) << 1);

    if (read_operation) {
        device_select |= 0x01;  // Set R/W bit for read
    }

    return device_select;
}
