/**
* watchdog.c - Simple, efficient watchdog implementation
*
* Created on: Apr 10, 2025
* Author: bettysidepiece
* Modified: May 11, 2025
*/

#include "watchdog.h"
#include "stm32l0xx_hal.h"

#define IWDG_KEY_WRITE_ACCESS 0x5555U
#define IWDG_KEY_RELOAD       0xAAAAU
#define IWDG_KEY_START        0xCCCCU

#define IWDG_PRESCALER_4      0x00U // Divider by 4
#define IWDG_PRESCALER_8      0x01U // Divider by 8
#define IWDG_PRESCALER_16     0x02U // Divider by 16
#define IWDG_PRESCALER_32     0x03U // Divider by 32
#define IWDG_PRESCALER_64     0x04U // Divider by 64
#define IWDG_PRESCALER_128    0x05U // Divider by 128
#define IWDG_PRESCALER_256    0x06U // Divider by 256

/**
 * @brief Initialize the Independent Watchdog
 * @param timeout_ms: Watchdog timeout in milliseconds
 * @retval None
 */
void WDG_Init(uint32_t timeout_ms)
{
    uint8_t prescaler;
    uint16_t reload_value;
    uint32_t timeout_count;
    float tick_ms;

    // Calculate appropriate prescaler and reload values based on LSI clock (~40kHz)
    if (timeout_ms <= 125) {
        prescaler = IWDG_PRESCALER_4;      // Divide by 4 -> 10kHz counter clock
        tick_ms = 0.1f;                    // 1 tick = 0.1ms
    } else if (timeout_ms <= 250) {
        prescaler = IWDG_PRESCALER_8;      // Divide by 8 -> 5kHz counter clock
        tick_ms = 0.2f;                    // 1 tick = 0.2ms
    } else if (timeout_ms <= 500) {
        prescaler = IWDG_PRESCALER_16;     // Divide by 16 -> 2.5kHz counter clock
        tick_ms = 0.4f;                    // 1 tick = 0.4ms
    } else if (timeout_ms <= 1000) {
        prescaler = IWDG_PRESCALER_32;     // Divide by 32 -> 1.25kHz counter clock
        tick_ms = 0.8f;                    // 1 tick = 0.8ms
    } else if (timeout_ms <= 2000) {
        prescaler = IWDG_PRESCALER_64;     // Divide by 64 -> 625Hz counter clock
        tick_ms = 1.6f;                    // 1 tick = 1.6ms
    } else if (timeout_ms <= 4000) {
        prescaler = IWDG_PRESCALER_128;    // Divide by 128 -> 312.5Hz counter clock
        tick_ms = 3.2f;                    // 1 tick = 3.2ms
    } else {
        prescaler = IWDG_PRESCALER_256;    // Divide by 256 -> 156.25Hz counter clock
        tick_ms = 6.4f;                    // 1 tick = 6.4ms
    }

    // Calculate reload value
    reload_value = (uint16_t)(timeout_ms / tick_ms);

    // Cap reload value to maximum 12 bits (0xFFF = 4095)
    if (reload_value > 0xFFF) {
        reload_value = 0xFFF;
        LOG_WARNING("Watchdog reload value capped to 0xFFF, actual timeout will be %ums",
                   (uint32_t)(reload_value * tick_ms));
    }

    LOG_INFO("Initializing Watchdog: timeout=%ums, prescaler=%u, reload=0x%03X",
            (uint32_t)(reload_value * tick_ms), prescaler, reload_value);

    // Enable write access to IWDG registers
    IWDG->KR = IWDG_KEY_WRITE_ACCESS;

    // Set prescaler
    IWDG->PR = prescaler;

    // Set reload value
    IWDG->RLR = reload_value;

    // Wait for registers to be updated with timeout
    timeout_count = 0;
    while ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) && (timeout_count < 1000)) {
        timeout_count++;
        // Small delay to prevent tight loop
        for (volatile int i = 0; i < 10; i++) { }
    }

    // Check if timeout occurred
    if (timeout_count >= 1000) {
        LOG_WARNING("Watchdog register update timeout - configuration may be incomplete");
    }

    // Refresh the counter value
    IWDG->KR = IWDG_KEY_RELOAD;

    // Enable IWDG
    IWDG->KR = IWDG_KEY_START;

    LOG_INFO("Watchdog initialized successfully");
}

/**
 * @brief Refresh ("kick") the watchdog timer
 * @param None
 * @retval None
 */
inline void WDG_Refresh(void)
{
    // Simple refresh - just reload the counter
    IWDG->KR = IWDG_KEY_RELOAD;
}

/**
 * @brief Configure IWDG to be disabled in debug mode
 * @note This prevents watchdog reset during debugging
 * @param None
 * @retval None
 */
void WDG_DisableInDebug(void)
{
    // Enable DBGMCU clock
    __HAL_RCC_DBGMCU_CLK_ENABLE();

    // Set the IWDG_STOP bit to stop IWDG in debug mode
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

    LOG_INFO("Watchdog disabled in debug mode");
}
