/*
 * main.c
 *
 * Created on: Jan 7, 2025
 * Author: bettysidepiece
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <assert.h>

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void Init_TIM3(void);
static void Init_TIM21(void);

/* Private variables --------------------------------------------------------*/
// Global state flags
volatile uint8_t scan_flag = 0;
volatile uint8_t report_ready_flag = 0;
volatile ITStatus uart2_tc_flag = SET;

// Hardware interface handlers
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim21;
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_usart2_tx;
extern USBD_HandleTypeDef hUsbDeviceFS;

static log_buffer_t log_queue = {0};
void logger_output(const char *message);



/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART2_UART_Init();
    LOG_INFO("Core System Hardware Initialised");

    MX_I2C1_Init();
    MX_I2C2_Init();
    LOG_INFO("I2C Peripheral Initialised");

    /* Initialize USB and Bluetooth */
    MX_USB_DEVICE_Init();
    kb_state.connection_mode = 1;

    if(kb_state.connection_mode) {
        initBluetooth();
    }

    /* Initialize Keyboard Hardware */
    initKeyboard();
    resetRows();
    LOG_INFO("Keyboard Hardware Initialised");

    /* Initialize Timers */
    MX_TIM2_Init();
    Init_TIM21();
    Init_TIM3();
    LOG_INFO("TIMER 2, 3 & 21 Initialised");

    HAL_TIM_Base_Start(&htim21);
    HAL_TIM_Base_Start_IT(&htim3);

    LOG_INFO("System ready and running");

    /* Main loop */
    while (1) {
        if(scan_flag == 1) {
            scanKeyMatrix();
            if(report_ready_flag) {
                reportArbiter();
                report_ready_flag = 0;
            }
            scan_flag = 0;
        }
    }
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /* Configure the main internal regulator output voltage */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Initialize RCC Oscillators */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_8;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /* Initialize CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        Error_Handler();
    }

    /* Configure peripheral clock sources */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_LPUART1
                                |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_USB;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
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

    if (HAL_TIM_Base_Init(&htim21) != HAL_OK) {
        Error_Handler();
    }
}


static void Init_TIM3(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 16000 - 1;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    HAL_TIM_Base_Init(&htim3);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

    // Set high priority interrupt
    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}


static void MX_TIM2_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 65535;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
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

    if (HAL_UART_Init(&hlpuart1) != HAL_OK) {
        Error_Handler();
    }
}


static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}


/* I2C Initialization Functions -------------------------------------------*/
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00B07CB4;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
        Error_Handler();
    }
}


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

    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
        Error_Handler();
    }
}


/* DMA Initialization Function --------------------------------------------*/
static void MX_DMA_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
}


/* GPIO Initialization Function ------------------------------------------*/
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO Ports Clock */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* Configure Output Levels */
    HAL_GPIO_WritePin(LMAT_RESET_GPIO_Port, LMAT_RESET_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LMAT_SDB_GPIO_Port, LMAT_SDB_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, UART_MODE_SW_Pin|EEPROM_WC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, R0_Pin|R1_Pin|R2_Pin|R3_Pin|R4_Pin|R5_Pin, GPIO_PIN_SET);

    /* Configure Column Pins */
    GPIO_InitStruct.Pin = C10_Pin|C11_Pin|C12_Pin|C13_Pin|C8_Pin|C9_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* Configure Interrupt Pins */
    GPIO_InitStruct.Pin = TOUCH_INT_Pin|BMS_ALERT_Pin|LMAT_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Configure Control Pins */
    GPIO_InitStruct.Pin = LMAT_RESET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LMAT_RESET_GPIO_Port, &GPIO_InitStruct);

    /* Configure Bluetooth Status Pins */
    GPIO_InitStruct.Pin = BLE_STAT1_Pin|BLE_STAT2_Pin|BT_SIGNAL_GOOD_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Configure USB and Proximity Pins */
    GPIO_InitStruct.Pin = VBUS_DETECT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PROX_SENSE_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Configure Row Output Pins */
    GPIO_InitStruct.Pin = R0_Pin|R1_Pin|R2_Pin|R3_Pin|R4_Pin|R5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Configure Additional Column Input Pins */
    GPIO_InitStruct.Pin = C0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(C0_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = C1_Pin|C2_Pin|C3_Pin|C4_Pin|C5_Pin|C6_Pin|C7_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Configure Keystroke Polling Signal Pin */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Configure EXTI Interrupt */
    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}


void logger_output(const char *message)
{
    uint32_t primask;
    uint16_t len = strlen(message);

    primask = __get_PRIMASK();
    __disable_irq();

    // Check if we have space in the queue
    if (log_queue.buffer_count < LOG_BUFFER_COUNT) {
        // Copy message to next available buffer
        strncpy(log_queue.buffers[log_queue.write_index],
                message,
                LOG_BUFFER_SIZE - 1);
        log_queue.buffers[log_queue.write_index][LOG_BUFFER_SIZE - 1] = '\0';

        log_queue.write_index = (log_queue.write_index + 1) % LOG_BUFFER_COUNT;
        log_queue.buffer_count++;

        // If DMA not busy, start transmission
        if (!log_queue.dma_busy && uart2_tc_flag == SET) {
            len = strlen(log_queue.buffers[log_queue.read_index]);
            if (HAL_UART_Transmit_DMA(&huart2,
                (uint8_t*)log_queue.buffers[log_queue.read_index],
                len) == HAL_OK) {
                log_queue.dma_busy = 1;
                uart2_tc_flag = RESET;
            }
        }
    }

    __set_PRIMASK(primask);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t len;

    if (huart == &huart2) {
        uart2_tc_flag = SET;
        log_queue.dma_busy = 0;

        // Critical section
        __disable_irq();

        if (log_queue.buffer_count > 0) {
            log_queue.buffer_count--;
            log_queue.read_index = (log_queue.read_index + 1) % LOG_BUFFER_COUNT;

            if (log_queue.buffer_count > 0) {
                len = strlen(log_queue.buffers[log_queue.read_index]);
                if (HAL_UART_Transmit_DMA(&huart2,
                    (uint8_t*)log_queue.buffers[log_queue.read_index],
                    len) == HAL_OK) {
                    log_queue.dma_busy = 1;
                    uart2_tc_flag = RESET;
                }
            }
        }

        __enable_irq();
    }
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    LOG_DEBUG("Hard-fault Occurred!\n");
    __disable_irq();
    while (1) {
        HAL_NVIC_EnableIRQ(USART2_IRQn);
        LOG_DEBUG("RESET/POWER Cycling Required\n");
        HAL_Delay(60000);
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add implementation to report the file name and line number */
    LOG_ERROR("Wrong parameters value: file %s on line %d\r\n", file, line);
}
#endif
