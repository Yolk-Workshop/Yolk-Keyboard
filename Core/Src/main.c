/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <assert.h>
#include "logger.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_lpuart1_rx;
DMA_HandleTypeDef hdma_lpuart1_tx;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
TIM_HandleTypeDef htim3;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern keyboard_state_t kb_state;

uint16_t row_pins[KEY_ROWS] = {R0_Pin, R1_Pin, R2_Pin, R3_Pin, R4_Pin, R5_Pin};

GPIO_TypeDef* const row_ports[KEY_ROWS] = {R0_GPIO_Port, R1_GPIO_Port, R2_GPIO_Port,
                                                    R3_GPIO_Port, R4_GPIO_Port, R5_GPIO_Port};

GPIO_TypeDef* const col_ports[KEY_COLS] = {
    C0_GPIO_Port, C1_GPIO_Port, C2_GPIO_Port, C3_GPIO_Port,
    C4_GPIO_Port, C5_GPIO_Port, C6_GPIO_Port, C7_GPIO_Port,
    C8_GPIO_Port, C9_GPIO_Port, C10_GPIO_Port, C11_GPIO_Port,
    C12_GPIO_Port, C13_GPIO_Port
};

const uint16_t col_pins[KEY_COLS] = {
    C0_Pin, C1_Pin, C2_Pin, C3_Pin, C4_Pin, C5_Pin, C6_Pin, C7_Pin,
    C8_Pin, C9_Pin, C10_Pin, C11_Pin, C12_Pin, C13_Pin
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void scanKeyMatrix(void);
static void Init_TIM3(void);
void sendUSBReport(void);
void sendBLEReport(void);
void initBluetooth(void);
void reportArbiter(void);
void CheckConnection(USBD_HandleTypeDef *pdev);
void resetRows(void);
void logger_output(const char *message);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t USBD_HID_SendReport(USBD_HandleTypeDef  *pdev,
	                                   uint8_t *report,
	                                   uint16_t len);

volatile uint8_t scan_flag = 0;
volatile uint8_t report_ready_flag = 0;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  LOG_INFO("All Peripherals Initialised");

  /* USER CODE BEGIN 2 */
  //init USB
  MX_USB_DEVICE_Init();
  CheckConnection(&hUsbDeviceFS);
  if(!kb_state.connection_mode){
	  MX_LPUART1_UART_Init();
  }
  initKeyboard();
  resetRows();
  LOG_INFO("Keyboard Hardware Initialised");

  //init timer 3
  Init_TIM3();
  LOG_INFO("TIMER 3 Initialised");
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  LOG_INFO("System ready and running");
  while (1)
  {
    /* USER CODE END WHILE */
	  if(scan_flag == 1){
		  scanKeyMatrix();
		  if(report_ready_flag){
			  reportArbiter();
			  report_ready_flag = 0;
			  //LOG_DEBUG("HID report sent");
		  }
		  scan_flag = 0;
		  //GPIOB->ODR ^= GPIO_PIN_12;
	  }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_LPUART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_HSI;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B07CB4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00B07CB4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

static void Init_TIM3(void) {

	__HAL_RCC_TIM3_CLK_ENABLE();

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    // For 2kHz (2000µs) with 32MHz clock:
    // 32MHz / 2kHz = 16000
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 32000 - 1;  // For 2000µs period
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    HAL_TIM_Base_Init(&htim3);  // Changed from PWM to Base

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

    // Set high priority interrupt
    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LMAT_RESET_GPIO_Port, LMAT_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LMAT_SDB_GPIO_Port, LMAT_SDB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, UART_MODE_SW_Pin|EEPROM_WC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, R0_Pin|R1_Pin|R2_Pin|R3_Pin
                          |R4_Pin|R5_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : C10_Pin C11_Pin C12_Pin C13_Pin
                           C8_Pin C9_Pin */
  GPIO_InitStruct.Pin = C10_Pin|C11_Pin|C12_Pin|C13_Pin
                          |C8_Pin|C9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : TOUCH_INT_Pin BMS_ALERT_Pin LMAT_INT_Pin */
  GPIO_InitStruct.Pin = TOUCH_INT_Pin|BMS_ALERT_Pin|LMAT_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LMAT_RESET_Pin */
  GPIO_InitStruct.Pin = LMAT_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LMAT_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BLE_STAT1_Pin BLE_STAT2_Pin BT_SIGNAL_GOOD_Pin */
  GPIO_InitStruct.Pin = BLE_STAT1_Pin|BLE_STAT2_Pin|BT_SIGNAL_GOOD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : VBUS_DETECT_Pin PROX_SENSE_INT_Pin */
  GPIO_InitStruct.Pin = VBUS_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PROX_SENSE_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LMAT_SDB_Pin */
  GPIO_InitStruct.Pin = LMAT_SDB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LMAT_SDB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UART_MODE_SW_Pin EEPROM_WC_Pin */
  GPIO_InitStruct.Pin = UART_MODE_SW_Pin|EEPROM_WC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BT_RX_INPUT_Pin AMB_I2C_INT_Pin BT_RESET_Pin BT_TX_ALERT_Pin */
  GPIO_InitStruct.Pin = BT_RX_INPUT_Pin|AMB_I2C_INT_Pin|BT_RESET_Pin|BT_TX_ALERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


  /*Configure GPIO pin : BT_PAIR_SW_Pin */
  GPIO_InitStruct.Pin = BT_PAIR_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BT_PAIR_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R0_Pin R1_Pin R2_Pin R3_Pin
                           R4_Pin R5_Pin */
  GPIO_InitStruct.Pin = R0_Pin|R1_Pin|R2_Pin|R3_Pin
                          |R4_Pin|R5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : C0_Pin */
  GPIO_InitStruct.Pin = C0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(C0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : C1_Pin C2_Pin C3_Pin C4_Pin
                           C5_Pin C6_Pin C7_Pin */
  GPIO_InitStruct.Pin = C1_Pin|C2_Pin|C3_Pin|C4_Pin
                          |C5_Pin|C6_Pin|C7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  // Configure GPIO pin 12 key stoke polling signal
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Enable and set EXTI interrupt priority
 HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
 HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * Decide whether to send USB or BLE report and send it
 */
void reportArbiter(void) {
    uint32_t current_time = HAL_GetTick();

    if (kb_state.output_mode == OUTPUT_USB &&
        (current_time - kb_state.last_report.usb_last_report >= HID_FS_BINTERVAL)) {
        sendUSBReport();
    }
    else if (kb_state.output_mode == OUTPUT_BLE &&
             (current_time - kb_state.last_report.ble_last_report >= HID_FS_BINTERVAL)) {
        sendBLEReport();
    }
}

/**
 * Send USB HID report if state has changed
 */
void sendUSBReport(void) {
    static hid_report_t last_report_usb = {0};
    uint32_t current_time = HAL_GetTick();

    // Compare current report with last report to avoid redundant sends
    if (memcmp(&kb_state.current_report, &last_report_usb, sizeof(hid_report_t)) != 0) {

    	//__disable_irq();
        uint8_t result = USBD_HID_SendReport(&hUsbDeviceFS,
                            (uint8_t*)&kb_state.current_report,
                            sizeof(hid_report_t));
        //__enable_irq();
        if (result == USBD_OK) {
            kb_state.last_report.usb_last_report = current_time;
            memcpy(&last_report_usb, &kb_state.current_report, sizeof(hid_report_t));
            clearReport();
        }
    }
}

/**
 * Send BLE HID report if state has changed
 */
void sendBLEReport(void) {
    static hid_report_t last_report = {0};
    uint32_t current_time = HAL_GetTick();

    if (memcmp(&kb_state.current_report, &last_report, sizeof(hid_report_t)) != 0) {
        uint8_t uart_buffer[sizeof(ble_hid_report_t) + 2];

        uart_buffer[0] = 0xFD;  // Start marker
        ble_report.report_id = 1;
        memcpy(&ble_report.report, &kb_state.current_report, sizeof(hid_report_t));
        memcpy(&uart_buffer[1], &ble_report, sizeof(ble_hid_report_t));
        uart_buffer[sizeof(ble_hid_report_t) + 1] = 0xFE;  // End marker

        if (HAL_UART_Transmit_DMA(&hlpuart1, uart_buffer,
                                 sizeof(ble_hid_report_t) + 2) == HAL_OK) {
            kb_state.last_report.ble_last_report = current_time;
            memcpy(&last_report, &kb_state.current_report, sizeof(hid_report_t));
            clearReport();
        }
    }
}

/**
 * Scan the keyboard matrix for key state changes
 */
void scanKeyMatrix(void) {
    for (uint8_t row = 0; row < KEY_ROWS; row++) {
        // Activate the current row
        row_ports[row]->ODR |= row_pins[row];

        // Small delay for stabilization
        for (volatile int i = 0; i < 200; i++) __asm("nop");

        // Scan columns
        for (uint8_t col = 0; col < KEY_COLS; col++) {
            bool new_state = (col_ports[col]->IDR & col_pins[col]);
            __disable_irq();
            debounceKey(row, col, new_state);
            __enable_irq();
        }

        // Deactivate row
        row_ports[row]->ODR &= ~row_pins[row];
    }
}

void CheckConnection(USBD_HandleTypeDef *pdev){
	if((VBUS_DETECT_GPIO_Port->IDR & VBUS_DETECT_Pin) == 0){
		// Check USB State
		if (pdev->dev_state == USBD_STATE_CONFIGURED) {
			kb_state.connection_mode = 0; // Host connected
			kb_state.output_mode = OUTPUT_USB;

			LOG_DEBUG("Connection Mode: USB");
			return;
		} else {
			kb_state.connection_mode = 1; // Host not fully enumerated
			kb_state.output_mode = OUTPUT_BLE;

			LOG_DEBUG("Connection Mode: BLE");
			return;
		}
	} else {
		kb_state.connection_mode = 1; // VBUS not detected
		kb_state.output_mode = OUTPUT_BLE;

		LOG_DEBUG("Connection Mode: BLE");
		return;
	}
}

void initBluetooth(void){
	MX_LPUART1_UART_Init();

	BT_RESET_GPIO_Port->ODR |= BT_RESET_Pin;
	LOG_DEBUG("Bluetooth Initialisation complete");
}

void resetRows(void){
	// Configure GPIO for rows
	for(int i = 0; i < KEY_ROWS; i++) {
		row_ports[i]->ODR &= ~row_pins[i];  // Start with all rows low
	}
}

void logger_output(const char *message) {
    HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  LOG_ERROR("Hard-fault Occurred!");
  while (1)
  {
		GPIOB->ODR &= ~GPIO_PIN_12;
		HAL_Delay(10);
		GPIOB->ODR &= ~GPIO_PIN_12;
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
