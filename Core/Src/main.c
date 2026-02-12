/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "serial_log.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint16_t analog0;
  uint16_t analog1;
  uint16_t analog2;
  uint16_t temperature_raw;
  uint8_t analog_fault;
  uint8_t temp_fault;
  uint8_t digital_fault;
  uint8_t boot_guard_active;
  uint8_t protection_latched;
  uint8_t adc_ok;
} CoreStatusSnapshot;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BOOT_GUARD_MS                         15000U
#define CORE_LOGIC_PERIOD_MS                  20U
#define ADC_POLL_TIMEOUT_MS                   5U

#define ANALOG_INPUT0_MIN                     0U
#define ANALOG_INPUT0_MAX                     4095U
#define ANALOG_INPUT1_MIN                     0U
#define ANALOG_INPUT1_MAX                     4095U
#define ANALOG_INPUT2_MIN                     0U
#define ANALOG_INPUT2_MAX                     4095U
#define TEMP_SENSOR_ADC_MIN                   0U
#define TEMP_SENSOR_ADC_MAX                   4095U

#define DIGITAL_INPUT_ACTIVE_LEVEL            GPIO_PIN_SET
#define DIGITAL_INPUT_STABLE_COUNT            3U

#define ADC_READ_FAIL_TRIGGERS_PROTECTION     1U

#define LED0_TOGGLE_PERIOD_MS                500U
#define LED1_TOGGLE_PERIOD_MS                100U
#define UART_STATUS_PRINT_PERIOD_MS          500U

#if ((LED0_TOGGLE_PERIOD_MS % CORE_LOGIC_PERIOD_MS) != 0)
#error "LED0_TOGGLE_PERIOD_MS must be a multiple of CORE_LOGIC_PERIOD_MS"
#endif
#if ((LED1_TOGGLE_PERIOD_MS % CORE_LOGIC_PERIOD_MS) != 0)
#error "LED1_TOGGLE_PERIOD_MS must be a multiple of CORE_LOGIC_PERIOD_MS"
#endif
#if ((UART_STATUS_PRINT_PERIOD_MS % CORE_LOGIC_PERIOD_MS) != 0)
#error "UART_STATUS_PRINT_PERIOD_MS must be a multiple of CORE_LOGIC_PERIOD_MS"
#endif

#define LED0_TOGGLE_TICKS                     (LED0_TOGGLE_PERIOD_MS / CORE_LOGIC_PERIOD_MS)
#define LED1_TOGGLE_TICKS                     (LED1_TOGGLE_PERIOD_MS / CORE_LOGIC_PERIOD_MS)
#define UART_STATUS_PRINT_TICKS               (UART_STATUS_PRINT_PERIOD_MS / CORE_LOGIC_PERIOD_MS)

/* If LED wiring is active-low (common), OFF=SET and ON=RESET. Adjust if needed. */
#define LED_OFF_STATE                         GPIO_PIN_SET

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static uint32_t boot_tick_ms;
static uint8_t digital_active_counts[3];
static uint8_t protection_latched;
static uint16_t led0_tick_count;
static uint16_t led1_tick_count;
static uint16_t uart_print_tick_count;
static CoreStatusSnapshot core_status;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void CoreProtection_Init(void);
static void CoreProtection_Update(void);
static void Led_Init(void);
static void Led_Update(void);
static void UartStatus_Update(void);
static HAL_StatusTypeDef ReadCoreAdcInputs(uint16_t *analog0,
                                           uint16_t *analog1,
                                           uint16_t *analog2,
                                           uint16_t *temperature_raw);
static uint8_t IsOutOfRange(uint16_t value, uint16_t min_value, uint16_t max_value);
static uint8_t UpdateDigitalCounter(uint8_t *counter, GPIO_PinState level);
static uint8_t ReadDigitalFaultStable(void);
static void SetOutEnableState(GPIO_PinState state);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void CoreProtection_Init(void)
{
  uint8_t i;

  boot_tick_ms = HAL_GetTick();
  protection_latched = 0U;
  led0_tick_count = 0U;
  led1_tick_count = 0U;
  uart_print_tick_count = 0U;
  core_status = (CoreStatusSnapshot){0};

  for (i = 0U; i < 3U; i++)
  {
    digital_active_counts[i] = 0U;
  }

  (void)HAL_ADCEx_Calibration_Start(&hadc);
  SetOutEnableState(GPIO_PIN_RESET);
}

static void Led_Init(void)
{
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, LED_OFF_STATE);
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, LED_OFF_STATE);
}

static void Led_Update(void)
{
  led0_tick_count++;
  if (led0_tick_count >= LED0_TOGGLE_TICKS)
  {
    led0_tick_count = 0U;
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  }

  if (protection_latched == 0U)
  {
    led1_tick_count = 0U;
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, LED_OFF_STATE);
    return;
  }

  led1_tick_count++;
  if (led1_tick_count >= LED1_TOGGLE_TICKS)
  {
    led1_tick_count = 0U;
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  }
}

static void UartStatus_Update(void)
{
  uart_print_tick_count++;
  if (uart_print_tick_count < UART_STATUS_PRINT_TICKS)
  {
    return;
  }
  uart_print_tick_count = 0U;

  SerialLog_Printf("tick=%lu guard=%u latched=%u adc_ok=%u a0=%u a1=%u a2=%u temp=%u af=%u tf=%u df=%u\r\n",
                   (unsigned long)HAL_GetTick(),
                   core_status.boot_guard_active,
                   core_status.protection_latched,
                   core_status.adc_ok,
                   core_status.analog0,
                   core_status.analog1,
                   core_status.analog2,
                   core_status.temperature_raw,
                   core_status.analog_fault,
                   core_status.temp_fault,
                   core_status.digital_fault);
}

static HAL_StatusTypeDef ReadCoreAdcInputs(uint16_t *analog0,
                                           uint16_t *analog1,
                                           uint16_t *analog2,
                                           uint16_t *temperature_raw)
{
  uint16_t samples[4];
  uint32_t i;

  if (HAL_ADC_Start(&hadc) != HAL_OK)
  {
    return HAL_ERROR;
  }

  for (i = 0U; i < 4U; i++)
  {
    if (HAL_ADC_PollForConversion(&hadc, ADC_POLL_TIMEOUT_MS) != HAL_OK)
    {
      (void)HAL_ADC_Stop(&hadc);
      return HAL_ERROR;
    }
    samples[i] = (uint16_t)HAL_ADC_GetValue(&hadc);
  }

  (void)HAL_ADC_Stop(&hadc);

  *analog0 = samples[0];
  *analog1 = samples[1];
  *analog2 = samples[2];
  *temperature_raw = samples[3];

  return HAL_OK;
}

static uint8_t IsOutOfRange(uint16_t value, uint16_t min_value, uint16_t max_value)
{
  if ((value < min_value) || (value > max_value))
  {
    return 1U;
  }
  return 0U;
}

static uint8_t UpdateDigitalCounter(uint8_t *counter, GPIO_PinState level)
{
  if (level == DIGITAL_INPUT_ACTIVE_LEVEL)
  {
    if (*counter < DIGITAL_INPUT_STABLE_COUNT)
    {
      (*counter)++;
    }
  }
  else
  {
    *counter = 0U;
  }

  if (*counter >= DIGITAL_INPUT_STABLE_COUNT)
  {
    return 1U;
  }
  return 0U;
}

static uint8_t ReadDigitalFaultStable(void)
{
  uint8_t input0_active;
  uint8_t input1_active;
  uint8_t input2_active;

  input0_active = UpdateDigitalCounter(&digital_active_counts[0],
                                       HAL_GPIO_ReadPin(DigitalInput0_GPIO_Port, DigitalInput0_Pin));
  input1_active = UpdateDigitalCounter(&digital_active_counts[1],
                                       HAL_GPIO_ReadPin(DigitalInput1_GPIO_Port, DigitalInput1_Pin));
  input2_active = UpdateDigitalCounter(&digital_active_counts[2],
                                       HAL_GPIO_ReadPin(DigitalInput2_GPIO_Port, DigitalInput2_Pin));

  if ((input0_active != 0U) || (input1_active != 0U) || (input2_active != 0U))
  {
    return 1U;
  }
  return 0U;
}

static void SetOutEnableState(GPIO_PinState state)
{
  HAL_GPIO_WritePin(OutEnable0_GPIO_Port, OutEnable0_Pin, state);
  HAL_GPIO_WritePin(OutEnable1_GPIO_Port, OutEnable1_Pin, state);
}

static void CoreProtection_Update(void)
{
  uint32_t now_ms;
  uint8_t boot_guard_active;
  uint8_t analog_fault = 0U;
  uint8_t digital_fault;
  uint8_t temp_fault = 0U;
  uint8_t protection_triggered;
  uint16_t analog0 = 0U;
  uint16_t analog1 = 0U;
  uint16_t analog2 = 0U;
  uint16_t temperature_raw = 0U;

  now_ms = HAL_GetTick();

  boot_guard_active = ((now_ms - boot_tick_ms) < BOOT_GUARD_MS) ? 1U : 0U;
  digital_fault = ReadDigitalFaultStable();
  core_status.boot_guard_active = boot_guard_active;
  core_status.digital_fault = digital_fault;

  if (ReadCoreAdcInputs(&analog0, &analog1, &analog2, &temperature_raw) == HAL_OK)
  {
    analog_fault |= IsOutOfRange(analog0, ANALOG_INPUT0_MIN, ANALOG_INPUT0_MAX);
    analog_fault |= IsOutOfRange(analog1, ANALOG_INPUT1_MIN, ANALOG_INPUT1_MAX);
    analog_fault |= IsOutOfRange(analog2, ANALOG_INPUT2_MIN, ANALOG_INPUT2_MAX);
    temp_fault = IsOutOfRange(temperature_raw, TEMP_SENSOR_ADC_MIN, TEMP_SENSOR_ADC_MAX);
    core_status.adc_ok = 1U;
  }
  else
  {
    analog_fault = ADC_READ_FAIL_TRIGGERS_PROTECTION;
    core_status.adc_ok = 0U;
  }
  core_status.analog0 = analog0;
  core_status.analog1 = analog1;
  core_status.analog2 = analog2;
  core_status.temperature_raw = temperature_raw;
  core_status.analog_fault = analog_fault;
  core_status.temp_fault = temp_fault;

  protection_triggered = ((analog_fault != 0U) || (temp_fault != 0U) || (digital_fault != 0U)) ? 1U : 0U;

  if (boot_guard_active != 0U)
  {
    protection_latched = 0U;
    SetOutEnableState(GPIO_PIN_RESET);
    core_status.protection_latched = protection_latched;
    return;
  }

  if (protection_triggered != 0U)
  {
    protection_latched = 1U;
  }
  SetOutEnableState((protection_latched != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  core_status.protection_latched = protection_latched;
}

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
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  SerialLog_Init(&huart1);
  SerialLog_Print("System boot\r\n");
  CoreProtection_Init();
  Led_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t loop_start_ms = HAL_GetTick();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    CoreProtection_Update();
    Led_Update();
    UartStatus_Update();
    while ((HAL_GetTick() - loop_start_ms) < CORE_LOGIC_PERIOD_MS)
    {
      __NOP();
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x00201D2B;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OutEnable0_Pin|OutEnable1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED0_Pin|LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : OutEnable0_Pin OutEnable1_Pin */
  GPIO_InitStruct.Pin = OutEnable0_Pin|OutEnable1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Key0_Pin Key1_Pin Key2_Pin */
  GPIO_InitStruct.Pin = Key0_Pin|Key1_Pin|Key2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Key3_Pin */
  GPIO_InitStruct.Pin = Key3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Key3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DigitalInput0_Pin DigitalInput1_Pin DigitalInput2_Pin */
  GPIO_InitStruct.Pin = DigitalInput0_Pin|DigitalInput1_Pin|DigitalInput2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED0_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  HAL_I2CEx_EnableFastModePlus(SYSCFG_CFGR1_I2C_FMP_PB8);

  /**/
  HAL_I2CEx_EnableFastModePlus(SYSCFG_CFGR1_I2C_FMP_PB9);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
