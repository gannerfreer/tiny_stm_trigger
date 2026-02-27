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
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include "serial_log.h"
#include "Oled/oled.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint16_t ad_t1_raw;
  uint16_t ad_t2_raw;
  uint16_t ad_t1_mv;
  uint16_t ad_t2_mv;
  uint16_t input_mv;
  uint16_t temperature_raw;
  int16_t temperature_c10;
  uint16_t pwm0_duty;
  uint16_t pwm1_duty;
  uint16_t pwm2_duty;
  uint8_t digital_state;
  uint8_t digital_fault;
  uint8_t temp_fault;
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

#define ADC_FULL_SCALE                        4095U
#define ADC_REF_MV                            3300U
#define TS_CAL1_ADDR                          ((uint16_t *)0x1FFFF7B8U)
#define TS_CAL1_TEMP_C                        30
#define TS_CAL1_VDDA_MV                        3300U
#define TS_AVG_SLOPE_UV_PER_C                 4300U

/* TODO: replace divider/calibration factors with measured values. */
/* AD_T1: 10k (top) / 2.2k (bottom) divider => V_in = V_adc * (12.2 / 2.2) = 61/11 */
#define AD_T1_DIVIDER_NUM                     61U
#define AD_T1_DIVIDER_DEN                     11U
#define AD_T1_CAL_NUM                         1U
#define AD_T1_CAL_DEN                         1U
#define AD_T2_DIVIDER_NUM                     1U
#define AD_T2_DIVIDER_DEN                     1U
#define AD_T2_CAL_NUM                         1U
#define AD_T2_CAL_DEN                         1U

#define AD_T1_SCALE_NUM                       (AD_T1_DIVIDER_NUM * AD_T1_CAL_NUM)
#define AD_T1_SCALE_DEN                       (AD_T1_DIVIDER_DEN * AD_T1_CAL_DEN)
#define AD_T2_SCALE_NUM                       (AD_T2_DIVIDER_NUM * AD_T2_CAL_NUM)
#define AD_T2_SCALE_DEN                       (AD_T2_DIVIDER_DEN * AD_T2_CAL_DEN)

#define PWM_BASE_FREQ_HZ                      10000U
#define PWM_TIMER_CLOCK_HZ                    48000000U
#define PWM_TIMER_PERIOD                      ((PWM_TIMER_CLOCK_HZ / PWM_BASE_FREQ_HZ) - 1U)
#define PWM_DUTY_MAX                           PWM_TIMER_PERIOD
#define PWM0_DUTY_LIMIT_PERCENT               20U
#define PWM0_DUTY_LIMIT                       ((PWM_DUTY_MAX * PWM0_DUTY_LIMIT_PERCENT) / 100U)
#define PWM0_TARGET_MV                         10500U
#define PWM1_FIXED_TARGET_MV                  1500U

#if (PWM0_DUTY_LIMIT_PERCENT > 100U)
#error "PWM0_DUTY_LIMIT_PERCENT must be <= 100"
#endif

/* PID gains: duty = Kp*err + Ki*sum(err) + Kd*dErr. Tune as needed. */
#define PWM0_PID_KP_NUM                        1U
#define PWM0_PID_KP_DEN                        4U
#define PWM0_PID_KI_NUM                        1U
#define PWM0_PID_KI_DEN                        200U
#define PWM0_PID_KD_NUM                        0U
#define PWM0_PID_KD_DEN                        1U

#if (PWM0_PID_KI_NUM == 0U)
#error "PWM0_PID_KI_NUM must be non-zero"
#endif

#define DIGITAL_STATE_OPEN                     (1U << 0)
#define DIGITAL_STATE_CLOSE                    (1U << 1)
#define DIGITAL_STATE_GAS_D1                   (1U << 2)
#define DIGITAL_STATE_GAS_D2                   (1U << 3)

#define OPEN_ACTIVE_LEVEL                     GPIO_PIN_RESET
#define CLOSE_ACTIVE_LEVEL                    GPIO_PIN_SET
#define GAS_ACTIVE_LEVEL                      GPIO_PIN_SET
#define DIGITAL_INPUT_STABLE_COUNT            3U

#define ADC_READ_FAIL_TRIGGERS_PROTECTION     1U

#define LED0_TOGGLE_PERIOD_MS                500U
#define LED1_TOGGLE_PERIOD_MS                100U
#define UART_STATUS_PRINT_PERIOD_MS          500U
#define OLED_UPDATE_PERIOD_MS                200U

#if ((LED0_TOGGLE_PERIOD_MS % CORE_LOGIC_PERIOD_MS) != 0)
#error "LED0_TOGGLE_PERIOD_MS must be a multiple of CORE_LOGIC_PERIOD_MS"
#endif
#if ((LED1_TOGGLE_PERIOD_MS % CORE_LOGIC_PERIOD_MS) != 0)
#error "LED1_TOGGLE_PERIOD_MS must be a multiple of CORE_LOGIC_PERIOD_MS"
#endif
#if ((UART_STATUS_PRINT_PERIOD_MS % CORE_LOGIC_PERIOD_MS) != 0)
#error "UART_STATUS_PRINT_PERIOD_MS must be a multiple of CORE_LOGIC_PERIOD_MS"
#endif
#if ((OLED_UPDATE_PERIOD_MS % CORE_LOGIC_PERIOD_MS) != 0)
#error "OLED_UPDATE_PERIOD_MS must be a multiple of CORE_LOGIC_PERIOD_MS"
#endif

#define LED0_TOGGLE_TICKS                     (LED0_TOGGLE_PERIOD_MS / CORE_LOGIC_PERIOD_MS)
#define LED1_TOGGLE_TICKS                     (LED1_TOGGLE_PERIOD_MS / CORE_LOGIC_PERIOD_MS)
#define UART_STATUS_PRINT_TICKS               (UART_STATUS_PRINT_PERIOD_MS / CORE_LOGIC_PERIOD_MS)
#define OLED_UPDATE_TICKS                     (OLED_UPDATE_PERIOD_MS / CORE_LOGIC_PERIOD_MS)
#define OLED_TRIGGER_PAGE_TICKS               5U

/* If LED wiring is active-low (common), OFF=SET and ON=RESET. Adjust if needed. */
#define LED_OFF_STATE                         GPIO_PIN_SET

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static uint32_t boot_tick_ms;
static uint8_t digital_active_counts[4];
static uint8_t protection_latched;
static uint16_t pwm0_duty;
static uint16_t pwm1_duty;
static uint16_t pwm2_duty;
static uint16_t led0_tick_count;
static uint16_t led1_tick_count;
static uint16_t uart_print_tick_count;
static uint16_t oled_update_tick_count;
static int32_t pwm0_pid_integral;
static int32_t pwm0_pid_prev_error;
static CoreStatusSnapshot core_status;
static uint32_t adc_fail_count;
static uint32_t adc_last_error;
static uint32_t adc_last_isr;
static uint32_t adc_last_cr;
static uint32_t adc_last_chselr;
static uint8_t adc_last_status;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
static void CoreProtection_Init(void);
static void CoreProtection_Update(void);
static void Led_Init(void);
static void Led_Update(void);
static void UartStatus_Update(void);
static void OledStatus_Update(void);
static void OledDebug_Update(void);
static void OledWriteLine(uint8_t line, const char *text);
static HAL_StatusTypeDef ReadCoreAdcInputs(uint16_t *ad_t1_raw,
                                           uint16_t *ad_t2_raw,
                                           uint16_t *temperature_raw);
static uint8_t UpdateDigitalCounter(uint8_t *counter,
                                    GPIO_PinState level,
                                    GPIO_PinState active_level);
static uint8_t ReadDigitalFaultStable(void);
static uint16_t AdcRawToMv(uint16_t raw);
static uint16_t ApplyVoltageScale(uint16_t mv, uint32_t scale_num, uint32_t scale_den);
static uint16_t ComputeInputVoltageMv(uint16_t ad_t1_mv, uint16_t ad_t2_mv);
static int16_t TemperatureRawToC10(uint16_t raw);
static void ResetPwm0Pid(void);
static uint16_t AdjustPwm0Duty(uint16_t feedback_mv);
static void ApplyPwmOutputs(uint16_t duty0, uint16_t duty1, uint16_t duty2);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void CoreProtection_Init(void)
{
  uint8_t i;

  boot_tick_ms = HAL_GetTick();
  protection_latched = 0U;
  pwm0_duty = 0U;
  pwm1_duty = 0U;
  pwm2_duty = 0U;
  led0_tick_count = 0U;
  led1_tick_count = 0U;
  uart_print_tick_count = 0U;
  oled_update_tick_count = 0U;
  pwm0_pid_integral = 0;
  pwm0_pid_prev_error = 0;
  core_status = (CoreStatusSnapshot){0};
  adc_fail_count = 0U;
  adc_last_error = 0U;
  adc_last_isr = 0U;
  adc_last_cr = 0U;
  adc_last_chselr = 0U;
  adc_last_status = (uint8_t)HAL_OK;

  for (i = 0U; i < 4U; i++)
  {
    digital_active_counts[i] = 0U;
  }

  (void)HAL_ADCEx_Calibration_Start(&hadc);
  /* Temporarily disable PWM2/PWM3 while debugging PWM1. */
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  ApplyPwmOutputs(0U, 0U, 0U);
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

  SerialLog_Printf("tick=%lu guard=%u latched=%u adc_ok=%u t1=%u t2=%u t1mv=%u t2mv=%u inmv=%u temp=%u din=0x%02X df=%u tf=%u pwm0=%u pwm1=%u pwm2=%u\r\n",
                   (unsigned long)HAL_GetTick(),
                   core_status.boot_guard_active,
                   core_status.protection_latched,
                   core_status.adc_ok,
                   core_status.ad_t1_raw,
                   core_status.ad_t2_raw,
                   core_status.ad_t1_mv,
                   core_status.ad_t2_mv,
                   core_status.input_mv,
                   core_status.temperature_raw,
                   core_status.digital_state,
                   core_status.digital_fault,
                   core_status.temp_fault,
                   core_status.pwm0_duty,
                   core_status.pwm1_duty,
                   core_status.pwm2_duty);
}

static uint8_t OledTryAppendToken(char *line, size_t line_size, const char *token)
{
  size_t len = strlen(line);
  size_t token_len = strlen(token);
  size_t needed = token_len + ((len > 0U) ? 1U : 0U);

  if ((len + needed) > (line_size - 1U))
  {
    return 0U;
  }

  if (len > 0U)
  {
    line[len] = '|';
    len++;
  }
  (void)memcpy(&line[len], token, token_len);
  line[len + token_len] = '\0';
  return 1U;
}

static void OledDebug_Update(void)
{
  static uint8_t trigger_page;
  static uint8_t trigger_page_tick;
  char line0[17];
  char line1[17];
  char tokens[65];
  size_t tokens_len;
  size_t page_count;
  size_t page_start;
  size_t i;
  uint8_t any_trigger = 0U;

  if (core_status.boot_guard_active != 0U)
  {
    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms = now_ms - boot_tick_ms;
    uint32_t remaining_ms = 0U;
    uint32_t remaining_s;

    if (elapsed_ms < BOOT_GUARD_MS)
    {
      remaining_ms = BOOT_GUARD_MS - elapsed_ms;
    }
    remaining_s = (remaining_ms + 999U) / 1000U;

    (void)snprintf(line0, sizeof(line0), "System warming");
    (void)snprintf(line1, sizeof(line1), "up T-%lus", (unsigned long)remaining_s);
    OLED_Debug_PrintLine(0U, line0);
    OLED_Debug_PrintLine(1U, line1);
    trigger_page = 0U;
    trigger_page_tick = 0U;
    return;
  }

  line0[0] = '\0';
  line1[0] = '\0';
  tokens[0] = '\0';

  if ((core_status.digital_state & DIGITAL_STATE_OPEN) != 0U)
  {
    (void)OledTryAppendToken(tokens, sizeof(tokens), "OPEN");
    any_trigger = 1U;
  }
  if ((core_status.digital_state & DIGITAL_STATE_CLOSE) != 0U)
  {
    (void)OledTryAppendToken(tokens, sizeof(tokens), "CLOSE");
    any_trigger = 1U;
  }
  if ((core_status.digital_state & DIGITAL_STATE_GAS_D1) != 0U)
  {
    (void)OledTryAppendToken(tokens, sizeof(tokens), "GAS1");
    any_trigger = 1U;
  }
  if ((core_status.digital_state & DIGITAL_STATE_GAS_D2) != 0U)
  {
    (void)OledTryAppendToken(tokens, sizeof(tokens), "GAS2");
    any_trigger = 1U;
  }
  if (core_status.adc_ok == 0U)
  {
    (void)OledTryAppendToken(tokens, sizeof(tokens), "ADC");
    any_trigger = 1U;
  }
  if (core_status.temp_fault != 0U)
  {
    (void)OledTryAppendToken(tokens, sizeof(tokens), "TMP");
    any_trigger = 1U;
  }

  if (any_trigger == 0U)
  {
    (void)snprintf(line0, sizeof(line0), "System Idle");
    OLED_Debug_PrintLine(0U, line0);
    OLED_Debug_PrintLine(1U, "");
    trigger_page = 0U;
    trigger_page_tick = 0U;
    return;
  }

  (void)snprintf(line0, sizeof(line0), "[System Action]");
  tokens_len = strlen(tokens);
  if (tokens_len == 0U)
  {
    line1[0] = '\0';
  }
  else
  {
    page_count = (tokens_len + 15U) / 16U;
    if (page_count == 0U)
    {
      page_count = 1U;
    }
    if (trigger_page >= page_count)
    {
      trigger_page = 0U;
    }
    trigger_page_tick++;
    if (trigger_page_tick >= OLED_TRIGGER_PAGE_TICKS)
    {
      trigger_page_tick = 0U;
      trigger_page++;
      if (trigger_page >= page_count)
      {
        trigger_page = 0U;
      }
    }

    page_start = (size_t)trigger_page * 16U;
    for (i = 0U; i < 16U; i++)
    {
      if ((page_start + i) >= tokens_len)
      {
        break;
      }
      line1[i] = tokens[page_start + i];
    }
    line1[i] = '\0';
  }

  OLED_Debug_PrintLine(0U, line0);
  OLED_Debug_PrintLine(1U, line1);
}

static void OledWriteLine(uint8_t line, const char *text)
{
  static const char blank_line[17] = "                ";

  OLED_ShowString(0U, line, blank_line, 8U);
  OLED_ShowString(0U, line, text, 8U);
}

static void OledStatus_Update(void)
{
  char line[17];

  oled_update_tick_count++;
  if (oled_update_tick_count < OLED_UPDATE_TICKS)
  {
    return;
  }
  oled_update_tick_count = 0U;

  OledDebug_Update();

  if (core_status.adc_ok == 0U)
  {
    const char *status_tag = "??";
    uint32_t fail_count_mod = adc_fail_count % 10000U;

    switch ((HAL_StatusTypeDef)adc_last_status)
    {
      case HAL_OK:
        status_tag = "OK";
        break;
      case HAL_ERROR:
        status_tag = "ER";
        break;
      case HAL_BUSY:
        status_tag = "BU";
        break;
      case HAL_TIMEOUT:
        status_tag = "TO";
        break;
      default:
        status_tag = "??";
        break;
    }

    (void)snprintf(line, sizeof(line), "ADC:%s ER:%04lX", status_tag,
                   (unsigned long)adc_last_error);
    OledWriteLine(2U, line);

    (void)snprintf(line, sizeof(line), "ISR:%04lX C:%04lu",
                   (unsigned long)(adc_last_isr & 0xFFFFU),
                   (unsigned long)fail_count_mod);
    OledWriteLine(3U, line);

    (void)snprintf(line, sizeof(line), "CR:%04lX CH:%05lX",
                   (unsigned long)(adc_last_cr & 0xFFFFU),
                   (unsigned long)(adc_last_chselr & 0x1FFFFU));
    OledWriteLine(4U, line);
    return;
  }

  (void)snprintf(line, sizeof(line), "T1:%5u mV", (unsigned int)core_status.ad_t1_mv);
  OledWriteLine(2U, line);

  (void)snprintf(line, sizeof(line), "T2:%5u mV", (unsigned int)core_status.ad_t2_mv);
  OledWriteLine(3U, line);

  {
    int16_t temp_c10 = core_status.temperature_c10;
    int16_t temp_abs = (temp_c10 < 0) ? (int16_t)(-temp_c10) : temp_c10;
    int16_t temp_c = (int16_t)(temp_abs / 10);
    int16_t temp_frac = (int16_t)(temp_abs % 10);
    char sign = (temp_c10 < 0) ? '-' : '+';

    (void)snprintf(line, sizeof(line), "Temp:%c%2d.%1dC", sign, temp_c, temp_frac);
  }
  OledWriteLine(4U, line);
}

static HAL_StatusTypeDef ReadCoreAdcInputs(uint16_t *ad_t1_raw,
                                           uint16_t *ad_t2_raw,
                                           uint16_t *temperature_raw)
{
  uint16_t samples[3];
  uint32_t i;
  HAL_StatusTypeDef status;

  __HAL_ADC_CLEAR_FLAG(&hadc, ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR | ADC_FLAG_EOSMP | ADC_FLAG_RDY);

  status = HAL_ADC_Start(&hadc);
  if (status != HAL_OK)
  {
    adc_last_status = (uint8_t)status;
    adc_last_error = HAL_ADC_GetError(&hadc);
    adc_last_isr = hadc.Instance->ISR;
    adc_last_cr = hadc.Instance->CR;
    adc_last_chselr = hadc.Instance->CHSELR;
    adc_fail_count++;
    return HAL_ERROR;
  }

  for (i = 0U; i < 3U; i++)
  {
    status = HAL_ADC_PollForConversion(&hadc, ADC_POLL_TIMEOUT_MS);
    if (status != HAL_OK)
    {
      adc_last_status = (uint8_t)status;
      adc_last_error = HAL_ADC_GetError(&hadc);
      adc_last_isr = hadc.Instance->ISR;
      adc_last_cr = hadc.Instance->CR;
      adc_last_chselr = hadc.Instance->CHSELR;
      adc_fail_count++;
      (void)HAL_ADC_Stop(&hadc);
      return status;
    }
    samples[i] = (uint16_t)HAL_ADC_GetValue(&hadc);
  }

  (void)HAL_ADC_Stop(&hadc);

  adc_last_status = (uint8_t)HAL_OK;
  adc_last_error = 0U;
  *ad_t1_raw = samples[0];
  *ad_t2_raw = samples[1];
  *temperature_raw = samples[2];

  return HAL_OK;
}

static uint8_t UpdateDigitalCounter(uint8_t *counter,
                                    GPIO_PinState level,
                                    GPIO_PinState active_level)
{
  if (level == active_level)
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
  uint8_t open_active;
  uint8_t close_active;
  uint8_t gas_d1_active;
  uint8_t gas_d2_active;
  GPIO_PinState open_level;
  GPIO_PinState close_level;
  GPIO_PinState gas_d1_level;
  GPIO_PinState gas_d2_level;
  uint8_t digital_state = 0U;

  open_level = HAL_GPIO_ReadPin(OPEN_GPIO_Port, OPEN_Pin);
  close_level = HAL_GPIO_ReadPin(CLOSE_GPIO_Port, CLOSE_Pin);
  gas_d1_level = HAL_GPIO_ReadPin(GAS_D1_GPIO_Port, GAS_D1_Pin);
  gas_d2_level = HAL_GPIO_ReadPin(GAS_D2_GPIO_Port, GAS_D2_Pin);

  if (open_level == OPEN_ACTIVE_LEVEL)
  {
    digital_state |= DIGITAL_STATE_OPEN;
  }
  if (close_level == CLOSE_ACTIVE_LEVEL)
  {
    digital_state |= DIGITAL_STATE_CLOSE;
  }
  if (gas_d1_level == GAS_ACTIVE_LEVEL)
  {
    digital_state |= DIGITAL_STATE_GAS_D1;
  }
  if (gas_d2_level == GAS_ACTIVE_LEVEL)
  {
    digital_state |= DIGITAL_STATE_GAS_D2;
  }

  open_active = UpdateDigitalCounter(&digital_active_counts[0], open_level, OPEN_ACTIVE_LEVEL);
  close_active = UpdateDigitalCounter(&digital_active_counts[1], close_level, CLOSE_ACTIVE_LEVEL);
  gas_d1_active = UpdateDigitalCounter(&digital_active_counts[2], gas_d1_level, GAS_ACTIVE_LEVEL);
  gas_d2_active = UpdateDigitalCounter(&digital_active_counts[3], gas_d2_level, GAS_ACTIVE_LEVEL);

  core_status.digital_state = digital_state;

  if ((open_active != 0U) || (close_active != 0U) || (gas_d1_active != 0U) || (gas_d2_active != 0U))
  {
    return 1U;
  }
  return 0U;
}

static uint16_t AdcRawToMv(uint16_t raw)
{
  uint32_t mv;

  mv = (uint32_t)raw * ADC_REF_MV;
  mv = (mv + (ADC_FULL_SCALE / 2U)) / ADC_FULL_SCALE;
  if (mv > 0xFFFFU)
  {
    mv = 0xFFFFU;
  }
  return (uint16_t)mv;
}

static uint16_t ApplyVoltageScale(uint16_t mv, uint32_t scale_num, uint32_t scale_den)
{
  uint32_t scaled;

  if (scale_den == 0U)
  {
    return mv;
  }

  scaled = (uint32_t)mv * scale_num;
  scaled = (scaled + (scale_den / 2U)) / scale_den;
  if (scaled > 0xFFFFU)
  {
    scaled = 0xFFFFU;
  }
  return (uint16_t)scaled;
}

static uint16_t ComputeInputVoltageMv(uint16_t ad_t1_mv, uint16_t ad_t2_mv)
{
  uint32_t sum;

  sum = (uint32_t)ad_t1_mv + (uint32_t)ad_t2_mv;
  return (uint16_t)((sum + 1U) / 2U);
}

static int16_t TemperatureRawToC10(uint16_t raw)
{
  uint32_t v_sense_uv;
  uint32_t v30_uv;
  int32_t delta_uv;
  int32_t temp_c10;
  int32_t num;

  v_sense_uv = (uint32_t)raw * ADC_REF_MV * 1000U;
  v_sense_uv = (v_sense_uv + (ADC_FULL_SCALE / 2U)) / ADC_FULL_SCALE;

  v30_uv = (uint32_t)(*TS_CAL1_ADDR) * TS_CAL1_VDDA_MV * 1000U;
  v30_uv = (v30_uv + (ADC_FULL_SCALE / 2U)) / ADC_FULL_SCALE;

  delta_uv = (int32_t)v_sense_uv - (int32_t)v30_uv;
  temp_c10 = (int32_t)TS_CAL1_TEMP_C * 10;
  num = delta_uv * 10;
  if (num >= 0)
  {
    num += (int32_t)TS_AVG_SLOPE_UV_PER_C / 2;
  }
  else
  {
    num -= (int32_t)TS_AVG_SLOPE_UV_PER_C / 2;
  }
  temp_c10 += num / (int32_t)TS_AVG_SLOPE_UV_PER_C;

  if (temp_c10 > INT16_MAX)
  {
    temp_c10 = INT16_MAX;
  }
  else if (temp_c10 < INT16_MIN)
  {
    temp_c10 = INT16_MIN;
  }

  return (int16_t)temp_c10;
}

static void ResetPwm0Pid(void)
{
  pwm0_pid_integral = 0;
  pwm0_pid_prev_error = 0;
}

static uint16_t AdjustPwm0Duty(uint16_t feedback_mv)
{
  uint32_t duty;

  /* PID algorithm disabled for now. Keep code commented for easy restore. */
#if 0
  int32_t error = (int32_t)PWM0_TARGET_MV - (int32_t)feedback_mv;
  int32_t p_term;
  int32_t i_term;
  int32_t d_term;
  int32_t output;
  int32_t i_limit;
  int32_t delta_error;

  p_term = (error * (int32_t)PWM0_PID_KP_NUM) / (int32_t)PWM0_PID_KP_DEN;

  pwm0_pid_integral += error;
  i_limit = ((int32_t)PWM0_DUTY_LIMIT * (int32_t)PWM0_PID_KI_DEN) / (int32_t)PWM0_PID_KI_NUM;
  if (pwm0_pid_integral > i_limit)
  {
    pwm0_pid_integral = i_limit;
  }
  else if (pwm0_pid_integral < -i_limit)
  {
    pwm0_pid_integral = -i_limit;
  }
  i_term = (pwm0_pid_integral * (int32_t)PWM0_PID_KI_NUM) / (int32_t)PWM0_PID_KI_DEN;

  delta_error = error - pwm0_pid_prev_error;
  d_term = (delta_error * (int32_t)PWM0_PID_KD_NUM) / (int32_t)PWM0_PID_KD_DEN;
  pwm0_pid_prev_error = error;

  output = p_term + i_term + d_term;
  if (output < 0)
  {
    output = 0;
  }
  else if (output > (int32_t)PWM0_DUTY_LIMIT)
  {
    output = (int32_t)PWM0_DUTY_LIMIT;
  }

  return (uint16_t)output;
#endif
  if (feedback_mv == 0U)
  {
    return 0U;
  }

  duty = ((uint32_t)PWM_DUTY_MAX * (uint32_t)PWM1_FIXED_TARGET_MV) + ((uint32_t)feedback_mv / 2U);
  duty /= (uint32_t)feedback_mv;
  if (duty > (uint32_t)PWM_DUTY_MAX)
  {
    duty = (uint32_t)PWM_DUTY_MAX;
  }

  return (uint16_t)duty;
}

static void ApplyPwmOutputs(uint16_t duty0, uint16_t duty1, uint16_t duty2)
{
  if (duty0 > PWM_DUTY_MAX)
  {
    duty0 = PWM_DUTY_MAX;
  }
  if (duty1 > PWM_DUTY_MAX)
  {
    duty1 = PWM_DUTY_MAX;
  }
  if (duty2 > PWM_DUTY_MAX)
  {
    duty2 = PWM_DUTY_MAX;
  }

  /* PWM0 -> TIM1_CH1 (PWM1_Pin), PWM2/PWM3 as GPIO outputs. */
  /* Temporarily disable PWM3 output while debugging PWM1/PWM2. */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty0);
  HAL_GPIO_WritePin(PWM2_GPIO_Port, PWM2_Pin, GPIO_PIN_RESET);
  /* HAL_GPIO_WritePin(PWM3_GPIO_Port, PWM3_Pin, (duty2 > 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET); */
  HAL_GPIO_WritePin(PWM3_GPIO_Port, PWM3_Pin, GPIO_PIN_RESET);

  pwm0_duty = duty0;
  pwm1_duty = duty1;
  pwm2_duty = duty2;
  core_status.pwm0_duty = duty0;
  core_status.pwm1_duty = duty1;
  core_status.pwm2_duty = duty2;
}

static void CoreProtection_Update(void)
{
  uint32_t now_ms;
  uint8_t boot_guard_active;
  uint8_t digital_fault;
  uint8_t adc_fault = 0U;
  uint8_t temp_fault = 0U;
  uint8_t protection_triggered;
  uint16_t ad_t1_raw = 0U;
  uint16_t ad_t2_raw = 0U;
  uint16_t temperature_raw = 0U;
  uint16_t ad_t1_pin_mv = 0U;
  uint16_t ad_t2_pin_mv = 0U;
  uint16_t ad_t1_mv = 0U;
  uint16_t ad_t2_mv = 0U;
  uint16_t input_mv = 0U;
  uint16_t target_pwm0;

  now_ms = HAL_GetTick();

  boot_guard_active = ((now_ms - boot_tick_ms) < BOOT_GUARD_MS) ? 1U : 0U;
  digital_fault = ReadDigitalFaultStable();
  core_status.boot_guard_active = boot_guard_active;
  core_status.digital_fault = digital_fault;

  if (ReadCoreAdcInputs(&ad_t1_raw, &ad_t2_raw, &temperature_raw) == HAL_OK)
  {
    ad_t1_pin_mv = AdcRawToMv(ad_t1_raw);
    ad_t2_pin_mv = AdcRawToMv(ad_t2_raw);
    ad_t1_mv = ApplyVoltageScale(ad_t1_pin_mv, AD_T1_SCALE_NUM, AD_T1_SCALE_DEN);
    ad_t2_mv = ApplyVoltageScale(ad_t2_pin_mv, AD_T2_SCALE_NUM, AD_T2_SCALE_DEN);
    input_mv = ComputeInputVoltageMv(ad_t1_mv, ad_t2_mv);
    core_status.temperature_c10 = TemperatureRawToC10(temperature_raw);
    temp_fault = (core_status.temperature_c10 > 700) ? 1U : 0U;
    core_status.adc_ok = 1U;
  }
  else
  {
    core_status.adc_ok = 0U;
    adc_fault = ADC_READ_FAIL_TRIGGERS_PROTECTION;
    core_status.temperature_c10 = 0;
  }
  core_status.ad_t1_raw = ad_t1_raw;
  core_status.ad_t2_raw = ad_t2_raw;
  core_status.ad_t1_mv = ad_t1_mv;
  core_status.ad_t2_mv = ad_t2_mv;
  core_status.input_mv = input_mv;
  core_status.temperature_raw = temperature_raw;
  core_status.temp_fault = temp_fault;

  protection_triggered = ((adc_fault != 0U) || (temp_fault != 0U) || (digital_fault != 0U)) ? 1U : 0U;

  if (boot_guard_active != 0U)
  {
    protection_latched = 0U;
    ResetPwm0Pid();
    ApplyPwmOutputs(0U, 0U, 0U);
    core_status.protection_latched = protection_latched;
    return;
  }

  if (protection_triggered != 0U)
  {
    protection_latched = 1U;
  }

  if (protection_latched == 0U)
  {
    ResetPwm0Pid();
    ApplyPwmOutputs(0U, 0U, 0U);
    core_status.protection_latched = protection_latched;
    return;
  }

  if (core_status.adc_ok != 0U)
  {
    if (input_mv == 0U)
    {
      target_pwm0 = 0U;
      ResetPwm0Pid();
    }
    else
    {
      target_pwm0 = AdjustPwm0Duty(ad_t1_mv);
    }
  }
  else
  {
    target_pwm0 = 0U;
    ResetPwm0Pid();
  }

  ApplyPwmOutputs(target_pwm0, PWM_DUTY_MAX, PWM_DUTY_MAX);
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  SerialLog_Init(&huart1);
  SerialLog_Print("System boot\r\n");
  CoreProtection_Init();
  Led_Init();
  OLED_Init(&hi2c1);
  OLED_Clear();

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
    OledStatus_Update();
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = PWM_TIMER_PERIOD;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  HAL_GPIO_WritePin(GPIOB, LED0_Pin|LED1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, PWM2_Pin|PWM3_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : GAS_D1_Pin GAS_D2_Pin */
  GPIO_InitStruct.Pin = GAS_D1_Pin|GAS_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CLOSE_Pin OPEN_Pin */
  GPIO_InitStruct.Pin = CLOSE_Pin|OPEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED0_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PWM2_Pin PWM3_Pin */
  GPIO_InitStruct.Pin = PWM2_Pin|PWM3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  HAL_I2CEx_EnableFastModePlus(SYSCFG_CFGR1_I2C_FMP_PB6);

  /**/
  HAL_I2CEx_EnableFastModePlus(SYSCFG_CFGR1_I2C_FMP_PB7);

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
