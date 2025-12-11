/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>      // for snprintf
#include "ds18b20.h"
#include "Max31855.h"
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
ds18b20_t ds18;

// 4th-order IIR (4 cascaded first-order sections)
typedef struct
{
    float stage[4];
} IIR4_LP_t;

// Voltage trend state
typedef enum
{
    TREND_STATE_UNKNOWN = 0,
    TREND_STATE_FLAT,
    TREND_STATE_UP,
    TREND_STATE_DOWN,
    TREND_STATE_INVALID
} TrendState_t;

typedef struct
{
    uint8_t     active;
    float       min_v;
    float       max_v;
    float       sum_v;
    uint32_t    count;
    float       prev_v;
    TrendState_t state;
} TrendTracker_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ADC + IIR related parameters
#define ADC_BUF_LEN   10U         // 10 samples per block (0.01 s @ 1 kSa/s)
#define ADC_REF_VOLT  3.3f        // ADC reference voltage (V)
#define ADC_MAX_VAL   4095.0f     // 12-bit ADC max value
// First-order IIR coefficient: alpha = 1 - exp(-2*pi*Fc/Fs), Fc=2Hz, Fs=1000Hz
#define IIR_ALPHA     0.0124877f  // pre-calculated
#define MODE_WINDOW    20U        // 20 blocks used for mode filtering

// Trend decision epsilon (e.g. 2 mV; adjust according to actual noise level)
#define TREND_EPS     0.001f

// ADC reading & voltage offsets (V)
#define ADC1_READING_OFFSET   0.002f
#define ADC2_READING_OFFSET   0.009f
#define ADC1_VOLT_OFFSET_POS   0.000f
#define ADC2_VOLT_OFFSET_POS   0.000f
#define ADC1_VOLT_OFFSET_NEG   0.000f
#define ADC2_VOLT_OFFSET_NEG   0.093f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char uart_buf[128];

float t1_offset = 0.5f;

// ADC DMA buffers (per channel)
uint16_t adc1_buf[ADC_BUF_LEN];
uint16_t adc2_buf[ADC_BUF_LEN];

// 4-order IIR state for each channel
IIR4_LP_t adc1_filter = {0};
IIR4_LP_t adc2_filter = {0};

// Latest averaged voltage value per 10-sample block (unit: V)
volatile float adc1_avg_v = 0.0f;
volatile float adc2_avg_v = 0.0f;

// Flags: whether each channel's block is ready
volatile uint8_t adc1_block_ready = 0;
volatile uint8_t adc2_block_ready = 0;
// When both channels' blocks are ready, this flag is set
volatile uint8_t adc_pair_ready = 0;

// ===== History buffer for "mode" (in mV, integer) =====
int16_t v1_hist[MODE_WINDOW];
int16_t v2_hist[MODE_WINDOW];
uint8_t mode_count = 0;

// Trend trackers for ADC1 & ADC2 (used during DS18B20 conversion window)
TrendTracker_t adc1_trend = {0};
TrendTracker_t adc2_trend = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ds18_tim_cb(TIM_HandleTypeDef *htim)
{
    ow_callback(&ds18.ow);
}

void ds18_done_cb(ow_err_t error)
{
}

// ==== MAX31855 / thermocouple UART message buffers ====
char MSG[100]  = "MAX31855 test\n\r";
char MSG0[100] = "Initializing sensor...\n\r";
char MSG1[100] = "ERROR\n\r";
char MSG2[100] = "DONE.\n\r";
char MSG3[100] = "Thermocouple fault(s) detected!\n\r";
char MSG4[100] = "FAULT: Thermocouple is open - no connections.\n\r";
char MSG5[100] = "FAULT: Thermocouple is short-circuited to GND.\n\r";
char MSG6[100] = "FAULT: Thermocouple is short-circuited to VCC.\n\r";
char MSG7[100];
char MSG8[100];
char MSG9[100] = "$$***************************************$$\n\r";
char MSG10[100];

// ===== 4th-order IIR: 4 cascaded first-order sections =====
static float IIR4_LP_Process(IIR4_LP_t *filt, float x)
{
    float y = x;
    for (int i = 0; i < 4; i++)
    {
        filt->stage[i] = filt->stage[i] + IIR_ALPHA * (y - filt->stage[i]);
        y = filt->stage[i];
    }
    return y;
}

// ===== ADC DMA conversion complete callback =====
// Called once per block (ADC_BUF_LEN samples)
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc1_block_ready = 1;
    }
    else if (hadc->Instance == ADC2)
    {
        adc2_block_ready = 1;
    }

    // When both channels have a full block, notify main loop
    if (adc1_block_ready && adc2_block_ready)
    {
        adc_pair_ready = 1;
        // Do NOT clear adc1_block_ready / adc2_block_ready here;
        // main loop will clear them after processing
    }
}

// ===== Trend tracking: Start / Update / Stop =====
static void Trend_Start(TrendTracker_t *tr)
{
    tr->active = 1;
    tr->min_v  = 0.0f;
    tr->max_v  = 0.0f;
    tr->sum_v  = 0.0f;
    tr->count  = 0;
    tr->prev_v = 0.0f;
    tr->state  = TREND_STATE_UNKNOWN;
}

static void Trend_Update(TrendTracker_t *tr, float v)
{
    if (!tr->active)
        return;

    if (tr->count == 0)
    {
        tr->min_v  = v;
        tr->max_v  = v;
        tr->sum_v  = v;
        tr->count  = 1;
        tr->prev_v = v;
        tr->state  = TREND_STATE_FLAT;
        return;
    }

    tr->sum_v += v;
    tr->count++;

    if (v < tr->min_v) tr->min_v = v;
    if (v > tr->max_v) tr->max_v = v;

    float delta = v - tr->prev_v;
    tr->prev_v  = v;

    if (fabsf(delta) < TREND_EPS)
    {
        // Consider as almost unchanged; keep previous UP/DOWN if already set
        if (tr->state == TREND_STATE_UNKNOWN)
            tr->state = TREND_STATE_FLAT;
    }
    else if (delta > 0.0f)
    {
        // Voltage rising
        if (tr->state == TREND_STATE_DOWN)
        {
            tr->state = TREND_STATE_INVALID;  // down first then up → probe shaking
        }
        else if (tr->state == TREND_STATE_UNKNOWN ||
                 tr->state == TREND_STATE_FLAT   ||
                 tr->state == TREND_STATE_UP)
        {
            tr->state = TREND_STATE_UP;
        }
    }
    else // delta < -TREND_EPS
    {
        // Voltage falling
        if (tr->state == TREND_STATE_UP)
        {
            tr->state = TREND_STATE_INVALID;  // up first then down → probe shaking
        }
        else if (tr->state == TREND_STATE_UNKNOWN ||
                 tr->state == TREND_STATE_FLAT   ||
                 tr->state == TREND_STATE_DOWN)
        {
            tr->state = TREND_STATE_DOWN;
        }
    }
}

static void Trend_Stop(TrendTracker_t *tr)
{
    tr->active = 0;
}

static const char* TrendStateToText(TrendState_t st)
{
    switch (st)
    {
    case TREND_STATE_FLAT:    return "FLAT";
    case TREND_STATE_UP:      return "UP";
    case TREND_STATE_DOWN:    return "DOWN";
    case TREND_STATE_INVALID: return "INVALID";
    case TREND_STATE_UNKNOWN:
    default:                  return "UNKNOWN";
    }
}

/* ===== NTC voltage → temperature lookup table (0~100°C) ===== */

typedef struct {
    float v;   // output voltage (V)
    float t;   // corresponding temperature (°C)
} NTC_Point_t;

static const NTC_Point_t ntc_table[] = {
    { 0.771845f, 0.0f },
    { 0.802445f, 1.0f },
    { 0.833655f, 2.0f },
    { 0.865458f, 3.0f },
    { 0.897822f, 4.0f },
    { 0.930726f, 5.0f },
    { 0.964140f, 6.0f },
    { 0.998034f, 7.0f },
    { 1.032379f, 8.0f },
    { 1.067136f, 9.0f },
    { 1.102278f, 10.0f },
    { 1.137770f, 11.0f },
    { 1.173571f, 12.0f },
    { 1.209646f, 13.0f },
    { 1.245955f, 14.0f },
    { 1.282465f, 15.0f },
    { 1.319135f, 16.0f },
    { 1.355927f, 17.0f },
    { 1.392799f, 18.0f },
    { 1.429710f, 19.0f },
    { 1.466634f, 20.0f },
    { 1.503520f, 21.0f },
    { 1.540336f, 22.0f },
    { 1.577046f, 23.0f },
    { 1.613613f, 24.0f },
    { 1.650000f, 25.0f },
    { 1.686177f, 26.0f },
    { 1.722105f, 27.0f },
    { 1.757769f, 28.0f },
    { 1.793118f, 29.0f },
    { 1.828133f, 30.0f },
    { 1.862786f, 31.0f },
    { 1.897064f, 32.0f },
    { 1.930920f, 33.0f },
    { 1.964344f, 34.0f },
    { 1.997325f, 35.0f },
    { 2.029820f, 36.0f },
    { 2.061843f, 37.0f },
    { 2.093351f, 38.0f },
    { 2.124345f, 39.0f },
    { 2.154807f, 40.0f },
    { 2.184722f, 41.0f },
    { 2.214096f, 42.0f },
    { 2.242914f, 43.0f },
    { 2.271163f, 44.0f },
    { 2.298835f, 45.0f },
    { 2.325942f, 46.0f },
    { 2.352472f, 47.0f },
    { 2.378413f, 48.0f },
    { 2.403794f, 49.0f },
    { 2.428578f, 50.0f },
    { 2.452802f, 51.0f },
    { 2.476455f, 52.0f },
    { 2.499527f, 53.0f },
    { 2.522049f, 54.0f },
    { 2.544000f, 55.0f },
    { 2.565398f, 56.0f },
    { 2.586268f, 57.0f },
    { 2.606573f, 58.0f },
    { 2.626364f, 59.0f },
    { 2.645630f, 60.0f },
    { 2.664384f, 61.0f },
    { 2.682621f, 62.0f },
    { 2.700358f, 63.0f },
    { 2.717593f, 64.0f },
    { 2.734369f, 65.0f },
    { 2.750688f, 66.0f },
    { 2.766530f, 67.0f },
    { 2.781923f, 68.0f },
    { 2.796871f, 69.0f },
    { 2.811382f, 70.0f },
    { 2.825488f, 71.0f },
    { 2.839174f, 72.0f },
    { 2.852475f, 73.0f },
    { 2.865379f, 74.0f },
    { 2.877899f, 75.0f },
    { 2.890047f, 76.0f },
    { 2.901813f, 77.0f },
    { 2.913264f, 78.0f },
    { 2.924365f, 79.0f },
    { 2.935134f, 80.0f },
    { 2.945561f, 81.0f },
    { 2.955691f, 82.0f },
    { 2.965519f, 83.0f },
    { 2.975037f, 84.0f },
    { 2.984265f, 85.0f },
    { 2.993224f, 86.0f },
    { 3.001910f, 87.0f },
    { 3.010317f, 88.0f },
    { 3.018495f, 89.0f },
    { 3.026412f, 90.0f },
    { 3.034092f, 91.0f },
    { 3.041531f, 92.0f },
    { 3.048752f, 93.0f },
    { 3.055725f, 94.0f },
    { 3.062531f, 95.0f },
    { 3.069111f, 96.0f },
    { 3.075489f, 97.0f },
    { 3.081664f, 98.0f },
    { 3.087662f, 99.0f },
    { 3.093479f, 100.0f },
};

#define NTC_TABLE_SIZE (sizeof(ntc_table)/sizeof(ntc_table[0]))

static float NTC_V_to_T(float v)
{
    // Clamp when outside the table range
    if (v <= ntc_table[0].v)
        return ntc_table[0].t;
    if (v >= ntc_table[NTC_TABLE_SIZE - 1].v)
        return ntc_table[NTC_TABLE_SIZE - 1].t;

    // Search within the linear segments
    for (int i = 0; i < NTC_TABLE_SIZE - 1; ++i)
    {
        float v0 = ntc_table[i].v;
        float v1 = ntc_table[i+1].v;

        if (v >= v0 && v <= v1)
        {
            float t0 = ntc_table[i].t;
            float t1 = ntc_table[i+1].t;

            // Linear interpolation: t = t0 + (v - v0)/(v1 - v0) * (t1 - t0)
            float ratio = (v - v0) / (v1 - v0);
            return t0 + ratio * (t1 - t0);
        }
    }

    // Should not reach here; defensive return
    return ntc_table[0].t;
}

// ADC2: Vout → temperature T
// vout unit: V (float)
static float ADC2_V_to_T(float vout)
{
    const float denom_base = 10.0f * 3.3f;  // 10 * 3.3
    float k   = vout / denom_base;          // k = Vout / (10*3.3)
    float num = 4000.0f * k;                // numerator
    float den = 1.0f - 2.0f * k;            // part of denominator

    if (den == 0.0f) {
        // Should not happen (Vout can't reach 16.5V), just be defensive
        return 0.0f;
    }

    float T = (num / den) / 3.85f;          // then divide by 3.85
    return T;                               // unit: °C (assuming 3.85 Ω/°C)
}

static void ProcessAdcPair(void)
{
    if (!adc_pair_ready)
        return;

    adc_pair_ready = 0;

    float sum1 = 0.0f;
    float sum2 = 0.0f;

    for (int i = 0; i < ADC_BUF_LEN; ++i)
    {
        float v1 = (float)adc1_buf[i] * (ADC_REF_VOLT / ADC_MAX_VAL);
        v1 += ADC1_READING_OFFSET + ADC1_VOLT_OFFSET_POS - ADC1_VOLT_OFFSET_NEG;
        float y1 = IIR4_LP_Process(&adc1_filter, v1);
        sum1 += y1;

        float v2 = (float)adc2_buf[i] * (ADC_REF_VOLT / ADC_MAX_VAL);
        v2 += ADC2_READING_OFFSET + ADC2_VOLT_OFFSET_POS - ADC2_VOLT_OFFSET_NEG;
        float y2 = IIR4_LP_Process(&adc2_filter, v2);
        sum2 += y2;
    }

    adc1_avg_v = sum1 / (float)ADC_BUF_LEN;
    adc2_avg_v = sum2 / (float)ADC_BUF_LEN;

    adc1_block_ready = 0;
    adc2_block_ready = 0;

    // Feed current block's IIR-filtered voltages into trend tracker
    // (only effective when active == 1)
    Trend_Update(&adc1_trend, adc1_avg_v);
    Trend_Update(&adc2_trend, adc2_avg_v);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  ow_init_t ow_init_struct;
  ow_init_struct.tim_handle   = &htim2;
  ow_init_struct.gpio         = GPIOA;
  ow_init_struct.pin          = GPIO_PIN_9;      // your 1-Wire pin
  ow_init_struct.tim_cb       = ds18_tim_cb;
  ow_init_struct.done_cb      = NULL;           // set if you want async callback later
  ow_init_struct.rom_id_filter = DS18B20_ID;    // 0x28, filter DS18B20 devices

  ds18b20_init(&ds18, &ow_init_struct);

  // Scan DS18B20 devices on 1-Wire bus
  ds18b20_update_rom_id(&ds18);
  while (ds18b20_is_busy(&ds18));

  // Configure alarm thresholds & resolution
  ds18b20_config_t ds18_conf = {
      .alarm_high = 50,
      .alarm_low  = -50,
      .cnv_bit    = DS18B20_CNV_BIT_12
  };

  ds18b20_conf(&ds18, &ds18_conf);
  while (ds18b20_is_busy(&ds18));

  // ==== Initialize MAX31855 / thermocouple ====
  HAL_UART_Transmit(&huart2, (uint8_t *)MSG,  strlen(MSG),  100);
  HAL_UART_Transmit(&huart2, (uint8_t *)MSG0, strlen(MSG0), 100);

  if (!begin()) {
      // Sensor init failed, print error and halt
      HAL_UART_Transmit(&huart2, (uint8_t *)MSG1, strlen(MSG1), 100);
      while (1) {
          HAL_Delay(10);
      }
  }

  HAL_UART_Transmit(&huart2, (uint8_t *)MSG2, strlen(MSG2), 100);

  // ==== Start ADC1 / ADC2 + DMA (TIM3 trigger, 1 kSa/s) ====
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_buf, ADC_BUF_LEN);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_buf, ADC_BUF_LEN);
  HAL_TIM_Base_Start(&htim3);   // TIM3: PSC=8399, ARR=9 → 1kHz trigger

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int16_t temp_c[2];
  uint32_t last_temp_ms = 0;   // last refresh time of temp sensors (ms)

  while (1)
  {
      uint32_t now = HAL_GetTick();
      ProcessAdcPair();

      /* ===== 1. Every 1 s: read DS18B20 & MAX31855 and print ===== */
      if (now - last_temp_ms >= 1000)
      {
        last_temp_ms = now;

        // === Start ADC1 / ADC2 trend tracking window ===
        Trend_Start(&adc1_trend);
        Trend_Start(&adc2_trend);

        ds18b20_cnv(&ds18);

        // While DS18B20 is busy, keep processing ADC blocks (IIR + trend)
        while (ds18b20_is_busy(&ds18))
        {
            ProcessAdcPair();
        }
        while (!ds18b20_is_cnv_done(&ds18))
        {
            ProcessAdcPair();
        }

        // End of trend window
        Trend_Stop(&adc1_trend);
        Trend_Stop(&adc2_trend);

        // ===== Select representative voltages per channel based on trend and convert to temperature =====
        float trend_v1 = 0.0f, temp1 = 0.0f;
        float trend_v2 = 0.0f, temp2 = 0.0f;
        TrendState_t st1 = adc1_trend.state;
        TrendState_t st2 = adc2_trend.state;

        // --- ADC1 ---
        if (adc1_trend.count == 0)
        {
            // No samples, mark as invalid
            st1 = TREND_STATE_INVALID;
        }
        else if (st1 == TREND_STATE_INVALID || st1 == TREND_STATE_UNKNOWN)
        {
            // Invalid state, do not output temperature
        }
        else
        {
            if (st1 == TREND_STATE_UP)
                trend_v1 = adc1_trend.max_v;
            else if (st1 == TREND_STATE_DOWN)
                trend_v1 = adc1_trend.min_v;
            else // FLAT
                trend_v1 = adc1_trend.sum_v / (float)adc1_trend.count;

            temp1 = NTC_V_to_T(trend_v1);
        }

        // --- ADC2 ---
        if (adc2_trend.count == 0)
        {
            st2 = TREND_STATE_INVALID;
        }
        else if (st2 == TREND_STATE_INVALID || st2 == TREND_STATE_UNKNOWN)
        {
            // Invalid state, do not output temperature
        }
        else
        {
            if (st2 == TREND_STATE_UP)
                trend_v2 = adc2_trend.max_v;
            else if (st2 == TREND_STATE_DOWN)
                trend_v2 = adc2_trend.min_v;
            else // FLAT
                trend_v2 = adc2_trend.sum_v / (float)adc2_trend.count;

            temp2 = ADC2_V_to_T(trend_v2);
        }

        // Convert to integer temperatures (×100) & voltages in mV
        int t1_100 = (int)(temp1 * 100.0f + 0.5f);
        int t2_100 = (int)(temp2 * 100.0f + 0.5f);
        int v1_mV  = (int)(trend_v1 * 1000.0f + 0.5f);
        int v2_mV  = (int)(trend_v2 * 1000.0f + 0.5f);
        int t1_100_adj = t1_100 + (int)(t1_offset * 100.0f);

        const char *st1_txt = TrendStateToText(st1);
        const char *st2_txt = TrendStateToText(st2);

        // Format one full line
        int len = snprintf(
            uart_buf,
            sizeof(uart_buf),
            "ADC_TREND: CH1=%d.%02d C, V=%d.%03d V (%s), "
            "CH2=%d.%02d C, V=%d.%03d V (%s)\r\n",
            // CH1 temperature
            t1_100_adj / 100,
            (t1_100_adj >= 0 ? t1_100_adj % 100 : -t1_100_adj % 100),
            // CH1 voltage
            v1_mV / 1000,
            (v1_mV >= 0 ? v1_mV % 1000 : -v1_mV % 1000),
            st1_txt,
            // CH2 temperature
            t2_100 / 100,
            (t2_100 >= 0 ? t2_100 % 100 : -t2_100 % 100),
            // CH2 voltage
            v2_mV / 1000,
            (v2_mV >= 0 ? v2_mV % 1000 : -v2_mV % 1000),
            st2_txt
        );

        // Safety: check snprintf result
        if (len > 0 && len < (int)sizeof(uart_buf))
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, len, 100);
        }

        // ===== 2. DS18B20 normal reading (keep original logic) =====
        ds18b20_req_read(&ds18, 0);
        while (ds18b20_is_busy(&ds18));
        temp_c[0] = ds18b20_read_c(&ds18);

        ds18b20_req_read(&ds18, 1);
        while (ds18b20_is_busy(&ds18));
        temp_c[1] = ds18b20_read_c(&ds18);

        int16_t t0 = temp_c[0];

        len = snprintf(uart_buf, sizeof(uart_buf),
                       "Digital Temperature = %d.%02d C \r\n",
                       t0 / 100, (t0 >= 0 ? t0 % 100 : -t0 % 100));
        HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, len, 100);

        /* ===== 3. MAX31855 thermocouple reading ===== */

        // Internal cold-junction temperature
        float tin = readInternal();
        int tin100 = (int)(tin * 100.0f);
        sprintf(MSG10, "Internal Temp = %d.%02d \r\n",
                tin100 / 100,
                (tin100 >= 0 ? tin100 % 100 : -tin100 % 100));
        HAL_UART_Transmit(&huart2, (uint8_t *)MSG10, strlen(MSG10), 100);

        double c = readCelsius();
        if (!isnan(c)) {
            // Celsius
            float c_f = (float)c;
            int c100 = (int)(c_f * 100.0f);
            sprintf(MSG7, "C = %d.%02d \r\n",
                    c100 / 100,
                    (c100 >= 0 ? c100 % 100 : -c100 % 100));
            HAL_UART_Transmit(&huart2, (uint8_t *)MSG7, strlen(MSG7), 100);

            // Fahrenheit
            float f = readFahrenheit();
            int f100 = (int)(f * 100.0f);
            sprintf(MSG8, "F = %d.%02d \r\n",
                    f100 / 100,
                    (f100 >= 0 ? f100 % 100 : -f100 % 100));
            HAL_UART_Transmit(&huart2, (uint8_t *)MSG8, strlen(MSG8), 100);

            HAL_UART_Transmit(&huart2, (uint8_t *)MSG9, strlen(MSG9), 100);
        }
        else
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)MSG3, strlen(MSG3), 100);
            uint8_t e = readError();
            if (e & MAX31855_FAULT_OPEN)
                HAL_UART_Transmit(&huart2, (uint8_t *)MSG4, strlen(MSG4), 100);
            if (e & MAX31855_FAULT_SHORT_GND)
                HAL_UART_Transmit(&huart2, (uint8_t *)MSG5, strlen(MSG5), 100);
            if (e & MAX31855_FAULT_SHORT_VCC)
                HAL_UART_Transmit(&huart2, (uint8_t *)MSG6, strlen(MSG6), 100);
        }
      }

      /* USER CODE END WHILE */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n") */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
