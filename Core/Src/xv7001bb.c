/**
  ******************************************************************************
  * @file    xv7001bb.c
  * @brief   XV7001BB SPI driver with fixed-point angle conversion.
  ******************************************************************************
  */

#include "xv7001bb.h"

#include "main.h"
#include "spi.h"

#define XV_ANGLE_MICRODEG_PER_TURN      360000000
#define XV_ANGLE_MICRODEG_HALF_TURN     180000000
#define XV_TIMESTAMP_GUARD_COUNT        4U

volatile xv7001bb_debug_t g_xv7001bb_debug;

static uint8_t s_initialized = 0U;
static int32_t s_angle_microdeg = 0;
static uint32_t s_previous_time_us = 0U;
static uint8_t s_angle_time_valid = 0U;
static int32_t s_bias_samples[XV7001BB_BIAS_SAMPLE_COUNT];
static int32_t s_bias_raw = 0;
static int32_t s_bias_sum_raw = 0;
static uint32_t s_bias_attempt_count = 0U;
static uint32_t s_bias_valid_count = 0U;
static uint32_t s_bias_rejected_count = 0U;
static uint8_t s_bias_state = XV7001BB_BIAS_STATE_IDLE;

static xv_status_t xv_map_hal_status(HAL_StatusTypeDef status)
{
  switch (status)
  {
    case HAL_OK:
      return XV_OK;
    case HAL_BUSY:
      return XV_ERR_BUSY;
    case HAL_TIMEOUT:
      return XV_ERR_TIMEOUT;
    default:
      return XV_ERR_IO;
  }
}

static xv_status_t xv_record_spi_status(HAL_StatusTypeDef status)
{
  xv_status_t result = xv_map_hal_status(status);

  g_xv7001bb_debug.last_spi_status = (uint32_t)status;
  if (result != XV_OK)
  {
    g_xv7001bb_debug.spi_error_count++;
  }
  return result;
}

static uint8_t xv_is_readable_register(uint8_t reg)
{
  switch (reg)
  {
    case XV7001BB_REG_DSP_CTL1:
    case XV7001BB_REG_DSP_CTL2:
    case XV7001BB_REG_DSP_CTL3:
    case XV7001BB_REG_STATUS:
    case XV7001BB_REG_TEMPERATURE:
    case XV7001BB_REG_RATE:
    case XV7001BB_REG_OUTPUT_CTL:
    case XV7001BB_REG_TEMP_FORMAT:
    case XV7001BB_REG_INTERFACE_CTL:
      return 1U;
    default:
      return 0U;
  }
}

static uint8_t xv_is_writable_register(uint8_t reg)
{
  switch (reg)
  {
    case XV7001BB_REG_DSP_CTL1:
    case XV7001BB_REG_DSP_CTL2:
    case XV7001BB_REG_DSP_CTL3:
    case XV7001BB_REG_OUTPUT_CTL:
    case XV7001BB_REG_TEMP_FORMAT:
    case XV7001BB_REG_INTERFACE_CTL:
      return 1U;
    default:
      return 0U;
  }
}

static uint8_t xv_is_valid_command(uint8_t command)
{
  switch (command)
  {
    case XV7001BB_REG_SLEEP_IN:
    case XV7001BB_REG_SLEEP_OUT:
    case XV7001BB_REG_STANDBY:
    case XV7001BB_REG_SOFTWARE_RESET:
      return 1U;
    default:
      return 0U;
  }
}

static uint8_t xv_is_legal_power_state(uint8_t status)
{
  uint8_t state = status & XV7001BB_STATUS_STATE_MASK;

  return (uint8_t)((state == XV7001BB_STATUS_POR) ||
                   (state == XV7001BB_STATUS_STANDBY) ||
                   (state == XV7001BB_STATUS_SLEEP) ||
                   (state == XV7001BB_STATUS_SLEEP_OUT));
}

static uint8_t xv_spi_ready(void)
{
  return (uint8_t)(HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_READY);
}

static xv_status_t xv_transfer(const uint8_t *tx_data,
                               uint8_t *rx_data,
                               uint16_t length)
{
  HAL_StatusTypeDef hal_status;

  if (!xv_spi_ready())
  {
    return XV_ERR_NOT_READY;
  }

  hal_status = SPI_TransmitReceive(tx_data,
                                   rx_data,
                                   length,
                                   SPI_TRANSFER_TIMEOUT_MS);
  return xv_record_spi_status(hal_status);
}

static uint32_t xv_get_timestamp_us(void)
{
  uint32_t tick_before;
  uint32_t tick_after;
  uint32_t timer_count;
  uint32_t guard;

  /*
   * TIM4 supplies the 1 MHz HAL time base. Reading both sides of TIM4->CNT
   * prevents a tick rollover from mixing two adjacent milliseconds.
   */
  guard = 0U;
  do
  {
    tick_before = HAL_GetTick();
    timer_count = TIM4->CNT;
    tick_after = HAL_GetTick();
    guard++;
  } while ((tick_before != tick_after) && (guard < XV_TIMESTAMP_GUARD_COUNT));

  if (tick_before != tick_after)
  {
    tick_before = tick_after;
    timer_count = TIM4->CNT;
  }

  return (tick_before * 1000U) + timer_count;
}

static uint8_t xv_rate_is_valid(int32_t raw_rate)
{
  return (uint8_t)((raw_rate >= -XV7001BB_RATE_RAW_LIMIT) &&
                   (raw_rate <= XV7001BB_RATE_RAW_LIMIT));
}

static int32_t xv_saturate_rate(int32_t corrected_raw_rate)
{
  if (corrected_raw_rate > XV7001BB_RATE_RAW_SATURATION)
  {
    g_xv7001bb_debug.rate_saturation_count++;
    return XV7001BB_RATE_RAW_SATURATION;
  }
  if (corrected_raw_rate < -XV7001BB_RATE_RAW_SATURATION)
  {
    g_xv7001bb_debug.rate_saturation_count++;
    return -XV7001BB_RATE_RAW_SATURATION;
  }
  return corrected_raw_rate;
}

static void xv_reset_bias_state(void)
{
  s_bias_raw = 0;
  s_bias_sum_raw = 0;
  s_bias_attempt_count = 0U;
  s_bias_valid_count = 0U;
  s_bias_rejected_count = 0U;
  s_bias_state = XV7001BB_BIAS_STATE_IDLE;

  g_xv7001bb_debug.bias_sample_count = 0U;
  g_xv7001bb_debug.bias_valid_sample_count = 0U;
  g_xv7001bb_debug.bias_rejected_count = 0U;
  g_xv7001bb_debug.bias_variance_raw2 = 0U;
  g_xv7001bb_debug.bias_raw = 0;
  g_xv7001bb_debug.bias_peak_to_peak_raw = 0;
  g_xv7001bb_debug.bias_dps = 0.0f;
  g_xv7001bb_debug.corrected_raw_rate = 0;
  g_xv7001bb_debug.rate_dps = 0.0f;
  g_xv7001bb_debug.bias_state = XV7001BB_BIAS_STATE_IDLE;
  g_xv7001bb_debug.bias_valid = 0U;
}

static xv_status_t xv_bias_unavailable_status(void)
{
  if (s_bias_state == XV7001BB_BIAS_STATE_FAILED)
  {
    return XV_ERR_CALIBRATION_FAILED;
  }
  return XV_ERR_NOT_CALIBRATED;
}

static void xv_sort_bias_samples(uint32_t count)
{
  uint32_t index;

  for (index = 1U; index < count; index++)
  {
    int32_t value = s_bias_samples[index];
    uint32_t insert_index = index;

    while ((insert_index > 0U) &&
           (s_bias_samples[insert_index - 1U] > value))
    {
      s_bias_samples[insert_index] =
          s_bias_samples[insert_index - 1U];
      insert_index--;
    }
    s_bias_samples[insert_index] = value;
  }
}

static void xv_complete_bias_calibration(void)
{
  int32_t mean_raw;
  int32_t minimum_raw;
  int32_t maximum_raw;
  int32_t sample_raw;
  int32_t delta_raw;
  int32_t absolute_mean_raw;
  uint64_t variance_raw2;
  uint64_t sum_square_raw = 0U;
  int64_t trimmed_sum_raw = 0;
  uint32_t first_sample;
  uint32_t end_sample;
  uint32_t trimmed_count;
  uint32_t index;

  if (s_bias_valid_count < XV7001BB_BIAS_MIN_VALID_SAMPLES)
  {
    s_bias_state = XV7001BB_BIAS_STATE_FAILED;
    s_bias_raw = 0;
  }
  else
  {
    xv_sort_bias_samples(s_bias_valid_count);
    first_sample = s_bias_valid_count / XV7001BB_BIAS_TRIM_DIVISOR;
    end_sample = s_bias_valid_count - first_sample;
    trimmed_count = end_sample - first_sample;

    for (index = first_sample; index < end_sample; index++)
    {
      trimmed_sum_raw += s_bias_samples[index];
    }
    if (trimmed_sum_raw >= 0)
    {
      mean_raw =
          (int32_t)((trimmed_sum_raw + (trimmed_count / 2U)) /
                    trimmed_count);
    }
    else
    {
      mean_raw =
          (int32_t)((trimmed_sum_raw - (trimmed_count / 2U)) /
                    trimmed_count);
    }

    minimum_raw = s_bias_samples[first_sample];
    maximum_raw = s_bias_samples[first_sample];
    for (index = first_sample; index < end_sample; index++)
    {
      sample_raw = s_bias_samples[index];
      if (sample_raw < minimum_raw)
      {
        minimum_raw = sample_raw;
      }
      if (sample_raw > maximum_raw)
      {
        maximum_raw = sample_raw;
      }

      delta_raw = sample_raw - mean_raw;
      sum_square_raw += (uint64_t)((int64_t)delta_raw * delta_raw);
    }

    variance_raw2 = sum_square_raw / trimmed_count;
    absolute_mean_raw = (mean_raw < 0) ? -mean_raw : mean_raw;

    g_xv7001bb_debug.bias_raw = mean_raw;
    g_xv7001bb_debug.bias_peak_to_peak_raw =
        maximum_raw - minimum_raw;
    g_xv7001bb_debug.bias_variance_raw2 = variance_raw2;

    if ((absolute_mean_raw <= XV7001BB_BIAS_MAX_OFFSET_RAW) &&
        ((maximum_raw - minimum_raw) <=
         XV7001BB_BIAS_MAX_PEAK_TO_PEAK_RAW) &&
        (variance_raw2 <= XV7001BB_BIAS_MAX_VARIANCE_RAW2))
    {
      s_bias_raw = mean_raw;
      s_bias_state = XV7001BB_BIAS_STATE_READY;
    }
    else
    {
      s_bias_raw = 0;
      s_bias_state = XV7001BB_BIAS_STATE_FAILED;
    }
  }

  g_xv7001bb_debug.bias_dps =
      (float)g_xv7001bb_debug.bias_raw * XV7001BB_RATE_DPS_PER_LSB;
  g_xv7001bb_debug.bias_state = s_bias_state;
  g_xv7001bb_debug.bias_valid =
      (uint8_t)(s_bias_state == XV7001BB_BIAS_STATE_READY);
}

static void xv_process_bias_sample(int32_t raw_rate)
{
  int32_t absolute_raw;

  s_bias_attempt_count++;
  absolute_raw = (raw_rate < 0) ? -raw_rate : raw_rate;

  if (absolute_raw > XV7001BB_BIAS_STATIC_RAW_LIMIT)
  {
    s_bias_rejected_count++;
  }
  else
  {
    s_bias_samples[s_bias_valid_count] = raw_rate;
    s_bias_valid_count++;
    s_bias_sum_raw += raw_rate;
  }

  g_xv7001bb_debug.bias_sample_count = s_bias_attempt_count;
  g_xv7001bb_debug.bias_valid_sample_count = s_bias_valid_count;
  g_xv7001bb_debug.bias_rejected_count = s_bias_rejected_count;

  if (s_bias_attempt_count >= XV7001BB_BIAS_SAMPLE_COUNT)
  {
    xv_complete_bias_calibration();
  }
}

//寄存器地址, 要写入的值
xv_status_t XV7001bb_WriteData(uint8_t reg, uint8_t value)
{
  uint8_t tx_data[2];
  uint8_t rx_data[2];
  xv_status_t status;

  if (!xv_is_writable_register(reg))
  {
    g_xv7001bb_debug.invalid_request_count++;
    return XV_ERR_PARAM;
  }

  tx_data[0] = (uint8_t)(reg & 0x1FU);
  tx_data[1] = value;
  status = xv_transfer(tx_data, rx_data, 2U);
  return status;
}
//当前驱动定义的所有“可读寄存器”都可以通过 ReadReg() 读取
xv_status_t XV7001bb_ReadReg(uint8_t reg, uint8_t *value)
{
  uint8_t tx_data[2] = {0U, 0U};
  uint8_t rx_data[2] = {0U, 0U};
  xv_status_t status;

  if ((value == NULL) || (!xv_is_readable_register(reg)))
  {
    g_xv7001bb_debug.invalid_request_count++;
    return XV_ERR_PARAM;
  }

  tx_data[0] = (uint8_t)(0x80U | (reg & 0x1FU));
  status = xv_transfer(tx_data, rx_data, 2U);
  if (status == XV_OK)
  {
    *value = rx_data[1];
  }
  return status;
}
//读取的是 XV7001BB 芯片内部的状态寄存器
xv_status_t XV7001bb_ReadStatus(uint8_t *status)
{
  xv_status_t result;
  uint8_t raw_status = 0U;

  if (status == NULL)
  {
    g_xv7001bb_debug.invalid_request_count++;
    return XV_ERR_PARAM;
  }

  result = XV7001bb_ReadReg(XV7001BB_REG_STATUS, &raw_status);
  if (result == XV_OK)
  {
    g_xv7001bb_debug.status_raw = raw_status;
    g_xv7001bb_debug.status_proc_ok =
        (uint8_t)((raw_status & XV7001BB_STATUS_PROCESS_OK) != 0U);
    g_xv7001bb_debug.status_power_state =
        (uint8_t)(raw_status & XV7001BB_STATUS_STATE_MASK);
    *status = raw_status;
  }
  return result;
}
//函数的作用就是向寄存器发送命令：睡眠 0x05、唤醒 0x06、待机 0x07、复位 0x09
xv_status_t XV7001bb_SendCommand(uint8_t command)
{
  uint8_t tx_data[1];
  uint8_t rx_data[1] = {0U};

  if (!xv_is_valid_command(command))
  {
    g_xv7001bb_debug.invalid_request_count++;
    return XV_ERR_PARAM;
  }

  tx_data[0] = (uint8_t)(command & 0x1FU);
  return xv_transfer(tx_data, rx_data, 1U);
}
//调用者可以直接拿到“摄氏度”数值。
xv_status_t XV7001bb_ReadTmp(float *temp_c)
{
  uint8_t tx_data[3] = {0x88U, 0U, 0U};
  uint8_t rx_data[3] = {0U, 0U, 0U};
  uint16_t raw12;
  int16_t signed12;
  float temperature;
  xv_status_t status;

  if (temp_c == NULL)
  {
    g_xv7001bb_debug.invalid_request_count++;
    return XV_ERR_PARAM;
  }
  if (!s_initialized)
  {
    return XV_ERR_NOT_READY;
  }

  status = xv_transfer(tx_data, rx_data, 3U);
  if (status != XV_OK)
  {
    return status;
  }

  raw12 = (uint16_t)(((uint16_t)rx_data[1] << 4) |
                     ((uint16_t)rx_data[2] >> 4));
  if ((raw12 & 0x0800U) != 0U)
  {
    signed12 = (int16_t)(raw12 | 0xF000U);
  }
  else
  {
    signed12 = (int16_t)raw12;
  }

  temperature = (float)signed12 * XV7001BB_TEMP_C_PER_LSB;
  g_xv7001bb_debug.raw_temperature = (uint32_t)(uint16_t)signed12;
  g_xv7001bb_debug.temperature_c = temperature;

  if ((temperature < -40.0f) || (temperature > 125.0f))
  {
    return XV_ERR_DATA_INVALID;
  }

  *temp_c = temperature;
  return XV_OK;
}
//读取24位有符号角速度，实测有效比例因子为17920 LSB/(°/s)
xv_status_t XV7001bb_ReadRawRate(int32_t *raw_rate)
{
  uint8_t tx_data[4] = {0x8AU, 0U, 0U, 0U};
  uint8_t rx_data[4] = {0U, 0U, 0U, 0U};
  uint32_t raw24;
  xv_status_t status;

  if (raw_rate == NULL)
  {
    g_xv7001bb_debug.invalid_request_count++;
    return XV_ERR_PARAM;
  }
  if (!s_initialized)
  {
    return XV_ERR_NOT_READY;
  }

  status = xv_transfer(tx_data, rx_data, 4U);
  if (status != XV_OK)
  {
    s_angle_time_valid = 0U;
    return status;
  }

  raw24 = ((uint32_t)rx_data[1] << 16) |
          ((uint32_t)rx_data[2] << 8) |
          (uint32_t)rx_data[3];
  if ((raw24 & 0x00800000U) != 0U)
  {
    raw24 |= 0xFF000000U;
  }
  *raw_rate = (int32_t)raw24;
  g_xv7001bb_debug.raw_rate = *raw_rate;
  g_xv7001bb_debug.sample_count++;

  if (s_bias_state == XV7001BB_BIAS_STATE_COLLECTING)
  {
    xv_process_bias_sample(*raw_rate);
  }

  if (!xv_rate_is_valid(*raw_rate))
  {
    s_angle_time_valid = 0U;
    return XV_ERR_DATA_INVALID;
  }
  return XV_OK;
}

xv_status_t XV7001bb_StartBiasCalibration(void)
{
  if (!s_initialized)
  {
    return XV_ERR_NOT_READY;
  }

  xv_reset_bias_state();
  s_bias_state = XV7001BB_BIAS_STATE_COLLECTING;
  g_xv7001bb_debug.bias_state = s_bias_state;

  s_angle_microdeg = 0;
  s_previous_time_us = 0U;
  s_angle_time_valid = 0U;
  g_xv7001bb_debug.angle_deg = 0.0f;
  g_xv7001bb_debug.angle_time_valid = 0U;

  return XV_OK;
}

xv_status_t XV7001bb_GetBias(float *bias_dps)
{
  if (bias_dps == NULL)
  {
    g_xv7001bb_debug.invalid_request_count++;
    return XV_ERR_PARAM;
  }
  if (!s_initialized)
  {
    return XV_ERR_NOT_READY;
  }
  if (s_bias_state != XV7001BB_BIAS_STATE_READY)
  {
    return xv_bias_unavailable_status();
  }

  *bias_dps = (float)s_bias_raw * XV7001BB_RATE_DPS_PER_LSB;
  return XV_OK;
}

//读取经零偏校准后的角速度，单位为 °/s。
xv_status_t XV7001bb_ReadRate(float *rate_dps)
{
  int32_t raw_rate;
  xv_status_t status;

  if (rate_dps == NULL)
  {
    g_xv7001bb_debug.invalid_request_count++;
    return XV_ERR_PARAM;
  }

  status = XV7001bb_ReadRawRate(&raw_rate);
  if (status == XV_OK)
  {
    if (s_bias_state != XV7001BB_BIAS_STATE_READY)
    {
      return xv_bias_unavailable_status();
    }

    raw_rate = xv_saturate_rate(raw_rate - s_bias_raw);
    *rate_dps = (float)raw_rate * XV7001BB_RATE_DPS_PER_LSB;
    g_xv7001bb_debug.rate_dps = *rate_dps;
  }
  return status;
}
//将角速度积分为角度，单位为度。
xv_status_t XV7001bb_ConvertRateToAngle(int32_t raw_rate,
                                        uint32_t sample_time_us,
                                        float *angle_deg)
{
  uint32_t dt_us;
  int32_t delta_microdeg;
  int64_t rate_time_product;
  int32_t corrected_raw_rate;

  if (angle_deg == NULL)
  {
    g_xv7001bb_debug.invalid_request_count++;
    return XV_ERR_PARAM;
  }
  if ((!s_initialized) || (!xv_rate_is_valid(raw_rate)))
  {
    s_angle_time_valid = 0U;
    g_xv7001bb_debug.rate_invalid_count++;
    return s_initialized ? XV_ERR_DATA_INVALID : XV_ERR_NOT_READY;
  }
  if (s_bias_state != XV7001BB_BIAS_STATE_READY)
  {
    s_angle_time_valid = 0U;
    g_xv7001bb_debug.angle_time_valid = 0U;
    return xv_bias_unavailable_status();
  }

  corrected_raw_rate = xv_saturate_rate(raw_rate - s_bias_raw);
  g_xv7001bb_debug.corrected_raw_rate = corrected_raw_rate;
  g_xv7001bb_debug.rate_dps =
      (float)corrected_raw_rate * XV7001BB_RATE_DPS_PER_LSB;

  if (!s_angle_time_valid)
  {
    s_previous_time_us = sample_time_us;
    s_angle_time_valid = 1U;
    dt_us = 0U;
  }
  else
  {
    dt_us = sample_time_us - s_previous_time_us;
    s_previous_time_us = sample_time_us;

    if (dt_us == 0U)
    {
      g_xv7001bb_debug.sample_dt_us = dt_us;
      g_xv7001bb_debug.dt_gap_count++;
      g_xv7001bb_debug.angle_time_valid = s_angle_time_valid;
      g_xv7001bb_debug.angle_deg =
          (float)s_angle_microdeg * XV7001BB_ANGLE_DEG_PER_UDEG;
      *angle_deg = g_xv7001bb_debug.angle_deg;
      return XV_OK;
    }
    if (dt_us > XV7001BB_MAX_INTEGRATION_DT_US)
    {
      dt_us = XV7001BB_MAX_INTEGRATION_DT_US;
      g_xv7001bb_debug.dt_gap_count++;
    }

    g_xv7001bb_debug.integration_count++;
    if ((g_xv7001bb_debug.sample_dt_min_us == 0U) ||
        (dt_us < g_xv7001bb_debug.sample_dt_min_us))
    {
      g_xv7001bb_debug.sample_dt_min_us = dt_us;
    }
    if (dt_us > g_xv7001bb_debug.sample_dt_max_us)
    {
      g_xv7001bb_debug.sample_dt_max_us = dt_us;
    }

    /*
     * A 24-bit rate code multiplied by dt_us exceeds signed 32-bit range
     * near full scale, so keep the product in 64 bits before scaling.
     */
    rate_time_product =
        (int64_t)corrected_raw_rate * (int64_t)dt_us;
    delta_microdeg =
        (int32_t)(rate_time_product / XV7001BB_RATE_RAW_CODES_PER_DPS);
    s_angle_microdeg += delta_microdeg;

    if (s_angle_microdeg >= XV_ANGLE_MICRODEG_HALF_TURN)
    {
      s_angle_microdeg -= XV_ANGLE_MICRODEG_PER_TURN;
    }
    else if (s_angle_microdeg < -XV_ANGLE_MICRODEG_HALF_TURN)
    {
      s_angle_microdeg += XV_ANGLE_MICRODEG_PER_TURN;
    }
  }

  g_xv7001bb_debug.sample_time_us = sample_time_us;
  g_xv7001bb_debug.sample_dt_us = dt_us;
  g_xv7001bb_debug.angle_time_valid = s_angle_time_valid;
  g_xv7001bb_debug.angle_deg =
      (float)s_angle_microdeg * XV7001BB_ANGLE_DEG_PER_UDEG;
  *angle_deg = g_xv7001bb_debug.angle_deg;
  return XV_OK;
}
//angle_deg：当前计算角度，单位是度
xv_status_t XV7001bb_ReadAngle(float *angle_deg)
{
  int32_t raw_rate;
  uint32_t sample_time_us;
  xv_status_t status;

  if (angle_deg == NULL)
  {
    g_xv7001bb_debug.invalid_request_count++;
    return XV_ERR_PARAM;
  }

  status = XV7001bb_ReadRawRate(&raw_rate);
  if (status != XV_OK)
  {
    s_angle_time_valid = 0U;
    g_xv7001bb_debug.angle_time_valid = 0U;
    return status;
  }

  sample_time_us = xv_get_timestamp_us();
  return XV7001bb_ConvertRateToAngle(raw_rate, sample_time_us, angle_deg);
}

xv_status_t XV7001bb_Init(void)
{
  uint8_t status_register;
  uint8_t control_default;
  uint8_t output_default;
  uint8_t filter_default;
  uint8_t readback;
  xv_status_t status;

  s_initialized = 0U;
  s_angle_microdeg = 0;
  s_previous_time_us = 0U;
  s_angle_time_valid = 0U;
  xv_reset_bias_state();
  g_xv7001bb_debug.angle_deg = 0.0f;
  g_xv7001bb_debug.angle_time_valid = 0U;
  g_xv7001bb_debug.initialized = 0U;
  g_xv7001bb_debug.integration_count = 0U;
  g_xv7001bb_debug.dt_gap_count = 0U;
  g_xv7001bb_debug.rate_invalid_count = 0U;
  g_xv7001bb_debug.rate_saturation_count = 0U;
  g_xv7001bb_debug.sample_dt_min_us = 0U;
  g_xv7001bb_debug.sample_dt_max_us = 0U;
  g_xv7001bb_debug.corrected_raw_rate = 0;
  g_xv7001bb_debug.init_calls++;

  if (!xv_spi_ready())
  {
    g_xv7001bb_debug.init_status = (uint32_t)XV_ERR_NOT_READY;
    return XV_ERR_NOT_READY;
  }

  /* Keep the bus deasserted before the first read transaction. */
  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);

  status = XV7001bb_ReadStatus(&status_register);
  if (status != XV_OK)
  {
    g_xv7001bb_debug.init_status = (uint32_t)status;
    return status;
  }
  if (!xv_is_legal_power_state(status_register))
  {
    g_xv7001bb_debug.init_status = (uint32_t)XV_ERR_NOT_READY;
    return XV_ERR_NOT_READY;
  }

  status = XV7001bb_ReadReg(XV7001BB_REG_INTERFACE_CTL, &control_default);
  if (status != XV_OK)
  {
    g_xv7001bb_debug.init_status = (uint32_t)status;
    return status;
  }
  g_xv7001bb_debug.reg_control_default = control_default;

  status = XV7001bb_ReadReg(XV7001BB_REG_OUTPUT_CTL, &output_default);
  if (status != XV_OK)
  {
    g_xv7001bb_debug.init_status = (uint32_t)status;
    return status;
  }
  g_xv7001bb_debug.reg_output_default = output_default;

  status = XV7001bb_ReadReg(XV7001BB_REG_DSP_CTL2, &filter_default);
  if (status != XV_OK)
  {
    g_xv7001bb_debug.init_status = (uint32_t)status;
    return status;
  }
  g_xv7001bb_debug.reg_filter_default = filter_default;

  status = XV7001bb_WriteData(XV7001BB_REG_INTERFACE_CTL, 0x00U);
  if (status != XV_OK)
  {
    g_xv7001bb_debug.init_status = (uint32_t)status;
    return status;
  }
  status = XV7001bb_ReadReg(XV7001BB_REG_INTERFACE_CTL, &readback);
  if ((status != XV_OK) || (readback != 0x00U))
  {
    g_xv7001bb_debug.init_status =
        (uint32_t)((status == XV_OK) ? XV_ERR_IO : status);
    return (status == XV_OK) ? XV_ERR_IO : status;
  }
  g_xv7001bb_debug.reg_control_value = readback;

  /* Select the sensor's 24-bit rate output format. */
  status = XV7001bb_WriteData(XV7001BB_REG_OUTPUT_CTL, 0x05U);
  if (status != XV_OK)
  {
    g_xv7001bb_debug.init_status = (uint32_t)status;
    return status;
  }
  status = XV7001bb_ReadReg(XV7001BB_REG_OUTPUT_CTL, &readback);
  if ((status != XV_OK) || (readback != 0x05U))
  {
    g_xv7001bb_debug.init_status =
        (uint32_t)((status == XV_OK) ? XV_ERR_IO : status);
    return (status == XV_OK) ? XV_ERR_IO : status;
  }
  g_xv7001bb_debug.reg_output_value = readback;

  /* 2nd-order LPF at 35 Hz for the 100 Hz application sampling cadence. */
  status = XV7001bb_WriteData(XV7001BB_REG_DSP_CTL2, 0x01U);
  if (status != XV_OK)
  {
    g_xv7001bb_debug.init_status = (uint32_t)status;
    return status;
  }
  status = XV7001bb_ReadReg(XV7001BB_REG_DSP_CTL2, &readback);
  if ((status != XV_OK) || (readback != 0x01U))
  {
    g_xv7001bb_debug.init_status =
        (uint32_t)((status == XV_OK) ? XV_ERR_IO : status);
    return (status == XV_OK) ? XV_ERR_IO : status;
  }
  g_xv7001bb_debug.reg_filter_value = readback;

  s_initialized = 1U;
  g_xv7001bb_debug.initialized = 1U;
  g_xv7001bb_debug.init_status = (uint32_t)XV_OK;
  return XV_OK;
}
