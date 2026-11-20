/**
  ******************************************************************************
  * @file    xv7001bb.h
  * @brief   XV7001BB four-wire SPI device interface and diagnostics.
  ******************************************************************************
  */

#ifndef XV7001BB_H
#define XV7001BB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* User registers defined by XV7001BB_AE_Ver. 1.01. */
#define XV7001BB_REG_DSP_CTL1          0x01U
#define XV7001BB_REG_DSP_CTL2          0x02U
#define XV7001BB_REG_DSP_CTL3          0x03U
#define XV7001BB_REG_STATUS            0x04U
#define XV7001BB_REG_SLEEP_IN          0x05U
#define XV7001BB_REG_SLEEP_OUT         0x06U
#define XV7001BB_REG_STANDBY           0x07U
#define XV7001BB_REG_TEMPERATURE       0x08U
#define XV7001BB_REG_SOFTWARE_RESET    0x09U
#define XV7001BB_REG_RATE              0x0AU
#define XV7001BB_REG_OUTPUT_CTL        0x0BU
#define XV7001BB_REG_TEMP_FORMAT       0x1CU
#define XV7001BB_REG_INTERFACE_CTL     0x1FU

#define XV7001BB_STATUS_PROCESS_OK     0x08U
#define XV7001BB_STATUS_STATE_MASK     0x07U
#define XV7001BB_STATUS_POR            0x04U
#define XV7001BB_STATUS_STANDBY        0x02U
#define XV7001BB_STATUS_SLEEP          0x00U
#define XV7001BB_STATUS_SLEEP_OUT      0x01U

/* The board powers VDDM and VDDI together. Wait at least 200 ms before Init. */
#define XV7001BB_POWER_UP_WAIT_MS      200U

/* First-version configuration: 16-bit rate output and a 2nd-order 35 Hz LPF. */
#define XV7001BB_RATE_LSB_PER_DPS       280.0f
#define XV7001BB_RATE_DPS_PER_LSB       0.003571428571428571f
#define XV7001BB_TEMP_C_PER_LSB         0.0625f
#define XV7001BB_ANGLE_DEG_PER_UDEG     0.000001f

/* Reject abnormal samples and integration gaps while permitting scheduler jitter. */
#define XV7001BB_RATE_RAW_LIMIT         30000
#define XV7001BB_MAX_INTEGRATION_DT_US  50000U

typedef enum
{
  XV_OK = 0,
  XV_ERR_PARAM,
  XV_ERR_IO,
  XV_ERR_TIMEOUT,
  XV_ERR_BUSY,
  XV_ERR_NOT_READY,
  XV_ERR_DATA_INVALID
} xv_status_t;

typedef struct
{
  uint32_t init_calls;
  uint32_t init_status;
  uint32_t last_spi_status;
  uint32_t spi_error_count;
  uint32_t invalid_request_count;
  uint32_t sample_count;
  uint32_t sample_time_us;
  uint32_t sample_dt_us;
  uint32_t raw_temperature;
  int32_t raw_rate;
  float rate_dps;
  float temperature_c;
  float angle_deg;
  uint8_t status_raw;
  uint8_t status_proc_ok;
  uint8_t status_power_state;
  uint8_t reg_control_default;
  uint8_t reg_control_value;
  uint8_t reg_output_default;
  uint8_t reg_output_value;
  uint8_t reg_filter_default;
  uint8_t reg_filter_value;
  uint8_t initialized;
  uint8_t angle_time_valid;
} xv7001bb_debug_t;

typedef struct
{
  uint32_t init_status;
  uint32_t write_test_status;
  uint32_t read_status_status;
  uint32_t read_reg_control_status;
  uint32_t read_reg_output_status;
  uint32_t read_reg_filter_status;
  uint32_t read_tmp_status;
  uint32_t read_raw_rate_status;
  uint32_t read_rate_status;
  uint32_t read_angle_status;
  uint32_t interface_check_complete;
  uint32_t loop_count;
  uint32_t period_ticks;
  uint32_t period_ticks_max;
  uint32_t error_count;
  uint8_t status_raw;
  uint8_t interface_ctl;
  uint8_t output_ctl;
  uint8_t filter_ctl;
  int32_t raw_rate;
  float temperature_c;
  float rate_dps;
  float angle_deg;
} xv7001bb_task_debug_t;

extern volatile xv7001bb_debug_t g_xv7001bb_debug;
extern volatile xv7001bb_task_debug_t g_xv7001bb_task_debug;

/**
  * @brief Initialize the sensor after the caller has supplied the power-up delay.
  * @note Call XV7001bb_Init() only after waiting XV7001bb_POWER_UP_WAIT_MS.
  */
xv_status_t XV7001bb_Init(void);

xv_status_t XV7001bb_WriteData(uint8_t reg, uint8_t value);
xv_status_t XV7001bb_ReadReg(uint8_t reg, uint8_t *value);
xv_status_t XV7001bb_ReadStatus(uint8_t *status);
xv_status_t XV7001bb_SendCommand(uint8_t command);
xv_status_t XV7001bb_ReadTmp(float *temp_c);
xv_status_t XV7001bb_ReadRawRate(int32_t *raw_rate);
xv_status_t XV7001bb_ReadRate(float *rate_dps);

/**
  * @brief Convert a raw 16-bit rate sample into an integrated angle in degrees.
  * @param raw_rate 16-bit two's-complement rate in LSB.
  * @param sample_time_us Monotonic 1 MHz-based timestamp in microseconds.
  * @param angle_deg Output angle wrapped to [-180, 180).
  */
xv_status_t XV7001bb_ConvertRateToAngle(int32_t raw_rate,
                                        uint32_t sample_time_us,
                                        float *angle_deg);

/**
  * @brief Read the current rate and integrate it into XV7001bb_ConvertRateToAngle.
  */
xv_status_t XV7001bb_ReadAngle(float *angle_deg);

#ifdef __cplusplus
}
#endif

#endif /* XV7001BB_H */
