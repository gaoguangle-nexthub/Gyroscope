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

/*
 * 24-bit rate output with a 2nd-order 35 Hz LPF. The measured effective
 * scale is 17920 LSB/(deg/s), equivalent to 70 LSB/(deg/s) in 16-bit form.
 */
#define XV7001BB_RATE_RAW_CODES_PER_DPS 17920
#define XV7001BB_RATE_DPS_PER_LSB       0.00005580357142857143f
#define XV7001BB_TEMP_C_PER_LSB         0.0625f
#define XV7001BB_ANGLE_DEG_PER_UDEG     0.000001f

/*
 * The sensor has a rated range of +/-100 deg/s. Accept every signed 24-bit
 * rate code, then clamp over-range readings to the rated full scale so that
 * angle integration continues at the maximum measurable rate.
 */
#define XV7001BB_RATE_RAW_LIMIT         8388608
#define XV7001BB_RATE_RAW_SATURATION    (XV7001BB_RATE_RAW_CODES_PER_DPS * 100)
#define XV7001BB_MAX_INTEGRATION_DT_US  50000U

/*
 * XV7001BB has no usable hardware zero-rate calibration command. Collect
 * 5 s of stationary samples at the application's 100 Hz cadence and require
 * at least 90% of them to pass the static and dispersion checks.
 */
#define XV7001BB_BIAS_SAMPLE_COUNT              500U
#define XV7001BB_BIAS_MIN_VALID_SAMPLES         450U
#define XV7001BB_BIAS_TRIM_DIVISOR              10U
#define XV7001BB_BIAS_SETTLE_MS                 1000U
/*
 * Raw-code equivalents of 7 deg/s per-sample and peak-to-peak limits,
 * 5 deg/s mean offset, and 1 deg/s RMS. The mean limit still covers
 * the specified temperature bias range, while the dispersion limits reject
 * slow motion that was previously wide enough to pass calibration.
 */
#define XV7001BB_BIAS_STATIC_RAW_LIMIT          125440
#define XV7001BB_BIAS_MAX_OFFSET_RAW            89600
#define XV7001BB_BIAS_MAX_PEAK_TO_PEAK_RAW      125440
#define XV7001BB_BIAS_MAX_VARIANCE_RAW2         321126400ULL

#define XV7001BB_BIAS_STATE_IDLE                0U
#define XV7001BB_BIAS_STATE_COLLECTING          1U
#define XV7001BB_BIAS_STATE_READY               2U
#define XV7001BB_BIAS_STATE_FAILED              3U

typedef enum
{
  XV_OK = 0,
  XV_ERR_PARAM,
  XV_ERR_IO,
  XV_ERR_TIMEOUT,
  XV_ERR_BUSY,
  XV_ERR_NOT_READY,
  XV_ERR_DATA_INVALID,
  XV_ERR_NOT_CALIBRATED,
  XV_ERR_CALIBRATION_FAILED
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
  uint32_t integration_count;
  uint32_t dt_gap_count;
  uint32_t rate_invalid_count;
  uint32_t rate_saturation_count;
  uint32_t sample_dt_min_us;
  uint32_t sample_dt_max_us;
  uint32_t bias_sample_count;
  uint32_t bias_valid_sample_count;
  uint32_t bias_rejected_count;
  uint32_t raw_temperature;
  uint64_t bias_variance_raw2;
  int32_t raw_rate;
  int32_t corrected_raw_rate;
  int32_t bias_raw;
  int32_t bias_peak_to_peak_raw;
  float rate_dps;
  float bias_dps;
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
  uint8_t bias_state;
  uint8_t bias_valid;
} xv7001bb_debug_t;

extern volatile xv7001bb_debug_t g_xv7001bb_debug;

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

/**
  * @brief Start a stationary zero-rate bias calibration.
  * @note The mechanism must remain stationary while 500 samples are collected.
  *       Calling this function clears the accumulated angle.
  */
xv_status_t XV7001bb_StartBiasCalibration(void);

/** @brief Return the calibrated bias in degrees per second. */
xv_status_t XV7001bb_GetBias(float *bias_dps);

/** @brief Read calibrated angular rate after the bias has been removed. */
xv_status_t XV7001bb_ReadRate(float *rate_dps);

/**
  * @brief Convert a raw 24-bit rate sample into an integrated angle in degrees.
  * @param raw_rate 24-bit two's-complement rate in LSB.
  * @param sample_time_us Monotonic 1 MHz-based timestamp in microseconds.
  * @param angle_deg Output angle wrapped to [-180, 180).
  * @note The calibrated zero-rate bias is subtracted before integration.
  *       Rates beyond the rated +/-100 deg/s range are clamped to full scale.
  *       No valid angle is produced before calibration succeeds.
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
