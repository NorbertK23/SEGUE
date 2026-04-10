/**
 * @file svc_mpu6050.h
 * @brief MPU6050 service-layer helpers for calibration and unit conversion.
 *
 * Responsibilities:
 * - gyro bias calibration
 * - raw bias application
 * - raw-to-physical-unit conversion
 *
 * The service layer depends on `drv_mpu6050` types and I/O functions, but it
 * contains no register-level policy.
 */


#ifndef SVC_MPU6050_H_
#define SVC_MPU6050_H_

#include "avr_stdint.h"
#include "drv_mpu6050.h"
#include "hal_twi.h"

/**
 * @brief Stored raw MPU6050 calibration biases.
 */
typedef struct
{
    int16_t gx_bias;
    int16_t gy_bias;
    int16_t gz_bias;

    int16_t ax_bias;
    int16_t ay_bias;
    int16_t az_bias;
} mpu6050_cal_t;

/**
 * @brief Scaled integer MPU6050 physical sample.
 *
 * Units:
 * - accel: mg
 * - gyro: mdps
 * - temp: cC
 */
typedef struct
{
    int32_t ax_mg;
    int32_t ay_mg;
    int32_t az_mg;

    int32_t gx_mdps;
    int32_t gy_mdps;
    int32_t gz_mdps;

    int32_t temp_cC;
} mpu6050_phys_t;

/**
 * @brief Zero all stored MPU6050 calibration biases.
 *
 * @param[out] cal Calibration structure to clear.
 */
void mpu6050_cal_clear(mpu6050_cal_t *cal);

/**
 * @brief Average `n_samples` raw gyro readings into stored gyro bias values.
 *
 * The caller must ensure the sensor is stationary and is responsible for any
 * pacing delay between samples.
 *
 * @param[out] cal Calibration structure to populate.
 * @param[in] n_samples Number of raw samples to average.
 * @return TWI transaction result.
 */
twi_result_t mpu6050_calibrate_gyro_bias(mpu6050_cal_t *cal, uint16_t n_samples);

/**
 * @brief Apply stored raw biases in place.
 *
 * @param[in,out] raw Raw sample to adjust.
 * @param[in] cal Calibration values to apply.
 */
void mpu6050_apply_cal_raw(mpu6050_raw14_t *raw, const mpu6050_cal_t *cal);

/**
 * @brief Convert a raw sample to scaled physical units.
 *
 * @param[in] raw Raw MPU6050 sample.
 * @param[out] out Converted physical-unit sample.
 * @param[in] cfg MPU6050 full-scale configuration used for the conversion.
 */
void mpu6050_raw_to_phys(const mpu6050_raw14_t *raw,
                         mpu6050_phys_t *out,
                         const mpu6050_cfg_t *cfg);

#endif /* SVC_MPU6050_H_ */
