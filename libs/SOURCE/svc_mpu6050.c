/*
 * svc_mpu6050.c
 *
 *  Created on: 13 Mar 2026
 *      Author: Norbert Kania
  * MPU-6050 service layer:
 *   calibration + unit conversion
 */


#include "svc_mpu6050.h"

static int32_t accel_lsb_per_g(const mpu6050_cfg_t *cfg)
{
    switch (cfg->accel_fs)
    {
    case MPU6050_AFS_2G:  return 16384;
    case MPU6050_AFS_4G:  return 8192;
    case MPU6050_AFS_8G:  return 4096;
    case MPU6050_AFS_16G: return 2048;
    default:              return 16384;
    }
}

static int32_t gyro_lsb_per_dps(const mpu6050_cfg_t *cfg)
{
    /* Integer approximations */
    switch (cfg->gyro_fs)
    {
    case MPU6050_GFS_250DPS:  return 131;
    case MPU6050_GFS_500DPS:  return 66;   /* true is 65.5 */
    case MPU6050_GFS_1000DPS: return 33;   /* true is 32.8 */
    case MPU6050_GFS_2000DPS: return 16;   /* true is 16.4 */
    default:                  return 131;
    }
}

void mpu6050_cal_clear(mpu6050_cal_t *cal)
{
    if (cal == (mpu6050_cal_t *)0)
    {
        return;
    }

    cal->gx_bias = 0;
    cal->gy_bias = 0;
    cal->gz_bias = 0;

    cal->ax_bias = 0;
    cal->ay_bias = 0;
    cal->az_bias = 0;
}

twi_result_t mpu6050_calibrate_gyro_bias(mpu6050_cal_t *cal, uint16_t n_samples)
{
    twi_result_t r;
    mpu6050_raw14_t s;
    int32_t sx;
    int32_t sy;
    int32_t sz;
    uint16_t i;

    if ((cal == (mpu6050_cal_t *)0) || (n_samples == 0u))
    {
        twi_result_t e = { TWI_ERR_PARAM, 0u };
        return e;
    }

    sx = 0;
    sy = 0;
    sz = 0;

    for (i = 0u; i < n_samples; i++)
    {
        r = mpu6050_read_raw14(&s);
        if (r.rc != TWI_OK)
        {
            return r;
        }

        sx += (int32_t)s.gx;
        sy += (int32_t)s.gy;
        sz += (int32_t)s.gz;
    }

    cal->gx_bias = (int16_t)(sx / (int32_t)n_samples);
    cal->gy_bias = (int16_t)(sy / (int32_t)n_samples);
    cal->gz_bias = (int16_t)(sz / (int32_t)n_samples);

    return r;
}

void mpu6050_apply_cal_raw(mpu6050_raw14_t *raw, const mpu6050_cal_t *cal)
{
    if ((raw == (mpu6050_raw14_t *)0) || (cal == (const mpu6050_cal_t *)0))
    {
        return;
    }

    raw->gx = (int16_t)(raw->gx - cal->gx_bias);
    raw->gy = (int16_t)(raw->gy - cal->gy_bias);
    raw->gz = (int16_t)(raw->gz - cal->gz_bias);

    raw->ax = (int16_t)(raw->ax - cal->ax_bias);
    raw->ay = (int16_t)(raw->ay - cal->ay_bias);
    raw->az = (int16_t)(raw->az - cal->az_bias);
}

void mpu6050_raw_to_phys(const mpu6050_raw14_t *raw,
                         mpu6050_phys_t *out,
                         const mpu6050_cfg_t *cfg)
{
    int32_t a_lsb_g;
    int32_t g_lsb_dps;

    if ((raw == (const mpu6050_raw14_t *)0) ||
        (out == (mpu6050_phys_t *)0) ||
        (cfg == (const mpu6050_cfg_t *)0))
    {
        return;
    }

    a_lsb_g = accel_lsb_per_g(cfg);
    g_lsb_dps = gyro_lsb_per_dps(cfg);

    /* accel: raw / (LSB/g) * 1000 = mg */
    out->ax_mg = ((int32_t)raw->ax * 1000) / a_lsb_g;
    out->ay_mg = ((int32_t)raw->ay * 1000) / a_lsb_g;
    out->az_mg = ((int32_t)raw->az * 1000) / a_lsb_g;

    /* gyro: raw / (LSB/dps) * 1000 = mdps */
    out->gx_mdps = ((int32_t)raw->gx * 1000) / g_lsb_dps;
    out->gy_mdps = ((int32_t)raw->gy * 1000) / g_lsb_dps;
    out->gz_mdps = ((int32_t)raw->gz * 1000) / g_lsb_dps;

    /* temp: raw/340 + 36.53 degC
     * in centi-degC:
     *   temp_cC = raw*100/340 + 3653
     */
    out->temp_cC = (((int32_t)raw->t * 100) / 340) + 3653;
}
