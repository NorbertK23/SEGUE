/*
 * drv_mpu6050.c
 *  Created on: 28 Feb 2026
 *      Author: Norbert Kania
 *
 * MPU-6050 register-level driver.
 * Fixed I2C address: 0x68 (AD0 tied to GND).
 */


#include "drv_mpu6050.h"
#include "util_delay.h"

twi_result_t mpu6050_write_reg_u8(uint8_t reg, uint8_t v)
{
    return twi_write_reg_u8(MPU6050_ADDR, reg, v);
}

twi_result_t mpu6050_read_reg_u8(uint8_t reg, uint8_t *v)
{
    return twi_read_reg_u8(MPU6050_ADDR, reg, v);
}

twi_result_t mpu6050_probe(void)
{
    return twi_probe(MPU6050_ADDR);
}

twi_result_t mpu6050_reset(void)
{
    twi_result_t r;

    r = mpu6050_write_reg_u8(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET);
    if (r.rc != TWI_OK)
    {
        return r;
    }

    /* Datasheet reset settling */
    delay_ms(100u);

    return r;
}

twi_result_t mpu6050_who_am_i(uint8_t *who)
{
    return mpu6050_read_reg_u8(MPU6050_REG_WHO_AM_I, who);
}

twi_result_t mpu6050_init(const mpu6050_cfg_t *cfg)
{
    twi_result_t r;
    uint8_t v;

    if (cfg == (const mpu6050_cfg_t *)0)
    {
        twi_result_t e = { TWI_ERR_PARAM, 0u };
        return e;
    }

    /* Wake device and select clock source */
    v = (uint8_t)cfg->clk & 0x07u;
    r = mpu6050_write_reg_u8(MPU6050_REG_PWR_MGMT_1, v);
    if (r.rc != TWI_OK)
    {
        return r;
    }

    delay_ms(5u);

    /* DLPF */
    v = (uint8_t)cfg->dlpf & 0x07u;
    r = mpu6050_write_reg_u8(MPU6050_REG_CONFIG, v);
    if (r.rc != TWI_OK)
    {
        return r;
    }

    /* Sample-rate divider */
    r = mpu6050_write_reg_u8(MPU6050_REG_SMPLRT_DIV, cfg->smplrt_div);
    if (r.rc != TWI_OK)
    {
        return r;
    }

    /* Gyro full-scale */
    v = (uint8_t)(((uint8_t)cfg->gyro_fs & 0x03u) << 3);
    r = mpu6050_write_reg_u8(MPU6050_REG_GYRO_CONFIG, v);
    if (r.rc != TWI_OK)
    {
        return r;
    }

    /* Accel full-scale */
    v = (uint8_t)(((uint8_t)cfg->accel_fs & 0x03u) << 3);
    r = mpu6050_write_reg_u8(MPU6050_REG_ACCEL_CONFIG, v);
    if (r.rc != TWI_OK)
    {
        return r;
    }

    return r;
}

twi_result_t mpu6050_read_raw14(mpu6050_raw14_t *out)
/* Big-endian register pairs */
{
    twi_result_t r;
    uint8_t b[14];

    if (out == (mpu6050_raw14_t *)0)
    {
        twi_result_t e = { TWI_ERR_PARAM, 0u };
        return e;
    }

    r = twi_read_reg(MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, b, 14u);
    if (r.rc != TWI_OK)
    {
        return r;
    }

    out->ax = (int16_t)( ( ( (uint16_t)b[0])  << 8) | (uint16_t)b[1]);
    out->ay = (int16_t)( ( ( (uint16_t)b[2])  << 8) | (uint16_t)b[3]);
    out->az = (int16_t)( ( ( (uint16_t)b[4])  << 8) | (uint16_t)b[5]);
    out->t  = (int16_t)( ( ( (uint16_t)b[6])  << 8) | (uint16_t)b[7]);
    out->gx = (int16_t)( ( ( (uint16_t)b[8])  << 8) | (uint16_t)b[9]);
    out->gy = (int16_t)( ( ( (uint16_t)b[10]) << 8) | (uint16_t)b[11]);
    out->gz = (int16_t)( ( ( (uint16_t)b[12]) << 8) | (uint16_t)b[13]);
    // combines the 2 bytes together into 16 bits
    return r;
}
