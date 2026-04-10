/**
 * @file drv_mpu6050.h
 * @brief Register-level MPU6050 driver for the board-fixed device instance.
 *
 * Fixed I2C address: `0x68` with AD0 tied low.
 *
 * Layering:
 * - uses `hal_twi`
 * - does not perform calibration, unit conversion, filtering, or estimation
 */

#ifndef DRV_MPU6050_H_
#define DRV_MPU6050_H_

#include "avr_stdint.h"
#include "hal_twi.h"

/** @brief Fixed 7-bit MPU6050 board address. */
#define MPU6050_ADDR                (0x68u)

/* ---- Registers ---- */
#define MPU6050_REG_SMPLRT_DIV      (0x19u)
#define MPU6050_REG_CONFIG          (0x1Au)
#define MPU6050_REG_GYRO_CONFIG     (0x1Bu)
#define MPU6050_REG_ACCEL_CONFIG    (0x1Cu)

#define MPU6050_REG_INT_ENABLE      (0x38u)
#define MPU6050_REG_INT_STATUS      (0x3Au)

#define MPU6050_REG_ACCEL_XOUT_H    (0x3Bu)  /* start of 14-byte burst */

#define MPU6050_REG_PWR_MGMT_1      (0x6Bu)
#define MPU6050_REG_WHO_AM_I        (0x75u)

/* ---- Bit fields ---- */
#define MPU6050_PWR1_DEVICE_RESET   (1u << 7)
#define MPU6050_PWR1_SLEEP          (1u << 6)

/* ---- Types ---- */

typedef enum
{
    MPU6050_CLK_INTERNAL  = 0u,
    MPU6050_CLK_PLL_XGYRO = 1u,
    MPU6050_CLK_PLL_YGYRO = 2u,
    MPU6050_CLK_PLL_ZGYRO = 3u
} mpu6050_clk_t;

/* CONFIG[2:0] */
typedef enum
{
    MPU6050_DLPF_260HZ = 0u,
    MPU6050_DLPF_184HZ = 1u,
    MPU6050_DLPF_94HZ  = 2u,
    MPU6050_DLPF_44HZ  = 3u,
    MPU6050_DLPF_21HZ  = 4u,
    MPU6050_DLPF_10HZ  = 5u,
    MPU6050_DLPF_5HZ   = 6u
} mpu6050_dlpf_t;

/* GYRO_CONFIG[4:3] */
typedef enum
{
    MPU6050_GFS_250DPS  = 0u,
    MPU6050_GFS_500DPS  = 1u,
    MPU6050_GFS_1000DPS = 2u,
    MPU6050_GFS_2000DPS = 3u
} mpu6050_gyro_fs_t;

/* ACCEL_CONFIG[4:3] */
typedef enum
{
    MPU6050_AFS_2G  = 0u,
    MPU6050_AFS_4G  = 1u,
    MPU6050_AFS_8G  = 2u,
    MPU6050_AFS_16G = 3u
} mpu6050_accel_fs_t;

typedef struct
{
    mpu6050_clk_t      clk;
    mpu6050_dlpf_t     dlpf;
    uint8_t            smplrt_div; /* Rate = 1kHz / (1 + div) when DLPF enabled */
    mpu6050_gyro_fs_t  gyro_fs;
    mpu6050_accel_fs_t accel_fs;
} mpu6050_cfg_t;

/**
 * @brief Raw 14-byte MPU6050 sample payload decoded into signed axes.
 */
typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t t;

    int16_t gx;
    int16_t gy;
    int16_t gz;
} mpu6050_raw14_t;

/**
 * @brief Write one 8-bit MPU6050 register.
 *
 * @param[in] reg Register address.
 * @param[in] v Value to write.
 * @return TWI transaction result.
 */
twi_result_t mpu6050_write_reg_u8(uint8_t reg, uint8_t v);

/**
 * @brief Read one 8-bit MPU6050 register.
 *
 * @param[in] reg Register address.
 * @param[out] v Destination for the register value.
 * @return TWI transaction result.
 */
twi_result_t mpu6050_read_reg_u8(uint8_t reg, uint8_t *v);

/** @brief Reset the MPU6050 through `PWR_MGMT_1`. */
twi_result_t mpu6050_reset(void);

/**
 * @brief Read the MPU6050 `WHO_AM_I` register.
 *
 * @param[out] who Destination for the returned ID value.
 * @return TWI transaction result.
 */
twi_result_t mpu6050_who_am_i(uint8_t *who);

/** @brief Probe whether the fixed MPU6050 address acknowledges on the bus. */
twi_result_t mpu6050_probe(void);

/**
 * @brief Apply the runtime configuration used by the application.
 *
 * @param[in] cfg Register-level MPU6050 configuration.
 * @return TWI transaction result.
 */
twi_result_t mpu6050_init(const mpu6050_cfg_t *cfg);

/**
 * @brief Read one raw 14-byte sensor sample burst.
 *
 * @param[out] out Destination for the decoded raw sample.
 * @return TWI transaction result.
 */
twi_result_t mpu6050_read_raw14(mpu6050_raw14_t *out);

#endif /* DRV_MPU6050_H_ */
