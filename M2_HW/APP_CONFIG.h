#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

#include "avr_stdint.h"
#include "drv_mpu6050.h"

/*
 * APP compile-time configuration.
 *
 * Units are carried in macro names:
 *   *_MS   ms
 *   *_MDEG mdeg
 *   *_MDPS mdps
 *   *_CPS  counts/s
 *   *_PM   promille motor command
 */

/* -------------------------------------------------------------------------- */
/* Build contract                                                             */
/* -------------------------------------------------------------------------- */

/* Control period is supplied by the build. */
#ifndef APP_CTRL_DT_MS
#error "APP_CTRL_DT_MS must be defined in project compiler settings"
#endif

#if (APP_CTRL_DT_MS == 0)
#error "APP_CTRL_DT_MS must be nonzero"
#endif

/* -------------------------------------------------------------------------- */
/* Compile-time options                                                       */
/* -------------------------------------------------------------------------- */

/* Compile-time motor disable for bench work. */
#ifndef EN_MOTORS
#define EN_MOTORS 1u
#endif

/* Compile-time telemetry gate. */
#ifndef APP_ENABLE_TELEMETRY
#define APP_ENABLE_TELEMETRY 0u
#endif

/* Telemetry period. Independent from APP_CTRL_DT_MS. */
#ifndef APP_TELEM_DT_MS
#define APP_TELEM_DT_MS 10u
#endif

/*
 * Selective brake policy.
 * Zero-command brake: small cmd, nontrivial wheel speed.
 * Reverse brake: small reverse cmd against a still-fast wheel.
 */
#ifndef APP_ENABLE_SELECTIVE_BRAKE
#define APP_ENABLE_SELECTIVE_BRAKE       0u
#endif

/*
 * APP_BRAKE_ZERO_CMD_PM
 * Higher: mode 1 triggers more often, because a wider range of small commands
 * counts as near zero.
 * Lower: mode 1 triggers less often, only when the command is very close to
 * zero.
 */
#define APP_BRAKE_ZERO_CMD_PM            60L
/*
 * APP_BRAKE_ZERO_VEL_CPS
 * Higher: mode 1 triggers less often, only at higher wheel speeds.
 * Lower: mode 1 triggers more often, even at modest wheel speeds.
 */
#define APP_BRAKE_ZERO_VEL_CPS           900L
/*
 * APP_BRAKE_REVERSE_CMD_PM
 * Higher: mode 2 triggers more often, because larger opposite-sign commands
 * still count as small reverse.
 * Lower: mode 2 triggers less often, only for very weak reverse commands.
 */
#define APP_BRAKE_REVERSE_CMD_PM         250L
/*
 * APP_BRAKE_REVERSE_VEL_CPS
 * Higher: mode 2 triggers less often, only when the wheel is still moving
 * quite fast.
 * Lower: mode 2 triggers more often, even when the wheel is not moving very
 * fast.
 */
#define APP_BRAKE_REVERSE_VEL_CPS        1800L

/*
 * Zero-cross active override.
 * delay_ms > 0: start after the detected crossing
 * delay_ms <= 0: start on the detected crossing
 * cmd_pm = 0 keeps the window as coast-only.
 */
#define APP_ZERO_CROSS_ACTIVE_DELAY_MS   0
#define APP_ZERO_CROSS_ACTIVE_MS         20u   /* prior trial: 25 ms */
#define APP_ZERO_CROSS_ACTIVE_MIN_VEL_CPS 5000L
#define APP_ZERO_CROSS_ACTIVE_CMD_PM     0L    /* prior trial: 1000 pm */

/* -------------------------------------------------------------------------- */
/* Application constants                                                      */
/* -------------------------------------------------------------------------- */

/* HAL UART0 TX ring size override. */
#define UART0_TX_BUFSIZE 256u

/* Core peripheral rates. */
#define BAUD                         250000UL
#define PWM_HZ                       1000u
/* Fast-mode I2C keeps the MPU burst read inside the control budget. */
#define TWI_HZ                       400000UL

/* Startup gyro bias calibration. Robot must stay still here. */
#define APP_GYRO_CAL_SAMPLES         200u
#define APP_GYRO_SETTLE_MS           1000u

/* Expected MPU6050 WHO_AM_I value. */
#define APP_MPU_WHO_EXPECTED         0x68u

/* Hard pitch fault limit. */
#define APP_PITCH_FAULT_MDEG         33000L

/* Arming window and dwell time. */
#define APP_ARM_PITCH_WINDOW_MDEG    5000L
#define APP_ARM_HOLD_MS              500u

/* Nominal lean target after APP pitch zeroing. */
#define APP_PITCH_SETPOINT_MDEG      (-2000)

/* D-term input clamp and D-term output clamp. */
#define RATE_CTRL_LIM_MDPS           200000L
#define APP_KD_TERM_LIM_PM           800L

/* Fixed-point gain divisor. gain_q / APP_GAIN_SCALE is the real gain. */
#define APP_GAIN_SCALE               10000L

/* Max catch-up control steps before faulting for overrun. */
#define APP_MAX_CATCHUP_CYCLES       3u

/* 1: fault on fresh invalid quadrature. 0: observe only. */
#define APP_FAULT_ON_ENCODER_INVALID 0u

/* Outer-loop and lean limits. */
#define APP_VEL_REF_LIM_CPS          4000L
#define APP_KV_TERM_LIM_MDEG         2500L

#define APP_ENC_LEAN_MIN_MDEG        (-10000L + APP_PITCH_SETPOINT_MDEG )
#define APP_ENC_LEAN_MAX_MDEG        (10000L + APP_PITCH_SETPOINT_MDEG )

/* -------------------------------------------------------------------------- */
/* Tunable gains                                                              */
/* -------------------------------------------------------------------------- */

/* Nominal control tuning, defined in SETTINGS.c. */
extern const int32_t g_app_pitch_offset_mdeg;
extern const int32_t g_app_kx_enc_q;
extern const int32_t g_app_kv_enc_q;
extern const int32_t g_app_kp_mpu_q;
extern const int32_t g_app_kd_mpu_q;

/* -------------------------------------------------------------------------- */
/* MPU configuration                                                          */
/* -------------------------------------------------------------------------- */

/* MPU6050 runtime configuration, defined in SETTINGS.c. */
extern const mpu6050_cfg_t g_app_mpu_cfg;

#endif /* APP_CONFIG_H_ */
