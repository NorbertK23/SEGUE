#include <avr/io.h>
#include <avr/interrupt.h>

#include "avr_stdint.h"
#include "util_term.h"
#include "util_delay.h"

#include "hal_time2.h"
#include "hal_twi.h"

#include "drv_tb6612fng.h"
#include "drv_mpu6050.h"

#include "svc_enc.h"
#include "svc_mpu6050.h"

#include "SETTINGS.h"
#include "TELEMETRY.h"
#include "CONTROL.h"

/*
 * APP control core.
 * Owns startup, estimation, arming, nominal control, output policy, and the
 * fixed-rate foreground loop.
 */

/* -------------------------------------------------------------------------- */
/* Local structural-control settings                                          */
/* -------------------------------------------------------------------------- */

/*
 * Local APP policy switches.
 * These shape CONTROL.c behavior but are not part of the wider APP contract.
 */

#ifndef APP_CTRL_VEL_FILT_DEN
#define APP_CTRL_VEL_FILT_DEN      2L
#endif

#ifndef APP_CTRL_VEL_FILT_OLD_NUM
#define APP_CTRL_VEL_FILT_OLD_NUM  1L
#endif

#ifndef APP_CTRL_VEL_FILT_NEW_NUM
#define APP_CTRL_VEL_FILT_NEW_NUM  1L
#endif

#ifndef APP_ARM_TRIM_RAMP_MS
#define APP_ARM_TRIM_RAMP_MS 200u
#endif

/* -------------------------------------------------------------------------- */
/* Small utility helpers                                                      */
/* -------------------------------------------------------------------------- */

/* Small signed clamp helper. */
static int32_t app_clamp_i32(int32_t x, int32_t lo, int32_t hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

/* Signed absolute value. */
static int32_t app_iabs32(int32_t x)
{
    return (x < 0) ? -x : x;
}

/* Unsigned magnitude helper for LUT and ratio work. */
static uint32_t app_uabs32(int32_t x)
{
    if (x < 0) {
        return (uint32_t)(-x);
    }
    return (uint32_t)x;
}

/* Return -1, 0, or +1. */
static int32_t app_sign_i32(int32_t x)
{
    if (x > 0) {
        return 1;
    }
    if (x < 0) {
        return -1;
    }
    return 0;
}

/* Saturating add used by the exact 32-bit mul/div helper. */
static uint32_t app_u32_sat_add(uint32_t a, uint32_t b)
{
    if ((uint32_t)(0xFFFFFFFFUL - a) < b) {
        return 0xFFFFFFFFUL;
    }

    return a + b;
}

/*
 * Exact unsigned (a * b) / div without a 64-bit intermediate.
 * Keeps the AVR build off the libgcc 64-bit helpers.
 */
static uint32_t app_u32_muldiv_u32(uint32_t a, uint32_t b, uint32_t div)
{
    uint32_t q;
    uint32_t rem;
    uint32_t term_q;
    uint32_t term_r;

    if (div == 0u) {
        return 0xFFFFFFFFUL;
    }

    q = 0u;
    rem = 0u;

    term_q = b / div;
    term_r = b % div;

    while (a != 0u) {
        if ((a & 1u) != 0u) {
            q = app_u32_sat_add(q, term_q);

            if (term_r != 0u) {
                if (rem >= (div - term_r)) {
                    rem = rem - (div - term_r);
                    q = app_u32_sat_add(q, 1u);
                } else {
                    rem += term_r;
                }
            }
        }

        a >>= 1u;
        if (a == 0u) {
            break;
        }

        term_q = app_u32_sat_add(term_q, term_q);

        if (term_r != 0u) {
            if (term_r >= (div - term_r)) {
                term_r = term_r - (div - term_r);
                term_q = app_u32_sat_add(term_q, 1u);
            } else {
                term_r += term_r;
            }
        }
    }

    return q;
}

/* Signed wrapper for the exact unsigned mul/div helper. */
static int32_t app_i32_muldiv_u32(int32_t x, uint32_t mul, uint32_t div)
{
    uint32_t mag;
    uint32_t out_u;

    mag = app_uabs32(x);
    out_u = app_u32_muldiv_u32(mag, mul, div);

    if (x < 0) {
        if (out_u >= 2147483648UL) {
            return (-2147483647L - 1L);
        }
        return -(int32_t)out_u;
    }

    if (out_u > 2147483647UL) {
        return 2147483647L;
    }

    return (int32_t)out_u;
}

/* First-order integer IIR helper. */
static int32_t app_filter_iir(int32_t y_prev,
                              int32_t x_now,
                              int32_t old_num,
                              int32_t new_num,
                              int32_t den)
{
    if (den <= 0L) {
        return x_now;
    }

    return ((old_num * y_prev) + (new_num * x_now)) / den;
}

/* Convert ms to control cycles, rounding up. */
static uint8_t app_ms_to_cycles(uint16_t ms)
{
    uint16_t cycles;

    if (ms == 0u) {
        return 0u;
    }

    cycles = (uint16_t)((ms + APP_CTRL_DT_MS - 1u) / APP_CTRL_DT_MS);

    if (cycles == 0u) {
        cycles = 1u;
    }
    if (cycles > 255u) {
        cycles = 255u;
    }

    return (uint8_t)cycles;
}

static uint8_t app_zero_cross_active_delay_cycles(void)
{
    int32_t delay_ms;

    delay_ms = (int32_t)APP_ZERO_CROSS_ACTIVE_DELAY_MS;
    if (delay_ms < 0L) {
        delay_ms = 0L;
    }

    return app_ms_to_cycles((uint16_t)delay_ms);
}

static uint8_t app_zero_cross_active_cycles(void)
{
    return app_ms_to_cycles(APP_ZERO_CROSS_ACTIVE_MS);
}

static uint8_t app_zero_cross_active_start(app_state_t *s,
                                           uint8_t delay_cycles,
                                           uint8_t active_cycles,
                                           int32_t vel_sign)
{
    if ((active_cycles == 0u) || (vel_sign == 0L)) {
        return 0u;
    }

    s->zero_cross_active_delay_cycles = delay_cycles;
    s->zero_cross_active_cycles = active_cycles;
    s->zero_cross_active_cmd_pm =
        (int16_t)(-vel_sign * APP_ZERO_CROSS_ACTIVE_CMD_PM);

    if (delay_cycles == 0u) {
        s->zero_cross_active_cycles--;
        return 1u;
    }

    return 0u;
}

/* -------------------------------------------------------------------------- */
/* Trim-ramp helpers                                                          */
/* -------------------------------------------------------------------------- */

/* Trim ramp softens the lean-step at arm time. */
static void app_trim_ramp_reset(app_state_t *s)
{
    s->trim_ramp_active = 0u;
    s->trim_ramp_cycles_left = 0u;
    s->trim_ramp_start_mdeg = 0;
    s->trim_ramp_target_mdeg = 0;
}

/* Start a fresh trim ramp from current pitch to steady-state trim. */
static void app_start_trim_ramp(app_state_t *s, int32_t pitch_est_mdeg)
{
    uint16_t total_cycles;

    total_cycles = (uint16_t)(APP_ARM_TRIM_RAMP_MS / APP_CTRL_DT_MS);
    if (total_cycles == 0u) {
        app_trim_ramp_reset(s);
        return;
    }

    s->trim_ramp_active = 1u;
    s->trim_ramp_cycles_left = total_cycles;
    s->trim_ramp_start_mdeg = pitch_est_mdeg;
    s->trim_ramp_target_mdeg = APP_PITCH_SETPOINT_MDEG;
}

/* Base lean is either the steady trim or the active arm-time ramp. */
static int32_t app_current_base_lean_mdeg(app_state_t *s)
{
    int32_t out;
    uint16_t total_cycles;
    int32_t step_num;
    int32_t step_den;

    if (s->trim_ramp_active == 0u) {
        return APP_PITCH_SETPOINT_MDEG;
    }

    total_cycles = (uint16_t)(APP_ARM_TRIM_RAMP_MS / APP_CTRL_DT_MS);
    if (total_cycles == 0u) {
        app_trim_ramp_reset(s);
        return APP_PITCH_SETPOINT_MDEG;
    }

    if (s->trim_ramp_cycles_left == 0u) {
        app_trim_ramp_reset(s);
        return APP_PITCH_SETPOINT_MDEG;
    }

    step_den = (int32_t)total_cycles;
    step_num = (int32_t)(total_cycles - s->trim_ramp_cycles_left);

    out = s->trim_ramp_start_mdeg +
          ((s->trim_ramp_target_mdeg - s->trim_ramp_start_mdeg) * step_num) / step_den;

    s->trim_ramp_cycles_left--;

    if (s->trim_ramp_cycles_left == 0u) {
        app_trim_ramp_reset(s);
        out = APP_PITCH_SETPOINT_MDEG;
    }

    return out;
}

/* -------------------------------------------------------------------------- */
/* Shared runtime reset                                                       */
/* -------------------------------------------------------------------------- */

/* Reset runtime-only state. Sensor calibration and latest samples stay valid. */
static void app_reset_runtime_state(app_state_t *s)
{
    s->enc_vel_filt = 0;
    app_trim_ramp_reset(s);

    s->motor_cmd_nom_raw   = 0;
    s->motor_cmd           = 0;
    s->enc_invalid_a_now   = 0u;
    s->enc_invalid_b_now   = 0u;

    s->imu_read_us_last    = 0u;
    s->ctrl_dt_us_last     = 0u;
    s->ctrl_exec_us_last   = 0u;
    s->sched_late_ms_last  = 0u;
    s->brake_mode                 = 0u;
    s->zero_cross_active_delay_cycles = 0u;
    s->zero_cross_active_cycles   = 0u;
    s->zero_cross_active_cmd_pm   = 0;
    s->pitch_est_prev_mdeg        = 0;

    s->motor_armed         = 0u;
    s->arm_window_ms       = 0u;
}

/* -------------------------------------------------------------------------- */
/* atan LUT for accelerometer pitch                                           */
/* -------------------------------------------------------------------------- */

/* Quarter-plane atan LUT in mdeg for accel pitch. */
static const int16_t g_atan_lut_mdeg[129] =
{
        0,   448,   895,  1343,  1790,  2237,  2684,  3130,
     3576,  4022,  4467,  4912,  5356,  5799,  6242,  6684,
     7125,  7565,  8005,  8443,  8881,  9317,  9752, 10187,
    10620, 11051, 11482, 11911, 12339, 12766, 13191, 13614,
    14036, 14457, 14876, 15293, 15709, 16123, 16535, 16945,
    17354, 17761, 18166, 18569, 18970, 19370, 19767, 20163,
    20556, 20947, 21337, 21724, 22109, 22493, 22874, 23253,
    23629, 24004, 24376, 24747, 25115, 25481, 25844, 26206,
    26565, 26922, 27277, 27629, 27979, 28327, 28673, 29017,
    29358, 29697, 30033, 30368, 30700, 31030, 31357, 31682,
    32005, 32326, 32645, 32961, 33275, 33587, 33896, 34203,
    34509, 34811, 35112, 35410, 35707, 36001, 36293, 36582,
    36870, 37155, 37439, 37720, 37999, 38276, 38550, 38823,
    39094, 39362, 39629, 39894, 40156, 40416, 40675, 40931,
    41186, 41438, 41689, 41938, 42184, 42429, 42672, 42913,
    43152, 43390, 43625, 43859, 44091, 44321, 44549, 44775,
    45000
};

/* Approximate atan(num / den) over [0, 1] with Q10 interpolation. */
static int32_t app_atan_ratio_lut_mdeg(uint32_t num, uint32_t den)
{
    uint32_t ratio_q10;
    uint16_t idx;
    uint16_t frac;
    int32_t  a0;
    int32_t  a1;

    if (den == 0u) {
        return 45000L;
    }

    ratio_q10 = (num << 10) / den;

    if (ratio_q10 >= 1024u) {
        return 45000L;
    }

    idx  = (uint16_t)(ratio_q10 >> 3);
    frac = (uint16_t)(ratio_q10 & 0x07u);

    a0 = (int32_t)g_atan_lut_mdeg[idx];
    a1 = (int32_t)g_atan_lut_mdeg[idx + 1u];

    return a0 + (((a1 - a0) * (int32_t)frac) >> 3);
}

/* Full atan2 approximation in mdeg using the LUT helper above. */
static int32_t app_atan2_lut_mdeg(int32_t y, int32_t x)
{
    uint32_t ax;
    uint32_t ay;
    int32_t  angle_q1;

    if (x == 0) {
        if (y > 0) {
            return 90000L;
        }
        if (y < 0) {
            return -90000L;
        }
        return 0L;
    }

    ax = app_uabs32(x);
    ay = app_uabs32(y);

    if (ax >= ay) {
        angle_q1 = app_atan_ratio_lut_mdeg(ay, ax);
    } else {
        angle_q1 = 90000L - app_atan_ratio_lut_mdeg(ax, ay);
    }

    if (x > 0) {
        if (y >= 0) {
            return angle_q1;
        }
        return -angle_q1;
    }

    if (y >= 0) {
        return 180000L - angle_q1;
    }

    return angle_q1 - 180000L;
}

/* Board sign convention. Do not flip without re-validating the full stack. */
static int32_t app_accel_pitch_mdeg(int32_t ax_mg, int32_t az_mg)
{
    return app_atan2_lut_mdeg(-ax_mg, az_mg);
}

/* Subtract the calibrated frame offset so balanced posture is 0 mdeg. */
static int32_t app_accel_pitch_comp_mdeg(int32_t ax_mg, int32_t az_mg)
{
    return app_accel_pitch_mdeg(ax_mg, az_mg) - g_app_pitch_offset_mdeg;
}

/* -------------------------------------------------------------------------- */
/* Output / fault handling                                                    */
/* -------------------------------------------------------------------------- */

/* Disable drive and clear runtime control state. */
void app_drive_off(app_state_t *s)
{
    drv_tb6612_set_both_cmd_promille(0, 0);
    drv_tb6612_disable();
    app_reset_runtime_state(s);
}

/* Fatal fault path. Drive off, print reason, stop forever. */
void app_enter_fault(app_state_t *s, const char *msg)
{
    app_drive_off(s);

    term_puts("FAULT: ");
    term_puts(msg);
    term_newline();

    for (;;) {
        ;
    }
}

/* -------------------------------------------------------------------------- */
/* MPU bring-up and sampling                                                  */
/* -------------------------------------------------------------------------- */

/* APP-owned MPU bring-up sequence. */
static uint8_t app_mpu_init(app_state_t *s)
{
    twi_result_t r;
    uint8_t who;

    (void)s;

    r = mpu6050_probe();
    if (r.rc != TWI_OK) {
        term_puts("MPU probe failed");
        term_newline();
        return 0u;
    }

    r = mpu6050_who_am_i(&who);
    if (r.rc != TWI_OK) {
        term_puts("MPU WHO_AM_I read failed");
        term_newline();
        return 0u;
    }

    term_puts("MPU WHO=");
    term_put_hex_u8(who);
    term_newline();

    if (who != APP_MPU_WHO_EXPECTED) {
        term_puts("MPU WHO_AM_I mismatch");
        term_newline();
        return 0u;
    }

    r = mpu6050_reset();
    if (r.rc != TWI_OK) {
        term_puts("MPU reset failed");
        term_newline();
        return 0u;
    }

    r = mpu6050_init(&g_app_mpu_cfg);
    if (r.rc != TWI_OK) {
        term_puts("MPU init failed");
        term_newline();
        return 0u;
    }

    mpu6050_cal_clear(&s->mpu_cal);

    term_puts("MPU init OK");
    term_newline();

    return 1u;
}

/* Startup gyro calibration. Robot must stay still. */
static uint8_t app_calibrate_gyro(app_state_t *s)
{
    twi_result_t r;

    term_puts("Keep robot still...");
    term_newline();

    delay_ms(APP_GYRO_SETTLE_MS);
    mpu6050_cal_clear(&s->mpu_cal);

    r = mpu6050_calibrate_gyro_bias(&s->mpu_cal, APP_GYRO_CAL_SAMPLES);
    if (r.rc != TWI_OK) {
        term_puts("Gyro calibration failed");
        term_newline();
        return 0u;
    }

    term_puts("Gyro calibration done");
    term_newline();

    term_puts("gx_bias=");
    term_put_i32((int32_t)s->mpu_cal.gx_bias);
    term_puts(" gy_bias=");
    term_put_i32((int32_t)s->mpu_cal.gy_bias);
    term_puts(" gz_bias=");
    term_put_i32((int32_t)s->mpu_cal.gz_bias);
    term_newline();

    return 1u;
}

/* Read one MPU sample and convert it to physical units. */
static uint8_t app_read_mpu_sample(app_state_t *s)
{
    twi_result_t r;

    r = mpu6050_read_raw14(&s->raw);
    if (r.rc != TWI_OK) {
        return 0u;
    }

    mpu6050_apply_cal_raw(&s->raw, &s->mpu_cal);
    mpu6050_raw_to_phys(&s->raw, &s->phys, &g_app_mpu_cfg);

    return 1u;
}

/* -------------------------------------------------------------------------- */
/* Pitch estimator                                                            */
/* -------------------------------------------------------------------------- */

/* Seed the estimator from accel pitch. */
static void app_init_pitch_estimator(app_state_t *s)
{
    int32_t pitch_acc_mdeg;

    pitch_acc_mdeg     = app_accel_pitch_comp_mdeg(s->phys.ax_mg, s->phys.az_mg);
    s->pitch_est_mdeg  = pitch_acc_mdeg;
    s->pitch_rate_mdps = s->phys.gy_mdps;
}

/* Complementary pitch estimator: gyro short-term, accel long-term. */
static void app_update_pitch_estimator(app_state_t *s, uint32_t dt_us)
{
    int32_t pitch_acc_mdeg;
    int32_t pitch_step_mdeg;

    s->pitch_rate_mdps = s->phys.gy_mdps;
    pitch_acc_mdeg     = app_accel_pitch_comp_mdeg(s->phys.ax_mg, s->phys.az_mg);

    if (dt_us == 0u) {
        dt_us = (uint32_t)APP_CTRL_DT_MS * 1000u;
    }

    pitch_step_mdeg = app_i32_muldiv_u32(s->pitch_rate_mdps, dt_us, 1000000u);

    s->pitch_est_mdeg =
        (69L * (s->pitch_est_mdeg + pitch_step_mdeg) +
         1L  * pitch_acc_mdeg) / 70L;
}

/* -------------------------------------------------------------------------- */
/* Encoder state / diagnostics                                                */
/* -------------------------------------------------------------------------- */

/* Latch the latest encoder state and invalid-transition deltas. */
static void app_update_encoder_state(app_state_t *s, uint32_t dt_us)
{
    uint32_t invalid_a_now;
    uint32_t invalid_b_now;
    int32_t delta_a;
    int32_t delta_b;
    int32_t enc_vel_raw;
    int32_t vel_a_cps;
    int32_t vel_b_cps;

    s->enc_count = (svc_enc_get_count_a() + svc_enc_get_count_b()) / 2L;

    delta_a = svc_enc_get_delta_a();
    delta_b = svc_enc_get_delta_b();

    if (dt_us == 0u) {
        vel_a_cps = svc_enc_get_vel_a_cps();
        vel_b_cps = svc_enc_get_vel_b_cps();
    } else {
        vel_a_cps = app_i32_muldiv_u32(delta_a, 1000000u, dt_us);
        vel_b_cps = app_i32_muldiv_u32(delta_b, 1000000u, dt_us);
    }

    enc_vel_raw = (vel_a_cps + vel_b_cps) / 2L;

    s->enc_vel_filt = app_filter_iir(s->enc_vel_filt,
                                     enc_vel_raw,
                                     APP_CTRL_VEL_FILT_OLD_NUM,
                                     APP_CTRL_VEL_FILT_NEW_NUM,
                                     APP_CTRL_VEL_FILT_DEN);

    invalid_a_now = svc_enc_get_invalid_a();
    invalid_b_now = svc_enc_get_invalid_b();

    s->enc_invalid_a_now = (invalid_a_now != s->enc_invalid_a_prev) ? 1u : 0u;
    s->enc_invalid_b_now = (invalid_b_now != s->enc_invalid_b_prev) ? 1u : 0u;

    s->enc_invalid_a_prev = invalid_a_now;
    s->enc_invalid_b_prev = invalid_b_now;
}

/* Optional fault on fresh invalid quadrature. */
static void app_check_encoder_health(app_state_t *s)
{
#if (APP_FAULT_ON_ENCODER_INVALID != 0u)
    if ((s->enc_invalid_a_now != 0u) || (s->enc_invalid_b_now != 0u)) {
        app_enter_fault(s, "encoder invalid");
    }
#else
    (void)s;
#endif
}

/* -------------------------------------------------------------------------- */
/* Arming and safety                                                          */
/* -------------------------------------------------------------------------- */

/* Arm only after holding close to trim for APP_ARM_HOLD_MS. */
static void app_update_arm_state(app_state_t *s)
{
    uint32_t now_ms;
    int32_t trim_err_mdeg;

    if (s->motor_armed != 0u) {
        return;
    }

    now_ms = hal_time2_ms();
    trim_err_mdeg = s->pitch_est_mdeg - APP_PITCH_SETPOINT_MDEG;

    if ((uint32_t)app_iabs32(trim_err_mdeg) <= (uint32_t)APP_ARM_PITCH_WINDOW_MDEG) {
        if (s->arm_window_ms == 0u) {
            s->arm_window_ms = now_ms;
        } else if ((now_ms - s->arm_window_ms) >= APP_ARM_HOLD_MS) {
#if (EN_MOTORS != 0u)
            drv_tb6612_enable();
#endif
            s->motor_armed = 1u;
            s->arm_window_ms = 0u;
            app_start_trim_ramp(s, s->pitch_est_mdeg);
        }
    } else {
        s->arm_window_ms = 0u;
    }
}

/* Hard body-angle fault once the drivetrain is armed. */
static void app_check_safety(app_state_t *s)
{
    if ((s->motor_armed != 0u) &&
        (app_iabs32(s->pitch_est_mdeg) > APP_PITCH_FAULT_MDEG)) {
        app_enter_fault(s, "pitch limit");
    }
}

/* -------------------------------------------------------------------------- */
/* Control law                                                                */
/* -------------------------------------------------------------------------- */

/*
 * Nominal cascade:
 * enc_count -> kx -> vel_ref -> kv -> enc_lean -> angle_err -> kp/kd -> cmd
 * angle_err = enc_lean - pitch_est_mdeg
 */
static void app_compute_nominal_control(app_state_t *s)
{
    int32_t base_lean_mdeg;

    /* Position loop drives wheel count back toward zero. */
    s->kx_term = -(g_app_kx_enc_q * s->enc_count) / APP_GAIN_SCALE;

    s->vel_ref_cps = app_clamp_i32(s->kx_term,
                                   -APP_VEL_REF_LIM_CPS,
                                    APP_VEL_REF_LIM_CPS);

    /* Velocity loop turns wheel-speed error into lean correction. */
    s->vel_err_cps = s->vel_ref_cps - s->enc_vel_filt;

    s->kv_term = (g_app_kv_enc_q * s->vel_err_cps) / APP_GAIN_SCALE;
    s->kv_term = app_clamp_i32(s->kv_term,
                               -APP_KV_TERM_LIM_MDEG,
                                APP_KV_TERM_LIM_MDEG);

    base_lean_mdeg = app_current_base_lean_mdeg(s);
    s->enc_lean = base_lean_mdeg + s->kv_term;
    s->enc_lean = app_clamp_i32(s->enc_lean,
                                APP_ENC_LEAN_MIN_MDEG,
                                APP_ENC_LEAN_MAX_MDEG);

    s->angle_err = s->enc_lean - s->pitch_est_mdeg;

    /* Clamp pitch-rate before the D term. */
    s->rate_ctrl = app_clamp_i32(s->pitch_rate_mdps,
                                 -RATE_CTRL_LIM_MDPS,
                                  RATE_CTRL_LIM_MDPS);

    s->kp_term = (-g_app_kp_mpu_q * s->angle_err) / APP_GAIN_SCALE;
    s->kd_term = (-g_app_kd_mpu_q * s->rate_ctrl) / APP_GAIN_SCALE;
    s->kd_term = app_clamp_i32(s->kd_term,
                               -APP_KD_TERM_LIM_PM,
                                APP_KD_TERM_LIM_PM);
    s->motor_cmd_nom_raw = s->kp_term + s->kd_term;
}

/* While disarmed, keep sensing alive but zero the control-side state. */
static void app_hold_disarmed_control_state(app_state_t *s)
{
    s->vel_ref_cps        = 0;
    s->vel_err_cps        = 0;
    s->enc_lean           = 0;
    s->angle_err          = 0;
    s->rate_ctrl          = 0;
    s->kx_term            = 0;
    s->kv_term            = 0;
    s->kp_term            = 0;
    s->kd_term            = 0;
    s->motor_cmd_nom_raw  = 0;
    s->motor_cmd          = 0;

    s->brake_mode                 = 0u;
    s->zero_cross_active_delay_cycles = 0u;
    s->zero_cross_active_cycles   = 0u;
    s->zero_cross_active_cmd_pm   = 0;
    s->pitch_est_prev_mdeg        = s->pitch_est_mdeg;
}

/* One-cycle control composition. */
static void app_compute_control(app_state_t *s)
{
    if (s->motor_armed == 0u) {
        app_hold_disarmed_control_state(s);
        return;
    }

    app_compute_nominal_control(s);
    s->motor_cmd = app_clamp_i32(s->motor_cmd_nom_raw, -1000L, 1000L);
}

/*
 * Selective brake policy.
 * 0: no brake
 * 1: near-zero cmd with nontrivial wheel speed
 * 2: small reverse cmd against a still-fast wheel
 */
#if (APP_ENABLE_SELECTIVE_BRAKE != 0u)
static uint8_t app_brake_mode(const app_state_t *s)
{
    int32_t cmd_abs;
    int32_t vel_abs;
    int32_t cmd_sign;
    int32_t vel_sign;

    cmd_abs = app_iabs32(s->motor_cmd);
    vel_abs = app_iabs32(s->enc_vel_filt);

    if ((cmd_abs <= APP_BRAKE_ZERO_CMD_PM) &&
        (vel_abs >= APP_BRAKE_ZERO_VEL_CPS)) {
        return 1u;
    }

    cmd_sign = app_sign_i32(s->motor_cmd);
    vel_sign = app_sign_i32(s->enc_vel_filt);

    if ((cmd_sign != 0L) &&
        (vel_sign != 0L) &&
        (cmd_sign != vel_sign) &&
        (cmd_abs <= APP_BRAKE_REVERSE_CMD_PM) &&
        (vel_abs >= APP_BRAKE_REVERSE_VEL_CPS)) {
        return 2u;
    }
    return 0u;
}
#endif

/*
 * Zero-cross override window.
 * The window arms from a detected pitch sign flip through 0 mdeg. Positive
 * delay starts after the crossing; non-positive delay starts immediately. The
 * wheel-velocity sign at the crossing sets the override direction; a zero
 * override command keeps the window as coast-only.
 */
static uint8_t app_zero_cross_active_override_now(app_state_t *s)
{
    uint8_t delay_cycles;
    uint8_t active_cycles;
    int32_t vel_sign;
    uint8_t crossed_zero;

    if (s->zero_cross_active_delay_cycles != 0u) {
        s->zero_cross_active_delay_cycles--;

        if ((s->zero_cross_active_delay_cycles == 0u) &&
            (s->zero_cross_active_cycles != 0u)) {
            s->zero_cross_active_cycles--;
            s->pitch_est_prev_mdeg = s->pitch_est_mdeg;
            return 1u;
        }

        s->pitch_est_prev_mdeg = s->pitch_est_mdeg;
        return 0u;
    }

    if (s->zero_cross_active_cycles != 0u) {
        s->zero_cross_active_cycles--;
        s->pitch_est_prev_mdeg = s->pitch_est_mdeg;
        return 1u;
    }

    crossed_zero = (uint8_t)((((s->pitch_est_prev_mdeg < 0L) &&
                               (s->pitch_est_mdeg >= 0L)) ||
                              ((s->pitch_est_prev_mdeg > 0L) &&
                               (s->pitch_est_mdeg <= 0L))) ? 1u : 0u);
    delay_cycles = app_zero_cross_active_delay_cycles();
    active_cycles = app_zero_cross_active_cycles();
    vel_sign = app_sign_i32(s->enc_vel_filt);

    if ((active_cycles != 0u) &&
        (crossed_zero != 0u) &&
        (vel_sign != 0L) &&
        (app_iabs32(s->enc_vel_filt) >= APP_ZERO_CROSS_ACTIVE_MIN_VEL_CPS)) {
        if (app_zero_cross_active_start(s, delay_cycles, active_cycles,
                                        vel_sign) != 0u) {
            s->pitch_est_prev_mdeg = s->pitch_est_mdeg;
            return 1u;
        }

        s->pitch_est_prev_mdeg = s->pitch_est_mdeg;
        return 0u;
    }

    s->zero_cross_active_cmd_pm = 0;
    s->pitch_est_prev_mdeg = s->pitch_est_mdeg;
    return 0u;
}

/* Apply the final output-stage policy and drive command. */
static void app_apply_motor_command(app_state_t *s)
{
    s->brake_mode = 0u;

    if (s->motor_armed == 0u) {
        s->zero_cross_active_delay_cycles = 0u;
        s->zero_cross_active_cycles = 0u;
        s->zero_cross_active_cmd_pm = 0;
        s->pitch_est_prev_mdeg = s->pitch_est_mdeg;
        drv_tb6612_set_both_cmd_promille(0, 0);
        return;
    }

    if (app_zero_cross_active_override_now(s) != 0u) {
        s->brake_mode = 3u;
        s->motor_cmd = s->zero_cross_active_cmd_pm;
        drv_tb6612_set_both_cmd_promille(s->zero_cross_active_cmd_pm,
                                         s->zero_cross_active_cmd_pm);
        return;
    }

#if (APP_ENABLE_SELECTIVE_BRAKE != 0u)
    s->brake_mode = app_brake_mode(s);

    if (s->brake_mode != 0u) {
        drv_tb6612_stop(DRV_TB_MOTOR_A, DRV_TB_BRAKE);
        drv_tb6612_stop(DRV_TB_MOTOR_B, DRV_TB_BRAKE);
        return;
    }
#endif

    drv_tb6612_set_both_cmd_promille((int16_t)s->motor_cmd,
                                     (int16_t)s->motor_cmd);
}

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

/* Full APP bring-up. Ordering is deliberate. */
uint8_t app_init(app_state_t *s)
{
    term_init(BAUD);
    term_puts("Boot");
    term_newline();

    hal_time2_init_1ms();

    drv_tb6612_init(PWM_HZ);
    drv_tb6612_disable();

    svc_enc_init();
    svc_enc_reset();

    twi_init(TWI_HZ);

    if (!app_mpu_init(s)) {
        return 0u;
    }

    if (!app_calibrate_gyro(s)) {
        return 0u;
    }

    if (!app_read_mpu_sample(s)) {
        return 0u;
    }

    app_init_pitch_estimator(s);

    s->enc_vel_filt = 0;

    app_reset_runtime_state(s);

    s->enc_invalid_a_prev = svc_enc_get_invalid_a();
    s->enc_invalid_b_prev = svc_enc_get_invalid_b();

    sei();

    term_puts("Init OK");
    term_newline();

    return 1u;
}

/* -------------------------------------------------------------------------- */
/* One control step                                                           */
/* -------------------------------------------------------------------------- */

/* One fixed-period foreground control update. */
void app_control_step(app_state_t *s)
{
    static uint8_t ctrl_start_valid = 0u;
    static hal_time2_stamp_t ctrl_start_prev;
    hal_time2_stamp_t ctrl_start;
    hal_time2_stamp_t imu_start;
    hal_time2_stamp_t imu_end;
    hal_time2_stamp_t ctrl_end;

    ctrl_start = hal_time2_now();

    if (ctrl_start_valid != 0u) {
        s->ctrl_dt_us_last = hal_time2_elapsed_us(ctrl_start_prev, ctrl_start);
    } else {
        s->ctrl_dt_us_last = (uint32_t)APP_CTRL_DT_MS * 1000u;
        ctrl_start_valid = 1u;
    }
    ctrl_start_prev = ctrl_start;

    svc_enc_update();

    imu_start = hal_time2_now();
    if (!app_read_mpu_sample(s)) {
        app_enter_fault(s, "MPU read failed");
    }
    imu_end = hal_time2_now();
    s->imu_read_us_last = hal_time2_elapsed_us(imu_start, imu_end);

    app_update_pitch_estimator(s, s->ctrl_dt_us_last);
    app_update_encoder_state(s, s->ctrl_dt_us_last);
    app_check_encoder_health(s);
    app_update_arm_state(s);
    app_compute_control(s);
    app_check_safety(s);
    app_apply_motor_command(s);

    ctrl_end = hal_time2_now();
    s->ctrl_exec_us_last = hal_time2_elapsed_us(ctrl_start, ctrl_end);
}

/* -------------------------------------------------------------------------- */
/* Main loop                                                                  */
/* -------------------------------------------------------------------------- */

/* Fixed-rate foreground scheduler with bounded control catch-up. */
void app_run(app_state_t *s)
{
    uint32_t next_ctrl_ms;
    uint32_t now_ms;

    app_drive_off(s);

    app_print_current_params();

#if (APP_ENABLE_TELEMETRY != 0u)
    app_print_telemetry_columns();
    s->telemetry_next_ms = hal_time2_ms() + APP_TELEM_DT_MS;
#endif

    next_ctrl_ms = hal_time2_ms() + APP_CTRL_DT_MS;

    for (;;) {
        uint8_t catchup_cycles;

        catchup_cycles = 0u;
        now_ms = hal_time2_ms();

        while ((int32_t)(now_ms - next_ctrl_ms) >= 0) {
            uint32_t lateness_ms;

            lateness_ms = now_ms - next_ctrl_ms;
            s->sched_late_ms_last = lateness_ms;

            catchup_cycles++;

            if (catchup_cycles > APP_MAX_CATCHUP_CYCLES) {
                app_enter_fault(s, "control overrun");
            }

            next_ctrl_ms += APP_CTRL_DT_MS;
            app_control_step(s);

            now_ms = hal_time2_ms();
        }

#if (APP_ENABLE_TELEMETRY != 0u)
        if ((int32_t)(hal_time2_ms() - s->telemetry_next_ms) >= 0) {
            s->telemetry_next_ms += APP_TELEM_DT_MS;
            app_print_telemetry(s);
        }
#endif
    }
}
