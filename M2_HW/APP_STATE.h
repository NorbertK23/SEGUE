#ifndef APP_STATE_H_
#define APP_STATE_H_

#include "avr_stdint.h"
#include "drv_mpu6050.h"
#include "svc_mpu6050.h"

/**
 * @file APP_STATE.h
 * @brief Central application runtime-state definition.
 *
 * APP owns this structure and passes it through the foreground control loop.
 * Lower HAL, DRV, and SVC layers do not store or mutate this object directly.
 */

/**
 * @brief Central application runtime state.
 *
 * Purpose:
 * - keep all foreground control state in one explicit place
 * - make telemetry export straightforward
 * - keep mode and state transitions visible and deterministic
 *
 * The structure is intentionally focused on persistent control state and the
 * small amount of current-cycle observability state still needed by APP or
 * telemetry.
 */
typedef struct
{
    /* ---------------------------------------------------------------------- */
    /* MPU sample path                                                        */
    /* ---------------------------------------------------------------------- */

    /*
     * Calibration data computed during startup.
     * Applied to each raw MPU sample before conversion to physical units.
     */
    mpu6050_cal_t   mpu_cal;

    /*
     * Latest raw 14-byte MPU sample after register read.
     * Units are device-native ADC counts.
     */
    mpu6050_raw14_t raw;

    /*
     * Latest calibrated and scaled physical sample.
     * Units come from svc_mpu6050:
     * - accel: mg
     * - gyro:  mdps
     * - temp:  cC
     */
    mpu6050_phys_t  phys;

    /* ---------------------------------------------------------------------- */
    /* Pitch estimator state                                                  */
    /* ---------------------------------------------------------------------- */

    /*
     * Main fused pitch estimate used by control and safety logic.
     * Units: milli-degrees.
     */
    int32_t pitch_est_mdeg;

    /*
     * Measured pitch angular rate used by the D term.
     * Units: milli-degrees per second.
     */
    int32_t pitch_rate_mdps;

    /* ---------------------------------------------------------------------- */
    /* Encoder-derived robot motion state                                     */
    /* ---------------------------------------------------------------------- */

    /*
     * Mean wheel-position count used as the robot translation coordinate.
     * Built from both wheel counts after service-layer sign normalization.
     */
    int32_t  enc_count;

    /*
     * Filtered wheel velocity used by telemetry and control decisions.
     * This is the main velocity signal seen by higher-level APP logic.
     */
    int32_t  enc_vel_filt;

    /*
     * Previous cumulative invalid counters, stored so APP can compute the
     * next cycle's delta and fault immediately on new invalid transitions.
     */
    uint32_t enc_invalid_a_prev;
    uint32_t enc_invalid_b_prev;

    /*
     * Fresh invalid encoder transition flags for the current cycle.
     */
    uint8_t  enc_invalid_a_now;
    uint8_t  enc_invalid_b_now;

    /* ---------------------------------------------------------------------- */
    /* Arming trim-ramp state                                                 */
    /* ---------------------------------------------------------------------- */

    /*
     * After arming, APP can ramp from the current body angle toward the
     * commanded trim instead of applying the trim step instantly.
     */
    uint8_t  trim_ramp_active;

    /*
     * Remaining control cycles in the active trim ramp.
     */
    uint16_t trim_ramp_cycles_left;

    /*
     * Trim-ramp endpoints in milli-degrees.
     * start  = lean at ramp entry
     * target = commanded steady-state trim
     */
    int32_t  trim_ramp_start_mdeg;
    int32_t  trim_ramp_target_mdeg;

    /* ---------------------------------------------------------------------- */
    /* Nominal control intermediates                                          */
    /* ---------------------------------------------------------------------- */

    /*
     * Position-loop output:
     * desired wheel velocity derived from encoder count error.
     */
    int32_t vel_ref_cps;

    /*
     * Velocity-loop error:
     * desired wheel velocity minus measured wheel velocity.
     */
    int32_t vel_err_cps;

    /*
     * Effective lean command seen by the inner loop.
     * This combines base trim and encoder-derived lean shift.
     */
    int32_t enc_lean;

    /*
     * Inner-loop angle error.
     * Defined as:
     *     angle_err = enc_lean - pitch_est_mdeg
     */
    int32_t angle_err;

    /*
     * Pitch-rate signal after APP-side clamping.
     * This is the D-term input after rate limiting.
     */
    int32_t rate_ctrl;

    /*
     * Saved loop contributions for telemetry and tuning.
     * These are not merely debug values; they expose the decomposition of the
     * nominal controller into interpretable pieces.
     */
    int32_t kx_term;
    int32_t kv_term;
    int32_t kp_term;
    int32_t kd_term;

    /*
     * Motor command progression:
     * - motor_cmd_nom_raw: nominal control command before output-policy overrides
     * - motor_cmd:         final drive command after output-policy overrides
     */
    int32_t motor_cmd_nom_raw;
    int32_t motor_cmd;

    /* ---------------------------------------------------------------------- */
    /* Control timing diagnostics                                             */
    /* ---------------------------------------------------------------------- */

    /*
     * IMU burst-read execution time for the latest control cycle.
     * Units: microseconds.
     */
    uint32_t imu_read_us_last;

    /*
     * Measured elapsed time between successive control-step starts.
     * Units: microseconds.
     */
    uint32_t ctrl_dt_us_last;

    /*
     * Full control-step execution time for the latest cycle.
     */
    uint32_t ctrl_exec_us_last;

    /*
     * Scheduler lateness measured at control-step dispatch time.
     * Units: milliseconds.
     */
    uint32_t sched_late_ms_last;

    /*
     * Latest selective-brake action applied to the drivetrain output stage.
     * 0 -> normal drive / coast path
     * 1 -> zero-command braking
     * 2 -> reversal braking
     * 3 -> zero-cross active override window
     *
     * This is kept as a compact telemetry-oriented code so brake activity can
     * be correlated against wheel-speed reversals without expanding the packed
     * status word.
     */
    uint8_t  brake_mode;

    /*
     * Short-lived zero-cross active-window state.
     *
     * zero_cross_active_delay_cycles:
     *   remaining control cycles before the delayed reverse pulse begins
     *
     * zero_cross_active_cycles:
     *   remaining control cycles in the active reverse pulse
     *
     * zero_cross_active_cmd_pm:
     *   signed override command chosen from the instantaneous wheel-velocity
     *   sign when pitch crosses through 0 mdeg
     *
     * pitch_est_prev_mdeg:
     *   previous-cycle pitch estimate used to detect a sign flip through 0 mdeg
     */
    uint8_t  zero_cross_active_delay_cycles;
    uint8_t  zero_cross_active_cycles;
    int16_t  zero_cross_active_cmd_pm;
    int32_t  pitch_est_prev_mdeg;

    /* ---------------------------------------------------------------------- */
    /* Arming and telemetry scheduling                                        */
    /* ---------------------------------------------------------------------- */

    /*
     * Motor arming state.
     * 0 -> outputs held off
     * 1 -> motor commands may be applied
     */
    uint8_t  motor_armed;

    /*
     * Timestamp used to qualify the arming window.
     * Zero means the robot is not currently accumulating valid arm time.
     */
    uint32_t arm_window_ms;

    /*
     * Next scheduled telemetry emission time in milliseconds.
     */
    uint32_t telemetry_next_ms;

} app_state_t;

/**
 * @brief Reset the full application state to a known zero baseline.
 *
 * Called before initialization so APP starts from deterministic contents.
 *
 * @param[out] s State object to clear.
 */
void app_state_reset(app_state_t *s);

#endif /* APP_STATE_H_ */
