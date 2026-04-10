#include <avr/io.h>
#include <avr/interrupt.h>

#include "avr_stdint.h"
#include "util_term.h"
#include "APP_CONFIG.h"
#include "TELEMETRY.h"

/*
 * Telemetry formatting and field selection.
 * This layer only reports APP state; it does not compute control actions.
 */

/* -------------------------------------------------------------------------- */
/* Telemetry output format selection                                          */
/* -------------------------------------------------------------------------- */

/*
 * 0u: mixed human-readable output
 * 1u: fixed-width raw hex for logging
 */
#ifndef APP_TELEM_PRINT_HEX
#define APP_TELEM_PRINT_HEX      1u
#endif

/* -------------------------------------------------------------------------- */
/* Compile-time telemetry field selection                                     */
/* -------------------------------------------------------------------------- */

/* Compile-time field switches. Keep UART load and CSV layout under control. */

#ifndef APP_TELEM_F_STATUS
#define APP_TELEM_F_STATUS       1u
#endif

#ifndef APP_TELEM_F_ENC_COUNT
#define APP_TELEM_F_ENC_COUNT    0u
#endif

#ifndef APP_TELEM_F_ENC_VEL
#define APP_TELEM_F_ENC_VEL      1u
#endif

#ifndef APP_TELEM_F_EST
#define APP_TELEM_F_EST          1u
#endif

#ifndef APP_TELEM_F_RATE
#define APP_TELEM_F_RATE         1u
#endif

#ifndef APP_TELEM_F_RATE_CTL
#define APP_TELEM_F_RATE_CTL     1u
#endif

#ifndef APP_TELEM_F_VEL_REF
#define APP_TELEM_F_VEL_REF      0u
#endif

#ifndef APP_TELEM_F_KX_TERM
#define APP_TELEM_F_KX_TERM      1u
#endif

#ifndef APP_TELEM_F_VEL_ERR
#define APP_TELEM_F_VEL_ERR      0u
#endif

#ifndef APP_TELEM_F_KV_TERM
#define APP_TELEM_F_KV_TERM      1u
#endif

#ifndef APP_TELEM_F_ENC_LEAN
#define APP_TELEM_F_ENC_LEAN     0u
#endif

#ifndef APP_TELEM_F_ERR
#define APP_TELEM_F_ERR          1u
#endif

#ifndef APP_TELEM_F_KP_TERM
#define APP_TELEM_F_KP_TERM      1u
#endif

#ifndef APP_TELEM_F_KD_TERM
#define APP_TELEM_F_KD_TERM      1u
#endif

#ifndef APP_TELEM_F_CMD_NOM
#define APP_TELEM_F_CMD_NOM      1u
#endif

#ifndef APP_TELEM_F_CMD
#define APP_TELEM_F_CMD          1u
#endif

#ifndef APP_TELEM_F_SAT
#define APP_TELEM_F_SAT          0u
#endif

#ifndef APP_TELEM_F_BRAKE
#define APP_TELEM_F_BRAKE        1u
#endif

#ifndef APP_TELEM_F_BRAKE_TERM
#define APP_TELEM_F_BRAKE_TERM   0u
#endif

#ifndef APP_TELEM_F_IMU_US
#define APP_TELEM_F_IMU_US       0u
#endif

#ifndef APP_TELEM_F_CTRL_DT_US
#define APP_TELEM_F_CTRL_DT_US   1u
#endif

#ifndef APP_TELEM_F_CTRL_EXEC_US
#define APP_TELEM_F_CTRL_EXEC_US 1u
#endif

#ifndef APP_TELEM_F_SCHED_LATE_MS
#define APP_TELEM_F_SCHED_LATE_MS 1u
#endif

/* -------------------------------------------------------------------------- */
/* Telemetry field IDs                                                        */
/* -------------------------------------------------------------------------- */

/* Shared field table: enum id, CSV label, value expression. */
#define APP_TELEM_FIELD_TABLE(X) \
    X(STATUS,        "st",            app_telem_status_value(s)) \
    X(ENC_COUNT,     "enc_count",     s->enc_count) \
    X(ENC_VEL,       "enc_vel",       s->enc_vel_filt) \
    X(EST,           "est",           s->pitch_est_mdeg) \
    X(RATE,          "rate",          s->pitch_rate_mdps) \
    X(RATE_CTL,      "rate_ctl",      s->rate_ctrl) \
    X(VEL_REF,       "vel_ref",       s->vel_ref_cps) \
    X(KX_TERM,       "kx_term",       s->kx_term) \
    X(VEL_ERR,       "vel_err",       s->vel_err_cps) \
    X(KV_TERM,       "kv_term",       s->kv_term) \
    X(ENC_LEAN,      "enc_lean",      s->enc_lean) \
    X(ERR,           "err",           s->angle_err) \
    X(KP_TERM,       "kp_term",       s->kp_term) \
    X(KD_TERM,       "kd_term",       s->kd_term) \
    X(CMD_NOM,       "cmd_nom",       s->motor_cmd_nom_raw) \
    X(CMD,           "cmd",           s->motor_cmd) \
    X(SAT,           "sat",           app_telem_sat_value(s)) \
    X(BRAKE,         "brk",           (int32_t)s->brake_mode) \
    X(BRAKE_TERM,    "brk_term",      app_telem_brake_term_value(s)) \
    X(IMU_US,        "imu_us",        (int32_t)s->imu_read_us_last) \
    X(CTRL_DT_US,    "ctrl_dt_us",    (int32_t)s->ctrl_dt_us_last) \
    X(CTRL_EXEC_US,  "ctrl_exec_us",  (int32_t)s->ctrl_exec_us_last) \
    X(SCHED_LATE_MS, "late_ms",       (int32_t)s->sched_late_ms_last)

typedef enum
{
#define APP_TELEM_ENUM(name, label, value_expr) TELEM_FIELD_##name,
    APP_TELEM_FIELD_TABLE(APP_TELEM_ENUM)
#undef APP_TELEM_ENUM
} telem_field_id_t;

/* -------------------------------------------------------------------------- */
/* Packed status field                                                        */
/* -------------------------------------------------------------------------- */

/* Packed status word for fast log decoding. */
typedef uint16_t app_telem_status_t;

#define APP_ST_ARMED            ((app_telem_status_t)(1u << 0))
#define APP_ST_ARM_WIN          ((app_telem_status_t)(1u << 1))
#define APP_ST_ENC_INV_A        ((app_telem_status_t)(1u << 12))
#define APP_ST_ENC_INV_B        ((app_telem_status_t)(1u << 13))

/* -------------------------------------------------------------------------- */
/* Compile-time selected telemetry field list                                 */
/* -------------------------------------------------------------------------- */

/* Final field order for this build. */
static const uint8_t g_telem_fields[] =
{
#if (APP_TELEM_F_STATUS != 0u)
    TELEM_FIELD_STATUS,
#endif
#if (APP_TELEM_F_ENC_COUNT != 0u)
    TELEM_FIELD_ENC_COUNT,
#endif
#if (APP_TELEM_F_ENC_VEL != 0u)
    TELEM_FIELD_ENC_VEL,
#endif
#if (APP_TELEM_F_EST != 0u)
    TELEM_FIELD_EST,
#endif
#if (APP_TELEM_F_RATE != 0u)
    TELEM_FIELD_RATE,
#endif
#if (APP_TELEM_F_RATE_CTL != 0u)
    TELEM_FIELD_RATE_CTL,
#endif
#if (APP_TELEM_F_VEL_REF != 0u)
    TELEM_FIELD_VEL_REF,
#endif
#if (APP_TELEM_F_KX_TERM != 0u)
    TELEM_FIELD_KX_TERM,
#endif
#if (APP_TELEM_F_VEL_ERR != 0u)
    TELEM_FIELD_VEL_ERR,
#endif
#if (APP_TELEM_F_KV_TERM != 0u)
    TELEM_FIELD_KV_TERM,
#endif
#if (APP_TELEM_F_ENC_LEAN != 0u)
    TELEM_FIELD_ENC_LEAN,
#endif
#if (APP_TELEM_F_ERR != 0u)
    TELEM_FIELD_ERR,
#endif
#if (APP_TELEM_F_KP_TERM != 0u)
    TELEM_FIELD_KP_TERM,
#endif
#if (APP_TELEM_F_KD_TERM != 0u)
    TELEM_FIELD_KD_TERM,
#endif
#if (APP_TELEM_F_CMD_NOM != 0u)
    TELEM_FIELD_CMD_NOM,
#endif
#if (APP_TELEM_F_CMD != 0u)
    TELEM_FIELD_CMD,
#endif
#if (APP_TELEM_F_SAT != 0u)
    TELEM_FIELD_SAT,
#endif
#if (APP_TELEM_F_BRAKE != 0u)
    TELEM_FIELD_BRAKE,
#endif
#if (APP_TELEM_F_BRAKE_TERM != 0u)
    TELEM_FIELD_BRAKE_TERM,
#endif
#if (APP_TELEM_F_IMU_US != 0u)
    TELEM_FIELD_IMU_US,
#endif
#if (APP_TELEM_F_CTRL_DT_US != 0u)
    TELEM_FIELD_CTRL_DT_US,
#endif
#if (APP_TELEM_F_CTRL_EXEC_US != 0u)
    TELEM_FIELD_CTRL_EXEC_US,
#endif
#if (APP_TELEM_F_SCHED_LATE_MS != 0u)
    TELEM_FIELD_SCHED_LATE_MS,
#endif
};

#define APP_TELEM_FIELD_COUNT \
    ((uint8_t)(sizeof(g_telem_fields) / sizeof(g_telem_fields[0])))

/* -------------------------------------------------------------------------- */
/* Internal helpers                                                           */
/* -------------------------------------------------------------------------- */

/* Build the packed status word from APP state. */
static app_telem_status_t app_telem_make_status(const app_state_t *s);

/* Local helpers used only by telemetry. */
static int32_t app_telem_clamp_i32(int32_t x, int32_t lo, int32_t hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

/* Status uses the same field-value path as the numeric telemetry fields. */
static int32_t app_telem_status_value(const app_state_t *s)
{
    return (int32_t)app_telem_make_status(s);
}

static app_telem_status_t app_telem_make_status(const app_state_t *s)
{
    app_telem_status_t st;

    st = 0u;

    if (s->motor_armed != 0u) {
        st |= APP_ST_ARMED;
    }

    if ((s->motor_armed == 0u) && (s->arm_window_ms != 0u)) {
        st |= APP_ST_ARM_WIN;
    }

    if (s->enc_invalid_a_now != 0u) {
        st |= APP_ST_ENC_INV_A;
    }

    if (s->enc_invalid_b_now != 0u) {
        st |= APP_ST_ENC_INV_B;
    }

    return st;
}

/* 1 when final command differs from the unclamped value. */
static int32_t app_telem_sat_value(const app_state_t *s)
{
    return (s->motor_cmd != app_telem_clamp_i32(s->motor_cmd_nom_raw,
                                                -1000L,
                                                 1000L)) ? 1 : 0;
}

static int32_t app_telem_nominal_output_cmd(const app_state_t *s)
{
    return app_telem_clamp_i32(s->motor_cmd_nom_raw, -1000L, 1000L);
}

/*
 * Brake override in command space.
 * This reports suppressed drive command, not physical brake torque.
 */
static int32_t app_telem_brake_term_value(const app_state_t *s)
{
    if (s->brake_mode == 3u) {
        return (int32_t)s->zero_cross_active_cmd_pm -
               app_telem_nominal_output_cmd(s);
    }

    return (s->brake_mode != 0u) ? (-s->motor_cmd) : 0;
}

/* Map one field id to its CSV label. */
static const char *app_telem_field_name(uint8_t field_id)
{
    switch (field_id) {
        #define APP_TELEM_NAME_CASE(name, label, value_expr) \
        case TELEM_FIELD_##name: \
            return label;
        APP_TELEM_FIELD_TABLE(APP_TELEM_NAME_CASE)
        #undef APP_TELEM_NAME_CASE

        default:
            return "unknown";
    }
}

/* Map one field id to its current value. */
static int32_t app_telem_field_value(uint8_t field_id, const app_state_t *s)
{
    switch (field_id) {
        #define APP_TELEM_VALUE_CASE(name, label, value_expr) \
        case TELEM_FIELD_##name: \
            return (int32_t)(value_expr);
        APP_TELEM_FIELD_TABLE(APP_TELEM_VALUE_CASE)
        #undef APP_TELEM_VALUE_CASE

        default:
            return 0;
    }
}

/* Print one field in the selected text or raw-hex format. */
static void app_telem_print_value(uint8_t field_id, const app_state_t *s)
{
    int32_t value;

    value = app_telem_field_value(field_id, s);

#if (APP_TELEM_PRINT_HEX != 0u)
    if (field_id == TELEM_FIELD_STATUS) {
        term_put_hex4_u16_raw((uint16_t)value);
    } else {
        term_put_hex8_u32_raw((uint32_t)value);
    }
#else
    if (field_id == TELEM_FIELD_STATUS) {
        term_put_hex_u16((uint16_t)value);
    } else {
        term_put_i32(value);
    }
#endif
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

/* Boot-time parameter dump for log traceability. */
void app_print_current_params(void)
{
    term_puts("pitch_ofs=");
    term_put_i32(g_app_pitch_offset_mdeg);
    term_puts(" ");
    term_puts("Kx=");
    term_put_i32(g_app_kx_enc_q);
    term_puts(" Kv=");
    term_put_i32(g_app_kv_enc_q);
    term_puts(" Kp=");
    term_put_i32(g_app_kp_mpu_q);
    term_puts(" Kd=");
    term_put_i32(g_app_kd_mpu_q);
    term_puts(" rate_lim=");
    term_put_i32(RATE_CTRL_LIM_MDPS);
    term_puts(" dt_ms=");
    term_put_u32(APP_CTRL_DT_MS);
    term_puts(" twi_hz=");
    term_put_u32(TWI_HZ);
#if (APP_ENABLE_SELECTIVE_BRAKE != 0u)
    term_puts(" brake=sel");
#else
    term_puts(" brake=off");
#endif
    term_newline();
}

/* Print the CSV header for the current field list. */
void app_print_telemetry_columns(void)
{
    uint8_t i;
    uint8_t field_id;

    for (i = 0u; i < APP_TELEM_FIELD_COUNT; i++) {
        if (i != 0u) {
            term_putc(',');
        }

        field_id = g_telem_fields[i];
        term_puts(app_telem_field_name(field_id));
    }

    term_newline();
}

/* Print one telemetry row in the current field order. */
void app_print_telemetry(const app_state_t *s)
{
    uint8_t i;
    uint8_t field_id;

    for (i = 0u; i < APP_TELEM_FIELD_COUNT; i++) {
        if (i != 0u) {
            term_putc(',');
        }

        field_id = g_telem_fields[i];
        app_telem_print_value(field_id, s);
    }

    term_newline();
}
