#include "APP_CONFIG.h"
#include "APP_STATE.h"

/* Active tuning set and fixed runtime configuration. */

/* Raw accel-pitch reading for the mechanically balanced frame. */
const int32_t g_app_pitch_offset_mdeg = -4500;

/* Nominal cascade gains. */
const int32_t g_app_kx_enc_q = 500;   /* prior trial: 2500 */
const int32_t g_app_kv_enc_q = 0;   /* prior trial: 550 */
const int32_t g_app_kp_mpu_q = 1400;  /* prior trial: 750 */
const int32_t g_app_kd_mpu_q = 120;   /* prior trial: 70 */

/* -------------------------------------------------------------------------- */
/* Fixed MPU6050 runtime configuration                                        */
/* -------------------------------------------------------------------------- */

/* Fixed MPU setup used at APP startup. */
const mpu6050_cfg_t g_app_mpu_cfg =
{
    .clk        = MPU6050_CLK_PLL_XGYRO,
    .dlpf       = MPU6050_DLPF_94HZ,
    .smplrt_div = 1u,
    .gyro_fs    = MPU6050_GFS_500DPS,
    .accel_fs   = MPU6050_AFS_2G
};

/* -------------------------------------------------------------------------- */
/* Application state reset                                                    */
/* -------------------------------------------------------------------------- */

/* Whole-state reset from a zero template. */
void app_state_reset(app_state_t *s)
{
    static const app_state_t zero_state =
    {
        { 0 }, /* mpu_cal */
        { 0 }, /* raw */
        { 0 }, /* phys */
        0      /* all remaining scalar members become zero */
    };

    *s = zero_state;
}
