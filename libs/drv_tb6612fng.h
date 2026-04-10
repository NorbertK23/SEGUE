/**
 * @file drv_tb6612fng.h
 * @brief Board-specific TB6612FNG motor-driver API for the ATmega328P rover.
 *
 * Fixed pin mapping:
 * - STBY -> D6  -> PD6
 * - Motor A: PWMA -> D9, AIN1 -> D7, AIN2 -> D8
 * - Motor B: PWMB -> D10, BIN1 -> D11, BIN2 -> D12
 */

#ifndef DRV_TB6612FNG_H_
#define DRV_TB6612FNG_H_

#include <avr/io.h>
#include "avr_stdint.h"
#include "hal_pwm1.h"

/** @brief Select one of the two TB6612FNG motor channels. */
typedef enum
{
    DRV_TB_MOTOR_A = 0,
    DRV_TB_MOTOR_B = 1
} drv_tb_motor_t;

/** @brief Select the stop behavior when forcing a motor off. */
typedef enum
{
    DRV_TB_COAST = 0,
    DRV_TB_BRAKE = 1
} drv_tb_stop_mode_t;

/**
 * @brief Initialize GPIO and Timer1 PWM for the TB6612FNG.
 *
 * Leaves the bridge disabled, both motors coasted, and both duties at zero.
 *
 * @param[in] pwm_hz Desired PWM carrier frequency.
 */
void drv_tb6612_init(uint16_t pwm_hz);

/** @brief Enable the TB6612FNG bridge through `STBY`. */
void drv_tb6612_enable(void);
/** @brief Disable the TB6612FNG bridge through `STBY`. */
void drv_tb6612_disable(void);

/**
 * @brief Stop one motor with the selected stop mode and zero duty.
 *
 * @param[in] m Target motor channel.
 * @param[in] mode Coast or electronic brake.
 */
void drv_tb6612_stop(drv_tb_motor_t m, drv_tb_stop_mode_t mode);

/**
 * @brief Apply a signed motor command in promille.
 *
 * Range: `-1000 .. +1000`
 * - positive = forward
 * - negative = reverse
 * - zero = coast
 *
 * @param[in] m Target motor channel.
 * @param[in] cmd_promille Signed drive command.
 */
void drv_tb6612_set_cmd_promille(drv_tb_motor_t m, int16_t cmd_promille);

/**
 * @brief Apply signed promille commands to both motors.
 *
 * @param[in] cmdA Command for motor A.
 * @param[in] cmdB Command for motor B.
 */
void drv_tb6612_set_both_cmd_promille(int16_t cmdA, int16_t cmdB);

#endif /* DRV_TB6612FNG_H_ */
