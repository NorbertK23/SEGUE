/**
 * @file hal_pwm1.h
 * @brief Timer1 PWM output HAL.
 *
 * Configuration:
 * - Fast PWM mode 14 with `ICR1` as TOP
 * - `OC1A -> PB1` and `OC1B -> PB2`
 * - intended for motor-drive PWM output
 */

#ifndef HAL_PWM1_H_
#define HAL_PWM1_H_

#include <avr/io.h>
#include "avr_stdint.h"

/** @brief Select Timer1 PWM channel A or B. */
typedef enum
{
    HAL_PWM1_CH_A = 0,  // OC1A (PB1)
    HAL_PWM1_CH_B = 1   // OC1B (PB2)
} hal_pwm1_ch_t;

/**
 * @brief Initialize Timer1 PWM generation.
 *
 * Uses prescaler `= 8`, computes `TOP` from `F_CPU`, and initializes both
 * channels to zero duty.
 *
 * @param[in] pwm_hz Desired PWM frequency in hertz.
 */
void hal_pwm1_init(uint16_t pwm_hz);

/** @brief Enable PWM output on the selected channel. */
void hal_pwm1_enable_channel(hal_pwm1_ch_t ch);

/** @brief Disable PWM output on the selected channel. */
void hal_pwm1_disable_channel(hal_pwm1_ch_t ch);

/**
 * @brief Set duty cycle in promille for the selected channel.
 *
 * @param[in] ch Target PWM channel.
 * @param[in] promille Duty cycle in the range `0..1000`.
 */
void hal_pwm1_set_duty_promille(hal_pwm1_ch_t ch, uint16_t promille);

#endif
