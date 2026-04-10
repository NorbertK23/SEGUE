/*
 * drv_tb6612fng.c
 *
 *  Created on: 4 Mar 2026
 *      Author: Norbert Kania
 * Board-specific TB6612FNG driver for ATmega328P / Arduino Nano pinout.
 */

#include <avr/io.h>
#include "avr_stdint.h"
#include "drv_tb6612fng.h"

/* =========================
 * Fixed board pin mapping
 * ========================= */

/* STBY -> D6 -> PD6 */
#define TB_STBY_DDR     DDRD
#define TB_STBY_PORT    PORTD
#define TB_STBY_BIT     PD6

/* Motor A direction pins */
#define TB_AIN1_DDR     DDRD
#define TB_AIN1_PORT    PORTD
#define TB_AIN1_BIT     PD7

#define TB_AIN2_DDR     DDRB
#define TB_AIN2_PORT    PORTB
#define TB_AIN2_BIT     PB0

/* Motor B direction pins */
#define TB_BIN1_DDR     DDRB
#define TB_BIN1_PORT    PORTB
#define TB_BIN1_BIT     PB3

#define TB_BIN2_DDR     DDRB
#define TB_BIN2_PORT    PORTB
#define TB_BIN2_BIT     PB4

/* PWM channel mapping */
#define TB_PWMA_CH      HAL_PWM1_CH_A   /* D9  / PB1 / OC1A */
#define TB_PWMB_CH      HAL_PWM1_CH_B   /* D10 / PB2 / OC1B */

/* =========================
 * Small private GPIO helpers
 * ========================= */

static void gpio_set_output(volatile uint8_t* ddr, uint8_t bit)
{
    *ddr |= (uint8_t)(1u << bit);
}

static void gpio_write_high(volatile uint8_t* port, uint8_t bit)
{
    *port |= (uint8_t)(1u << bit);
}

static void gpio_write_low(volatile uint8_t* port, uint8_t bit)
{
    *port &= (uint8_t)~(1u << bit);
}

/* =========================
 * Private motor helpers
 * ========================= */

static void tb_set_pwm(drv_tb_motor_t m, uint16_t promille)
{
    if (m == DRV_TB_MOTOR_A)
    {
        hal_pwm1_set_duty_promille(TB_PWMA_CH, promille);
    }
    else
    {
        hal_pwm1_set_duty_promille(TB_PWMB_CH, promille);
    }
}

static void tb_motor_a_forward(void)
{
    /* Forward convention: IN1=1, IN2=0 */
    gpio_write_high(&TB_AIN1_PORT, TB_AIN1_BIT);
    gpio_write_low(&TB_AIN2_PORT, TB_AIN2_BIT);
}

static void tb_motor_a_reverse(void)
{
    gpio_write_low(&TB_AIN1_PORT, TB_AIN1_BIT);
    gpio_write_high(&TB_AIN2_PORT, TB_AIN2_BIT);
}

static void tb_motor_b_forward(void)
{
    /* Forward convention: IN1=1, IN2=0 */
    gpio_write_high(&TB_BIN1_PORT, TB_BIN1_BIT);
    gpio_write_low(&TB_BIN2_PORT, TB_BIN2_BIT);
}

static void tb_motor_b_reverse(void)
{
    gpio_write_low(&TB_BIN1_PORT, TB_BIN1_BIT);
    gpio_write_high(&TB_BIN2_PORT, TB_BIN2_BIT);
}

static void tb_motor_a_stop(drv_tb_stop_mode_t mode)
{
    if (mode == DRV_TB_BRAKE)
    {
        gpio_write_high(&TB_AIN1_PORT, TB_AIN1_BIT);
        gpio_write_high(&TB_AIN2_PORT, TB_AIN2_BIT);
    }
    else
    {
        gpio_write_low(&TB_AIN1_PORT, TB_AIN1_BIT);
        gpio_write_low(&TB_AIN2_PORT, TB_AIN2_BIT);
    }
}

static void tb_motor_b_stop(drv_tb_stop_mode_t mode)
{
    if (mode == DRV_TB_BRAKE)
    {
        gpio_write_high(&TB_BIN1_PORT, TB_BIN1_BIT);
        gpio_write_high(&TB_BIN2_PORT, TB_BIN2_BIT);
    }
    else
    {
        gpio_write_low(&TB_BIN1_PORT, TB_BIN1_BIT);
        gpio_write_low(&TB_BIN2_PORT, TB_BIN2_BIT);
    }
}

/* =========================
 * Public API
 * ========================= */

void drv_tb6612_init(uint16_t pwm_hz)
{
    /* Direction and standby pins as outputs */
    gpio_set_output(&TB_STBY_DDR, TB_STBY_BIT);

    gpio_set_output(&TB_AIN1_DDR, TB_AIN1_BIT);
    gpio_set_output(&TB_AIN2_DDR, TB_AIN2_BIT);

    gpio_set_output(&TB_BIN1_DDR, TB_BIN1_BIT);
    gpio_set_output(&TB_BIN2_DDR, TB_BIN2_BIT);

    /* Safe state before PWM starts */
    gpio_write_low(&TB_STBY_PORT, TB_STBY_BIT);
    tb_motor_a_stop(DRV_TB_COAST);
    tb_motor_b_stop(DRV_TB_COAST);

    /* PWM init */
    hal_pwm1_init(pwm_hz);
    hal_pwm1_enable_channel(TB_PWMA_CH);
    hal_pwm1_enable_channel(TB_PWMB_CH);

    tb_set_pwm(DRV_TB_MOTOR_A, 0u);
    tb_set_pwm(DRV_TB_MOTOR_B, 0u);
}

void drv_tb6612_enable(void)
{
    gpio_write_high(&TB_STBY_PORT, TB_STBY_BIT);
}

void drv_tb6612_disable(void)
{
    tb_set_pwm(DRV_TB_MOTOR_A, 0u);
    tb_set_pwm(DRV_TB_MOTOR_B, 0u);

    tb_motor_a_stop(DRV_TB_COAST);
    tb_motor_b_stop(DRV_TB_COAST);

    gpio_write_low(&TB_STBY_PORT, TB_STBY_BIT);
}

void drv_tb6612_stop(drv_tb_motor_t m, drv_tb_stop_mode_t mode)
{
    tb_set_pwm(m, 0u);

    if (m == DRV_TB_MOTOR_A)
    {
        tb_motor_a_stop(mode);
    }
    else
    {
        tb_motor_b_stop(mode);
    }
}

void drv_tb6612_set_cmd_promille(drv_tb_motor_t m, int16_t cmd_promille)
{
    uint16_t duty;

    if (cmd_promille > 1000)
    {
        cmd_promille = 1000;
    }
    if (cmd_promille < -1000)
    {
        cmd_promille = -1000;
    }

    if (cmd_promille == 0)
    {
        drv_tb6612_stop(m, DRV_TB_COAST);
        return;
    }

    if (cmd_promille < 0)
    {
        duty = (uint16_t)(-cmd_promille);

        if (m == DRV_TB_MOTOR_A)
        {
            tb_motor_a_reverse();
        }
        else
        {
            tb_motor_b_reverse();
        }
    }
    else
    {
        duty = (uint16_t)cmd_promille;

        if (m == DRV_TB_MOTOR_A)
        {
            tb_motor_a_forward();
        }
        else
        {
            tb_motor_b_forward();
        }
    }

    tb_set_pwm(m, duty);
}

void drv_tb6612_set_both_cmd_promille(int16_t cmdA, int16_t cmdB)
{
    drv_tb6612_set_cmd_promille(DRV_TB_MOTOR_A, cmdA);
    drv_tb6612_set_cmd_promille(DRV_TB_MOTOR_B, cmdB);
}
