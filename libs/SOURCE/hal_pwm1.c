/*
 * pwm1.c
 *
 *  Created on: 16 Feb 2026
 *      Author: Norbert Kania
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "avr_stdint.h"
#include "hal_pwm1.h"

static void hal_pwm1_set_top_icr1(uint16_t top)
{
    uint8_t s = SREG;
    cli();
    ICR1 = top;
    SREG = s;
}

static void hal_pwm1_write_ocr1a(uint16_t v)
{
    uint8_t s = SREG;
    cli();
    OCR1A = v;
    SREG = s;
}

static void hal_pwm1_write_ocr1b(uint16_t v)
{
    uint8_t s = SREG;
    cli();
    OCR1B = v;
    SREG = s;
}

void hal_pwm1_init(uint16_t pwm_hz)
{
    /* Stop timer while configuring */
    TCCR1A = 0u;
    TCCR1B = 0u;
    TCNT1  = 0u;

    /* Mode 14: Fast PWM, TOP=ICR1
       WGM13:0 = 1110b
       - WGM11=1 (TCCR1A)
       - WGM12=1, WGM13=1 (TCCR1B)
    */
    TCCR1A = (1u << WGM11);
    TCCR1B = (1u << WGM13) | (1u << WGM12);

    /* Prescaler = 8 */
    TCCR1B |= (1u << CS11);

    /* Compute TOP for requested frequency:
       f_pwm = F_CPU / (prescaler * (1 + TOP))
       => TOP = (F_CPU/(prescaler*f_pwm)) - 1
    */
    if (pwm_hz == 0u) pwm_hz = 1u;

    {
        uint32_t top32 = (uint32_t)F_CPU;
        top32 /= (uint32_t)8u;
        top32 /= (uint32_t)pwm_hz;

        if (top32 == 0u) top32 = 1u;
        top32 -= 1u;

        if (top32 > 0xFFFFu) top32 = 0xFFFFu;

        hal_pwm1_set_top_icr1((uint16_t)top32);
    }

    /* Start with 0% duty */
    hal_pwm1_write_ocr1a(0u);
    hal_pwm1_write_ocr1b(0u);

    /* Channels are NOT enabled by default; call hal_pwm1_enable_channel(). */
}

void hal_pwm1_enable_channel(hal_pwm1_ch_t ch)
{
    if (ch == HAL_PWM1_CH_A)
    {
        /* PB1 output */
        DDRB |= (1u << PB1);
        /* Non-inverting on OC1A: COM1A1=1 COM1A0=0 */
        TCCR1A = (uint8_t)((TCCR1A & (uint8_t)~((1u << COM1A1) | (1u << COM1A0))) | (1u << COM1A1));
    }
    else
    {
        /* PB2 output */
        DDRB |= (1u << PB2);
        /* Non-inverting on OC1B: COM1B1=1 COM1B0=0 */
        TCCR1A = (uint8_t)((TCCR1A & (uint8_t)~((1u << COM1B1) | (1u << COM1B0))) | (1u << COM1B1));
    }
}

void hal_pwm1_disable_channel(hal_pwm1_ch_t ch)
{
    if (ch == HAL_PWM1_CH_A)
    {
        /* Disconnect OC1A: COM1A1=0 COM1A0=0 */
        TCCR1A = (uint8_t)(TCCR1A & (uint8_t)~((1u << COM1A1) | (1u << COM1A0)));
    }
    else
    {
        /* Disconnect OC1B: COM1B1=0 COM1B0=0 */
        TCCR1A = (uint8_t)(TCCR1A & (uint8_t)~((1u << COM1B1) | (1u << COM1B0)));
    }
}

void hal_pwm1_set_duty_promille(hal_pwm1_ch_t ch, uint16_t promille)
{
    if (promille > 1000u) promille = 1000u;

    /* OCR = promille/1000 * (TOP+1), clamp to TOP */
    uint16_t top = ICR1;
    uint32_t ocr = ((uint32_t)promille * (uint32_t)(top + 1u)) / 1000u;
    if (ocr > (uint32_t)top) ocr = (uint32_t)top;

    if (ch == HAL_PWM1_CH_A)
    {
        hal_pwm1_write_ocr1a((uint16_t)ocr);
    }
    else
    {
        hal_pwm1_write_ocr1b((uint16_t)ocr);
    }
}
