/*
 * hal_time2.c
 *
 *  Created on: 5 Mar 2026
 *      Author: Norbert Kania
 * Timer2 timebase implementation for ATmega328P.
 */


#include <avr/io.h>
#include <avr/interrupt.h>

#include "avr_stdint.h"
#include "hal_time2.h"

/* 1 ms tick counter (incremented in ISR) */
static volatile uint32_t g_time2_ms = 0u;

ISR(TIMER2_COMPA_vect)
{
    g_time2_ms++;
}

void hal_time2_init_1ms(void)
{
    uint8_t s = SREG;
    cli();

    /* Stop Timer2 */
    TCCR2A = 0u;
    TCCR2B = 0u;
    TCNT2  = 0u;

    /* CTC mode: WGM21=1 */
    TCCR2A = (1u << WGM21);

    /* TOP = OCR2A => 250 counts => 1 ms at 250 kHz (16MHz/64) */
    OCR2A = 249u;

    /* Prescaler = 64: CS22=1, CS21=0, CS20=0 */
    TCCR2B = (1u << CS22);

    /* Enable compare match A interrupt */
    TIMSK2 = (1u << OCIE2A);

    /* Clear any pending flag */
    TIFR2 = (1u << OCF2A);

    SREG = s;
}

uint32_t hal_time2_ms(void)
{
    uint32_t v;
    uint8_t s = SREG;
    cli();
    v = g_time2_ms;
    SREG = s;
    return v;
}

hal_time2_stamp_t hal_time2_now(void)
{
    hal_time2_stamp_t t;
    uint8_t s = SREG;
    cli();
    t.ms = g_time2_ms;
    t.t2 = TCNT2;
    SREG = s;
    return t;
}

uint32_t hal_time2_elapsed_us(hal_time2_stamp_t a, hal_time2_stamp_t b)
{
    int16_t dt_ticks = (int16_t)b.t2 - (int16_t)a.t2;
    uint32_t dt_ms   = b.ms - a.ms;

    /* If we wrapped TCNT2 inside this interval, adjust */
    if (dt_ticks < 0) {
        dt_ticks += (int16_t)(OCR2A + 1u); /* +250 ticks */
        if (dt_ms > 0u) dt_ms -= 1u;
    }

    /* At 16MHz with prescaler 64: 1 tick = 4 us */
    return (dt_ms * 1000u) + ((uint32_t)dt_ticks * 4u);
}
