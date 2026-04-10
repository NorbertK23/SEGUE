/*
 * delay.c
 *
 *  Created on: 9 Feb 2026
 *      Author: Norbert
 */

#include "avr_stdint.h"
#include <avr/io.h>
#include "util_delay.h"

static inline void _delay_loop_1(uint8_t __count) {
    __asm__ volatile (
        "1: dec %0" "\n\t"// decrement the count =1C
        "brne 1b" // branch to label '1' if result not zero t=2C f=1C
        : "=r" (__count)// output operand
        : "0" (__count)// input op.
    );
}


#define CALIBRATION_LOOPS  (21u)

void delay_us(uint16_t us)
{
	while (us--) {
		_delay_loop_1((uint8_t)4);
	}
}


void delay_ms(uint16_t ms)
{
    while (ms--) {
    	uint8_t i;
        for (i = 0; i < (uint8_t)CALIBRATION_LOOPS; i++) {
            _delay_loop_1((uint8_t)252);
        }
    }
}

void delay_s(uint16_t s)
{
    while (s--) {
        delay_ms((uint16_t)1000);
    }
}
