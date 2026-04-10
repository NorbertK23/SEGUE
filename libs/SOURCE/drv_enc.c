/*
 * drv_enc.c
 *
 *  Created on: 11 Mar 2026
 *      Author: Norbert Kania
 */

/*
 * MAC1 = D2
 * MAC2 = D4
 * MBC1 = D3
 * MBC2 = D5
 */


/*
 * DDRx &= ~(1 << n);
 * PORTx |=  (1 << n);
 * pin n = input_pullup
*/

#include <avr/io.h>
#include <avr/interrupt.h>

#include "drv_enc.h"
#include "avr_stdint.h"

#define MAC1 PD2
#define MAC2 PD4
#define MBC1 PD3
#define MBC2 PD5

volatile static int32_t countA = 0;
volatile static int32_t countB = 0;
volatile static uint32_t invalidA = 0;
volatile static uint32_t invalidB = 0;

volatile static uint8_t prev_MA;
volatile static uint8_t prev_MB;


static void enc_ISR_init()
{
	/*
	 * PCICR = Pin Change Interrupt Control Register
	 * PCIE2 = enable pin-change interrupt group 2 (PORTD)
	 */
	PCICR |= (1u << PCIE2);
	/*
	 * PCMSK2 = Pin Change Mask Register for group 2
	 * this chooses which individual pins inside port D are allowed to trigger
	 */
	PCMSK2 |= (1u << MAC1) | (1u << MAC2) | (1u << MBC1) | (1u << MBC2);
	/*
	 * PCIFR = Pin Change Interrupt Flag Register
	 * PCIF2 = pending flag for group 2
	 * writing 1 clears that flag on AVR
	 */
	PCIFR |= (1u << PCIF2);
}


static uint8_t read_MA_PIND(uint8_t pind)
{
	return ((pind>>(MAC1-1))&2u) | ((pind>>(MAC2))&1u);;
}

static uint8_t read_MB_PIND(uint8_t pind)
{
	return ((pind>>(MBC1-1))&2u) | ((pind>>(MBC2))&1u);
}
//	 *A=0, B=0 → 0
//	 *A=0, B=1 → 1
//	 *A=1, B=0 → 2
//	 *A=1, B=1 → 3

static int8_t decoder(uint8_t prev, uint8_t curr)
/*
 * 0 = no change
 * +1 = one valid step in one direction
 * -1 = one valid step in the other direction
 * 2 = invalid jump
 */
{
	if (prev == curr) return 0;

	if (prev == 0)
	{
		if (curr == 2) return 1;
		if (curr == 1) return -1;
		if (curr == 3) return 2;
	}
	if (prev == 2)
	{
		if (curr == 3) return 1;
		if (curr == 0) return -1;
		if (curr == 1) return 2;
	}
	if (prev == 3)
	{
		if (curr == 1) return 1;
		if (curr == 2) return -1;
		if (curr == 0) return 2;
	}
	if (prev == 1)
	{
		if (curr == 0) return 1;
		if (curr == 3) return -1;
		if (curr == 2) return 2;
	}
	return 2; // if undefined
}

static void encoder_state_update(uint8_t curr, volatile uint8_t *prev, volatile int32_t *count, volatile uint32_t *invalid)
{
	int8_t step;
	step = decoder(*prev,curr);
	if (step == 2)
	{
		*invalid += 1;
	}
	else if (step != 0) // only +-1 adds
	{
		*count += step;
	}


	*prev = curr;

}

ISR(PCINT2_vect)
{
    uint8_t curr_pind;
    uint8_t MA;
	uint8_t MB;

    curr_pind = PIND;
    MA = read_MA_PIND(curr_pind);
    MB = read_MB_PIND(curr_pind);
    encoder_state_update(MA, &prev_MA, &countA, &invalidA);
    encoder_state_update(MB, &prev_MB, &countB, &invalidB);
}

void drv_enc_init(void)
{
	// Set the PD2-PD5 to inputs
	DDRD &= ~ ( (1u<<MAC1) | (1u<<MAC2) | (1u<<MBC1) | (1u<<MBC2));
	// turn on internal resistors
	PORTD |= (1u<<MAC1) | (1u<<MAC2) | (1u<<MBC1) | (1u<<MBC2);

	uint8_t curr_pind = PIND;
	prev_MA = read_MA_PIND(curr_pind);
	prev_MB = read_MB_PIND(curr_pind);
	enc_ISR_init();
}

int32_t drv_enc_get_count_a(void)
{
    uint8_t sreg;
    int32_t value;

    sreg = SREG;
    cli();
    value = countA;
    SREG = sreg;

    return value;
}

int32_t drv_enc_get_count_b(void)
{
    uint8_t sreg;
    int32_t value;

    sreg = SREG;
    cli();
    value = countB;
    SREG = sreg;

    return value;
}

uint32_t drv_enc_get_invalid_a(void)
{
    uint8_t sreg;
    uint32_t value;

    sreg = SREG;
    cli();
    value = invalidA;
    SREG = sreg;

    return value;
}

uint32_t drv_enc_get_invalid_b(void)
{
    uint8_t sreg;
    uint32_t value;

    sreg = SREG;
    cli();
    value = invalidB;
    SREG = sreg;

    return value;
}

void drv_enc_reset(void)
{
    uint8_t sreg;

    sreg = SREG;
    cli();

    countA = 0;
    countB = 0;
    invalidA = 0;
    invalidB = 0;
	uint8_t curr_pind = PIND;
	prev_MA = read_MA_PIND(curr_pind);
	prev_MB = read_MB_PIND(curr_pind);

    SREG = sreg;
}
