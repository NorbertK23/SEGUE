/*
 * LED.c
 *
 *  Created on: 2 Feb 2026
 *      Author: Norbert Kania
 */
#include <avr/io.h>
#include "hal_led.h"

static int toggle_count = 0; // "static" keeps it hidden from main.c

void LED_Init(void) {
    LED_DDR |= LED_PIN;
}

void LED_On(void) {
    LED_PORT |= LED_PIN;
}

void LED_Off(void) {
    LED_PORT &= ~LED_PIN;
}

void LED_Toggle(void) {
    int mask = LED_PIN;

    LED_PORT ^= mask;
    toggle_count++; // Update the global counter
}

void delay_loop_1(unsigned long count)
{
    while (count--) {
        __asm__ __volatile__("nop");
    }
}
