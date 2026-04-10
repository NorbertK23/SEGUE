/**
 * @file util_delay.h
 * @brief Simple blocking busy-wait delay helpers.
 */

#ifndef UTIL_DELAY_H_
#define UTIL_DELAY_H_

#include "avr_stdint.h"

/** @brief Busy-wait for the requested number of microseconds. */
void delay_us(uint16_t us);

/** @brief Busy-wait for the requested number of milliseconds. */
void delay_ms(uint16_t ms);

/** @brief Busy-wait for the requested number of seconds. */
void delay_s(uint16_t s);

#endif /* UTIL_DELAY_H_ */
