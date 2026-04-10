/**
 * @file hal_time2.h
 * @brief Timer2-based millisecond timebase and profiling stamp API.
 *
 * Configuration:
 * - CTC mode
 * - `OCR2A = 249`
 * - prescaler `= 64`
 * - ATmega328P @ `16 MHz` gives a `4 us` timer tick and `1 ms` compare period
 */


#ifndef HAL_TIME2_H_
#define HAL_TIME2_H_

#include <avr/io.h>
#include "avr_stdint.h"

/** @brief Atomic Timer2 timestamp consisting of milliseconds plus raw `TCNT2`. */
typedef struct
{
    uint32_t ms;   /* millisecond counter */
    uint8_t  t2;   /* Timer2 counter (0..OCR2A) */
} hal_time2_stamp_t;

/** @brief Initialize Timer2 as a `1 ms` system tick. */
void hal_time2_init_1ms(void);

/** @brief Atomically read the millisecond counter. */
uint32_t hal_time2_ms(void);

/** @brief Atomically sample both the millisecond counter and `TCNT2`. */
hal_time2_stamp_t hal_time2_now(void);

/**
 * @brief Compute elapsed microseconds between two Timer2 stamps.
 *
 * Resolution is `4 us`.
 *
 * @param[in] a Earlier timestamp.
 * @param[in] b Later timestamp.
 * @return Elapsed time in microseconds.
 */
uint32_t hal_time2_elapsed_us(hal_time2_stamp_t a, hal_time2_stamp_t b);

#endif /* HAL_TIME2_H_ */
