/**
 * @file drv_enc.h
 * @brief Low-level encoder driver API.
 */

#ifndef DRV_ENC_H_
#define DRV_ENC_H_

#include "avr_stdint.h"
//#include <avr/io.h>
//#include "avr_stdint.h"
//#include "util_term.h"
//#include "util_delay.h"

/** @brief Initialize the encoder driver hardware. */
void drv_enc_init(void);

/** @brief Get cumulative count for encoder A. */
int32_t drv_enc_get_count_a(void);

/** @brief Get cumulative count for encoder B. */
int32_t drv_enc_get_count_b(void);

/** @brief Get cumulative invalid-transition count for encoder A. */
uint32_t drv_enc_get_invalid_a(void);

/** @brief Get cumulative invalid-transition count for encoder B. */
uint32_t drv_enc_get_invalid_b(void);

/** @brief Reset driver-side encoder counters. */
void drv_enc_reset(void);

#endif /* DRV_ENC_H_ */
