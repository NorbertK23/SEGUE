/**
 * @file svc_enc.h
 * @brief Encoder service-layer API.
 *
 * This layer sits above the low-level encoder driver and exposes counts,
 * per-cycle deltas, derived velocities, and invalid-transition counters.
 */

#ifndef SVC_ENC_H_
#define SVC_ENC_H_
#include "avr_stdint.h"

/** @brief Initialize the encoder service layer. */
void svc_enc_init(void);
/** @brief Reset service-layer encoder state. */
void svc_enc_reset(void);
/** @brief Update service-layer encoder state for the current cycle. */
void svc_enc_update(void);

/** @brief Get service-layer cumulative count for encoder A. */
int32_t svc_enc_get_count_a(void);
/** @brief Get service-layer cumulative count for encoder B. */
int32_t svc_enc_get_count_b(void);

/** @brief Get most recent count delta for encoder A. */
int32_t svc_enc_get_delta_a(void);
/** @brief Get most recent count delta for encoder B. */
int32_t svc_enc_get_delta_b(void);

/** @brief Get derived velocity for encoder A in counts per second. */
int32_t svc_enc_get_vel_a_cps(void);
/** @brief Get derived velocity for encoder B in counts per second. */
int32_t svc_enc_get_vel_b_cps(void);

/** @brief Get cumulative invalid-transition count for encoder A. */
uint32_t svc_enc_get_invalid_a(void);
/** @brief Get cumulative invalid-transition count for encoder B. */
uint32_t svc_enc_get_invalid_b(void);

#endif /* SVC_ENC_H_ */
