/**
 * @file hal_twi.h
 * @brief Blocking AVR TWI/I2C helper API.
 *
 * This HAL layer exposes low-level master transactions and a few common
 * register helpers for 7-bit addressed devices.
 */

#ifndef HAL_TWI_H_
#define HAL_TWI_H_

#include <avr/io.h>
#include "avr_stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------- Configuration ------------------------------ */

/** @brief Loop-count timeout used while waiting for `TWINT`. */
#ifndef TWI_TIMEOUT_TICKS
#define TWI_TIMEOUT_TICKS  (60000u)
#endif

/** @brief Enable weak internal pullups on SDA/SCL during init. */
#ifndef TWI_ENABLE_INTERNAL_PULLUPS
#define TWI_ENABLE_INTERNAL_PULLUPS  (1u)
#endif

/** @brief Enable optional pre-init SCL pulse recovery. */
#ifndef TWI_ENABLE_BUS_RECOVERY
#define TWI_ENABLE_BUS_RECOVERY  (1u)
#endif

/* ------------------------------- Status codes ----------------------------- */

#define TWI_STATUS_MASK             0xF8

#define TWI_START_TX                0x08
#define TWI_REP_START_TX            0x10

#define TWI_SLA_W_ACK               0x18
#define TWI_SLA_W_NACK              0x20
#define TWI_DATA_TX_ACK             0x28
#define TWI_DATA_TX_NACK            0x30

#define TWI_ARB_LOST                0x38

#define TWI_SLA_R_ACK               0x40
#define TWI_SLA_R_NACK              0x48
#define TWI_DATA_RX_ACK             0x50
#define TWI_DATA_RX_NACK            0x58

/* ------------------------------- Return codes ----------------------------- */

typedef enum
{
    TWI_OK = 0,
	TWI_ERR_PARAM,
    TWI_ERR_TIMEOUT,
    TWI_ERR_STATUS,
    TWI_ERR_ARB_LOST,
    TWI_ERR_BUS
} twi_rc_t;

/**
 * @brief Result returned by TWI HAL operations.
 */
typedef struct
{
    twi_rc_t rc;        /* high-level return code */
    uint8_t  status;    /* masked TWSR status at failure */
} twi_result_t;

/**
 * @brief Initialize AVR TWI hardware for the requested SCL frequency.
 *
 * @param[in] scl_hz Target bus frequency in hertz.
 */
void twi_init(uint32_t scl_hz);

/** @brief Disable the TWI peripheral. */
void twi_disable(void);

/** @brief Attempt to recover a stuck bus by clocking SCL. */
void twi_bus_recover(void);

/** @brief Issue a START condition. */
twi_result_t twi_start(void);
/** @brief Issue a repeated START condition. */
twi_result_t twi_repeated_start(void);
/** @brief Issue a STOP condition. */
twi_result_t twi_stop(void);

/** @brief Send a 7-bit slave address with write direction. */
twi_result_t twi_send_sla_w(uint8_t addr7);
/** @brief Send a 7-bit slave address with read direction. */
twi_result_t twi_send_sla_r(uint8_t addr7);

/** @brief Write one data byte on the active transaction. */
twi_result_t twi_write_u8(uint8_t data);
/** @brief Read one byte and ACK it. */
twi_result_t twi_read_u8_ack(uint8_t *out);
/** @brief Read one byte and NACK it. */
twi_result_t twi_read_u8_nack(uint8_t *out);

/** @brief Probe whether a 7-bit address acknowledges on write. */
twi_result_t twi_probe(uint8_t addr7);

/**
 * @brief Write a one-byte register on a 7-bit addressed device.
 */
twi_result_t twi_write_reg_u8(uint8_t addr7, uint8_t reg, uint8_t value);

/**
 * @brief Read a one-byte register from a 7-bit addressed device.
 */
twi_result_t twi_read_reg_u8(uint8_t addr7, uint8_t reg, uint8_t *out);

/**
 * @brief Read a multi-byte register region from a 7-bit addressed device.
 */
twi_result_t twi_read_reg(uint8_t addr7, uint8_t reg, uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* HAL_TWI_H_ */
