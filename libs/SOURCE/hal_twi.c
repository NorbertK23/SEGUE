/*
 * twi.c
 *
 *  Created on: 27 Feb 2026
 *      Author: Norbert Kania
 */

//#include <avr/io.h>
#include "hal_twi.h"
#include "util_delay.h"   /* for delay_us if bus recovery enabled */

static inline uint8_t twi_status(void)
{
    return (uint8_t)(TWSR & TWI_STATUS_MASK);
}

static inline twi_result_t twi_make(twi_rc_t rc, uint8_t st)
{
    twi_result_t r;
    r.rc = rc;
    r.status = st;
    return r;
}

static twi_result_t twi_wait_twint(void)
{
    uint32_t n = (uint32_t)TWI_TIMEOUT_TICKS;
    while ((TWCR & (1u << TWINT)) == 0u) {
        if (n-- == 0u) {
            return twi_make(TWI_ERR_TIMEOUT, twi_status());
        }
    }
    return twi_make(TWI_OK, twi_status());
}

void twi_disable(void)
{
    TWCR = 0u;
}

void twi_bus_recover(void)
{
#if TWI_ENABLE_BUS_RECOVERY
    /* Disable TWI so we can manipulate pins */
    twi_disable();

    /* Release SDA/SCL: inputs */
    DDRC &= (uint8_t)~((1u << PC4) | (1u << PC5));

#if TWI_ENABLE_INTERNAL_PULLUPS
    PORTC |= (1u << PC4) | (1u << PC5);
#else
    PORTC &= (uint8_t)~((1u << PC4) | (1u << PC5));
#endif

    delay_us(10);

    /* If SDA stuck low, pulse SCL up to 9 times */
    for (uint8_t i = 0; i < 9u; i++) {
        if (PINC & (1u << PC4)) {
            break; /* SDA released */
        }

        /* Drive SCL low */
        DDRC |= (1u << PC5);
        PORTC &= (uint8_t)~(1u << PC5);
        delay_us(5);

        /* Release SCL high */
        DDRC &= (uint8_t)~(1u << PC5);
        delay_us(5);
    }
#endif
}

void twi_init(uint32_t scl_hz)
{
#if TWI_ENABLE_BUS_RECOVERY
    twi_bus_recover();
#endif

    /* Ensure SDA/SCL not driven by us */
    DDRC &= (uint8_t)~((1u << PC4) | (1u << PC5));

#if TWI_ENABLE_INTERNAL_PULLUPS
    PORTC |= (1u << PC4) | (1u << PC5);
#endif

    /* Prescaler = 1 */
    TWSR = 0x00;

    if (scl_hz == 0u) {
        scl_hz = 100000u;
    }

    /* TWBR = ((F_CPU / SCL) - 16) / (2*prescaler) */
    {
        uint32_t twbr_calc = ((F_CPU / scl_hz) - 16u) / 2u;
        if (twbr_calc > 255u) twbr_calc = 255u;
        TWBR = (uint8_t)twbr_calc; // TWBR = 72 for 100 kHz;
    }

    /* Enable TWI */
    TWCR = (1u << TWEN);
}

/* ----------------------------- Transaction primitives ---------------------- */

twi_result_t twi_start(void)
{
    TWCR = (1u << TWINT) | (1u << TWSTA) | (1u << TWEN);
    twi_result_t w = twi_wait_twint();
    if (w.rc != TWI_OK) return w;

    uint8_t st = twi_status();
    if (st == TWI_START_TX) return twi_make(TWI_OK, st);
    if (st == TWI_ARB_LOST) return twi_make(TWI_ERR_ARB_LOST, st);
    return twi_make(TWI_ERR_STATUS, st);
}

twi_result_t twi_repeated_start(void)
{
    /* Same mechanism as start; status should be REP_START_TX */
    TWCR = (1u << TWINT) | (1u << TWSTA) | (1u << TWEN);
    twi_result_t w = twi_wait_twint();
    if (w.rc != TWI_OK) return w;

    uint8_t st = twi_status();
    if (st == TWI_REP_START_TX || st == TWI_START_TX) return twi_make(TWI_OK, st);
    if (st == TWI_ARB_LOST) return twi_make(TWI_ERR_ARB_LOST, st);
    return twi_make(TWI_ERR_STATUS, st);
}

twi_result_t twi_stop(void)
{
    TWCR = (1u << TWINT) | (1u << TWSTO) | (1u << TWEN);

    /* Waiting for TWSTO clear is optional; do bounded wait to avoid hangs */
    uint32_t n = (uint32_t)TWI_TIMEOUT_TICKS;
    while (TWCR & (1u << TWSTO)) {
        if (n-- == 0u) {
            return twi_make(TWI_ERR_TIMEOUT, twi_status());
        }
    }
    return twi_make(TWI_OK, twi_status());
}

/* ----------------------------- Address phase ------------------------------ */

twi_result_t twi_send_sla_w(uint8_t addr7)
{
    TWDR = (uint8_t)((addr7 << 1) | 0u);
    TWCR = (1u << TWINT) | (1u << TWEN);

    twi_result_t w = twi_wait_twint();
    if (w.rc != TWI_OK) return w;

    uint8_t st = twi_status();
    if (st == TWI_SLA_W_ACK) return twi_make(TWI_OK, st);
    if (st == TWI_SLA_W_NACK) return twi_make(TWI_ERR_STATUS, st);
    if (st == TWI_ARB_LOST)  return twi_make(TWI_ERR_ARB_LOST, st);
    return twi_make(TWI_ERR_STATUS, st);
}

twi_result_t twi_send_sla_r(uint8_t addr7)
{
    TWDR = (uint8_t)((addr7 << 1) | 1u);
    TWCR = (1u << TWINT) | (1u << TWEN);

    twi_result_t w = twi_wait_twint();
    if (w.rc != TWI_OK) return w;

    uint8_t st = twi_status();
    if (st == TWI_SLA_R_ACK) return twi_make(TWI_OK, st);
    if (st == TWI_SLA_R_NACK) return twi_make(TWI_ERR_STATUS, st);
    if (st == TWI_ARB_LOST)  return twi_make(TWI_ERR_ARB_LOST, st);
    return twi_make(TWI_ERR_STATUS, st);
}

/* ----------------------------- Data phase -------------------------------- */

twi_result_t twi_write_u8(uint8_t data)
{
    TWDR = data;
    TWCR = (1u << TWINT) | (1u << TWEN);

    twi_result_t w = twi_wait_twint();
    if (w.rc != TWI_OK) return w;

    uint8_t st = twi_status();
    if (st == TWI_DATA_TX_ACK) return twi_make(TWI_OK, st);
    if (st == TWI_DATA_TX_NACK) return twi_make(TWI_ERR_STATUS, st);
    if (st == TWI_ARB_LOST) return twi_make(TWI_ERR_ARB_LOST, st);
    return twi_make(TWI_ERR_STATUS, st);
}

twi_result_t twi_read_u8_ack(uint8_t *out)
{
    if (!out) return twi_make(TWI_ERR_BUS, twi_status());

    TWCR = (1u << TWINT) | (1u << TWEN) | (1u << TWEA);
    twi_result_t w = twi_wait_twint();
    if (w.rc != TWI_OK) return w;

    uint8_t st = twi_status();
    if (st != TWI_DATA_RX_ACK) return twi_make(TWI_ERR_STATUS, st);

    *out = TWDR;
    return twi_make(TWI_OK, st);
}

twi_result_t twi_read_u8_nack(uint8_t *out)
{
    if (!out) return twi_make(TWI_ERR_BUS, twi_status());

    TWCR = (1u << TWINT) | (1u << TWEN);
    twi_result_t w = twi_wait_twint();
    if (w.rc != TWI_OK) return w;

    uint8_t st = twi_status();
    if (st != TWI_DATA_RX_NACK) return twi_make(TWI_ERR_STATUS, st);

    *out = TWDR;
    return twi_make(TWI_OK, st);
}

/* ----------------------------- Convenience -------------------------------- */

twi_result_t twi_probe(uint8_t addr7)
{
    twi_result_t r;

    r = twi_start();
    if (r.rc != TWI_OK) { (void)twi_stop(); return r; }

    r = twi_send_sla_w(addr7);
    (void)twi_stop();
    return r;
}

twi_result_t twi_write_reg_u8(uint8_t addr7, uint8_t reg, uint8_t value)
{
    twi_result_t r;

    r = twi_start();
    if (r.rc != TWI_OK) { (void)twi_stop(); return r; }

    r = twi_send_sla_w(addr7);
    if (r.rc != TWI_OK) { (void)twi_stop(); return r; }

    r = twi_write_u8(reg);
    if (r.rc != TWI_OK) { (void)twi_stop(); return r; }

    r = twi_write_u8(value);
    (void)twi_stop();
    return r;
}

// SINGLE READ
twi_result_t twi_read_reg_u8(uint8_t addr7, uint8_t reg, uint8_t *out)
{
    twi_result_t r;

    r = twi_start();
    if (r.rc != TWI_OK) { (void)twi_stop(); return r; }

    r = twi_send_sla_w(addr7);
    if (r.rc != TWI_OK) { (void)twi_stop(); return r; }

    r = twi_write_u8(reg);
    if (r.rc != TWI_OK) { (void)twi_stop(); return r; }

    r = twi_repeated_start();
    if (r.rc != TWI_OK) { (void)twi_stop(); return r; }

    r = twi_send_sla_r(addr7);
    if (r.rc != TWI_OK) { (void)twi_stop(); return r; }

    r = twi_read_u8_nack(out);
    (void)twi_stop();
    return r;
}

// BURST READ
twi_result_t twi_read_reg(uint8_t addr7, uint8_t reg, uint8_t *buf, uint8_t len)
{
    twi_result_t r;

    if (!buf || len == 0u) return twi_make(TWI_ERR_BUS, twi_status());

    r = twi_start();
    if (r.rc != TWI_OK) { (void)twi_stop(); return r; }

    r = twi_send_sla_w(addr7);
    if (r.rc != TWI_OK) { (void)twi_stop(); return r; }

    r = twi_write_u8(reg);
    if (r.rc != TWI_OK) { (void)twi_stop(); return r; }

    r = twi_repeated_start();
    if (r.rc != TWI_OK) { (void)twi_stop(); return r; }

    r = twi_send_sla_r(addr7);
    if (r.rc != TWI_OK) { (void)twi_stop(); return r; }

    for (uint8_t i = 0; i < len; i++) {
        if (i + 1u < len) {
            r = twi_read_u8_ack(&buf[i]);
        } else {
            r = twi_read_u8_nack(&buf[i]);
        }
        if (r.rc != TWI_OK) { (void)twi_stop(); return r; }
    }

    (void)twi_stop();
    return twi_make(TWI_OK, twi_status());
}
