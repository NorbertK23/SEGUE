/*
 * util_term.c
 *
 *  Created on: 23 Feb 2026
 *      Author: Norbert Kania
 *
 * Terminal helper layer on top of UART0.
 *
 * Design goals:
 * - keep human-readable helpers for boot / diagnostics
 * - add fast fixed-width hex helpers for telemetry
 * - batch small writes so one field is queued in one HAL call
 */

#include "util_term.h"
#include "hal_uart0.h"

/* -------------------------------------------------------------------------- */
/* Local helpers                                                              */
/* -------------------------------------------------------------------------- */

static const uint8_t g_hex_lut[16] =
{
    (uint8_t)'0', (uint8_t)'1', (uint8_t)'2', (uint8_t)'3',
    (uint8_t)'4', (uint8_t)'5', (uint8_t)'6', (uint8_t)'7',
    (uint8_t)'8', (uint8_t)'9', (uint8_t)'A', (uint8_t)'B',
    (uint8_t)'C', (uint8_t)'D', (uint8_t)'E', (uint8_t)'F'
};

static uint8_t u32_to_dec_buf(uint32_t v, uint8_t *buf)
{
    uint8_t pos;

    /* buf must have room for 10 chars */
    pos = 10u;

    do {
        uint32_t q;
        uint8_t rem;

        q = v / 10u;
        rem = (uint8_t)(v - (q * 10u));
        buf[--pos] = (uint8_t)('0' + rem);
        v = q;
    } while (v != 0u);

    return pos;
}

static void put_hex4_u16_to_buf(uint16_t v, uint8_t *buf)
{
    buf[0] = g_hex_lut[(uint8_t)((v >> 12) & 0x0Fu)];
    buf[1] = g_hex_lut[(uint8_t)((v >> 8)  & 0x0Fu)];
    buf[2] = g_hex_lut[(uint8_t)((v >> 4)  & 0x0Fu)];
    buf[3] = g_hex_lut[(uint8_t)((v >> 0)  & 0x0Fu)];
}

static void put_hex8_u32_to_buf(uint32_t v, uint8_t *buf)
{
    buf[0] = g_hex_lut[(uint8_t)((v >> 28) & 0x0Fu)];
    buf[1] = g_hex_lut[(uint8_t)((v >> 24) & 0x0Fu)];
    buf[2] = g_hex_lut[(uint8_t)((v >> 20) & 0x0Fu)];
    buf[3] = g_hex_lut[(uint8_t)((v >> 16) & 0x0Fu)];
    buf[4] = g_hex_lut[(uint8_t)((v >> 12) & 0x0Fu)];
    buf[5] = g_hex_lut[(uint8_t)((v >> 8)  & 0x0Fu)];
    buf[6] = g_hex_lut[(uint8_t)((v >> 4)  & 0x0Fu)];
    buf[7] = g_hex_lut[(uint8_t)((v >> 0)  & 0x0Fu)];
}

/* -------------------------------------------------------------------------- */
/* Init                                                                       */
/* -------------------------------------------------------------------------- */

void term_init(uint32_t baud)
{
    uart_init(baud);
}

/* -------------------------------------------------------------------------- */
/* Basic output                                                               */
/* -------------------------------------------------------------------------- */

void term_putc(char c)
{
    uart0_putc((uint8_t)c);
}

uint8_t term_try_putc(char c)
{
    return uart0_try_putc((uint8_t)c);
}

void term_puts(const char *s)
{
    uart0_puts(s);
}

void term_newline(void)
{
    static const uint8_t nl[2] = { (uint8_t)'\r', (uint8_t)'\n' };

    uart0_write(nl, 2u);
}

uint8_t term_try_newline(void)
{
    static const uint8_t nl[2] = { (uint8_t)'\r', (uint8_t)'\n' };

    return uart0_try_write(nl, 2u);
}

/* -------------------------------------------------------------------------- */
/* Decimal output                                                             */
/* -------------------------------------------------------------------------- */

void term_put_u32(uint32_t v)
{
    uint8_t buf[10];
    uint8_t pos;

    pos = u32_to_dec_buf(v, buf);
    uart0_write(&buf[pos], (uint8_t)(10u - pos));
}

void term_put_i32(int32_t v)
{
    uint8_t buf[11];
    uint8_t pos;
    uint32_t mag;

    if (v < 0) {
        /*
         * Two's-complement magnitude without signed overflow on INT32_MIN.
         */
        mag = ((uint32_t)(~(uint32_t)v)) + 1u;
        pos = u32_to_dec_buf(mag, &buf[1]);
        buf[pos] = (uint8_t)'-';
        uart0_write(&buf[pos], (uint8_t)(11u - pos));
    } else {
        mag = (uint32_t)v;
        pos = u32_to_dec_buf(mag, buf);
        uart0_write(&buf[pos], (uint8_t)(10u - pos));
    }
}

/* -------------------------------------------------------------------------- */
/* Hex output: human-readable prefixed                                        */
/* -------------------------------------------------------------------------- */

void term_put_hex_u8(uint8_t v)
{
    uint8_t buf[4];

    buf[0] = (uint8_t)'0';
    buf[1] = (uint8_t)'x';
    buf[2] = g_hex_lut[(uint8_t)((v >> 4) & 0x0Fu)];
    buf[3] = g_hex_lut[(uint8_t)((v >> 0) & 0x0Fu)];

    uart0_write(buf, 4u);
}

void term_put_hex_u16(uint16_t v)
{
    uint8_t buf[6];

    buf[0] = (uint8_t)'0';
    buf[1] = (uint8_t)'x';
    put_hex4_u16_to_buf(v, &buf[2]);

    uart0_write(buf, 6u);
}

void term_put_hex_u32(uint32_t v)
{
    uint8_t buf[10];

    buf[0] = (uint8_t)'0';
    buf[1] = (uint8_t)'x';
    put_hex8_u32_to_buf(v, &buf[2]);

    uart0_write(buf, 10u);
}

/* -------------------------------------------------------------------------- */
/* Hex output: raw fixed-width, no prefix                                     */
/* -------------------------------------------------------------------------- */

void term_put_hex4_u16_raw(uint16_t v)
{
    uint8_t buf[4];

    put_hex4_u16_to_buf(v, buf);
    uart0_write(buf, 4u);
}

void term_put_hex8_u32_raw(uint32_t v)
{
    uint8_t buf[8];

    put_hex8_u32_to_buf(v, buf);
    uart0_write(buf, 8u);
}

uint8_t term_try_put_hex4_u16_raw(uint16_t v)
{
    uint8_t buf[4];

    put_hex4_u16_to_buf(v, buf);
    return uart0_try_write(buf, 4u);
}

uint8_t term_try_put_hex8_u32_raw(uint32_t v)
{
    uint8_t buf[8];

    put_hex8_u32_to_buf(v, buf);
    return uart0_try_write(buf, 8u);
}

/* -------------------------------------------------------------------------- */
/* Debug                                                                      */
/* -------------------------------------------------------------------------- */

#ifdef TERM_ENABLE_DEBUG
void term_debug(const char *s)
{
    term_puts(s);
}
#endif
