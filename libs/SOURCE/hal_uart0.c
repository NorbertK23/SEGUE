/*
 * uart_rx_ISR.c
 *
 *  Created on: 16 Feb 2026
 *  Author: Norbert Kania
 *  Credits: Baldur Thorgilsson's: BaldursATmegaGuide
 *  UART0 driver for ATmega328P
 *  RX ring buffer with RX ISR
 *- TX ring buffer with UDRE ISR
 */


#include "hal_uart0.h"

/* -------------------------------------------------------------------------- */
/* RX ring buffer                                                             */
/* -------------------------------------------------------------------------- */

/* Capacity is UART0_RX_BUFSIZE - 1 bytes */
static volatile uint8_t rx_buf[UART0_RX_BUFSIZE];
static volatile uint8_t rx_in  = 0u;
static volatile uint8_t rx_out = 0u;

/* -------------------------------------------------------------------------- */
/* TX ring buffer                                                             */
/* -------------------------------------------------------------------------- */

/* Capacity is UART0_TX_BUFSIZE - 1 bytes */
static volatile uint8_t tx_buf[UART0_TX_BUFSIZE];
static volatile uint8_t tx_in  = 0u;
static volatile uint8_t tx_out = 0u;

/* -------------------------------------------------------------------------- */
/* Local helpers                                                              */
/* -------------------------------------------------------------------------- */

static inline uint8_t next_index_rx(uint8_t idx)
{
    idx++;
    if (idx >= (uint8_t)UART0_RX_BUFSIZE) {
        idx = 0u;
    }
    return idx;
}

static inline uint8_t next_index_tx(uint8_t idx)
{
    idx++;
    if (idx >= (uint8_t)UART0_TX_BUFSIZE) {
        idx = 0u;
    }
    return idx;
}

/* Unsafe helpers: use only with interrupts off or inside ISR */

static inline uint8_t rx_empty_unsafe(void)
{
    return (rx_in == rx_out) ? 1u : 0u;
}

static inline uint8_t rx_full_unsafe(void)
{
    return (next_index_rx(rx_in) == rx_out) ? 1u : 0u;
}

static inline uint8_t tx_empty_unsafe(void)
{
    return (tx_in == tx_out) ? 1u : 0u;
}

static inline uint8_t tx_full_unsafe(void)
{
    return (next_index_tx(tx_in) == tx_out) ? 1u : 0u;
}

static inline uint8_t tx_free_unsafe(void)
{
    uint8_t used;

    if (tx_in >= tx_out) {
        used = (uint8_t)(tx_in - tx_out);
    } else {
        used = (uint8_t)(UART0_TX_BUFSIZE - (uint8_t)(tx_out - tx_in));
    }

    return (uint8_t)((UART0_TX_BUFSIZE - 1u) - used);
}

/* -------------------------------------------------------------------------- */
/* Flush helpers                                                              */
/* -------------------------------------------------------------------------- */

void uart0_rx_flush(void)
{
    uint8_t s = SREG;
    cli();

    rx_in = 0u;
    rx_out = 0u;

    SREG = s;
}

//static void uart0_tx_reset_unsafe(void)
//{
//    tx_in = 0u;
//    tx_out = 0u;
//    UCSR0B &= (uint8_t)~(1u << UDRIE0);
//}

static void uart0_tx_kick_unsafe(void)
{
    /*
     * Enable UDRE interrupt.
     * If UDR is empty, ISR will fire and pull from TX ring.
     */
    UCSR0B |= (1u << UDRIE0);
}

void uart0_tx_flush(void)
{
    /*
     * Wait until:
     * 1. TX ring buffer is empty
     * 2. UDR is empty
     * 3. last frame finished sending (TXC0 set)
     *
     * TXC0 is cleared by writing 1 to it.
     */
    for (;;) {
        uint8_t done;
        uint8_t s = SREG;

        cli();
        done = 0u;

        if ((tx_in == tx_out) &&
            ((UCSR0A & (1u << UDRE0)) != 0u) &&
            ((UCSR0A & (1u << TXC0)) != 0u)) {
            done = 1u;
        }

        SREG = s;

        if (done != 0u) {
            break;
        }
    }
}

uint8_t uart0_tx_empty(void)
{
    uint8_t s;
    uint8_t empty;

    s = SREG;
    cli();

    empty = (tx_in == tx_out) ? 1u : 0u;

    SREG = s;
    return empty;
}

uint8_t uart0_tx_pending(void)
{
    uint8_t s;
    uint8_t in;
    uint8_t out;
    uint8_t count;

    s = SREG;
    cli();

    in  = tx_in;
    out = tx_out;

    SREG = s;

    if (in >= out) {
        count = (uint8_t)(in - out);
    } else {
        count = (uint8_t)(UART0_TX_BUFSIZE - (uint8_t)(out - in));
    }

    return count;
}

/* -------------------------------------------------------------------------- */
/* Init                                                                       */
/* -------------------------------------------------------------------------- */

void uart0_init_ubrr(uint16_t ubrr)
{
    uint8_t s = SREG;
    cli();

    rx_in = 0u;
    rx_out = 0u;
    tx_in = 0u;
    tx_out = 0u;

    /* Disable UART while reconfiguring */
    UCSR0B = 0u;

    /* Normal speed, clear TX complete */
    UCSR0A = (1u << TXC0);

    /* Set baud rate */
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr & 0xFF);

    /* 8N1 */
    UCSR0C = (1u << UCSZ01) | (1u << UCSZ00);

    /* Enable RX, TX, RX interrupt. TX interrupt stays off until needed. */
    UCSR0B = (1u << RXEN0) | (1u << TXEN0) | (1u << RXCIE0);

    SREG = s;
}

/* -------------------------------------------------------------------------- */
/* RX ISR and RX API                                                          */
/* -------------------------------------------------------------------------- */

ISR(USART_RX_vect)
{
    uint8_t c;

    /* Reading UDR0 clears RXC0 */
    c = UDR0;

    if (!rx_full_unsafe()) {
        rx_buf[rx_in] = c;
        rx_in = next_index_rx(rx_in);
    } else {
        /* Overflow: drop byte */
    }
}

uint8_t uart0_available(void)
{
    uint8_t s;
    uint8_t in;
    uint8_t out;

    s = SREG;
    cli();

    in  = rx_in;
    out = rx_out;

    SREG = s;

    if (in >= out) {
        return (uint8_t)(in - out);
    }

    return (uint8_t)(UART0_RX_BUFSIZE - (uint8_t)(out - in));
}

uint8_t uart0_getc(uint8_t *out)
{
    uint8_t got;
    uint8_t s;

    got = 0u;

    s = SREG;
    cli();

    if (!rx_empty_unsafe()) {
        *out = rx_buf[rx_out];
        rx_out = next_index_rx(rx_out);
        got = 1u;
    }

    SREG = s;
    return got;
}

/* -------------------------------------------------------------------------- */
/* TX ISR and TX API                                                          */
/* -------------------------------------------------------------------------- */

ISR(USART_UDRE_vect)
{
    if (!tx_empty_unsafe()) {
        /*
         * Clear TXC before writing new byte, so uart0_tx_flush()
         * can later observe completion of the final frame.
         */
        UCSR0A = (1u << TXC0);

        UDR0 = tx_buf[tx_out];
        tx_out = next_index_tx(tx_out);
    } else {
        /* Nothing left to send: stop UDRE interrupts */
        UCSR0B &= (uint8_t)~(1u << UDRIE0);
    }
}

uint8_t uart0_try_putc(uint8_t c)
{
    uint8_t ok;
    uint8_t s;

    ok = 0u;

    s = SREG;
    cli();

    if (!tx_full_unsafe()) {
        tx_buf[tx_in] = c;
        tx_in = next_index_tx(tx_in);
        uart0_tx_kick_unsafe();
        ok = 1u;
    }

    SREG = s;
    return ok;
}

void uart0_putc(uint8_t c)
{
    while (uart0_try_putc(c) == 0u) {
        ;
    }
}


uint8_t uart0_try_write(const uint8_t *data, uint8_t len)
{
    uint8_t ok;
    uint8_t s;
    uint8_t i;

    ok = 0u;

    s = SREG;
    cli();

    if (len <= tx_free_unsafe()) {
        for (i = 0u; i < len; i++) {
            tx_buf[tx_in] = data[i];
            tx_in = next_index_tx(tx_in);
        }

        uart0_tx_kick_unsafe();
        ok = 1u;
    }

    SREG = s;
    return ok;
}

void uart0_write(const uint8_t *data, uint8_t len)
{
    while (len != 0u) {
        if (uart0_try_write(data, len) != 0u) {
            break;
        }
    }
}


uint8_t uart0_try_puts(const char *s)
{
    uint8_t n;

    n = 0u;

    while (*s != 0) {
        if (uart0_try_putc((uint8_t)*s) == 0u) {
            break;
        }
        s++;
        n++;
    }

    return n;
}

void uart0_puts(const char *s)
{
    while (*s != 0) {
        uart0_putc((uint8_t)*s);
        s++;
    }
}
