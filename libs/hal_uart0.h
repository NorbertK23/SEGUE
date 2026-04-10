/**
 * @file hal_uart0.h
 * @brief Interrupt-driven UART0 TX/RX ring-buffer API.
 */

#ifndef HAL_UART0_H_
#define HAL_UART0_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include "avr_stdint.h"

#ifndef F_CPU
# error "F_CPU must be defined (e.g. -DF_CPU=16000000UL)"
#endif

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

#ifndef UART0_RX_BUFSIZE
#define UART0_RX_BUFSIZE 256u
#endif

#ifndef UART0_TX_BUFSIZE
#define UART0_TX_BUFSIZE 128u
#endif

/** @brief Compile-time UBRR calculation for normal-speed UART mode. */
#define UART0_UBRR_FROM_BAUD(baud_) \
    ((uint16_t)(((F_CPU) + 8UL*(uint32_t)(baud_)) / (16UL*(uint32_t)(baud_)) - 1UL))

#define uart_init(baud_) uart0_init_ubrr(UART0_UBRR_FROM_BAUD(baud_))

/** @brief Initialize UART0 from a precomputed UBRR value. */
void uart0_init_ubrr(uint16_t ubrr);

/* -------------------------------------------------------------------------- */
/* RX API                                                                     */
/* -------------------------------------------------------------------------- */

/** @brief Return nonzero if at least one RX byte is available. */
uint8_t uart0_available(void);
/** @brief Read one RX byte if available. */
uint8_t uart0_getc(uint8_t *out);
/** @brief Discard all queued RX data. */
void uart0_rx_flush(void);

/* -------------------------------------------------------------------------- */
/* TX API                                                                     */
/* -------------------------------------------------------------------------- */

/** @brief Queue one TX byte, blocking until space is available. */
void uart0_putc(uint8_t c);

/** @brief Try to queue one TX byte without blocking. */
uint8_t uart0_try_putc(uint8_t c);

/** @brief Queue a null-terminated string, blocking as needed. */
void uart0_puts(const char *s);

/** @brief Try to queue as much of a string as fits without blocking. */
uint8_t uart0_try_puts(const char *s);

/** @brief Return the number of queued TX bytes. */
uint8_t uart0_tx_pending(void);

/**
 * @brief Return nonzero if no queued TX bytes remain pending.
 *
 * This does not guarantee the last stop bit has fully left the wire. Use
 * ::uart0_tx_flush for that.
 */
uint8_t uart0_tx_empty(void);

/** @brief Block until all queued TX bytes have fully transmitted. */
void uart0_tx_flush(void);

/** @brief Try to queue a raw byte span without blocking. */
uint8_t uart0_try_write(const uint8_t *data, uint8_t len);

/** @brief Queue a raw byte span, blocking as needed. */
void uart0_write(const uint8_t *data, uint8_t len);

#endif /* HAL_UART0_H_ */
