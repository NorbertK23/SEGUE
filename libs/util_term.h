/**
 * @file util_term.h
 * @brief Lightweight UART terminal formatting helpers.
 */

#ifndef UTIL_TERM_H_
#define UTIL_TERM_H_

#include "avr_stdint.h"
/** @brief Initialize the terminal formatter on UART0. */
void term_init(uint32_t baud);

/** @brief Send one character. */
void term_putc(char c);

/** @brief Send a null-terminated ASCII string. */
void term_puts(const char *s);

/** @brief Send a CRLF newline sequence. */
void term_newline(void);

/** @brief Print an unsigned 32-bit integer in decimal. */
void term_put_u32(uint32_t v);

/** @brief Print a signed 32-bit integer in decimal. */
void term_put_i32(int32_t v);

/** @brief Print an unsigned 8-bit value in hexadecimal. */
void term_put_hex_u8(uint8_t v);

/** @brief Print an unsigned 16-bit value in hexadecimal. */
void term_put_hex_u16(uint16_t v);

/** @brief Print an unsigned 32-bit value in hexadecimal. */
void term_put_hex_u32(uint32_t v);

/** @brief Try to queue one character without blocking. */
uint8_t term_try_putc(char c);
/** @brief Try to queue a CRLF newline without blocking. */
uint8_t term_try_newline(void);

/** @brief Print a raw 4-digit hexadecimal value without prefix or spacing. */
void term_put_hex4_u16_raw(uint16_t v);
/** @brief Print a raw 8-digit hexadecimal value without prefix or spacing. */
void term_put_hex8_u32_raw(uint32_t v);

/** @brief Try to print a raw 4-digit hexadecimal value without blocking. */
uint8_t term_try_put_hex4_u16_raw(uint16_t v);
/** @brief Try to print a raw 8-digit hexadecimal value without blocking. */
uint8_t term_try_put_hex8_u32_raw(uint32_t v);


/** @brief Optional debug-output helper enabled by `TERM_ENABLE_DEBUG`. */
#ifdef TERM_ENABLE_DEBUG
void term_debug(const char *s);
#else
#define term_debug(x) ((void)0)
#endif

#endif /* UTIL_TERM_H_ */
