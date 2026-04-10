/**
 * @file avr_stdint.h
 * @brief Exact-width integer typedefs for the AVR toolchain used by the
 *        project.
 */

#ifndef AVR_STDINT_H_
#define AVR_STDINT_H_

/**
 * @brief AVR ATmega328P / avr-gcc data-model assumptions.
 *
 * - `char`: 8-bit
 * - `int`: 16-bit
 * - `long`: 32-bit
 * - `long long`: 64-bit
 * - pointer: 16-bit near pointer
 */

/* Exact-width integer types */
typedef signed char         int8_t;
typedef unsigned char       uint8_t;

typedef signed int          int16_t;
typedef unsigned int        uint16_t;

typedef signed long         int32_t;
typedef unsigned long       uint32_t;

typedef signed long long    int64_t;
typedef unsigned long long  uint64_t;

#endif /* AVR_STDINT_H_ */
