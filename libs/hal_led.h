#ifndef HAL_LED_H_
#define HAL_LED_H_

/**
 * @file hal_led.h
 * @brief Board LED helper API.
 */

#define LED_DDR  DDRB
#define LED_PORT PORTB
#define LED_PIN  (1 << 5)

/** @brief Configure the board LED GPIO. */
void LED_Init(void);
/** @brief Drive the board LED on. */
void LED_On(void);
/** @brief Drive the board LED off. */
void LED_Off(void);
/** @brief Toggle the board LED state. */
void LED_Toggle(void);

/** @brief Low-level delay loop helper used by the LED code path. */
void delay_loop_1(unsigned long count);

#endif
