/*
	Minimal Arduino/AVR stub environment so the RS485 code paths can be
	compiled and exercised with a host C++ compiler. Only what the code
	under test actually uses is provided.
*/
#ifndef _DCSBIOS_TEST_ARDUINO_STUB_H_
#define _DCSBIOS_TEST_ARDUINO_STUB_H_

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

// --- fake clock, controlled by the tests ---
extern unsigned long fake_micros;
inline unsigned long micros() { return fake_micros; }
inline unsigned long millis() { return fake_micros / 1000; }

// --- interrupt control (single-threaded tests: no-ops) ---
inline void interrupts() {}
inline void noInterrupts() {}

// --- digital I/O ---
#define LOW 0
#define HIGH 1
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline int digitalRead(uint8_t) { return HIGH; }
inline int analogRead(uint8_t) { return 0; }

// --- port access used for the TXENABLE pin ---
extern volatile uint8_t fake_txen_port;
inline volatile uint8_t* portOutputRegister(uint8_t) { return &fake_txen_port; }
inline uint8_t digitalPinToPort(uint8_t) { return 0; }
inline uint8_t digitalPinToBitMask(uint8_t) { return 1; }

// --- AVR USART registers (ATmega328P/2560 bit positions) ---
extern volatile uint8_t UDR0, UCSR0A, UCSR0B, UCSR0C, UBRR0H, UBRR0L;
extern volatile uint8_t UDR1, UCSR1A, UCSR1B, UCSR1C, UBRR1H, UBRR1L;
extern volatile uint8_t UDR2, UCSR2A, UCSR2B, UCSR2C, UBRR2H, UBRR2L;
extern volatile uint8_t UDR3, UCSR3A, UCSR3B, UCSR3C, UBRR3H, UBRR3L;
extern volatile uint8_t PRR0, PRR1;
extern volatile uint8_t PORTH, PORTB;

#define RXC0 7
#define TXC0 6
#define UDRE0 5
#define RXCIE0 7
#define TXCIE0 6
#define UDRIE0 5
#define RXEN0 4
#define TXEN0 3
#define UCSZ01 2
#define UCSZ00 1
#define PRUSART0 1
#define PRUSART1 0
#define PRUSART2 4
#define PRUSART3 2

#define _SFR_BYTE(sfr) (sfr)
#define _BV(bit) (1 << (bit))

// Interrupt handlers become plain functions the tests can call.
#define ISR(vector) void vector()

#ifndef NULL
#define NULL 0
#endif

#endif
