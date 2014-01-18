/* *****************************************************************************
 * serial_common.c
 * See atmegaclib2.h header for the copyrights...
 * *****************************************************************************
 */
#ifndef F_CPU
#define F_CPU 16000000UL //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include "atmegaclib2.h"
#include <stdlib.h>
//
void serial_putchar(uint8_t data) {
	/* Wait for empty transmit buffer */
#if defined(USART_RXC_vect) // added for ATmega16A
	while (!(UCSRA & (1 << UDRE)))
	;
	/* Start transmission */
	UDR = data;
#else
	while (!(UCSR0A & (1 << UDRE0)))
		;
	/* Start transmission */
	UDR0 = data;
#endif
}

void serial_putstr(int8_t * s) {
	register uint8_t c;
	while ((c = *s++))
		serial_putchar(c);
}

// Example:
//  uint8_t const pgm_str1[] PROGMEM =  "!Smiley!\0";
//  uint8_t const pgm_str2[] PROGMEM =  "!Micros!\0";
//  serial_putstr_f((int8_t *)pgm_str1);
//  serial_putstr_f((int8_t *)pgm_str2);
void serial_putstr_f(int8_t *s) {
	register uint8_t c;
	while ((c = pgm_read_byte(s++)))
		serial_putchar(c);
}

// NOT working ...
//void serial_putstr_e(uint8_t *s)
//  {
//	  register uint8_t c;
//	  while ((c = eeprom_read_byte(s++)))
//		  serial_putchar(c);
//  }

void serial_putint(int value) {
	int8_t string[18];
	itoa(value, (char *) string, 10);
	serial_putstr(string);
}

void serial_putU08(uint8_t value) {
	int8_t s[4];
	byte2dec(value, s);
	serial_putstr(s);
}

void serial_puthexU08(uint8_t value) {
	int8_t s[3];
	byte2hex(value, s);
	serial_putstr(s);
}

void serial_puthexU16(uint16_t value) {
	int8_t s[5];
	word2hex(value, s);
	serial_putstr(s);
}

void serial_puthexU32(uint32_t value) {
	int8_t s[9];
	double2hex(value, s);
	serial_putstr(s);
}


