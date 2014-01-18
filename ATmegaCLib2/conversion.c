/* *****************************************************************************
 * conversion.c
 * See atmegaclib2.h header for the copyrights...
 * *****************************************************************************
 */
#ifndef F_CPU
#define F_CPU 16000000UL //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include "atmegaclib2.h"

uint8_t bcd2bin(uint8_t bcd) {
#ifdef OPTIMIZE_SPEED
	return (10 * (bcd >> 4) | (bcd & 0x0f));
#else
	uint8_t Temp = bcd & 0x0F;
	while (bcd >= 0x10) {
		Temp += 10;
		bcd -= 0x10;
	}
	return Temp;
#endif
}

uint8_t bin2bcd(uint8_t bin) {
#ifdef OPTIMIZE_SPEED
	return (((bin / 10) << 4) | (bin % 10));
#else
	uint8_t Temp = 0;
	while (bin > 9) {
		Temp += 0x10;
		bin -= 10;
	}
	return Temp + bin;
#endif
}

uint8_t nibble2hex(uint8_t val) {
	uint8_t s;
	s = '0' + (val & 0xf);
	if (s > '9')
		s += 'A' - '9' - 1;
	return s;
}

void byte2dec(uint8_t val, int8_t *s) {
	if (val > 99) {
		s[2] = '0' + (val % 10);
		val /= 10;
		s[1] = '0' + (val % 10);
		val /= 10;
		s[0] = '0' + val;
		//
		s[3] = 0;
	} else if (val > 9) {
		s[1] = '0' + (val % 10);
		val /= 10;
		s[0] = '0' + val;
		s[2] = 0;
		//
		//s[3] = 0;
	} else {
		s[0] = '0' + val;
		//s[2] = 0;
		s[1] = 0;
		//
		///s[3] = 0;
	}
}

void double2hex(uint32_t val, int8_t *s) {
	s[0] = nibble2hex(val >> 28);
	s[1] = nibble2hex(val >> 24);
	s[2] = nibble2hex(val >> 20);
	s[3] = nibble2hex(val >> 16);
	s[4] = nibble2hex(val >> 12);
	s[5] = nibble2hex(val >> 8);
	s[6] = nibble2hex(val >> 4);
	s[7] = nibble2hex(val);
	s[8] = 0;
}

void word2hex(uint16_t val, int8_t *s) {
	s[0] = nibble2hex(val >> 12);
	s[1] = nibble2hex(val >> 8);
	s[2] = nibble2hex(val >> 4);
	s[3] = nibble2hex(val);
	s[4] = 0;
}

void byte2hex(uint8_t val, int8_t *s) {
	s[0] = nibble2hex(val >> 4);
	s[1] = nibble2hex(val);
	s[2] = 0;
}



