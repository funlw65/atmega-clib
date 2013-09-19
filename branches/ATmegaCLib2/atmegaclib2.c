/* *****************************************************************************
 *  BSD License
 *  ATmega-CLib - a BSD library for using GNU toolchain mainly with EvB4.3 and
 *                EvB5.1 boards, but also Arduino, Sanguino, etc...
 *  Portions Copyright:
 *  - (c) 1998 Wouter van Ooijen, http://www.voti.nl/winkel/index.html
 *  - (c) 2004 Robert Krysztof, website's gone
 *  - (c) 2007 Stefan Engelke, http://www.tinkerer.eu/AVRLib/nRF24L01
 *  - (c) 2009 Michael Spiceland, https://code.google.com/p/libarduino/
 *  - (c) 2009 Joep Suijs, http://www.blogger.com/profile/06821529393453332522
 *  - (c) 2009 Vasile Surducan, http://vsurducan.blogspot.com/
 *  - (c) 2010 Bert van Dam, http://members.home.nl/b.vandam/lonely/index.html
 *  - (c) 2010 Paul Stoffregen, http://www.pjrc.com/teensy/td_libs_OneWire.html
 *  - (c) 2010 Chennai Dharmani, http://www.dharmanitech.com
 *  - (c) 2011 Joe Pardue, http://code.google.com/p/avrtoolbox/
 *  - (c) 2011 Martin Thomas, http://www.siwawi.arubi.uni-kl.de/avr-projects/
 *  - (c) 2011 PJRC.COM, LLC - Paul Stoffregen, http://www.pjrc.com/
 *  - (c) 2011 ChaN, http://elm-chan.org/fsw/strf/xprintf.html
 *  - (c) xxxx Aleksander Mielczarek, http://olek.tk/en/rfm12.php
 *  - (c) xxxx J C Wippler, https://github.com/jcw/jeelib
 *  - (c) 2012 Hans-Gert Dahmen, http://www.das-labor.org/wiki/RFM12_library/en
 *  - (c) 2012 Peter Fuhrmann, http://www.das-labor.org/wiki/RFM12_library/en
 *  - (c) 2012 Soeren Heisrath, http://www.das-labor.org/wiki/RFM12_library/en
 *  - (c) 2012 Vasile Guta Ciucur, https://sites.google.com/site/funlw65/
 *  - (c) xxxx Fabian Maximilian Thiele, website's gone
 *
 *******************************************************************************
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  - Neither the name of Vasile Guta Ciucur nor the names of
 *    its contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef F_CPU
#define F_CPU 16000000UL //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <util/crc16.h>
#include "atmegaclib2.h"
#ifdef ENABLE_IR
#include "irkeys.h"
#endif
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#ifdef ENABLE_ONE_WIRE
#include <util/atomic.h>
#endif

/* stuff used in all modes */
inline void onboard_led_enable(void) {
#if defined(__AVR_ATmega16__)      || \
	defined(__AVR_ATmega16A__)     || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega32A__)     || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
	sbi(DDRB, 0);
	// DEBUG LED, used also for bootloader
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
	sbi(DDRB, 7);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)   // Arduino 28 pins
	sbi(DDRB, 5);
#endif
}

inline void onboard_led_on(void) {
#if defined(__AVR_ATmega16__)      || \
	defined(__AVR_ATmega16A__)     || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega32A__)     || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
	sbi(PORTB, 0);
	// DEBUG LED, used also for bootloader
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
	sbi(PORTB, 7);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
	sbi(PORTB, 5);
#endif
}

inline void onboard_led_off(void) {
#if defined(__AVR_ATmega16__)      || \
	defined(__AVR_ATmega16A__)     || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega32A__)     || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
	cbi(PORTB, 0);
	// DEBUG LED, used also for bootloader
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
	cbi(PORTB, 7);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
	cbi(PORTB, 5);
#endif
}

inline void onboard_led_toggle(void) {
#if defined(__AVR_ATmega16__)      || \
	defined(__AVR_ATmega16A__)     || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega32A__)     || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
	tbi(PORTB, 0);
	// DEBUG LED, used also for bootloader
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
	tbi(PORTB, 7);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
	tbi(PORTB, 5);
#endif
}
//--
#ifdef ENABLE_MILLIS
#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )
// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile uint32_t timer0_overflow_count = 0;
volatile uint32_t timer0_millis = 0;
static uint8_t timer0_fract = 0;

//#if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
//ISR(TIM0_OVF_vect)
//#else
ISR(TIMER0_OVF_vect)
//#endif
{
	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	uint32_t m = timer0_millis;
	uint8_t f = timer0_fract;

	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}

	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}

void millis_init(void) {
#if defined(TCCR0A) && defined(WGM01)
	sbi(TCCR0A, WGM01);
	sbi(TCCR0A, WGM00);
#endif
	// set timer 0 prescale factor to 64
#if defined(__AVR_ATmega128__)
	// CPU specific: different values for the ATmega128
	sbi(TCCR0, CS02);
#elif defined(TCCR0) && defined(CS01) && defined(CS00)
	// this combination is for the standard atmega8
	sbi(TCCR0, CS01);
	sbi(TCCR0, CS00);
#elif defined(TCCR0B) && defined(CS01) && defined(CS00)
	// this combination is for the standard 168/328/1280/2560
	sbi(TCCR0B, CS01);
	sbi(TCCR0B, CS00);
#elif defined(TCCR0A) && defined(CS01) && defined(CS00)
	// this combination is for the __AVR_ATmega645__ series
	sbi(TCCR0A, CS01);
	sbi(TCCR0A, CS00);
#else
#error Timer 0 prescale factor 64 not set correctly
#endif
	// enable timer 0 overflow interrupt
#if defined(TIMSK) && defined(TOIE0)
	sbi(TIMSK, TOIE0);
#elif defined(TIMSK0) && defined(TOIE0)
	sbi(TIMSK0, TOIE0);
#else
#error	Timer 0 overflow interrupt not set correctly
#endif
}

uint32_t millis() {
	uint32_t m;
	uint8_t oldSREG = SREG;

	// disable interrupts while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_millis)
	cli();
	m = timer0_millis;
	SREG = oldSREG;
	return m;
}
#endif
//--
#ifdef ENABLE_NB_DELAYS
#define DELAY_SLOTS  3 // the number of non-blocking delays we need
int16_t isr_countdowns[DELAY_SLOTS]; // the array where the "delays" are stored

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )
// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile uint32_t timer0_overflow_count = 0;
static uint8_t timer0_fract = 0;

#if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
ISR(TIM0_OVF_vect)
#else
ISR(TIMER0_OVF_vect)
#endif
{
	uint8_t f = timer0_fract, i, j;
	i = 1;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		i = 2;
	}
	timer0_fract = f;
	timer0_overflow_count++;
	for (j = 0; j < DELAY_SLOTS; j++) {
		if (isr_countdowns[j] > 0) {
			//
			isr_countdowns[j] = isr_countdowns[j] - i;
		}
	}
}

void timer0_isr_init(void) {
#if defined(TCCR0A) && defined(WGM01)
	sbi(TCCR0A, WGM01);
	sbi(TCCR0A, WGM00);
#endif
	// set timer 0 prescale factor to 64
#if defined(__AVR_ATmega128__)
	// CPU specific: different values for the ATmega128
	sbi(TCCR0, CS02);
#elif defined(TCCR0) && defined(CS01) && defined(CS00)
	// this combination is for the standard atmega8
	sbi(TCCR0, CS01);
	sbi(TCCR0, CS00);
#elif defined(TCCR0B) && defined(CS01) && defined(CS00)
	// this combination is for the standard 168/328/1280/2560
	sbi(TCCR0B, CS01);
	sbi(TCCR0B, CS00);
#elif defined(TCCR0A) && defined(CS01) && defined(CS00)
	// this combination is for the __AVR_ATmega645__ series
	sbi(TCCR0A, CS01);
	sbi(TCCR0A, CS00);
#else
#error Timer 0 prescale factor 64 not set correctly
#endif
	// enable timer 0 overflow interrupt
#if defined(TIMSK) && defined(TOIE0)
	sbi(TIMSK, TOIE0);
#elif defined(TIMSK0) && defined(TOIE0)
	sbi(TIMSK0, TOIE0);
#else
#error	Timer 0 overflow interrupt not set correctly
#endif
}

// check if the time in a specific slot expired
uint8_t check_delay(uint8_t slot) {
	if (slot >= DELAY_SLOTS)
	return TRUE; //protection against user input error
	uint8_t oldSREG = SREG;
	cli();
	if (isr_countdowns[slot] <= 0) {
		SREG = oldSREG;
		return TRUE;
	}
	SREG = oldSREG;
	return FALSE;
}

//set the duration of a delay (in milliseconds) in a specific slot
void set_delay(uint8_t slot, uint16_t ms_time) {
	if (slot >= DELAY_SLOTS)
	return; //protection against user input error
	uint8_t oldSREG = SREG;
	cli();
	isr_countdowns[slot] = ms_time;
	SREG = oldSREG;
}
#endif //ENABLE_NB_DELAYS
//--
#ifdef ENABLE_FREQMEASURE
#define FREQMEASURE_BUFFER_LEN 12
volatile uint32_t buffer_value[FREQMEASURE_BUFFER_LEN];
volatile uint8_t buffer_head;
volatile uint8_t buffer_tail;
uint16_t capture_msw;
uint32_t capture_previous;

void FreqMeasure_begin(void) {
	capture_init();
	capture_msw = 0;
	capture_previous = 0;
	buffer_head = 0;
	buffer_tail = 0;
	capture_start();
}

uint8_t FreqMeasure_available(void) {
	uint8_t head, tail;

	head = buffer_head;
	tail = buffer_tail;
	if (head >= tail)
	return head - tail;
	return FREQMEASURE_BUFFER_LEN + head - tail;
}

uint32_t FreqMeasure_read(void) {
	uint8_t head, tail;
	uint32_t value;

	head = buffer_head;
	tail = buffer_tail;
	if (head == tail)
	return 0xFFFFFFFF;
	tail = tail + 1;
	if (tail >= FREQMEASURE_BUFFER_LEN)
	tail = 0;
	value = buffer_value[tail];
	buffer_tail = tail;
	return value;
}

void FreqMeasure_end(void) {
	capture_shutdown();
}

ISR(TIMER_OVERFLOW_VECTOR) {
	capture_msw++;
}

ISR(TIMER_CAPTURE_VECTOR) {
	uint16_t capture_lsw;
	uint32_t capture, period;
	uint8_t i;

	// get the timer capture
	capture_lsw = capture_read();
	// Handle the case where but capture and overflow interrupts were pending
	// (eg, interrupts were disabled for a while), or where the overflow occurred
	// while this ISR was starting up.  However, if we read a 16 bit number that
	// is very close to overflow, then ignore any overflow since it probably
	// just happened.
	if (capture_overflow() && capture_lsw < 0xFF00) {
		capture_overflow_reset();
		capture_msw++;
	}
	// compute the waveform period
	capture = ((uint32_t) capture_msw << 16) | capture_lsw;
	period = capture - capture_previous;
	capture_previous = capture;
	// store it into the buffer
	i = buffer_head + 1;
	if (i >= FREQMEASURE_BUFFER_LEN)
	i = 0;
	if (i != buffer_tail) {
		buffer_value[i] = period;
		buffer_head = i;
	}
}
#endif //ENABLE_FREQMEASURE
//--
#ifdef ENABLE_CONVERSION
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
#endif // end conversion
//--
#ifdef ENABLE_XPRINTF

#if _USE_XFUNC_OUT
#include <stdarg.h>
void (*xfunc_out)(unsigned char); /* Pointer to the output stream */
static char *outptr;

/*----------------------------------------------*/
/* Put a character                              */
/*----------------------------------------------*/

void xputc (char c)
{
	if (_CR_CRLF && c == '\n') xputc('\r'); /* CR -> CRLF */

	if (outptr) {
		*outptr++ = (unsigned char)c;
		return;
	}

	if (xfunc_out) xfunc_out((unsigned char)c);
}

/*----------------------------------------------*/
/* Put a null-terminated string                 */
/*----------------------------------------------*/

void xputs ( /* Put a string to the default device */
		const char* str /* Pointer to the string */
)
{
	while (*str)
	xputc(*str++);
}

void xfputs ( /* Put a string to the specified device */
		void(*func)(unsigned char), /* Pointer to the output function */
		const char* str /* Pointer to the string */
)
{
	void (*pf)(unsigned char);

	pf = xfunc_out; /* Save current output device */
	xfunc_out = func; /* Switch output to specified device */
	while (*str) /* Put the string */
	xputc(*str++);
	xfunc_out = pf; /* Restore output device */
}

/*----------------------------------------------*/
/* Formatted string output                      */
/*----------------------------------------------*/
/*  xprintf("%d", 1234);			"1234"
 xprintf("%6d,%3d%%", -200, 5);	"  -200,  5%"
 xprintf("%-6u", 100);			"100   "
 xprintf("%ld", 12345678L);		"12345678"
 xprintf("%04x", 0xA3);			"00a3"
 xprintf("%08LX", 0x123ABC);		"00123ABC"
 xprintf("%016b", 0x550F);		"0101010100001111"
 xprintf("%s", "String");		"String"
 xprintf("%-4s", "abc");			"abc "
 xprintf("%4s", "abc");			" abc"
 xprintf("%c", 'a');				"a"
 xprintf("%f", 10.0);            <xprintf lacks floating point support>
 */

#if WANT_XPRINTF
static
void xvprintf (
		const char* fmt, /* Pointer to the format string */
		va_list arp /* Pointer to arguments */
)
{
	unsigned int r, i, j, w, f;
	unsigned long v;
	char s[16], c, d, *p;

	for (;;) {
		c = *fmt++; /* Get a char */
		if (!c) break; /* End of format? */
		if (c != '%') { /* Pass through it if not a % sequense */
			xputc(c); continue;
		}
		f = 0;
		c = *fmt++; /* Get first char of the sequense */
		if (c == '0') { /* Flag: '0' padded */
			f = 1; c = *fmt++;
		} else {
			if (c == '-') { /* Flag: left justified */
				f = 2; c = *fmt++;
			}
		}
		for (w = 0; c >= '0' && c <= '9'; c = *fmt++) /* Minimum width */
		w = w * 10 + c - '0';
		if (c == 'l' || c == 'L') { /* Prefix: Size is long int */
			f |= 4; c = *fmt++;
		}
		if (!c) break; /* End of format? */
		d = c;
		if (d >= 'a') d -= 0x20;
		switch (d) { /* Type is... */
			case 'S' : /* String */
			p = va_arg(arp, char*);
			for (j = 0; p[j]; j++);
			while (!(f & 2) && j++ < w) xputc(' ');
			xputs(p);
			while (j++ < w) xputc(' ');
			continue;
			case 'C' : /* Character */
			xputc((char)va_arg(arp, int)); continue;
			case 'B' : /* Binary */
			r = 2; break;
			case 'O' : /* Octal */
			r = 8; break;
			case 'D' : /* Signed decimal */
			case 'U' : /* Unsigned decimal */
			r = 10; break;
			case 'X' : /* Hexdecimal */
			r = 16; break;
			default: /* Unknown type (passthrough) */
			xputc(c); continue;
		}

		/* Get an argument and put it in numeral */
		v = (f & 4) ? va_arg(arp, long) : ((d == 'D') ? (long)va_arg(arp, int) : (long)va_arg(arp, unsigned int));
		if (d == 'D' && (v & 0x80000000)) {
			v = 0 - v;
			f |= 8;
		}
		i = 0;
		do {
			d = (char)(v % r); v /= r;
			if (d > 9) d += (c == 'x') ? 0x27 : 0x07;
			s[i++] = d + '0';
		}while (v && i < sizeof(s));
		if (f & 8) s[i++] = '-';
		j = i; d = (f & 1) ? '0' : ' ';
		while (!(f & 2) && j++ < w) xputc(d);
		do xputc(s[--i]); while(i);
		while (j++ < w) xputc(' ');
	}
}

void xprintf ( /* Put a formatted string to the default device */
		const char* fmt, /* Pointer to the format string */
		... /* Optional arguments */
)
{
	va_list arp;

	va_start(arp, fmt);
	xvprintf(fmt, arp);
	va_end(arp);
}

void xsprintf ( /* Put a formatted string to the memory */
		char* buff, /* Pointer to the output buffer */
		const char* fmt, /* Pointer to the format string */
		... /* Optional arguments */
)
{
	va_list arp;

	outptr = buff; /* Switch destination for memory */

	va_start(arp, fmt);
	xvprintf(fmt, arp);
	va_end(arp);

	*outptr = 0; /* Terminate output string with a \0 */
	outptr = 0; /* Switch destination for device */
}

void xfprintf ( /* Put a formatted string to the specified device */
		void(*func)(unsigned char), /* Pointer to the output function */
		const char* fmt, /* Pointer to the format string */
		... /* Optional arguments */
)
{
	va_list arp;
	void (*pf)(unsigned char);

	pf = xfunc_out; /* Save current output device */
	xfunc_out = func; /* Switch output to specified device */

	va_start(arp, fmt);
	xvprintf(fmt, arp);
	va_end(arp);

	xfunc_out = pf; /* Restore output device */
}

/*----------------------------------------------*/
/* Dump a line of binary dump                   */
/*----------------------------------------------*/

void put_dump (
		const void* buff, /* Pointer to the array to be dumped */
		unsigned long addr, /* Heading address value */
		int len, /* Number of items to be dumped */
		int width /* Size of the items (DF_CHAR, DF_SHORT, DF_LONG) */
)
{
	int i;
	const unsigned char *bp;
	const unsigned short *sp;
	const unsigned long *lp;

	xprintf("%08lX ", addr); /* address */

	switch (width) {
		case DW_CHAR:
		bp = buff;
		for (i = 0; i < len; i++) /* Hexdecimal dump */
		xprintf(" %02X", bp[i]);
		xputc(' ');
		for (i = 0; i < len; i++) /* ASCII dump */
		xputc((bp[i] >= ' ' && bp[i] <= '~') ? bp[i] : '.');
		break;
		case DW_SHORT:
		sp = buff;
		do /* Hexdecimal dump */
		xprintf(" %04X", *sp++);
		while (--len);
		break;
		case DW_LONG:
		lp = buff;
		do /* Hexdecimal dump */
		xprintf(" %08LX", *lp++);
		while (--len);
		break;
	}

	xputc('\n');
}

#endif

#define PB(a) (pgm_read_byte(&(a)))

static
void xvprintf_P (
		const char* fmt, /* Pointer to the format string */
		va_list arp /* Pointer to arguments */
)
{
	unsigned int r, i, j, w, f;
	unsigned long v;
	char s[16], c, d, *p;

	for (;;) {
		c = PB(*fmt++); /* Get a char */
		if (!c) break; /* End of format? */
		if (c != '%') { /* Pass through it if not a % sequense */
			xputc(c); continue;
		}
		f = 0;
		c = PB(*fmt++); /* Get first char of the sequense */
		if (c == '0') { /* Flag: '0' padded */
			f = 1; c = PB(*fmt++);
		} else {
			if (c == '-') { /* Flag: left justified */
				f = 2; c = PB(*fmt++);
			}
		}
		for (w = 0; c >= '0' && c <= '9'; c = PB(*fmt++)) /* Minimum width */
		w = w * 10 + c - '0';
		if (c == 'l' || c == 'L') { /* Prefix: Size is long int */
			f |= 4; c = PB(*fmt++);
		}
		if (!c) break; /* End of format? */
		d = c;
		if (d >= 'a') d -= 0x20;
		switch (d) { /* Type is... */
			case 'S' : /* String */
			p = va_arg(arp, char*);
			for (j = 0; p[j]; j++);
			while (!(f & 2) && j++ < w) xputc(' ');
			xputs(p);
			while (j++ < w) xputc(' ');
			continue;
			case 'C' : /* Character */
			xputc((char)va_arg(arp, int)); continue;
			case 'B' : /* Binary */
			r = 2; break;
			case 'O' : /* Octal */
			r = 8; break;
			case 'D' : /* Signed decimal */
			case 'U' : /* Unsigned decimal */
			r = 10; break;
			case 'X' : /* Hexdecimal */
			r = 16; break;
			default: /* Unknown type (passthrough) */
			xputc(c); continue;
		}

		/* Get an argument and put it in numeral */
		v = (f & 4) ? va_arg(arp, long) : ((d == 'D') ? (long)va_arg(arp, int) : (long)va_arg(arp, unsigned int));
		if (d == 'D' && (v & 0x80000000)) {
			v = 0 - v;
			f |= 8;
		}
		i = 0;
		do {
			d = (char)(v % r); v /= r;
			if (d > 9) d += (c == 'x') ? 0x27 : 0x07;
			s[i++] = d + '0';
		}while (v && i < sizeof(s));
		if (f & 8) s[i++] = '-';
		j = i; d = (f & 1) ? '0' : ' ';
		while (!(f & 2) && j++ < w) xputc(d);
		do xputc(s[--i]); while(i);
		while (j++ < w) xputc(' ');
	}
}

void xprintf_P ( /* Put a formatted string to the default device */
		const char* fmt, /* Pointer to the format string */
		... /* Optional arguments */
)
{
	va_list arp;

	va_start(arp, fmt);
	xvprintf_P(fmt, arp);
	va_end(arp);
}

void xsprintf_P ( /* Put a formatted string to the memory */
		char* buff, /* Pointer to the output buffer */
		const char* fmt, /* Pointer to the format string */
		... /* Optional arguments */
)
{
	va_list arp;

	outptr = buff; /* Switch destination for memory */

	va_start(arp, fmt);
	xvprintf_P(fmt, arp);
	va_end(arp);

	*outptr = 0; /* Terminate output string with a \0 */
	outptr = 0; /* Switch destination for device */
}

#endif /* _USE_XFUNC_OUT */

#if _USE_XFUNC_IN
unsigned char (*xfunc_in)(void); /* Pointer to the input stream */

/*----------------------------------------------*/
/* Get a line from the input                    */
/*----------------------------------------------*/

int xgets ( /* 0:End of stream, 1:A line arrived */
		char* buff, /* Pointer to the buffer */
		int len /* Buffer length */
)
{
	int c, i;

	if (!xfunc_in) return 0; /* No input function specified */

	i = 0;
	for (;;) {
		c = xfunc_in(); /* Get a char from the incoming stream */
		if (!c) return 0; /* End of stream? */
		if (c == '\r') break; /* End of line? */
		if (c == '\b' && i) { /* Back space? */
			i--;
			if (_LINE_ECHO) xputc(c);
			continue;
		}
		if (c >= ' ' && i < len - 1) { /* Visible chars */
			buff[i++] = c;
			if (_LINE_ECHO) xputc(c);
		}
	}
	buff[i] = 0; /* Terminate with a \0 */
	if (_LINE_ECHO) xputc('\n');
	return 1;
}

int xfgets ( /* 0:End of stream, 1:A line arrived */
		unsigned char (*func)(void), /* Pointer to the input stream function */
		char* buff, /* Pointer to the buffer */
		int len /* Buffer length */
)
{
	unsigned char (*pf)(void);
	int n;

	pf = xfunc_in; /* Save current input device */
	xfunc_in = func; /* Switch input to specified device */
	n = xgets(buff, len); /* Get a line */
	xfunc_in = pf; /* Restore input device */

	return n;
}

/*----------------------------------------------*/
/* Get a value of the string                    */
/*----------------------------------------------*/
/*	"123 -5   0x3ff 0b1111 0377  w "
 ^                           1st call returns 123 and next ptr
 ^                        2nd call returns -5 and next ptr
 ^                3rd call returns 1023 and next ptr
 ^         4th call returns 15 and next ptr
 ^    5th call returns 255 and next ptr
 ^ 6th call fails and returns 0
 */

int xatoi ( /* 0:Failed, 1:Successful */
		char **str, /* Pointer to pointer to the string */
		long *res /* Pointer to the valiable to store the value */
)
{
	unsigned long val;
	unsigned char c, r, s = 0;

	*res = 0;

	while ((c = **str) == ' ') (*str)++; /* Skip leading spaces */

	if (c == '-') { /* negative? */
		s = 1;
		c = *(++(*str));
	}

	if (c == '0') {
		c = *(++(*str));
		switch (c) {
			case 'x': /* hexdecimal */
			r = 16; c = *(++(*str));
			break;
			case 'b': /* binary */
			r = 2; c = *(++(*str));
			break;
			default:
			if (c <= ' ') return 1; /* single zero */
			if (c < '0' || c > '9') return 0; /* invalid char */
			r = 8; /* octal */
		}
	} else {
		if (c < '0' || c > '9') return 0; /* EOL or invalid char */
		r = 10; /* decimal */
	}

	val = 0;
	while (c > ' ') {
		if (c >= 'a') c -= 0x20;
		c -= '0';
		if (c >= 17) {
			c -= 7;
			if (c <= 9) return 0; /* invalid char */
		}
		if (c >= r) return 0; /* invalid char for current radix */
		val = val * r + c;
		c = *(++(*str));
	}
	if (s) val = 0 - val; /* apply sign if needed */

	*res = val;
	return 1;
}
#endif /* _USE_XFUNC_IN */

#endif //ENABLE_XPRINTF
//--
#ifdef ENABLE_SERIAL
//Interrupt based.
//
uint8_t uart_buffer[UART_BUFFER_SIZE];
volatile uint8_t uart_readptr;
volatile uint8_t uart_writeptr;

ISR(UART0_ISR_VECT)// see the header file...
{
	uart_buffer[uart_writeptr] = UART0_DATA;
	uart_writeptr = (uart_writeptr + 1) % UART_BUFFER_SIZE;
}

void serial_init(void) {
	uart_writeptr = 0;
	uart_readptr = 0;

	// set default baud rate
#if defined(USART_RXC_vect) // for ATmega16A
	UBRRH = UART_BAUD_SELECT >> 8;
	UBRRL = UART_BAUD_SELECT;

	// enable receive, transmit and enable receive interrupts
	UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);

	// don't forget sei()
#else
	UBRR0H = UART_BAUD_SELECT >> 8;
	UBRR0L = UART_BAUD_SELECT;

	// enable receive, transmit and enable receive interrupts
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

	// don't forget sei()
#endif
}

uint8_t serial_getchar(void) {
	uint8_t c;

	//Non-blocking function - but serial_available() MUST be used!!!
	c = uart_buffer[uart_readptr];
	uart_readptr = (uart_readptr + 1) % UART_BUFFER_SIZE;
	return c;
}

uint8_t serial_available(void) {
	if (uart_writeptr != uart_readptr)
	return TRUE;
	else
	return FALSE;
}

void serial_flush(void) { // sort of...
	// only reinitializing the "pointers"
	uart_writeptr = 0;
	uart_readptr = 0;
}

#endif // end serial with interrupt
//--
#ifdef ENABLE_SERIAL_POLL
/* Initialize UART */
void serial_init(void) {
#if defined(USART_RXC_vect) // for ATmega16A
	/* Set the baud rate */
	UBRRH = (uint8_t) (UART_BAUD_SELECT >> 8);
	UBRRL = (uint8_t) UART_BAUD_SELECT;
	/* Enable UART receiver and transmitter */

	UCSRB = ((1 << RXEN) | (1 << TXEN));
	/* Set frame format: 8 data 2stop */
	UCSRC = (1 << USBS) | (1 << UCSZ1) | (1 << UCSZ0); //For devices with Extended IO
	//UCSR0C = (1<<URSEL)|(1<<USBS0)|(1<<UCSZ01)|(1<<UCSZ00);   //For devices without Extended IO
#else
	/* Set the baud rate */
	UBRR0H = (uint8_t) (UART_BAUD_SELECT >> 8);
	UBRR0L = (uint8_t) UART_BAUD_SELECT;
	/* Enable UART receiver and transmitter */UCSR0B = ((1 << RXEN0)
			| (1 << TXEN0));
	/* Set frame format: 8 data 2stop */
	UCSR0C = (1 << USBS0) | (1 << UCSZ01) | (1 << UCSZ00); //For devices with Extended IO
	//UCSR0C = (1<<URSEL)|(1<<USBS0)|(1<<UCSZ01)|(1<<UCSZ00);   //For devices without Extended IO
#endif
}

/* Read and write functions */
uint8_t serial_getchar(void) {
#if defined(USART_RXC_vect) // for ATmega16A
	/* Wait for incoming data */
	while (!(UCSRA & (1 << RXC)))
	;
	/* Return the data */
	return UDR;
#else
	/* Wait for incoming data */
	while (!(UCSR0A & (1 << RXC0)))
		;
	/* Return the data */
	return UDR0;
#endif
}
#endif // END SERIAL_POLL
//
#if defined(ENABLE_SERIAL) || defined(ENABLE_SERIAL_POLL)
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

#endif
//--
#ifdef ENABLE_SPI
#ifdef ENABLE_SPI_INT

ISR(SPI_STC_vect) {
	SPI_TC = TRUE;
}
#endif

void SPI_master_init(void) {
	// Set SS to high so a connected chip will be "deselected" by default
	sbi(SS_PORT, SS);

	// When the SS pin is set as OUTPUT, it can be used as
	// a general purpose output port (it doesn't influence
	// SPI operations).
	sbi(SS_DDR, SS);

#ifdef ENABLE_SPI_INT
	SPI_TC = FALSE;
#endif
	// Warning: if the SS pin ever becomes a LOW INPUT then SPI
	// automatically switches to Slave, so the data direction of
	// the SS pin MUST be kept as OUTPUT.
	SPCR |= _BV(MSTR);
	SPCR |= _BV(SPE);

	// Set direction register for SCK and MOSI pin.
	// MISO pin automatically overrides to INPUT.
	// By doing this AFTER enabling SPI, we avoid accidentally
	// clocking in a single bit since the lines go directly
	// from "input" to SPI control.
	// http://code.google.com/p/arduino/issues/detail?id=888
	sbi(SCK_DDR, SCK);
	sbi(MOSI_DDR, MOSI);
}

void SPI_master_setBitOrder(uint8_t bitOrder) {
	if (bitOrder == LSBFIRST) {
		SPCR |= _BV(DORD);
	} else {
		SPCR &= ~(_BV(DORD));
	}
}

void SPI_master_setDataMode(uint8_t mode) {
	SPCR = (SPCR & ~SPI_MODE_MASK) | mode;
}

void SPI_master_setClockDivider(uint8_t rate) {
	SPCR = (SPCR & ~SPI_CLOCK_MASK) | (rate & SPI_CLOCK_MASK);
	SPSR = (SPSR & ~SPI_2XCLOCK_MASK) | ((rate >> 2) & SPI_2XCLOCK_MASK);
}

void SPI_attachInterrupt(void) {
	SPCR |= _BV(SPIE);
}

void SPI_detachInterrupt(void) {
	SPCR &= ~_BV(SPIE);
}

void SPI_master_stop(void) {
	SPCR &= ~_BV(SPE);
}

uint8_t SPI_master_transmit(uint8_t data) {
	uint8_t reply;
	// Start transmission
	SPDR = data;
	// Wait for transmission complete
#ifdef ENABLE_SPI_INT
	while(!SPI_TC)
	;
#else
	while (!(SPSR & (1 << SPIF)))
		;
#endif
#ifdef ENABLE_SPI_INT
	SPI_TC = FALSE;
#endif
	reply = SPDR;
	return reply;
}

uint8_t SPI_master_receive(void) {
	uint8_t data;
	// Start reception
	SPDR = 0xFF;
	// Wait for transmission complete
#ifdef ENABLE_SPI_INT
	while(!SPI_TC)
	;
#else
	while (!(SPSR & (1 << SPIF)))
		;
#endif
#ifdef ENABLE_SPI_INT
	SPI_TC = FALSE;
#endif
	data = SPDR;
	return data;
}

//the following function is required by (Arduino/ATmega)ISP programmer

uint8_t SPI_master_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
	uint8_t n;
	//int8_t error;
	//n = 0;
	SPI_master_transmit(a);
	n = SPI_master_transmit(b);
	//if (n != a) error = -1;
	n = SPI_master_transmit(c);
	return SPI_master_transmit(d);
}

#endif // ENABLE_SPI
//--
#ifdef ENABLE_SD_CARD

//******************************************************************
//Function	: to initialize the SD/SDHC card in SPI mode
//Arguments	: none
//return	: unsigned char; will be 0 if no error,
// 			  otherwise the response byte will be sent
//******************************************************************
uint8_t SD_init(void) {
	uint8_t i, response, SD_version;
	uint16_t retry = 0;

	//set the direction of the CS pin
	sbi(SD_CS_DDR, SD_CS_PIN);
	//output
	SD_cardType = 0;
	for (i = 0; i < 10; i++)
	SPI_master_transmit(0xff);//80 clock pulses spent before sending the first command
	SD_CS_ASSERT;
	do {

		response = SD_sendCommand(GO_IDLE_STATE, 0); //send 'reset & go idle' command
		retry++;
		if (retry > 0x20)
		return 1;//time out, card not detected

	}while (response != 0x01);

	SD_CS_DEASSERT;
	SPI_master_transmit(0xff);
	SPI_master_transmit(0xff);

	retry = 0;

	SD_version = 2; //default set to SD compliance with ver2.x;
					//this may change after checking the next command
	do {
		response = SD_sendCommand(SEND_IF_COND, 0x000001AA); //Check power supply status, mandatory for SDHC card
		retry++;
		if (retry > 0xfe) {
			//TX_NEWLINE;
			SD_version = 1;
			SD_cardType = 1;
			break;
		} //time out

	}while (response != 0x01);

	retry = 0;

	do {
		response = SD_sendCommand(APP_CMD, 0); //CMD55, must be sent before sending any ACMD command
		response = SD_sendCommand(SD_SEND_OP_COND, 0x40000000);//ACMD41

		retry++;
		if (retry > 0xfe) {
			//TX_NEWLINE;
			return 2;//time out, card initialization failed
		}

	}while (response != 0x00);

	retry = 0;
	SDHC_flag = 0;

	if (SD_version == 2) {
		do {
			response = SD_sendCommand(READ_OCR, 0);
			retry++;
			if (retry > 0xfe) {
				//TX_NEWLINE;
				SD_cardType = 0;
				break;
			} //time out

		}while (response != 0x00);

		if (SDHC_flag == 1)
		SD_cardType = 2;
		else
		SD_cardType = 3;
	}

	//SD_sendCommand(CRC_ON_OFF, OFF); //disable CRC; default - CRC disabled in SPI mode
	//SD_sendCommand(SET_BLOCK_LEN, 512); //set block size to 512; default size is 512

	return 0;//successful return
} // end SD_init

// Call this after SD_init()
uint8_t SD_card_type(void) { //because now SD_cardType is not public...
	return SD_cardType;
}

//******************************************************************
//Function	: to send a command to SD card
//Arguments	: unsigned char (8-bit command value)
// 			  & unsigned long (32-bit command argument)
//return	: unsigned char; response byte
//******************************************************************
uint8_t SD_sendCommand(uint8_t cmd, uint32_t arg) {
	uint8_t response, retry = 0, status;

	//SD card accepts byte address while SDHC accepts block address in multiples of 512
	//so, if it's SD card we need to convert block address into corresponding byte address by
	//multiplying it with 512. which is equivalent to shifting it left 9 times
	//following 'if' loop does that

	if (SDHC_flag == 0)
	if (cmd == READ_SINGLE_BLOCK || cmd == READ_MULTIPLE_BLOCKS
			|| cmd == WRITE_SINGLE_BLOCK || cmd == WRITE_MULTIPLE_BLOCKS
			|| cmd == ERASE_BLOCK_START_ADDR || cmd == ERASE_BLOCK_END_ADDR) {
		arg = arg << 9;
	}

	SD_CS_ASSERT;

	SPI_master_transmit(cmd | 0x40); //send command, first two bits always '01'
	SPI_master_transmit(arg >> 24);
	SPI_master_transmit(arg >> 16);
	SPI_master_transmit(arg >> 8);
	SPI_master_transmit(arg);

	if (cmd == SEND_IF_COND)//it is compulsory to send correct CRC for CMD8 (CRC=0x87) & CMD0 (CRC=0x95)
	SPI_master_transmit(0x87);//for remaining commands, CRC is ignored in SPI mode
	else
	SPI_master_transmit(0x95);

	while ((response = SPI_master_receive()) == 0xff)//wait response
	if (retry++ > 0xfe)
	break;//time out error

	if (response == 0x00 && cmd == 58)//checking response of CMD58
	{
		status = SPI_master_receive() & 0x40; //first byte of the OCR register (bit 31:24)
		if (status == 0x40)
		SDHC_flag = 1;//we need it to verify SDHC card
		else
		SDHC_flag = 0;

		SPI_master_receive();//remaining 3 bytes of the OCR register are ignored here
		SPI_master_receive();//one can use these bytes to check power supply limits of SD
		SPI_master_receive();
	}

	SPI_master_receive(); //extra 8 CLK
	SD_CS_DEASSERT;

	return response;//return state
} // end SD_sendCommand

//*****************************************************************
//Function	: to erase specified no. of blocks of SD card
//Arguments	: none
//return	: unsigned char; will be 0 if no error,
// 			  otherwise the response byte will be sent
//*****************************************************************
uint8_t SD_erase(uint32_t SD_startBlock, uint32_t SD_totalBlocks) {
	uint8_t response;

	response = SD_sendCommand(ERASE_BLOCK_START_ADDR, SD_startBlock); //send starting block address
	if (response != 0x00)//check for SD status: 0x00 - OK (No flags set)
	return response;

	response = SD_sendCommand(ERASE_BLOCK_END_ADDR,
			(SD_startBlock + SD_totalBlocks - 1));//send end block address
	if (response != 0x00)
	return response;

	response = SD_sendCommand(ERASE_SELECTED_BLOCKS, 0);//erase all selected blocks
	if (response != 0x00)
	return response;

	return 0;//normal return
} // end SD_erase

//******************************************************************
//Function	: to read a single block from SD card
//Arguments	: none
//return	: unsigned char; will be 0 if no error,
// 			  otherwise the response byte will be sent
//******************************************************************
uint8_t SD_readSingleBlock(uint32_t SD_startBlock) {
	uint8_t response;
	uint16_t i, retry = 0;

	response = SD_sendCommand(READ_SINGLE_BLOCK, SD_startBlock); //read a Block command

	if (response != 0x00)
	return response;//check for SD status: 0x00 - OK (No flags set)

	SD_CS_ASSERT;

	retry = 0;
	while (SPI_master_receive() != 0xfe)//wait for start block token 0xfe (0x11111110)
	if (retry++ > 0xfffe) {
		SD_CS_DEASSERT;
		return 1;
	} //return if time-out

	for (i = 0; i < 512; i++)//read 512 bytes
	SD_buffer[i] = SPI_master_receive();

	SPI_master_receive();//receive incoming CRC (16-bit), CRC is ignored here
	SPI_master_receive();

	SPI_master_receive();//extra 8 clock pulses
	SD_CS_DEASSERT;

	return 0;
}

//******************************************************************
//Function	: to write to a single block of SD card
//Arguments	: none
//return	: unsigned char; will be 0 if no error,
// 			  otherwise the response byte will be sent
//******************************************************************
uint8_t SD_writeSingleBlock(uint32_t SD_startBlock) {
	uint8_t response;
	uint16_t i, retry = 0;

	response = SD_sendCommand(WRITE_SINGLE_BLOCK, SD_startBlock); //write a Block command

	if (response != 0x00)
	return response;//check for SD status: 0x00 - OK (No flags set)
	SD_CS_ASSERT;

	SPI_master_transmit(0xfe);//Send start block token 0xfe (0x11111110)

	for (i = 0; i < 512; i++)//send 512 bytes data
	SPI_master_transmit(SD_buffer[i]);

	SPI_master_transmit(0xff);//transmit dummy CRC (16-bit), CRC is ignored here
	SPI_master_transmit(0xff);

	response = SPI_master_receive();

	if ((response & 0x1f) != 0x05)//response= 0xXXX0AAA1 ; AAA='010' - data accepted
	{ //AAA='101'-data rejected due to CRC error
		SD_CS_DEASSERT;//AAA='110'-data rejected due to write error
		return response;
	}

	while (!SPI_master_receive()) //wait for SD card to complete writing and get idle
	if (retry++ > 0xfffe) {
		SD_CS_DEASSERT;
		return 1;
	}

	SD_CS_DEASSERT;
	SPI_master_transmit(0xff); //just spend 8 clock cycle delay before re asserting the CS line
	SD_CS_ASSERT;//re-asserting the CS line to verify if card is still busy

	while (!SPI_master_receive())//wait for SD card to complete writing and get idle
	if (retry++ > 0xfffe) {
		SD_CS_DEASSERT;
		return 1;
	}
	SD_CS_DEASSERT;

	return 0;
}
#endif // ENABLE_SD_CARD
// TODO: define FAT32 error codes!
#ifdef ENABLE_FAT32

// TODO: to be rewritten... depends on PCF8583 RTC functions...
uint8_t getDateTime_FAT(void) {
#ifdef ENABLE_PCF8583
	//it should get the time from RTC but for now, returns "error" code
	return 1;
#else
	return 1; // returning "error" code if RTC not available
			  // date and time are 0 - good enough for testing and budget
#endif
}

//***************************************************************************
//Function: to read data from boot sector of SD card, to determine important
//parameters like bytesPerSector, sectorsPerCluster etc.
//Arguments: none
//return: none
//***************************************************************************
uint8_t F32_getBootSectorData(void) {
	struct BS_Structure *bpb; //mapping the buffer onto the structure
	struct MBRinfo_Structure *mbr;
	struct partitionInfo_Structure *partition;
	uint32_t dataSectors;

	unusedSectors = 0;

	SD_readSingleBlock(0);
	bpb = (struct BS_Structure *) SD_buffer;

	if (bpb->jumpBoot[0] != 0xE9 && bpb->jumpBoot[0] != 0xEB)//check if it is boot sector
	{
		mbr = (struct MBRinfo_Structure *) SD_buffer; //if it is not boot sector, it must be MBR

		if (mbr->signature != 0xaa55)
		return 1;//if it is not even MBR then it's not FAT32

		partition = (struct partitionInfo_Structure *) (mbr->partitionData);//first partition
		unusedSectors = partition->firstSector;//the unused sectors, hidden to the FAT

		SD_readSingleBlock(partition->firstSector);//read the bpb sector
		bpb = (struct BS_Structure *) SD_buffer;
		if (bpb->jumpBoot[0] != 0xE9 && bpb->jumpBoot[0] != 0xEB)
		return 1;
	}

	bytesPerSector = bpb->bytesPerSector;
#ifdef ENABLE_SD_CARD_DEBUG
	//serial_puthexU16(bytesPerSector); serial_putchar(' ');
#endif
	sectorPerCluster = bpb->sectorPerCluster;
#ifdef ENABLE_SD_CARD_DEBUG
	//serial_puthexU08(sectorPerCluster); serial_putchar(' ');
#endif
	reservedSectorCount = bpb->reservedSectorCount;
	rootCluster = bpb->rootCluster; // + (sector / sectorPerCluster) +1;
	firstDataSector = bpb->hiddenSectors + reservedSectorCount
	+ (bpb->numberofFATs * bpb->FATsize_F32);

	dataSectors = bpb->totalSectors_F32 - bpb->reservedSectorCount
	- (bpb->numberofFATs * bpb->FATsize_F32);
	totalClusters = dataSectors / sectorPerCluster;
#ifdef ENABLE_SD_CARD_DEBUG
	//serial_puthexU32(totalClusters); serial_putchar(' ');
#endif
	if ((F32_getSetFreeCluster(TOTAL_FREE, GET, 0)) > totalClusters) //check if FSinfo free clusters count is valid
	freeClusterCountUpdated = 0;
	else
	freeClusterCountUpdated = 1;
	return 0;
}

//***************************************************************************
//Function: to calculate first sector address of any given cluster
//Arguments: cluster number for which first sector is to be found
//return: first sector address
//***************************************************************************
uint32_t F32_getFirstSector(uint32_t clusterNumber) {
	return (((clusterNumber - 2) * sectorPerCluster) + firstDataSector);
}

//***************************************************************************
//Function: get cluster entry value from FAT to find out the next cluster in the chain
//or set new cluster entry in FAT
//Arguments: 1. current cluster number, 2. get_set (=GET, if next cluster is to be found or = SET,
//if next cluster is to be set 3. next cluster number, if argument#2 = SET, else 0
//return: next cluster number, if if argument#2 = GET, else 0
//****************************************************************************
uint32_t F32_getSetNextCluster(uint32_t clusterNumber, uint8_t get_set,
		uint32_t clusterEntry) {
	uint16_t FATEntryOffset;
	uint32_t *FATEntryValue;
	uint32_t FATEntrySector;
	uint8_t retry = 0;

//get sector number of the cluster entry in the FAT
	FATEntrySector = unusedSectors + reservedSectorCount
	+ ((clusterNumber * 4) / bytesPerSector);

//get the offset address in that sector number
	FATEntryOffset = (uint16_t) ((clusterNumber * 4) % bytesPerSector);

//read the sector into a buffer
	while (retry < 10) {
		if (!SD_readSingleBlock(FATEntrySector))
		break;
		retry++;
	}

//get the cluster address from the buffer
	FATEntryValue = (uint32_t *) &SD_buffer[FATEntryOffset];

	if (get_set == GET)
	return ((*FATEntryValue) & 0x0fffffff);

	*FATEntryValue = clusterEntry;//for setting new value in cluster entry in FAT

	SD_writeSingleBlock(FATEntrySector);

	return (0);
}

//********************************************************************************************
//Function: to get or set next free cluster or total free clusters in FSinfo sector of SD card
//Arguments: 1.flag:TOTAL_FREE or NEXT_FREE,
//       2.flag: GET or SET
//       3.new FS entry, when argument2 is SET; or 0, when argument2 is GET
//return: next free cluster, if arg1 is NEXT_FREE & arg2 is GET
//        total number of free clusters, if arg1 is TOTAL_FREE & arg2 is GET
//      0xffffffff, if any error or if arg2 is SET
//********************************************************************************************
uint32_t F32_getSetFreeCluster(uint8_t totOrNext, uint8_t get_set,
		uint32_t FSEntry) {

	struct FSInfo_Structure *FS = (struct FSInfo_Structure *) &SD_buffer;

	SD_readSingleBlock(unusedSectors + 1);

	if ((FS->leadSignature != 0x41615252)
			|| (FS->structureSignature != 0x61417272)
			|| (FS->trailSignature != 0xaa550000))
	return 0xffffffff;

	if (get_set == GET) {
		if (totOrNext == TOTAL_FREE)
		return (FS->freeClusterCount);
		else
		// when totOrNext = NEXT_FREE
		return (FS->nextFreeCluster);
	} else {
		if (totOrNext == TOTAL_FREE)
		FS->freeClusterCount = FSEntry;
		else
		// when totOrNext = NEXT_FREE
		FS->nextFreeCluster = FSEntry;

		SD_writeSingleBlock(unusedSectors + 1);//update FSinfo
	}
	return 0xffffffff;
}

//***************************************************************************
//Function: to get DIR/FILE list or a single file address (cluster number) or to delete a specified file
//Arguments: #1 - flag: GET_LIST, GET_FILE or DELETE #2 - pointer to file name (0 if arg#1 is GET_LIST)
//return: first cluster of the file, if flag = GET_FILE
//        print file/dir list of the root directory, if flag = GET_LIST
//      Delete the file mentioned in arg#2, if flag = DELETE
//****************************************************************************
struct dir_Structure* F32_findFiles(uint8_t flag, uint8_t *fileName) {
	uint32_t cluster, sector, firstSector, firstCluster, nextCluster;
	struct dir_Structure *dir;
	uint16_t i;
	uint8_t j;

	cluster = rootCluster; //root cluster

	while (1) {
		firstSector = F32_getFirstSector(cluster);

		for (sector = 0; sector < sectorPerCluster; sector++) {
			SD_readSingleBlock(firstSector + sector);

			for (i = 0; i < bytesPerSector; i += 32) {
				dir = (struct dir_Structure *) &SD_buffer[i];

				if (dir->name[0] == EMPTY) //indicates end of the file list of the directory
				{
#ifdef ENABLE_SD_CARD_DEBUG
					if (flag == DELETE)
					serial_puts_f("\r\nFile does not exist!");
#endif
					return 0;
				}
				if ((dir->name[0] != DELETED)
						&& (dir->attrib != ATTR_LONG_NAME)) {
					if ((flag == GET_FILE) || (flag == DELETE)) {
						for (j = 0; j < 11; j++)
						if (dir->name[j] != fileName[j])
						break;
						if (j == 11) {
							if (flag == GET_FILE) {
								appendFileSector = firstSector + sector;
								appendFileLocation = i;
								appendStartCluster =
								(((uint32_t) dir->firstClusterHI) << 16)
								| dir->firstClusterLO;
								fileSize = dir->fileSize;
								return (dir);
							} else //when flag = DELETE
							{
#ifdef ENABLE_SD_CARD_DEBUG
								//TX_NEWLINE;
								serial_puts_f("\r\nDeleting..");
								//TX_NEWLINE;
								//TX_NEWLINE;
#endif
								firstCluster = (((uint32_t) dir->firstClusterHI)
										<< 16) | dir->firstClusterLO;

								//mark file as 'deleted' in FAT table
								dir->name[0] = DELETED;
								SD_writeSingleBlock(firstSector + sector);

								F32_freeMemoryUpdate(ADD, dir->fileSize);

								//update next free cluster entry in FSinfo sector
								cluster = F32_getSetFreeCluster(NEXT_FREE, GET,
										0);
								if (firstCluster < cluster)
								F32_getSetFreeCluster(NEXT_FREE, SET,
										firstCluster);

								//mark all the clusters allocated to the file as 'free'
								while (1) {
									nextCluster = F32_getSetNextCluster(
											firstCluster, GET, 0);
									F32_getSetNextCluster(firstCluster, SET, 0);
									if (nextCluster > 0x0ffffff6) {
#ifdef ENABLE_SD_CARD_DEBUG
										serial_puts_f("\r\nFile deleted!");
#endif
										return 0;
									}
									firstCluster = nextCluster;
								}
							}
						}
					} else //when flag = GET_LIST
					{
#ifdef ENABLE_SD_CARD_DEBUG
						TX_NEWLINE;
#endif
						for (j = 0; j < 11; j++) {
#ifdef ENABLE_SD_CARD_DEBUG
							if (j == 8)
							serial_putc(' ');
							serial_putc(dir->name[j]);
#endif
						}
#ifdef ENABLE_SD_CARD_DEBUG
						serial_puts("   ");
#endif
						if ((dir->attrib != 0x10) && (dir->attrib != 0x08)) {
#ifdef ENABLE_SD_CARD_DEBUG
							serial_puts_f("FILE");
							serial_puts_f("   ");
#endif
							F32_displayMemory(LOW, dir->fileSize);
						} else {
#ifdef ENABLE_SD_CARD_DEBUG
							serial_putstr(
									(dir->attrib == 0x10) ? (int8_t *)"DIR\0" : (int8_t *)"ROOT\0");
#endif
						}
					}
				}
			}
		}

		cluster = (F32_getSetNextCluster(cluster, GET, 0));

		if (cluster > 0x0ffffff6)
		return 0;
		if (cluster == 0) {
#ifdef ENABLE_SD_CARD_DEBUG
			serial_puts_f("\r\nError in getting cluster");
#endif
			return 0;
		}
	}
	return 0;
}

//***************************************************************************
//Function: if flag=READ then to read file from SD card and send contents to UART
//if flag=VERIFY then functions will verify whether a specified file is already existing
//Arguments: flag (READ or VERIFY) and pointer to the file name
//return: 0, if normal operation or flag is READ
//        1, if file is already existing and flag = VERIFY; or if flag=READ and file does not exist
//      2, if file name is incompatible
//***************************************************************************
uint8_t F32_readFile(uint8_t flag, uint8_t *fileName) {
	struct dir_Structure *dir;
	uint32_t cluster, byteCounter = 0, fileSize, firstSector;
	uint16_t k;
	uint8_t j, error;

	//error = F32_convertFileName(fileName); //convert fileName into FAT format
	//if (error)
	//	return 2;

	dir = F32_findFiles(GET_FILE, fileName);//get the file location
	if (dir == 0) {
		if (flag == READ)
		return (1);
		else
		return (0);
	}

	if (flag == VERIFY)
	return (1); //specified file name is already existing

	cluster = (((uint32_t) dir->firstClusterHI) << 16) | dir->firstClusterLO;

	fileSize = dir->fileSize;
#ifdef ENABLE_SD_CARD_DEBUG
	//TX_NEWLINE;
	//TX_NEWLINE;
#endif
	while (1) {
		firstSector = F32_getFirstSector(cluster);

		for (j = 0; j < sectorPerCluster; j++) {
			SD_readSingleBlock(firstSector + j);

			for (k = 0; k < 512; k++) {
#ifdef ENABLE_SD_CARD_DEBUG
				serial_putc(SD_buffer[k]);
#endif
				if ((byteCounter++) >= fileSize)
				return 0;
			}
		}
		cluster = F32_getSetNextCluster(cluster, GET, 0);
		if (cluster == 0) {
#ifdef ENABLE_SD_CARD_DEBUG
			serial_puts_f("\r\nError in getting cluster");
#endif
			return 0;
		}
	}
	return 0;
}

//***************************************************************************
//Function: to convert normal short file name into FAT format
//Arguments: pointer to the file name
//return: 0, if successful else 1.
//***************************************************************************
/*uint8_t F32_convertFileName(uint8_t *fileName) {
 uint8_t fileNameFAT[11];
 uint8_t i, j, k;


 //i = 0;
 //for (j = 0; j < 12; j++){
 //	if (fileName[j] != '.') {
 //		fileNameFAT[i] = fileName[j];
 //		i = i + 1;
 //	}

 //}

 for (j = 0; j < 12; j++)
 if (fileName[j] == '.')
 break;

 if (j > 8) {
 #ifdef ENABLE_SD_CARD_DEBUG
 serial_putstr_f((const int8_t *)"\r\nInvalid fileName..\0");
 #endif
 return 1;

 }

 for (k = 0; k < j; k++) //setting file name
 fileNameFAT[k] = fileName[k];

 for (k = j; k <= 7; k++) //filling file name trail with blanks
 fileNameFAT[k] = ' ';

 j++;
 for (k = 8; k < 11; k++) //setting file extension
 {
 if (fileName[j] != 0)
 fileNameFAT[k] = fileName[j++];
 else
 //filling extension trail with blanks
 while (k < 11)
 fileNameFAT[k++] = ' ';
 }

 for (j = 0; j < 11; j++) //converting small letters to caps
 if ((fileNameFAT[j] >= 0x61) && (fileNameFAT[j] <= 0x7a))
 fileNameFAT[j] -= 0x20;

 for (j = 0; j < 11; j++)
 fileName[j] = fileNameFAT[j];

 return 0;
 }*/

//************************************************************************************
//Function: to create a file in FAT32 format in the root directory if given
//      file name does not exist; if the file already exists then append the data
//Arguments: pointer to the file name
//return: none
//************************************************************************************
uint8_t F32_writeFile(uint8_t *fileName, uint8_t *dataString) {
	uint8_t j, k, data = 0, error, fileCreatedFlag = 0, start = 0, appendFile =
	0, sector = 0;
	uint16_t i, firstClusterHigh = 0, firstClusterLow = 0; //value 0 is assigned just to avoid warning in compilation
	struct dir_Structure *dir;
	uint32_t cluster, nextCluster, prevCluster, firstSector, clusterCount,
	extraMemory;

	j = F32_readFile(VERIFY, fileName);

	if (j == 1) {
#ifdef ENABLE_SD_CARD_DEBUG
		serial_puts_f("\r\nFile already exists, appending data..");
#endif
		appendFile = 1;
		cluster = appendStartCluster;
		clusterCount = 0;
		while (1) {
			nextCluster = F32_getSetNextCluster(cluster, GET, 0);
			if (nextCluster == FAT32_EOF)
			break;
			cluster = nextCluster;
			clusterCount++;
		}

		sector = (fileSize - (clusterCount * sectorPerCluster * bytesPerSector))
		/ bytesPerSector; //last sector number of the last cluster of the file
		start = 1;
	} else if (j == 2)
	return 1; //invalid file name

	else {
#ifdef ENABLE_SD_CARD_DEBUG
		//TX_NEWLINE;
		serial_puts_f("\r\nCreating File..");
#endif
		cluster = F32_getSetFreeCluster(NEXT_FREE, GET, 0);
		if (cluster > totalClusters)
		cluster = rootCluster;

		cluster = F32_searchNextFreeCluster(cluster);
		if (cluster == 0) {
#ifdef ENABLE_SD_CARD_DEBUG
			//TX_NEWLINE;
			serial_puts_f("\r\nNo free cluster!");
#endif
			return 1;
		}
		F32_getSetNextCluster(cluster, SET, FAT32_EOF); //last cluster of the file, marked EOF

		firstClusterHigh = (uint16_t) ((cluster & 0xffff0000) >> 16);
		firstClusterLow = (uint16_t) (cluster & 0x0000ffff);
		fileSize = 0;
	}

	k = 0;

	while (1) {
		if (start) {
			start = 0;
			SD_startBlock = F32_getFirstSector(cluster) + sector;
			SD_readSingleBlock(SD_startBlock);
			i = fileSize % bytesPerSector;
			j = sector;
		} else {
			SD_startBlock = F32_getFirstSector(cluster);
			i = 0;
			j = 0;
		}

		do {
			data = dataString[k++];
#ifdef ENABLE_SD_CARD_DEBUG
			serial_putchar(data); // was data
#endif
			SD_buffer[i++] = data;
			fileSize++;

			if (i >= 512) //though 'i' will never become greater than 512, it's kept here to avoid
			{ //infinite loop in case it happens to be greater than 512 due to some data corruption
				i = 0;
				error = SD_writeSingleBlock(SD_startBlock);
				j++;
				if (j == sectorPerCluster) {
					j = 0;
					break;
				}
				SD_startBlock++;
			}
		}while ((data != '\n') && (k < MAX_STRING_SIZE)); //stop when newline character is found
		//or when string size limit reached

		if ((data == '\n') || (k >= MAX_STRING_SIZE)) {
			for (; i < 512; i++) //fill the rest of the buffer with 0x00
			SD_buffer[i] = 0x00;
			error = SD_writeSingleBlock(SD_startBlock);

			break;
		}

		prevCluster = cluster;

		cluster = F32_searchNextFreeCluster(prevCluster); //look for a free cluster starting from the current cluster

		if (cluster == 0) {
#ifdef ENABLE_SD_CARD_DEBUG
			//TX_NEWLINE;
			serial_puts_f("\r\nNo free cluster!");
#endif
			return 1;
		}

		F32_getSetNextCluster(prevCluster, SET, cluster);
		F32_getSetNextCluster(cluster, SET, FAT32_EOF); //last cluster of the file, marked EOF
	}

	F32_getSetFreeCluster(NEXT_FREE, SET, cluster); //update FSinfo next free cluster entry

	error = getDateTime_FAT();//get current date & time from the RTC
	if (error) {
		dateFAT = 0;
		timeFAT = 0;
	}

	if (appendFile) //executes this loop if file is to be appended
	{
		SD_readSingleBlock(appendFileSector);
		dir = (struct dir_Structure *) &SD_buffer[appendFileLocation];

		dir->lastAccessDate = 0; //date of last access ignored
		dir->writeTime = timeFAT;//setting new time of last write, obtained from RTC
		dir->writeDate = dateFAT;//setting new date of last write, obtained from RTC
		extraMemory = fileSize - dir->fileSize;
		dir->fileSize = fileSize;
		SD_writeSingleBlock(appendFileSector);
		F32_freeMemoryUpdate(REMOVE, extraMemory);//updating free memory count in FSinfo sector;

#ifdef ENABLE_SD_CARD_DEBUG
		//TX_NEWLINE;
		serial_puts_f("\r\nFile appended!");
#endif
		return 0;
	}

//executes following portion when new file is created

	prevCluster = rootCluster;//root cluster

	while (1) {
		firstSector = F32_getFirstSector(prevCluster);

		for (sector = 0; sector < sectorPerCluster; sector++) {
			SD_readSingleBlock(firstSector + sector);

			for (i = 0; i < bytesPerSector; i += 32) {
				dir = (struct dir_Structure *) &SD_buffer[i];

				if (fileCreatedFlag) //to mark last directory entry with 0x00 (empty) mark
				{ //indicating end of the directory file list
				  //dir->name[0] = EMPTY;
				  //SD_writeSingleBlock (firstSector + sector);
					return 0;
				}

				if ((dir->name[0] == EMPTY) || (dir->name[0] == DELETED)) //looking for an empty slot to enter file info
				{
					for (j = 0; j < 11; j++)
					dir->name[j] = fileName[j];
					dir->attrib = ATTR_ARCHIVE; //setting file attribute as 'archive'
					dir->NTreserved = 0;//always set to 0
					dir->timeTenth = 0;//always set to 0
					dir->createTime = timeFAT;//setting time of file creation, obtained from RTC
					dir->createDate = dateFAT;//setting date of file creation, obtained from RTC
					dir->lastAccessDate = 0;//date of last access ignored
					dir->writeTime = timeFAT;//setting new time of last write, obtained from RTC
					dir->writeDate = dateFAT;//setting new date of last write, obtained from RTC
					dir->firstClusterHI = firstClusterHigh;
					dir->firstClusterLO = firstClusterLow;
					dir->fileSize = fileSize;

					SD_writeSingleBlock(firstSector + sector);
					fileCreatedFlag = 1;
#ifdef ENABLE_SD_CARD_DEBUG
					//TX_NEWLINE;
					//TX_NEWLINE;
					serial_puts_f("\r\nFile Created! ");
#endif

					F32_freeMemoryUpdate(REMOVE, fileSize); //updating free memory count in FSinfo sector

				}
			}
		}

		cluster = F32_getSetNextCluster(prevCluster, GET, 0);

		if (cluster > 0x0ffffff6) {
			if (cluster == FAT32_EOF) //this situation will come when total files in root is multiple of (32*sectorPerCluster)
			{
				cluster = F32_searchNextFreeCluster(prevCluster); //find next cluster for root directory entries
				F32_getSetNextCluster(prevCluster, SET, cluster);//link the new cluster of root to the previous cluster
				F32_getSetNextCluster(cluster, SET, FAT32_EOF);//set the new cluster as end of the root directory
			}

			else {
#ifdef ENABLE_SD_CARD_DEBUG
				serial_puts_f("\r\nEnd of Cluster Chain");
#endif
				return 1;
			}
		}
		if (cluster == 0) {
#ifdef ENABLE_SD_CARD_DEBUG
			serial_puts_f("\r\nError in getting cluster");
#endif
			return 1;
		}

		prevCluster = cluster;
	}

	return 0;
}

//***************************************************************************
//Function: to search for the next free cluster in the root directory
//          starting from a specified cluster
//Arguments: Starting cluster
//return: the next free cluster
//****************************************************************
uint32_t F32_searchNextFreeCluster(uint32_t startCluster) {
	uint32_t cluster, *value, sector;
	uint8_t i;

	startCluster -= (startCluster % 128); //to start with the first file in a FAT sector
	for (cluster = startCluster; cluster < totalClusters; cluster += 128) {
		sector = unusedSectors + reservedSectorCount
		+ ((cluster * 4) / bytesPerSector);
		SD_readSingleBlock(sector);
		for (i = 0; i < 128; i++) {
			value = (uint32_t *) &SD_buffer[i * 4];
			if (((*value) & 0x0fffffff) == 0)
			return (cluster + i);
		}
	}

	return 0;
}

//************************************************************
//Function: To convert the uint32_t value of memory into
//          text string and send to UART
//Arguments: 1. uint8_t flag. If flag is HIGH, memory will be displayed in KBytes, else in Bytes.
//       2. uint32_t memory value
//return: none
//************************************************************
void F32_displayMemory(uint8_t flag, uint32_t memory) {
	uint8_t memoryString[] = "              Bytes"; //19 character long string for memory display
	uint8_t i;
	for (i = 12; i > 0; i--)//converting freeMemory into ASCII string
	{
		if (i == 5 || i == 9) {
			memoryString[i - 1] = ',';
			i--;
		}
		memoryString[i - 1] = (memory % 10) | 0x30;
		memory /= 10;
		if (memory == 0)
		break;
	}
	if (flag == HIGH)
	memoryString[13] = 'K';
#if defined(ENABLE_SERIAL) || defined(ENABLE_SERIAL_POLL)
	serial_puts(memoryString);
#endif
}

//********************************************************************
//Function: to delete a specified file from the root directory
//Arguments: pointer to the file name
//return: none
//********************************************************************
void F32_deleteFile(uint8_t *fileName) {
	//uint8_t error;

	//error = F32_convertFileName(fileName);
	//if (error)
	//	return;

	F32_findFiles(DELETE, fileName);
}

//********************************************************************
//Function: update the free memory count in the FSinfo sector.
//      Whenever a file is deleted or created, this function will be called
//      to ADD or REMOVE clusters occupied by the file
//Arguments: #1.flag ADD or REMOVE #2.file size in Bytes
//return: none
//********************************************************************
void F32_freeMemoryUpdate(uint8_t flag, uint32_t size) {
	uint32_t freeClusters;
	//convert file size into number of clusters occupied
	if ((size % 512) == 0)
	size = size / 512;
	else
	size = (size / 512) + 1;
	if ((size % 8) == 0)
	size = size / 8;
	else
	size = (size / 8) + 1;

	if (freeClusterCountUpdated) {
		freeClusters = F32_getSetFreeCluster(TOTAL_FREE, GET, 0);
		if (flag == ADD)
		freeClusters = freeClusters + size;
		else
		//when flag = REMOVE
		freeClusters = freeClusters - size;
		F32_getSetFreeCluster(TOTAL_FREE, SET, freeClusters);
	}
}

//******** END ****** www.dharmanitech.com *****
#endif //ENABLE_FAT32
//--
#ifdef ENABLE_RFM12B
//Setup a simple timeout
inline void timeout_init(void) {
	TCCR2B = 0; //disable the timer
	TCNT2 = 0;//start counting from 0
	TCCR2B = 7;//turn the timer on (prescaler 1024)
	TIFR2 = (1 << TOV2);//clear the overflow flag
}

//Test if the timeout expired
inline uint8_t timeout(void) {
	return (TIFR2 & (1 << TOV2)); //return non-zero if the timer overflowed
}

//Test if the module is ready for sending / receiving next byte
uint8_t rf12_is_ready(void) {
	//RF_SPI_LOW_SPEED;
	cbi(RF_CS_PORT, RF_CS_PIN);
	//enable the module
	_delay_us(1);//let it respond
	uint8_t r = bit_get(MISO_PORT, MISO);//read the SO line (first bit of status word)
	sbi(RF_CS_PORT, RF_CS_PIN);
	//disable the module
	return r;//return the value of the first bit
}

//Exchange a word (two bytes, big-endian) with the module
uint16_t rf12_trans(uint16_t to_send) {
	uint16_t received = 0; //buffer for data we are going to read
	//RF_SPI_LOW_SPEED;
	cbi(RF_CS_PORT, RF_CS_PIN);
	//enable the module
	SPDR = (to_send >> 8) & 0xFF;//send the upper byte
	while (!(SPSR & (1 << SPIF)))
	;//wait until the transmission is complete
	received = SPDR;//store received byte
	received <<= 8;//move it on its proper position
	SPDR = (0xFF & to_send);//send the lower byte
	while (!(SPSR & (1 << SPIF)))
	;//wait until the transmission is complete
	received |= SPDR;//store received byte
	sbi(RF_CS_PORT, RF_CS_PIN);
	//disable the module
	return received;//return the data from the module
}

//send one byte through the radio
void rf12_txbyte(uint8_t b) {
	while (!rf12_is_ready()) //wait while the module is not ready...
	if (timeout())//...if it is too long...
	return;//...abort the operation
	rf12_trans(0xB800 | b);//send the desired byte
}

//receive one byte through the radio
uint8_t rf12_rxbyte(void) {
	while (!rf12_is_ready()) //wait while the module is not ready...
	if (timeout())//...if it is too long...
	return 0;//...abort the operation
	return rf12_trans(0xB000);//read the byte from the receive FIFO
}

//adaptation to use the statements from rf12b_code.pdf
#define RFXX_WRT_CMD(x) rf12_trans(x)

//prepare the radio module
//you must execute spi_init() before this
void radio_config(void) {
#ifdef RED_LED_DDR
	// set the pin as output
	sbi(RED_LED_DDR, RED_LED_PIN);
#endif
#ifdef GREEN_LED_DDR
	// set the pin as output
	sbi(GREEN_LED_DDR, GREEN_LED_PIN);
#endif
	sbi(RF_CS_PORT, RF_CS_PIN); //disable RFM12B module
	RF_SPI_LOW_SPEED;// set the SPI at low speed
	_delay_ms(10);//wait a moment
	rf12_trans(0xFE00);//send the reset command
	_delay_ms(150);//wait for reset to complete

	//Example setup
#ifdef RF_FREQ_868MHz
	RFXX_WRT_CMD(0x80E7); //EL,EF,868band,12.0pF
#endif
#ifdef RF_FREQ_433MHz
	RFXX_WRT_CMD(0x80D8); //EL,EF,433band,12.5pF
#endif
	RFXX_WRT_CMD(0x8219); //!er,!ebb,!ET,ES,EX,!eb,!ew,DC
#ifdef RF_FREQ_868MHz
	RFXX_WRT_CMD(0xA67C); //868MHz
#endif
#ifdef RF_FREQ_433MHz
	RFXX_WRT_CMD(0xA640); //433MHz
#endif
	//RFXX_WRT_CMD(0xC611);//19.2kbps
	//RFXX_WRT_CMD(0xC623);//9.6kbps
	RFXX_WRT_CMD(0xC603);
	//115200 = 3, 4800bps = 48, 19200 = 12, 9600 = 23, 28800 = C, 38400 = 9
	RFXX_WRT_CMD(0x94A0);//VDI,FAST,134kHz,0dBm,-103dBm
	RFXX_WRT_CMD(0xC2AC);//AL,!ml,DIG,DQD4
	RFXX_WRT_CMD(0xCA81);//FIFO8,SYNC,!ff,DR
	RFXX_WRT_CMD(0xCED4);//SYNC=2DD4;
	RFXX_WRT_CMD(0xC483);//@PWR,NO RSTRIC,!st,!fi,OE,EN
	RFXX_WRT_CMD(0x9850);//!mp,90kHz,MAX OUT
	RFXX_WRT_CMD(0xE000);//NOT USE
	RFXX_WRT_CMD(0xC800);//NOT USE
	RFXX_WRT_CMD(0xC040);//1.66MHz,2.2V
}

//Send data packet through the radio
void radio_send(uint8_t volatile * buffer, uint8_t len) {
	timeout_init(); //setup the timeout timer
	rf12_trans(0x8238);//start transmitter
	rf12_txbyte(0xAA);//send the preamble, four times 0xAA
	rf12_txbyte(0xAA);
	rf12_txbyte(0xAA);
	rf12_txbyte(0xAA);
	rf12_txbyte(0x2D);//then the predefined sync words
	rf12_txbyte(0xD4);
	rf12_txbyte(0xC0);//and a secret 0xC0DE
	rf12_txbyte(0xDE);
	rf12_txbyte(len);//next the length of the data
	while (len--)
	rf12_txbyte(*buffer++);//and then the data itself
	rf12_txbyte(0x00);//finish the transmission with two dummy bytes
	rf12_txbyte(0x00);
	rf12_txbyte(0x00);
	rf12_txbyte(0x00);
	while (!rf12_is_ready() && !timeout())
	;//wait for the completion of the send operation
	rf12_trans(0x8208);//go to idle, disable the transmitter
}

//receive data packet through the radio
int16_t radio_rcv(uint8_t volatile * buffer, uint8_t max_len) {
	uint8_t len, i, timeout_counter;
	timeout_init(); //setup the timeout timer
	timeout_counter = 3;//after some timeouts the procedure will give-up
	while (1) {
		rf12_trans(0x8208); //send the module to the idle
		rf12_trans(0x82C8);//and restart as a receiver
		_delay_us(150);
		rf12_trans(0xCA81);//disable the FIFO, and...
		rf12_trans(0xCA83);//...enable again, just to clear it
		while (1)//wait for the transmission to start
		{
			if (timeout()) //if the timeout occurred...
			{
				if (!(timeout_counter--)) //count it, and if no more trials remain
				{
					rf12_trans(0x8208); //put the module to the idle state
					return -1;//and return an error code
				}
				timeout_init(); //setup the timer for the next measurement
			}
			if (rf12_is_ready())
			break; //proceed if the module captured some data
		}
		timeout_init(); //restart the timeout timer
		i = rf12_trans(0xB000);//retrieve the received byte
		if (i != 0xC0)
		continue;//test if its correct
		i = rf12_rxbyte();//try to receive the next byte
		if (i != 0xDE)
		continue;//test if its correct
		len = rf12_rxbyte();//try to receive the 'length' byte
		if (len > max_len)
		continue;//test if the passed buffer is large enough
		//if all the bytes received so far are correct, we may assume that the
		//transmission is not a "false positive", so the program will continue reception
		break;
	}
	i = len; //we re going to read 'len' bytes
	while (i--)//loop while there is anything more to read
	{
		*buffer++ = rf12_rxbyte(); //receive next byte, and advance write pointer
		if (timeout())//if a timeout occured
		{
			rf12_trans(0x8208); //stop receiving
			return -2;//and return error code
		}
	}
	rf12_trans(0x8208); //put the module to the idle state
	return len;//return packet length
}

#endif //ENABLE_RFM12B
//--
#ifdef ENABLE_GPL_RFM12B
#if RFM12_USE_RX_CALLBACK
	volatile static (*rfm12_rx_callback_func)(uint8_t, uint8_t *) = (void *)0x0000;
	void rfm12_set_callback ((*in_func)(uint8_t, uint8_t *)) {
		rfm12_rx_callback_func = in_func;
	}
#endif


/************************
 * library internal globals
*/

//! Buffer and status for packet transmission.
rf_tx_buffer_t rf_tx_buffer;

//if receive mode is not disabled (default)
#if !(RFM12_TRANSMIT_ONLY)
	//! Buffers and status to receive packets.
	rf_rx_buffer_t rf_rx_buffers[2];
#endif /* RFM12_TRANSMIT_ONLY */

//! Global control and status.
rfm12_control_t ctrl;


/************************
 * load other core and external components
 * (putting them directly into here allows GCC to optimize better)
*/

/* include spi functions into here */
//hardware spi helper macros
#define SS_ASSERT() RF_PORT_SS &= ~(1<<RF_BIT_SS)
#define SS_RELEASE() RF_PORT_SS |= (1<<RF_BIT_SS)


#if RFM12_SPI_SOFTWARE
/* @description Actual sending function to send raw data to the Module
 * @note do NOT call this function directly, unless you know what you're doing.
 */
static uint8_t spi_data(uint8_t c) {
	uint8_t x, d = d;
	for (x = 0; x < 8; x++) {
		if (c & 0x80) {
			MOSI_PORT |= (1<<MOSI);
		} else {
			MOSI_PORT &= ~(1<<MOSI);
		}
		SCK_PORT |= (1<<SCK);
		d <<= 1;
		if (MISO_PIN & (1<<MISO)) {
			d |= 1;
		}
		SCK_PORT &= ~(1<<SCK);
		c <<= 1;
	}
	return d;
}
#endif


//non-inlined version of rfm12_data
//warning: without the attribute, gcc will inline this even if -Os is set
static void __attribute__ ((noinline)) rfm12_data(uint16_t d) {
	SS_ASSERT();
	#if !(RFM12_SPI_SOFTWARE)
		SPDR = d >> 8;
		while (!(SPSR & (1<<SPIF)));

		SPDR = d & 0xff;
		while (!(SPSR & (1<<SPIF)));
	#else
		spi_data(d >> 8);
		spi_data(d & 0xff);
	#endif
	SS_RELEASE();
}


//non-inlined version of rfm12_read
//warning: without the attribute, gcc will inline this even if -Os is set
static uint16_t __attribute__ ((noinline)) rfm12_read(uint16_t c) {
	uint16_t retval;
	SS_ASSERT();

	#if !(RFM12_SPI_SOFTWARE)
		SPDR = c >> 8;
		while (!(SPSR & (1<<SPIF)));
		retval = SPDR << 8;
		SPDR = c & 0xff;
		while (!(SPSR & (1<<SPIF)));
		retval |= SPDR;
	#else
		retval = spi_data(c >> 8);
		retval <<= 8;
		retval |= spi_data(c & 0xff);
	#endif
	SS_RELEASE();
	return retval;
}


/* @description reads the upper 8 bits of the status
 * register (the interrupt flags)
 */
static uint8_t rfm12_read_int_flags_inline(void) {
	#if !(RFM12_SPI_SOFTWARE)
		SS_ASSERT();
		SPDR = 0;
		while (!(SPSR & (1<<SPIF)));
		SS_RELEASE();
		return SPDR;
	#else
		SS_ASSERT();
		unsigned char x, d = d;
		MOSI_PORT &= ~(1<<MOSI);
		for (x = 0; x < 8; x++) {
			SCK_PORT |= (1<<SCK);
			d <<= 1;
			if (MISO_PIN & (1<<MISO)) {
				d |= 1;
			}
			SCK_PORT &= ~(1<<SCK);
		}
		SS_RELEASE();
		return d;
	#endif
}

static void spi_init(void) {
#define PORT_SPI PORTB
#define DDR_SPI  DDRB
	MOSI_DDR |= (_BV(MOSI));
	SCK_DDR  |= (_BV(SCK));
	#if !(RFM12_SPI_SOFTWARE)
		PORT_SPI |= (_BV(SS));
		DDR_SPI  |= (_BV(SS));
	#endif

	MISO_DDR &= ~(_BV(MISO));

	#if !(RFM12_SPI_SOFTWARE)
		SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0); //SPI Master, clk/16
	#endif
}

/*
 * include control / init functions into here
 * all of the stuff in there is optional, so there's no code-bloat.
*/
//#define RFM12_LIVECTRL_HOST 1//if we are building for the microcontroller, we are the host.
//#include "include/rfm12_livectrl.c"
#if RFM12_LIVECTRL

#if RFM12_LIVECTRL_CLIENT

	#if __AVR__
		//#include <avr/pgmspace.h>
		//yes, we include the c file because of ease of configuration this way
		//the C preprocessor can decide wether we need the file or not.
		//#include "../xprintf/xprintf.c"
	#else
		#define PSTR(s)    s
		#define strcpy_P   strcpy
		#define xsprintf_P sprintf
	#endif

	void baseband_to_string(char *s, uint16_t var) {
		switch (var) {
			case RFM12_BAND_315:
				strcpy_P(s, PSTR("315MHz"));
				break;
			case RFM12_BAND_433:
				strcpy_P(s, PSTR("433MHz"));
				break;
			case RFM12_BAND_868:
				strcpy_P(s, PSTR("868MHz"));
				break;
			case RFM12_BAND_915:
				strcpy_P(s, PSTR("915MHz"));
				break;
			default:
				*s = 0;
				break;
		}
	}


	void frequency_to_string(char *s, uint16_t val) {
		//Band [MHz] C1 C2
		//315         1 31
		//433         1 43
		//868         2 43
		//915         3 30
		//f0 = 10 * C1 * (C2 + F/4000) [MHz]
		//f0 = 10 * C1 * (C2*1000 + F/4)    [kHz]

		//433:
		//f0 = 430 + F/400  [MHz]
		//f0 = 430000 + F * 2.5 [kHz]

		//915:
		//f0 = 900 + F * (3 /400)  [MHz]
		//f0 = 900000 + F * 7.5 [kHz]

		//868:
		//f0 = 860 + F * (2 /400)  [MHz]
		//f0 = 860000 + F * 5 [kHz]


		uint16_t mhz;
		uint16_t khz;
		uint16_t band_setting = livectrl_cmds[RFM12_LIVECTRL_BASEBAND].current_value;

		if (band_setting == RFM12_BAND_433) {
			mhz = 430 + val / 400;
			khz = ((val % 400) * 5) / 2;
		} else if (band_setting == RFM12_BAND_915) {
			val *= 3;
			mhz = 900 + val / 400;
			khz = ((val % 400) * 5) / 2;
		} else if (band_setting == RFM12_BAND_868) {
			val *= 2;
			mhz = 860 + val / 400;
			khz = ((val % 400) * 5) / 2;
		} else {
			mhz = 0;
			khz = 0;
		}
		#if (DISP_LEN == 8)
			xsprintf_P(s, PSTR("%3d.%03d"), mhz, khz);
		#else
			xsprintf_P(s, PSTR("%3d.%03d MHz"), mhz, khz);
		#endif
	}

	void datarate_to_string(char *s, uint16_t val) {
		/*
			4. Data Rate Command
			Bit 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0 POR
			1 1 0 0 0 1 1 0 cs r6 r5 r4 r3 r2 r1 r0 C623h
			The actual bit rate in transmit mode and the expected bit rate of the received data stream in receive mode is determined by the 7-bit
			parameter R (bits r6 to r0) and bit cs.
			BR = 10000 / 29 / (R+1) / (1+cs*7) [kbps]
		*/

		uint16_t n;

		if (val & 0x80) {
			//low bitrate
			n = 29 * 8;
		} else {
			//high bitrate
			n = 29;
		}
		val &= 0x7f;

		n *= val;

		uint32_t bitrate = 10000000ul / n;

		xsprintf_P(s, PSTR("%4ld bps"), bitrate);
	}


	#define FULLSCALE_TX_POWER 8;

	void tx_power_to_string(char *s, uint16_t var) {
		int16_t val = var;
		val *= -3;
		val += FULLSCALE_TX_POWER;

		xsprintf_P(s, PSTR("%3d dBm"), val);
	}

	void fsk_shift_to_string(char *s, uint16_t val) {
		val >>= 4; //right adjust

		val = (val + 1) * 15;

		xsprintf_P(s, PSTR("+-%d kHz"), val);
	}

	void lna_to_string(char *s, uint16_t var) {
		uint8_t val = var;
		val >>= 3; //right adjust

		switch (val) {
			case 1: val = 6; break;
			case 2: val = 14; break;
			case 3: val = 20; break;
		}

		xsprintf_P(s, PSTR("%3d dB"), -val);
	}

	void rssi_to_string(char *s, uint16_t val) {
		xsprintf_P(s, PSTR("%4d dBm"), -61 - (7 - val) * 6);
	}

	void filter_bw_to_string(char *s, uint16_t val) {
		val >>= 5; //right adjust

		val = ((7 - val) * 200) / 3;

		xsprintf_P(s, PSTR("%3d kHz"), val);
	}

	void xtal_load_to_string(char *s, uint16_t val) {
		val += 1;
		uint8_t pf = val / 2 + 8;
		uint8_t n = 0;
		if(val & 0x01) n = 5;
		xsprintf_P(s, PSTR("%2d.%dpF"), pf, n);
	}


	#define IFCLIENT(a,b,c,d,e) a,b,c,d,e
#else // RFM12_LIVECTRL_CLIENT
	#define IFCLIENT(a,b,c,d,e)
#endif // RFM12_LIVECTRL_CLIENT

#if RFM12_LIVECTRL_HOST
	#define IFHOST(a) a
#else
	#define IFHOST(a) 0
#endif

livectrl_cmd_t livectrl_cmds[] = {
	{ RFM12_CMD_CFG,       RFM12_CFG_BAND_MASK,     IFHOST(&ctrl.cfg_shadow),    RFM12_BASEBAND,                        IFCLIENT(0x00,   0x30, 0x10, "Baseband"  , baseband_to_string  )},
	{ RFM12_CMD_CFG,       RFM12_CFG_XTAL_MASK,     IFHOST(&ctrl.cfg_shadow),    RFM12_XTAL_LOAD,                       IFCLIENT(0x00,   0x0f,    1, "Xtal Load" , xtal_load_to_string  )},
	{ RFM12_CMD_FREQUENCY, RFM12_FREQUENCY_MASK,    0,                           RFM12_FREQUENCY_CALC(RFM12_FREQUENCY), IFCLIENT(0x00, 0x0fff,    4, "Frequency" , frequency_to_string )},
	{ RFM12_CMD_DATARATE,  RFM12_DATARATE_MASK,     0,                           DATARATE_VALUE,                        IFCLIENT(0x03,   0xff,    1, "Data rate" , datarate_to_string  )},
	{ RFM12_CMD_TXCONF,    RFM12_TXCONF_POWER_MASK, IFHOST(&ctrl.txconf_shadow), RFM12_POWER,                           IFCLIENT(0x00,   0x07,    1, "TX Power"  , tx_power_to_string  )},
	{ RFM12_CMD_TXCONF,    RFM12_TXCONF_FSK_MASK,   IFHOST(&ctrl.txconf_shadow), RFM12_TXCONF_FS_CALC(FSK_SHIFT),       IFCLIENT(0x00,   0xf0, 0x10, "FSK Shift" , fsk_shift_to_string )},
	{ RFM12_CMD_RXCTRL,    RFM12_RXCTRL_LNA_MASK,   IFHOST(&ctrl.rxctrl_shadow), RFM12_LNA_GAIN,                        IFCLIENT(0x00,   0x18, 0x08, "LNA"       , lna_to_string      )},
	{ RFM12_CMD_RXCTRL,    RFM12_RXCTRL_RSSI_MASK,  IFHOST(&ctrl.rxctrl_shadow), RFM12_RSSI_THRESHOLD,                  IFCLIENT(0x00,   0x07,    1, "RSSI"      , rssi_to_string      )},
	{ RFM12_CMD_RXCTRL,    RFM12_RXCTRL_BW_MASK,    IFHOST(&ctrl.rxctrl_shadow), RFM12_FILTER_BW,                       IFCLIENT(0x20,   0xC0, 0x20, "Filter BW" , filter_bw_to_string )},
};


#if RFM12_LIVECTRL_LOAD_SAVE_SETTINGS
	#include <avr/eeprom.h>

	void rfm12_save_settings() {
		uint8_t x;
		uint16_t checksumm = 0;

		for (x = 0; x < NUM_LIVECTRL_CMDS; x++) {
			uint16_t val = livectrl_cmds[x].current_value;
			checksumm += val;
			eeprom_write_word((void*)(2 * x), val);
		}

		eeprom_write_word((void*)(2 * x), checksumm);
	}

	void rfm12_load_settings() {

		uint8_t x;
		uint16_t val;
		uint16_t checksumm = 0;

		for (x = 0; x < NUM_LIVECTRL_CMDS; x++) {
			val = eeprom_read_word((void*)(2 * x));
			checksumm += val;
		}

		val = eeprom_read_word((void*)(2 * x));
		if (val != checksumm) return; //eeprom invalid, keep default values from array

		//set the settings if eeprom valid
		for (x = 0; x < NUM_LIVECTRL_CMDS; x++) {
			val = eeprom_read_word((void*)(2 * x));
			rfm12_livectrl(x, val);
		}
	}
#endif

#if RFM12_LIVECTRL_HOST
	void rfm12_data_safe(uint16_t d) {
		//disable the interrupt (as we're working directly with the transceiver now)
		RFM12_INT_OFF();
		rfm12_data(d);
		RFM12_INT_ON();
	}


	void rfm12_livectrl(uint8_t cmd, uint16_t value) {
		uint16_t tmp = 0;
		livectrl_cmd_t  *livectrl_cmd = &livectrl_cmds[cmd];

		livectrl_cmd->current_value = value; //update current value

		//the shadow register is somewhat redundant with the current value,
		//but it makes sense never the less:
		//the current_value only saves the bits for this one setting (for menu,saving,loding settings)
		//while the shadow register keeps track of ALL bits the rfm12 has in that register.
		//the shadow will also be used from rfm12_tick or maybe the interrupt

		if (livectrl_cmd->shadow_register) {
			tmp = *livectrl_cmd->shadow_register;         //load shadow value if any
			tmp &= ~livectrl_cmd->rfm12_hw_parameter_mask;//clear parameter bits
		}
		tmp |= livectrl_cmd->rfm12_hw_command | (livectrl_cmd->rfm12_hw_parameter_mask & value);

		*livectrl_cmd->shadow_register = tmp;

		rfm12_data_safe(tmp);
	}
#endif // RFM12_LIVECTRL_HOST

#if RFM12_LIVECTRL_CLIENT
	void rfm12_livectrl_get_parameter_string(uint8_t cmd, char *str) {
		livectrl_cmd_t *livectrl_cmd = &livectrl_cmds[cmd];

		uint16_t var = livectrl_cmd->current_value;
		livectrl_cmd->to_string(str, var);
	}
#endif // RFM12_LIVECTRL_CLIENT


#endif // RFM12_LIVECTRL
// end rfm12_livectrl.c

/*
 * include extra features here
 * all of the stuff in there is optional, so there's no code-bloat..
*/
//#include "include/rfm12_extra.c"
/************************
 * amplitude modulation receive mode
*/

#if RFM12_RECEIVE_ASK
	//! The ASK mode receive buffer structure.
	/** You will need to poll the state field of this structure to determine
	* if data is available, see \ref ask_defines "ASK mode defines". \n
	* Received data can be read from the buf field.
	* It is necessary to reset the state field  to RFM12_ASK_STATE_EMPTY after reading.
	*
	* \note You need to define RFM12_RECEIVE_ASK as 1 to enable this.
	*/
	rfm12_rfrxbuf_t ask_rxbuf;


	//! ASK mode ADC interrupt.
	/** This interrupt function directly measures the receive signal strength
	* on an analog output pin of the rf12 ic.
	*
	* You will need to solder something onto your rf12 module to make this to work.
	*
	* \note You need to define RFM12_RECEIVE_ASK as 1 to enable this.
	* \see adc_init() and rfm12_rfrxbuf_t
	*/
	ISR(ADC_vect, ISR_NOBLOCK) {
		static uint16_t adc_average;
		static uint8_t pulse_timer;
		uint8_t value;
		static uint8_t oldvalue;
		static uint8_t ignore;
		uint16_t adc;


		ADCSRA = (1<<ADEN) | (1<<ADFR) | (0<<ADIE) //start free running mode
				| (1<<ADPS2) | (1<<ADPS1);  //preescaler to clk/64
											//samplerate = 16MHz/(64*13) = 19231 Hz


		adc = ADC;

		adc_average -= adc_average / 64;
		adc_average +=adc;

		value = (ADC > ((adc_average / 64) + 50)) ? 1 : 0;

		if (value) {
			PORTD |= (1<<PD7);
		} else {
			PORTD &= ~(1<<PD7);
		}


		if (TCNT0 > 0xE0) {
			ignore = 0;
		}

		if (ask_rxbuf.state == RFM12_ASK_STATE_EMPTY) {
			if (value && (!ignore)) {
				//pulse_timer = 0;
				TCNT0 = 0;
				ask_rxbuf.p = 0;
				ask_rxbuf.state = RFM12_ASK_STATE_RECEIVING;
			}
		} else if (ask_rxbuf.state == RFM12_ASK_STATE_FULL) {
			if (value) {
				TCNT0 = 0;
				ignore = 1;
			}

		} else if (ask_rxbuf.state == RFM12_ASK_STATE_RECEIVING) {
			if (value != oldvalue) {

				ask_rxbuf.buf[ask_rxbuf.p] = TCNT0;
				TCNT0 = 0;
				//pulse_timer = 0;
				if (ask_rxbuf.p != (RFM12_ASK_RFRXBUF_SIZE - 1) ) {
					ask_rxbuf.p++;
				}
			} else if (TCNT0 > 0xe0) {
				//if( !value ){
				//PORTD |= (1<<PD6);
					ask_rxbuf.state = RFM12_ASK_STATE_FULL;
				//}else{
				//	ask_rxbuf.state = STATE_EMPTY;
				//}
			}
		}

		oldvalue = value;

		ADCSRA = (1<<ADEN) | (1<<ADFR) | (1<<ADIE) //start free running mode
				| (1<<ADPS2) | (1<<ADPS1);  //preescaler to clk/64
											//samplerate = 16MHz/(64*13) = 19231 Hz

	}


	//! ASK mode ADC interrupt setup.
	/** This will setup the ADC interrupt to receive ASK modulated signals.
	* rfm12_init() calls this function automatically if ASK receive mode is enabled.
	*
	* \note You need to define RFM12_RECEIVE_ASK as 1 to enable this.
	* \see ISR(ADC_vect, ISR_NOBLOCK) and rfm12_rfrxbuf_t
	*/
	void adc_init(void) {
		ADMUX  = (1<<REFS0) | (1<<REFS1); //Internal 2.56V Reference, MUX0

		ADCSRA = (1<<ADEN) | (1<<ADSC) | (1<<ADFR) | (1<<ADIE) //start free running mode
				| (1<<ADPS2) | (1<<ADPS1);  //preescaler to clk/64
											//samplerate = 16MHz/(64*13) = 19231 Hz

	}
#endif /* RFM12_RECEIVE_ASK */


/************************
 * ASK modulated raw tx mode
*/

#if RFM12_TRANSMIT_ASK
	//! En- or disable ASK transmissions.
	/** When enabling ASK tx mode, this function puts the internal state machine
	* into transmit mode and disables the interrupt.
	* Otherwise it will restore normale operation.
	*
	* \param [setting] Pass 1 to enable the raw mode, 0 to disable it.
	* \note You need to define RFM12_TRANSMIT_ASK as 1 to enable this.
	* \warning This will interfere with the wakeup timer feature.
	* \todo Use power management shadow register if the wakeup timer feature is enabled.
	* \see rfm12_tx_on() and rfm12_tx_off()
	*/
	void rfm12_ask_tx_mode(uint8_t setting) {
		if (setting)
		{
		#if 0
			/* disable the receiver */
			rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT);

			/* fill preamble into buffer */
			rfm12_data(RFM12_CMD_TX | PREAMBLE);
			rfm12_data(RFM12_CMD_TX | PREAMBLE);
		#endif
			ctrl.rfm12_state = STATE_TX;
			RFM12_INT_OFF();
		} else
		{
			/* re-enable the receiver */
			rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT | RFM12_PWRMGT_ER);
			RFM12_INT_ON();
			ctrl.rfm12_state = STATE_RX_IDLE;
		}
	}


	//! Enable the transmitter immediately (ASK transmission mode).
	/** This will send out the current buffer contents.
	* This function is used to emulate amplitude modulated signals.
	*
	* \note You need to define RFM12_TRANSMIT_ASK as 1 to enable this.
	* \warning This will interfere with the wakeup timer feature.
	* \todo Use power management shadow register if the wakeup timer feature is enabled.
	* \see rfm12_tx_off() and rfm12_ask_tx_mode()
	*/
	inline void rfm12_tx_on(void) {
		/* set enable transmission bit now. */
		rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT | RFM12_PWRMGT_ET | RFM12_PWRMGT_ES | RFM12_PWRMGT_EX);
	}


	//! Set default power mode (usually transmitter off, receiver on).
	/** This will usually stop a transmission.
	* This function is used to emulate amplitude modulated signals.
	*
	* \note You need to define RFM12_TRANSMIT_ASK as 1 to enable this.
	* \warning This will interfere with the wakeup timer feature.
	* \todo Use power management shadow register if the wakeup timer feature is enabled.
	* \see rfm12_tx_on() and rfm12_ask_tx_mode()
	*/
	inline void rfm12_tx_off(void) {
		/* turn off everything. */
		rfm12_data(RFM12_CMD_PWRMGT);
	}
#endif /* RFM12_TRANSMIT_ASK */


/************************
 * rfm12 wakeup timer mode
*/

#if RFM12_USE_WAKEUP_TIMER
	//! This function sets the wakeup timer register.
	/** \param [val]  The wakeup timer period value to be passed to the rf12. \n
	* See the rf12 datasheet for valid values.
	*/
	void rfm12_set_wakeup_timer(uint16_t val) {
		//disable the interrupt (as we're working directly with the transceiver now)
		//we won't loose interrupts, as the AVR caches them in the int flag
		RFM12_INT_OFF();

		//set wakeup timer
		rfm12_data (RFM12_CMD_WAKEUP | (val & 0x1FFF));

		//restart the wakeup timer by toggling the bit on and off
		rfm12_data(ctrl.pwrmgt_shadow & ~RFM12_PWRMGT_EW);
		rfm12_data(ctrl.pwrmgt_shadow);

		RFM12_INT_ON();
	}
#endif /* RFM12_USE_WAKEUP_TIMER */


/************************
 * rfm12 power up / power down
*/

#if RFM12_USE_POWER_CONTROL
	//! This function powers down the rfm12 modules receiver to save power.
	/**
	 * It can not receive in that state.
	 */
	void rfm12_power_down() {
		//wait for rfm12 to get to state STATE_RX_IDLE
		//before turning of the receiver.
		//reason: this way transmissions that have been triggered before
		//can be completed before we power down the rfm12.

		while (ctrl.rfm12_state != STATE_RX_IDLE);

		//disable the interrupt (as we're working directly with the transceiver now)
		//we won't loose interrupts, as the AVR caches them in the int flag
		RFM12_INT_OFF();

		//disable receiver
		ctrl.pwrmgt_shadow &= ~RFM12_PWRMGT_ER;
		rfm12_data(ctrl.pwrmgt_shadow);

		ctrl.rfm12_state = STATE_POWER_DOWN;

		RFM12_INT_ON();
	}

	//! This function powers the rfm12 modules receiver back up again
	/**
	 * Should only be called after rfm12_power_down()
	 */
	void rfm12_power_up() {
		//disable the interrupt (as we're working directly with the transceiver now)
		//we won't loose interrupts, as the AVR caches them in the int flag
		RFM12_INT_OFF();

		ctrl.rfm12_state = STATE_RX_IDLE;

		//enable receiver
		ctrl.pwrmgt_shadow |= RFM12_PWRMGT_ER;
		rfm12_data(ctrl.pwrmgt_shadow);

		RFM12_INT_ON();
	}

#endif

/************************
 * rfm12 low battery detector mode
*/

#if RFM12_LOW_BATT_DETECTOR
	//! This function sets the low battery detector and microcontroller clock divider register.
	/** \param [val]  The register value to be passed to the rf12. \n
	* See the rf12 datasheet for valid values.
	* \see rfm12_get_batt_status()
	*/
	void rfm12_set_batt_detector(uint16_t val) {
		//disable the interrupt (as we're working directly with the transceiver now)
		//we won't loose interrupts, as the AVR caches them in the int flag
		RFM12_INT_OFF();

		//set the low battery detector and microcontroller clock divider register
		rfm12_data (RFM12_CMD_LBDMCD | (val & 0x01FF));

		RFM12_INT_ON();
	}

	//! Return the current low battery detector status.
	/** \returns One of these \ref batt_states "battery states" .
	* \see rfm12_set_batt_detector() and the \ref batt_states "battery state" defines
	*/
	uint8_t rfm12_get_batt_status(void) {
		return ctrl.low_batt;
	}
#endif /* RFM12_LOW_BATT_DETECTOR */

// end rfm12_extra.c

/************************
 * Begin of library
*/


//! Interrupt handler to handle all transmit and receive data transfers to the rfm12.
/** The receiver will generate an interrupt request (IT) for the
* microcontroller - by pulling the nIRQ pin low - on the following events:
* - The TX register is ready to receive the next byte (RGIT)
* - The FIFO has received the preprogrammed amount of bits (FFIT)
* - Power-on reset (POR)
* - FIFO overflow (FFOV) / TX register underrun (RGUR)
* - Wake-up timer timeout (WKUP)
* - Negative pulse on the interrupt input pin nINT (EXT)
* - Supply voltage below the preprogrammed value is detected (LBD)
*
* The rfm12 status register is read to determine which event has occured.
* Reading the status register will clear the event flags.
*
* The interrupt handles the RGIT and FFIT events by default.
* Upon specific configuration of the library the WKUP and LBD events
* are handled additionally.
*
* \see rfm12_control_t, rf_rx_buffer_t and rf_tx_buffer_t
*/
//if polling is used, do not define an interrupt handler, but a polling function
#if (RFM12_USE_POLLING)
void rfm12_poll(void)
#else
ISR(RFM12_INT_VECT, ISR_NOBLOCK)
#endif
{
	RFM12_INT_OFF();
	uint8_t status;
	uint8_t recheck_interrupt;
	//if receive mode is not disabled (default)
	#if !(RFM12_TRANSMIT_ONLY)
		static uint8_t checksum; //static local variables produce smaller code size than globals
	#endif /* !(RFM12_TRANSMIT_ONLY) */

	do {
		//clear AVR int flag
		RFM12_INT_FLAG = (1<<RFM12_FLAG_BIT);

		//first we read the first byte of the status register
		//to get the interrupt flags
		status = rfm12_read_int_flags_inline();

		//if we use at least one of the status bits, we need to check the status again
		//for the case in which another interrupt condition occured while we were handeling
		//the first one.
		recheck_interrupt = 0;

		#if RFM12_UART_DEBUG >= 2
			serial_putc('S');
			serial_putc(status);
		#endif

		//low battery detector feature
		#if RFM12_LOW_BATT_DETECTOR
			if (status & (RFM12_STATUS_LBD >> 8)) {
				//debug
				#if RFM12_UART_DEBUG >= 2
					serial_putc('L');
				#endif

				//set status variable to low battery
				ctrl.low_batt = RFM12_BATT_LOW;
				recheck_interrupt = 1;
			}
		#endif /* RFM12_LOW_BATT_DETECTOR */

		//wakeup timer feature
		#if RFM12_USE_WAKEUP_TIMER
			if (status & (RFM12_STATUS_WKUP >> 8)) {
				//debug
				#if RFM12_UART_DEBUG >= 2
					serial_putc('W');
				#endif

				ctrl.wkup_flag = 1;
				recheck_interrupt = 1;
			}
			if (status & ((RFM12_STATUS_WKUP | RFM12_STATUS_FFIT) >> 8) ) {
				//restart the wakeup timer by toggling the bit on and off
				rfm12_data(ctrl.pwrmgt_shadow & ~RFM12_PWRMGT_EW);
				rfm12_data(ctrl.pwrmgt_shadow);
			}
		#endif /* RFM12_USE_WAKEUP_TIMER */

		//check if the fifo interrupt occurred
		if (status & (RFM12_STATUS_FFIT>>8)) {
			//yes
			recheck_interrupt = 1;
			//see what we have to do (start rx, rx or tx)
			switch (ctrl.rfm12_state) {
				case STATE_RX_IDLE: {
					//if receive mode is not disabled (default)
					#if !(RFM12_TRANSMIT_ONLY)
						uint8_t data;

						//init the bytecounter - remember, we will read the length byte, so this must be 1
						ctrl.bytecount = 1;

						//read the length byte,  and write it to the checksum
						//remember, the first byte is the length byte
						data = rfm12_read(RFM12_CMD_READ);
						checksum = data;

						//add the packet overhead and store it into a working variable
						ctrl.num_bytes = data + PACKET_OVERHEAD;

						//debug
						#if RFM12_UART_DEBUG >= 2
							serial_putc('I');
							serial_putc(data);
						#endif

						//see whether our buffer is free
						//FIXME: put this into global statekeeping struct, the free state can be set by the function which pulls the packet, i guess
						if (rf_rx_buffers[ctrl.buffer_in_num].status == STATUS_FREE) {
							//the current receive buffer is empty, so we start receiving
							ctrl.rfm12_state = STATE_RX_ACTIVE;

							//store the received length into the packet buffer
							//this length field will be used by application reading the
							//buffer.
							rf_rx_buffers[ctrl.buffer_in_num].len = data;

							//end the interrupt without resetting the fifo
							goto no_fifo_reset;
						}

						/* if we're here, the buffer is full, so we ignore this transmission by resetting the fifo (at the end of the function)  */
					#endif /* !(RFM12_TRANSMIT_ONLY) */

					} break;

				case STATE_RX_ACTIVE: {
					//if receive mode is not disabled (default)
					#if !(RFM12_TRANSMIT_ONLY)
						uint8_t data;
						//read a byte
						data = rfm12_read(RFM12_CMD_READ);

						//check if transmission is complete
						if (ctrl.bytecount < ctrl.num_bytes) {
							//debug
							#if RFM12_UART_DEBUG >= 2
								uart_putc('R');
								uart_putc(data);
							#endif

							//xor the remaining bytes onto the checksum
							//note: only the header will be effectively checked
							checksum ^= data;

							//put next byte into buffer, if there is enough space
							if (ctrl.bytecount < (RFM12_RX_BUFFER_SIZE + 3)) {
								//hackhack: begin writing to struct at offsetof len
								(& rf_rx_buffers[ctrl.buffer_in_num].len)[ctrl.bytecount] = data;
							}
#ifndef DISABLE_CHECKSUMM
							//check header against checksum
							if (ctrl.bytecount == 2 && checksum != 0xff) {
								//if the checksum does not match, reset the fifo
								break;
							}
#endif

							//increment bytecount
							ctrl.bytecount++;

							//end the interrupt without resetting the fifo
							goto no_fifo_reset;
						}

						/* if we're here, receiving is done */
						/* the fifo will be reset at the end of the function */

						//debug
						#if RFM12_UART_DEBUG >= 2
							uart_putc('D');
						#endif

						//indicate that the buffer is ready to be used
						rf_rx_buffers[ctrl.buffer_in_num].status = STATUS_COMPLETE;

						#if RFM12_USE_RX_CALLBACK
							if (rfm12_rx_callback_func != 0x0000) {
								rfm12_rx_callback_func (ctrl.rf_buffer_in->len, ctrl.rf_buffer_in.buffer);
							}
						#endif

						//switch to other buffer
						ctrl.buffer_in_num ^= 1;

						#if RFM12_USE_RX_CALLBACK
							rfm12_rx_clear(); /* clear immediately since the data has been processed by the callback func */
						#endif
					#endif /* !(RFM12_TRANSMIT_ONLY) */
					} break;

				case STATE_TX:
					//debug
					#if RFM12_UART_DEBUG >= 2
						uart_putc('T');
					#endif

					if (ctrl.bytecount < ctrl.num_bytes) {
						//load the next byte from our buffer struct.
						rfm12_data( RFM12_CMD_TX | rf_tx_buffer.sync[ctrl.bytecount++]);

						//end the interrupt without resetting the fifo
						goto no_fifo_reset;
					}

					/* if we're here, we're finished transmitting the bytes */
					/* the fifo will be reset at the end of the function */

					//Transmitter on RFM12BP off
					#ifdef TX_LEAVE_HOOK
						TX_LEAVE_HOOK;
					#endif

					//flag the buffer as free again
					ctrl.txstate = STATUS_FREE;

					//turn off the transmitter and enable receiver
					//the receiver is not enabled in transmit only mode (by PWRMGT_RECEIVE makro)
					#if RFM12_PWRMGT_SHADOW
						ctrl.pwrmgt_shadow &= ~(RFM12_PWRMGT_ET); /* disable transmitter */
						ctrl.pwrmgt_shadow |= (PWRMGT_RECEIVE);   /* activate predefined receive mode */
						rfm12_data(ctrl.pwrmgt_shadow);
					#else /* no RFM12_PWRMGT_SHADOW */
						rfm12_data( PWRMGT_RECEIVE );
					#endif /* RFM12_PWRMGT_SHADOW */

					//Receiver on RFM12BP on
					#ifdef RX_ENTER_HOOK
						RX_ENTER_HOOK;
					#endif

					//load a dummy byte to clear int status
					rfm12_data( RFM12_CMD_TX | 0xaa);
					break;
					#if RFM12_USE_POWER_CONTROL
						case STATE_POWER_DOWN:
							//load a dummy byte to clear int status
							rfm12_data( RFM12_CMD_TX | 0xaa);
							break;
					#endif
			}//end of switch

			//set the state machine to idle
			ctrl.rfm12_state = STATE_RX_IDLE;

			//reset the receiver fifo, if receive mode is not disabled (default)
			#if !(RFM12_TRANSMIT_ONLY)
				#if RFM12_UART_DEBUG >= 2
					uart_putc('F');
				#endif
				rfm12_data( RFM12_CMD_FIFORESET | CLEAR_FIFO_INLINE);
				rfm12_data( RFM12_CMD_FIFORESET | ACCEPT_DATA_INLINE);
			#endif /* !(RFM12_TRANSMIT_ONLY) */

			uint8_t b;
			no_fifo_reset:
			b = b;
		}
	} while (recheck_interrupt);

	#if RFM12_UART_DEBUG >= 2
		uart_putc('E');
	#endif

	//turn the int back on
	RFM12_INT_ON();
}


//! The tick function implements collision avoidance and initiates transmissions.
/** This function has to be called periodically.
* It will read the rfm12 status register to check if a carrier is being received,
* which would indicate activity on the chosen radio channel. \n
* If there has been no activity for long enough, the channel is believed to be free.
*
* When there is a packet waiting for transmission and the collision avoidance
* algorithm indicates that the air is free, then the interrupt control variables are
* setup for packet transmission and the rfm12 is switched to transmit mode.
* This function also fills the rfm12 tx fifo with a preamble.
*
* \warning Warning, if you do not call this function periodically, then no packet will get transmitted.
* \see rfm12_tx() and rfm12_start_tx()
*/
void rfm12_tick(void) {
	//collision detection is enabled by default
	#if !(RFM12_NOCOLLISIONDETECTION)
		uint16_t status;

		//start with a channel free count of 16, this is necessary for the ASK receive feature to work
		static uint8_t channel_free_count = 16; //static local variables produce smaller code size than globals
	#endif

	//debug
	#if RFM12_UART_DEBUG
		static uint8_t oldstate;
		uint8_t state = ctrl.rfm12_state;
		if (oldstate != state) {
			uart_putstr ("mode change: ");
			switch (state) {
				case STATE_RX_IDLE:
					uart_putc ('i');
					break;
				case STATE_RX_ACTIVE:
					uart_putc ('r');
					break;
				case STATE_TX:
					uart_putc ('t');
					break;
				default:
					uart_putc ('?');
			}
			uart_putstr ("\r\n");
			oldstate = state;
		}
	#endif

	//don't disturb RFM12 if transmitting or receiving
	if (ctrl.rfm12_state != STATE_RX_IDLE) {
		return;
	}

	//collision detection is enabled by default
	#if !(RFM12_NOCOLLISIONDETECTION)
		//disable the interrupt (as we're working directly with the transceiver now)
		//hint: we could be losing an interrupt here, because we read the status register.
		//this applys for the Wakeup timer, as it's flag is reset by reading.
		RFM12_INT_OFF();
		status = rfm12_read(RFM12_CMD_STATUS);
		RFM12_INT_ON();

		//wakeup timer workaround (if we don't restart the timer after timeout, it will stay off.)
		#if RFM12_USE_WAKEUP_TIMER
			if (status & (RFM12_STATUS_WKUP)) {
				ctrl.wkup_flag = 1;

				RFM12_INT_OFF();
				//restart the wakeup timer by toggling the bit on and off
				rfm12_data(ctrl.pwrmgt_shadow & ~RFM12_PWRMGT_EW);
				rfm12_data(ctrl.pwrmgt_shadow);
				RFM12_INT_ON();
			}
		#endif /* RFM12_USE_WAKEUP_TIMER */

		//check if we see a carrier
		if (status & RFM12_STATUS_RSSI) {
			//yes: reset free counter and return
			channel_free_count = CHANNEL_FREE_TIME;
			return;
		}
		//no

		//is the channel free long enough ?
		if (channel_free_count != 0) {
			//no:
			channel_free_count--; // decrement counter
			return;
		}
		//yes: we can begin transmitting
	#endif

	//do we have something to transmit?
	if (ctrl.txstate == STATUS_OCCUPIED) { //yes: start transmitting
		//disable the interrupt (as we're working directly with the transceiver now)
		//we won't loose interrupts, as the AVR caches them in the int flag.
		//we could disturb an ongoing reception,
		//if it has just started some cpu cycles ago
		//(as the check for this case is some lines (cpu cycles) above)
		//anyhow, we MUST transmit at some point...
		RFM12_INT_OFF();

		//disable receiver - if you don't do this, tx packets will get lost
		//as the fifo seems to be in use by the receiver

		#if RFM12_PWRMGT_SHADOW
			ctrl.pwrmgt_shadow &= ~(RFM12_PWRMGT_ER); /* disable receiver */
			rfm12_data(ctrl.pwrmgt_shadow);
		#else
			rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT ); /* disable receiver */
		#endif

		//RFM12BP receiver off
		#ifdef RX_LEAVE_HOOK
			RX_LEAVE_HOOK;
		#endif

		//calculate number of bytes to be sent by ISR
		//2 sync bytes + len byte + type byte + checksum + message length + 1 dummy byte
		ctrl.num_bytes = rf_tx_buffer.len + 6;

		//reset byte sent counter
		ctrl.bytecount = 0;

		//set mode for interrupt handler
		ctrl.rfm12_state = STATE_TX;

		//RFM12BP transmitter on
		#ifdef TX_ENTER_HOOK
			TX_ENTER_HOOK;
		#endif

		//fill 2byte 0xAA preamble into data register
		//the preamble helps the receivers AFC circuit to lock onto the exact frequency
		//(hint: the tx FIFO [if el is enabled] is two staged, so we can safely write 2 bytes before starting)
		rfm12_data(RFM12_CMD_TX | PREAMBLE);
		rfm12_data(RFM12_CMD_TX | PREAMBLE);

		//set ET in power register to enable transmission (hint: TX starts now)
		#if RFM12_PWRMGT_SHADOW
			ctrl.pwrmgt_shadow |= RFM12_PWRMGT_ET;
			rfm12_data (ctrl.pwrmgt_shadow);
		#else
			rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT | RFM12_PWRMGT_ET);
		#endif

		//enable the interrupt to continue the transmission
		RFM12_INT_ON();
	}
}


//! Enqueue an already buffered packet for transmission
/** If there is no active transmission, the packet header is written to the
* transmission control buffer and the packet will be enqueued for transmission. \n
* This function is not responsible for buffering the actual packet data.
* The data has to be copied into the transmit buffer beforehand,
* which can be accomplished by the rfm12_tx() function.
*
* \note Note that this function does not start the transmission, it merely enqueues the packet. \n
* Transmissions are started by rfm12_tick().
* \param [type] The packet header type field
* \param [length] The packet data length
* \returns One of these defines: \ref tx_retvals "TX return values"
* \see rfm12_tx() and rfm12_tick()
*/
#if (RFM12_NORETURNS)
void
#else
uint8_t
#endif
rfm12_start_tx(uint8_t type, uint8_t length) {
	//exit if the buffer isn't free
	if (ctrl.txstate != STATUS_FREE)
		return TXRETURN(RFM12_TX_OCCUPIED);

	//write airlab header to buffer
	rf_tx_buffer.len = length;
	rf_tx_buffer.type = type;
	rf_tx_buffer.checksum = length ^ type ^ 0xff;

	//schedule packet for transmission
	ctrl.txstate = STATUS_OCCUPIED;

	return TXRETURN(RFM12_TX_ENQUEUED);
}


//! Copy a packet to the buffer and call rfm12_start_tx() to enqueue it for transmission.
/** If there is no active transmission, the buffer contents will be copied to the
* internal transmission buffer. Finally the buffered packet is going to be enqueued by
* calling rfm12_start_tx(). If automatic buffering of packet data is not necessary,
* which is the case when the packet data does not change while the packet is enqueued
* for transmission, then one could directly store the data in \ref rf_tx_buffer
* (see rf_tx_buffer_t) and use the rfm12_start_tx() function.
*
* \note Note that this function does not start the transmission, it merely enqueues the packet. \n
* Transmissions are started by rfm12_tick().
* \param [len] The packet data length
* \param [type] The packet header type field
* \param [data] Pointer to the packet data
* \returns One of these defines: \ref tx_retvals "TX return values"
* \see rfm12_start_tx() and rfm12_tick()
*/
#if !(RFM12_SMALLAPI)
	#if (RFM12_NORETURNS)
	void
	#else
	uint8_t
	#endif
	rfm12_tx(uint8_t len, uint8_t type, uint8_t *data) {
		#if RFM12_UART_DEBUG
			uart_putstr ("sending packet\r\n");
		#endif

		if (len > RFM12_TX_BUFFER_SIZE) return TXRETURN(RFM12_TX_ERROR);

		//exit if the buffer isn't free
		if (ctrl.txstate != STATUS_FREE)
			return TXRETURN(RFM12_TX_OCCUPIED);

		memcpy(rf_tx_buffer.buffer, data, len);

		#if (!(RFM12_NORETURNS))
		return rfm12_start_tx(type, len);
		#else
		rfm12_start_tx(type, len);
		#endif
	}
#endif /* RFM12_SMALLAPI */


//if receive mode is not disabled (default)
#if !(RFM12_TRANSMIT_ONLY)
	//! Function to clear buffer complete/occupied status.
	/** This function will set the current receive buffer status to free and switch
	* to the other buffer, which can then be read using rfm12_rx_buffer().
	*
	* \see rfm12_rx_status(), rfm12_rx_len(), rfm12_rx_type(), rfm12_rx_buffer() and rf_rx_buffers
	*/
	//warning: without the attribute, gcc will inline this even if -Os is set
	void __attribute__((noinline)) rfm12_rx_clear(void) {
			//mark the current buffer as empty
			rf_rx_buffers[ctrl.buffer_out_num].status = STATUS_FREE;

			//switch to the other buffer
			ctrl.buffer_out_num ^= 1;

	}
#endif /* !(RFM12_TRANSMIT_ONLY) */


//enable internal data register and fifo
//setup selected band
#define RFM12_CMD_CFG_DEFAULT   (RFM12_CMD_CFG | RFM12_CFG_EL | RFM12_CFG_EF | RFM12_BASEBAND | RFM12_XTAL_LOAD)

//set rx parameters: int-in/vdi-out pin is vdi-out,
//Bandwith, LNA, RSSI
#define RFM12_CMD_RXCTRL_DEFAULT (RFM12_CMD_RXCTRL | RFM12_RXCTRL_P16_VDI | RFM12_RXCTRL_VDI_FAST | RFM12_FILTER_BW | RFM12_LNA_GAIN | RFM12_RSSI_THRESHOLD )

//set AFC to automatic, (+4 or -3)*2.5kHz Limit, fine mode, active and enabled
#define RFM12_CMD_AFC_DEFAULT  (RFM12_CMD_AFC | RFM12_AFC_AUTO_KEEP | RFM12_AFC_LIMIT_4 | RFM12_AFC_FI | RFM12_AFC_OE | RFM12_AFC_EN)

//set TX Power, frequency shift
#define RFM12_CMD_TXCONF_DEFAULT  (RFM12_CMD_TXCONF | RFM12_POWER | RFM12_TXCONF_FS_CALC(FSK_SHIFT) )

static const uint16_t init_cmds[] PROGMEM = {
	//defined above (so shadow register is inited with same value)
	RFM12_CMD_CFG_DEFAULT,

	//set power default state (usually disable clock output)
	//do not write the power register two times in a short time
	//as it seems to need some recovery
	(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT),

	//set frequency
	(RFM12_CMD_FREQUENCY | RFM12_FREQUENCY_CALC(RFM12_FREQUENCY) ),

	//set data rate
	(RFM12_CMD_DATARATE | DATARATE_VALUE ),

	//defined above (so shadow register is inited with same value)
	RFM12_CMD_RXCTRL_DEFAULT,

	//automatic clock lock control(AL), digital Filter(!S),
	//Data quality detector value 3, slow clock recovery lock
	(RFM12_CMD_DATAFILTER | RFM12_DATAFILTER_AL | 3),

	//2 Byte Sync Pattern, Start fifo fill when sychron pattern received,
	//disable sensitive reset, Fifo filled interrupt at 8 bits
	(RFM12_CMD_FIFORESET | RFM12_FIFORESET_DR | (8 << 4)),

	//defined above (so shadow register is inited with same value)
	RFM12_CMD_AFC_DEFAULT,

	//defined above (so shadow register is inited with same value)
	RFM12_CMD_TXCONF_DEFAULT,

	//disable low dutycycle mode
	(RFM12_CMD_DUTYCYCLE),

	//disable wakeup timer
	(RFM12_CMD_WAKEUP),

	//enable rf receiver chain, if receiving is not disabled (default)
	//the magic is done via defines
	(RFM12_CMD_PWRMGT | PWRMGT_RECEIVE),
};

//! This is the main library initialization function
/**This function takes care of all module initialization, including:
* - Setup of the used frequency band and external capacitor
* - Setting the exact frequency (channel)
* - Setting the transmission data rate
* - Configuring various module related rx parameters, including the amplification
* - Enabling the digital data filter
* - Enabling the use of the modules fifo, as well as enabling sync pattern detection
* - Configuring the automatic frequency correction
* - Setting the transmit power
*
* This initialization function also sets up various library internal configuration structs and
* puts the module into receive mode before returning.
*
* \note Please note that the transmit power and receive amplification values are currently hard coded.
* Have a look into rfm12_hw.h for possible settings.
*/
void rfm12_init(void) {
	//initialize spi
	SS_RELEASE();
	RF_DDR_SS |= (1<<RF_BIT_SS);
	spi_init();

	//typically sets DDR registers for RFM12BP TX/RX pin
	#ifdef TX_INIT_HOOK
		TX_INIT_HOOK;
	#endif

	#ifdef RX_INIT_HOOK
		RX_INIT_HOOK;
	#endif

	//store the syncronization pattern to the transmission buffer
	//the sync pattern is used by the receiver to distinguish noise from real transmissions
	//the sync pattern is hardcoded into the receiver
	rf_tx_buffer.sync[0] = SYNC_MSB;
	rf_tx_buffer.sync[1] = SYNC_LSB;

	//if receive mode is not disabled (default)
	#if !(RFM12_TRANSMIT_ONLY)
		//init buffer pointers
		ctrl.buffer_in_num = 0;
		ctrl.buffer_out_num = 0;
	#endif /* !(RFM12_TRANSMIT_ONLY) */

	//low battery detector feature initialization
	#if RFM12_LOW_BATT_DETECTOR
		ctrl.low_batt = RFM12_BATT_OKAY;
	#endif /* RFM12_LOW_BATT_DETECTOR */

	#if RFM12_PWRMGT_SHADOW
		//set power management shadow register to receiver chain enabled or disabled
		//the define correctly handles the transmit only mode
		ctrl.pwrmgt_shadow = (RFM12_CMD_PWRMGT | PWRMGT_RECEIVE);
	#endif


	#if RFM12_LIVECTRL
		//init shadow registers with values about to be written to rfm12
		ctrl.rxctrl_shadow = RFM12_CMD_RXCTRL_DEFAULT;
		ctrl.afc_shadow = RFM12_CMD_AFC_DEFAULT;
		ctrl.txconf_shadow = RFM12_CMD_TXCONF_DEFAULT;
		ctrl.cfg_shadow =    RFM12_CMD_CFG_DEFAULT;
	#endif

	//write all the initialisation values to rfm12
	uint8_t x;
	for (x = 0; x < ( sizeof(init_cmds) / 2) ; x++) {
		rfm12_data(pgm_read_word(&init_cmds[x]));
	}

	#ifdef RX_ENTER_HOOK
		RX_ENTER_HOOK;
	#endif

	#if RFM12_USE_CLOCK_OUTPUT || RFM12_LOW_BATT_DETECTOR
		rfm12_data(RFM12_CMD_LBDMCD | RFM12_LBD_VOLTAGE | RFM12_CLOCK_OUT_FREQUENCY ); //set low battery detect, clock output
	#endif

	//ASK receive mode feature initialization
	#if RFM12_RECEIVE_ASK
		adc_init();
	#endif

	//setup interrupt for falling edge trigger
	RFM12_INT_SETUP();

	//clear int flag
	rfm12_read(RFM12_CMD_STATUS);
	RFM12_INT_FLAG = (1<<RFM12_FLAG_BIT);

	//init receiver fifo, we now begin receiving.
	rfm12_data(CLEAR_FIFO);
	rfm12_data(ACCEPT_DATA);

	//activate the interrupt
	RFM12_INT_ON();
}

#endif // ENABLE_GPL_RFM12B
//--
#ifdef ENABLE_MIRF24
void mirf_init()
// Initializes pins to communicate with the MiRF module
// Should be called in the early initializing phase at startup.
{
    sbi(mirf_CE_PORT, mirf_CE_PIN);
    sbi(mirf_CSN_PORT, mirf_CSN_PIN);

    mirf_ceLow;
    mirf_csnHi;

    // Initialize spi module
    SPI_master_init();
}

void mirf_configRegister(uint8_t reg, uint8_t value)
// Clocks only one byte into the given MiRF register
{
    mirf_csnLow;
    SPI_master_transmit(RF24_W_REGISTER | (RF24_REGISTER_MASK & reg));
    SPI_master_transmit(value);
    mirf_csnHi;
}

void mirf_powerUpRx(){
	mirf_PTX = 0;
	mirf_ceLow;
	mirf_configRegister(RF24_CONFIG, mirf_CONFIG | ( (1<<RF24_PWR_UP) | (1<<RF24_PRIM_RX) ) );
	mirf_ceHi;
	mirf_configRegister(RF24_STATUS,(1 << RF24_TX_DS) | (1 << RF24_MAX_RT));
}

void mirf_flushRx(){
	mirf_csnLow;
    SPI_master_transmit( RF24_FLUSH_RX );
    mirf_csnHi;
}

void mirf_powerUpTx(){
	mirf_PTX = 1;
	mirf_configRegister(RF24_CONFIG, mirf_CONFIG | ( (1<<RF24_PWR_UP) | (0<<RF24_PRIM_RX) ) );
}

void mirf_config()
// Sets the important registers in the MiRF module and powers the module
// in receiving mode
// NB: channel and payload must be set now.
{
    // Set RF channel
	mirf_configRegister(RF24_RF_CH,mirf_CHANNEL);

    // Set length of incoming payload
	mirf_configRegister(RF24_RX_PW_P0, mirf_PAYLOAD);
	mirf_configRegister(RF24_RX_PW_P1, mirf_PAYLOAD);

    // Start receiver
	mirf_powerUpRx();
	mirf_flushRx();
}

void mirf_transferSync(uint8_t *dataout,uint8_t *datain,uint8_t len){
	uint8_t i;
	for(i = 0;i < len;i++){
		datain[i] = SPI_master_transmit(dataout[i]);
	}
}

void mirf_transmitSync(uint8_t *dataout,uint8_t len){
	uint8_t i;
	for(i = 0;i < len;i++){
		SPI_master_transmit(dataout[i]);
	}
}


void mirf_readRegister(uint8_t reg, uint8_t * value, uint8_t len)
// Reads an array of bytes from the given start position in the MiRF registers.
{
	mirf_csnLow;
    SPI_master_transmit(RF24_R_REGISTER | (RF24_REGISTER_MASK & reg));
    mirf_transferSync(value,value,len);
    mirf_csnHi;
}

void mirf_writeRegister(uint8_t reg, uint8_t * value, uint8_t len)
// Writes an array of bytes into inte the MiRF registers.
{
	mirf_csnLow;
    SPI_master_transmit(RF24_W_REGISTER | (RF24_REGISTER_MASK & reg));
    mirf_transmitSync(value,len);
    mirf_csnHi;
}

void mirf_setRADDR(uint8_t * adr)
// Sets the receiving address
{
	mirf_ceLow;
	mirf_writeRegister(RF24_RX_ADDR_P1,adr,mirf_ADDR_LEN);
	mirf_ceHi;
}

void mirf_setTADDR(uint8_t * adr)
// Sets the transmitting address
{
	/*
	 * RX_ADDR_P0 must be set to the sending addr for auto ack to work.
	 */

	mirf_writeRegister(RF24_RX_ADDR_P0,adr,mirf_ADDR_LEN);
	mirf_writeRegister(RF24_TX_ADDR,adr,mirf_ADDR_LEN);
}

extern uint8_t mirf_rxFifoEmpty(){
	uint8_t fifoStatus;

	mirf_readRegister(RF24_FIFO_STATUS,&fifoStatus,sizeof(fifoStatus));
	return (fifoStatus & (1 << RF24_RX_EMPTY));
}

uint8_t mirf_getStatus(){
	uint8_t rv;
	mirf_readRegister(RF24_STATUS,&rv,1);
	return rv;
}

extern uint8_t mirf_dataReady()
// Checks if data is available for reading
{
    // See note in getData() function - just checking RX_DR isn't good enough
	uint8_t status = mirf_getStatus();

    // We can short circuit on RX_DR, but if it's not set, we still need
    // to check the FIFO for any pending packets
    if ( status & (1 << RF24_RX_DR) ) return 1;
    return !mirf_rxFifoEmpty();
}


extern void mirf_getData(uint8_t * data)
// Reads payload bytes into data array
{
	mirf_csnLow;                               // Pull down chip select
    SPI_master_transmit( RF24_R_RX_PAYLOAD );            // Send cmd to read rx payload
    mirf_transferSync(data,data,mirf_PAYLOAD); // Read payload
    mirf_csnHi;                               // Pull up chip select
    // NVI: per product spec, p 67, note c:
    //  "The RX_DR IRQ is asserted by a new packet arrival event. The procedure
    //  for handling this interrupt should be: 1) read payload through SPI,
    //  2) clear RX_DR IRQ, 3) read FIFO_STATUS to check if there are more
    //  payloads available in RX FIFO, 4) if there are more data in RX FIFO,
    //  repeat from step 1)."
    // So if we're going to clear RX_DR here, we need to check the RX FIFO
    // in the dataReady() function
    mirf_configRegister(RF24_STATUS,(1<<RF24_RX_DR));   // Reset status register
}

void mirf_send(uint8_t * value)
// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
{
    uint8_t status;
    status = mirf_getStatus();

    while (mirf_PTX) {
	    status = mirf_getStatus();

	    if((status & ((1 << RF24_TX_DS)  | (1 << RF24_MAX_RT)))){
	    	mirf_PTX = 0;
		    break;
	    }
    }                  // Wait until last paket is send

    mirf_ceLow;

    mirf_powerUpTx();       // Set to transmitter mode , Power up

    mirf_csnLow;                    // Pull down chip select
    SPI_master_transmit( RF24_FLUSH_TX );     // Write cmd to flush tx fifo
    mirf_csnHi;                    // Pull up chip select

    mirf_csnLow;                    // Pull down chip select
    SPI_master_transmit( RF24_W_TX_PAYLOAD ); // Write cmd to write payload
    mirf_transmitSync(value,mirf_PAYLOAD);   // Write payload
    mirf_csnHi;                    // Pull up chip select

    mirf_ceHi;                     // Start transmission
}

void mirf_powerDown(){
	mirf_ceLow;
	mirf_configRegister(RF24_CONFIG, mirf_CONFIG );
}

#endif // ENABLE_MIRF24
//--
#ifdef ENABLE_ONE_WIRE
#ifdef OW_ONE_BUS

#define OW_GET_IN()   ( OW_IN & (1<<OW_PIN))
#define OW_OUT_LOW()  ( OW_OUT &= (~(1 << OW_PIN)) )
#define OW_OUT_HIGH() ( OW_OUT |= (1 << OW_PIN) )
#define OW_DIR_IN()   ( OW_DDR &= (~(1 << OW_PIN )) )
#define OW_DIR_OUT()  ( OW_DDR |= (1 << OW_PIN) )

#else

/* set bus-config with ow_set_bus() */
uint8_t OW_PIN_MASK;
volatile uint8_t* OW_IN;
volatile uint8_t* OW_OUT;
volatile uint8_t* OW_DDR;

#define OW_GET_IN()   ( *OW_IN & OW_PIN_MASK )
#define OW_OUT_LOW()  ( *OW_OUT &= (uint8_t) ~OW_PIN_MASK )
#define OW_OUT_HIGH() ( *OW_OUT |= (uint8_t)  OW_PIN_MASK )
#define OW_DIR_IN()   ( *OW_DDR &= (uint8_t) ~OW_PIN_MASK )
#define OW_DIR_OUT()  ( *OW_DDR |= (uint8_t)  OW_PIN_MASK )

void ow_set_bus(volatile uint8_t* in,
		volatile uint8_t* out,
		volatile uint8_t* ddr,
		uint8_t pin)
{
	OW_DDR=ddr;
	OW_OUT=out;
	OW_IN=in;
	OW_PIN_MASK = (1 << pin);
	ow_reset();
}

#endif

uint8_t ow_input_pin_state() {
	return OW_GET_IN();
}

void ow_parasite_enable(void) {
	OW_OUT_HIGH();
	OW_DIR_OUT();
}

void ow_parasite_disable(void) {
	OW_DIR_IN();
#if (!OW_USE_INTERNAL_PULLUP)
	OW_OUT_LOW();
#endif
}

uint8_t ow_reset(void) {
	uint8_t err;

	OW_OUT_LOW();
	OW_DIR_OUT(); // pull OW-Pin low for 480us
	_delay_us(480);

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		// set Pin as input - wait for clients to pull low
		OW_DIR_IN();// input
#if OW_USE_INTERNAL_PULLUP
		OW_OUT_HIGH();
#endif

		_delay_us(64); // was 66
		err = OW_GET_IN();// no presence detect
						  // if err!=0: nobody pulled to low, still high
	}

	// after a delay the clients should release the line
	// and input-pin gets back to high by pull-up-resistor
	_delay_us(480 - 64);// was 480-66
	if (OW_GET_IN() == 0) {
		err = 1; // short circuit, expected low but got high
	}

	return err;
}

/* Timing issue when using runtime-bus-selection (!OW_ONE_BUS):
 The master should sample at the end of the 15-slot after initiating
 the read-time-slot. The variable bus-settings need more
 cycles than the constant ones so the delays had to be shortened
 to achieve a 15uS overall delay
 Setting/clearing a bit in I/O Register needs 1 cyle in OW_ONE_BUS
 but around 14 cyles in configurable bus (us-Delay is 4 cyles per uS) */
static uint8_t ow_bit_io_intern(uint8_t b, uint8_t with_parasite_enable) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
#if OW_USE_INTERNAL_PULLUP
		OW_OUT_LOW();
#endif
		OW_DIR_OUT(); // drive bus low
		_delay_us(2);// T_INT > 1usec accoding to timing-diagramm
		if (b) {
			OW_DIR_IN(); // to write "1" release bus, resistor pulls high
#if OW_USE_INTERNAL_PULLUP
			OW_OUT_HIGH();
#endif
		}

		// "Output data from the DS18B20 is valid for 15usec after the falling
		// edge that initiated the read time slot. Therefore, the master must
		// release the bus and then sample the bus state within 15ussec from
		// the start of the slot."
		_delay_us(15 - 2 - OW_CONF_DELAYOFFSET);

		if (OW_GET_IN() == 0) {
			b = 0; // sample at end of read-timeslot
		}

		_delay_us(60 - 15 - 2 + OW_CONF_DELAYOFFSET);
#if OW_USE_INTERNAL_PULLUP
		OW_OUT_HIGH();
#endif
		OW_DIR_IN();

		if (with_parasite_enable) {
			ow_parasite_enable();
		}

	} /* ATOMIC_BLOCK */

	_delay_us(OW_RECOVERY_TIME); // may be increased for longer wires

	return b;
}

uint8_t ow_bit_io(uint8_t b) {
	return ow_bit_io_intern(b & 1, 0);
}

uint8_t ow_byte_wr(uint8_t b) {
	uint8_t i = 8, j;

	do {
		j = ow_bit_io(b & 1);
		b >>= 1;
		if (j) {
			b |= 0x80;
		}
	}while (--i);

	return b;
}

uint8_t ow_byte_wr_with_parasite_enable(uint8_t b) {
	uint8_t i = 8, j;

	do {
		if (i != 1) {
			j = ow_bit_io_intern(b & 1, 0);
		} else {
			j = ow_bit_io_intern(b & 1, 1);
		}
		b >>= 1;
		if (j) {
			b |= 0x80;
		}
	}while (--i);

	return b;
}

uint8_t ow_byte_rd(void) {
	// read by sending only "1"s, so bus gets released
	// after the init low-pulse in every slot
	return ow_byte_wr(0xFF);
}

uint8_t ow_rom_search(uint8_t diff, uint8_t *id) {
	uint8_t i, j, next_diff;
	uint8_t b;

	if (ow_reset()) {
		return OW_PRESENCE_ERR; // error, no device found <--- early exit!
	}

	ow_byte_wr(OW_SEARCH_ROM); // ROM search command
	next_diff = OW_LAST_DEVICE;// unchanged on last device

	i = OW_ROMCODE_SIZE * 8;// 8 bytes

	do {
		j = 8; // 8 bits
		do {
			b = ow_bit_io(1); // read bit
			if (ow_bit_io(1)) { // read complement bit
				if (b) { // 0b11
					return OW_DATA_ERR;// data error <--- early exit!
				}
			} else {
				if (!b) { // 0b00 = 2 devices
					if (diff > i || ((*id & 1) && diff != i)) {
						b = 1; // now 1
						next_diff = i;// next pass 0
					}
				}
			}
			ow_bit_io(b); // write bit
			*id >>= 1;
			if (b) {
				*id |= 0x80; // store bit
			}

			i--;

		}while (--j);

		id++; // next byte

	}while (i);

	return next_diff; // to continue search
}

static void ow_command_intern(uint8_t command, uint8_t *id,
		uint8_t with_parasite_enable) {
	uint8_t i;

	ow_reset();

	if (id) {
		ow_byte_wr(OW_MATCH_ROM); // to a single device
		i = OW_ROMCODE_SIZE;
		do {
			ow_byte_wr(*id);
			id++;
		}while (--i);
	} else {
		ow_byte_wr(OW_SKIP_ROM); // to all devices
	}

	if (with_parasite_enable) {
		ow_byte_wr_with_parasite_enable(command);
	} else {
		ow_byte_wr(command);
	}
}

void ow_command(uint8_t command, uint8_t *id) {
	ow_command_intern(command, id, 0);
}

void ow_command_with_parasite_enable(uint8_t command, uint8_t *id) {
	ow_command_intern(command, id, 1);
}

#ifdef ENABLE_DS18_2_

#define CRC8INIT    0x00
#define CRC8POLY    0x18              //0X18 = X^8+X^5+X^4+X^0
uint8_t crc8(uint8_t *data, uint16_t number_of_bytes_in_data) {
	uint8_t crc;
	uint16_t loop_count;
	uint8_t bit_counter;
	uint8_t b;
	uint8_t feedback_bit;

	crc = CRC8INIT;

	for (loop_count = 0; loop_count != number_of_bytes_in_data; loop_count++) {
		b = data[loop_count];

		bit_counter = 8;
		do {
			feedback_bit = (crc ^ b) & 0x01;

			if (feedback_bit == 0x01) {
				crc = crc ^ CRC8POLY;
			}
			crc = (crc >> 1) & 0x7F;
			if (feedback_bit == 0x01) {
				crc = crc | 0x80;
			}

			b = b >> 1;
			bit_counter--;

		}while (bit_counter > 0);
	}

	return crc;
}
//
/*----------- start of "debug-functions" ---------------*/

#if DS18X20_VERBOSE
#if (!DS18X20_DECICELSIUS)
#error "DS18X20_DECICELSIUS must be enabled for verbose-mode"
#endif

/* functions for debugging-output - undef DS18X20_VERBOSE in .h
 if you run out of program-memory */

static int16_t DS18X20_raw_to_decicelsius(uint8_t fc, uint8_t sp[]);

void DS18X20_show_id_uart(uint8_t *id, size_t n) {
	size_t i;

	for (i = 0; i < n; i++) {
		if (i == 0) {
			serial_puts_f("FC:");
		} else if (i == n - 1) {
			serial_puts_f("CRC:");
		}
		if (i == 1) {
			serial_puts_f("SN: ");
		}
		serial_puthexU08(id[i]);
		serial_puts_f(" ");
		if (i == 0) {
			if (id[0] == DS18S20_FAMILY_CODE) {
				serial_puts_f("(18S)");
			} else if (id[0] == DS18B20_FAMILY_CODE) {
				serial_puts_f("(18B)");
			} else if (id[0] == DS1822_FAMILY_CODE) {
				serial_puts_f("(22)");
			} else {
				serial_puts_f("( ? )");
			}
		}
	}
	if (crc8(id, OW_ROMCODE_SIZE))
	serial_puts_f(" CRC FAIL ");
	else
	serial_puts_f(" CRC O.K. ");
}

static void show_sp_uart(uint8_t *sp, size_t n) {
	size_t i;

	serial_puts_f("SP:");
	for (i = 0; i < n; i++) {
		if (i == n - 1) {
			serial_puts_f("CRC:");
		}
		serial_puthexU08(sp[i]);
		serial_puts_f(" ");
	}
}

/*
 convert raw value from DS18x20 to Celsius
 input is:
 - familycode fc (0x10/0x28 see header)
 - scratchpad-buffer
 output is:
 - cel full celsius
 - fractions of celsius in millicelsius*(10^-1)/625 (the 4 LS-Bits)
 - subzero =0 positive / 1 negative
 always returns  DS18X20_OK
 */
static uint8_t DS18X20_meas_to_cel(uint8_t fc, uint8_t *sp, uint8_t* subzero,
		uint8_t* cel, uint8_t* cel_frac_bits) {
	uint16_t meas;
	uint8_t i;

	meas = sp[0]; // LSB
	meas |= ((uint16_t) sp[1]) << 8;// MSB

	//  only work on 12bit-base
	if (fc == DS18S20_FAMILY_CODE) { // 9 -> 12 bit if 18S20
		/* Extended res. measurements for DS18S20 contributed by Carsten Foss */
		meas &= (uint16_t) 0xfffe; // Discard LSB, needed for later extended precicion calc
		meas <<= 3;// Convert to 12-bit, now degrees are in 1/16 degrees units
		meas += (16 - sp[6]) - 4;// Add the compensation and remember to subtract 0.25 degree (4/16)
	}

	// check for negative
	if (meas & 0x8000) {
		*subzero = 1; // mark negative
		meas ^= 0xffff;// convert to positive => (twos complement)++
		meas++;
	} else {
		*subzero = 0;
	}

	// clear undefined bits for B != 12bit
	if (fc == DS18B20_FAMILY_CODE || fc == DS1822_FAMILY_CODE) {
		i = sp[DS18B20_CONF_REG];
		if ((i & DS18B20_12_BIT) == DS18B20_12_BIT) {
			;
		} else if ((i & DS18B20_11_BIT) == DS18B20_11_BIT) {
			meas &= ~(DS18B20_11_BIT_UNDF);
		} else if ((i & DS18B20_10_BIT) == DS18B20_10_BIT) {
			meas &= ~(DS18B20_10_BIT_UNDF);
		} else { // if ( (i & DS18B20_9_BIT) == DS18B20_9_BIT ) {
			meas &= ~(DS18B20_9_BIT_UNDF);
		}
	}

	*cel = (uint8_t) (meas >> 4);
	*cel_frac_bits = (uint8_t) (meas & 0x000F);

	return DS18X20_OK;
}

static void DS18X20_uart_put_temp(const uint8_t subzero, const uint8_t cel,
		const uint8_t cel_frac_bits) {
	char buffer[sizeof(int) * 8 + 1];
	size_t i;

	serial_putchar((subzero) ? '-' : '+');
	serial_putint((int) cel);
	serial_puts_f(".");
	itoa(cel_frac_bits * DS18X20_FRACCONV, buffer, 10);
	for (i = 0; i < 4 - strlen(buffer); i++) {
		serial_puts_f("0");
	}
	serial_puts(buffer);
	serial_puts_f("_C");
}

/* verbose output rom-search follows read-scratchpad in one loop */
uint8_t DS18X20_read_meas_all_verbose(void) {
	uint8_t id[OW_ROMCODE_SIZE], sp[DS18X20_SP_SIZE], diff;
	uint8_t i;
	uint16_t meas;
	int16_t decicelsius;
	int8_t s[10];
	uint8_t subzero, cel, cel_frac_bits;

	for (diff = OW_SEARCH_FIRST; diff != OW_LAST_DEVICE;) {
		diff = ow_rom_search(diff, &id[0]);

		if (diff == OW_PRESENCE_ERR) {
			serial_puts_f("No Sensor found\r\n");
			return OW_PRESENCE_ERR; // <--- early exit!
		}

		if (diff == OW_DATA_ERR) {
			serial_puts_f("Bus Error\r\n");
			return OW_DATA_ERR; // <--- early exit!
		}

		DS18X20_show_id_uart(id, OW_ROMCODE_SIZE);

		if (id[0] == DS18B20_FAMILY_CODE || id[0] == DS18S20_FAMILY_CODE
				|| id[0] == DS1822_FAMILY_CODE) {
			// temperature sensor

			serial_puts_f("\r\n");

			ow_byte_wr(DS18X20_READ);// read command

			for (i = 0; i < DS18X20_SP_SIZE; i++) {
				sp[i] = ow_byte_rd();
			}

			show_sp_uart(sp, DS18X20_SP_SIZE);

			if (crc8(&sp[0], DS18X20_SP_SIZE)) {
				serial_puts_f(" CRC FAIL ");
			} else {
				serial_puts_f(" CRC O.K. ");
			}
			serial_puts_f("\r\n");

			meas = sp[0]; // LSB Temp. from Scrachpad-Data
			meas |= (uint16_t) (sp[1] << 8);// MSB

			serial_puts_f(" T_raw=");
			serial_puthexU08((uint8_t) (meas >> 8));
			serial_puthexU08((uint8_t) meas);
			serial_puts_f(" ");

			if (id[0] == DS18S20_FAMILY_CODE) { // 18S20
				serial_puts_f("S20/09");
			} else if (id[0] == DS18B20_FAMILY_CODE
					|| id[0] == DS1822_FAMILY_CODE) { // 18B20 or 1822
				i = sp[DS18B20_CONF_REG];
				if ((i & DS18B20_12_BIT) == DS18B20_12_BIT) {
					serial_puts_f("B20/12");
				} else if ((i & DS18B20_11_BIT) == DS18B20_11_BIT) {
					serial_puts_f("B20/11");
				} else if ((i & DS18B20_10_BIT) == DS18B20_10_BIT) {
					serial_puts_f(" B20/10 ");
				} else { // if ( (i & DS18B20_9_BIT) == DS18B20_9_BIT ) {
					serial_puts_f("B20/09");
				}
			}
			serial_puts_f(" ");

			DS18X20_meas_to_cel(id[0], sp, &subzero, &cel, &cel_frac_bits);
			DS18X20_uart_put_temp(subzero, cel, cel_frac_bits);

			decicelsius = DS18X20_raw_to_decicelsius(id[0], sp);
			if (decicelsius == DS18X20_INVALID_DECICELSIUS) {
				serial_puts_f("* INVALID *");
			} else {
				serial_puts_f(" conv: ");
				serial_putint(decicelsius);
				serial_puts_f(" deci_C ");
				DS18X20_format_from_decicelsius(decicelsius, s, 10);
				serial_puts_f(" fmt: ");
				serial_puts(s);
				serial_puts_f("_C ");
			}

			serial_puts_f("\r\n");

		} // if meas-sensor

	} // loop all sensors

	serial_puts_f("\r\n");

	return DS18X20_OK;
}

#endif /* DS18X20_VERBOSE */

#if DS18X20_VERBOSE
#define uart_puts_P_verbose(s__)  serial_putstr_f((int8_t *)PSTR(s__))
#else
#define uart_puts_P_verbose(s__)
#endif

/*----------- end of "debug-functions" ---------------*/

/* find DS18X20 Sensors on 1-Wire-Bus
 input/ouput: diff is the result of the last rom-search
 *diff = OW_SEARCH_FIRST for first call
 output: id is the rom-code of the sensor found */
uint8_t DS18X20_find_sensor(uint8_t *diff, uint8_t id[]) {
	uint8_t go;
	uint8_t ret;

	ret = DS18X20_OK;
	go = 1;
	do {
		*diff = ow_rom_search(*diff, &id[0]);
		if (*diff == OW_PRESENCE_ERR || *diff == OW_DATA_ERR
				|| *diff == OW_LAST_DEVICE) {
			go = 0;
			ret = DS18X20_ERROR;
		} else {
			if (id[0] == DS18B20_FAMILY_CODE || id[0] == DS18S20_FAMILY_CODE
					|| id[0] == DS1822_FAMILY_CODE) {
				go = 0;
			}
		}
	}while (go);

	return ret;
}

/* get power status of DS18x20
 input:   id = rom_code
 returns: DS18X20_POWER_EXTERN or DS18X20_POWER_PARASITE */
uint8_t DS18X20_get_power_status(uint8_t id[]) {
	uint8_t pstat;

	ow_reset();
	ow_command(DS18X20_READ_POWER_SUPPLY, id);
	pstat = ow_bit_io(1);
	ow_reset();
	return (pstat) ? DS18X20_POWER_EXTERN : DS18X20_POWER_PARASITE;
}

/* start measurement (CONVERT_T) for all sensors if input id==NULL
 or for single sensor where id is the rom-code */
uint8_t DS18X20_start_meas(uint8_t with_power_extern, uint8_t id[]) {
	uint8_t ret;

	ow_reset();
	if (ow_input_pin_state()) { // only send if bus is "idle" = high
		if (with_power_extern != DS18X20_POWER_EXTERN) {
			ow_command_with_parasite_enable(DS18X20_CONVERT_T, id);
			/* not longer needed: ow_parasite_enable(); */
		} else {
			ow_command(DS18X20_CONVERT_T, id);
		}
		ret = DS18X20_OK;
	} else {
#if DS18X20_VERBOSE
		serial_puts_f( "DS18X20_start_meas: Short Circuit!\r\n");
#endif
		ret = DS18X20_START_FAIL;
	}

	return ret;
}

// returns 1 if conversion is in progress, 0 if finished
// not available when parasite powered.
uint8_t DS18X20_conversion_in_progress(void) {
	return ow_bit_io(1) ? DS18X20_CONVERSION_DONE : DS18X20_CONVERTING;
}

static uint8_t read_scratchpad(uint8_t id[], uint8_t sp[], uint8_t n) {
	uint8_t i;
	uint8_t ret;

	ow_command(DS18X20_READ, id);
	for (i = 0; i < n; i++) {
		sp[i] = ow_byte_rd();
	}
	if (crc8(&sp[0], DS18X20_SP_SIZE)) {
		ret = DS18X20_ERROR_CRC;
	} else {
		ret = DS18X20_OK;
	}

	return ret;
}

#if DS18X20_DECICELSIUS

/* convert scratchpad data to physical value in unit decicelsius */
static int16_t DS18X20_raw_to_decicelsius(uint8_t familycode, uint8_t sp[]) {
	uint16_t measure;
	uint8_t negative;
	int16_t decicelsius;
	uint16_t fract;

	measure = sp[0] | (sp[1] << 8);
	//measure = 0xFF5E; // test -10.125
	//measure = 0xFE6F; // test -25.0625

	if (familycode == DS18S20_FAMILY_CODE) { // 9 -> 12 bit if 18S20
		/* Extended measurements for DS18S20 contributed by Carsten Foss */
		measure &= (uint16_t) 0xfffe; // Discard LSB, needed for later extended precicion calc
		measure <<= 3;// Convert to 12-bit, now degrees are in 1/16 degrees units
		measure += (16 - sp[6]) - 4;// Add the compensation and remember to subtract 0.25 degree (4/16)
	}

	// check for negative
	if (measure & 0x8000) {
		negative = 1; // mark negative
		measure ^= 0xffff;// convert to positive => (twos complement)++
		measure++;
	} else {
		negative = 0;
	}

	// clear undefined bits for DS18B20 != 12bit resolution
	if (familycode == DS18B20_FAMILY_CODE || familycode == DS1822_FAMILY_CODE) {
		switch (sp[DS18B20_CONF_REG] & DS18B20_RES_MASK) {
			case DS18B20_9_BIT:
			measure &= ~(DS18B20_9_BIT_UNDF);
			break;
			case DS18B20_10_BIT:
			measure &= ~(DS18B20_10_BIT_UNDF);
			break;
			case DS18B20_11_BIT:
			measure &= ~(DS18B20_11_BIT_UNDF);
			break;
			default:
			// 12 bit - all bits valid
			break;
		}
	}

	decicelsius = (measure >> 4);
	decicelsius *= 10;

	// decicelsius += ((measure & 0x000F) * 640 + 512) / 1024;
	// 625/1000 = 640/1024
	fract = (measure & 0x000F) * 640;
	if (!negative) {
		fract += 512;
	}
	fract /= 1024;
	decicelsius += fract;

	if (negative) {
		decicelsius = -decicelsius;
	}

	if ( /* decicelsius == 850 || */decicelsius < -550 || decicelsius > 1250) {
		return DS18X20_INVALID_DECICELSIUS;
	} else {
		return decicelsius;
	}
}

/* format decicelsius-value into string, itoa method inspired
 by code from Chris Takahashi for the MSP430 libc, BSD-license
 modifications mthomas: variable-types, fixed radix 10, use div(),
 insert decimal-point */
uint8_t DS18X20_format_from_decicelsius(int16_t decicelsius, int8_t str[],
		uint8_t n) {
	uint8_t sign = 0;
	int8_t temp[7];
	int8_t temp_loc = 0;
	uint8_t str_loc = 0;
	div_t dt;
	uint8_t ret;

	// range from -550:-55.0°C to 1250:+125.0°C -> min. 6+1 chars
	if (n >= (6 + 1) && decicelsius > -1000 && decicelsius < 10000) {

		if (decicelsius < 0) {
			sign = 1;
			decicelsius = -decicelsius;
		}

		// construct a backward string of the number.
		do {
			dt = div(decicelsius, 10);
			temp[temp_loc++] = dt.rem + '0';
			decicelsius = dt.quot;
		}while (decicelsius > 0);

		if (sign) {
			temp[temp_loc] = '-';
		} else {
			///temp_loc--;
			temp[temp_loc] = '+';
		}

		// reverse the string.into the output
		while (temp_loc >= 0) {
			str[str_loc++] = temp[(uint8_t) temp_loc--];
			if (temp_loc == 0) {
				str[str_loc++] = DS18X20_DECIMAL_CHAR;
			}
		}
		str[str_loc] = '\0';

		ret = DS18X20_OK;
	} else {
		ret = DS18X20_ERROR;
	}

	return ret;
}

/* reads temperature (scratchpad) of sensor with rom-code id
 output: decicelsius
 returns DS18X20_OK on success */
uint8_t DS18X20_read_decicelsius(uint8_t id[], int16_t *decicelsius) {
	uint8_t sp[DS18X20_SP_SIZE];
	uint8_t ret;

	ow_reset();
	ret = read_scratchpad(id, sp, DS18X20_SP_SIZE);
	if (ret == DS18X20_OK) {
		*decicelsius = DS18X20_raw_to_decicelsius(id[0], sp);
	}
	return ret;
}

/* reads temperature (scratchpad) of sensor without id (single sensor)
 output: decicelsius
 returns DS18X20_OK on success */
uint8_t DS18X20_read_decicelsius_single(uint8_t familycode,
		int16_t *decicelsius) {
	uint8_t sp[DS18X20_SP_SIZE];
	uint8_t ret;

	ret = read_scratchpad(NULL, sp, DS18X20_SP_SIZE);
	if (ret == DS18X20_OK) {
		*decicelsius = DS18X20_raw_to_decicelsius(familycode, sp);
	}
	return ret;
}

#endif /* DS18X20_DECICELSIUS */

#if DS18X20_MAX_RESOLUTION

static int32_t DS18X20_raw_to_maxres(uint8_t familycode, uint8_t sp[]) {
	uint16_t measure;
	uint8_t negative;
	int32_t temperaturevalue;

	measure = sp[0] | (sp[1] << 8);
	//measure = 0xFF5E; // test -10.125
	//measure = 0xFE6F; // test -25.0625

	if (familycode == DS18S20_FAMILY_CODE) { // 9 -> 12 bit if 18S20
		/* Extended measurements for DS18S20 contributed by Carsten Foss */
		measure &= (uint16_t) 0xfffe; // Discard LSB, needed for later extended precicion calc
		measure <<= 3;// Convert to 12-bit, now degrees are in 1/16 degrees units
		measure += (16 - sp[6]) - 4;// Add the compensation and remember to subtract 0.25 degree (4/16)
	}

	// check for negative
	if (measure & 0x8000) {
		negative = 1; // mark negative
		measure ^= 0xffff;// convert to positive => (twos complement)++
		measure++;
	} else {
		negative = 0;
	}

	// clear undefined bits for DS18B20 != 12bit resolution
	if (familycode == DS18B20_FAMILY_CODE || familycode == DS1822_FAMILY_CODE) {
		switch (sp[DS18B20_CONF_REG] & DS18B20_RES_MASK) {
			case DS18B20_9_BIT:
			measure &= ~(DS18B20_9_BIT_UNDF);
			break;
			case DS18B20_10_BIT:
			measure &= ~(DS18B20_10_BIT_UNDF);
			break;
			case DS18B20_11_BIT:
			measure &= ~(DS18B20_11_BIT_UNDF);
			break;
			default:
			// 12 bit - all bits valid
			break;
		}
	}

	temperaturevalue = (measure >> 4);
	temperaturevalue *= 10000;
	temperaturevalue += (measure & 0x000F) * DS18X20_FRACCONV;

	if (negative) {
		temperaturevalue = -temperaturevalue;
	}

	return temperaturevalue;
}

uint8_t DS18X20_read_maxres(uint8_t id[], int32_t *temperaturevalue) {
	uint8_t sp[DS18X20_SP_SIZE];
	uint8_t ret;

	ow_reset();
	ret = read_scratchpad(id, sp, DS18X20_SP_SIZE);
	if (ret == DS18X20_OK) {
		*temperaturevalue = DS18X20_raw_to_maxres(id[0], sp);
	}
	return ret;
}

uint8_t DS18X20_read_maxres_single(uint8_t familycode,
		int32_t *temperaturevalue) {
	uint8_t sp[DS18X20_SP_SIZE];
	uint8_t ret;

	ret = read_scratchpad(NULL, sp, DS18X20_SP_SIZE);
	if (ret == DS18X20_OK) {
		*temperaturevalue = DS18X20_raw_to_maxres(familycode, sp);
	}
	return ret;

}

uint8_t DS18X20_format_from_maxres(int32_t temperaturevalue, char str[],
		uint8_t n) {
	uint8_t sign = 0;
	char temp[10];
	int8_t temp_loc = 0;
	uint8_t str_loc = 0;
	ldiv_t ldt;
	uint8_t ret;

	// range from -550000:-55.0000°C to 1250000:+125.0000°C -> min. 9+1 chars
	if (n >= (9 + 1) && temperaturevalue > -1000000L
			&& temperaturevalue < 10000000L) {

		if (temperaturevalue < 0) {
			sign = 1;
			temperaturevalue = -temperaturevalue;
		}

		do {
			ldt = ldiv(temperaturevalue, 10);
			temp[temp_loc++] = ldt.rem + '0';
			temperaturevalue = ldt.quot;
		}while (temperaturevalue > 0);

		// mk 20110209
		if ((temp_loc < 4) && (temp_loc > 1)) {
			temp[temp_loc++] = '0';
		} // mk end

		if (sign) {
			temp[temp_loc] = '-';
		} else {
			temp[temp_loc] = '+';
		}

		while (temp_loc >= 0) {
			str[str_loc++] = temp[(uint8_t) temp_loc--];
			if (temp_loc == 3) {
				str[str_loc++] = DS18X20_DECIMAL_CHAR;
			}
		}
		str[str_loc] = '\0';

		ret = DS18X20_OK;
	} else {
		ret = DS18X20_ERROR;
	}

	return ret;
}

#endif /* DS18X20_MAX_RESOLUTION */

#if DS18X20_EEPROMSUPPORT

uint8_t DS18X20_write_scratchpad(uint8_t id[], uint8_t th, uint8_t tl,
		uint8_t conf) {
	uint8_t ret;

	ow_reset();
	if (ow_input_pin_state()) { // only send if bus is "idle" = high
		ow_command(DS18X20_WRITE_SCRATCHPAD, id);
		ow_byte_wr(th);
		ow_byte_wr(tl);
		if (id[0] == DS18B20_FAMILY_CODE || id[0] == DS1822_FAMILY_CODE) {
			ow_byte_wr(conf); // config only available on DS18B20 and DS1822
		}
		ret = DS18X20_OK;
	} else {
		uart_puts_P_verbose( "DS18X20_write_scratchpad: Short Circuit!\r\n");
		ret = DS18X20_ERROR;
	}

	return ret;
}

uint8_t DS18X20_read_scratchpad(uint8_t id[], uint8_t sp[], uint8_t n) {
	uint8_t ret;

	ow_reset();
	if (ow_input_pin_state()) { // only send if bus is "idle" = high
		ret = read_scratchpad(id, sp, n);
	} else {
		uart_puts_P_verbose( "DS18X20_read_scratchpad: Short Circuit!\r\n");
		ret = DS18X20_ERROR;
	}

	return ret;
}

uint8_t DS18X20_scratchpad_to_eeprom(uint8_t with_power_extern, uint8_t id[]) {
	uint8_t ret;

	ow_reset();
	if (ow_input_pin_state()) { // only send if bus is "idle" = high
		if (with_power_extern != DS18X20_POWER_EXTERN) {
			ow_command_with_parasite_enable(DS18X20_COPY_SCRATCHPAD, id);
			/* not longer needed: ow_parasite_enable(); */
		} else {
			ow_command(DS18X20_COPY_SCRATCHPAD, id);
		}
		_delay_ms(DS18X20_COPYSP_DELAY); // wait for 10 ms
		if (with_power_extern != DS18X20_POWER_EXTERN) {
			ow_parasite_disable();
		}
		ret = DS18X20_OK;
	} else {
		uart_puts_P_verbose( "DS18X20_copy_scratchpad: Short Circuit!\r\n");
		ret = DS18X20_START_FAIL;
	}

	return ret;
}

uint8_t DS18X20_eeprom_to_scratchpad(uint8_t id[]) {
	uint8_t ret;
	uint8_t retry_count = 255;

	ow_reset();
	if (ow_input_pin_state()) { // only send if bus is "idle" = high
		ow_command(DS18X20_RECALL_E2, id);
		while (retry_count-- && !(ow_bit_io(1))) {
			;
		}
		if (retry_count) {
			ret = DS18X20_OK;
		} else {
			uart_puts_P_verbose( "DS18X20_recall_E2: timeout!\r\n");
			ret = DS18X20_ERROR;
		}
	} else {
		uart_puts_P_verbose( "DS18X20_recall_E2: Short Circuit!\r\n");
		ret = DS18X20_ERROR;
	}

	return ret;
}

#endif /* DS18X20_EEPROMSUPPORT */

#endif // ENABLE_DS18_2_
#endif //ENABLE_ONE_WIRE
//--
#ifdef ENABLE_IR
volatile uint8_t ir_readptr = 0;
volatile uint8_t ir_writeptr = 0;
volatile uint8_t ir_buffer[IR_BUFFER_SIZE];

/*****************************************************************************
 *  receive from IR Receiver - external interrupt 0 on pin PD2 (arduino pin 2)
 *                                                         PD0 on ATmega2560
 *
 * Use the external interrupt to decode the Sony IR protocol.  We use
 * both rising and falling edges to measure pulse widths.  The timer/counter
 * is needed to take these measurements.  If the address matches our device,
 * we store the command in a circular buffer for retrieval outside of
 * interrupt context.
 *
 * tested with Vishay TSOP392 IR receiver and universal remote
 ****************************************************************************/
//..
ISR(INT0_vect) {
	static uint8_t value;
	static uint8_t address;
	static uint8_t sigcount = 0;
#if defined(__AVR_ATmega48__)      || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)    || \
    defined(__AVR_ATmega16__)      || \
    defined(__AVR_ATmega16A__)     || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega32A__)     || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
	if (PIND & _BV(2)) { // rising edge
#elif defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__))
		if (PIND & _BV(0)) { // rising edge
#endif
			uint16_t count;
			count = TCNT0;

			onboard_led_off();

#if (F_CPU == 16000000)
			if (count < 14) { // 0
#endif
#if (F_CPU == 8000000)
				if (count < 7) { // 0
#endif
					sigcount++;
					if (sigcount < 8) {
						value = value & ~(1 << (sigcount - 1));
					} else {
						address = address & ~(1 << (sigcount - 8));
					}
#if (F_CPU == 16000000)
				} else if (count < 30) { // 1
#endif
#if (F_CPU == 8000000)
				} else if (count < 15) { // 1
#endif
					sigcount++;
					if (sigcount < 8) {
						value = value | (1 << (sigcount - 1));
					} else {
						address = address | (1 << (sigcount - 8));
					}
				} else { // this starts a new one
					sigcount = 0;
					value = 0;
					address = 0;
				}

				/* we have a command and it is for us */
				if ((sigcount == 12)
						&& ((address == 26) || (address == 17)
								|| (address == REMOTE_DEVICE_SONY_TV000))) {
					ir_buffer[ir_writeptr] = value;
					ir_writeptr = (ir_writeptr + 1) % IR_BUFFER_SIZE;
#ifdef IR_DEBOUNCE
					_delay_ms(200); /* debounce */
#endif
				}

			} else { // falling edge
				TCNT0 = 0;// reset
				onboard_led_on();
			}
		}

		/**************************************************************************
		 * ir_init
		 * AVR PD2 - arduino digital pin 2
		 **************************************************************************/
		void ir_init(void) {
#if defined(__AVR_ATmega48__)      || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)    || \
    defined(__AVR_ATmega16__)      || \
    defined(__AVR_ATmega16A__)     || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega32A__)     || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
			cbi(DDRD, 2);
#elif defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__))
			cbi(DDRD, 0);
#endif

			// 8-bit timer to count IR stuff
			TCCR0B |= _BV(CS02) | _BV(CS00);// CLK / 64 for 8mhz
			TCNT0 = 0;//reset the timer

			// we use external interrupt INT0 for the IR receiver
			EICRA |= _BV(ISC00);// interrupt on rising and falling edge of INT0
			EIMSK |= _BV(INT0);// enable INT0 interrupts

			ir_readptr = 0;
			ir_writeptr = 0;

			enable_onboard_led();

			// don't forget sei()
		}

		/*************************************************************************
		 * ir_get()
		 *
		 * blocks until we get the next IR value out of our buffer
		 * returns the value of the command
		 *************************************************************************/
		uint8_t ir_get(void) {
			uint8_t value;
			while (ir_writeptr == ir_readptr)
			; /* block waiting for a value */
			value = ir_buffer[ir_readptr]; /* pull out a sample*/
			ir_readptr = (ir_readptr + 1) % IR_BUFFER_SIZE;
			return value;
		}

		/*************************************************************************
		 * ir_get_nonblock()
		 *
		 * returns the value of the command
		 * returns 255 if there is not a new value for us
		 *************************************************************************/
		uint8_t ir_get_nonblock(void) {
			if (ir_writeptr == ir_readptr) /* don't block waiting for a value */
			return 255;

			uint8_t value;
			value = ir_buffer[ir_readptr]; /* pull out a sample */
			ir_readptr = (ir_readptr + 1) % IR_BUFFER_SIZE;
			return value;
		}
#endif // end IR
//--
#ifdef ENABLE_PWMSERVO
		/*************************************************************************
		 * pwmservo_init(pwmno)
		 * pwmno 1 & 2 are preferred b/c they provide higher resolution
		 * pwmno:
		 * 1 - OC1A (avr pin 15 PB1) - arduino digital pin 9
		 * 2 - OC1B (avr pin 16 PB2) - arduino digital pin 10
		 * 3 - OC2A (avr pin 17 PB3) - arduino digital pin 11
		 * 4 - OC2B (avr pin  5 PD3) - arduino digital pin 3
		 * 5 - OC0A (avr pin 12 PD6) - arduino digital pin 6  (conflicts with IR)
		 * 6 - OC0B (avr pin 11 PD5) - arduino digital pin 5  (conflicts with IR)
		 *************************************************************************/
		void pwmservo_init(uint8_t pwmno) {
			// FIXME: need to reserve pins and counters at compile time
			// FIXME: conflict w/ IR if it runs at 8MHz
			if (!pwmno || (pwmno > 6))///* invalid */
			return;

			if ((pwmno == 1) || (pwmno == 2))// /* TCNT1 */
			{
				if (pwmno == 1) {
#if defined(__AVR_ATmega16__)      || \
	defined(__AVR_ATmega16A__)     || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega32A__)     || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
					sbi(DDRD, DDD5);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
					sbi(DDRB, DDB5);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
					sbi(DDRB, 1);
#endif
					OCR1A = SERVO_MID_POS16; ///* initial value */
					TCCR1A |= _BV(COM1A1);///* turn on PWM 1 */
				}
				if (pwmno == 2) {
#if defined(__AVR_ATmega16__)      || \
	defined(__AVR_ATmega16A__)     || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega32A__)     || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
					sbi(DDRD, DDD4);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
					sbi(DDRB, DDB6);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
					sbi(DDRB, 2);
#endif
					OCR1B = SERVO_MID_POS16;
					TCCR1A |= _BV(COM1B1); ///* turn on PWM 2 */
				}
				TCCR1A |= _BV(WGM11); ///* PWM phase correct */
				TCCR1B |= _BV(WGM13) | _BV(WGM12);///* PWM phase correct */
				TCCR1B |= _BV(CS11);
#if (F_CPU == 16000000)
				ICR1 = 0x9C3F; ///* 16bit */
#endif
#if (F_CPU == 8000000)
				ICR1 = 0x4E1F; ///* 16bit */
#endif
				TCNT1H = 0; ///* initial value */
				TCNT1L = 0;///* initial value */
			}

			if ((pwmno == 3) || (pwmno == 4)) // /* TCNT1 */
			{
				if (pwmno == 3) {
#if defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__) // Sanguino
					sbi(DDRD, DDD7);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
					sbi(DDRB, DDB4);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
					sbi(DDRB, 3);
#endif
					OCR2A = SERVO_MID_POS8; // /* initial value */
					TCCR2A |= _BV(COM2A1);// /* turn on PWM 1 */
				}
				if (pwmno == 4) {
#if defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__) // Sanguino
					sbi(DDRD, DDD6);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
					sbi(DDRH, DDH6);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
					sbi(DDRD, 3);
#endif
					OCR2B = SERVO_MID_POS8;
					TCCR2A |= _BV(COM2B1); // /* turn on PWM 2 */
				}
#if (F_CPU == 16000000)
				TCCR2A |= _BV(WGM20) | _BV(WGM21); // /* fast PWM */
#endif
#if (F_CPU == 8000000)
				TCCR2A |= _BV(WGM20); ///* fast PWM */
#endif
				TCCR2B |= _BV(CS20) | _BV(CS21) | _BV(CS22);
				TCNT2 = 0; ///* initial value */
			}
#if !defined(ENABLE_IR) && !defined(ENABLE_MILLIS)
			// define the next two channels (5 & 6)
#endif
		}

		void __pwmservo_set(uint8_t servo, uint16_t pwmval) {
			//printf("__pwmservo_set setting %d\n\r", pwmval);
			if (servo == 1)
			OCR1A = pwmval;
			else if (servo == 2)
			OCR1B = pwmval;
			else if (servo == 3)
			OCR2A = pwmval;
			else if (servo == 4)
			OCR2B = pwmval;
		}

		/***************************************************************************
		 * pwmservo_set
		 * servo - 1-6 (servo number that we already called init on)
		 * pwmval - from 0-255 - provides the position for the servo
		 *          0 makes a 1.5ms pulse
		 *          255 makes a 2.5ms pulse
		 ***************************************************************************/
		void pwmservo_set(uint8_t servo, uint8_t pwmval) {
			//printf("pwmservo_set setting %d to %d\n\r", servo, pwmval);

			if ((servo == 1) || (servo == 2))
			__pwmservo_set(servo,
					(((uint32_t) pwmval
									* ((uint32_t) SERVO_MAX_POS16
											- (uint32_t) SERVO_MIN_POS16)) / (uint32_t) 256)
					+ (uint32_t) SERVO_MIN_POS16);

			if ((servo == 3) || (servo == 4))
			__pwmservo_set(servo,
					(((uint32_t) pwmval
									* ((uint32_t) SERVO_MAX_POS8 - (uint32_t) SERVO_MIN_POS8))
							/ (uint32_t) 256) + (uint32_t) SERVO_MIN_POS8);
		}

		/* not tested yet
		 void pwmservo_setf(uint8_t servo, float pwmval)
		 {
		 if (servo == 1)
		 OCR1A = pwmval * 255.0;
		 else if (servo == 2)
		 OCR1B = pwmval * 255.0;
		 } */
#endif //end PWM SERVO
#ifdef ENABLE_PWM
		/*************************************************************************
		 * pwm_init(pwmno)
		 * pwmno:
		 * 1 - OC1A (avr pin 15 PB1) - arduino digital pin 9
		 * 2 - OC1B (avr pin 16 PB2) - arduino digital pin 10
		 * 3 - OC2A (avr pin 17 PB3) - arduino digital pin 11
		 * 4 - OC2B (avr pin  5 PD3) - arduino digital pin 3
		 * 5 - OC0A (avr pin 12 PD6) - arduino digital pin 6  (conflicts with IR)
		 * 6 - OC0B (avr pin 11 PD5) - arduino digital pin 5  (conflicts with IR)
		 *************************************************************************/
		void pwm_init(uint8_t pwmno) {
			// FIXME: need to reserve pins and counters at compile time
			if (!pwmno || (pwmno > 6))// invalid
			return;

			if ((pwmno == 1) || (pwmno == 2))// /* TCNT1 */
			{
				if (pwmno == 1) {
#if defined(__AVR_ATmega16__)      || \
	defined(__AVR_ATmega16A__)     || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega32A__)     || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
					sbi(DDRD, DDD5);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
					sbi(DDRB, DDB5);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
					sbi(DDRB, 1);
#endif
					OCR1A = 0; ///* initial value */
					TCCR1A |= _BV(COM1A1);// /* turn on PWM 1 */
				}
				if (pwmno == 2) {
#if defined(__AVR_ATmega16__)      || \
	defined(__AVR_ATmega16A__)     || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega32A__)     || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
					sbi(DDRD, DDD4);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
					sbi(DDRB, DDB6);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
					sbi(DDRB, 2);
#endif
					OCR1B = 0;
					TCCR1A |= _BV(COM1B1); ///* turn on PWM 2 */
				}
				TCCR1A |= _BV(WGM10); ///* PWM 8bit */
				//TCCR1B |= _BV(WGM13) | _BV(WGM12);// /* PWM phase correct */
				TCCR1B |= _BV(CS12);// /* div by 256 */
				TCNT1H = 0;///* initial value */
				TCNT1L = 0;///* initial value */
			}

			if ((pwmno == 3) || (pwmno == 4)) ///* TCNT1 */
			{
				if (pwmno == 3) {
#if defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__) // Sanguino
					sbi(DDRD, DDD7);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
					sbi(DDRB, DDB4);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
					sbi(DDRB, 3);
#endif
					OCR2A = 0; ///* initial value */
					TCCR2A |= _BV(COM2A1);///* turn on PWM 1 */
				}
				if (pwmno == 4) {
#if defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__) // Sanguino
					sbi(DDRD, DDD6);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
					sbi(DDRH, DDH6);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
					sbi(DDRD, 3);
#endif
					OCR2B = 0;
					TCCR2A |= _BV(COM2B1); ///* turn on PWM 2 */
				}
				TCCR2A |= _BV(WGM20); ///* PWM 8bit */
				TCCR2B |= _BV(CS21) | _BV(CS22);// /* div by 256 for ovfl 244hz*/
				TCNT2 = 0;///* initial value */
			}
#if !defined(ENABLE_IR) && !defined(ENABLE_MILLIS)
			// define the next two channels (5 & 6)
#endif
		}

		/***************************************************************************
		 * pwm_set
		 * pwmchan - 1-6 (servo number that we already called init on)
		 * pwmval - from 0-255 - provides the position for the servo
		 *          0 makes a 1.5ms pulse
		 *          255 makes a 2.5ms pulse
		 ***************************************************************************/
		void pwm_set(uint8_t pwmchan, uint8_t pwmval) {
			//printf("__pwm_set setting %d\n\r", pwmval);
			if (pwmchan == 1)
			OCR1A = pwmval;
			else if (pwmchan == 2)
			OCR1B = pwmval;
			else if (pwmchan == 3)
			OCR2A = pwmval;
			else if (pwmchan == 4)
			OCR2B = pwmval;
		}

		/* not tested yet
		 void pwm_setf(uint8_t pwmchan, float pwmval)
		 {
		 if (pwmchan == 1)
		 OCR1A = pwmval * 255.0;
		 else if (servo == 2)
		 OCR1B = pwmval * 255.0;
		 } */
#endif
//--
#ifdef ENABLE_ADC
		/***************************************************************************
		 * adc_init()
		 *
		 * gets our ADC ready to take 10bit samples. See atmegaclib.h for
		 * ADC reference and prescaler definitions.
		 ***************************************************************************/
		void adc_init(uint8_t adc_reference, uint8_t adc_prescaler) {
			///* initialize the ADC - 10bit mode */
			ADMUX |= (adc_reference << 6);
			ADCSRA |= _BV(ADEN);// for now we don't do this in the ISR | _BV(ADIE);
			ADCSRA |= (adc_prescaler & 7);//adc prescaler
			//Powering ADC peripheral
#ifdef PRR0
			PRR0 &= ~_BV(PRADC);
#elif defined(PRR)
			PRR &= ~_BV(PRADC);
#endif
		}

		/***************************************************************************
		 * adc_get()
		 *
		 * adcnum - specifies which ADC pin you want to read from
		 *        - 0 through 5 corresponds to PC0-PC5 (32 pin SMD and 28 pin DIP capsule) and
		 *        - 0 through 7 corresponds to PA0-PA7 (44 pin SMD and 40 pin DIP capsule)
		 *        - 0 through 15 is for ATmega1280/2560
		 *
		 * returns 16 bit unsigned value between 0 and 1024 with 0 meaning 0v and
		 *          1024 meaning at or above voltage on AREF pin
		 ***************************************************************************/
		uint16_t adc_get(uint8_t adcnum) {
			static uint8_t current_adcnum = 17; // high enough to avoid conflict if atmega1280 support...
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
			adcnum &= 15;
#elif(defined(__AVR_ATmega16__)  || \
	defined(__AVR_ATmega16A__)     || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega32A__)     || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__))
			adcnum &= 7;
#elif defined(__AVR_ATmega48__)  || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
			adcnum &= 5;
#endif
#if defined(ADCSRB) && defined(MUX5)
			// the MUX5 bit of ADCSRB selects whether we're reading from channels
			// 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
			ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((adcnum >> 3) & 0x01) << MUX5);
#endif
			if (adcnum != current_adcnum) {
				ADMUX = (ADMUX & 0xF0) | adcnum; /* set up mux */
				current_adcnum = adcnum; /* cache for next time */
			}
			ADCSRA |= (1 << ADSC); /* start ADC conversion */
			while (ADCSRA & (1 << ADSC)) {
				;
			} /* block for the result */
			return ADC;
		}

		void adc_poweroff_digital_pinbuffer(uint8_t adcnum) {
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
			adcnum &= 15;
#elif(defined(__AVR_ATmega16__)    || \
	defined(__AVR_ATmega16A__)     || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega32A__)     || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__))
			adcnum &= 7;
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
			adcnum &= 5;
#endif
#if defined(DIDR0)
			sbi(DIDR0, adcnum);
#endif
#if defined(DIDR2)
			sbi(DIDR2, (adcnum & 7));
#endif
		}
#endif // ENABLE_ADC
//--
#ifdef ENABLE_LCD
		static void lcd_nibble(uint8_t d);
		static void lcd_byte(uint8_t d);

		uint8_t lcd_pos = LCD_LINE1;
		void lcd_init(void) {

			// set LCD DDR pins to 1 for output
			LCD_D4_DDR |= (1 << LCD_D4_PIN);
			LCD_D5_DDR |= (1 << LCD_D5_PIN);
			LCD_D6_DDR |= (1 << LCD_D6_PIN);
			LCD_D7_DDR |= (1 << LCD_D7_PIN);
			LCD_E_DDR |= (1 << LCD_E_PIN);
			LCD_RS_DDR |= (1 << LCD_RS_PIN);

			/*// set LCD DDR pins to 1 for output
			 sbi(LCD_D4_DDR,LCD_D4_PIN);
			 sbi(LCD_D5_DDR,LCD_D5_PIN);
			 sbi(LCD_D6_DDR,LCD_D6_PIN);
			 sbi(LCD_D7_DDR,LCD_D7_PIN);
			 sbi(LCD_E_DDR,LCD_E_PIN);
			 */sbi(LCD_RS_DDR, LCD_RS_PIN);
			/**/

// set the E and RS PORT pins to 0
			LCD_E_PORT &= ~(1 << LCD_E_PIN);
			LCD_RS_PORT &= ~(1 << LCD_RS_PIN);

			/*
			 // set the E and RS PORT pins to 0
			 cbi(LCD_E_PORT,LCD_E_PIN);
			 cbi(LCD_RS_PORT,LCD_RS_PIN);*//**/

			_delay_ms(15);
			lcd_nibble(0x30);
			_delay_ms(4.1);
			lcd_nibble(0x30);
			_delay_us(100);
			lcd_nibble(0x30);
			_delay_us(LCD_TIME_DAT);
			lcd_nibble(0x20);// 4 bit mode
			_delay_us(LCD_TIME_DAT);
#if LCD_LINE == 1
			lcd_command( 0x20 ); // 1 line
#else
			lcd_command(0x28); // 2 lines 5*7
#endif
			lcd_command(0x08); // display off
			lcd_command(0x01);// display clear
			lcd_command(0x06);// cursor increment
			lcd_command(0x0C);// on, no cursor, no blink

			// Set initial display conditions
			_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;

			// Initialize to default text direction (for romance languages)
			_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
			// set the entry mode
			lcd_command(LCD_ENTRYMODESET | _displaymode);
		}

		void lcd_clear() {
			lcd_command(0x01);
		}

		void lcd_home() {
			lcd_set_cursor(0, 0);
		}

		void lcd_putchar(uint8_t d) {
			sbi(LCD_RS_PORT, LCD_RS_PIN);

			lcd_byte(d);

			switch (++lcd_pos) {
				case LCD_LINE1 + LCD_COLUMN:
#ifdef LCD_LINE2
				d = LCD_LINE2;
				break;
				case LCD_LINE2 + LCD_COLUMN:
#ifdef LCD_LINE3
				d = LCD_LINE3;
				break;
				case LCD_LINE3 + LCD_COLUMN:
#ifdef LCD_LINE4
				d = LCD_LINE4;
				break;
				case LCD_LINE4 + LCD_COLUMN:
#endif
#endif
#endif
				d = LCD_LINE1;
				break;
				default:
				return;
			}
			lcd_command(d);
		}

		void lcd_putstr(int8_t *s) // display string from SRAM
		{
			for (int8_t *s1 = s; *s1; s1++) // until zero byte
			lcd_putchar((int8_t) *s1);
		}

		void lcd_putstr_f(int8_t *FlashString) {
			uint8_t i = 0;
			// Check for '\0' string terminator or maximum LCD width
			while (pgm_read_byte(&FlashString[i]) && (i < LCD_COLUMN)) {
				lcd_putchar(pgm_read_byte(&FlashString[i++]));
			}
		}

		void lcd_putint(int value) {
			int8_t string[18];
			itoa(value, (char *) string, 10);
			lcd_putstr((int8_t *) string);
		}

		void lcd_putU08(uint8_t value) {
			int8_t s[4];
			byte2dec(value, s);
			lcd_putstr(s);
		}

		void lcd_puthexU08(uint8_t value) {
			int8_t s[3];
			byte2hex(value, s);
			lcd_putstr(s);
		}

		void lcd_puthexU16(uint16_t value) {
			int8_t s[5];
			word2hex(value, s);
			lcd_putstr(s);
		}

		void lcd_blank(uint8_t len) // blank n digits
		{
			while (len--)
			lcd_putchar(' ');
		}

		void lcd_cursor_on(void) {
			_displaycontrol |= LCD_CURSORON;
			lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
		}
		void lcd_cursor_off(void) {
			_displaycontrol &= ~LCD_CURSORON;
			lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
		}

		void lcd_blink_on(void) {
			_displaycontrol |= LCD_BLINKON;
			lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
		}

		void lcd_blink_off(void) {
			_displaycontrol &= ~LCD_BLINKON;
			lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);

		}
		void lcd_display_on(void) {
			_displaycontrol |= LCD_DISPLAYON;
			lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);

		}
		void lcd_display_off(void) {
			_displaycontrol &= ~LCD_DISPLAYON;
			lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);

		}

// Private functions
		static void lcd_nibble(uint8_t d) {

			cbi(LCD_D7_PORT, LCD_D7_PIN);
			if (d & 1 << 7)
			sbi(LCD_D7_PORT, LCD_D7_PIN);
			cbi(LCD_D6_PORT, LCD_D6_PIN);
			if (d & 1 << 6)
			sbi(LCD_D6_PORT, LCD_D6_PIN);
			cbi(LCD_D5_PORT, LCD_D5_PIN);
			if (d & 1 << 5)
			sbi(LCD_D5_PORT, LCD_D5_PIN);
			cbi(LCD_D4_PORT, LCD_D4_PIN);
			if (d & 1 << 4)
			sbi(LCD_D4_PORT, LCD_D4_PIN);

			sbi(LCD_E_PORT, LCD_E_PIN);
			_delay_us(LCD_TIME_ENA);
			cbi(LCD_E_PORT, LCD_E_PIN);
		}

		static void lcd_byte(uint8_t d) {
			lcd_nibble(d);
			lcd_nibble(d << 4);
			_delay_us(LCD_TIME_DAT);
		}

		void lcd_command(uint8_t d) {
			cbi(LCD_RS_PORT, LCD_RS_PIN);
			lcd_byte(d);
			switch (d) {
				case 0 ... 3: // on longer commands
				_delay_us(LCD_TIME_CLR);
				d = LCD_LINE1;
				case 0x80 ... 0xFF:// set position
				lcd_pos = d;
				//break;
			}
		}

#endif //ENABLE_LCD
//--
#ifdef ENABLE_GLCD

		glcdCoord ks0108Coord;
		uint8_t ks0108Inverted = 0;
		ks0108FontCallback ks0108FontRead;
		uint8_t ks0108FontColor;
		const uint8_t* ks0108Font;

		void GLCD_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2,
				uint8_t color) {
			uint8_t length, i, y, yAlt, xTmp, yTmp;
			int16_t m;

			//
			// vertical line
			//
			if (x1 == x2) {
				// x1|y1 must be the upper point
				if (y1 > y2) {
					yTmp = y1;
					y1 = y2;
					y2 = yTmp;
				}
				GLCD_DrawVertLine(x1, y1, y2-y1, color);

				//
				// horizontal line
				//
			} else if (y1 == y2) {
				// x1|y1 must be the left point
				if (x1 > x2) {
					xTmp = x1;
					x1 = x2;
					x2 = xTmp;
				}
				GLCD_DrawHoriLine(x1, y1, x2-x1, color);

				//
				// schiefe line :)
				//
			} else {
				// angle >= 45°
				if ((y2 - y1) >= (x2 - x1) || (y1 - y2) >= (x2 - x1)) {
					// x1 must be smaller than x2
					if (x1 > x2) {
						xTmp = x1;
						yTmp = y1;
						x1 = x2;
						y1 = y2;
						x2 = xTmp;
						y2 = yTmp;
					}

					length = x2 - x1; // not really the length :)
					m = ((y2 - y1) * 200) / length;
					yAlt = y1;

					for (i = 0; i <= length; i++) {
						y = ((m * i) / 200) + y1;

						if ((m * i) % 200 >= 100)
						y++;
						else if ((m * i) % 200 <= -100)
						y--;

						GLCD_DrawLine(x1 + i, yAlt, x1 + i, y, color);

						if (length <= (y2 - y1) && y1 < y2)
						yAlt = y + 1;
						else if (length <= (y1 - y2) && y1 > y2)
						yAlt = y - 1;
						else
						yAlt = y;
					}

					// angle < 45°
				} else {
					// y1 must be smaller than y2
					if (y1 > y2) {
						xTmp = x1;
						yTmp = y1;
						x1 = x2;
						y1 = y2;
						x2 = xTmp;
						y2 = yTmp;
					}

					length = y2 - y1;
					m = ((x2 - x1) * 200) / length;
					yAlt = x1;

					for (i = 0; i <= length; i++) {
						y = ((m * i) / 200) + x1;

						if ((m * i) % 200 >= 100)
						y++;
						else if ((m * i) % 200 <= -100)
						y--;

						GLCD_DrawLine(yAlt, y1 + i, y, y1 + i, color);
						if (length <= (x2 - x1) && x1 < x2)
						yAlt = y + 1;
						else if (length <= (x1 - x2) && x1 > x2)
						yAlt = y - 1;
						else
						yAlt = y;
					}
				}
			}
		}

		void GLCD_DrawRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height,
				uint8_t color) {
			GLCD_DrawHoriLine(x, y, width, color);
			// top
			GLCD_DrawHoriLine(x, y+height, width, color);
			// bottom
			GLCD_DrawVertLine(x, y, height, color);
			// left
			GLCD_DrawVertLine(x+width, y, height, color);
			// right
		}

		void GLCD_DrawRoundRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height,
				uint8_t radius, uint8_t color) {
			int16_t tSwitch, x1 = 0, y1 = radius;
			tSwitch = 3 - 2 * radius;

			while (x1 <= y1) {
				GLCD_SetDot(x + radius - x1, y + radius - y1, color);
				GLCD_SetDot(x + radius - y1, y + radius - x1, color);

				GLCD_SetDot(x + width - radius + x1, y + radius - y1, color);
				GLCD_SetDot(x + width - radius + y1, y + radius - x1, color);

				GLCD_SetDot(x + width - radius + x1, y + height - radius + y1, color);
				GLCD_SetDot(x + width - radius + y1, y + height - radius + x1, color);

				GLCD_SetDot(x + radius - x1, y + height - radius + y1, color);
				GLCD_SetDot(x + radius - y1, y + height - radius + x1, color);

				if (tSwitch < 0) {
					tSwitch += (4 * x1 + 6);
				} else {
					tSwitch += (4 * (x1 - y1) + 10);
					y1--;
				}
				x1++;
			}

			GLCD_DrawHoriLine(x+radius, y, width-(2*radius), color);
			// top
			GLCD_DrawHoriLine(x+radius, y+height, width-(2*radius), color);
			// bottom
			GLCD_DrawVertLine(x, y+radius, height-(2*radius), color);
			// left
			GLCD_DrawVertLine(x+width, y+radius, height-(2*radius), color);
			// right
		}

		/*
		 * Hardware-Functions
		 */
		void GLCD_FillRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height,
				uint8_t color) {
			uint8_t mask, pageOffset, h, i, data;
			height++;

			pageOffset = y % 8;
			y -= pageOffset;
			mask = 0xFF;
			if (height < 8 - pageOffset) {
				mask >>= (8 - height);
				h = height;
			} else {
				h = 8 - pageOffset;
			}
			mask <<= pageOffset;

			GLCD_GotoXY(x, y);
			for (i = 0; i <= width; i++) {
				data = GLCD_ReadData();

				if (color == GLCD_BLACK) {
					data |= mask;
				} else {
					data &= ~mask;
				}

				GLCD_WriteData(data);
			}

			while (h + 8 <= height) {
				h += 8;
				y += 8;
				GLCD_GotoXY(x, y);

				for (i = 0; i <= width; i++) {
					GLCD_WriteData(color);
				}
			}

			if (h < height) {
				mask = ~(0xFF << (height - h));
				GLCD_GotoXY(x, y + 8);

				for (i = 0; i <= width; i++) {
					data = GLCD_ReadData();

					if (color == GLCD_BLACK) {
						data |= mask;
					} else {
						data &= ~mask;
					}

					GLCD_WriteData(data);
				}
			}
		}

		void GLCD_InvertRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height) {
			uint8_t mask, pageOffset, h, i, data, tmpData;
			height++;

			pageOffset = y % 8;
			y -= pageOffset;
			mask = 0xFF;
			if (height < 8 - pageOffset) {
				mask >>= (8 - height);
				h = height;
			} else {
				h = 8 - pageOffset;
			}
			mask <<= pageOffset;

			GLCD_GotoXY(x, y);
			for (i = 0; i <= width; i++) {
				data = GLCD_ReadData();
				tmpData = ~data;
				data = (tmpData & mask) | (data & ~mask);
				GLCD_WriteData(data);
			}

			while (h + 8 <= height) {
				h += 8;
				y += 8;
				GLCD_GotoXY(x, y);

				for (i = 0; i <= width; i++) {
					data = GLCD_ReadData();
					GLCD_WriteData(~data);
				}
			}

			if (h < height) {
				mask = ~(0xFF << (height - h));
				GLCD_GotoXY(x, y + 8);

				for (i = 0; i <= width; i++) {
					data = GLCD_ReadData();
					tmpData = ~data;
					data = (tmpData & mask) | (data & ~mask);
					GLCD_WriteData(data);
				}
			}
		}

		void GLCD_SetInverted(uint8_t invert) {
			if (ks0108Inverted != invert) {
				GLCD_InvertRect(0, 0, 127, 63);
				ks0108Inverted = invert;
			}
		}

		void GLCD_SetDot(uint8_t x, uint8_t y, uint8_t color) {
			uint8_t data;

			GLCD_GotoXY(x, y - y % 8); // read data from display memory
			data = GLCD_ReadData();

			if (color == GLCD_BLACK) {
				data |= 0x01 << (y % 8); // set dot
			} else {
				data &= ~(0x01 << (y % 8)); // clear dot
			}

			GLCD_WriteData(data); // write data back to display
		}

//
// Font Functions
//

		uint8_t GLCD_ReadFontData(const uint8_t* ptr) {
			return pgm_read_byte(ptr);
		}

		void GLCD_SelectFont(const uint8_t* font, ks0108FontCallback callback,
				uint8_t color) {
			ks0108Font = font;
			ks0108FontRead = callback;
			ks0108FontColor = color;
		}

		int GLCD_PutChar(char c) {
			uint8_t width = 0;
			uint8_t height = ks0108FontRead(ks0108Font + GLCD_FONT_HEIGHT);
			uint8_t bytes = (height + 7) / 8;

			uint8_t firstChar = ks0108FontRead(ks0108Font + GLCD_FONT_FIRST_CHAR);
			uint8_t charCount = ks0108FontRead(ks0108Font + GLCD_FONT_CHAR_COUNT);

			uint16_t index = 0;
			uint8_t x = ks0108Coord.x, y = ks0108Coord.y;

			if (c < firstChar || c >= (firstChar + charCount)) {
				return 1;
			}
			c -= firstChar;

			// read width data, to get the index
			for (uint8_t i = 0; i < c; i++) {
				index += ks0108FontRead(ks0108Font + GLCD_FONT_WIDTH_TABLE + i);
			}
			index = index * bytes + charCount + GLCD_FONT_WIDTH_TABLE;
			width = ks0108FontRead(ks0108Font + GLCD_FONT_WIDTH_TABLE + c);

			// last but not least, draw the character
			for (uint8_t i = 0; i < bytes; i++) {
				uint8_t page = i * width;
				for (uint8_t j = 0; j < width; j++) {
					uint8_t data = ks0108FontRead(ks0108Font + index + page + j);

					if (height < (i + 1) * 8) {
						data >>= (i + 1) * 8 - height;
					}

					if (ks0108FontColor == GLCD_BLACK) {
						GLCD_WriteData(data);
					} else {
						GLCD_WriteData(~data);
					}
				}
				// 1px gap between chars
				if (ks0108FontColor == GLCD_BLACK) {
					GLCD_WriteData(0x00);
				} else {
					GLCD_WriteData(0xFF);
				}
				GLCD_GotoXY(x, ks0108Coord.y + 8);
			}
			GLCD_GotoXY(x + width + 1, y);

			return 0;
		}

		void GLCD_Puts(char* str) {
			int x = ks0108Coord.x;
			while (*str != 0) {
				if (*str == '\n') {
					GLCD_GotoXY(x,
							ks0108Coord.y
							+ ks0108FontRead(ks0108Font + GLCD_FONT_HEIGHT));
				} else {
					GLCD_PutChar(*str);
				}
				str++;
			}
		}

		void GLCD_Puts_P(PGM_P str) {
			int x = ks0108Coord.x;
			while (pgm_read_byte(str) != 0) {
				if (pgm_read_byte(str) == '\n') {
					GLCD_GotoXY(x,
							ks0108Coord.y
							+ ks0108FontRead(ks0108Font + GLCD_FONT_HEIGHT));
				} else {
					GLCD_PutChar(pgm_read_byte(str));
				}
				str++;
			}
		}

		uint8_t GLCD_CharWidth(char c) {
			uint8_t width = 0;
			uint8_t firstChar = ks0108FontRead(ks0108Font + GLCD_FONT_FIRST_CHAR);
			uint8_t charCount = ks0108FontRead(ks0108Font + GLCD_FONT_CHAR_COUNT);

			// read width data
			if (c >= firstChar && c < (firstChar + charCount)) {
				c -= firstChar;
				width = ks0108FontRead(ks0108Font + GLCD_FONT_WIDTH_TABLE + c) + 1;
			}

			return width;
		}

		uint16_t GLCD_StringWidth(char* str) {
			uint16_t width = 0;

			while (*str != 0) {
				width += GLCD_CharWidth(*str++);
			}

			return width;
		}

		uint16_t GLCD_StringWidth_P(PGM_P str) {
			uint16_t width = 0;

			while (pgm_read_byte(str) != 0) {
				width += GLCD_CharWidth(pgm_read_byte(str++));
			}

			return width;
		}

		void GLCD_GotoXY(uint8_t x, uint8_t y) {
			uint8_t chip = GLCD_CHIP1, cmd;

			if (x > 127)
			x = 0; // ensure that coordinates are legal
			if (y > 63)
			y = 0;

			ks0108Coord.x = x;// save new coordinates
			ks0108Coord.y = y;
			ks0108Coord.page = y / 8;

			if (x >= 64) { // select the right chip
				x -= 64;
				chip = GLCD_CHIP2;
			}
			cmd = GLCD_SET_ADD | x;
			GLCD_WriteCommand(cmd, chip); // set x address on active chip

			cmd = GLCD_SET_PAGE | ks0108Coord.page;// set y address on both chips
			GLCD_WriteCommand(cmd, GLCD_CHIP1);
			GLCD_WriteCommand(cmd, GLCD_CHIP2);
		}

		void GLCD_Init(uint8_t invert) {
			ks0108Coord.x = 0;
			ks0108Coord.y = 0;
			ks0108Coord.page = 0;

			ks0108Inverted = invert;

			GLCD_CMD_DIR = 0xFF; // command port is output
			GLCD_WriteCommand(GLCD_ON, GLCD_CHIP1);// power on
			GLCD_WriteCommand(GLCD_ON, GLCD_CHIP2);

			GLCD_WriteCommand(GLCD_DISP_START, GLCD_CHIP1);// display start line = 0
			GLCD_WriteCommand(GLCD_DISP_START, GLCD_CHIP2);
			GLCD_ClearScreen();// display clear
			GLCD_GotoXY(0, 0);
		}

		inline void GLCD_Enable(void) {
			GLCD_CMD_PORT |= 0x01 << GLCD_EN; // EN high level width: min. 450ns
			asm volatile("nop\n\t"
					"nop\n\t"
					"nop\n\t"
					::);
			GLCD_CMD_PORT &= ~(0x01 << GLCD_EN);
			for (volatile uint8_t i = 0; i < 8; i++)
			;// a little delay loop (faster than reading the busy flag)
		}

		uint8_t GLCD_DoReadData(uint8_t first) {
			uint8_t data;
			volatile uint8_t i;

			GLCD_DATA_OUT = 0x00;
			GLCD_DATA_DIR = 0x00; // data port is input

			if (ks0108Coord.x < 64) {
				GLCD_CMD_PORT &= ~(0x01 << GLCD_CSEL2); // deselect chip 2
				GLCD_CMD_PORT |= 0x01 << GLCD_CSEL1;// select chip 1
			} else if (ks0108Coord.x >= 64) {
				GLCD_CMD_PORT &= ~(0x01 << GLCD_CSEL1); // deselect chip 1
				GLCD_CMD_PORT |= 0x01 << GLCD_CSEL2;// select chip 2
			}
			if (ks0108Coord.x == 64 && first) { // chip2 X-address = 0
				GLCD_WriteCommand(GLCD_SET_ADD, GLCD_CHIP2);// wuff wuff
			}

			GLCD_CMD_PORT |= 0x01 << GLCD_D_I; // D/I = 1
			GLCD_CMD_PORT |= 0x01 << GLCD_R_W;// R/W = 1

			GLCD_CMD_PORT |= 0x01 << GLCD_EN;// EN high level width: min. 450ns
			asm volatile("nop\n\t"
					"nop\n\t"
					"nop\n\t"
					::);

			data = GLCD_DATA_IN;// read Data

			GLCD_CMD_PORT &= ~(0x01 << GLCD_EN);
			for (i = 0; i < 8; i++)
			;// a little delay loop (faster than reading the busy flag)

			GLCD_DATA_DIR = 0xFF;

			GLCD_GotoXY(ks0108Coord.x, ks0108Coord.y);

			if (ks0108Inverted)
			data = ~data;
			return data;
		}

		inline uint8_t GLCD_ReadData(void) {
			GLCD_DoReadData(1); // dummy read
			return GLCD_DoReadData(0);// "real" read
		}

		void GLCD_WriteCommand(uint8_t cmd, uint8_t chip) {
			if (chip == GLCD_CHIP1) {
				GLCD_CMD_PORT &= ~(0x01 << GLCD_CSEL2); // deselect chip 2
				GLCD_CMD_PORT |= 0x01 << GLCD_CSEL1;// select chip 1
			} else if (chip == GLCD_CHIP2) {
				GLCD_CMD_PORT &= ~(0x01 << GLCD_CSEL1); // deselect chip 1
				GLCD_CMD_PORT |= 0x01 << GLCD_CSEL2;// select chip 2
			}

			GLCD_CMD_PORT &= ~(0x01 << GLCD_D_I); // D/I = 0
			GLCD_CMD_PORT &= ~(0x01 << GLCD_R_W);// R/W = 0
			GLCD_DATA_DIR = 0xFF;// data port is output
			GLCD_DATA_OUT = cmd;// write command
			GLCD_Enable();// enable
			GLCD_DATA_OUT = 0x00;
		}

		void GLCD_WriteData(uint8_t data) {
			uint8_t displayData, yOffset, cmdPort;

#ifdef DEBUG
			volatile uint16_t i;
			for(i=0; i<5000; i++);
#endif

			if (ks0108Coord.x >= 128)
			return;

			if (ks0108Coord.x < 64) {
				GLCD_CMD_PORT &= ~(0x01 << GLCD_CSEL2); // deselect chip 2
				GLCD_CMD_PORT |= 0x01 << GLCD_CSEL1;// select chip 1
			} else if (ks0108Coord.x >= 64) {
				GLCD_CMD_PORT &= ~(0x01 << GLCD_CSEL1); // deselect chip 1
				GLCD_CMD_PORT |= 0x01 << GLCD_CSEL2;// select chip 2
			}
			if (ks0108Coord.x == 64) // chip2 X-address = 0
			GLCD_WriteCommand(GLCD_SET_ADD, GLCD_CHIP2);

			GLCD_CMD_PORT |= 0x01 << GLCD_D_I;// D/I = 1
			GLCD_CMD_PORT &= ~(0x01 << GLCD_R_W);// R/W = 0
			GLCD_DATA_DIR = 0xFF;// data port is output

			yOffset = ks0108Coord.y % 8;
			if (yOffset != 0) {
				// first page
				cmdPort = GLCD_CMD_PORT;// save command port
				displayData = GLCD_ReadData();

				GLCD_CMD_PORT = cmdPort;// restore command port
				GLCD_DATA_DIR = 0xFF;// data port is output

				displayData |= data << yOffset;
				if (ks0108Inverted)
				displayData = ~displayData;
				GLCD_DATA_OUT = displayData;// write data
				GLCD_Enable();// enable

				// second page
				GLCD_GotoXY(ks0108Coord.x, ks0108Coord.y + 8);

				displayData = GLCD_ReadData();

				GLCD_CMD_PORT = cmdPort;// restore command port
				GLCD_DATA_DIR = 0xFF;// data port is output

				displayData |= data >> (8 - yOffset);
				if (ks0108Inverted)
				displayData = ~displayData;
				GLCD_DATA_OUT = displayData;// write data
				GLCD_Enable();// enable

				GLCD_GotoXY(ks0108Coord.x + 1, ks0108Coord.y - 8);
			} else {
				if (ks0108Inverted)
				data = ~data;
				GLCD_DATA_OUT = data; // write data
				GLCD_Enable();// enable
				ks0108Coord.x++;
			}
			GLCD_DATA_OUT = 0x00;
		}

#endif // ENABLE_GLCD
//--
#ifdef ENABLE_7SEG
#ifdef SEG_COMMON_ANODE
		const uint8_t seg_mask = 0xff;
#else
		const uint8_t seg_mask = 0x00;
#endif

#ifdef SEG_DP_PORT
#define SEG_NR_CHARS 38
#else
#define SEG_NR_CHARS 37
#endif

		const uint8_t PROGMEM seg_code[SEG_NR_CHARS]= {
			// index 0 is character 0
			SEG_A+SEG_B+SEG_C+SEG_D+SEG_E+SEG_F,
			// index 1 is character 1
			SEG_B+SEG_C,
			// index 2 is character 2
			SEG_A+SEG_B+SEG_D+SEG_E+SEG_G,
			// index 3 is character 3
			SEG_A+SEG_B+SEG_C+SEG_D+SEG_G,
			// index 4 is character 4
			SEG_F+SEG_G+SEG_B+SEG_C,
			// index 5 is character 5
			SEG_A+SEG_C+SEG_D+SEG_F+SEG_G,
			// index 6 is character 6
			SEG_A+SEG_C+SEG_D+SEG_E+SEG_F+SEG_G,
			// index 7 is character 7
			SEG_A+SEG_B+SEG_C,
			// index 8 is character 8
			SEG_A+SEG_B+SEG_C+SEG_D+SEG_E+SEG_F+SEG_G,
			// index 9 is character 9
			SEG_A+SEG_B+SEG_C+SEG_D+SEG_F+SEG_G,
			// index 10 is character A
			SEG_A+SEG_B+SEG_C+SEG_E+SEG_F+SEG_G,
			// index 11 is character b
			SEG_C+SEG_D+SEG_E+SEG_F+SEG_G,
			// index 12 is character C
			SEG_A+SEG_D+SEG_E+SEG_F,
			// index 13 is character d
			SEG_B+SEG_C+SEG_D+SEG_E+SEG_G,
			// index 14 is character E
			SEG_A+SEG_D+SEG_E+SEG_F+SEG_G,
			// index 15 is character F
			SEG_A+SEG_E+SEG_F+SEG_G,
			// index 16 is character S
			SEG_A+SEG_F+SEG_G+SEG_C+SEG_D,
			// index 17 is character c
			SEG_G+SEG_E+SEG_D,
			// index 18 is character r
			SEG_G+SEG_E,
			// index 19 is character H
			SEG_F+SEG_E+SEG_G+SEG_B+SEG_C,
			// index 20 is character i
			SEG_C,
			// index 21 is character L
			SEG_F+SEG_E+SEG_D,
			// index 22 is character o
			SEG_G+SEG_C+SEG_D+SEG_E,
			// index 23 is character P
			SEG_A+SEG_B+SEG_G+SEG_F+SEG_E,
			// index 24 is character U
			SEG_F+SEG_E+SEG_D+SEG_C+SEG_B,
			// index 25 is character u
			SEG_E+SEG_D+SEG_C,
			// index 26 is character h
			SEG_F+SEG_E+SEG_G+SEG_C,
			// index 27 is character Y
			SEG_F+SEG_G+SEG_B+SEG_C+SEG_D,
			// index 28 is character J
			SEG_B+SEG_C+SEG_D,
			// index 29 is character N
			SEG_E+SEG_F+SEG_A+SEG_B+SEG_C,
			// index 30 is character n
			SEG_C+SEG_G+SEG_E,
			// index 31 is character T
			SEG_A+SEG_F+SEG_E,
			// index 32 is character = (equal)
			SEG_G+SEG_D,
			// index 33 is character - (minus)
			SEG_G,
			// index 34 is character _ (underline)
			SEG_D,
			// index 35 is character G
			SEG_A+SEG_C+SEG_D+SEG_E+SEG_F,
			// index 36 is character space
			0
#ifdef SEG_DP_PORT
			// index 37 is character DP (dot) only if you enabled it in atmegaclib.h header...
			// Otherwise, your array ends at index 36.
			, SEG_DP
#endif
		};
		void seg_init(void) {
#ifdef SEG_DIGITS_8
			SEG_DIGITS_BUFFER[0] = 36;
			SEG_DIGITS_BUFFER[1] = 36;
			SEG_DIGITS_BUFFER[2] = 36;
			SEG_DIGITS_BUFFER[3] = 36;
			SEG_DIGITS_BUFFER[4] = 36;
			SEG_DIGITS_BUFFER[5] = 36;
			SEG_DIGITS_BUFFER[6] = 36;
			SEG_DIGITS_BUFFER[7] = 36;

#elif  defined(SEG_DIGITS_7)
			SEG_DIGITS_BUFFER[0] = 36;
			SEG_DIGITS_BUFFER[1] = 36;
			SEG_DIGITS_BUFFER[2] = 36;
			SEG_DIGITS_BUFFER[3] = 36;
			SEG_DIGITS_BUFFER[4] = 36;
			SEG_DIGITS_BUFFER[5] = 36;
			SEG_DIGITS_BUFFER[6] = 36;
#elif  defined(SEG_DIGITS_6)
			SEG_DIGITS_BUFFER[0] = 36;
			SEG_DIGITS_BUFFER[1] = 36;
			SEG_DIGITS_BUFFER[2] = 36;
			SEG_DIGITS_BUFFER[3] = 36;
			SEG_DIGITS_BUFFER[4] = 36;
			SEG_DIGITS_BUFFER[5] = 36;
#elif  defined(SEG_DIGITS_5)
			SEG_DIGITS_BUFFER[0] = 36;
			SEG_DIGITS_BUFFER[1] = 36;
			SEG_DIGITS_BUFFER[2] = 36;
			SEG_DIGITS_BUFFER[3] = 36;
			SEG_DIGITS_BUFFER[4] = 36;
#elif  defined(SEG_DIGITS_4)
			SEG_DIGITS_BUFFER[0] = 36;
			SEG_DIGITS_BUFFER[1] = 36;
			SEG_DIGITS_BUFFER[2] = 36;
			SEG_DIGITS_BUFFER[3] = 36;
#elif  defined(SEG_DIGITS_3)
			SEG_DIGITS_BUFFER[0] = 36;
			SEG_DIGITS_BUFFER[1] = 36;
			SEG_DIGITS_BUFFER[2] = 36;
#elif  defined(SEG_DIGITS_2)
			SEG_DIGITS_BUFFER[0] = 36;
			SEG_DIGITS_BUFFER[1] = 36;
#else
			SEG_DIGITS_BUFFER[0] = 36;
#endif
			// set direction for segment pins
#ifdef SEG_DP_DDR
			sbi(SEG_DP_DDR, SEG_DP_PIN);
#endif
			sbi(SEG_A_DDR, SEG_A_PIN);
			sbi(SEG_B_DDR, SEG_B_PIN);
			sbi(SEG_C_DDR, SEG_C_PIN);
			sbi(SEG_D_DDR, SEG_D_PIN);
			sbi(SEG_E_DDR, SEG_E_PIN);
			sbi(SEG_F_DDR, SEG_F_PIN);
			sbi(SEG_G_DDR, SEG_G_PIN);

			// set direction for common pins
#ifdef SEG_COMM_0_DDR
			sbi(SEG_COMM_0_DDR, SEG_COMM_0_PIN);
#endif
#ifdef SEG_COMM_1_DDR
			sbi(SEG_COMM_1_DDR, SEG_COMM_1_PIN);
#endif
#ifdef SEG_COMM_2_DDR
			sbi(SEG_COMM_2_DDR, SEG_COMM_2_PIN);
#endif
#ifdef SEG_COMM_3_DDR
			sbi(SEG_COMM_3_DDR, SEG_COMM_3_PIN);
#endif
#ifdef SEG_COMM_4_DDR
			sbi(SEG_COMM_4_DDR, SEG_COMM_4_PIN);
#endif
#ifdef SEG_COMM_5_DDR
			sbi(SEG_COMM_5_DDR, SEG_COMM_5_PIN);
#endif
#ifdef SEG_COMM_6_DDR
			sbi(SEG_COMM_6_DDR, SEG_COMM_6_PIN);
#endif
#ifdef SEG_COMM_7_DDR
			sbi(SEG_COMM_7_DDR, SEG_COMM_7_PIN);
#endif

		}

		uint8_t seg_digit_from_array(uint8_t index) {
			return seg_mask ^ pgm_read_byte(&seg_code[index]);
		}

		void seg_nibble(uint8_t segments) {
#ifdef SEG_DP_DDR
			cbi(SEG_DP_PORT, SEG_DP_PIN);
			if (bit_isset(segments,7))
			sbi(SEG_DP_PORT, SEG_DP_PIN);
#endif
			cbi(SEG_G_PORT, SEG_G_PIN);
			if (bit_isset(segments,6))
			sbi(SEG_G_PORT, SEG_G_PIN);
			cbi(SEG_F_PORT, SEG_F_PIN);
			if (bit_isset(segments,5))
			sbi(SEG_F_PORT, SEG_F_PIN);
			cbi(SEG_E_PORT, SEG_E_PIN);
			if (bit_isset(segments,4))
			sbi(SEG_E_PORT, SEG_E_PIN);
			cbi(SEG_D_PORT, SEG_D_PIN);
			if (bit_isset(segments,3))
			sbi(SEG_D_PORT, SEG_D_PIN);
			cbi(SEG_C_PORT, SEG_C_PIN);
			if (bit_isset(segments,2))
			sbi(SEG_C_PORT, SEG_C_PIN);
			cbi(SEG_B_PORT, SEG_B_PIN);
			if (bit_isset(segments,1))
			sbi(SEG_B_PORT, SEG_B_PIN);
			cbi(SEG_A_PORT, SEG_A_PIN);
			if (bit_isset(segments,0))
			sbi(SEG_A_PORT, SEG_A_PIN);
		}

		void seg_common_select(uint8_t digit) {
#ifdef SEG_COMM_7_PORT
			cbi(SEG_COMM_7_PORT,SEG_COMM_7_PIN);
			if(bit_isset(digit,7)) sbi(SEG_COMM_7_PORT,SEG_COMM_7_PIN);
#endif
#ifdef SEG_COMM_6_PORT
			cbi(SEG_COMM_6_PORT,SEG_COMM_6_PIN);
			if(bit_isset(digit,6)) sbi(SEG_COMM_6_PORT,SEG_COMM_6_PIN);
#endif
#ifdef SEG_COMM_5_PORT
			cbi(SEG_COMM_5_PORT,SEG_COMM_5_PIN);
			if(bit_isset(digit,5)) sbi(SEG_COMM_5_PORT,SEG_COMM_5_PIN);
#endif
#ifdef SEG_COMM_4_PORT
			cbi(SEG_COMM_4_PORT,SEG_COMM_4_PIN);
			if(bit_isset(digit,4)) sbi(SEG_COMM_4_PORT,SEG_COMM_4_PIN);
#endif
#ifdef SEG_COMM_3_PORT
			cbi(SEG_COMM_3_PORT, SEG_COMM_3_PIN);
			if (bit_isset(digit,3))
			sbi(SEG_COMM_3_PORT, SEG_COMM_3_PIN);
#endif
#ifdef SEG_COMM_2_PORT
			cbi(SEG_COMM_2_PORT, SEG_COMM_2_PIN);
			if (bit_isset(digit,2))
			sbi(SEG_COMM_2_PORT, SEG_COMM_2_PIN);
#endif
#ifdef SEG_COMM_1_PORT
			cbi(SEG_COMM_1_PORT, SEG_COMM_1_PIN);
			if (bit_isset(digit,1))
			sbi(SEG_COMM_1_PORT, SEG_COMM_1_PIN);
#endif
			// At least one pin must be set for selecting the digit
			// (minimum 1 digit, maximum 8 digits)
			cbi(SEG_COMM_0_PORT, SEG_COMM_0_PIN);
			if (bit_isset(digit,0))
			sbi(SEG_COMM_0_PORT, SEG_COMM_0_PIN);
		}

// Usage if common pin is active in 0 logic and you want to select the digit nr.4:
// seg_select_digit(SELECT_DIGIT_FOUR, 0);
		void seg_select_digit(MyDigit digit, uint8_t active_logic) {
			uint8_t digit_mask;
			if (active_logic == 0)
			digit_mask = 0xFF;
			else
			digit_mask = 0x00;
			seg_common_select(digit_mask ^ digit);
		}

#endif
//--
#ifdef ENABLE_I2C_SOFTWARE
		uint8_t I2C_write(uint8_t b) {
			uint8_t i;
			I2C_SDA_WR();
			for (i = 0; i < 8; i++) {
				if (b & 0x80)
				I2C_SDA_H();
				else
				I2C_SDA_L();
				_delay_us(10);
				I2C_SCL_H();
				_delay_us(10);
				I2C_SCL_L();
				b <<= 1;
			}
			I2C_SDA_RD();
			I2C_SDA_H();
			_delay_us(10);
			I2C_SCL_H();
			_delay_us(10);
			i = 0xFF;
			do {
				if (bit_is_clear(I2C_PORT_I,I2C_SDA))
				break;
				_delay_us(10);
			}while (--i > 0);
			I2C_SCL_L();
			_delay_us(10);
			return (i);
		}

		uint8_t I2C_read(uint8_t ack) {
			uint8_t i;
			uint8_t b = 0;
			I2C_SDA_RD();
			I2C_SDA_H();
			_delay_us(10);
			for (i = 0; i < 8; i++) {
				I2C_SCL_H();
				_delay_us(10);
				b <<= 1;
				if (bit_is_set(I2C_PORT_I,I2C_SDA))
				b |= 1;
				I2C_SCL_L();
				_delay_us(10);
			}
			I2C_SDA_WR();
			if (ack == 0)
			I2C_SDA_L();
			else
			I2C_SDA_H();
			_delay_us(10);
			I2C_SCL_H();
			_delay_us(10);
			I2C_SCL_L();
			_delay_us(10);
			I2C_SDA_L();
			_delay_us(10);
			return (b);
		}

		void I2C_start(void) {
			I2C_SCL_H();
			I2C_SDA_H();
			I2C_SDA_WR();
			I2C_SCL_WR();
			_delay_us(10);
			I2C_SDA_L();
			_delay_us(10);
			I2C_SCL_L();
			_delay_us(10);
		}

		void I2C_stop(void) {
			I2C_SDA_WR(); // SDA na zapis
			I2C_SCL_H();
			_delay_us(10);
			I2C_SDA_H();
			_delay_us(10);
		}
#endif // end I2C software
//--
#ifdef ENABLE_TWI

#define  TWI_START            0x08
#define  TWI_REP_START        0x10
#define  TWI_MT_SLA_ACK       0x18
#define  TWI_MT_SLA_NACK      0x20
#define  TWI_MT_DATA_ACK      0x28
#define  TWI_MT_DATA_NACK     0x30
#define  TWI_MR_SLA_ACK       0x40
#define  TWI_MR_SLA_NACK      0x48
#define  TWI_MR_DATA_ACK      0x50
#define  TWI_MR_DATA_NACK     0x58
#define  TWI_ARB_LOST         0x38

#define  TWI_ERROR_CODE   0x7e

		void TWI_init(void) {
			TWCR = 0x00; //disable twi
#if defined(TWPS0)
			TWSR = 0;
#endif
			TWBR = (F_CPU / TWI_FREQ - 16) / 2;

			// enable twi module, acks, and twi interrupt
			//TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
		}

//*************************************************
//Function to start i2c communication
//*************************************************
		uint8_t TWI_start(void) {
			TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); //Send START condition
			//Wait for TWINT flag set. This indicates that the
			//  START condition has been transmitted
			while (!(TWCR & (1 << TWINT)))
			;
			//Check value of TWI Status Register
			if ((TWSR & 0xF8) == TWI_START)
			return (0);
			else
			return (1);
		}

//*************************************************
//Function for repeat start condition
//*************************************************
		uint8_t TWI_repeatStart(void) {
			TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); //Send START condition
			//Wait for TWINT flag set. This indicates that the
			while (!(TWCR & (1 << TWINT)))
			;
			//START condition has been transmitted
			//Check value of TWI Status Register
			if ((TWSR & 0xF8) == TWI_REP_START)
			return (0);
			else
			return (1);
		}

//**************************************************
//Function to transmit address of the slave
//*************************************************
		uint8_t TWI_sendAddress(uint8_t address) {
			uint8_t STATUS;

			if ((address & 0x01) == 0)
			STATUS = TWI_MT_SLA_ACK;
			else
			STATUS = TWI_MR_SLA_ACK;

			TWDR = address;
			//Load SLA_W into TWDR Register. Clear TWINT bit
			//in TWCR to start transmission of address
			TWCR = (1 << TWINT) | (1 << TWEN);
			//Wait for TWINT flag set. This indicates that the SLA+W has been transmitted,
			// and ACK/NACK has been received.
			while (!(TWCR & (1 << TWINT)))
			;
			//Check value of TWI Status Register
			if ((TWSR & 0xF8) == STATUS)
			return (0);
			else
			return (1);
		}

//**************************************************
//Function to transmit a data byte
//*************************************************
		uint8_t TWI_sendData(uint8_t data) {
			TWDR = data;
			//Load SLA_W into TWDR Register. Clear TWINT bit
			//in TWCR to start transmission of data
			TWCR = (1 << TWINT) | (1 << TWEN);
			//Wait for TWINT flag set. This indicates that the data has been
			// transmitted, and ACK/NACK has been received.
			while (!(TWCR & (1 << TWINT)))
			;
			//Check value of TWI Status Register
			if ((TWSR & 0xF8) != TWI_MT_DATA_ACK)
			return (1);
			else
			return (0);
		}

//*****************************************************
//Function to receive a data byte and send ACKnowledge
//*****************************************************
		uint8_t TWI_receiveData_ACK(void) {
			uint8_t data;

			TWCR = (1 << TWEA) | (1 << TWINT) | (1 << TWEN);
			//Wait for TWINT flag set. This indicates that the data has been received
			while (!(TWCR & (1 << TWINT)))
			;
			//Check value of TWI Status Register
			if ((TWSR & 0xF8) != TWI_MR_DATA_ACK)
			return (TWI_ERROR_CODE);
			data = TWDR;
			return (data);
		}

//******************************************************************
//Function to receive the last data byte (no acknowledge from master
//******************************************************************
		uint8_t TWI_receiveData_NACK(void) {
			uint8_t data;

			TWCR = (1 << TWINT) | (1 << TWEN);
			//Wait for TWINT flag set. This indicates that the data has been received
			while (!(TWCR & (1 << TWINT)))
			;
			//Check value of TWI Status Register
			if ((TWSR & 0xF8) != TWI_MR_DATA_NACK)
			return (TWI_ERROR_CODE);
			data = TWDR;
			return (data);
		}

//**************************************************
//Function to end the i2c communication
//*************************************************
		void TWI_stop(void) {
			TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO); //Transmit STOP condition
		}
#endif // ENABLE_TWI
//--
#ifdef ENABLE_PCF8583
		uint8_t PCF8583_read(uint8_t address) {
#if defined(PCF8583_USE_TWI)
			uint8_t error, a, b;
			//a=(PCF8583_A0<<1)|0xa0;
			b = PCF8583_A0;
			a = (b << 1) | Physical_Address;
			error = TWI_start();
			error = TWI_sendAddress(a);
			error = TWI_sendData(address);
			error = TWI_repeatStart();
			error = TWI_sendAddress(a | 1);
			a = TWI_receiveData_NACK();
			TWI_stop();
			return a;
#else // use I2C Software
			uint8_t a, b;
			//a=(PCF8583_A0<<1)|0xa0;
			b = PCF8583_A0;
			a = (b << 1) | Physical_Address;
			I2C_start();
			I2C_write(a);
			I2C_write(address);
			I2C_start();
			I2C_write(a | 1);
			a = I2C_read(1);
			I2C_stop();
			return a;
#endif
		}

		void PCF8583_write(uint8_t address, uint8_t data) {
#if defined(PCF8583_USE_TWI)
			uint8_t a, b, error;
			b = PCF8583_A0;
			a = (b << 1) | Physical_Address;
			error = TWI_start();
			error = TWI_sendAddress(a);
			error = TWI_sendData(address);
			error = TWI_sendData(data);
			TWI_stop();
#else // use I2C Software
			uint8_t a, b;
			b = PCF8583_A0;
			a = (b << 1) | Physical_Address;
			I2C_start();
			I2C_write(a);
			I2C_write(address);
			I2C_write(data);
			I2C_stop();
#endif
		}

		uint8_t PCF8583_read_bcd(uint8_t address) {
			return bcd2bin(PCF8583_read(address));
		}

		void PCF8583_write_bcd(uint8_t address, uint8_t data) {
			PCF8583_write(address, bin2bcd(data));
		}

		volatile uint8_t PCF8583_status;
		volatile uint8_t PCF8583_alarm;

		uint8_t PCF8583_get_status(void) {
			PCF8583_status = PCF8583_read(0);
			PCF8583_alarm = (PCF8583_status & 2);
			return PCF8583_status;
		}

		void PCF8583_init(void) {
			PCF8583_status = 0;
			PCF8583_alarm = 0;
			PCF8583_write(0, 0);
			PCF8583_write(4, PCF8583_read(4) & 0x3f);
			PCF8583_write(8, 0x90);
		}

		void PCF8583_stop(void) {
			PCF8583_get_status();
			PCF8583_status |= 0x80;
			PCF8583_write(0, PCF8583_status);
		}

		void PCF8583_start(void) {
			PCF8583_get_status();
			PCF8583_status &= 0x7f;
			PCF8583_write(0, PCF8583_status);
		}

		void PCF8583_hold_off(void) {
			PCF8583_get_status();
			PCF8583_status &= 0xbf;
			PCF8583_write(0, PCF8583_status);
		}

		void PCF8583_hold_on(void) {
			PCF8583_get_status();
			PCF8583_status |= 0x40;
			PCF8583_write(0, PCF8583_status);
		}

		void PCF8583_alarm_off(void) {
			PCF8583_get_status();
			PCF8583_status &= 0xfb;
			PCF8583_write(0, PCF8583_status);
		}

		void PCF8583_alarm_on(void) {
			PCF8583_get_status();
			PCF8583_status |= 4;
			PCF8583_write(0, PCF8583_status);
		}

		void PCF8583_write_word(uint8_t address, uint16_t data) {
			PCF8583_write(address, (uint8_t) data & 0xff);
			PCF8583_write(++address, (uint8_t) (data >> 8));
		}

		void PCF8583_write_date(uint8_t address, uint8_t day, uint16_t year) {
			PCF8583_write(address, bin2bcd(day) | (((uint8_t) year & 3) << 6));
		}

		void PCF8583_get_time(uint8_t *hour, uint8_t *min, uint8_t *sec, uint8_t *hsec) {
			PCF8583_hold_on();
			*hsec = PCF8583_read_bcd(1);
			*sec = PCF8583_read_bcd(2);
			*min = PCF8583_read_bcd(3);
			*hour = PCF8583_read_bcd(4);
			PCF8583_hold_off();
		}

		void PCF8583_set_time(uint8_t hour, uint8_t min, uint8_t sec, uint8_t hsec) {
//  if (hour>23) hour=0;
//  if (min>59) min=0;
//  if (sec>59) sec=0;
//  if (hsec>100) hsec=0;
			PCF8583_stop();
			PCF8583_write_bcd(1, hsec);
			PCF8583_write_bcd(2, sec);
			PCF8583_write_bcd(3, min);
			PCF8583_write_bcd(4, hour);
			PCF8583_start();
		}

		void PCF8583_get_date(uint8_t *day, uint8_t *month, uint16_t *year) {
			uint8_t dy;
			uint16_t y1;
			PCF8583_hold_on();
			dy = PCF8583_read(5);
			*month = bcd2bin(PCF8583_read(6) & 0x1f);
			PCF8583_hold_off();
			*day = bcd2bin(dy & 0x3f);
			dy >>= 6;
			y1 = PCF8583_read(16) | ((uint16_t) PCF8583_read(17) << 8);
			if (((uint8_t) y1 & 3) != dy)
			PCF8583_write_word(16, ++y1);
			*year = y1;
		}

		void PCF8583_set_date(uint8_t day, uint8_t month, uint16_t year) {
			PCF8583_write_word(16, year);
			PCF8583_stop();
			PCF8583_write_date(5, day, year);
			PCF8583_write_bcd(6, month);
			PCF8583_start();
		}

		void PCF8583_get_alarm_time(uint8_t *hour, uint8_t *min, uint8_t *sec,
				uint8_t *hsec) {
			*hsec = PCF8583_read_bcd(0x9);
			*sec = PCF8583_read_bcd(0xa);
			*min = PCF8583_read_bcd(0xb);
			*hour = PCF8583_read_bcd(0xc);
		}

		void PCF8583_set_alarm_time(uint8_t hour, uint8_t min, uint8_t sec,
				uint8_t hsec) {
			PCF8583_write_bcd(0x9, hsec);
			PCF8583_write_bcd(0xa, sec);
			PCF8583_write_bcd(0xb, min);
			PCF8583_write_bcd(0xc, hour);
		}

		void PCF8583_get_alarm_date(uint8_t *day, uint8_t *month) {
			*day = bcd2bin(PCF8583_read(0xd) & 0x3f);
			*month = bcd2bin(PCF8583_read(0xe) & 0x1f);
		}

		void PCF8583_set_alarm_date(uint8_t day, uint8_t month) {
			PCF8583_write_date(0xd, day, 0);
			PCF8583_write_bcd(0xe, month);
		}
#endif //ENABLE_PCF8583
