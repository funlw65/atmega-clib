/* *****************************************************************************
 * nbdelays.c
 * See atmegaclib2.h header for the copyrights...
 * *****************************************************************************
 */
#ifndef F_CPU
#define F_CPU 16000000UL //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "atmegaclib2.h"


//#define DELAY_SLOTS  3 // the number of non-blocking delays we need
//int16_t isr_countdowns[DELAY_SLOTS]; // the array where the "delays" are stored

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



