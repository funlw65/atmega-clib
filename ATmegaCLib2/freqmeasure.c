/* *****************************************************************************
 * freqmeasure.c
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



