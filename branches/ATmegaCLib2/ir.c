/* *****************************************************************************
 * ir.c
 * See atmegaclib2.h header for the copyrights...
 * *****************************************************************************
 */
#ifndef F_CPU
#define F_CPU 16000000UL //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include "atmegaclib2.h"

//--
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
		TCNT0 = 0; // reset
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
	TCCR0B |= _BV(CS02) | _BV(CS00); // CLK / 64 for 8mhz
	TCNT0 = 0; //reset the timer

	// we use external interrupt INT0 for the IR receiver
	EICRA |= _BV(ISC00); // interrupt on rising and falling edge of INT0
	EIMSK |= _BV(INT0); // enable INT0 interrupts

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

