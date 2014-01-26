/*
 * aduinocompat.c
 * Created on Jan 26, 2014
 */

#include "atmegaclib2.h"

#if defined(__AVR_ATmega48__)    || \
	defined(__AVR_ATmega48P__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)   // Arduino 28 pins
void pinMode(uint8_t pin, uint8_t mode)
{
	if (pin < 8)
	{
		if (mode == INPUT)
		setpin_in(DDRD, pin);
		else
		setpin_out(DDRD, pin);
	} else if(pin < 14) {
		pin = pin - 8;
		if (mode == INPUT)
		setpin_in(DDRB, pin);
		else
		setpin_out(DDRB, pin);
	} else {
		pin = pin - 14;
		if (mode == INPUT)
		setpin_in(DDRC, pin);
		else
		setpin_out(DDRC, pin);

	}
}

void digitalWrite(uint8_t pin, uint8_t value)
{
	if (pin < 8)
	{
		if (value > LOW)
		setpin(PORTD, pin);
		else
		clearpin(PORTD, pin);
	} else if(pin < 14) {
		pin = pin - 8;
		if (value > LOW)
		setpin(PORTB, pin);
		else
		clearpin(PORTB, pin);
	} else {
		pin = pin - 14;
		if (value > LOW)
		setpin(PORTC, pin);
		else
		clearpin(PORTC, pin);

	}
}

uint8_t digitalRead(uint8_t pin)
{
	if (pin < 8)
	return (PIND & (1<<pin));
	else if(pin < 14) {
		pin = pin - 8;
		return (PINB & (1<<pin));
	} else {
		pin = pin - 14;
		return (PINC & (1<<pin));

	}
}

#ifdef ENABLE_PWM
/* we have arduino api compat and pwm enabled */
void analogWrite(uint8_t pin, uint8_t value)
{
	switch(pin) {
		case 9:
		pwm_set(1, value);
		break;
		case 10:
		pwm_set(2, value);
		break;
		case 11:
		pwm_set(3, value);
		break;
		case 3:
		pwm_set(4, value);
		break;
		default:
		return;
		break;
	}
}
#endif

#elif defined(__AVR_ATmega16__)      || \
		defined(__AVR_ATmega16A__)     || \
	    defined(__AVR_ATmega164P__)    || \
	    defined(__AVR_ATmega32__)      || \
	    defined(__AVR_ATmega32A__)     || \
	    defined(__AVR_ATmega324P__)    || \
	    defined(__AVR_ATmega324PA__)   || \
	    defined(__AVR_ATmega644__)     || \
	    defined(__AVR_ATmega644P__)    || \
	    defined(__AVR_ATmega1284P__)

void pinMode(uint8_t pin, uint8_t mode) {
	if (pin < 8) {
		if (mode == INPUT)
			setpin_in(DDRB, pin);
		else
			setpin_out(DDRB, pin);
	} else if (pin < 16) {
		pin = pin - 8;
		if (mode == INPUT)
			setpin_in(DDRD, pin);
		else
			setpin_out(DDRD, pin);
	} else if (pin < 24) {
		pin = pin - 16;
		if (mode == INPUT)
			setpin_in(DDRC, pin);
		else
			setpin_out(DDRC, pin);

	} else {
		pin = pin - 24;
		if (mode == INPUT)
			setpin_in(DDRA, pin);
		else
			setpin_out(DDRA, pin);

	}
}

void digitalWrite(uint8_t pin, uint8_t value) {
	if (pin < 8) {
		if (value > LOW)
			setpin(PORTB, pin);
		else
			clearpin(PORTB, pin);
	} else if (pin < 16) {
		pin = pin - 8;
		if (value > LOW)
			setpin(PORTD, pin);
		else
			clearpin(PORTD, pin);
	} else if (pin < 24) {
		pin = pin - 16;
		if (value > LOW)
			setpin(PORTC, pin);
		else
			clearpin(PORTC, pin);

	} else {
		pin = pin - 24;
		if (value > LOW)
			setpin(PORTA, pin);
		else
			clearpin(PORTA, pin);

	}
}

uint8_t digitalRead(uint8_t pin) {
	if (pin < 8)
		return (PINB & (1 << pin));
	else if (pin < 16) {
		pin = pin - 8;
		return (PIND & (1 << pin));
	} else if (pin < 24){
		pin = pin - 14;
		return (PINC & (1 << pin));

	} else {
		pin = pin - 24;
		return (PINA & (1 << pin));

	}
}

#ifdef ENABLE_PWM
/* we have arduino api compat and pwm enabled */
void analogWrite(uint8_t pin, uint8_t value)
{
	switch(pin) {
		case 9:
		pwm_set(1, value);
		break;
		case 10:
		pwm_set(2, value);
		break;
		case 11:
		pwm_set(3, value);
		break;
		case 3:
		pwm_set(4, value);
		break;
		default:
		return;
		break;
	}
}
#endif

#endif

