/* *****************************************************************************
 * pwm.c
 * See atmegaclib2.h header for the copyrights...
 * *****************************************************************************
 */
#ifndef F_CPU
#define F_CPU 16000000UL //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include "atmegaclib2.h"

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
	if (!pwmno || (pwmno > 6)) // invalid
		return;

	if ((pwmno == 1) || (pwmno == 2)) // /* TCNT1 */
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
		defined(__AVR_ATmega48P__)     || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
			sbi(DDRB, 1);
#endif
			OCR1A = 0; ///* initial value */
			TCCR1A |= _BV(COM1A1); // /* turn on PWM 1 */
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
		defined(__AVR_ATmega48P__)     || \
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
		TCCR1B |= _BV(CS12); // /* div by 256 */
		TCNT1H = 0; ///* initial value */
		TCNT1L = 0; ///* initial value */
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
		defined(__AVR_ATmega48P__)     || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
			sbi(DDRB, 3);
#endif
			OCR2A = 0; ///* initial value */
			TCCR2A |= _BV(COM2A1); ///* turn on PWM 1 */
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
		defined(__AVR_ATmega48P__)     || \
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
		TCCR2B |= _BV(CS21) | _BV(CS22); // /* div by 256 for ovfl 244hz*/
		TCNT2 = 0; ///* initial value */
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

