/* *****************************************************************************
 * adc.c
 * See atmegaclib2.h header for the copyrights...
 * *****************************************************************************
 */
#ifndef F_CPU
#define F_CPU 16000000UL //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include "atmegaclib2.h"

//--
/***************************************************************************
 * adc_init()
 *
 * gets our ADC ready to take 10bit samples. See atmegaclib.h for
 * ADC reference and prescaler definitions.
 ***************************************************************************/
void adc_init(uint8_t adc_reference, uint8_t adc_prescaler) {
	///* initialize the ADC - 10bit mode */
	ADMUX |= (adc_reference << 6);
	ADCSRA |= _BV(ADEN); // for now we don't do this in the ISR | _BV(ADIE);
	ADCSRA |= (adc_prescaler & 7); //adc prescaler
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
		defined(__AVR_ATmega48P__)     || \
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
		defined(__AVR_ATmega48P__)     || \
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

