/*
 * main.c
 *
 *  Created on: Aug 25, 2012
 *      Author: Vasile Guta Ciucur
 *
 *  Program: Displays the ADC value on 4 digits 7 segment.
 *  Please, configure everything on atmegaclib.h header according with your hardware:
 *  - number of digits, buffer index (which must be identical with number of digits), etc...
 *******************************************************************************
 *  BSD License.
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

//Enable the following definitions on "atmegaclib.h" header:
//#define ENABLE_ADC        // analog to digital converter
//#define ENABLE_CONVERSION // useful for Serial, LCD and 7SEG Display
//#define ENABLE_7SEG       // starting from one digit, up to eight digits.
//Then, define the pins for 7seg display and the required number of
// digits in the user zone, on the same header.

#ifndef F_CPU
#define F_CPU 16000000U //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include <util/delay.h>
#include "atmegaclib2.h"
#include <adc.c>
#include <conversion.c>
#include <7seg.c>

uint8_t i;

void seg_adc_to_buffer(uint16_t adc_value) {
	if (adc_value > 999) {
		SEG_DIGITS_BUFFER[0] = adc_value % 10;
		adc_value /= 10;
		SEG_DIGITS_BUFFER[1] = adc_value % 10;
		adc_value /= 10;
		SEG_DIGITS_BUFFER[2] = adc_value % 10;
		adc_value /= 10;
		SEG_DIGITS_BUFFER[3] = adc_value;
	} else if (adc_value > 99) {
		SEG_DIGITS_BUFFER[0] = adc_value % 10;
		adc_value /= 10;
		SEG_DIGITS_BUFFER[1] = adc_value % 10;
		adc_value /= 10;
		SEG_DIGITS_BUFFER[2] = adc_value;
		SEG_DIGITS_BUFFER[3] = 36;
	} else if (adc_value > 9) {
		SEG_DIGITS_BUFFER[0] = adc_value % 10;
		adc_value /= 10;
		SEG_DIGITS_BUFFER[1] = adc_value;
		SEG_DIGITS_BUFFER[2] = 36;
		SEG_DIGITS_BUFFER[3] = 36;
	} else {
		SEG_DIGITS_BUFFER[0] = adc_value;
		SEG_DIGITS_BUFFER[1] = 36;
		SEG_DIGITS_BUFFER[2] = 36;
		SEG_DIGITS_BUFFER[3] = 36;
	}
}

void seg_display_from_buffer(void) {
	seg_select_digit(SELECT_DIGIT_FOUR, 0);
	seg_nibble(seg_digit_from_array(36)); // turn off digit
	seg_nibble(seg_digit_from_array(SEG_DIGITS_BUFFER[3]));
	_delay_us(SEG_DELAY);
	seg_select_digit(SELECT_DIGIT_THREE, 0);
	seg_nibble(seg_digit_from_array(36)); // turn off digit
	seg_nibble(seg_digit_from_array(SEG_DIGITS_BUFFER[2]));
	_delay_us(SEG_DELAY);
	seg_select_digit(SELECT_DIGIT_TWO, 0);
	seg_nibble(seg_digit_from_array(36)); // turn off digit
	seg_nibble(seg_digit_from_array(SEG_DIGITS_BUFFER[1]));
	_delay_us(SEG_DELAY);
	seg_select_digit(SELECT_DIGIT_ONE, 0);
	seg_nibble(seg_digit_from_array(36)); // turn off digit
	seg_nibble(seg_digit_from_array(SEG_DIGITS_BUFFER[0]));
	_delay_us(SEG_DELAY);
}

void main(void) __attribute__((noreturn)); //this allows me to have a void main() function
void main(void) {
	// setting the pins direction...
#if defined(__AVR_ATmega48__)   || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
	cbi(DDRC, 0); //analog input - 10K pot.
#elif defined(__AVR_ATmega16__)   || \
	defined(__AVR_ATmega16A__)   || \
    defined(__AVR_ATmega164__)     || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega164PA__)   || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega324__)     || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega328P__)    || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega644PA__)   || \
    defined(__AVR_ATmega1284P__)   || \
    defined(__AVR_ATmega1284__)
	cbi(DDRA, 0);
	//analog input - 10K pot.
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	// I did not tested the ADC for these microcontrollers
	cbi(DDRF, 0);
#endif

	// initializes the buffer, (involved) pins direction
	seg_init();

	adc_init(VREF_AVCC_CAP_AREF, ADC_PRESCALER_128);
	//power off the digital buffers from analog inputs
	adc_poweroff_digital_pinbuffer(0);
	// --
	while (1) {
		//
		seg_adc_to_buffer(adc_get(0));
		seg_display_from_buffer();
		//
	}
}
