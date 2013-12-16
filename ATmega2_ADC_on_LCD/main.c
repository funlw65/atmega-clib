/*
 * main.c
 *
 *  Created on: July 4, 2013
 *      Author: Vasile Guta Ciucur
 *
 * Description: It reads two 10K potentiometers connected to analog 0 and 1
 *              and displays the digital values on an LCD
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

/*
//Enable the following definitions on "atmegaclib2.h" header:

#define ENABLE_ADC        // analog to digital converter
#define ENABLE_CONVERSION // useful for Serial, LCD and 7SEG Display
#define ENABLE_LCD        // require CONVERSION

//Then, define the pins for LCD in the user zone, on the same header.

 */

#ifndef F_CPU
	#define F_CPU 16000000U //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h> // use it along with lcd_puts_f() to reduce RAM consumption
#include <atmegaclib2.h>
#include <atmegaclib2.c>

uint16_t adcval1, adcval2;

void main(void) __attribute__((noreturn)); //this allows me to have a void main() function
void main (void)
{
	// setting the pins direction...
#if defined(__AVR_ATmega48__)      || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
	cbi(DDRC, 0);//analog input - 10K pot.
	cbi(DDRC, 1);//analog input - 10K pot.
#elif defined(__AVR_ATmega16__)    || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
	cbi(DDRA, 0);//analog input - 10K pot.
	cbi(DDRA, 1);//analog input - 10K pot.
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	// I did not tested the ADC for these microcontrollers
	cbi(DDRF, 0);
	cbi(DDRF, 1);
#endif

	// other initializations
	//serial_init();
	adc_init(VREF_AVCC_CAP_AREF, ADC_PRESCALER_128);
	// "cut" the digital buffers from analog inputs
	adc_poweroff_digital_pinbuffer(0);
	adc_poweroff_digital_pinbuffer(1);
	lcd_init();
	lcd_puts_f("ADC ON LCD!");
	// --
	for(;;){
		//
		adcval1 = adc_get(0); // read analog input 0
		lcd_set_cursor(0, 1);// column, line
		lcd_puts_f("P1=    ");
		lcd_set_cursor(3, 1);
		lcd_putint(adcval1);
		_delay_ms(250);
		adcval2 = adc_get(1); // read analog input 1
		lcd_set_cursor(8, 1);// column, line
		lcd_puts_f("P2=    ");
		lcd_set_cursor(11, 1);
		lcd_putint(adcval2);
		_delay_ms(250);
	}
}
