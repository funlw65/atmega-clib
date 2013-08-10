/*
 * main.c
 *
 *  Created on: Aug 10, 2013
 *      Author: http://www.pjrc.com/teensy/td_libs_FreqMeasure.html
 *  Adapted by: Vasile Guta Ciucur
 *
 *  Description: Program test for FreqMeasure functions.
 *
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
//#define UART_BAUD_RATE     57600 // default is 57600
//#define UART_BAUD_SELECT   (F_CPU / (UART_BAUD_RATE * 16L) - 1) //- don't touch
//#define ENABLE_SERIAL_POLL // require CONVERSION, conflicts with SERIAL
//#define ENABLE_FREQMEASURE   // it can use one of TIMER1, TIMER3, TIMER4, TIMER5
//#define ENABLE_CONVERSION    // useful for Serial, LCD and 7SEG Display

//For Arduino UNO and Sanguino we have only one pin for counting frequency.
// which is PB0 for Arduino Uno and PD6 for Sanguino

#ifndef F_CPU
#define F_CPU 16000000U // required by Atmel Studio 6
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h> // use with dtostrf() function
#include <atmegaclib2.h>

float sum = 0;
int count = 0;
char strbuff[8]; // adjust as you need

void main(void) __attribute__((noreturn));
void main(void) {
	sei();
	serial_init();
	FreqMeasure_begin();

	for (;;) {
		//
		if (FreqMeasure_available()) {
			//
			sum = sum + FreqMeasure_read();
			count = count + 1;
			if (count > 30) {
				float frequency = F_CPU / (sum / count);
				dtostrf(frequency, 4, 2, strbuff); //tune as you need.
				serial_puts(strbuff);
				serial_puts("\r\n");
				sum = 0;
				count = 0;
			}
		}
	}
}

