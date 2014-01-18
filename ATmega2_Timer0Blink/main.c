/*
 * main.c
 *
 *  Created on: Jun 19, 2013
 *      Author: Vasile Guta Ciucur
 *  Description: Blinks 3 LEDs on 8Bit Offense board (Sanguino like) using
 *                 non-blocking delays:
 *               Red    LED on RD5 blinks at 250 milliseconds,
 *               Green  LED on RC6 blinks at 500 milliseconds,
 *               Yellow LED on RC7 blinks at 750 milliseconds
 * If you want to see the board, here is the link:
 * http://myshed.wordpress.com/2012/10/02/un-fel-de-sanguino/
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

//#define  ENABLE_NB_DELAYS // Non-blocking, slotted delays (instead of millis()) using Timer0

// Scroll down, still in the user zone, and find the definition for DELAY_SLOTS;
// change it to 3 if is not already (we will need 3 delays, one for each LED) so, you have:

// #define DELAY_SLOTS  3 // the number of nb. delays



#ifndef F_CPU
#define F_CPU 16000000U // required by Atmel Studio 6
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
//#include <util/delay.h>
#include "atmegaclib2.h"
#include <nbdelays.c>


void main(void) __attribute__((noreturn));
void main(void) {
	// setting (LED) pins as outputs
	sbi(DDRD, 5);
	sbi(DDRC, 6);
	sbi(DDRC, 7);
	// init the non_blocking delays
	timer0_isr_init();
	sei();
	// set and start the delays in milliseconds for every LED
	set_delay(0, 250);
	set_delay(1, 500);
	set_delay(2, 750);
	for(;;){
		// check if delay 0 (250ms) expired
		if(check_delay(0)){
			// toggle the LED on pin D5
			tbi(PORTD, 5);
			// set and start the delay again
			set_delay(0, 250);
		}
		// check if delay 1 (500ms) expired
		if(check_delay(1)){
			// toggle the LED on pin C6
			tbi(PORTC, 6);
			// set and start the delay again
			set_delay(1, 500);
		}
		// check if delay 3 (750ms) expired
		if(check_delay(2)){
			// toggle the LED on pin C7
			tbi(PORTC, 7);
			// set and start the delay again
			set_delay(2, 750);
		}
	}
}

