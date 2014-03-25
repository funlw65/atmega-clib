/*
 * main.c
 * Created on Jan 23, 2014
 * by Vasile
 * Description: 
 *   The original "BlinkWithoutSelay" Arduino sample using millis().
 *
 * Copyright and License:
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
 *  Now the project have his own atmegaclib2.h header.
 */

#ifndef F_CPU
#define F_CPU 16000000U // required by Atmel Studio 6
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include "atmegaclib2.h"
#include <atmegaclib2_files.c>

uint32_t previousMillis = 0; // will store last time when millis LED was updated
uint32_t interval       = 1000; //1000 milliseconds delay
//uint8_t  ledState       = 0;

void main(void) __attribute__((noreturn));
void main(void) {
	millis_init();
	onboard_led_enable();
	sei();
	while (1) {
		//
		uint32_t currentMillis = millis();
		if (currentMillis - previousMillis > interval) {
			previousMillis = currentMillis;
			onboard_led_toggle();
		}
	}
}
