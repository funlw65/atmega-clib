/*
 * main.c
 * Created on Jan 26, 2014
 * by Vasile Guta Ciucur
 * Description: just translated the example from here:
 * http://code.google.com/p/ad9850-arduino/source/browse/trunk/examples/Sweep/Sweep.ino
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

/*
 *  A schematic and an article to be written ...
 */

/*
 *  Note: If you don't use Arduino compatibility mode,
 *        then you should define the pins and ports inside atmegaclib.h
 *        Look for "#ifdef ENABLE_ARDUINO_COMPAT" sections
 */

#ifndef F_CPU
#define F_CPU 16000000U //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include "atmegaclib2.h"
#include <atmegaclib2_files.c>

void main(void) __attribute__((noreturn)); //this allows me to have a void main() function
void main(void) {
#ifdef ENABLE_ARDUINO_COMPAT
#if defined(__AVR_ATmega48__)      || \
	defined(__AVR_ATmega48P__)     || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
	AD9850_init(5, 6, 7); //Arduino UNO
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
	AD9850_init(21, 22, 23); //Sanguino
#endif
#else
	AD9850_init(); //pins are defined inside atmegaclib2.h header
#endif
	/* NOTE: For device to start-up in serial mode,
	 hardwire pin 2 at 0, pin 3 at 1, and pin 4 at 1 */
	//don't know what this means

	// sweep form 1MHz to 10MHz
	for (uint32_t i = 1e3; i < 1e4; i++) {
		AD9850_setfreq(i * 1e3);
	}
}
