/*
 * main.c
 *
 *  Created on: Aug 25, 2012
 *      Author: Vasile Guta Ciucur
 *
 *  Program: Displays the entire character set of 7seg library on a single digit.
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

//Enable the following definitions on "atmegaclib2.h" header:

//#define ENABLE_7SEG // starting from one digit, up to eight digits.

//Then, define the pins and the number of digits for the 7seg display in the
// user zone, on the same header.

#ifndef F_CPU
	#define F_CPU 16000000U //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include <util/delay.h>
#include <atmegaclib2.h>

uint8_t i;

void main(void) __attribute__((noreturn)); //this allows me to have a void main() function
void main (void)
{
	// initializes the buffer, (involved) pins direction
	seg_init();
	// select the digit.
	// the second parameter means that it needs logic 0 for activation, on my EVB4.3 board
	seg_select_digit(SELECT_DIGIT_ONE, 0);
	// --
	while(1){
		for(i = 0; i < 37; i++)
			seg_nibble(seg_digit_from_array(i));
		_delay_ms(500);
	}
}
// test svn update (Sep 19, 2012)
