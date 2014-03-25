/*
 * Copyright:      Fabian Maximilian Thiele  mailto:me@apetech.de
 * Author:         Fabian Maximilian Thiele
 * Remarks:        this Copyright must be included
 * known Problems: none
 * Version:        1.1
 * Description:    KS0108 Library Demo Program
 *
 */

/*
 * main.c
 *
 *  Created on: Aug 7, 2013
 *      Author: Vasile Guta Ciucur
 *  Description: Just the Fabian example, adapted (WIP) to ATmegaCLib2
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

// don't forget to set the ports on the header in user zone if you use diff. hw.

#ifndef F_CPU
#define F_CPU 16000000U // required by Atmel Studio 6
#endif

#include <avr/io.h>
//#include <avr/interrupt.h>
//#include <util/delay.h>
#include <avr/pgmspace.h>
#include <arial_bold_14.h>
#include <corsiva_12.h>
#include "atmegaclib2.h"
#include <atmegaclib2_files.c>

void main(void) __attribute__((noreturn));
void main(void) {
	// Wait a little while the display starts up
	for(volatile uint16_t i=0; i<15000; i++);

	// Initialize the LCD
	GLCD_Init(0);

	// Select a font
	GLCD_SelectFont(Arial_Bold_14, GLCD_ReadFontData, GLCD_BLACK);
	// Set a position
	GLCD_GotoXY(15,10);
	// Print some text
	GLCD_Puts_P(PSTR("KS0108-Treiber"));
	// a nice little round rect
	GLCD_DrawRoundRect(5, 5, 117, 20, 8, GLCD_BLACK);

	// Once again :)
	// Select a font
	GLCD_SelectFont(Corsiva_12, GLCD_ReadFontData, GLCD_BLACK);
	// Set a position
	GLCD_GotoXY(5,30);
	// Print some text
	GLCD_Puts_P(PSTR("http://www.apetech.de\nmailto:me@apetech.de"));

	while(1);
}
