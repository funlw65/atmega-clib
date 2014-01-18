/*
 * main.c
 *
 *  Created on: Aug 16, 2013
 *      Author: Vasile Guta Ciucur
 *
 *  Description: The program resembles the example which come with
 *               the PCF8583 library for Arduino. The code size is dramatically
 *               reduced: is a difference between the two, of ~5K bytes!!!
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

// Activate the following definitions on "atmegaclib2.h" header, not here:
// Uncomment only one of these:
//#define ENABLE_I2C_SOFTWARE // I2C software if you don't use TWI
//#define ENABLE_TWI          // TWI if you don't use I2C software

// And define the required pins (I2C software) in the user zone
// of the same header. Or, just use TWI - this way, other pin definitions are not required.

// IMPORTANT! The date and hour are set via serial and you must send a
// string to the microcontroller with the following format:
// 130818192300;
// Where : 13 is 2013 year,
//         08 is the August month
//         18 is the day,
//         19 is the hour,
//         23 are the minutes,
//         00 are the seconds
// and the string ends with ; character, telling the microcontroller that
// we have done sending the configuration string.

#ifndef F_CPU
#define F_CPU 16000000U // required by Atmel Studio 6
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "atmegaclib2.h"
#include <conversion.c>
#include <serial.c>
#include <serial_common.c>
#ifdef ENABLE_TWI
#include <twi.c>
#else
#include <i2c.c>
#endif
#include <pcf8583.c>

uint8_t month, day, hour, min, sec, hsec, buff_index;
uint16_t year;
int8_t date_buff[14];

//this allows me to have a "void main()" function
void main(void) __attribute__((noreturn));
void main(void) {
	//
	serial_init(); // interrupt based USART communication
#ifdef ENABLE_TWI
	TWI_init();
#endif
	//
	PCF8583_init();
	sei();
	// activate interrupts

	buff_index = 0;
	serial_puts_f("\r\nBooting...\0");
	serial_puts_f("done.\r\n\0");

	while (1) {
		//
		if (serial_available() == TRUE) {
			date_buff[buff_index] = serial_getchar();

			if (date_buff[buff_index] == ';') {
				if (buff_index == 12) {
					// Use of (byte) type casting and ASCII math to achieve the result.
					year = (uint8_t) ((date_buff[0] - 48) * 10
							+ (date_buff[1] - 48)) + 2000;
					month = (uint8_t) ((date_buff[2] - 48) * 10
							+ (date_buff[3] - 48));
					day = (uint8_t) ((date_buff[4] - 48) * 10
							+ (date_buff[5] - 48));
					hour = (uint8_t) ((date_buff[6] - 48) * 10
							+ (date_buff[7] - 48));
					min = (uint8_t) ((date_buff[8] - 48) * 10
							+ (date_buff[9] - 48));
					sec = (uint8_t) ((date_buff[10] - 48) * 10
							+ (date_buff[11] - 48));
					serial_puts_f("... setting date ...\0");
					PCF8583_set_time(hour, min, sec, 0);
					PCF8583_set_date(day, month, year);
					serial_puts_f(" done!\r\n\0");
				} else
					serial_puts_f(
							"Configuration error: wrong string length! Try again.\r\n\0");
				// Now, just reset indexes and prepare for a new config.string if needed.
				serial_flush();
				date_buff[buff_index] = ' ';
				buff_index = 0;
			} else
				buff_index++; // if not termination char, keep incrementing
		}
		// read the date and time.
		PCF8583_get_time(&hour, &min, &sec, &hsec);
		PCF8583_get_date(&day, &month, &year);

		// Formatting data without "printf" function!
		// Romanian configuration for displaying the date:
		//   day first, then month, then year.
		// Rearrange to suit your regional settings.
		serial_putU08(day);
		serial_putchar('/');
		serial_putU08(month);
		serial_putchar('/');
		serial_putint(year);
		serial_putchar(' ');
		serial_putU08(hour);
		serial_putchar(':');
		serial_putU08(min);
		serial_putchar(':');
		serial_putU08(sec);
		serial_putchar('\r');
		serial_putchar('\n');

		_delay_ms(990);
	}
}
