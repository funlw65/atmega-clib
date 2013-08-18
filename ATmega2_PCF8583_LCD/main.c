/*
 * main.c
 *
 *  Created on: Aug 18, 2013
 *      Author: Vasile Guta Ciucur
 *  Description:
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
//#define UART_BAUD_RATE      57600 // default is 57600
//#define UART_BAUD_SELECT    (F_CPU / (UART_BAUD_RATE * 16L) - 1)
//#define ENABLE_SERIAL       // require CONVERSION, conflicts with SERIAL_POLL
//#define ENABLE_I2C_SOFTWARE // I2C software if you don't use TWI
//#define ENABLE_TWI          // TWI if you don't use I2C software
//#define ENABLE_CONVERSION   // useful for Serial, LCD and 7SEG Display
//#define ENABLE_PCF8583      // require CONVERSION and I2C
//#define ENABLE_LCD          // require CONVERSION

// Bellow, but still in the user zone,
// go define the required pins (I2C software) in the user zone
// of the same header. Or, just activate the USE_TWI definition for PCF8583 to
// be able to use TWI - this way, other pin definitions are not required.

// Then chose what pins you will use for LCD.

#ifndef F_CPU
#define F_CPU 16000000U // required by Atmel Studio 6
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <atmegaclib2.h>

uint8_t month, day, hour, min, sec, hsec, buff_index;
uint16_t year;
int8_t date_buff[14];

//this allows me to have a "void main()" function
void main(void) __attribute__((noreturn));
void main(void) {
	//
	serial_init(); // interrupt based USART communication
	//Enable the following (one) line if you use TWI instead of I2C_Software
	TWI_init();
	//
	PCF8583_init();
	// activate interrupts
	sei();
	lcd_init();
	lcd_puts_f("Data:");
	lcd_set_cursor(0, 1); // go to column 0, row 1 (the second one)
	lcd_puts_f("Ora: ");

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

		// Romanian configuration for displaying the date:
		//   day first, then month, then year.
		// Rearrange to suit your regional settings.
		lcd_set_cursor(5, 0);
		lcd_puts_f("          "); // delete the zone where date is printed
		lcd_set_cursor(5, 0); // back to initial position and start printing
		lcd_putU08(day);
		lcd_putchar('-');
		lcd_putU08(month);
		lcd_putchar('-');
		lcd_putint(year);
		lcd_set_cursor(5, 1);
		lcd_puts_f("        "); // delete the zone where hour is printed
		lcd_set_cursor(5, 1); // back to initial position and start printing
		lcd_putU08(hour);
		lcd_putchar(':');
		lcd_putU08(min);
		lcd_putchar(':');
		lcd_putU08(sec);

		_delay_ms(990);
	}
}
