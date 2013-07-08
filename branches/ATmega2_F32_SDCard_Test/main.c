/*
 * main.c
 *
 *  Created on: Nov 27, 2012
 *      Author: Vasile Guta Ciucur
 *      Site  : https://sites.google.com/site/funlw65/
 * *****************************************************************************
 * Description:
 *  ATmega-FAT32-Test - It write on the SD-Card a string with the size set
 *                      by the user. The data inside string is programmer's business.
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
//#define ENABLE_CONVERSION   // useful for Serial, LCD and 7SEG Display
//#define ENABLE_SPI          // hardware SPI (master)
//#define ENABLE_SD_CARD      // raw SD Card operations; require SPI
//#define ENABLE_FAT32        // require PCF8583, SPI and SD_CARD

//Scroll down the header file (don't cross the user zone) and make the required
// settings for the CS (chip  select) pin, according to your schematics. IS
// inside "#ifdef ENABLE_SD_CARD" conditionals ...

#ifndef F_CPU
#define F_CPU 16000000U // required by Atmel Studio 6
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <atmegaclib2.h>

//#define KEY_PRESSED   (!(PINC & 0x80))

uint8_t fileName[11] = "DATALOGSTXT"; //exactly 11 chars long, no more, no less!
// Exactly 8 chars for name, and 3 chars for extension
// The name and/or extension must be completed with spaces if the name is less
// than 8 chars and extension less than 3 chars.

uint8_t temp_buffer[] = "This is a record and a number follows 0\r\n";

//function to blink LED in case of any error
void blinkErrorLED(void) {
	while (1) { //blink red LED continuously, if error
		onboard_led_toggle();
		_delay_ms(100);
	}
}

//*************************** MAIN *******************************//
int main(void) {
	uint8_t error, i, mySD_type;

	_delay_ms(100); //delay for VCC stabilization

	serial_init();
	SPI_master_init();
	sei();

	onboard_led_enable(); // onboard LED used as visual indicator of an error.
	                      // Is useless on standard Arduino board because it
	                      // conflicts with SPI (BAD design - one of the things
	                      // which must be corrected when Arduino team will have
	                      // enough courage to redesign the Arduino board, even
	                      // if that means breaking the shield compatibility)
	onboard_led_off(); //keep user LED off for now

	//initialize the dataString buffer
	for (i = 0; i < MAX_STRING_SIZE; i++) {
		//
		dataString[i] = 0;
	}

	serial_puts_f("\r\nSD Card Test Just Started");

	// reset the sd_card type variable

	for (i = 0; i < 10; i++) {
		error = SD_init();
		if (!error)
			break;
	}

	if (error) {
		if (error == 1)
			serial_puts_f("\r\nSD card not detected..");
		if (error == 2)
			serial_puts_f("\r\nCard Initialization failed..");

		blinkErrorLED();
	}

	mySD_type = SD_card_type();
	switch (mySD_type) {
	case 1:
		serial_puts_f("\r\nStandard Capacity Card (Ver.1.x) Detected!");
		break;
	case 2:
		serial_puts_f("\r\nHigh Capacity Card Detected!");
		break;
	case 3:
		serial_puts_f("\r\nStandard Capacity Card (Ver.2.x) Detected!");
		break;
	default:
		serial_puts_f("\r\nUnknown SD Card Detected!");
		break;
	}

	error = F32_getBootSectorData(); //read boot sector and keep necessary data in global variables
	if (error) {
		serial_puts_f("\r\nFAT32 not found!");
		//FAT32 incompatible drive
		blinkErrorLED();
	}

	SPI_HIGH_SPEED; //SCK - 4 MHz
	_delay_ms(1); //some delay for settling new spi speed

	// ok, let's write something on the card
	onboard_led_on(); //turn on red LED to indicate that recording has started

	//From here onwards, add data by putting chars in dataString then write the string on SD-Card
	//dataString is declared in atmegaclib.h
	//make sure dataString doesn't exceed its MAX_STRING_SIZE, defined in FAT32.h
	//Also, end the data string with '\r' & '\n' characters to maintain CSV format

	F32_findFiles(0, 0);

	for (i = 0; i < 41; i++)
		dataString[i] = temp_buffer[i];

	error = F32_writeFile(fileName, (uint8_t *) dataString);
	if (error)
		blinkErrorLED();

	dataString[38] = '1';

	error = F32_writeFile(fileName, (uint8_t *) dataString);
	if (error)
		blinkErrorLED();

	dataString[38] = '2';

	error = F32_writeFile(fileName, (uint8_t *) dataString);
	if (error)
		blinkErrorLED();

	dataString[38] = '3';

	error = F32_writeFile(fileName, (uint8_t *) dataString);
	if (error)
		blinkErrorLED();

	dataString[38] = '4';

	error = F32_writeFile(fileName, (uint8_t *) dataString);
	if (error)
		blinkErrorLED();

	dataString[38] = '5';

	error = F32_writeFile(fileName, (uint8_t *) dataString);
	if (error)
		blinkErrorLED();

	dataString[38] = '6';

	error = F32_writeFile(fileName, (uint8_t *) dataString);
	if (error)
		blinkErrorLED();

	dataString[38] = '7';

	error = F32_writeFile(fileName, (uint8_t *) dataString);
	if (error)
		blinkErrorLED();

	dataString[38] = '8';

	error = F32_writeFile(fileName, (uint8_t *) dataString);
	if (error)
		blinkErrorLED();

	onboard_led_off(); //recording stopped

	return 0;

} // end of main

