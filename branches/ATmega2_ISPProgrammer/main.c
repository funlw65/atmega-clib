// ArduinoISP version 04m3
// Copyright (c) 2008-2011 Randall Bohn
// Ported to ATmegaCLib2 by Vasile Guta Ciucur, 2013
// If you require a license, see
//     http://www.opensource.org/licenses/bsd-license.php
//
// This sketch turns the Arduino into a AVRISP
// using the following arduino pins:
//
// pin name:    not-mega:         mega(1280 and 2560)
// slave reset: 10:               53
// MOSI:        11:               51
// MISO:        12:               50
// SCK:         13:               52
//
// Put an LED (with resistor) on the following pins:
// 9: Heartbeat   - shows the programmer is running
// 8: Error       - Lights up if something goes wrong (use red if that makes sense)
// 7: Programming - In communication with the slave
//
// 23 July 2011 Randall Bohn
// -Address Arduino issue 509 :: Portability of ArduinoISP
// http://code.google.com/p/arduino/issues/detail?id=509
//
// October 2010 by Randall Bohn
// - Write to EEPROM > 256 bytes
// - Better use of LEDs:
// -- Flash LED_PMODE on each flash commit
// -- Flash LED_PMODE while writing EEPROM (both give visual feedback of writing progress)
// - Light LED_ERR whenever we hit a STK_NOSYNC. Turn it off when back in sync.
// - Use pins_arduino.h (should also work on Arduino Mega)
//
// October 2009 by David A. Mellis
// - Added support for the read signature command
//
// February 2009 by Randall Bohn
// - Added support for writing to EEPROM (what took so long?)
// Windows users should consider WinAVR's avrdude instead of the
// avrdude included with Arduino software.
//
// January 2008 by Randall Bohn
// - Thanks to Amplificar for helping me with the STK500 protocol
// - The AVRISP/STK500 (mk I) protocol is used in the arduino bootloader
// - The SPI functions herein were developed for the AVR910_ARD programmer
// - More information at http://code.google.com/p/mega-isp

/*
 * main.c
 *
 *  Created on: Jul 13, 2013
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

//Enable the following definitions on "atmegaclib2.h" header:
/*
#define UART_BAUD_RATE			19200 // default is 57600
#define UART_BAUD_SELECT		(F_CPU / (UART_BAUD_RATE * 16L) - 1)
#define ENABLE_SERIAL // Interrupt based, require CONVERSION, conflicts with SERIAL_POLL
#define ENABLE_PWM         // motor or led control (conflicts with pwmservo)
#define ENABLE_CONVERSION    // useful for Serial, LCD and 7SEG Display
#define ENABLE_ISPPROG     // Use Arduino as ISP Programmer - require SPI, conflict SD_Card
#define ENABLE_SPI         // hardware SPI (master)
*/

//scroll down the header file to "#ifdef ENABLE_ISPPROG" line and
// change the definitions for LED_ERR and LED_PMODE according to
// your intended hardware but don't touch the LED_HB, as it needs
// to be on the channel 1 of pwm (as defined by Arduino library)
// - it is already configured for the selected microcontroller.

// The program use all 4 SPI pins (including SS), already configured for
// the selected microcontroller. You must connect the SS pin of the
// programmer to the reset pin of the target microcontroller.

// HAVE FUN!


#ifndef F_CPU
#define F_CPU 16000000U // required by Atmel Studio 6
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <atmegaclib2.h>

#define PROG_FLICKER TRUE

#define HWVER 2
#define SWMAJ 1
#define SWMIN 18

// STK Definitions
#define STK_OK      0x10
#define STK_FAILED  0x11
#define STK_UNKNOWN 0x12
#define STK_INSYNC  0x14
#define STK_NOSYNC  0x15
#define CRC_EOP     0x20 //OK, it is a space...
void pulse(int port, int pin, int times);

int error = 0;
int pmode = 0;
// address for reading and writing, set by 'U' command
int here;
uint8_t buff[256]; // global block storage

#define beget16(addr) (*addr * 256 + *(addr+1) )
typedef struct param {
	uint8_t devicecode;
	uint8_t revision;
	uint8_t progtype;
	uint8_t parmode;
	uint8_t polling;
	uint8_t selftimed;
	uint8_t lockbytes;
	uint8_t fusebytes;
	int flashpoll;
	int eeprompoll;
	int pagesize;
	int eepromsize;
	int flashsize;
} parameter;

parameter param;

// this provides a heartbeat on pin 9, so you can tell the software is running.
uint8_t hbval = 128;
int8_t hbdelta = 8;
void heartbeat() {
	if (hbval > 192)
		hbdelta = -hbdelta;
	if (hbval < 32)
		hbdelta = -hbdelta;
	hbval += hbdelta;
	//analogWrite(LED_HB, hbval);
	pwm_set(1, hbval); // HeartBeat LED is on PWM CH1
	_delay_ms(20);
}

uint8_t getch() {
	while (serial_available() == FALSE)
		;
	return serial_getchar();
}

void fill(int n) {
	for (int x = 0; x < n; x++) {
		buff[x] = getch();
	}
}

#define PTIME 30
void pulse(int port, int pin, int times) {
	do {
		//digitalWrite(pin, HIGH);
		sbi(port, pin);
		_delay_ms(PTIME);
		//digitalWrite(pin, LOW);
		cbi(port, pin);
		_delay_ms(PTIME);
	} while (times--);
}

void prog_lamp(int state) {
	if (PROG_FLICKER) {
		//digitalWrite(LED_PMODE, state);
		if (state)
			sbi(LED_PMODE_PORT, LED_PMODE);
		else
			cbi(LED_PMODE_PORT, LED_PMODE);
	}
}

void empty_reply() {
	if (CRC_EOP == getch()) {
		serial_putchar(STK_INSYNC);
		serial_putchar(STK_OK);
	} else {
		error++;
		serial_putchar(STK_NOSYNC);
	}
}

void breply(uint8_t b) {
	if (CRC_EOP == getch()) {
		serial_putchar(STK_INSYNC);
		serial_putchar(b);
		serial_putchar(STK_OK);
	} else {
		error++;
		serial_putchar(STK_NOSYNC);
	}
}

void get_version(uint8_t c) {
	switch (c) {
	case 0x80:
		breply(HWVER);
		break;
	case 0x81:
		breply(SWMAJ);
		break;
	case 0x82:
		breply(SWMIN);
		break;
	case 0x93:
		breply('S'); // serial programmer
		break;
	default:
		breply(0);
		break;
	}
}

void set_parameters() {
	// call this after reading parameter packet into buff[]
	param.devicecode = buff[0];
	param.revision   = buff[1];
	param.progtype   = buff[2];
	param.parmode    = buff[3];
	param.polling    = buff[4];
	param.selftimed  = buff[5];
	param.lockbytes  = buff[6];
	param.fusebytes  = buff[7];
	param.flashpoll  = buff[8];
	// ignore buff[9] (= buff[8])
	// following are 16 bits (big endian)
	param.eeprompoll = beget16(&buff[10]);
	param.pagesize   = beget16(&buff[12]);
	param.eepromsize = beget16(&buff[14]);

	// 32 bits flash size (big endian)
	param.flashsize = buff[16] * 0x01000000 + buff[17] * 0x00010000
			+ buff[18] * 0x00000100 + buff[19];

}

void start_pmode() {
	SPCR = 0x13; //don't touch :P - I can explain if needed :))
	SPI_master_init();
	_delay_ms(50);
	cbi(SS_PORT, SS);
	// select slave
	SPI_master_transaction(0xAC, 0x53, 0x00, 0x00);
	pmode = 1;
}

void end_pmode() {
	SPI_master_stop();
	// deselect slave
	sbi(SS_PORT, SS);
	// change direction of SPI pins to inputs.
	cbi(SCK_DDR, SCK);
	cbi(MOSI_DDR, MOSI);
	cbi(SS_DDR, SS);
	pmode = 0;
}

void universal() {
	//int w;
	uint8_t ch;

	fill(4);
	ch = SPI_master_transaction(buff[0], buff[1], buff[2], buff[3]);
	breply(ch);
}

void flash(uint8_t hilo, int addr, uint8_t data) {
	SPI_master_transaction(0x40 + 8 * hilo, addr >> 8 & 0xFF, addr & 0xFF,
			data);
}
void commit(int addr) {
	if (PROG_FLICKER)
		prog_lamp(LOW);
	SPI_master_transaction(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0);
	if (PROG_FLICKER) {
		_delay_ms(PTIME);
		prog_lamp(HIGH);
	}
}

//#define _current_page(x) (here & 0xFFFFE0)
int current_page(int addr) {
	if (param.pagesize == 32)
		return here & 0xFFFFFFF0;
	if (param.pagesize == 64)
		return here & 0xFFFFFFE0;
	if (param.pagesize == 128)
		return here & 0xFFFFFFC0;
	if (param.pagesize == 256)
		return here & 0xFFFFFF80;
	return here;
}

uint8_t write_flash_pages(int length) {
	int x = 0;
	int page = current_page(here);
	while (x < length) {
		if (page != current_page(here)) {
			commit(page);
			page = current_page(here);
		}
		flash(LOW, here, buff[x++]);
		flash(HIGH, here, buff[x++]);
		here++;
	}
	commit(page);
	return STK_OK;
}

void write_flash(int length) {
	fill(length);
	if (CRC_EOP == getch()) {
		serial_putchar(STK_INSYNC);
		serial_putchar(write_flash_pages(length));
	} else {
		error++;
		serial_putchar(STK_NOSYNC);
	}
}

#define EECHUNK (32)
// write (length) bytes, (start) is a byte address
uint8_t write_eeprom_chunk(int start, int length) {
	// this writes byte-by-byte,
	// page writing may be faster (4 bytes at a time)
	fill(length);
	prog_lamp(LOW);
	for (int x = 0; x < length; x++) {
		int addr = start + x;
		SPI_master_transaction(0xC0, (addr >> 8) & 0xFF, addr & 0xFF, buff[x]);
		_delay_ms(45);
	}
	prog_lamp(HIGH);
	return STK_OK;
}

uint8_t write_eeprom(int length) {
	// here is a word address, get the byte address
	int start = here * 2;
	int remaining = length;
	if (length > param.eepromsize) {
		error++;
		return STK_FAILED;
	}
	while (remaining > EECHUNK) {
		write_eeprom_chunk(start, EECHUNK);
		start += EECHUNK;
		remaining -= EECHUNK;
	}
	write_eeprom_chunk(start, remaining);
	return STK_OK;
}

void program_page() {
	uint8_t result = STK_FAILED;
	int length = 256 * getch();
	length += getch();
	uint8_t memtype = getch();
	// flash memory @here, (length) bytes
	if (memtype == 'F') {
		write_flash(length);
		return;
	}
	if (memtype == 'E') {
		result = (uint8_t) write_eeprom(length);
		if (CRC_EOP == getch()) {
			serial_putchar(STK_INSYNC);
			serial_putchar(result);
		} else {
			error++;
			serial_putchar(STK_NOSYNC);
		}
		return;
	}
	serial_putchar(STK_FAILED);
	return;
}

uint8_t flash_read(uint8_t hilo, int addr) {
	return SPI_master_transaction(0x20 + hilo * 8, (addr >> 8) & 0xFF,
			addr & 0xFF, 0);
}

char flash_read_page(int length) {
	for (int x = 0; x < length; x += 2) {
		uint8_t low = flash_read(LOW, here);
		serial_putchar(low);
		uint8_t high = flash_read(HIGH, here);
		serial_putchar(high);
		here++;
	}
	return STK_OK;
}

char eeprom_read_page(int length) {
	// here again we have a word address
	int start = here * 2;
	for (int x = 0; x < length; x++) {
		int addr = start + x;
		uint8_t ee = SPI_master_transaction(0xA0, (addr >> 8) & 0xFF,
				addr & 0xFF, 0xFF);
		serial_putchar(ee);
	}
	return STK_OK;
}

void read_page() {
	uint8_t result = STK_FAILED;
	int length = 256 * getch();
	length += getch();
	uint8_t memtype = getch();
	if (CRC_EOP != getch()) {
		error++;
		serial_putchar(STK_NOSYNC);
		return;
	}
	serial_putchar(STK_INSYNC);
	if (memtype == 'F')
		result = flash_read_page(length);
	if (memtype == 'E')
		result = eeprom_read_page(length);
	serial_putchar(result);
	return;
}

void read_signature() {
	if (CRC_EOP != getch()) {
		error++;
		serial_putchar(STK_NOSYNC);
		return;
	}
	serial_putchar(STK_INSYNC);
	uint8_t high = SPI_master_transaction(0x30, 0x00, 0x00, 0x00);
	serial_putchar(high);
	uint8_t middle = SPI_master_transaction(0x30, 0x00, 0x01, 0x00);
	serial_putchar(middle);
	uint8_t low = SPI_master_transaction(0x30, 0x00, 0x02, 0x00);
	serial_putchar(low);
	serial_putchar(STK_OK);
}
////////////////////////////////////
////////////////////////////////////
//            M A I N
////////////////////////////////////
////////////////////////////////////

int main(void) {
	uint8_t data, low, high;
	sei();
	serial_init();
	SPI_master_setDataMode(SPI_MODE0);
	SPI_master_setBitOrder(MSBFIRST);
	// Clock Div can be 2,4,8,16,32,64, or 128
	SPI_master_setClockDivider(SPI_CLOCK_DIV128);
	pwm_init(1); // required by HeartBeat LED
	//pinMode(LED_PMODE, OUTPUT);
	sbi(LED_PMODE_DDR, LED_PMODE);
	pulse(LED_PMODE_PORT, LED_PMODE, 2);
	//pinMode(LED_ERR, OUTPUT);
	sbi(LED_ERR_DDR, LED_ERR);
	pulse(LED_ERR_PORT, LED_ERR, 2);
	//pinMode(LED_HB, OUTPUT);
	sbi(LED_HB_DDR, LED_HB);
	pulse(LED_HB_PORT, LED_HB, 2);
	for (;;) {
		// is pmode active?
		if (pmode)
			//digitalWrite(LED_PMODE, HIGH);
			sbi(LED_PMODE_PORT, LED_PMODE);
		else
			//digitalWrite(LED_PMODE, LOW);
			cbi(LED_PMODE_PORT, LED_PMODE);
		// is there an error?
		if (error)
			//digitalWrite(LED_ERR, HIGH);
			sbi(LED_ERR_PORT, LED_ERR);
		else
			//digitalWrite(LED_ERR, LOW);
			cbi(LED_ERR_PORT, LED_ERR);

		// beat the heart of the ISP Programmer :)
		heartbeat();
		if (serial_available() == TRUE) {
			uint8_t ch = getch();
			switch (ch) {
			case '0': // signon
				error = 0;
				empty_reply();
				break;
			case '1':
				if (getch() == CRC_EOP) {
					serial_putchar(STK_INSYNC);
					serial_puts_f("AVR ISP");
					serial_putchar(STK_OK);
				}
				break;
			case 'A':
				get_version(getch());
				break;
			case 'B':
				fill(20);
				set_parameters();
				empty_reply();
				break;
			case 'E': // extended parameters - ignore for now
				fill(5);
				empty_reply();
				break;

			case 'P':
				start_pmode();
				empty_reply();
				break;
			case 'U': // set address (word)
				here = getch();
				here += 256 * getch();
				empty_reply();
				break;

			case 0x60: //STK_PROG_FLASH
				low = getch();
				high = getch();
				empty_reply();
				break;
			case 0x61: //STK_PROG_DATA
				data = getch();
				empty_reply();
				break;

			case 0x64: //STK_PROG_PAGE
				program_page();
				break;

			case 0x74: //STK_READ_PAGE 't'
				read_page();
				break;

			case 'V': //0x56
				universal();
				break;
			case 'Q': //0x51
				error = 0;
				end_pmode();
				empty_reply();
				break;

			case 0x75: //STK_READ_SIGN 'u'
				read_signature();
				break;

				// expecting a command, not CRC_EOP
				// this is how we can get back in sync
			case CRC_EOP:
				error++;
				serial_putchar(STK_NOSYNC);
				break;

				// anything else we will return STK_UNKNOWN
			default:
				error++;
				if (CRC_EOP == getch())
					serial_putchar(STK_UNKNOWN);
				else
					serial_putchar(STK_NOSYNC);
				break;
			}
		}
	}
	return 0;
}

