/* *****************************************************************************
 *  BSD License
 *  ATmega-CLib - a BSD library for using GNU toolchain with Arduino, EvB4.3,...
 *  Portions Copyright:
 *  - (c) 1998 Wouter van Ooijen, http://www.voti.nl/winkel/index.html
 *  - (c) 2004 Robert Krysztof, website is gone
 *  - (c) 2009 Michael Spiceland, https://code.google.com/p/libarduino/
 *  - (c) 2009 Joep Suijs, http://www.blogger.com/profile/06821529393453332522
 *  - (c) 2009 Vasile Surducan, http://vsurducan.blogspot.com/
 *  - (c) 2010 Bert van Dam, http://members.home.nl/b.vandam/lonely/index.html
 *  - (c) 2010 Paul Stoffregen, http://www.pjrc.com/teensy/td_libs_OneWire.html
 *  - (c) 2010 Chennai Dharmani, http://www.dharmanitech.com
 *  - (c) 2011 Joe Pardue, http://code.google.com/p/avrtoolbox/
 *  - (c) 2012 Vasile Guta Ciucur, https://sites.google.com/site/funlw65/
 *
 *******************************************************************************
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
 *  - Neither the name of Joe Pardue nor the names of
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
#ifndef F_CPU
#define F_CPU 16000000UL //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdarg.h>
#include <stdlib.h>
#include <atmegaclib2.h>
#ifdef ENABLE_IR
#include "irkeys.h"
#endif
#include <avr/pgmspace.h>
#include <avr/eeprom.h>


/* stuff used in all modes */
inline void onboard_led_enable(void) {
#if defined(__AVR_ATmega16__)      || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
	sbi(DDRB, 0);
	// DEBUG LED, used also for bootloader
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
	sbi(DDRB, 7);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)   // Arduino 28 pins
	sbi(DDRB, 5);
#endif
}

inline void onboard_led_on(void) {
#if defined(__AVR_ATmega16__)      || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
	sbi(PORTB, 0);
	// DEBUG LED, used also for bootloader
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
	sbi(PORTB, 7);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
	sbi(PORTB, 5);
#endif
}

inline void onboard_led_off(void) {
#if defined(__AVR_ATmega16__)      || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
	cbi(PORTB, 0);
	// DEBUG LED, used also for bootloader
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
	cbi(PORTB, 7);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
	cbi(PORTB, 5);
#endif
}

inline void onboard_led_toggle(void){
#if defined(__AVR_ATmega16__)      || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
	tbi(PORTB, 0);
	// DEBUG LED, used also for bootloader
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
	tbi(PORTB, 7);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
	tbi(PORTB, 5);
#endif
}

#ifdef ENABLE_NB_DELAYS
#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )
// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile uint32_t timer0_overflow_count = 0;
//volatile uint32_t timer0_millis = 0;
static uint8_t timer0_fract = 0;

#if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
ISR(TIM0_OVF_vect)
#else
ISR(TIMER0_OVF_vect)
#endif
{
	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	//uint32_t m = timer0_millis;
	uint8_t f = timer0_fract, i = 1, j;

	//m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		//m += 1;
		i = 2;
	}

	timer0_fract = f;
	//timer0_millis = m;
	timer0_overflow_count++;
	for (j=0; j<DELAY_SLOTS;j++){
		if(isr_countdowns[j] > 0){
			//
			isr_countdowns[j] = isr_countdowns[j] - i;
		}
	}
}

void timer0_isr_init(void) {
#if defined(TCCR0A) && defined(WGM01)
	sbi(TCCR0A, WGM01);
	sbi(TCCR0A, WGM00);
#endif
	// set timer 0 prescale factor to 64
#if defined(__AVR_ATmega128__)
	// CPU specific: different values for the ATmega128
	sbi(TCCR0, CS02);
#elif defined(TCCR0) && defined(CS01) && defined(CS00)
	// this combination is for the standard atmega8
	sbi(TCCR0, CS01);
	sbi(TCCR0, CS00);
#elif defined(TCCR0B) && defined(CS01) && defined(CS00)
	// this combination is for the standard 168/328/1280/2560
	sbi(TCCR0B, CS01);
	sbi(TCCR0B, CS00);
#elif defined(TCCR0A) && defined(CS01) && defined(CS00)
	// this combination is for the __AVR_ATmega645__ series
	sbi(TCCR0A, CS01);
	sbi(TCCR0A, CS00);
#else
#error Timer 0 prescale factor 64 not set correctly
#endif
	// enable timer 0 overflow interrupt
#if defined(TIMSK) && defined(TOIE0)
	sbi(TIMSK, TOIE0);
#elif defined(TIMSK0) && defined(TOIE0)
	sbi(TIMSK0, TOIE0);
#else
#error	Timer 0 overflow interrupt not set correctly
#endif
}

// check if the time in a specific slot expired
uint8_t check_delay(uint8_t slot)
{
	if (slot >= DELAY_SLOTS) return TRUE; //protection against user input error
	uint8_t oldSREG = SREG;
	cli();
	if (isr_countdowns[slot]<=0){
		SREG = oldSREG;
		return TRUE;
	}
	SREG = oldSREG;
	return FALSE;
}

//set the duration of a delay (in milliseconds) in a specific slot
void set_delay(uint8_t slot, uint16_t ms_time){
	if(slot >= DELAY_SLOTS) return; //protection against user input error
	uint8_t oldSREG = SREG;

	cli();
	isr_countdowns[slot] = ms_time;
	SREG = oldSREG;
}
#endif //ENABLE_NB_DELAYS

#ifdef ENABLE_CONVERSION
uint8_t bcd2bin(uint8_t bcd) {
#ifdef OPTIMIZE_SPEED
	return (10*(bcd>>4)|(bcd&0x0f));
#else
	uint8_t Temp = bcd & 0x0F;
	while (bcd >= 0x10) {
		Temp += 10;
		bcd -= 0x10;
	}
	return Temp;
#endif
}

uint8_t bin2bcd(uint8_t bin) {
#ifdef OPTIMIZE_SPEED
	return (((bin/10)<<4)|(bin%10));
#else
	uint8_t Temp = 0;
	while (bin > 9) {
		Temp += 0x10;
		bin -= 10;
	}
	return Temp + bin;
#endif
}

uint8_t nibble2hex(uint8_t val) {
	uint8_t s;
	s = '0' + (val & 0xf);
	if (s > '9')
		s += 'A' - '9' - 1;
	return s;
}

void byte2dec(uint8_t val, uint8_t *s) {
	if (val > 99) {
		s[2] = '0' + (val % 10);
		val /= 10;
		s[1] = '0' + (val % 10);
		val /= 10;
		s[0] = '0' + val;
		//
		s[3] = 0;
	} else if (val > 9) {
		s[1] = '0' + (val % 10);
		val /= 10;
		s[0] = '0' + val;
		s[2] = 0;
		//
		//s[3] = 0;
	} else {
		s[0] = '0' + val;
		//s[2] = 0;
		s[1] = 0;
		//
		///s[3] = 0;
	}
}

void double2hex(uint32_t val, uint8_t *s) {
	s[0] = nibble2hex(val >> 28);
	s[1] = nibble2hex(val >> 24);
	s[2] = nibble2hex(val >> 20);
	s[3] = nibble2hex(val >> 16);
	s[4] = nibble2hex(val >> 12);
	s[5] = nibble2hex(val >> 8);
	s[6] = nibble2hex(val >> 4);
	s[7] = nibble2hex(val);
	s[8] = 0;
}

void word2hex(uint16_t val, uint8_t *s) {
	s[0] = nibble2hex(val >> 12);
	s[1] = nibble2hex(val >> 8);
	s[2] = nibble2hex(val >> 4);
	s[3] = nibble2hex(val);
	s[4] = 0;
}

void byte2hex(uint8_t val, uint8_t *s) {
	s[0] = nibble2hex(val >> 4);
	s[1] = nibble2hex(val);
	s[2] = 0;
}
#endif // end conversion

//Interrupt based.
#ifdef ENABLE_SERIAL

//
uint8_t uart_buffer[UART_BUFFER_SIZE];
volatile uint8_t uart_readptr;
volatile uint8_t uart_writeptr;

ISR(UART0_ISR_VECT) // see the header file...
{
	uart_buffer[uart_writeptr] = UART0_DATA;
	uart_writeptr = (uart_writeptr + 1) % UART_BUFFER_SIZE;
}

void serial_init(void) {
	uart_writeptr = 0;
	uart_readptr = 0;

	// set default baud rate
	UBRR0H = UART_BAUD_SELECT >> 8;
	UBRR0L = UART_BAUD_SELECT;

	// enable receive, transmit and enable receive interrupts
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

	// don't forget sei()
}

uint8_t serial_getchar(void) {
	uint8_t c;

	//Non-blocking function - but serial_available() MUST be used!!!
	c = uart_buffer[uart_readptr];
	uart_readptr = (uart_readptr + 1) % UART_BUFFER_SIZE;
	return c;
}

uint8_t serial_available(void) {
	if (uart_writeptr != uart_readptr)
		return TRUE;
	else
		return FALSE;
}

void serial_flush(void) { // sort of...
	// only reinitializing the "pointers"
	uart_writeptr = 0;
	uart_readptr = 0;
}

#endif // end serial with interrupt
#ifdef ENABLE_SERIAL_POLL
/* Initialize UART */
void serial_init( void )
{
	/* Set the baud rate */
	UBRR0H = (unsigned char) (UART_BAUD_SELECT>>8);
	UBRR0L = (unsigned char) UART_BAUD_SELECT;
	/* Enable UART receiver and transmitter */
	UCSR0B = ( ( 1 << RXEN0 ) | ( 1 << TXEN0 ) );
	/* Set frame format: 8 data 2stop */
	UCSR0C = (1<<USBS0)|(1<<UCSZ01)|(1<<UCSZ00); //For devices with Extended IO
	//UCSR0C = (1<<URSEL)|(1<<USBS0)|(1<<UCSZ01)|(1<<UCSZ00);   //For devices without Extended IO
}

/* Read and write functions */
uint8_t serial_getchar( void )
{
	/* Wait for incoming data */
	while ( !(UCSR0A & (1<<RXC0)) );
	/* Return the data */
	return UDR0;
}
#endif // END SERIAL_POLL
#if defined(ENABLE_SERIAL) || defined(ENABLE_SERIAL_POLL)
void serial_putchar(uint8_t data) {
	/* Wait for empty transmit buffer */
	while (!(UCSR0A & (1 << UDRE0)))
		;
	/* Start transmission */
	UDR0 = data;
}

void serial_putstr(uint8_t * s) {
	register uint8_t c;
	while ((c = *s++))
		serial_putchar(c);
}

// Example:
//  uint8_t const pgm_str1[] PROGMEM =  "!Smiley!\0";
//  uint8_t const pgm_str2[] PROGMEM =  "!Micros!\0";
//  serial_putstr_f((uint8_t *)pgm_str1);
//  serial_putstr_f((uint8_t *)pgm_str2);
void serial_putstr_f(uint8_t *s) {
	register uint8_t c;
	while ((c = pgm_read_byte(s++)))
		serial_putchar(c);
}



// NOT working ...
//void serial_putstr_e(uint8_t *s)
//  {
//	  register uint8_t c;
//	  while ((c = eeprom_read_byte(s++)))
//		  serial_putchar(c);
//  }

void serial_putint(int value, uint8_t radix) {
	uint8_t string[18];
	itoa(value, (char *) string, radix);
	serial_putstr(string);
}

void serial_putU08(uint8_t value) {
	uint8_t s[4];
	byte2dec(value, s);
	serial_putstr(s);
}

void serial_puthexU08(uint8_t value) {
	uint8_t s[3];
	byte2hex(value, s);
	serial_putstr(s);
}

void serial_puthexU16(uint16_t value) {
	uint8_t s[5];
	word2hex(value, s);
	serial_putstr(s);
}

void serial_puthexU32(uint32_t value) {
	uint8_t s[9];
	double2hex(value, s);
	serial_putstr(s);
}

#endif

#ifdef ENABLE_SPI
//SPI initialize for SD card
//clock rate: 125Khz (fosc/64)
// OR do it for ISPProgrammer
void SPI_master_init(void) {
#ifdef ENABLE_ISPPROG
#if defined(__AVR_ATmega16__)    || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
	//pin direction
	sbi(DDRB, PB7);	// SCK
	sbi(DDRB, PB5);	// MOSI
	sbi(DDRB, PB4);	// SS, all three are OUTPUTS
	//initialization
	cbi(PORTB, PB7); // SCK  LOW
	cbi(PORTB, PB5); // MOSI LOW
	sbi(PORTB, PB4); // SS   HIGH
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560P__) // Arduino Mega1280
	sbi(DDRB, PB1);// SCK
	sbi(DDRB, PB2);// MOSI
	sbi(DDRB, PB0);// SS, all three are OUTPUTS
	//initialization
	cbi(PORTB, PB1);// SCK  LOW
	cbi(PORTB, PB2);// MOSI LOW
	sbi(PORTB, PB0);// SS   HIGH
#elif defined(__AVR_ATmega48__)    || \
      defined(__AVR_ATmega88__)      || \
      defined(__AVR_ATmega88P__)     || \
      defined(__AVR_ATmega168__)     || \
      defined(__AVR_ATmega168P__)    || \
      defined(__AVR_ATmega328P__)    // Arduino 28 pins
	sbi(DDRB, PB5);// SCK
	sbi(DDRB, PB3);// MOSI
	sbi(DDRB, PB2);// SS, all three are OUTPUTS
	//initialization
	cbi(PORTB, PB5);// SCK  LOW
	cbi(PORTB, PB3);// MOSI LOW
	sbi(PORTB, PB2);// SS   HIGH
#endif
	SPI_ISP;
#else
	//set the spi pins directions
	// the MISO pin is set already by default in master mode
#if defined(__AVR_ATmega16__)    || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
	//pin direction
	sbi(DDRB, PB7);
	// SCK
	sbi(DDRB, PB5);
	// MOSI
	sbi(DDRB, PB4);
	// SS, all three are OUTPUTS
	//initialization
	cbi(PORTB, PB7);
	// SCK  LOW
	cbi(PORTB, PB5);
	// MOSI LOW
	sbi(PORTB, PB4);
	// SS   HIGH
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560P__) // Arduino Mega1280
	sbi(DDRB, PB1);// SCK
	sbi(DDRB, PB2);// MOSI
	sbi(DDRB, PB0);// SS, all three are OUTPUTS
	//initialization
	cbi(PORTB, PB1);// SCK  LOW
	cbi(PORTB, PB2);// MOSI LOW
	sbi(PORTB, PB0);// SS   HIGH
#elif defined(__AVR_ATmega48__)    || \
      defined(__AVR_ATmega88__)      || \
      defined(__AVR_ATmega88P__)     || \
      defined(__AVR_ATmega168__)     || \
      defined(__AVR_ATmega168P__)    || \
      defined(__AVR_ATmega328P__)    // Arduino 28 pins
	sbi(DDRB, PB5);// SCK
	sbi(DDRB, PB3);// MOSI
	sbi(DDRB, PB2);// SS, all three are OUTPUTS
	//initialization
	cbi(PORTB, PB5);// SCK  LOW
	cbi(PORTB, PB3);// MOSI LOW
	sbi(PORTB, PB2);// SS   HIGH
#endif

	// I'm not happy with this, is not transparent to the user...
	// anyway, see atmegaclib.h but settings are as follows:
	// Master mode, MSB first, SCK phase low, SCK idle low, Low speed (fosc/64)
	SPI_LOW_SPEED;
#endif //ISPPROG
}

uint8_t SPI_master_transmit(uint8_t data) {
	// Start transmission
	SPDR = data;

	// Wait for transmission complete
	while (!(SPSR & (1 << SPIF)));
	data = SPDR;

	return (data);
}

uint8_t SPI_master_receive(void) {
	uint8_t data;
	// Wait for reception complete

	SPDR = 0xff;
	while (!(SPSR & (1 << SPIF)));
	data = SPDR;

	// Return data register
	return data;
}

//the following is required by ArduinoISP programmer
uint8_t SPI_master_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  uint8_t n;
  SPI_master_transmit(a);
  n=SPI_master_transmit(b);
  //if (n != a) error = -1;
  n=SPI_master_transmit(c);
  return SPI_master_transmit(d);
}

#endif // ENABLE_SPI
#ifdef ENABLE_SD_CARD


//******************************************************************
//Function	: to initialize the SD/SDHC card in SPI mode
//Arguments	: none
//return	: unsigned char; will be 0 if no error,
// 			  otherwise the response byte will be sent
//******************************************************************
uint8_t SD_init(void) {
	uint8_t i, response, SD_version;
	uint16_t retry = 0;

	//set the direction of the CS pin
	sbi(SD_CS_DDR, SD_CS_PIN);
	//output
	SD_cardType = 0;
	for (i = 0; i < 10; i++)
		SPI_master_transmit(0xff); //80 clock pulses spent before sending the first command
	SD_CS_ASSERT;
	do {

		response = SD_sendCommand(GO_IDLE_STATE, 0); //send 'reset & go idle' command
		retry++;
		if (retry > 0x20)
			return 1; //time out, card not detected

	} while (response != 0x01);

	SD_CS_DEASSERT;
	SPI_master_transmit(0xff);
	SPI_master_transmit(0xff);

	retry = 0;

	SD_version = 2; //default set to SD compliance with ver2.x;
					//this may change after checking the next command
	do {
		response = SD_sendCommand(SEND_IF_COND, 0x000001AA); //Check power supply status, mandatory for SDHC card
		retry++;
		if (retry > 0xfe) {
			//TX_NEWLINE;
			SD_version = 1;
			SD_cardType = 1;
			break;
		} //time out

	} while (response != 0x01);

	retry = 0;

	do {
		response = SD_sendCommand(APP_CMD, 0); //CMD55, must be sent before sending any ACMD command
		response = SD_sendCommand(SD_SEND_OP_COND, 0x40000000); //ACMD41

		retry++;
		if (retry > 0xfe) {
			//TX_NEWLINE;
			return 2; //time out, card initialization failed
		}

	} while (response != 0x00);

	retry = 0;
	SDHC_flag = 0;

	if (SD_version == 2) {
		do {
			response = SD_sendCommand(READ_OCR, 0);
			retry++;
			if (retry > 0xfe) {
				//TX_NEWLINE;
				SD_cardType = 0;
				break;
			} //time out

		} while (response != 0x00);

		if (SDHC_flag == 1)
			SD_cardType = 2;
		else
			SD_cardType = 3;
	}

	//SD_sendCommand(CRC_ON_OFF, OFF); //disable CRC; default - CRC disabled in SPI mode
	//SD_sendCommand(SET_BLOCK_LEN, 512); //set block size to 512; default size is 512

	return 0; //successful return
} // end SD_init

// Call this after SD_init()
uint8_t SD_card_type(void) { //because now SD_cardType is not public...
	return SD_cardType;
}

//******************************************************************
//Function	: to send a command to SD card
//Arguments	: unsigned char (8-bit command value)
// 			  & unsigned long (32-bit command argument)
//return	: unsigned char; response byte
//******************************************************************
uint8_t SD_sendCommand(uint8_t cmd, uint32_t arg) {
	uint8_t response, retry = 0, status;

	//SD card accepts byte address while SDHC accepts block address in multiples of 512
	//so, if it's SD card we need to convert block address into corresponding byte address by
	//multiplying it with 512. which is equivalent to shifting it left 9 times
	//following 'if' loop does that

	if (SDHC_flag == 0)
		if (cmd == READ_SINGLE_BLOCK || cmd == READ_MULTIPLE_BLOCKS
				|| cmd == WRITE_SINGLE_BLOCK || cmd == WRITE_MULTIPLE_BLOCKS
				|| cmd == ERASE_BLOCK_START_ADDR || cmd == ERASE_BLOCK_END_ADDR) {
			arg = arg << 9;
		}

	SD_CS_ASSERT;

	SPI_master_transmit(cmd | 0x40); //send command, first two bits always '01'
	SPI_master_transmit(arg >> 24);
	SPI_master_transmit(arg >> 16);
	SPI_master_transmit(arg >> 8);
	SPI_master_transmit(arg);

	if (cmd == SEND_IF_COND) //it is compulsory to send correct CRC for CMD8 (CRC=0x87) & CMD0 (CRC=0x95)
		SPI_master_transmit(0x87); //for remaining commands, CRC is ignored in SPI mode
	else
		SPI_master_transmit(0x95);

	while ((response = SPI_master_receive()) == 0xff) //wait response
		if (retry++ > 0xfe)
			break; //time out error

	if (response == 0x00 && cmd == 58) //checking response of CMD58
			{
		status = SPI_master_receive() & 0x40; //first byte of the OCR register (bit 31:24)
		if (status == 0x40)
			SDHC_flag = 1; //we need it to verify SDHC card
		else
			SDHC_flag = 0;

		SPI_master_receive(); //remaining 3 bytes of the OCR register are ignored here
		SPI_master_receive(); //one can use these bytes to check power supply limits of SD
		SPI_master_receive();
	}

	SPI_master_receive(); //extra 8 CLK
	SD_CS_DEASSERT;

	return response; //return state
} // end SD_sendCommand

//*****************************************************************
//Function	: to erase specified no. of blocks of SD card
//Arguments	: none
//return	: unsigned char; will be 0 if no error,
// 			  otherwise the response byte will be sent
//*****************************************************************
uint8_t SD_erase(uint32_t SD_startBlock, uint32_t SD_totalBlocks) {
	uint8_t response;

	response = SD_sendCommand(ERASE_BLOCK_START_ADDR, SD_startBlock); //send starting block address
	if (response != 0x00) //check for SD status: 0x00 - OK (No flags set)
		return response;

	response = SD_sendCommand(ERASE_BLOCK_END_ADDR,
			(SD_startBlock + SD_totalBlocks - 1)); //send end block address
	if (response != 0x00)
		return response;

	response = SD_sendCommand(ERASE_SELECTED_BLOCKS, 0); //erase all selected blocks
	if (response != 0x00)
		return response;

	return 0; //normal return
} // end SD_erase

//******************************************************************
//Function	: to read a single block from SD card
//Arguments	: none
//return	: unsigned char; will be 0 if no error,
// 			  otherwise the response byte will be sent
//******************************************************************
uint8_t SD_readSingleBlock(uint32_t SD_startBlock) {
	uint8_t response;
	uint16_t i, retry = 0;

	response = SD_sendCommand(READ_SINGLE_BLOCK, SD_startBlock); //read a Block command

	if (response != 0x00)
		return response; //check for SD status: 0x00 - OK (No flags set)

	SD_CS_ASSERT;

	retry = 0;
	while (SPI_master_receive() != 0xfe) //wait for start block token 0xfe (0x11111110)
		if (retry++ > 0xfffe) {
			SD_CS_DEASSERT;
			return 1;
		} //return if time-out

	for (i = 0; i < 512; i++) //read 512 bytes
		SD_buffer[i] = SPI_master_receive();

	SPI_master_receive(); //receive incoming CRC (16-bit), CRC is ignored here
	SPI_master_receive();

	SPI_master_receive(); //extra 8 clock pulses
	SD_CS_DEASSERT;

	return 0;
}

//******************************************************************
//Function	: to write to a single block of SD card
//Arguments	: none
//return	: unsigned char; will be 0 if no error,
// 			  otherwise the response byte will be sent
//******************************************************************
uint8_t SD_writeSingleBlock(uint32_t SD_startBlock) {
	uint8_t response;
	uint16_t i, retry = 0;

	response = SD_sendCommand(WRITE_SINGLE_BLOCK, SD_startBlock); //write a Block command

	if (response != 0x00)
		return response; //check for SD status: 0x00 - OK (No flags set)
	SD_CS_ASSERT;

	SPI_master_transmit(0xfe); //Send start block token 0xfe (0x11111110)

	for (i = 0; i < 512; i++) //send 512 bytes data
		SPI_master_transmit(SD_buffer[i]);

	SPI_master_transmit(0xff); //transmit dummy CRC (16-bit), CRC is ignored here
	SPI_master_transmit(0xff);

	response = SPI_master_receive();

	if ((response & 0x1f) != 0x05) //response= 0xXXX0AAA1 ; AAA='010' - data accepted
			{ //AAA='101'-data rejected due to CRC error
		SD_CS_DEASSERT; //AAA='110'-data rejected due to write error
		return response;
	}

	while (!SPI_master_receive()) //wait for SD card to complete writing and get idle
		if (retry++ > 0xfffe) {
			SD_CS_DEASSERT;
			return 1;
		}

	SD_CS_DEASSERT;
	SPI_master_transmit(0xff); //just spend 8 clock cycle delay before re asserting the CS line
	SD_CS_ASSERT; //re-asserting the CS line to verify if card is still busy

	while (!SPI_master_receive()) //wait for SD card to complete writing and get idle
		if (retry++ > 0xfffe) {
			SD_CS_DEASSERT;
			return 1;
		}
	SD_CS_DEASSERT;

	return 0;
}
#endif // ENABLE_SD_CARD

// TODO: define FAT32 error codes!
#ifdef ENABLE_FAT32


// TODO: to be rewritten... depends on PCF8583 RTC functions...
uint8_t getDateTime_FAT(void) {
#ifdef ENABLE_PCF8583
	//it should get the time from RTC but for now, returns "error" code
	return 1;
#else
	return 1; // returning "error" code if RTC not available
			  // date and time are 0 - good enough for testing and budget
#endif
}

//***************************************************************************
//Function: to read data from boot sector of SD card, to determine important
//parameters like bytesPerSector, sectorsPerCluster etc.
//Arguments: none
//return: none
//***************************************************************************
uint8_t F32_getBootSectorData(void) {
	struct BS_Structure *bpb; //mapping the buffer onto the structure
	struct MBRinfo_Structure *mbr;
	struct partitionInfo_Structure *partition;
	uint32_t dataSectors;

	unusedSectors = 0;

	SD_readSingleBlock(0);
	bpb = (struct BS_Structure *) SD_buffer;

	if (bpb->jumpBoot[0] != 0xE9 && bpb->jumpBoot[0] != 0xEB) //check if it is boot sector
			{
		mbr = (struct MBRinfo_Structure *) SD_buffer; //if it is not boot sector, it must be MBR

		if (mbr->signature != 0xaa55)
			return 1; //if it is not even MBR then it's not FAT32

		partition = (struct partitionInfo_Structure *) (mbr->partitionData); //first partition
		unusedSectors = partition->firstSector; //the unused sectors, hidden to the FAT

		SD_readSingleBlock(partition->firstSector); //read the bpb sector
		bpb = (struct BS_Structure *) SD_buffer;
		if (bpb->jumpBoot[0] != 0xE9 && bpb->jumpBoot[0] != 0xEB)
			return 1;
	}

	bytesPerSector = bpb->bytesPerSector;
#ifdef ENABLE_SD_CARD_DEBUG
	//serial_puthexU16(bytesPerSector); serial_putchar(' ');
#endif
	sectorPerCluster = bpb->sectorPerCluster;
#ifdef ENABLE_SD_CARD_DEBUG
	//serial_puthexU08(sectorPerCluster); serial_putchar(' ');
#endif
	reservedSectorCount = bpb->reservedSectorCount;
	rootCluster = bpb->rootCluster; // + (sector / sectorPerCluster) +1;
	firstDataSector = bpb->hiddenSectors + reservedSectorCount
			+ (bpb->numberofFATs * bpb->FATsize_F32);

	dataSectors = bpb->totalSectors_F32 - bpb->reservedSectorCount
			- (bpb->numberofFATs * bpb->FATsize_F32);
	totalClusters = dataSectors / sectorPerCluster;
#ifdef ENABLE_SD_CARD_DEBUG
	//serial_puthexU32(totalClusters); serial_putchar(' ');
#endif
	if ((F32_getSetFreeCluster(TOTAL_FREE, GET, 0)) > totalClusters) //check if FSinfo free clusters count is valid
		freeClusterCountUpdated = 0;
	else
		freeClusterCountUpdated = 1;
	return 0;
}

//***************************************************************************
//Function: to calculate first sector address of any given cluster
//Arguments: cluster number for which first sector is to be found
//return: first sector address
//***************************************************************************
uint32_t F32_getFirstSector(uint32_t clusterNumber) {
	return (((clusterNumber - 2) * sectorPerCluster) + firstDataSector);
}

//***************************************************************************
//Function: get cluster entry value from FAT to find out the next cluster in the chain
//or set new cluster entry in FAT
//Arguments: 1. current cluster number, 2. get_set (=GET, if next cluster is to be found or = SET,
//if next cluster is to be set 3. next cluster number, if argument#2 = SET, else 0
//return: next cluster number, if if argument#2 = GET, else 0
//****************************************************************************
uint32_t F32_getSetNextCluster(uint32_t clusterNumber, uint8_t get_set,
		uint32_t clusterEntry) {
	uint16_t FATEntryOffset;
	uint32_t *FATEntryValue;
	uint32_t FATEntrySector;
	uint8_t retry = 0;

//get sector number of the cluster entry in the FAT
	FATEntrySector = unusedSectors + reservedSectorCount
			+ ((clusterNumber * 4) / bytesPerSector);

//get the offset address in that sector number
	FATEntryOffset = (uint16_t) ((clusterNumber * 4) % bytesPerSector);

//read the sector into a buffer
	while (retry < 10) {
		if (!SD_readSingleBlock(FATEntrySector))
			break;
		retry++;
	}

//get the cluster address from the buffer
	FATEntryValue = (uint32_t *) &SD_buffer[FATEntryOffset];

	if (get_set == GET)
		return ((*FATEntryValue) & 0x0fffffff);

	*FATEntryValue = clusterEntry; //for setting new value in cluster entry in FAT

	SD_writeSingleBlock(FATEntrySector);

	return (0);
}

//********************************************************************************************
//Function: to get or set next free cluster or total free clusters in FSinfo sector of SD card
//Arguments: 1.flag:TOTAL_FREE or NEXT_FREE,
//       2.flag: GET or SET
//       3.new FS entry, when argument2 is SET; or 0, when argument2 is GET
//return: next free cluster, if arg1 is NEXT_FREE & arg2 is GET
//        total number of free clusters, if arg1 is TOTAL_FREE & arg2 is GET
//      0xffffffff, if any error or if arg2 is SET
//********************************************************************************************
uint32_t F32_getSetFreeCluster(uint8_t totOrNext, uint8_t get_set,
		uint32_t FSEntry) {


	struct FSInfo_Structure *FS = (struct FSInfo_Structure *) &SD_buffer;

	SD_readSingleBlock(unusedSectors + 1);

	if ((FS->leadSignature != 0x41615252)
			|| (FS->structureSignature != 0x61417272)
			|| (FS->trailSignature != 0xaa550000))
		return 0xffffffff;

	if (get_set == GET) {
		if (totOrNext == TOTAL_FREE)
			return (FS->freeClusterCount);
		else
			// when totOrNext = NEXT_FREE
			return (FS->nextFreeCluster);
	} else {
		if (totOrNext == TOTAL_FREE)
			FS->freeClusterCount = FSEntry;
		else
			// when totOrNext = NEXT_FREE
			FS->nextFreeCluster = FSEntry;

		SD_writeSingleBlock(unusedSectors + 1); //update FSinfo
	}
	return 0xffffffff;
}

//***************************************************************************
//Function: to get DIR/FILE list or a single file address (cluster number) or to delete a specified file
//Arguments: #1 - flag: GET_LIST, GET_FILE or DELETE #2 - pointer to file name (0 if arg#1 is GET_LIST)
//return: first cluster of the file, if flag = GET_FILE
//        print file/dir list of the root directory, if flag = GET_LIST
//      Delete the file mentioned in arg#2, if flag = DELETE
//****************************************************************************
struct dir_Structure* F32_findFiles(uint8_t flag, uint8_t *fileName) {
	uint32_t cluster, sector, firstSector, firstCluster, nextCluster;
	struct dir_Structure *dir;
	uint16_t i;
	uint8_t j;

	cluster = rootCluster; //root cluster

	while (1) {
		firstSector = F32_getFirstSector(cluster);

		for (sector = 0; sector < sectorPerCluster; sector++) {
			SD_readSingleBlock(firstSector + sector);

			for (i = 0; i < bytesPerSector; i += 32) {
				dir = (struct dir_Structure *) &SD_buffer[i];

				if (dir->name[0] == EMPTY) //indicates end of the file list of the directory
				{
#ifdef ENABLE_SD_CARD_DEBUG
					if(flag == DELETE) serial_putstr_f((uint8_t *)PSTR("\r\nFile does not exist!\0"));
#endif
					return 0;
				}
				if ((dir->name[0] != DELETED)
						&& (dir->attrib != ATTR_LONG_NAME)) {
					if ((flag == GET_FILE) || (flag == DELETE)) {
						for (j = 0; j < 11; j++)
							if (dir->name[j] != fileName[j])
								break;
						if (j == 11) {
							if (flag == GET_FILE) {
								appendFileSector = firstSector + sector;
								appendFileLocation = i;
								appendStartCluster =
										(((uint32_t) dir->firstClusterHI) << 16)
												| dir->firstClusterLO;
								fileSize = dir->fileSize;
								return (dir);
							} else //when flag = DELETE
							{
#ifdef ENABLE_SD_CARD_DEBUG
								//TX_NEWLINE;
								serial_putstr_f((uint8_t *)PSTR("\r\nDeleting..\0"));
								//TX_NEWLINE;
								//TX_NEWLINE;
#endif
								firstCluster = (((uint32_t) dir->firstClusterHI)
										<< 16) | dir->firstClusterLO;

								//mark file as 'deleted' in FAT table
								dir->name[0] = DELETED;
								SD_writeSingleBlock(firstSector + sector);

								F32_freeMemoryUpdate(ADD, dir->fileSize);

								//update next free cluster entry in FSinfo sector
								cluster = F32_getSetFreeCluster(NEXT_FREE, GET,
										0);
								if (firstCluster < cluster)
									F32_getSetFreeCluster(NEXT_FREE, SET,
											firstCluster);

								//mark all the clusters allocated to the file as 'free'
								while (1) {
									nextCluster = F32_getSetNextCluster(
											firstCluster, GET, 0);
									F32_getSetNextCluster(firstCluster, SET, 0);
									if (nextCluster > 0x0ffffff6) {
#ifdef ENABLE_SD_CARD_DEBUG
										serial_putstr_f((uint8_t *)PSTR("\r\nFile deleted!"));
#endif
										return 0;
									}
									firstCluster = nextCluster;
								}
							}
						}
					} else //when flag = GET_LIST
					{
#ifdef ENABLE_SD_CARD_DEBUG
						TX_NEWLINE;
#endif
						for (j = 0; j < 11; j++) {
#ifdef ENABLE_SD_CARD_DEBUG
							if(j == 8) serial_putchar(' ');
							serial_putchar(dir->name[j]);
#endif
						}
#ifdef ENABLE_SD_CARD_DEBUG
						serial_putstr((uint8_t *)"   \0");
#endif
						if ((dir->attrib != 0x10) && (dir->attrib != 0x08)) {
#ifdef ENABLE_SD_CARD_DEBUG
							serial_putstr_f((uint8_t *)PSTR("FILE\0"));
							serial_putstr_f((uint8_t *)PSTR("   \0"));
#endif
							F32_displayMemory(LOW, dir->fileSize);
						} else{
#ifdef ENABLE_SD_CARD_DEBUG
							serial_putstr(
									(dir->attrib == 0x10) ?
											(uint8_t *) "DIR\0" :
											(uint8_t *) "ROOT\0");
#endif
						}
					}
				}
			}
		}

		cluster = (F32_getSetNextCluster(cluster, GET, 0));

		if (cluster > 0x0ffffff6)
			return 0;
		if (cluster == 0) {
#ifdef ENABLE_SD_CARD_DEBUG
			serial_putstr_f((uint8_t *)PSTR("\r\nError in getting cluster\0"));
#endif
			return 0;
		}
	}
	return 0;
}

//***************************************************************************
//Function: if flag=READ then to read file from SD card and send contents to UART
//if flag=VERIFY then functions will verify whether a specified file is already existing
//Arguments: flag (READ or VERIFY) and pointer to the file name
//return: 0, if normal operation or flag is READ
//        1, if file is already existing and flag = VERIFY; or if flag=READ and file does not exist
//      2, if file name is incompatible
//***************************************************************************
uint8_t F32_readFile(uint8_t flag, uint8_t *fileName) {
	struct dir_Structure *dir;
	uint32_t cluster, byteCounter = 0, fileSize, firstSector;
	uint16_t k;
	uint8_t j, error;

	//error = F32_convertFileName(fileName); //convert fileName into FAT format
	//if (error)
	//	return 2;

	dir = F32_findFiles(GET_FILE, fileName); //get the file location
	if (dir == 0) {
		if (flag == READ)
			return (1);
		else
			return (0);
	}

	if (flag == VERIFY)
		return (1); //specified file name is already existing

	cluster = (((uint32_t) dir->firstClusterHI) << 16) | dir->firstClusterLO;

	fileSize = dir->fileSize;
#ifdef ENABLE_SD_CARD_DEBUG
	//TX_NEWLINE;
	//TX_NEWLINE;
#endif
	while (1) {
		firstSector = F32_getFirstSector(cluster);

		for (j = 0; j < sectorPerCluster; j++) {
			SD_readSingleBlock(firstSector + j);

			for (k = 0; k < 512; k++) {
#ifdef ENABLE_SD_CARD_DEBUG
				serial_putchar(SD_buffer[k]);
#endif
				if ((byteCounter++) >= fileSize)
					return 0;
			}
		}
		cluster = F32_getSetNextCluster(cluster, GET, 0);
		if (cluster == 0) {
#ifdef ENABLE_SD_CARD_DEBUG
			serial_putstr_f((uint8_t *)PSTR("\r\nError in getting cluster\0"));
#endif
			return 0;
		}
	}
	return 0;
}

//***************************************************************************
//Function: to convert normal short file name into FAT format
//Arguments: pointer to the file name
//return: 0, if successful else 1.
//***************************************************************************
/*uint8_t F32_convertFileName(uint8_t *fileName) {
	uint8_t fileNameFAT[11];
	uint8_t i, j, k;


	//i = 0;
	//for (j = 0; j < 12; j++){
	//	if (fileName[j] != '.') {
	//		fileNameFAT[i] = fileName[j];
	//		i = i + 1;
	//	}

	//}

	for (j = 0; j < 12; j++)
		if (fileName[j] == '.')
			break;

	if (j > 8) {
#ifdef ENABLE_SD_CARD_DEBUG
		serial_putstr_f((uint8_t *)PSTR("\r\nInvalid fileName..\0"));
#endif
		return 1;

	}

	for (k = 0; k < j; k++) //setting file name
		fileNameFAT[k] = fileName[k];

	for (k = j; k <= 7; k++) //filling file name trail with blanks
		fileNameFAT[k] = ' ';

	j++;
	for (k = 8; k < 11; k++) //setting file extension
			{
		if (fileName[j] != 0)
			fileNameFAT[k] = fileName[j++];
		else
			//filling extension trail with blanks
			while (k < 11)
				fileNameFAT[k++] = ' ';
	}

	for (j = 0; j < 11; j++) //converting small letters to caps
		if ((fileNameFAT[j] >= 0x61) && (fileNameFAT[j] <= 0x7a))
			fileNameFAT[j] -= 0x20;

	for (j = 0; j < 11; j++)
		fileName[j] = fileNameFAT[j];

	return 0;
}*/

//************************************************************************************
//Function: to create a file in FAT32 format in the root directory if given
//      file name does not exist; if the file already exists then append the data
//Arguments: pointer to the file name
//return: none
//************************************************************************************

uint8_t F32_writeFile(uint8_t *fileName, uint8_t *dataString) {
	uint8_t j, k, data = 0, error, fileCreatedFlag = 0, start = 0, appendFile = 0,
			sector = 0;
	uint16_t i, firstClusterHigh = 0, firstClusterLow = 0; //value 0 is assigned just to avoid warning in compilation
	struct dir_Structure *dir;
	uint32_t cluster, nextCluster, prevCluster, firstSector, clusterCount,
			extraMemory;

	j = F32_readFile(VERIFY, fileName);

	if (j == 1) {
#ifdef ENABLE_SD_CARD_DEBUG
		serial_putstr_f((uint8_t *)PSTR("\r\nFile already exists, appending data..\0"));
#endif
		appendFile = 1;
		cluster = appendStartCluster;
		clusterCount = 0;
		while (1) {
			nextCluster = F32_getSetNextCluster(cluster, GET, 0);
			if (nextCluster == FAT32_EOF)
				break;
			cluster = nextCluster;
			clusterCount++;
		}

		sector = (fileSize - (clusterCount * sectorPerCluster * bytesPerSector))
				/ bytesPerSector; //last sector number of the last cluster of the file
		start = 1;
	} else if (j == 2)
		return 1; //invalid file name

	else {
#ifdef ENABLE_SD_CARD_DEBUG
		//TX_NEWLINE;
		serial_putstr_f((uint8_t *)PSTR("\r\nCreating File..\0"));
#endif
		cluster = F32_getSetFreeCluster(NEXT_FREE, GET, 0);
		if (cluster > totalClusters)
			cluster = rootCluster;

		cluster = F32_searchNextFreeCluster(cluster);
		if (cluster == 0) {
#ifdef ENABLE_SD_CARD_DEBUG
			//TX_NEWLINE;
			serial_putstr_f((uint8_t *)PSTR("\r\nNo free cluster!\0"));
#endif
			return 1;
		}
		F32_getSetNextCluster(cluster, SET, FAT32_EOF); //last cluster of the file, marked EOF

		firstClusterHigh = (uint16_t) ((cluster & 0xffff0000) >> 16);
		firstClusterLow = (uint16_t) (cluster & 0x0000ffff);
		fileSize = 0;
	}

	k = 0;

	while (1) {
		if (start) {
			start = 0;
			SD_startBlock = F32_getFirstSector(cluster) + sector;
			SD_readSingleBlock(SD_startBlock);
			i = fileSize % bytesPerSector;
			j = sector;
		} else {
			SD_startBlock = F32_getFirstSector(cluster);
			i = 0;
			j = 0;
		}

		do {
			data = dataString[k++];
#ifdef ENABLE_SD_CARD_DEBUG
			serial_putchar(data); // was data
#endif
			SD_buffer[i++] = data;
			fileSize++;

			if (i >= 512) //though 'i' will never become greater than 512, it's kept here to avoid
					{ //infinite loop in case it happens to be greater than 512 due to some data corruption
				i = 0;
				error = SD_writeSingleBlock(SD_startBlock);
				j++;
				if (j == sectorPerCluster) {
					j = 0;
					break;
				}
				SD_startBlock++;
			}
		} while ((data != '\n') && (k < MAX_STRING_SIZE)); //stop when newline character is found
		//or when string size limit reached

		if ((data == '\n') || (k >= MAX_STRING_SIZE)) {
			for (; i < 512; i++) //fill the rest of the buffer with 0x00
				SD_buffer[i] = 0x00;
			error = SD_writeSingleBlock(SD_startBlock);

			break;
		}

		prevCluster = cluster;

		cluster = F32_searchNextFreeCluster(prevCluster); //look for a free cluster starting from the current cluster

		if (cluster == 0) {
#ifdef ENABLE_SD_CARD_DEBUG
			//TX_NEWLINE;
			serial_putstr_f((uint8_t *)PSTR("\r\nNo free cluster!\0"));
#endif
			return 1;
		}

		F32_getSetNextCluster(prevCluster, SET, cluster);
		F32_getSetNextCluster(cluster, SET, FAT32_EOF); //last cluster of the file, marked EOF
	}

	F32_getSetFreeCluster(NEXT_FREE, SET, cluster); //update FSinfo next free cluster entry

	error = getDateTime_FAT(); //get current date & time from the RTC
	if (error) {
		dateFAT = 0;
		timeFAT = 0;
	}

	if (appendFile) //executes this loop if file is to be appended
	{
		SD_readSingleBlock(appendFileSector);
		dir = (struct dir_Structure *) &SD_buffer[appendFileLocation];

		dir->lastAccessDate = 0; //date of last access ignored
		dir->writeTime = timeFAT; //setting new time of last write, obtained from RTC
		dir->writeDate = dateFAT; //setting new date of last write, obtained from RTC
		extraMemory = fileSize - dir->fileSize;
		dir->fileSize = fileSize;
		SD_writeSingleBlock(appendFileSector);
		F32_freeMemoryUpdate(REMOVE, extraMemory); //updating free memory count in FSinfo sector;

#ifdef ENABLE_SD_CARD_DEBUG
		//TX_NEWLINE;
		serial_putstr_f((uint8_t *)PSTR("\r\nFile appended!\0"));
#endif
		return 0;
	}

//executes following portion when new file is created

	prevCluster = rootCluster; //root cluster

	while (1) {
		firstSector = F32_getFirstSector(prevCluster);

		for (sector = 0; sector < sectorPerCluster; sector++) {
			SD_readSingleBlock(firstSector + sector);

			for (i = 0; i < bytesPerSector; i += 32) {
				dir = (struct dir_Structure *) &SD_buffer[i];

				if (fileCreatedFlag) //to mark last directory entry with 0x00 (empty) mark
				{ //indicating end of the directory file list
					//dir->name[0] = EMPTY;
					//SD_writeSingleBlock (firstSector + sector);
					return 0;
				}

				if ((dir->name[0] == EMPTY) || (dir->name[0] == DELETED)) //looking for an empty slot to enter file info
						{
					for (j = 0; j < 11; j++)
						dir->name[j] = fileName[j];
					dir->attrib = ATTR_ARCHIVE; //setting file attribute as 'archive'
					dir->NTreserved = 0; //always set to 0
					dir->timeTenth = 0; //always set to 0
					dir->createTime = timeFAT; //setting time of file creation, obtained from RTC
					dir->createDate = dateFAT; //setting date of file creation, obtained from RTC
					dir->lastAccessDate = 0; //date of last access ignored
					dir->writeTime = timeFAT; //setting new time of last write, obtained from RTC
					dir->writeDate = dateFAT; //setting new date of last write, obtained from RTC
					dir->firstClusterHI = firstClusterHigh;
					dir->firstClusterLO = firstClusterLow;
					dir->fileSize = fileSize;

					SD_writeSingleBlock(firstSector + sector);
					fileCreatedFlag = 1;
#ifdef ENABLE_SD_CARD_DEBUG
					//TX_NEWLINE;
					//TX_NEWLINE;
					serial_putstr_f((uint8_t *)PSTR("\r\nFile Created! \0"));
#endif

					F32_freeMemoryUpdate(REMOVE, fileSize); //updating free memory count in FSinfo sector

				}
			}
		}

		cluster = F32_getSetNextCluster(prevCluster, GET, 0);

		if (cluster > 0x0ffffff6) {
			if (cluster == FAT32_EOF) //this situation will come when total files in root is multiple of (32*sectorPerCluster)
			{
				cluster = F32_searchNextFreeCluster(prevCluster); //find next cluster for root directory entries
				F32_getSetNextCluster(prevCluster, SET, cluster); //link the new cluster of root to the previous cluster
				F32_getSetNextCluster(cluster, SET, FAT32_EOF); //set the new cluster as end of the root directory
			}

			else {
#ifdef ENABLE_SD_CARD_DEBUG
				serial_putstr_f((uint8_t *)PSTR("\r\nEnd of Cluster Chain\0"));
#endif
				return 1;
			}
		}
		if (cluster == 0) {
#ifdef ENABLE_SD_CARD_DEBUG
			serial_putstr_f((uint8_t *)PSTR("\r\nError in getting cluster\0"));
#endif
			return 1;
		}

		prevCluster = cluster;
	}

	return 0;
}

//***************************************************************************
//Function: to search for the next free cluster in the root directory
//          starting from a specified cluster
//Arguments: Starting cluster
//return: the next free cluster
//****************************************************************
uint32_t F32_searchNextFreeCluster(uint32_t startCluster) {
	uint32_t cluster, *value, sector;
	uint8_t i;

	startCluster -= (startCluster % 128); //to start with the first file in a FAT sector
	for (cluster = startCluster; cluster < totalClusters; cluster += 128) {
		sector = unusedSectors + reservedSectorCount
				+ ((cluster * 4) / bytesPerSector);
		SD_readSingleBlock(sector);
		for (i = 0; i < 128; i++) {
			value = (uint32_t *) &SD_buffer[i * 4];
			if (((*value) & 0x0fffffff) == 0)
				return (cluster + i);
		}
	}

	return 0;
}

//************************************************************
//Function: To convert the uint32_t value of memory into
//          text string and send to UART
//Arguments: 1. uint8_t flag. If flag is HIGH, memory will be displayed in KBytes, else in Bytes.
//       2. uint32_t memory value
//return: none
//************************************************************
void F32_displayMemory(uint8_t flag, uint32_t memory) {
	uint8_t memoryString[] = "              Bytes\0"; //19 character long string for memory display
	uint8_t i;
	for (i = 12; i > 0; i--) //converting freeMemory into ASCII string
			{
		if (i == 5 || i == 9) {
			memoryString[i - 1] = ',';
			i--;
		}
		memoryString[i - 1] = (memory % 10) | 0x30;
		memory /= 10;
		if (memory == 0)
			break;
	}
	if (flag == HIGH)
		memoryString[13] = 'K';
#if defined(ENABLE_SERIAL) || defined(ENABLE_SERIAL_POLL)
	serial_putstr(memoryString);
#endif
}

//********************************************************************
//Function: to delete a specified file from the root directory
//Arguments: pointer to the file name
//return: none
//********************************************************************
void F32_deleteFile(uint8_t *fileName) {
	//uint8_t error;

	//error = F32_convertFileName(fileName);
	//if (error)
	//	return;

	F32_findFiles(DELETE, fileName);
}

//********************************************************************
//Function: update the free memory count in the FSinfo sector.
//      Whenever a file is deleted or created, this function will be called
//      to ADD or REMOVE clusters occupied by the file
//Arguments: #1.flag ADD or REMOVE #2.file size in Bytes
//return: none
//********************************************************************
void F32_freeMemoryUpdate(uint8_t flag, uint32_t size) {
	uint32_t freeClusters;
	//convert file size into number of clusters occupied
	if ((size % 512) == 0)
		size = size / 512;
	else
		size = (size / 512) + 1;
	if ((size % 8) == 0)
		size = size / 8;
	else
		size = (size / 8) + 1;

	if (freeClusterCountUpdated) {
		freeClusters = F32_getSetFreeCluster(TOTAL_FREE, GET, 0);
		if (flag == ADD)
			freeClusters = freeClusters + size;
		else
			//when flag = REMOVE
			freeClusters = freeClusters - size;
		F32_getSetFreeCluster(TOTAL_FREE, SET, freeClusters);
	}
}

//******** END ****** www.dharmanitech.com *****
#endif //ENABLE_FAT32
#ifdef ENABLE_ONE_WIRE
uint8_t parasite;
// Perform the onewire reset function.  We will wait up to 250uS for
// the bus to come high, if it doesn't then it is broken or shorted
// and we return a 0;
//
// Returns 1 if a device asserted a presence pulse, 0 otherwise.
//
uint8_t ow_reset(void)
{
	uint8_t r;
	uint8_t retries = 125;

	cli();
	OW_DIR_IN();
	sei();
	// wait until the wire is high... just in case
	do {
		if (--retries == 0) return 0;
		_delay_us(2);
	}while ( !OW_GET_IN());

	cli();
	OW_OUT_LOW();
	OW_DIR_OUT(); // drive output low
	sei();
	_delay_us(500);
	cli();
	OW_DIR_IN();// allow it to float
	_delay_us(80);
	r = !OW_GET_IN();
	sei();
	_delay_us(420);
	return r;
}

//
// Write a bit. Port and bit is used to cut lookup time and provide
// more certain timing.
//
void ow_write_bit(uint8_t v)
{
	if (v & 1) {
		cli();
		OW_OUT_LOW();
		OW_DIR_OUT(); // drive output low
		_delay_us(10);
		OW_OUT_HIGH();// drive output high
		sei();
		_delay_us(55);
	} else {
		cli();
		OW_OUT_LOW();
		OW_DIR_OUT(); // drive output low
		_delay_us(65);
		OW_OUT_HIGH();// drive output high
		sei();
		_delay_us(5);
	}
}

//
// Read a bit. Port and bit is used to cut lookup time and provide
// more certain timing.
//
uint8_t ow_read_bit(void)
{
	uint8_t r;

	cli();
	OW_DIR_OUT();
	OW_OUT_LOW();
	_delay_us(3);
	OW_DIR_IN(); // let pin float, pull up will raise
	_delay_us(9);
	r = OW_GET_IN();
	sei();
	_delay_us(53);
	return r;
}

//
// Write a byte. The writing code uses the active drivers to raise the
// pin high, if you need power after the write (e.g. DS18S20 in
// parasite power mode) then set 'power' to 1, otherwise the pin will
// go tri-state at the end of the write to avoid heating in a short or
// other mishap.
//
void ow_write_byte(uint8_t v, uint8_t power /* = 0 */)
{
	uint8_t bitMask;

	for (bitMask = 0x01; bitMask; bitMask <<= 1) {
		ow_write_bit( (bitMask & v)?1:0);
	}
	if ( !power) {
		cli();
		OW_DIR_IN();
		OW_OUT_LOW();
		sei();
	}
}

//
// Read a byte
//
uint8_t ow_read_byte() {
	uint8_t bitMask;
	uint8_t r = 0;

	for (bitMask = 0x01; bitMask; bitMask <<= 1) {
		if ( ow_read_bit()) r |= bitMask;
	}
	return r;
}

//
// Do a ROM select
//
void ow_select_rom( uint8_t rom[8])
{
	int i;

	ow_write_byte(OW_MATCH_ROM, parasite); // Choose ROM
	for( i = 0; i < 8; i++) ow_write_byte(rom[i], parasite);
}

//
// Do a ROM skip
//
void ow_skip_rom()
{
	ow_write_byte(OW_SKIP_ROM, parasite); // Skip ROM
}

void ow_depower()
{
	cli();
	OW_DIR_IN();
	sei();
}

#if ONEWIRE_SEARCH

// global search state
uint8_t ROM_NO[8];
uint8_t LastDiscrepancy;
uint8_t LastFamilyDiscrepancy;
uint8_t LastDeviceFlag;

//
// You need to use this function to start a search again from the beginning.
// You do not need to do it for the first search, though you could.
//
void ow_reset_search()
{
	// reset the search state
	LastDiscrepancy = 0;
	LastDeviceFlag = FALSE;
	LastFamilyDiscrepancy = 0;
	for(int i = 7;; i--)
	{
		ROM_NO[i] = 0;
		if ( i == 0) break;
	}
}

//
// Perform a search. If this function returns a '1' then it has
// enumerated the next device and you may retrieve the ROM from the
// OneWire::address variable. If there are no devices, no further
// devices, or something horrible happens in the middle of the
// enumeration then a 0 is returned.  If a new device is found then
// its address is copied to newAddr.  Use OneWire::reset_search() to
// start over.
//
// --- Replaced by the one from the Dallas Semiconductor web site ---
//--------------------------------------------------------------------------
// Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
// search state.
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : device not found, end of search
//
uint8_t ow_search(uint8_t *newAddr)
{
	uint8_t id_bit_number;
	uint8_t last_zero, rom_byte_number, search_result;
	uint8_t id_bit, cmp_id_bit;

	uint8_t rom_byte_mask, search_direction;

	// initialize for search
	id_bit_number = 1;
	last_zero = 0;
	rom_byte_number = 0;
	rom_byte_mask = 1;
	search_result = 0;

	// if the last call was not the last one
	if (!LastDeviceFlag)
	{
		// 1-Wire reset
		if (!ow_reset())
		{
			// reset the search
			LastDiscrepancy = 0;
			LastDeviceFlag = FALSE;
			LastFamilyDiscrepancy = 0;
			return FALSE;
		}

		// issue the search command
		ow_write_byte(OW_SEARCH_ROM, parasite);

		// loop to do the search
		do
		{
			// read a bit and its complement
			id_bit = ow_read_bit();
			cmp_id_bit = ow_read_bit();

			// check for no devices on 1-wire
			if ((id_bit == 1) && (cmp_id_bit == 1))
			break;
			else
			{
				// all devices coupled have 0 or 1
				if (id_bit != cmp_id_bit)
				search_direction = id_bit;// bit write value for search
				else
				{
					// if this discrepancy if before the Last Discrepancy
					// on a previous next then pick the same as last time
					if (id_bit_number < LastDiscrepancy)
					search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
					else
					// if equal to last pick 1, if not then pick 0
					search_direction = (id_bit_number == LastDiscrepancy);

					// if 0 was picked then record its position in LastZero
					if (search_direction == 0)
					{
						last_zero = id_bit_number;

						// check for Last discrepancy in family
						if (last_zero < 9)
						LastFamilyDiscrepancy = last_zero;
					}
				}

				// set or clear the bit in the ROM byte rom_byte_number
				// with mask rom_byte_mask
				if (search_direction == 1)
				ROM_NO[rom_byte_number] |= rom_byte_mask;
				else
				ROM_NO[rom_byte_number] &= ~rom_byte_mask;

				// serial number search direction write bit
				ow_write_bit(search_direction);

				// increment the byte counter id_bit_number
				// and shift the mask rom_byte_mask
				id_bit_number++;
				rom_byte_mask <<= 1;

				// if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
				if (rom_byte_mask == 0)
				{
					rom_byte_number++;
					rom_byte_mask = 1;
				}
			}
		}
		while(rom_byte_number < 8); // loop until through all ROM bytes 0-7

		// if the search was successful then
		if (!(id_bit_number < 65))
		{
			// search successful so set LastDiscrepancy,LastDeviceFlag,search_result
			LastDiscrepancy = last_zero;

			// check for last device
			if (LastDiscrepancy == 0)
			LastDeviceFlag = TRUE;

			search_result = TRUE;
		}
	}

	// if no device found then reset counters so next 'search' will be like a first
	if (!search_result || !ROM_NO[0])
	{
		LastDiscrepancy = 0;
		LastDeviceFlag = FALSE;
		LastFamilyDiscrepancy = 0;
		search_result = FALSE;
	}
	for (int i = 0; i < 8; i++) newAddr[i] = ROM_NO[i];
	return search_result;
}
#endif //ONEWIRE_SEARCH
#if ONEWIRE_CRC
// The 1-Wire CRC scheme is described in Maxim Application Note 27:
// "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"
//

#if ONEWIRE_CRC8_TABLE
// This table comes from Dallas sample code where it is freely reusable,
// though Copyright (C) 2000 Dallas Semiconductor Corporation
static const uint8_t PROGMEM dscrc_table[] = {
	0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
	157,195, 33,127,252,162, 64, 30, 95, 1,227,189, 62, 96,130,220,
	35,125,159,193, 66, 28,254,160,225,191, 93, 3,128,222, 60, 98,
	190,224, 2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
	70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89, 7,
	219,133,103, 57,186,228, 6, 88, 25, 71,165,251,120, 38,196,154,
	101, 59,217,135, 4, 90,184,230,167,249, 27, 69,198,152,122, 36,
	248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91, 5,231,185,
	140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
	17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
	175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
	50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
	202,148,118, 40,171,245, 23, 73, 8, 86,180,234,105, 55,213,139,
	87, 9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
	233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
	116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53};

//
// Compute a Dallas Semiconductor 8 bit CRC. These show up in the ROM
// and the registers.  (note: this might better be done without to
// table, it would probably be smaller and certainly fast enough
// compared to all those delayMicrosecond() calls.  But I got
// confused, so I use this table from the examples.)
//
uint8_t ow_crc8( uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;

	while (len--) {
		crc = pgm_read_byte(dscrc_table + (crc ^ *addr++));
	}
	return crc;
}
#else
//
// Compute a Dallas Semiconductor 8 bit CRC directly.
//
uint8_t ow_crc8( uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;

	while (len--) {
		uint8_t inbyte = *addr++;
		for (uint8_t i = 8; i; i--) {
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
	}
	return crc;
}
#endif //ONEWIRE_CRC8_TABLE

#if ONEWIRE_CRC16
static short oddparity[16] = {0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0};

//
// Compute a Dallas Semiconductor 16 bit CRC. I have never seen one of
// these, but here it is.
//
unsigned short ow_crc16(unsigned short *data, unsigned short len)
{
	unsigned short i;
	unsigned short crc = 0;

	for ( i = 0; i < len; i++) {
		unsigned short cdata = data[len];

		cdata = (cdata ^ (crc & 0xff)) & 0xff;
		crc >>= 8;

		if (oddparity[cdata & 0xf] ^ oddparity[cdata >> 4]) crc ^= 0xc001;

		cdata <<= 6;
		crc ^= cdata;
		cdata <<= 1;
		crc ^= cdata;
	}
	return crc;
}
#endif //ONEWIRE_CRC16
#endif // ONEWIRE_CRC

// TODO: The code is horrible (it was made in C++) and must be rewritten.
#ifdef ENABLE_DALLAS_TEMP
// private variables



#endif // ENABLE_DALLAS_TEMP
#endif //ENABLE_ONE_WIRE

#ifdef ENABLE_IR
volatile uint8_t ir_readptr = 0;
volatile uint8_t ir_writeptr = 0;
volatile uint8_t ir_buffer[IR_BUFFER_SIZE];

/*****************************************************************************
 *  receive from IR Receiver - external interrupt 0 on pin PD2 (arduino pin 2)
 *                                                         PD0 on ATmega2560
 *
 * Use the external interrupt to decode the Sony IR protocol.  We use
 * both rising and falling edges to measure pulse widths.  The timer/counter
 * is needed to take these measurements.  If the address matches our device,
 * we store the command in a circular buffer for retrieval outside of
 * interrupt context.
 *
 * tested with Vishay TSOP392 IR receiver and universal remote
 ****************************************************************************/
ISR(INT0_vect)
{
	static uint8_t value;
	static uint8_t address;
	static uint8_t sigcount = 0;
#if defined(__AVR_ATmega48__)      || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)    || \
    defined(__AVR_ATmega16__)      || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
	if (PIND & _BV(2)) { // rising edge
#elif defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__))
		if (PIND & _BV(0)) { // rising edge
#endif
			uint16_t count;
			count = TCNT0;

			onboard_led_off();

#if (F_CPU == 16000000)
			if (count < 14) { // 0
#endif
#if (F_CPU == 8000000)
				if (count < 7) { // 0
#endif
					sigcount++;
					if (sigcount < 8) {
						value = value & ~(1 << (sigcount-1));
					} else {
						address = address & ~(1 << (sigcount-8));
					}
#if (F_CPU == 16000000)
				} else if (count < 30) { // 1
#endif
#if (F_CPU == 8000000)
				} else if (count < 15) { // 1
#endif
					sigcount++;
					if (sigcount < 8) {
						value = value | (1 << (sigcount-1));
					} else {
						address = address | (1 << (sigcount-8));
					}
				} else { // this starts a new one
					sigcount = 0;
					value = 0;
					address = 0;
				}

				/* we have a command and it is for us */
				if ((sigcount == 12) && ((address == 26) || (address == 17)
								|| (address == REMOTE_DEVICE_SONY_TV000)))
				{
					ir_buffer[ir_writeptr] = value;
					ir_writeptr = (ir_writeptr + 1) % IR_BUFFER_SIZE;
#ifdef IR_DEBOUNCE
					_delay_ms(200); /* debounce */
#endif
				}

			} else { // falling edge
				TCNT0 = 0;// reset
				onboard_led_on();
			}
		}

		/**************************************************************************
		 * ir_init
		 * AVR PD2 - arduino digital pin 2
		 **************************************************************************/
		void ir_init(void)
		{
#if defined(__AVR_ATmega48__)      || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)    || \
    defined(__AVR_ATmega16__)      || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
			cbi(DDRD, 2);
#elif defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__))
			cbi(DDRD, 0);
#endif

			// 8-bit timer to count IR stuff
			TCCR0B |= _BV(CS02) | _BV(CS00);// CLK / 64 for 8mhz
			TCNT0 = 0;//reset the timer

			// we use external interrupt INT0 for the IR receiver
			EICRA |= _BV(ISC00);// interrupt on rising and falling edge of INT0
			EIMSK |= _BV(INT0);// enable INT0 interrupts

			ir_readptr = 0;
			ir_writeptr = 0;

			enable_onboard_led();

			// don't forget sei()
		}

		/*************************************************************************
		 * ir_get()
		 *
		 * blocks until we get the next IR value out of our buffer
		 * returns the value of the command
		 *************************************************************************/
		uint8_t ir_get(void)
		{
			uint8_t value;
			while( ir_writeptr == ir_readptr); /* block waiting for a value */
			value = ir_buffer[ir_readptr]; /* pull out a sample*/
			ir_readptr = (ir_readptr + 1) % IR_BUFFER_SIZE;
			return value;
		}

		/*************************************************************************
		 * ir_get_nonblock()
		 *
		 * returns the value of the command
		 * returns 255 if there is not a new value for us
		 *************************************************************************/
		uint8_t ir_get_nonblock(void)
		{
			if (ir_writeptr == ir_readptr) /* don't block waiting for a value */
			return 255;

			uint8_t value;
			value = ir_buffer[ir_readptr]; /* pull out a sample */
			ir_readptr = (ir_readptr + 1) % IR_BUFFER_SIZE;
			return value;
		}
#endif // end IR
#ifdef ENABLE_PWMSERVO
		/*************************************************************************
		 * pwmservo_init(pwmno)
		 * pwmno 1 & 2 are preferred b/c they provide higher resolution
		 * pwmno:
		 * 1 - OC1A (avr pin 15 PB1) - arduino digital pin 9
		 * 2 - OC1B (avr pin 16 PB2) - arduino digital pin 10
		 * 3 - OC2A (avr pin 17 PB3) - arduino digital pin 11
		 * 4 - OC2B (avr pin  5 PD3) - arduino digital pin 3
		 * 5 - OC0A (avr pin 12 PD6) - arduino digital pin 6  (conflicts with IR)
		 * 6 - OC0B (avr pin 11 PD5) - arduino digital pin 5  (conflicts with IR)
		 *************************************************************************/
		void pwmservo_init(uint8_t pwmno)
		{
			// FIXME: need to reserve pins and counters at compile time
			// FIXME: conflict w/ IR if it runs at 8MHz
			if (!pwmno || (pwmno > 6))///* invalid */
			return;

			if ((pwmno == 1) || (pwmno == 2))// /* TCNT1 */
			{
				if (pwmno == 1) {
#if defined(__AVR_ATmega16__)      || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
					sbi(DDRD, DDD5);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
					sbi(DDRB, DDB5);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
					sbi(DDRB, 1);
#endif
					OCR1A = SERVO_MID_POS16; ///* initial value */
					TCCR1A |= _BV(COM1A1);///* turn on PWM 1 */
				}
				if (pwmno == 2) {
#if defined(__AVR_ATmega16__)      || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
					sbi(DDRD, DDD4);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
					sbi(DDRB, DDB6);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
					sbi(DDRB, 2);
#endif
					OCR1B = SERVO_MID_POS16;
					TCCR1A |= _BV(COM1B1); ///* turn on PWM 2 */
				}
				TCCR1A |= _BV(WGM11); ///* PWM phase correct */
				TCCR1B |= _BV(WGM13) | _BV(WGM12);///* PWM phase correct */
				TCCR1B |= _BV(CS11);
#if (F_CPU == 16000000)
				ICR1 = 0x9C3F; ///* 16bit */
#endif
#if (F_CPU == 8000000)
				ICR1 = 0x4E1F; ///* 16bit */
#endif
				TCNT1H = 0; ///* initial value */
				TCNT1L = 0;///* initial value */
			}

			if ((pwmno == 3) || (pwmno == 4)) // /* TCNT1 */
			{
				if (pwmno == 3) {
#if defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__) // Sanguino
					sbi(DDRD, DDD7);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
					sbi(DDRB, DDB4);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
					sbi(DDRB, 3);
#endif
					OCR2A = SERVO_MID_POS8; // /* initial value */
					TCCR2A |= _BV(COM2A1);// /* turn on PWM 1 */
				}
				if (pwmno == 4) {
#if defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__) // Sanguino
					sbi(DDRD, DDD6);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
					sbi(DDRH, DDH6);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
					sbi(DDRD, 3);
#endif
					OCR2B = SERVO_MID_POS8;
					TCCR2A |= _BV(COM2B1); // /* turn on PWM 2 */
				}
#if (F_CPU == 16000000)
				TCCR2A |= _BV(WGM20) | _BV(WGM21); // /* fast PWM */
#endif
#if (F_CPU == 8000000)
				TCCR2A |= _BV(WGM20); ///* fast PWM */
#endif
				TCCR2B |= _BV(CS20) | _BV(CS21) | _BV(CS22);
				TCNT2 = 0; ///* initial value */
			}
#if !defined(ENABLE_IR) && !defined(ENABLE_MILLIS)
			// define the next two channels (5 & 6)
#endif
		}

		void __pwmservo_set(uint8_t servo, uint16_t pwmval)
		{
			//printf("__pwmservo_set setting %d\n\r", pwmval);
			if (servo == 1)
			OCR1A = pwmval;
			else if (servo == 2)
			OCR1B = pwmval;
			else if (servo == 3)
			OCR2A = pwmval;
			else if (servo == 4)
			OCR2B = pwmval;
		}

		/***************************************************************************
		 * pwmservo_set
		 * servo - 1-6 (servo number that we already called init on)
		 * pwmval - from 0-255 - provides the position for the servo
		 *          0 makes a 1.5ms pulse
		 *          255 makes a 2.5ms pulse
		 ***************************************************************************/
		void pwmservo_set(uint8_t servo, uint8_t pwmval)
		{
			//printf("pwmservo_set setting %d to %d\n\r", servo, pwmval);

			if ((servo == 1) || (servo == 2))
			__pwmservo_set(
					servo,
					(((uint32_t)pwmval*((uint32_t)SERVO_MAX_POS16-(uint32_t)SERVO_MIN_POS16))/(uint32_t)256) + (uint32_t)SERVO_MIN_POS16
			);

			if ((servo == 3) || (servo == 4))
			__pwmservo_set(
					servo,
					(((uint32_t)pwmval*((uint32_t)SERVO_MAX_POS8-(uint32_t)SERVO_MIN_POS8))/(uint32_t)256) + (uint32_t)SERVO_MIN_POS8
			);
		}

		/* not tested yet
		 void pwmservo_setf(uint8_t servo, float pwmval)
		 {
		 if (servo == 1)
		 OCR1A = pwmval * 255.0;
		 else if (servo == 2)
		 OCR1B = pwmval * 255.0;
		 } */
#endif //end PWM SERVO
#ifdef ENABLE_PWM
		/*************************************************************************
		 * pwm_init(pwmno)
		 * pwmno:
		 * 1 - OC1A (avr pin 15 PB1) - arduino digital pin 9
		 * 2 - OC1B (avr pin 16 PB2) - arduino digital pin 10
		 * 3 - OC2A (avr pin 17 PB3) - arduino digital pin 11
		 * 4 - OC2B (avr pin  5 PD3) - arduino digital pin 3
		 * 5 - OC0A (avr pin 12 PD6) - arduino digital pin 6  (conflicts with IR)
		 * 6 - OC0B (avr pin 11 PD5) - arduino digital pin 5  (conflicts with IR)
		 *************************************************************************/
		void pwm_init(uint8_t pwmno)
		{
			// FIXME: need to reserve pins and counters at compile time
			if (!pwmno || (pwmno > 6))// invalid
			return;

			if ((pwmno == 1) || (pwmno == 2))// /* TCNT1 */
			{
				if (pwmno == 1) {
#if defined(__AVR_ATmega16__)      || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
					sbi(DDRD, DDD5);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
					sbi(DDRB, DDB5);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
					sbi(DDRB, 1);
#endif
					OCR1A = 0; ///* initial value */
					TCCR1A |= _BV(COM1A1);// /* turn on PWM 1 */
				}
				if (pwmno == 2) {
#if defined(__AVR_ATmega16__)      || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
					sbi(DDRD, DDD4);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
					sbi(DDRB, DDB6);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
					sbi(DDRB, 2);
#endif
					OCR1B = 0;
					TCCR1A |= _BV(COM1B1); ///* turn on PWM 2 */
				}
				TCCR1A |= _BV(WGM10); ///* PWM 8bit */
				//TCCR1B |= _BV(WGM13) | _BV(WGM12);// /* PWM phase correct */
				TCCR1B |= _BV(CS12);// /* div by 256 */
				TCNT1H = 0;///* initial value */
				TCNT1L = 0;///* initial value */
			}

			if ((pwmno == 3) || (pwmno == 4)) ///* TCNT1 */
			{
				if (pwmno == 3) {
#if defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__) // Sanguino
					sbi(DDRD, DDD7);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
					sbi(DDRB, DDB4);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
					sbi(DDRB, 3);
#endif
					OCR2A = 0; ///* initial value */
					TCCR2A |= _BV(COM2A1);///* turn on PWM 1 */
				}
				if (pwmno == 4) {
#if defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__) // Sanguino
					sbi(DDRD, DDD6);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
					sbi(DDRH, DDH6);
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
					sbi(DDRD, 3);
#endif
					OCR2B = 0;
					TCCR2A |= _BV(COM2B1); ///* turn on PWM 2 */
				}
				TCCR2A |= _BV(WGM20); ///* PWM 8bit */
				TCCR2B |= _BV(CS21) | _BV(CS22);// /* div by 256 for ovfl 244hz*/
				TCNT2 = 0;///* initial value */
			}
#if !defined(ENABLE_IR) && !defined(ENABLE_MILLIS)
			// define the next two channels (5 & 6)
#endif
		}

		/***************************************************************************
		 * pwm_set
		 * pwmchan - 1-6 (servo number that we already called init on)
		 * pwmval - from 0-255 - provides the position for the servo
		 *          0 makes a 1.5ms pulse
		 *          255 makes a 2.5ms pulse
		 ***************************************************************************/
		void pwm_set(uint8_t pwmchan, uint8_t pwmval)
		{
			//printf("__pwm_set setting %d\n\r", pwmval);
			if (pwmchan == 1)
			OCR1A = pwmval;
			else if (pwmchan == 2)
			OCR1B = pwmval;
			else if (pwmchan == 3)
			OCR2A = pwmval;
			else if (pwmchan == 4)
			OCR2B = pwmval;
		}

		/* not tested yet
		 void pwm_setf(uint8_t pwmchan, float pwmval)
		 {
		 if (pwmchan == 1)
		 OCR1A = pwmval * 255.0;
		 else if (servo == 2)
		 OCR1B = pwmval * 255.0;
		 } */
#endif

#ifdef ENABLE_ADC
		/***************************************************************************
		 * adc_init()
		 *
		 * gets our ADC ready to take 10bit samples. See atmegaclib.h for
		 * ADC reference and prescaler definitions.
		 ***************************************************************************/
		void adc_init(uint8_t adc_reference, uint8_t adc_prescaler)
		{
			///* initialize the ADC - 10bit mode */
			ADMUX |= (adc_reference << 6);
			ADCSRA |= _BV(ADEN);// for now we don't do this in the ISR | _BV(ADIE);
			ADCSRA |= (adc_prescaler & 7);//adc prescaler
			//Powering ADC peripheral
#ifdef PRR0
			PRR0 &= ~_BV(PRADC);
#elif defined(PRR)
			PRR &= ~_BV(PRADC);
#endif

		}

		/***************************************************************************
		 * adc_get()
		 *
		 * adcnum - specifies which ADC pin you want to read from
		 *        - 0 through 5 corresponds to PC0-PC5 (32 pin SMD and 28 pin DIP capsule) and
		 *        - 0 through 7 corresponds to PA0-PA7 (44 pin SMD and 40 pin DIP capsule)
		 *        - 0 through 15 is for ATmega1280/2560
		 *
		 * returns 16 bit unsigned value between 0 and 1024 with 0 meaning 0v and
		 *          1024 meaning at or above voltage on AREF pin
		 ***************************************************************************/
		uint16_t adc_get(uint8_t adcnum)
		{
			static uint8_t current_adcnum = 17; // high enough to avoid conflict if atmega1280 support...
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
			adcnum &= 15;
#elif(defined(__AVR_ATmega16__)  || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__))
			adcnum &= 7;
#elif defined(__AVR_ATmega48__)  || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
			adcnum &= 5;
#endif
#if defined(ADCSRB) && defined(MUX5)
			// the MUX5 bit of ADCSRB selects whether we're reading from channels
			// 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
			ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((adcnum >> 3) & 0x01) << MUX5);
#endif
			if (adcnum != current_adcnum) {
				ADMUX = (ADMUX & 0xF0) | adcnum; /* set up mux */
				current_adcnum = adcnum; /* cache for next time */
			}
			ADCSRA |= (1 << ADSC); /* start ADC conversion */
			while (ADCSRA & (1 << ADSC)) {;} /* block for the result */
			return ADC;
		}

		void adc_poweroff_digital_pinbuffer(uint8_t adcnum)
		{
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
			adcnum &= 15;
#elif(defined(__AVR_ATmega16__)    || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__))
			adcnum &= 7;
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)
			adcnum &= 5;
#endif
#if defined(DIDR0)
			sbi(DIDR0, adcnum);
#endif
#if defined(DIDR2)
			sbi(DIDR2, (adcnum & 7));
#endif
		}
#endif // ENABLE_ADC
#ifdef ENABLE_LCD
		static void lcd_nibble( uint8_t d );
		static void lcd_byte( uint8_t d );

		uint8_t lcd_pos = LCD_LINE1;
		void lcd_init( void )
		{

			// set LCD DDR pins to 1 for output
			LCD_D4_DDR |= (1<<LCD_D4_PIN);
			LCD_D5_DDR |= (1<<LCD_D5_PIN);
			LCD_D6_DDR |= (1<<LCD_D6_PIN);
			LCD_D7_DDR |= (1<<LCD_D7_PIN);
			LCD_E_DDR |= (1<<LCD_E_PIN);
			LCD_RS_DDR |= (1<<LCD_RS_PIN);

			/*// set LCD DDR pins to 1 for output
			 sbi(LCD_D4_DDR,LCD_D4_PIN);
			 sbi(LCD_D5_DDR,LCD_D5_PIN);
			 sbi(LCD_D6_DDR,LCD_D6_PIN);
			 sbi(LCD_D7_DDR,LCD_D7_PIN);
			 sbi(LCD_E_DDR,LCD_E_PIN);
			 */sbi(LCD_RS_DDR,LCD_RS_PIN);
			/**/

// set the E and RS PORT pins to 0
			LCD_E_PORT &= ~(1<<LCD_E_PIN);
			LCD_RS_PORT &= ~(1<<LCD_RS_PIN);

			/*
			 // set the E and RS PORT pins to 0
			 cbi(LCD_E_PORT,LCD_E_PIN);
			 cbi(LCD_RS_PORT,LCD_RS_PIN);*//**/

			_delay_ms( 15 );
			lcd_nibble( 0x30 );
			_delay_ms( 4.1 );
			lcd_nibble( 0x30 );
			_delay_us( 100 );
			lcd_nibble( 0x30 );
			_delay_us( LCD_TIME_DAT );
			lcd_nibble( 0x20 );// 4 bit mode
			_delay_us( LCD_TIME_DAT );
#if LCD_LINE == 1
			lcd_command( 0x20 ); // 1 line
#else
			lcd_command( 0x28 ); // 2 lines 5*7
#endif
			lcd_command( 0x08 ); // display off
			lcd_command( 0x01 );// display clear
			lcd_command( 0x06 );// cursor increment
			lcd_command( 0x0C );// on, no cursor, no blink

			// Set initial display conditions
			_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;

			// Initialize to default text direction (for romance languages)
			_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
			// set the entry mode
			lcd_command(LCD_ENTRYMODESET | _displaymode);
		}

		void lcd_clear()
		{
			lcd_command(0x01);
		}

		void lcd_home()
		{
			lcd_set_cursor(0,0);
		}

		void lcd_putchar( uint8_t d )
		{
			sbi(LCD_RS_PORT,LCD_RS_PIN);

			lcd_byte( d );

			switch( ++lcd_pos ) {
				case LCD_LINE1 + LCD_COLUMN:
#ifdef LCD_LINE2
				d = LCD_LINE2;
				break;
				case LCD_LINE2 + LCD_COLUMN:
#ifdef LCD_LINE3
				d = LCD_LINE3;
				break;
				case LCD_LINE3 + LCD_COLUMN:
#ifdef LCD_LINE4
				d = LCD_LINE4;
				break;
				case LCD_LINE4 + LCD_COLUMN:
#endif
#endif
#endif
				d = LCD_LINE1;
				break;
				default:
				return;
			}
			lcd_command( d );
		}

		void lcd_putstr(uint8_t *s ) // display string from SRAM
		{
			for( uint8_t *s1 = s; *s1; s1++ ) // until zero byte
			lcd_putchar((uint8_t) *s1 );
		}

		void lcd_putstr_f(const uint8_t *FlashString)
		{
			uint8_t i = 0;
			// Check for '\0' string terminator or maximum LCD width
			while(pgm_read_byte(&FlashString[i]) && (i < LCD_COLUMN))
			{
				lcd_putchar(pgm_read_byte(&FlashString[i++]));
			}
		}

		void lcd_putint(int value, uint8_t radix)
		{
			uint8_t string[18];
			itoa(value, (char *)string, radix);
			lcd_putstr(string);
		}

		void lcd_putU08(uint8_t value)
		{
			uint8_t s[4];
			byte2dec(value,s);
			lcd_putstr(s);
		}

		void lcd_puthexU08(uint8_t value)
		{
			uint8_t s[3];
			byte2hex(value,s);
			lcd_putstr(s);
		}

		void lcd_puthexU16(uint16_t value)
		{
			uint8_t s[5];
			word2hex(value,s);
			lcd_putstr(s);
		}

		void lcd_blank( uint8_t len ) // blank n digits
		{
			while( len-- )
			lcd_putchar( ' ' );
		}

		void lcd_cursor_on(void)
		{
			_displaycontrol |= LCD_CURSORON;
			lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
		}
		void lcd_cursor_off(void)
		{
			_displaycontrol &= ~LCD_CURSORON;
			lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
		}

		void lcd_blink_on(void)
		{
			_displaycontrol |= LCD_BLINKON;
			lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
		}

		void lcd_blink_off(void)
		{
			_displaycontrol &= ~LCD_BLINKON;
			lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);

		}
		void lcd_display_on(void)
		{
			_displaycontrol |= LCD_DISPLAYON;
			lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);

		}
		void lcd_display_off(void)
		{
			_displaycontrol &= ~LCD_DISPLAYON;
			lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);

		}

// Private functions
		static void lcd_nibble( uint8_t d )
		{

			cbi(LCD_D7_PORT,LCD_D7_PIN);
			if( d & 1<<7 ) sbi(LCD_D7_PORT,LCD_D7_PIN);
			cbi(LCD_D6_PORT,LCD_D6_PIN);
			if( d & 1<<6 ) sbi(LCD_D6_PORT,LCD_D6_PIN);
			cbi(LCD_D5_PORT,LCD_D5_PIN);
			if( d & 1<<5 ) sbi(LCD_D5_PORT,LCD_D5_PIN);
			cbi(LCD_D4_PORT,LCD_D4_PIN);
			if( d & 1<<4 ) sbi(LCD_D4_PORT,LCD_D4_PIN);

			sbi(LCD_E_PORT,LCD_E_PIN);
			_delay_us( LCD_TIME_ENA );
			cbi(LCD_E_PORT,LCD_E_PIN);
		}

		static void lcd_byte( uint8_t d )
		{
			lcd_nibble( d );
			lcd_nibble( d<<4 );
			_delay_us( LCD_TIME_DAT );
		}

		void lcd_command( uint8_t d )
		{
			cbi(LCD_RS_PORT,LCD_RS_PIN);
			lcd_byte( d );
			switch( d ) {
				case 0 ... 3: // on longer commands
				_delay_us( LCD_TIME_CLR );
				d = LCD_LINE1;
				case 0x80 ... 0xFF:// set position
				lcd_pos = d;
				break;
			}
		}

#endif //ENABLE_LCD
#ifdef ENABLE_7SEG
		// Define the segments
#define SEG_A 0b00000001
#define SEG_B 0b00000010
#define SEG_C 0b00000100
#define SEG_D 0b00001000
#define SEG_E 0b00010000
#define SEG_F 0b00100000
#define SEG_G 0b01000000
#ifdef SEG_DP_PORT
#define SEG_DP 0b10000000
#endif

#ifdef SEG_COMMON_ANODE
		const uint8_t seg_mask = 0xff;
#else
		const uint8_t seg_mask = 0x00;
#endif

		const uint8_t seg_code[]= {
			// index 0 is character 0
			SEG_A+SEG_B+SEG_C+SEG_D+SEG_E+SEG_F,
			// index 1 is character 1
			SEG_B+SEG_C,
			// index 2 is character 2
			SEG_A+SEG_B+SEG_D+SEG_E+SEG_G,
			// index 3 is character 3
			SEG_A+SEG_B+SEG_C+SEG_D+SEG_G,
			// index 4 is character 4
			SEG_F+SEG_G+SEG_B+SEG_C,
			// index 5 is character 5
			SEG_A+SEG_C+SEG_D+SEG_F+SEG_G,
			// index 6 is character 6
			SEG_A+SEG_C+SEG_D+SEG_E+SEG_F+SEG_G,
			// index 7 is character 7
			SEG_A+SEG_B+SEG_C,
			// index 8 is character 8
			SEG_A+SEG_B+SEG_C+SEG_D+SEG_E+SEG_F+SEG_G,
			// index 9 is character 9
			SEG_A+SEG_B+SEG_C+SEG_D+SEG_F+SEG_G,
			// index 10 is character A
			SEG_A+SEG_B+SEG_C+SEG_E+SEG_F+SEG_G,
			// index 11 is character b
			SEG_C+SEG_D+SEG_E+SEG_F+SEG_G,
			// index 12 is character C
			SEG_A+SEG_D+SEG_E+SEG_F,
			// index 13 is character d
			SEG_B+SEG_C+SEG_D+SEG_E+SEG_G,
			// index 14 is character E
			SEG_A+SEG_D+SEG_E+SEG_F+SEG_G,
			// index 15 is character F
			SEG_A+SEG_E+SEG_F+SEG_G,
			// index 16 is character S
			SEG_A+SEG_F+SEG_G+SEG_C+SEG_D,
			// index 17 is character c
			SEG_G+SEG_E+SEG_D,
			// index 18 is character r
			SEG_G+SEG_E,
			// index 19 is character H
			SEG_F+SEG_E+SEG_G+SEG_B+SEG_C,
			// index 20 is character i
			SEG_C,
			// index 21 is character L
			SEG_F+SEG_E+SEG_D,
			// index 22 is character o
			SEG_G+SEG_C+SEG_D+SEG_E,
			// index 23 is character P
			SEG_A+SEG_B+SEG_G+SEG_F+SEG_E,
			// index 24 is character U
			SEG_F+SEG_E+SEG_D+SEG_C+SEG_B,
			// index 25 is character u
			SEG_E+SEG_D+SEG_C,
			// index 26 is character h
			SEG_F+SEG_E+SEG_G+SEG_C,
			// index 27 is character Y
			SEG_F+SEG_G+SEG_B+SEG_C+SEG_D,
			// index 28 is character J
			SEG_B+SEG_C+SEG_D,
			// index 29 is character N
			SEG_E+SEG_F+SEG_A+SEG_B+SEG_C,
			// index 30 is character n
			SEG_C+SEG_G+SEG_E,
			// index 31 is character T
			SEG_A+SEG_F+SEG_E,
			// index 32 is character = (equal)
			SEG_G+SEG_D,
			// index 33 is character - (minus)
			SEG_G,
			// index 34 is character _ (underline)
			SEG_D,
			// index 35 is character G
			SEG_A+SEG_C+SEG_D+SEG_E+SEG_F,
			// index 36 is character space
			0
#ifdef SEG_DP_PORT
			// index 37 is character DP (dot) only if you enabled it in atmegaclib.h header...
			// Otherwise, your array ends at index 36.
			, SEG_DP
#endif
		};

		void seg_init(void)
		{
#ifdef SEG_DIGITS_8
			SEG_DIGITS_BUFFER[0] = 36;
			SEG_DIGITS_BUFFER[1] = 36;
			SEG_DIGITS_BUFFER[2] = 36;
			SEG_DIGITS_BUFFER[3] = 36;
			SEG_DIGITS_BUFFER[4] = 36;
			SEG_DIGITS_BUFFER[5] = 36;
			SEG_DIGITS_BUFFER[6] = 36;
			SEG_DIGITS_BUFFER[7] = 36;

#elif  defined(SEG_DIGITS_7)
			SEG_DIGITS_BUFFER[0] = 36;
			SEG_DIGITS_BUFFER[1] = 36;
			SEG_DIGITS_BUFFER[2] = 36;
			SEG_DIGITS_BUFFER[3] = 36;
			SEG_DIGITS_BUFFER[4] = 36;
			SEG_DIGITS_BUFFER[5] = 36;
			SEG_DIGITS_BUFFER[6] = 36;
#elif  defined(SEG_DIGITS_6)
			SEG_DIGITS_BUFFER[0] = 36;
			SEG_DIGITS_BUFFER[1] = 36;
			SEG_DIGITS_BUFFER[2] = 36;
			SEG_DIGITS_BUFFER[3] = 36;
			SEG_DIGITS_BUFFER[4] = 36;
			SEG_DIGITS_BUFFER[5] = 36;
#elif  defined(SEG_DIGITS_5)
			SEG_DIGITS_BUFFER[0] = 36;
			SEG_DIGITS_BUFFER[1] = 36;
			SEG_DIGITS_BUFFER[2] = 36;
			SEG_DIGITS_BUFFER[3] = 36;
			SEG_DIGITS_BUFFER[4] = 36;
#elif  defined(SEG_DIGITS_4)
			SEG_DIGITS_BUFFER[0] = 36;
			SEG_DIGITS_BUFFER[1] = 36;
			SEG_DIGITS_BUFFER[2] = 36;
			SEG_DIGITS_BUFFER[3] = 36;
#elif  defined(SEG_DIGITS_3)
			SEG_DIGITS_BUFFER[0] = 36;
			SEG_DIGITS_BUFFER[1] = 36;
			SEG_DIGITS_BUFFER[2] = 36;
#elif  defined(SEG_DIGITS_2)
			SEG_DIGITS_BUFFER[0] = 36;
			SEG_DIGITS_BUFFER[1] = 36;
#else
			SEG_DIGITS_BUFFER[0] = 36;
#endif
			// set direction for segment pins
#ifdef SEG_DP_DDR
			sbi(SEG_DP_DDR, SEG_DP_PIN);
#endif
			sbi(SEG_A_DDR, SEG_A_PIN);
			sbi(SEG_B_DDR, SEG_B_PIN);
			sbi(SEG_C_DDR, SEG_C_PIN);
			sbi(SEG_D_DDR, SEG_D_PIN);
			sbi(SEG_E_DDR, SEG_E_PIN);
			sbi(SEG_F_DDR, SEG_F_PIN);
			sbi(SEG_G_DDR, SEG_G_PIN);

			// set direction for common pins
#ifdef SEG_COMM_0_DDR
			sbi(SEG_COMM_0_DDR, SEG_COMM_0_PIN);
#endif
#ifdef SEG_COMM_1_DDR
			sbi(SEG_COMM_1_DDR, SEG_COMM_1_PIN);
#endif
#ifdef SEG_COMM_2_DDR
			sbi(SEG_COMM_2_DDR, SEG_COMM_2_PIN);
#endif
#ifdef SEG_COMM_3_DDR
			sbi(SEG_COMM_3_DDR, SEG_COMM_3_PIN);
#endif
#ifdef SEG_COMM_4_DDR
			sbi(SEG_COMM_4_DDR, SEG_COMM_4_PIN);
#endif
#ifdef SEG_COMM_5_DDR
			sbi(SEG_COMM_5_DDR, SEG_COMM_5_PIN);
#endif
#ifdef SEG_COMM_6_DDR
			sbi(SEG_COMM_6_DDR, SEG_COMM_6_PIN);
#endif
#ifdef SEG_COMM_7_DDR
			sbi(SEG_COMM_7_DDR, SEG_COMM_7_PIN);
#endif

		}

		uint8_t seg_digit_from_array(uint8_t index)
		{
			return seg_mask ^ seg_code[index];
		}

		void seg_nibble(uint8_t segments)
		{
#ifdef SEG_DP_DDR
			cbi(SEG_DP_PORT,SEG_DP_PIN);
			if(bit_isset(segments,7)) sbi(SEG_DP_PORT,SEG_DP_PIN);
#endif
			cbi(SEG_G_PORT,SEG_G_PIN);
			if(bit_isset(segments,6)) sbi(SEG_G_PORT,SEG_G_PIN);
			cbi(SEG_F_PORT,SEG_F_PIN);
			if(bit_isset(segments,5)) sbi(SEG_F_PORT,SEG_F_PIN);
			cbi(SEG_E_PORT,SEG_E_PIN);
			if(bit_isset(segments,4)) sbi(SEG_E_PORT,SEG_E_PIN);
			cbi(SEG_D_PORT,SEG_D_PIN);
			if(bit_isset(segments,3)) sbi(SEG_D_PORT,SEG_D_PIN);
			cbi(SEG_C_PORT,SEG_C_PIN);
			if(bit_isset(segments,2)) sbi(SEG_C_PORT,SEG_C_PIN);
			cbi(SEG_B_PORT,SEG_B_PIN);
			if(bit_isset(segments,1)) sbi(SEG_B_PORT,SEG_B_PIN);
			cbi(SEG_A_PORT,SEG_A_PIN);
			if(bit_isset(segments,0)) sbi(SEG_A_PORT,SEG_A_PIN);
		}

		void seg_common_select(uint8_t digit)
		{
#ifdef SEG_COMM_7_PORT
			cbi(SEG_COMM_7_PORT,SEG_COMM_7_PIN);
			if(bit_isset(digit,7)) sbi(SEG_COMM_7_PORT,SEG_COMM_7_PIN);
#endif
#ifdef SEG_COMM_6_PORT
			cbi(SEG_COMM_6_PORT,SEG_COMM_6_PIN);
			if(bit_isset(digit,6)) sbi(SEG_COMM_6_PORT,SEG_COMM_6_PIN);
#endif
#ifdef SEG_COMM_5_PORT
			cbi(SEG_COMM_5_PORT,SEG_COMM_5_PIN);
			if(bit_isset(digit,5)) sbi(SEG_COMM_5_PORT,SEG_COMM_5_PIN);
#endif
#ifdef SEG_COMM_4_PORT
			cbi(SEG_COMM_4_PORT,SEG_COMM_4_PIN);
			if(bit_isset(digit,4)) sbi(SEG_COMM_4_PORT,SEG_COMM_4_PIN);
#endif
#ifdef SEG_COMM_3_PORT
			cbi(SEG_COMM_3_PORT,SEG_COMM_3_PIN);
			if(bit_isset(digit,3)) sbi(SEG_COMM_3_PORT,SEG_COMM_3_PIN);
#endif
#ifdef SEG_COMM_2_PORT
			cbi(SEG_COMM_2_PORT,SEG_COMM_2_PIN);
			if(bit_isset(digit,2)) sbi(SEG_COMM_2_PORT,SEG_COMM_2_PIN);
#endif
#ifdef SEG_COMM_1_PORT
			cbi(SEG_COMM_1_PORT,SEG_COMM_1_PIN);
			if(bit_isset(digit,1)) sbi(SEG_COMM_1_PORT,SEG_COMM_1_PIN);
#endif
			// At least one pin must be set for selecting the digit
			// (minimum 1 digit, maximum 8 digits)
			cbi(SEG_COMM_0_PORT,SEG_COMM_0_PIN);
			if(bit_isset(digit,0)) sbi(SEG_COMM_0_PORT,SEG_COMM_0_PIN);
		}

		// Usage if common pin is active in 0 logic and you want to select the digit nr.4:
		// seg_select_digit(SELECT_DIGIT_FOUR, 0);
		void seg_select_digit(MyDigit digit, uint8_t active_logic)
		{
			uint8_t digit_mask;
			if (active_logic == 0) digit_mask = 0xFF;
			else digit_mask = 0x00;
			seg_common_select(digit_mask ^ digit);
		}

#endif

#ifdef ENABLE_I2C_SOFTWARE
		uint8_t I2C_write(uint8_t b)
		{
			uint8_t i;
			I2C_SDA_WR();
			for (i=0; i<8; i++)
			{
				if (b & 0x80)
				I2C_SDA_H();
				else
				I2C_SDA_L();
				_delay_us(10);
				I2C_SCL_H();
				_delay_us(10);
				I2C_SCL_L();
				b <<= 1;
			}
			I2C_SDA_RD();
			I2C_SDA_H();
			_delay_us(10);
			I2C_SCL_H();
			_delay_us(10);
			i=0xFF;
			do
			{
				if (bit_is_clear(I2C_PORT_I,I2C_SDA)) break;
				_delay_us(10);
			}
			while(--i>0);
			I2C_SCL_L();
			_delay_us(10);
			return(i);
		}

		uint8_t I2C_read(uint8_t ack)
		{
			uint8_t i;
			uint8_t b = 0;
			I2C_SDA_RD();
			I2C_SDA_H();
			_delay_us(10);
			for (i=0; i<8; i++)
			{
				I2C_SCL_H();
				_delay_us(10);
				b <<= 1;
				if (bit_is_set(I2C_PORT_I,I2C_SDA))
				b |= 1;
				I2C_SCL_L();
				_delay_us(10);
			}
			I2C_SDA_WR();
			if (ack == 0)
			I2C_SDA_L();
			else
			I2C_SDA_H();
			_delay_us(10);
			I2C_SCL_H();
			_delay_us(10);
			I2C_SCL_L();
			_delay_us(10);
			I2C_SDA_L();
			_delay_us(10);
			return(b);
		}

		void I2C_start(void)
		{
			I2C_SCL_H();
			I2C_SDA_H();
			I2C_SDA_WR();
			I2C_SCL_WR();
			_delay_us(10);
			I2C_SDA_L();
			_delay_us(10);
			I2C_SCL_L();
			_delay_us(10);
		}

		void I2C_stop(void)
		{
			I2C_SDA_WR(); // SDA na zapis
			I2C_SCL_H();
			_delay_us(10);
			I2C_SDA_H();
			_delay_us(10);
		}
#endif // end I2C software
#ifdef ENABLE_TWI

#define  TWI_START            0x08
#define  TWI_REP_START        0x10
#define  TWI_MT_SLA_ACK       0x18
#define  TWI_MT_SLA_NACK      0x20
#define  TWI_MT_DATA_ACK      0x28
#define  TWI_MT_DATA_NACK     0x30
#define  TWI_MR_SLA_ACK       0x40
#define  TWI_MR_SLA_NACK      0x48
#define  TWI_MR_DATA_ACK      0x50
#define  TWI_MR_DATA_NACK     0x58
#define  TWI_ARB_LOST         0x38

#define  TWI_ERROR_CODE   0x7e

		void TWI_init(void)
		{
			TWCR = 0x00; //disable twi
#if defined(TWPS0)
			TWSR = 0;
#endif
			TWBR = (F_CPU / TWI_FREQ - 16)/2;

			// enable twi module, acks, and twi interrupt
			//TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
		}

//*************************************************
//Function to start i2c communication
//*************************************************
		uint8_t TWI_start(void)
		{
			TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); //Send START condition
			//Wait for TWINT flag set. This indicates that the
			//  START condition has been transmitted
			while (!(TWCR & (1<<TWINT)));
			//Check value of TWI Status Register
			if ((TWSR & 0xF8) == TWI_START) return(0);
			else return(1);
		}

//*************************************************
//Function for repeat start condition
//*************************************************
		uint8_t TWI_repeatStart(void)
		{
			TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); //Send START condition
			//Wait for TWINT flag set. This indicates that the
			while (!(TWCR & (1<<TWINT)));
			//START condition has been transmitted
			//Check value of TWI Status Register
			if ((TWSR & 0xF8) == TWI_REP_START) return(0);
			else return(1);
		}

//**************************************************
//Function to transmit address of the slave
//*************************************************
		uint8_t TWI_sendAddress(uint8_t address)
		{
			uint8_t STATUS;

			if((address & 0x01) == 0) STATUS = TWI_MT_SLA_ACK;
			else STATUS = TWI_MR_SLA_ACK;

			TWDR = address;
			//Load SLA_W into TWDR Register. Clear TWINT bit
			//in TWCR to start transmission of address
			TWCR = (1<<TWINT)|(1<<TWEN);
			//Wait for TWINT flag set. This indicates that the SLA+W has been transmitted,
			// and ACK/NACK has been received.
			while (!(TWCR & (1<<TWINT)));
			//Check value of TWI Status Register
			if ((TWSR & 0xF8) == STATUS) return(0);
			else return(1);
		}

//**************************************************
//Function to transmit a data byte
//*************************************************
		uint8_t TWI_sendData(uint8_t data)
		{
			TWDR = data;
			//Load SLA_W into TWDR Register. Clear TWINT bit
			//in TWCR to start transmission of data
			TWCR = (1<<TWINT) |(1<<TWEN);
			//Wait for TWINT flag set. This indicates that the data has been
			// transmitted, and ACK/NACK has been received.
			while (!(TWCR & (1<<TWINT)));
			//Check value of TWI Status Register
			if ((TWSR & 0xF8) != TWI_MT_DATA_ACK) return(1);
			else return(0);
		}

//*****************************************************
//Function to receive a data byte and send ACKnowledge
//*****************************************************
		uint8_t TWI_receiveData_ACK(void)
		{
			uint8_t data;

			TWCR = (1<<TWEA)|(1<<TWINT)|(1<<TWEN);
			//Wait for TWINT flag set. This indicates that the data has been received
			while (!(TWCR & (1<<TWINT)));
			//Check value of TWI Status Register
			if ((TWSR & 0xF8) != TWI_MR_DATA_ACK) return(TWI_ERROR_CODE);
			data = TWDR;
			return(data);
		}

//******************************************************************
//Function to receive the last data byte (no acknowledge from master
//******************************************************************
		uint8_t TWI_receiveData_NACK(void)
		{
			uint8_t data;

			TWCR = (1<<TWINT)|(1<<TWEN);
			//Wait for TWINT flag set. This indicates that the data has been received
			while (!(TWCR & (1<<TWINT)));
			//Check value of TWI Status Register
			if ((TWSR & 0xF8) != TWI_MR_DATA_NACK) return(TWI_ERROR_CODE);
			data = TWDR;
			return(data);
		}

//**************************************************
//Function to end the i2c communication
//*************************************************
		void TWI_stop(void)
		{
			TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO); //Transmit STOP condition
		}
#endif // ENABLE_TWI

#ifdef ENABLE_PCF8583
		uint8_t PCF8583_read(uint8_t address)
		{
#if defined(PCF8583_USE_TWI)
			uint8_t error, a, b;
			//a=(PCF8583_A0<<1)|0xa0;
			b = PCF8583_A0;
			a = (b << 1) | Physical_Address;
			error = TWI_start();
			error = TWI_sendAddress(a);
			error = TWI_sendData(address);
			error = TWI_repeatStart();
			error = TWI_sendAddress(a|1);
			a = TWI_receiveData_NACK();
			TWI_stop();
			return a;
#else // use I2C Software
			uint8_t a, b;
			//a=(PCF8583_A0<<1)|0xa0;
			b = PCF8583_A0;
			a = (b << 1) | Physical_Address;
			I2C_start();
			I2C_write(a);
			I2C_write(address);
			I2C_start();
			I2C_write(a|1);
			a = I2C_read(1);
			I2C_stop();
			return a;
#endif
		}

		void PCF8583_write(uint8_t address,uint8_t data)
		{
#if defined(PCF8583_USE_TWI)
			uint8_t a, b, error;
			b = PCF8583_A0;
			a = (b << 1) | Physical_Address;
			error = TWI_start();
			error = TWI_sendAddress(a);
			error = TWI_sendData(address);
			error = TWI_sendData(data);
			TWI_stop();
#else // use I2C Software
			uint8_t a, b;
			b = PCF8583_A0;
			a = (b << 1) | Physical_Address;
			I2C_start();
			I2C_write(a);
			I2C_write(address);
			I2C_write(data);
			I2C_stop();
#endif
		}

		uint8_t PCF8583_read_bcd(uint8_t address)
		{
			return bcd2bin(PCF8583_read(address));
		}

		void PCF8583_write_bcd(uint8_t address,uint8_t data)
		{
			PCF8583_write(address,bin2bcd(data));
		}

		volatile uint8_t PCF8583_status;
		volatile uint8_t PCF8583_alarm;

		uint8_t PCF8583_get_status(void)
		{
			PCF8583_status=PCF8583_read(0);
			PCF8583_alarm=(PCF8583_status&2);
			return PCF8583_status;
		}

		void PCF8583_init(void)
		{
			PCF8583_status=0;
			PCF8583_alarm=0;
			PCF8583_write(0,0);
			PCF8583_write(4,PCF8583_read(4)&0x3f);
			PCF8583_write(8,0x90);
		}

		void PCF8583_stop(void)
		{
			PCF8583_get_status();
			PCF8583_status|=0x80;
			PCF8583_write(0,PCF8583_status);
		}

		void PCF8583_start(void)
		{
			PCF8583_get_status();
			PCF8583_status&=0x7f;
			PCF8583_write(0,PCF8583_status);
		}

		void PCF8583_hold_off(void)
		{
			PCF8583_get_status();
			PCF8583_status&=0xbf;
			PCF8583_write(0,PCF8583_status);
		}

		void PCF8583_hold_on(void)
		{
			PCF8583_get_status();
			PCF8583_status|=0x40;
			PCF8583_write(0,PCF8583_status);
		}

		void PCF8583_alarm_off(void)
		{
			PCF8583_get_status();
			PCF8583_status&=0xfb;
			PCF8583_write(0,PCF8583_status);
		}

		void PCF8583_alarm_on(void)
		{
			PCF8583_get_status();
			PCF8583_status|=4;
			PCF8583_write(0,PCF8583_status);
		}

		void PCF8583_write_word(uint8_t address,uint16_t data)
		{
			PCF8583_write(address,(uint8_t) data & 0xff);
			PCF8583_write(++address,(uint8_t)(data >> 8));
		}

		void PCF8583_write_date(uint8_t address,uint8_t day,uint16_t year)
		{
			PCF8583_write(address,bin2bcd(day)|(((uint8_t) year&3)<<6));
		}

		void PCF8583_get_time(uint8_t *hour,uint8_t *min,uint8_t *sec,uint8_t *hsec)
		{
			PCF8583_hold_on();
			*hsec=PCF8583_read_bcd(1);
			*sec=PCF8583_read_bcd(2);
			*min=PCF8583_read_bcd(3);
			*hour=PCF8583_read_bcd(4);
			PCF8583_hold_off();
		}

		void PCF8583_set_time(uint8_t hour,uint8_t min,uint8_t sec,uint8_t hsec)
		{
//  if (hour>23) hour=0;
//  if (min>59) min=0;
//  if (sec>59) sec=0;
//  if (hsec>100) hsec=0;
			PCF8583_stop();
			PCF8583_write_bcd(1,hsec);
			PCF8583_write_bcd(2,sec);
			PCF8583_write_bcd(3,min);
			PCF8583_write_bcd(4,hour);
			PCF8583_start();
		}

		void PCF8583_get_date(uint8_t *day,uint8_t *month,uint16_t *year)
		{
			uint8_t dy;
			uint16_t y1;
			PCF8583_hold_on();
			dy=PCF8583_read(5);
			*month=bcd2bin(PCF8583_read(6)&0x1f);
			PCF8583_hold_off();
			*day=bcd2bin(dy&0x3f);
			dy>>=6;
			y1=PCF8583_read(16)|((uint16_t) PCF8583_read(17)<<8);
			if (((uint8_t) y1&3)!=dy) PCF8583_write_word(16,++y1);
			*year=y1;
		}

		void PCF8583_set_date(uint8_t day,uint8_t month,uint16_t year)
		{
			PCF8583_write_word(16,year);
			PCF8583_stop();
			PCF8583_write_date(5,day,year);
			PCF8583_write_bcd(6,month);
			PCF8583_start();
		}

		void PCF8583_get_alarm_time(uint8_t *hour,uint8_t *min,uint8_t *sec,uint8_t *hsec)
		{
			*hsec=PCF8583_read_bcd(0x9);
			*sec=PCF8583_read_bcd(0xa);
			*min=PCF8583_read_bcd(0xb);
			*hour=PCF8583_read_bcd(0xc);
		}

		void PCF8583_set_alarm_time(uint8_t hour,uint8_t min,uint8_t sec,uint8_t hsec)
		{
			PCF8583_write_bcd(0x9,hsec);
			PCF8583_write_bcd(0xa,sec);
			PCF8583_write_bcd(0xb,min);
			PCF8583_write_bcd(0xc,hour);
		}

		void PCF8583_get_alarm_date(uint8_t *day,uint8_t *month)
		{
			*day=bcd2bin(PCF8583_read(0xd)&0x3f);
			*month=bcd2bin(PCF8583_read(0xe)&0x1f);
		}

		void PCF8583_set_alarm_date(uint8_t day,uint8_t month)
		{
			PCF8583_write_date(0xd,day,0);
			PCF8583_write_bcd(0xe,month);
		}
#endif //ENABLE_PCF8583

