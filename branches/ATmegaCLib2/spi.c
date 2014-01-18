/* *****************************************************************************
 * spi.c
 * See atmegaclib2.h header for the copyrights...
 * *****************************************************************************
 */
#ifndef F_CPU
#define F_CPU 16000000UL //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include "atmegaclib2.h"
//--
#ifdef ENABLE_SPI_INT

ISR(SPI_STC_vect) {
	SPI_TC = TRUE;
}
#endif

void SPI_master_init(void) {
	// Set SS to high so a connected chip will be "deselected" by default
	sbi(SS_PORT, SS);

	// When the SS pin is set as OUTPUT, it can be used as
	// a general purpose output port (it doesn't influence
	// SPI operations).
	sbi(SS_DDR, SS);

#ifdef ENABLE_SPI_INT
	SPI_TC = FALSE;
#endif
	// Warning: if the SS pin ever becomes a LOW INPUT then SPI
	// automatically switches to Slave, so the data direction of
	// the SS pin MUST be kept as OUTPUT.
	SPCR |= _BV(MSTR);
	SPCR |= _BV(SPE);

	// Set direction register for SCK and MOSI pin.
	// MISO pin automatically overrides to INPUT.
	// By doing this AFTER enabling SPI, we avoid accidentally
	// clocking in a single bit since the lines go directly
	// from "input" to SPI control.
	// http://code.google.com/p/arduino/issues/detail?id=888
	sbi(SCK_DDR, SCK);
	sbi(MOSI_DDR, MOSI);
}

void SPI_master_setBitOrder(uint8_t bitOrder) {
	if (bitOrder == LSBFIRST) {
		SPCR |= _BV(DORD);
	} else {
		SPCR &= ~(_BV(DORD));
	}
}

void SPI_master_setDataMode(uint8_t mode) {
	SPCR = (SPCR & ~SPI_MODE_MASK) | mode;
}

void SPI_master_setClockDivider(uint8_t rate) {
	SPCR = (SPCR & ~SPI_CLOCK_MASK) | (rate & SPI_CLOCK_MASK);
	SPSR = (SPSR & ~SPI_2XCLOCK_MASK) | ((rate >> 2) & SPI_2XCLOCK_MASK);
}

void SPI_attachInterrupt(void) {
	SPCR |= _BV(SPIE);
}

void SPI_detachInterrupt(void) {
	SPCR &= ~_BV(SPIE);
}

void SPI_master_stop(void) {
	SPCR &= ~_BV(SPE);
}

uint8_t SPI_master_transmit(uint8_t data) {
	uint8_t reply;
	// Start transmission
	SPDR = data;
	// Wait for transmission complete
#ifdef ENABLE_SPI_INT
	while(!SPI_TC)
	;
#else
	while (!(SPSR & (1 << SPIF)))
		;
#endif
#ifdef ENABLE_SPI_INT
	SPI_TC = FALSE;
#endif
	reply = SPDR;
	return reply;
}

uint8_t SPI_master_receive(void) {
	uint8_t data;
	// Start reception
	SPDR = 0xFF;
	// Wait for transmission complete
#ifdef ENABLE_SPI_INT
	while(!SPI_TC)
	;
#else
	while (!(SPSR & (1 << SPIF)))
		;
#endif
#ifdef ENABLE_SPI_INT
	SPI_TC = FALSE;
#endif
	data = SPDR;
	return data;
}

//the following function is required by (Arduino/ATmega)ISP programmer

uint8_t SPI_master_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
	uint8_t n;
	//int8_t error;
	//n = 0;
	SPI_master_transmit(a);
	n = SPI_master_transmit(b);
	//if (n != a) error = -1;
	n = SPI_master_transmit(c);
	return SPI_master_transmit(d);
}

