/* *****************************************************************************
 * serial.c
 * See atmegaclib2.h header for the copyrights...
 * *****************************************************************************
 */
#ifndef F_CPU
#define F_CPU 16000000UL //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include "atmegaclib2.h"
//--
//Interrupt based.
//
uint8_t uart_buffer[UART_BUFFER_SIZE];
volatile uint8_t uart_readptr;
volatile uint8_t uart_writeptr;

ISR(UART0_ISR_VECT)// see the header file...
{
	uart_buffer[uart_writeptr] = UART0_DATA;
	uart_writeptr = (uart_writeptr + 1) % UART_BUFFER_SIZE;
}

void serial_init(void) {
	uart_writeptr = 0;
	uart_readptr = 0;

	// set default baud rate
#if defined(USART_RXC_vect) // for ATmega16A
	UBRRH = UART_BAUD_SELECT >> 8;
	UBRRL = UART_BAUD_SELECT;

	// enable receive, transmit and enable receive interrupts
	UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);

	// don't forget sei()
#else
	UBRR0H = UART_BAUD_SELECT >> 8;
	UBRR0L = UART_BAUD_SELECT;

	// enable receive, transmit and enable receive interrupts
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

	// don't forget sei()
#endif
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



