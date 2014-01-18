/* *****************************************************************************
 * serial_poll.c
 * See atmegaclib2.h header for the copyrights...
 * *****************************************************************************
 */
#ifndef F_CPU
#define F_CPU 16000000UL //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include "atmegaclib2.h"
//--
/* Initialize UART */
void serial_init(void) {
#if defined(USART_RXC_vect) // for ATmega16A
	/* Set the baud rate */
	UBRRH = (uint8_t) (UART_BAUD_SELECT >> 8);
	UBRRL = (uint8_t) UART_BAUD_SELECT;
	/* Enable UART receiver and transmitter */

	UCSRB = ((1 << RXEN) | (1 << TXEN));
	/* Set frame format: 8 data 2stop */
	UCSRC = (1 << USBS) | (1 << UCSZ1) | (1 << UCSZ0); //For devices with Extended IO
	//UCSR0C = (1<<URSEL)|(1<<USBS0)|(1<<UCSZ01)|(1<<UCSZ00);   //For devices without Extended IO
#else
	/* Set the baud rate */
	UBRR0H = (uint8_t) (UART_BAUD_SELECT >> 8);
	UBRR0L = (uint8_t) UART_BAUD_SELECT;
	/* Enable UART receiver and transmitter */UCSR0B = ((1 << RXEN0)
			| (1 << TXEN0));
	/* Set frame format: 8 data 2stop */
	UCSR0C = (1 << USBS0) | (1 << UCSZ01) | (1 << UCSZ00); //For devices with Extended IO
	//UCSR0C = (1<<URSEL)|(1<<USBS0)|(1<<UCSZ01)|(1<<UCSZ00);   //For devices without Extended IO
#endif
}

/* Read and write functions */
uint8_t serial_getchar(void) {
#if defined(USART_RXC_vect) // for ATmega16A
	/* Wait for incoming data */
	while (!(UCSRA & (1 << RXC)))
	;
	/* Return the data */
	return UDR;
#else
	/* Wait for incoming data */
	while (!(UCSR0A & (1 << RXC0)))
		;
	/* Return the data */
	return UDR0;
#endif
}



