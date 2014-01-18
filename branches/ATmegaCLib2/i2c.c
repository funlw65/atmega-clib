/* *****************************************************************************
 * i2c.c
 * See atmegaclib2.h header for the copyrights...
 * *****************************************************************************
 */
#ifndef F_CPU
#define F_CPU 16000000UL //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include "atmegaclib2.h"

//--
uint8_t I2C_write(uint8_t b) {
	uint8_t i;
	I2C_SDA_WR();
	for (i = 0; i < 8; i++) {
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
	i = 0xFF;
	do {
		if (bit_is_clear(I2C_PORT_I,I2C_SDA))
			break;
		_delay_us(10);
	} while (--i > 0);
	I2C_SCL_L();
	_delay_us(10);
	return (i);
}

uint8_t I2C_read(uint8_t ack) {
	uint8_t i;
	uint8_t b = 0;
	I2C_SDA_RD();
	I2C_SDA_H();
	_delay_us(10);
	for (i = 0; i < 8; i++) {
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
	return (b);
}

void I2C_start(void) {
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

void I2C_stop(void) {
	I2C_SDA_WR(); // SDA na zapis
	I2C_SCL_H();
	_delay_us(10);
	I2C_SDA_H();
	_delay_us(10);
}

