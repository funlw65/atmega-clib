/* *****************************************************************************
 * pcf8583.c
 * See atmegaclib2.h header for the copyrights...
 * *****************************************************************************
 */
#ifndef F_CPU
#define F_CPU 16000000UL //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include "atmegaclib2.h"
//--
uint8_t PCF8583_read(uint8_t address) {
#if defined(PCF8583_USE_TWI)
	uint8_t error, a, b;
	//a=(PCF8583_A0<<1)|0xa0;
	b = PCF8583_A0;
	a = (b << 1) | Physical_Address;
	error = TWI_start();
	error = TWI_sendAddress(a);
	error = TWI_sendData(address);
	error = TWI_repeatStart();
	error = TWI_sendAddress(a | 1);
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
	I2C_write(a | 1);
	a = I2C_read(1);
	I2C_stop();
	return a;
#endif
}

void PCF8583_write(uint8_t address, uint8_t data) {
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

uint8_t PCF8583_read_bcd(uint8_t address) {
	return bcd2bin(PCF8583_read(address));
}

void PCF8583_write_bcd(uint8_t address, uint8_t data) {
	PCF8583_write(address, bin2bcd(data));
}

volatile uint8_t PCF8583_status;
volatile uint8_t PCF8583_alarm;

uint8_t PCF8583_get_status(void) {
	PCF8583_status = PCF8583_read(0);
	PCF8583_alarm = (PCF8583_status & 2);
	return PCF8583_status;
}

void PCF8583_init(void) {
	PCF8583_status = 0;
	PCF8583_alarm = 0;
	PCF8583_write(0, 0);
	PCF8583_write(4, PCF8583_read(4) & 0x3f);
	PCF8583_write(8, 0x90);
}

void PCF8583_stop(void) {
	PCF8583_get_status();
	PCF8583_status |= 0x80;
	PCF8583_write(0, PCF8583_status);
}

void PCF8583_start(void) {
	PCF8583_get_status();
	PCF8583_status &= 0x7f;
	PCF8583_write(0, PCF8583_status);
}

void PCF8583_hold_off(void) {
	PCF8583_get_status();
	PCF8583_status &= 0xbf;
	PCF8583_write(0, PCF8583_status);
}

void PCF8583_hold_on(void) {
	PCF8583_get_status();
	PCF8583_status |= 0x40;
	PCF8583_write(0, PCF8583_status);
}

void PCF8583_alarm_off(void) {
	PCF8583_get_status();
	PCF8583_status &= 0xfb;
	PCF8583_write(0, PCF8583_status);
}

void PCF8583_alarm_on(void) {
	PCF8583_get_status();
	PCF8583_status |= 4;
	PCF8583_write(0, PCF8583_status);
}

void PCF8583_write_word(uint8_t address, uint16_t data) {
	PCF8583_write(address, (uint8_t) data & 0xff);
	PCF8583_write(++address, (uint8_t) (data >> 8));
}

void PCF8583_write_date(uint8_t address, uint8_t day, uint16_t year) {
	PCF8583_write(address, bin2bcd(day) | (((uint8_t) year & 3) << 6));
}

void PCF8583_get_time(uint8_t *hour, uint8_t *min, uint8_t *sec, uint8_t *hsec) {
	PCF8583_hold_on();
	*hsec = PCF8583_read_bcd(1);
	*sec = PCF8583_read_bcd(2);
	*min = PCF8583_read_bcd(3);
	*hour = PCF8583_read_bcd(4);
	PCF8583_hold_off();
}

void PCF8583_set_time(uint8_t hour, uint8_t min, uint8_t sec, uint8_t hsec) {
//  if (hour>23) hour=0;
//  if (min>59) min=0;
//  if (sec>59) sec=0;
//  if (hsec>100) hsec=0;
	PCF8583_stop();
	PCF8583_write_bcd(1, hsec);
	PCF8583_write_bcd(2, sec);
	PCF8583_write_bcd(3, min);
	PCF8583_write_bcd(4, hour);
	PCF8583_start();
}

void PCF8583_get_date(uint8_t *day, uint8_t *month, uint16_t *year) {
	uint8_t dy;
	uint16_t y1;
	PCF8583_hold_on();
	dy = PCF8583_read(5);
	*month = bcd2bin(PCF8583_read(6) & 0x1f);
	PCF8583_hold_off();
	*day = bcd2bin(dy & 0x3f);
	dy >>= 6;
	y1 = PCF8583_read(16) | ((uint16_t) PCF8583_read(17) << 8);
	if (((uint8_t) y1 & 3) != dy)
		PCF8583_write_word(16, ++y1);
	*year = y1;
}

void PCF8583_set_date(uint8_t day, uint8_t month, uint16_t year) {
	PCF8583_write_word(16, year);
	PCF8583_stop();
	PCF8583_write_date(5, day, year);
	PCF8583_write_bcd(6, month);
	PCF8583_start();
}

void PCF8583_get_alarm_time(uint8_t *hour, uint8_t *min, uint8_t *sec,
		uint8_t *hsec) {
	*hsec = PCF8583_read_bcd(0x9);
	*sec = PCF8583_read_bcd(0xa);
	*min = PCF8583_read_bcd(0xb);
	*hour = PCF8583_read_bcd(0xc);
}

void PCF8583_set_alarm_time(uint8_t hour, uint8_t min, uint8_t sec,
		uint8_t hsec) {
	PCF8583_write_bcd(0x9, hsec);
	PCF8583_write_bcd(0xa, sec);
	PCF8583_write_bcd(0xb, min);
	PCF8583_write_bcd(0xc, hour);
}

void PCF8583_get_alarm_date(uint8_t *day, uint8_t *month) {
	*day = bcd2bin(PCF8583_read(0xd) & 0x3f);
	*month = bcd2bin(PCF8583_read(0xe) & 0x1f);
}

void PCF8583_set_alarm_date(uint8_t day, uint8_t month) {
	PCF8583_write_date(0xd, day, 0);
	PCF8583_write_bcd(0xe, month);
}

