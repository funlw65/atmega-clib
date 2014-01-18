/* *****************************************************************************
 * twi.c
 * See atmegaclib2.h header for the copyrights...
 * *****************************************************************************
 */
#ifndef F_CPU
#define F_CPU 16000000UL //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include "atmegaclib2.h"

//--

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

void TWI_init(void) {
	TWCR = 0x00; //disable twi
#if defined(TWPS0)
	TWSR = 0;
#endif
	TWBR = (F_CPU / TWI_FREQ - 16) / 2;

	// enable twi module, acks, and twi interrupt
	//TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
}

//*************************************************
//Function to start i2c communication
//*************************************************
uint8_t TWI_start(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); //Send START condition
	//Wait for TWINT flag set. This indicates that the
	//  START condition has been transmitted
	while (!(TWCR & (1 << TWINT)))
		;
	//Check value of TWI Status Register
	if ((TWSR & 0xF8) == TWI_START)
		return (0);
	else
		return (1);
}

//*************************************************
//Function for repeat start condition
//*************************************************
uint8_t TWI_repeatStart(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); //Send START condition
	//Wait for TWINT flag set. This indicates that the
	while (!(TWCR & (1 << TWINT)))
		;
	//START condition has been transmitted
	//Check value of TWI Status Register
	if ((TWSR & 0xF8) == TWI_REP_START)
		return (0);
	else
		return (1);
}

//**************************************************
//Function to transmit address of the slave
//*************************************************
uint8_t TWI_sendAddress(uint8_t address) {
	uint8_t STATUS;

	if ((address & 0x01) == 0)
		STATUS = TWI_MT_SLA_ACK;
	else
		STATUS = TWI_MR_SLA_ACK;

	TWDR = address;
	//Load SLA_W into TWDR Register. Clear TWINT bit
	//in TWCR to start transmission of address
	TWCR = (1 << TWINT) | (1 << TWEN);
	//Wait for TWINT flag set. This indicates that the SLA+W has been transmitted,
	// and ACK/NACK has been received.
	while (!(TWCR & (1 << TWINT)))
		;
	//Check value of TWI Status Register
	if ((TWSR & 0xF8) == STATUS)
		return (0);
	else
		return (1);
}

//**************************************************
//Function to transmit a data byte
//*************************************************
uint8_t TWI_sendData(uint8_t data) {
	TWDR = data;
	//Load SLA_W into TWDR Register. Clear TWINT bit
	//in TWCR to start transmission of data
	TWCR = (1 << TWINT) | (1 << TWEN);
	//Wait for TWINT flag set. This indicates that the data has been
	// transmitted, and ACK/NACK has been received.
	while (!(TWCR & (1 << TWINT)))
		;
	//Check value of TWI Status Register
	if ((TWSR & 0xF8) != TWI_MT_DATA_ACK)
		return (1);
	else
		return (0);
}

//*****************************************************
//Function to receive a data byte and send ACKnowledge
//*****************************************************
uint8_t TWI_receiveData_ACK(void) {
	uint8_t data;

	TWCR = (1 << TWEA) | (1 << TWINT) | (1 << TWEN);
	//Wait for TWINT flag set. This indicates that the data has been received
	while (!(TWCR & (1 << TWINT)))
		;
	//Check value of TWI Status Register
	if ((TWSR & 0xF8) != TWI_MR_DATA_ACK)
		return (TWI_ERROR_CODE);
	data = TWDR;
	return (data);
}

//******************************************************************
//Function to receive the last data byte (no acknowledge from master
//******************************************************************
uint8_t TWI_receiveData_NACK(void) {
	uint8_t data;

	TWCR = (1 << TWINT) | (1 << TWEN);
	//Wait for TWINT flag set. This indicates that the data has been received
	while (!(TWCR & (1 << TWINT)))
		;
	//Check value of TWI Status Register
	if ((TWSR & 0xF8) != TWI_MR_DATA_NACK)
		return (TWI_ERROR_CODE);
	data = TWDR;
	return (data);
}

//**************************************************
//Function to end the i2c communication
//*************************************************
void TWI_stop(void) {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO); //Transmit STOP condition
}

