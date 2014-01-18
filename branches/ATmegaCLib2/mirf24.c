/* *****************************************************************************
 * mirf24.c
 * See atmegaclib2.h header for the copyrights...
 * *****************************************************************************
 */
#ifndef F_CPU
#define F_CPU 16000000UL //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include "atmegaclib2.h"
//--

void mirf_init()
// Initializes pins to communicate with the MiRF module
// Should be called in the early initializing phase at startup.
{
    sbi(mirf_CE_PORT, mirf_CE_PIN);
    sbi(mirf_CSN_PORT, mirf_CSN_PIN);

    mirf_ceLow;
    mirf_csnHi;

    // Initialize spi module
    SPI_master_init();
}

void mirf_configRegister(uint8_t reg, uint8_t value)
// Clocks only one byte into the given MiRF register
{
    mirf_csnLow;
    SPI_master_transmit(RF24_W_REGISTER | (RF24_REGISTER_MASK & reg));
    SPI_master_transmit(value);
    mirf_csnHi;
}

void mirf_powerUpRx(){
	mirf_PTX = 0;
	mirf_ceLow;
	mirf_configRegister(RF24_CONFIG, mirf_CONFIG | ( (1<<RF24_PWR_UP) | (1<<RF24_PRIM_RX) ) );
	mirf_ceHi;
	mirf_configRegister(RF24_STATUS,(1 << RF24_TX_DS) | (1 << RF24_MAX_RT));
}

void mirf_flushRx(){
	mirf_csnLow;
    SPI_master_transmit( RF24_FLUSH_RX );
    mirf_csnHi;
}

void mirf_powerUpTx(){
	mirf_PTX = 1;
	mirf_configRegister(RF24_CONFIG, mirf_CONFIG | ( (1<<RF24_PWR_UP) | (0<<RF24_PRIM_RX) ) );
}

void mirf_config()
// Sets the important registers in the MiRF module and powers the module
// in receiving mode
// NB: channel and payload must be set now.
{
    // Set RF channel
	mirf_configRegister(RF24_RF_CH,mirf_CHANNEL);

    // Set length of incoming payload
	mirf_configRegister(RF24_RX_PW_P0, mirf_PAYLOAD);
	mirf_configRegister(RF24_RX_PW_P1, mirf_PAYLOAD);

    // Start receiver
	mirf_powerUpRx();
	mirf_flushRx();
}

void mirf_transferSync(uint8_t *dataout,uint8_t *datain,uint8_t len){
	uint8_t i;
	for(i = 0;i < len;i++){
		datain[i] = SPI_master_transmit(dataout[i]);
	}
}

void mirf_transmitSync(uint8_t *dataout,uint8_t len){
	uint8_t i;
	for(i = 0;i < len;i++){
		SPI_master_transmit(dataout[i]);
	}
}


void mirf_readRegister(uint8_t reg, uint8_t * value, uint8_t len)
// Reads an array of bytes from the given start position in the MiRF registers.
{
	mirf_csnLow;
    SPI_master_transmit(RF24_R_REGISTER | (RF24_REGISTER_MASK & reg));
    mirf_transferSync(value,value,len);
    mirf_csnHi;
}

void mirf_writeRegister(uint8_t reg, uint8_t * value, uint8_t len)
// Writes an array of bytes into inte the MiRF registers.
{
	mirf_csnLow;
    SPI_master_transmit(RF24_W_REGISTER | (RF24_REGISTER_MASK & reg));
    mirf_transmitSync(value,len);
    mirf_csnHi;
}

void mirf_setRADDR(uint8_t * adr)
// Sets the receiving address
{
	mirf_ceLow;
	mirf_writeRegister(RF24_RX_ADDR_P1,adr,mirf_ADDR_LEN);
	mirf_ceHi;
}

void mirf_setTADDR(uint8_t * adr)
// Sets the transmitting address
{
	/*
	 * RX_ADDR_P0 must be set to the sending addr for auto ack to work.
	 */

	mirf_writeRegister(RF24_RX_ADDR_P0,adr,mirf_ADDR_LEN);
	mirf_writeRegister(RF24_TX_ADDR,adr,mirf_ADDR_LEN);
}

extern uint8_t mirf_rxFifoEmpty(){
	uint8_t fifoStatus;

	mirf_readRegister(RF24_FIFO_STATUS,&fifoStatus,sizeof(fifoStatus));
	return (fifoStatus & (1 << RF24_RX_EMPTY));
}

uint8_t mirf_getStatus(){
	uint8_t rv;
	mirf_readRegister(RF24_STATUS,&rv,1);
	return rv;
}

extern uint8_t mirf_dataReady()
// Checks if data is available for reading
{
    // See note in getData() function - just checking RX_DR isn't good enough
	uint8_t status = mirf_getStatus();

    // We can short circuit on RX_DR, but if it's not set, we still need
    // to check the FIFO for any pending packets
    if ( status & (1 << RF24_RX_DR) ) return 1;
    return !mirf_rxFifoEmpty();
}


extern void mirf_getData(uint8_t * data)
// Reads payload bytes into data array
{
	mirf_csnLow;                               // Pull down chip select
    SPI_master_transmit( RF24_R_RX_PAYLOAD );            // Send cmd to read rx payload
    mirf_transferSync(data,data,mirf_PAYLOAD); // Read payload
    mirf_csnHi;                               // Pull up chip select
    // NVI: per product spec, p 67, note c:
    //  "The RX_DR IRQ is asserted by a new packet arrival event. The procedure
    //  for handling this interrupt should be: 1) read payload through SPI,
    //  2) clear RX_DR IRQ, 3) read FIFO_STATUS to check if there are more
    //  payloads available in RX FIFO, 4) if there are more data in RX FIFO,
    //  repeat from step 1)."
    // So if we're going to clear RX_DR here, we need to check the RX FIFO
    // in the dataReady() function
    mirf_configRegister(RF24_STATUS,(1<<RF24_RX_DR));   // Reset status register
}

void mirf_send(uint8_t * value)
// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
{
    uint8_t status;
    status = mirf_getStatus();

    while (mirf_PTX) {
	    status = mirf_getStatus();

	    if((status & ((1 << RF24_TX_DS)  | (1 << RF24_MAX_RT)))){
	    	mirf_PTX = 0;
		    break;
	    }
    }                  // Wait until last paket is send

    mirf_ceLow;

    mirf_powerUpTx();       // Set to transmitter mode , Power up

    mirf_csnLow;                    // Pull down chip select
    SPI_master_transmit( RF24_FLUSH_TX );     // Write cmd to flush tx fifo
    mirf_csnHi;                    // Pull up chip select

    mirf_csnLow;                    // Pull down chip select
    SPI_master_transmit( RF24_W_TX_PAYLOAD ); // Write cmd to write payload
    mirf_transmitSync(value,mirf_PAYLOAD);   // Write payload
    mirf_csnHi;                    // Pull up chip select

    mirf_ceHi;                     // Start transmission
}

void mirf_powerDown(){
	mirf_ceLow;
	mirf_configRegister(RF24_CONFIG, mirf_CONFIG );
}



