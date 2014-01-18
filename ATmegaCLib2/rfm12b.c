/* *****************************************************************************
 * rfm12b.c
 * See atmegaclib2.h header for the copyrights...
 * *****************************************************************************
 */
#ifndef F_CPU
#define F_CPU 16000000UL //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include "atmegaclib2.h"

//--
//Setup a simple timeout
inline void timeout_init(void) {
	TCCR2B = 0; //disable the timer
	TCNT2 = 0;//start counting from 0
	TCCR2B = 7;//turn the timer on (prescaler 1024)
	TIFR2 = (1 << TOV2);//clear the overflow flag
}

//Test if the timeout expired
inline uint8_t timeout(void) {
	return (TIFR2 & (1 << TOV2)); //return non-zero if the timer overflowed
}

//Test if the module is ready for sending / receiving next byte
uint8_t rf12_is_ready(void) {
	//RF_SPI_LOW_SPEED;
	cbi(RF_CS_PORT, RF_CS_PIN);
	//enable the module
	_delay_us(1);//let it respond
	uint8_t r = bit_get(MISO_PORT, MISO);//read the SO line (first bit of status word)
	sbi(RF_CS_PORT, RF_CS_PIN);
	//disable the module
	return r;//return the value of the first bit
}

//Exchange a word (two bytes, big-endian) with the module
uint16_t rf12_trans(uint16_t to_send) {
	uint16_t received = 0; //buffer for data we are going to read
	//RF_SPI_LOW_SPEED;
	cbi(RF_CS_PORT, RF_CS_PIN);
	//enable the module
	SPDR = (to_send >> 8) & 0xFF;//send the upper byte
	while (!(SPSR & (1 << SPIF)))
	;//wait until the transmission is complete
	received = SPDR;//store received byte
	received <<= 8;//move it on its proper position
	SPDR = (0xFF & to_send);//send the lower byte
	while (!(SPSR & (1 << SPIF)))
	;//wait until the transmission is complete
	received |= SPDR;//store received byte
	sbi(RF_CS_PORT, RF_CS_PIN);
	//disable the module
	return received;//return the data from the module
}

//send one byte through the radio
void rf12_txbyte(uint8_t b) {
	while (!rf12_is_ready()) //wait while the module is not ready...
	if (timeout())//...if it is too long...
	return;//...abort the operation
	rf12_trans(0xB800 | b);//send the desired byte
}

//receive one byte through the radio
uint8_t rf12_rxbyte(void) {
	while (!rf12_is_ready()) //wait while the module is not ready...
	if (timeout())//...if it is too long...
	return 0;//...abort the operation
	return rf12_trans(0xB000);//read the byte from the receive FIFO
}

//adaptation to use the statements from rf12b_code.pdf
#define RFXX_WRT_CMD(x) rf12_trans(x)

//prepare the radio module
//you must execute spi_init() before this
void radio_config(void) {
#ifdef RED_LED_DDR
	// set the pin as output
	sbi(RED_LED_DDR, RED_LED_PIN);
#endif
#ifdef GREEN_LED_DDR
	// set the pin as output
	sbi(GREEN_LED_DDR, GREEN_LED_PIN);
#endif
	sbi(RF_CS_PORT, RF_CS_PIN); //disable RFM12B module
	RF_SPI_LOW_SPEED;// set the SPI at low speed
	_delay_ms(10);//wait a moment
	rf12_trans(0xFE00);//send the reset command
	_delay_ms(150);//wait for reset to complete

	//Example setup
#ifdef RF_FREQ_868MHz
	RFXX_WRT_CMD(0x80E7); //EL,EF,868band,12.0pF
#endif
#ifdef RF_FREQ_433MHz
	RFXX_WRT_CMD(0x80D8); //EL,EF,433band,12.5pF
#endif
	RFXX_WRT_CMD(0x8219); //!er,!ebb,!ET,ES,EX,!eb,!ew,DC
#ifdef RF_FREQ_868MHz
	RFXX_WRT_CMD(0xA67C); //868MHz
#endif
#ifdef RF_FREQ_433MHz
	RFXX_WRT_CMD(0xA640); //433MHz
#endif
	//RFXX_WRT_CMD(0xC611);//19.2kbps
	//RFXX_WRT_CMD(0xC623);//9.6kbps
	RFXX_WRT_CMD(0xC603);
	//115200 = 3, 4800bps = 48, 19200 = 12, 9600 = 23, 28800 = C, 38400 = 9
	RFXX_WRT_CMD(0x94A0);//VDI,FAST,134kHz,0dBm,-103dBm
	RFXX_WRT_CMD(0xC2AC);//AL,!ml,DIG,DQD4
	RFXX_WRT_CMD(0xCA81);//FIFO8,SYNC,!ff,DR
	RFXX_WRT_CMD(0xCED4);//SYNC=2DD4;
	RFXX_WRT_CMD(0xC483);//@PWR,NO RSTRIC,!st,!fi,OE,EN
	RFXX_WRT_CMD(0x9850);//!mp,90kHz,MAX OUT
	RFXX_WRT_CMD(0xE000);//NOT USE
	RFXX_WRT_CMD(0xC800);//NOT USE
	RFXX_WRT_CMD(0xC040);//1.66MHz,2.2V
}

//Send data packet through the radio
void radio_send(uint8_t volatile * buffer, uint8_t len) {
	timeout_init(); //setup the timeout timer
	rf12_trans(0x8238);//start transmitter
	rf12_txbyte(0xAA);//send the preamble, four times 0xAA
	rf12_txbyte(0xAA);
	rf12_txbyte(0xAA);
	rf12_txbyte(0xAA);
	rf12_txbyte(0x2D);//then the predefined sync words
	rf12_txbyte(0xD4);
	rf12_txbyte(0xC0);//and a secret 0xC0DE
	rf12_txbyte(0xDE);
	rf12_txbyte(len);//next the length of the data
	while (len--)
	rf12_txbyte(*buffer++);//and then the data itself
	rf12_txbyte(0x00);//finish the transmission with two dummy bytes
	rf12_txbyte(0x00);
	rf12_txbyte(0x00);
	rf12_txbyte(0x00);
	while (!rf12_is_ready() && !timeout())
	;//wait for the completion of the send operation
	rf12_trans(0x8208);//go to idle, disable the transmitter
}

//receive data packet through the radio
int16_t radio_rcv(uint8_t volatile * buffer, uint8_t max_len) {
	uint8_t len, i, timeout_counter;
	timeout_init(); //setup the timeout timer
	timeout_counter = 3;//after some timeouts the procedure will give-up
	while (1) {
		rf12_trans(0x8208); //send the module to the idle
		rf12_trans(0x82C8);//and restart as a receiver
		_delay_us(150);
		rf12_trans(0xCA81);//disable the FIFO, and...
		rf12_trans(0xCA83);//...enable again, just to clear it
		while (1)//wait for the transmission to start
		{
			if (timeout()) //if the timeout occurred...
			{
				if (!(timeout_counter--)) //count it, and if no more trials remain
				{
					rf12_trans(0x8208); //put the module to the idle state
					return -1;//and return an error code
				}
				timeout_init(); //setup the timer for the next measurement
			}
			if (rf12_is_ready())
			break; //proceed if the module captured some data
		}
		timeout_init(); //restart the timeout timer
		i = rf12_trans(0xB000);//retrieve the received byte
		if (i != 0xC0)
		continue;//test if its correct
		i = rf12_rxbyte();//try to receive the next byte
		if (i != 0xDE)
		continue;//test if its correct
		len = rf12_rxbyte();//try to receive the 'length' byte
		if (len > max_len)
		continue;//test if the passed buffer is large enough
		//if all the bytes received so far are correct, we may assume that the
		//transmission is not a "false positive", so the program will continue reception
		break;
	}
	i = len; //we re going to read 'len' bytes
	while (i--)//loop while there is anything more to read
	{
		*buffer++ = rf12_rxbyte(); //receive next byte, and advance write pointer
		if (timeout())//if a timeout occured
		{
			rf12_trans(0x8208); //stop receiving
			return -2;//and return error code
		}
	}
	rf12_trans(0x8208); //put the module to the idle state
	return len;//return packet length
}


