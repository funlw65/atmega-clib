/* *****************************************************************************
 *  BSD License
 *  ATmega-CLib - a BSD library for using GNU toolchain mainly with EvB4.3 and
 *                EvB5.1 boards, but also Arduino, Sanguino, etc...
 *  Portions Copyright:
 *  - (c) 1998 Wouter van Ooijen, http://www.voti.nl/winkel/index.html
 *  - (c) 2004 Robert Krysztof, website's gone
 *  - (c) 2007 Stefan Engelke, http://www.tinkerer.eu/AVRLib/nRF24L01
 *  - (c) 2009 Michael Spiceland, https://code.google.com/p/libarduino/
 *  - (c) 2009 Joep Suijs, http://www.blogger.com/profile/06821529393453332522
 *  - (c) 2009 Vasile Surducan, http://vsurducan.blogspot.com/
 *  - (c) 2010 Bert van Dam, http://members.home.nl/b.vandam/lonely/index.html
 *  - (c) 2010 Paul Stoffregen, http://www.pjrc.com/teensy/td_libs_OneWire.html
 *  - (c) 2010 Chennai Dharmani, http://www.dharmanitech.com
 *  - (c) 2011 Joe Pardue, http://code.google.com/p/avrtoolbox/
 *  - (c) 2011 Martin Thomas, http://www.siwawi.arubi.uni-kl.de/avr-projects/
 *  - (c) 2011 PJRC.COM, LLC - Paul Stoffregen, http://www.pjrc.com/
 *  - (c) 2011 ChaN, http://elm-chan.org/fsw/strf/xprintf.html
 *  - (c) xxxx Aleksander Mielczarek, http://olek.tk/en/rfm12.php
 *  - (c) 2012 Hans-Gert Dahmen, http://www.das-labor.org/wiki/RFM12_library/en
 *  - (c) 2012 Peter Fuhrmann, http://www.das-labor.org/wiki/RFM12_library/en
 *  - (c) 2012 Soeren Heisrath, http://www.das-labor.org/wiki/RFM12_library/en
 *  - (c) 2012 Vasile Guta Ciucur, https://sites.google.com/site/funlw65/
 *  - (c) xxxx Fabian Maximilian Thiele, website's gone
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

#ifndef ATMEGACLIB_H
#define ATMEGACLIB_H
#include <inttypes.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include "nRF24L01.h"

// ============== USER ZONE --- DO WHATEVER SETTINGS YOU NEED ==================

// *****************************************************************************
// Enabling/disabling additional functionality
// *****************************************************************************
#define UART_BAUD_RATE     57600 // default is 57600
#define UART_BAUD_SELECT   (F_CPU / (UART_BAUD_RATE * 16L) - 1) //- don't touch
//#define ENABLE_SERIAL // Interrupt based, require CONVERSION, conflicts with SERIAL_POLL
#define ENABLE_SERIAL_POLL // require CONVERSION, conflicts with SERIAL
//#define ENABLE_PWMSERVO    // servo control (conflicts with regular pwm)
//#define ENABLE_PWM         // motor or led control (conflicts with pwmservo)
//#define ENABLE_IR          // infrared receiver, SONY protocol- it use TIMER0 and INT0
#define ENABLE_FREQMEASURE   // it can use one of TIMER1, TIMER3, TIMER4, TIMER5, affects PWM
//#define IR_DEBOUNCE        // uncomment to debounce IR with a delay
//#define ENABLE_ADC         // analog to digital converter
//#define ENABLE_TWI         // hardware I2C
//#define ENABLE_I2C_SOFTWARE // software I2C
#define ENABLE_CONVERSION    // useful for Serial, LCD and 7SEG Display
//#define ENABLE_XPRINTF       // Hmmm...
//#define ENABLE_PCF8583     // require CONVERSION and I2C/TWI
//#define ENABLE_ONE_WIRE    // one wire protocol
//#define ENABLE_DS18_2_ // Dallas temperature sensors, require ONE_WIRE
//#define ENABLE_NB_DELAYS // Non-blocking, slotted delays (instead of millis()) using Timer0
//#define ENABLE_MILLIS     // use TIMER0, conflicts with NB_DELAYS
//#define ENABLE_EXTINT // External Interrupt(INT0,INT1,INT2) - not implemented yet.
//#define ENABLE_PCINT   // Pin on Change Interrupt(PCINT) - not implemented yet.
//#define ENABLE_LCD         // require CONVERSION
//#define ENABLE_GLCD
//#define ENABLE_7SEG        // starting from one digit, up to eight digits.
//#define ENABLE_ISPPROG     // Use Arduino as ISP Programmer - require SPI, conflict SD_Card
//#define ENABLE_SPI         // hardware SPI (master)
//#define ENABLE_SPI_INT     // hardware SPI use Interrupts
//#define ENABLE_SD_CARD_DEBUG // SD_ and F32_ functions send info on serial console
//#define ENABLE_SD_CARD       // raw SD Card operations; require SPI
//#define ENABLE_FAT32         // require PCF8583, SPI and SD_CARD
//#define ENABLE_RFM12B      // radio comm.- uses TIMER2, requires SPI
//#define ENABLE_GPL_RFM12B //requires SPI, XPRINTF, conflicts with RFM12B and IR
//#define ENABLE_MIRF24     // requires SPI
//#define OPTIMIZE_SPEED
// *****************************************************************************
// End block of "enable/disable" features
// *****************************************************************************

//--
#ifdef ENABLE_XPRINTF
#define _USE_XFUNC_OUT	1	/* 1: Use output functions */
#define	_CR_CRLF		1	/* 1: Convert \n ==> \r\n in the output char */

#define _USE_XFUNC_IN	0	/* 1: Use input function */
#define	_LINE_ECHO		1	/* 1: Echo back input chars in xgets function */
#endif
//--
//Slotted delays
#ifdef ENABLE_NB_DELAYS
//the following define the number of non-blocking delays.
// set it to the required number of non-blocking delays you need.
#define DELAY_SLOTS  3 // the number of non-blocking delays needed by user
int16_t isr_countdowns[DELAY_SLOTS];
#endif

#ifdef ENABLE_ISPPROG
// DEFINE 3 LEDS INDICATORS FOR THE ISP PROGRAMMER
// TODO: To change the Heart Beat LED assignment because it must be a PWM channel!!!
#if defined(__AVR_ATmega16__)      || \
	defined(__AVR_ATmega16A__)     || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega32A__)     || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)
//pinMode(LED_PMODE, OUTPUT);
#define LED_HB    PD5 // LED HEART BEAT
#define LED_ERR   PC6 // LED ERROR
#define LED_PMODE PC7 // LED PROGRAMMING MODE
#define RESET     PB2
//--
#define LED_HB_PORT    PORTD
#define LED_ERR_PORT   PORTC
#define LED_PMODE_PORT PORTC
#define RESET_PORT     PORTB
//--
#define LED_HB_DDR    DDRD
#define LED_ERR_DDR   DDRC
#define LED_PMODE_DDR DDRC
#define RESET_DDR     DDRB
//--
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega1280
#define LED_HB    PC6 // LED HEART BEAT
#define LED_ERR   PD5 // LED ERROR
#define LED_PMODE PC7 // LED PROGRAMMING MODE
#define RESET     PB2
//--
#define LED_HB_PORT    PORTC
#define LED_ERR_PORT   PORTD
#define LED_PMODE_PORT PORTC
#define RESET_PORT     PORTB
//--
#define LED_HB_DDR    DDRC
#define LED_ERR_DDR   DDRD
#define LED_PMODE_DDR DDRC
#define RESET_DDR     DDRB
//--
#elif defined(__AVR_ATmega48__)    || \
    defined(__AVR_ATmega88__)      || \
    defined(__AVR_ATmega88P__)     || \
    defined(__AVR_ATmega168__)     || \
    defined(__AVR_ATmega168P__)    || \
    defined(__AVR_ATmega328P__)   // Arduino 28 pins
#define LED_HB    PB1 // LED HEART BEAT
#define LED_ERR   PB0 // LED ERROR
#define LED_PMODE PD7 // LED PROGRAMMING MODE
#define RESET     PB2
//--
#define LED_HB_PORT    PORTB
#define LED_ERR_PORT   PORTB
#define LED_PMODE_PORT PORTD
#define RESET_PORT     PORTB
//--
#define LED_HB_DDR    DDRB
#define LED_ERR_DDR   DDRB
#define LED_PMODE_DDR DDRD
#define RESET_DDR     DDRB
//--
#endif
#endif // ENABLE_ISPPROG
// Continue with user settings, and chose your values...
//------------------------
// I2C port and pin selection - at your choice.
//------------------------
#ifdef ENABLE_I2C_SOFTWARE
#define I2C_PORT PORTC	//port for I2C line (PORTC)
#define I2C_SDA  PC1 	//pin  for SDA line (PC1)
#define I2C_SCL  PC0	  //pin  for SCL line (PC0)
#endif

#ifdef ENABLE_TWI
#include <compat/twi.h>
// Note1: When you enable TWI, it disconnects from the normal I/O driver
// and uses the open drain automatically.
// Note2: The AVR TWI is byte-oriented and interrupt based...
#define TWI_STANDARD_SPEED  100000UL // - don't change
#define TWI_FAST_SPEED      400000UL // - don't change
// change TWI_FREQ according to your settings
#define TWI_FREQ TWI_STANDARD_SPEED
#endif //ENABLE_TWI
//--
#if defined(ENABLE_SERIAL)// interrupt based
#define UART_BUFFER_SIZE  64 // buffer size for USART RX (receiving)
// change it to your needs (16, 32, 64, 128)
#endif

// chose the timer if your microcontroller allows that.
#ifdef ENABLE_FREQMEASURE
// Arduino Uno, Duemilanove, LilyPad, Mini, Fio, etc
#if defined(__AVR_ATmega48__)    || \
	    defined(__AVR_ATmega88__)      || \
	    defined(__AVR_ATmega88P__)     || \
	    defined(__AVR_ATmega168__)     || \
	    defined(__AVR_ATmega168P__)    || \
	    defined(__AVR_ATmega328P__)
#define CAPTURE_USE_TIMER1       // ICP1 is pin 8 (PB0)
// Teensy 1.0
#elif defined(__AVR_AT90USB162__)
#define CAPTURE_USE_TIMER1       // ICP1 is pin 16
// Teensy 2.0
#elif defined(__AVR_ATmega32U4__)
// #define CAPTURE_USE_TIMER1    // ICP1 is pin 22
#define CAPTURE_USE_TIMER3       // ICP3 is pin 10
// Teensy++ 1.0 & 2.0
#elif defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__) || defined(__AVR_AT90USB1287__)
// #define CAPTURE_USE_TIMER1    // ICP1 is pin 4
#define CAPTURE_USE_TIMER3       // ICP3 is pin 17
// Sanguino
#elif defined(__AVR_ATmega16__)      || \
		defined(__AVR_ATmega16A__)     || \
	    defined(__AVR_ATmega164P__)    || \
	    defined(__AVR_ATmega32__)      || \
	    defined(__AVR_ATmega32A__)     || \
	    defined(__AVR_ATmega324P__)    || \
	    defined(__AVR_ATmega324PA__)   || \
	    defined(__AVR_ATmega644__)     || \
	    defined(__AVR_ATmega644P__)    || \
	    defined(__AVR_ATmega1284P__)
#define CAPTURE_USE_TIMER1       // ICP1 is pin 14 (PD6)
// Arduino Mega
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
// #define CAPTURE_USE_TIMER1    // ICP1 is not connected
// #define CAPTURE_USE_TIMER3    // ICP3 is not connected
#define CAPTURE_USE_TIMER4       // ICP4 is pin 49
// #define CAPTURE_USE_TIMER5    // ICP5 is pin 48

#else
#error "Unknown chip, please edit me with timer+counter definitions"

#endif
#endif //ENABLE_FREQMEASURE
//--
#ifdef ENABLE_SD_CARD
#ifndef ENABLE_SPI
#error "This needs SPI so, uncomment ENANBLE_SPI from the library header"
#endif
//define the pin used for CS (chip select) the SD Card
//it can be the SPI SS pin if you want (recommended if SD is the first SPI peripheral)
//default is set for atmega168p/328p (PB2) but it is an ongoing project,
//tested with different micro controllers so, the settings can vary from a
// SVN update to another...
#define SD_CS_DDR  DDRB
#define SD_CS_PORT PORTB
#define SD_CS_PIN  PB4
#endif //ENABLE_SD_CARD
//--
#ifdef ENABLE_FAT32
#ifndef ENABLE_SPI
#error "This needs SPI so, uncomment ENANBLE_SPI from the library header"
#endif
#ifndef ENABLE_SD_CARD
#error "This needs SD_CARD so, uncomment ENANBLE_SD_CARD from the library header"
#endif
#define MAX_STRING_SIZE     100 //defining the maximum size of the dataString
#endif //ENABLE_FAT32
//set 1 wire pin
#ifdef ENABLE_ONE_WIRE
// Define OW_ONE_BUS if only one 1-Wire-Bus is used
// in the application -> shorter code.
// If not defined make sure to call ow_set_bus() before using
// a bus. Runtime bus-select increases code size by around 300
// bytes so use OW_ONE_BUS if possible
#define OW_ONE_BUS

#ifdef OW_ONE_BUS
#define OW_PIN  PC5
#define OW_IN   PINC
#define OW_DDR  DDRC
#define OW_OUT  PORTC
#define OW_CONF_DELAYOFFSET 0

#else

#if ( F_CPU < 1843200 )
#warning | Experimental multi-bus-mode is not tested for
#warning | frequencies below 1,84MHz. Use OW_ONE_WIRE or
#warning | faster clock-source (i.e. internal 2MHz R/C-Osc.).
#endif
#define OW_CONF_CYCLESPERACCESS 13
#define OW_CONF_DELAYOFFSET ( (uint16_t)( ((OW_CONF_CYCLESPERACCESS) * 1000000L) / F_CPU ) )
#endif //OW_ONE_BUS
// Recovery time (T_Rec) minimum 1usec - increase for long lines
// 5 usecs is a value give in some Maxim AppNotes
// 30u secs seem to be reliable for longer lines
//#define OW_RECOVERY_TIME        5  // usec
//#define OW_RECOVERY_TIME       30 // usec
#define OW_RECOVERY_TIME         10 // usec
// Use AVR's internal pull-up resistor instead of external 4,7k resistor.
// Based on information from Sascha Schade. Experimental but worked in tests
// with one DS18B20 and one DS18S20 on a rather short bus (60cm), where both
// sensors have been parasite-powered.
#define OW_USE_INTERNAL_PULLUP    0  // 0=external, 1=internal
//
#ifdef ENABLE_DS18_2_
// DS18x20 EERPROM support disabled(0) or enabled(1) :
#define DS18X20_EEPROMSUPPORT     0
// decicelsius functions disabled(0) or enabled(1):
#define DS18X20_DECICELSIUS       1
// max. resolution functions disabled(0) or enabled(1):
#define DS18X20_MAX_RESOLUTION    0
// extended output via UART disabled(0) or enabled(1) :
#define DS18X20_VERBOSE           0 //require serial comm.
#endif //ENABLE_DS18_2_
#endif //ENABLE_ONE_WIRE
//
#ifdef ENABLE_LCD
// Define the specific ports and pins used for the LCD
#define LCD_D4_PORT PORTD
#define LCD_D4_DDR DDRD
#define LCD_D4_PIN PD4

#define LCD_D5_PORT PORTD
#define LCD_D5_DDR DDRD
#define LCD_D5_PIN PD5

#define LCD_D6_PORT PORTD
#define LCD_D6_DDR DDRD
#define LCD_D6_PIN PD6

#define LCD_D7_PORT PORTD
#define LCD_D7_DDR DDRD
#define LCD_D7_PIN PD7

#define LCD_E_PORT PORTD
#define LCD_E_DDR DDRD
#define LCD_E_PIN PD3

#define LCD_RS_PORT PORTD
#define LCD_RS_DDR DDRD
#define LCD_RS_PIN PD2

//Define the wanted LCD type (only one line can be active at a time):
//#define LCD_1X8
//#define LCD_1X16
//#define LCD_1X20
//#define LCD_1X40
//#define LCD_2X8       // some 1x16 are wired as 2x8
//#define LCD_2X12
#define LCD_2X16
//#define LCD_2X20
//#define LCD_2X24
//#define LCD_2X40
//#define LCD_4X16
//#define LCD_4X20
#endif //ENABLE_LCD
//--
#ifdef ENABLE_GLCD
// define which ports are allocated for your graphic LCD
// You need two full ports for this!

// Included some changes for future additions, as separate settings for
// individual pins but don't touch that yet!!!
#define GLCD_CMD_PORT   PORTA		// Command Output Register
// THE FOLLWOING ARE NOT IMPLEMENTED YET, DON'T USE
// here goes the settings for every pin if maximum flexibility is desired
// #define GLCD_CMD_PORT_D_I
// #define GLCD_CMD_PORT_RW
// #define GLCD_CMD_PORT_EN
// #define GLCD_CMD_PORT_CSEL1
// #define GLCD_CMD_PORT_CSEL2

#define GLCD_CMD_DIR    DDRA		// Data Direction Register for Command Port
// THE FOLLWOING ARE NOT IMPLEMENTED YET, DON'T USE
// here goes the settings for every pin if maximum flexibility is desired
// #define GLCD_CMD_DIR_D_I
// #define GLCD_CMD_DIR_RW
// #define GLCD_CMD_DIR_EN
// #define GLCD_CMD_DIR_CSEL1
// #define GLCD_CMD_DIR_CSEL2

#define GLCD_DATA_IN    PINC		// Data Input Register
// THE FOLLWOING ARE NOT IMPLEMENTED YET, DON'T USE
// here goes the settings for every pin if maximum flexibility is desired
// #define GLCD_DATA_IN_0
// #define GLCD_DATA_IN_1
// #define GLCD_DATA_IN_2
// #define GLCD_DATA_IN_3
// #define GLCD_DATA_IN_4
// #define GLCD_DATA_IN_5
// #define GLCD_DATA_IN_6
// #define GLCD_DATA_IN_7

#define GLCD_DATA_OUT   PORTC		// Data Output Register
// THE FOLLWOING ARE NOT IMPLEMENTED YET, DON'T USE
// here goes the settings for every pin if maximum flexibility is desired
// #define GLCD_DATA_OUT_0
// #define GLCD_DATA_OUT_1
// #define GLCD_DATA_OUT_2
// #define GLCD_DATA_OUT_3
// #define GLCD_DATA_OUT_4
// #define GLCD_DATA_OUT_5
// #define GLCD_DATA_OUT_6
// #define GLCD_DATA_OUT_7

#define GLCD_DATA_DIR   DDRC		// Data Direction Register for Data Port
// THE FOLLWOING ARE NOT IMPLEMENTED YET, DON'T USE
// here goes the settings for every pin if maximum flexibility is desired
// #define GLCD_DATA_DIR_0
// #define GLCD_DATA_DIR_1
// #define GLCD_DATA_DIR_2
// #define GLCD_DATA_DIR_3
// #define GLCD_DATA_DIR_4
// #define GLCD_DATA_DIR_5
// #define GLCD_DATA_DIR_6
// #define GLCD_DATA_DIR_7

// Command Port Bits
#define GLCD_D_I        0x00		// D/I Bit Number
#define GLCD_R_W        0x01		// R/W Bit Number
#define GLCD_EN         0x02		// EN Bit Number
#define GLCD_CSEL1      0x03		// CS1 Bit Number
#define GLCD_CSEL2      0x04		// CS2 Bit Number
#endif //ENABLE_GLCD
//--
#ifdef ENABLE_7SEG
// Define delay in microseconds between digits selection to avoid ghost effect
#define SEG_DELAY 1500
// Define the type of 7seg (common anode or common cathode)
#define SEG_COMMON_ANODE // comment this for common cathode type
//select the type of 7seg display (how many digits you have/use)
//must be the same as the buffer size (SEG_DIGITS_BUFFER[])
#define SEG_DIGITS_1 1
//#define SEG_DIGITS_2 2
//#define SEG_DIGITS_3 3
//#define SEG_DIGITS_4 4
//#define SEG_DIGITS_5 5
//#define SEG_DIGITS_6 6
//#define SEG_DIGITS_7 7
//#define SEG_DIGITS_8 8
// Define buffer for how many digits you have/are going to use
uint8_t SEG_DIGITS_BUFFER[1];

// Define the specific ports and pins used for the 7 segment display
// Default is for Arduino UNO or Severino M2 boards
#define SEG_A_PORT PORTD
#define SEG_A_DDR DDRD
#define SEG_A_PIN PD2

#define SEG_B_PORT PORTD
#define SEG_B_DDR DDRD
#define SEG_B_PIN PD3

#define SEG_C_PORT PORTD
#define SEG_C_DDR DDRD
#define SEG_C_PIN PD4

#define SEG_D_PORT PORTD
#define SEG_D_DDR DDRD
#define SEG_D_PIN PD5

#define SEG_E_PORT PORTD
#define SEG_E_DDR DDRD
#define SEG_E_PIN PD6

#define SEG_F_PORT PORTD
#define SEG_F_DDR DDRD
#define SEG_F_PIN PD7

#define SEG_G_PORT PORTB
#define SEG_G_DDR DDRB
#define SEG_G_PIN PB2

// comment/uncomment the following 3 lines if you use/don't use the dot(DP) indicator
// don't forget to edit them according to your hardware settings.
#define SEG_DP_PORT PORTB
#define SEG_DP_DDR DDRB
#define SEG_DP_PIN PB1

//Now, definitions for common pin (cathode or anode)
//Disable or Enable definitions, according with how many 7seg. digits you have
//Of course, you must change definitions according to your hardware connections
#define SEG_COMM_0_PORT PORTC
#define SEG_COMM_0_DDR DDRC
#define SEG_COMM_0_PIN PC2

//#define SEG_COMM_1_PORT PORTC
//#define SEG_COMM_1_DDR DDRC
//#define SEG_COMM_1_PIN PC3

//#define SEG_COMM_2_PORT PORTC
//#define SEG_COMM_2_DDR DDRC
//#define SEG_COMM_2_PIN PC4

//#define SEG_COMM_3_PORT PORTC
//#define SEG_COMM_3_DDR DDRC
//#define SEG_COMM_3_PIN PC5

//#define SEG_COMM_4_PORT PORT
//#define SEG_COMM_4_DDR DDR
//#define SEG_COMM_4_PIN P

//#define SEG_COMM_5_PORT PORT
//#define SEG_COMM_5_DDR DDR
//#define SEG_COMM_5_PIN P

//#define SEG_COMM_6_PORT PORT
//#define SEG_COMM_6_DDR DDR
//#define SEG_COMM_6_PIN P

//#define SEG_COMM_7_PORT PORT
//#define SEG_COMM_7_DDR DDR
//#define SEG_COMM_7_PIN P

#endif
//--
#ifdef ENABLE_PCF8583
// set the PCF8583 address and which I2C it use...
#ifdef ENABLE_TWI
#define PCF8583_USE_TWI // disable if you want to use the software I2C
#endif
#define PCF8583_A0 0;          // relative base address, let it be
#define Physical_Address 0xA2; // On EVB board, the address for PCF is 0xA2
// - set to 0xA0 if pin A0 of PCF8583 is connected to GND
// - set to 0xA2 if pin A0 of PCF8583 is connected to VCC
#endif
//--
#ifdef ENABLE_RFM12B
#ifndef ENABLE_SPI
#error "This needs SPI so, uncomment ENANBLE_SPI from the library header"
#endif
// Enable (regarding to your freq.) only one of the following two definitions
//#define RF_FREQ_868MHz 1
#define RF_FREQ_433MHz 1

// Define the CS pin for RFM12B module
// (check if it conflicts with SD card or any other SPI module)
#define RF_CS_DDR  DDRB
#define RF_CS_PORT PORTB
#define RF_CS_PIN  PB4
//define activity LEDs (disable them if you don't have available pins)
#define GREEN_LED_DDR  DDRB
#define GREEN_LED_PORT PORTB
#define GREEN_LED_PIN  PB4

#define RED_LED_DDR  DDRB
#define RED_LED_PORT PORTB
#define RED_LED_PIN  PB4

#endif //ENABLE_RFM12B
//--
#ifdef ENABLE_GPL_RFM12B
/******************************************************
 *                                                    *
 *           C O N F I G U R A T I O N                *
 *                                                    *
 ******************************************************/

/*
 Connect the RFM12 to the AVR as follows:

 RFM12           | AVR
 ----------------+------------
 SDO             | MISO
 nIRQ            | INT0
 FSK/DATA/nFFS   | VCC
 DCLK/CFIL/FFIT  |  -
 CLK             |  -
 nRES            |  -
 GND             | GND
 ANT             |  -
 VDD             | VCC
 GND             | GND
 nINT/VDI        | -
 SDI             | MOSI
 SCK             | SCK
 nSEL            | Slave select pin defined below RF_BIT_SS, etc...
 */

/************************
 * RFM12 PIN DEFINITIONS
 */

//Pin that the RFM12's slave select is connected to
#define RF_DDR_SS DDRC
#define RF_PORT_SS PORTC
#define RF_BIT_SS 3

//SPI setup
// - we don't need this part, as we have our SPI functions

/************************
 * RFM12 CONFIGURATION OPTIONS
 */

//baseband of the module (either RFM12_BAND_433, RFM12_BAND_868 or RFM12_BAND_912)
#define RFM12_BASEBAND RFM12_BAND_433

//center frequency to use (+- FSK frequency shift)
#define RFM12_FREQUENCY       433170000UL

//Transmit FSK frequency shift in kHz
#define FSK_SHIFT             125000

//Receive RSSI Threshold
#define RFM12_RSSI_THRESHOLD  RFM12_RXCTRL_RSSI_79

//Receive Filter Bandwidth
#define RFM12_FILTER_BW       RFM12_RXCTRL_BW_400

//Output power relative to maximum (0dB is maximum)
#define RFM12_POWER           RFM12_TXCONF_POWER_0

//Receive LNA gain
#define RFM12_LNA_GAIN        RFM12_RXCTRL_LNA_6

//crystal load capacitance - the frequency can be verified by measuring the
//clock output of RFM12 and comparing to 1MHz.
//11.5pF seems to be o.k. for RFM12, and 10.5pF for RFM12BP, but this may vary.
#define RFM12_XTAL_LOAD       RFM12_XTAL_11_5PF

//use this for datarates >= 2700 Baud
#define DATARATE_VALUE        RFM12_DATARATE_CALC_HIGH(9600.0)

//use this for 340 Baud < datarate < 2700 Baud
//#define DATARATE_VALUE      RFM12_DATARATE_CALC_LOW(1200.0)

//TX BUFFER SIZE
#define RFM12_TX_BUFFER_SIZE  30

//RX BUFFER SIZE (there are going to be 2 Buffers of this size for double_buffering)
#define RFM12_RX_BUFFER_SIZE  30

/************************
 * RFM12 INTERRUPT VECTOR
 * set the name for the interrupt vector here
 */

#if defined(EICRA) && defined(ISC00) && defined(EIMSK)
//the interrupt vector
#define RFM12_INT_VECT (INT0_vect)

//the interrupt mask register
#define RFM12_INT_MSK EIMSK

//the interrupt bit in the mask register
#define RFM12_INT_BIT (INT0)

//the interrupt flag register
#define RFM12_INT_FLAG EIFR

//the interrupt bit in the flag register
#define RFM12_FLAG_BIT (INTF0)

//setup the interrupt to trigger on negative edge
#define RFM12_INT_SETUP()   EICRA |= (1<<ISC11)


#elif defined(MCUCR) && defined(ISC00) && defined(GICR)
//the interrupt vector
#define RFM12_INT_VECT (INT0_vect)

//the interrupt mask register
#define RFM12_INT_MSK GICR

//the interrupt bit in the mask register
#define RFM12_INT_BIT (INT0)

//the interrupt flag register
#define RFM12_INT_FLAG GIFR

//the interrupt bit in the flag register
#define RFM12_FLAG_BIT (INTF0)

//setup the interrupt to trigger on negative edge
#define RFM12_INT_SETUP()   MCUCR |= (1<<ISC11)

#elif defined(MCUCR) && defined(ISC00) && defined(GIMSK)
//the interrupt vector
#define RFM12_INT_VECT (INT0_vect)

//the interrupt mask register
#define RFM12_INT_MSK GIMSK

//the interrupt bit in the mask register
#define RFM12_INT_BIT (INT0)

//the interrupt flag register
#define RFM12_INT_FLAG GIFR

//the interrupt bit in the flag register
#define RFM12_FLAG_BIT (INTF0)

//setup the interrupt to trigger on negative edge
#define RFM12_INT_SETUP()   MCUCR |= (1<<ISC11)


#else
#error "Interrupt not finished for this CPU"
#endif

/************************
 * FEATURE CONFIGURATION
 */

#define RFM12_LIVECTRL 0
#define RFM12_LIVECTRL_CLIENT 0
#define RFM12_LIVECTRL_HOST 0
#define RFM12_LIVECTRL_LOAD_SAVE_SETTINGS 0
#define RFM12_NORETURNS 0
#define RFM12_NOCOLLISIONDETECTION 0
#define RFM12_TRANSMIT_ONLY 0
#define RFM12_SPI_SOFTWARE 0
#define RFM12_USE_POLLING 0
#define RFM12_RECEIVE_ASK 0
#define RFM12_TRANSMIT_ASK 0
#define RFM12_USE_WAKEUP_TIMER 0
#define RFM12_USE_POWER_CONTROL 0
#define RFM12_LOW_POWER 0
#define RFM12_USE_CLOCK_OUTPUT 0
#define RFM12_LOW_BATT_DETECTOR 0

#define RFM12_LBD_VOLTAGE             RFM12_LBD_VOLTAGE_3V0

#define RFM12_CLOCK_OUT_FREQUENCY     RFM12_CLOCK_OUT_FREQUENCY_1_00_MHz

/* use a callback function that is called directly from the
 * interrupt routine whenever there is a data packet available. When
 * this value is set to 1, you must use the function
 * "rfm12_set_callback(your_function)" to point to your
 * callback function in order to receive packets.
 */
#define RFM12_USE_RX_CALLBACK 0

/************************
 * RFM12BP support (high power version of RFM12)
 */

//To use RFM12BP, which needs control signals for RX enable and TX enable,
//use these defines (set to your pinout of course).
//The TX-Part can also be used to control a TX-LED with the nomral RFM12
/*
 #define RX_INIT_HOOK  DDRD |= _BV(PD5)
 #define RX_LEAVE_HOOK PORTD &= ~_BV(PD5)
 #define RX_ENTER_HOOK PORTD |= _BV(PD5)

 #define TX_INIT_HOOK  DDRD |= _BV(PD4)
 #define TX_LEAVE_HOOK PORTD &= ~_BV(PD4)
 #define TX_ENTER_HOOK PORTD |= _BV(PD4)
 */

/************************
 * UART DEBUGGING
 * en- or disable debugging via uart.
 */

#define RFM12_UART_DEBUG 0

#endif // ENABLE_GPL_RFM12B
//--
#ifdef ENABLE_MIRF24
//Define the pin on change interrupt which you connect to nIRQ of NRF24 module.
// - it can be between 0 and 23 on 28pin devices,
// - and       between 0 and 31 on 40pin devices (look at your device's pinout).
//#define mirf_PCINT 0
// define CE pin (selecting RX/TX mode) - any digital pin on microcontroller
#define mirf_CE_DDR  DDRC
#define mirf_CE_PORT PORTC
#define mirf_CE_PIN  PC6
// define CSN (SPI chip select for NRF24 module) - any digital pin on microcontroller
#define mirf_CSN_DDR  DDRC
#define mirf_CSN_PORT PORTC
#define mirf_CSN_PIN  PC7

#define mirf_CHANNEL 2
#define mirf_PAYLOAD 16

#define mirf_ADDR_LEN	5
#define mirf_CONFIG ((1<<RF24_EN_CRC) | (0<<RF24_CRCO) )

#endif // ENABLE_MIRF24

// ============ END USER ZONE --- NO EDITING ALLOWED!       ====================
// = ************************************************************************* =
// ============ DON'T MAKE MODIFICATIONS BELLOW THIS BORDER ====================

#define FALSE  0
#define TRUE   1
#define HIGH   1
#define LOW    0
#define OFF    0
#define ON     1
#define OUTPUT 1
#define INPUT  0

// stolen from and-tech.pl in attempt to use some of their functions
#define cbi(PORT, BIT) (_SFR_BYTE(PORT) &= ~_BV(BIT))// clear bit
#define sbi(PORT, BIT) (_SFR_BYTE(PORT) |= _BV(BIT)) // set bit
#define tbi(PORT, BIT) (_SFR_BYTE(PORT) ^= _BV(BIT)) // toggle bit
#define DDR(x) (_SFR_IO8(_SFR_IO_ADDR(x)-1))
#define PIN(x) (_SFR_IO8(_SFR_IO_ADDR(x)-2))
//-

// For the following macros, see Joe's Pardue documentation at
// http://code.google.com/p/avrtoolbox/
#define bit(x)	(1 << (x))
#define bit_get(p,m) ((p) & bit(m))
#define bit_set(p,m) ((p) |= bit(m))
#define bit_clear(p,m) ((p) &= ~bit(m))
#define bit_flip(p,m) ((p) ^= bit(m)) // toggle bit (see tbi)
#define bit_write(c,p,m) (c ? bit_set(p,m) : bit_clear(p,m))
#define bit_isset(p, bit) bit_get(p, bit)
#define bit_isclear(p, bit) (!bit_get(p, bit))
#define bit_rotate_right(a) a = ((a & 0x01)? ((a >> 1)| (1ULL << ((sizeof(a) * 8) - 1))) : (a >> 1))
#define bit_rotate_left(a) a = ( (a &  (  1ULL << ((sizeof(a) * 8) -1)))   ? ((a << 1) | 1) : (a << 1))
#define bit_to_shift8(mask) (((mask) & 0x01) ? 0 : (((mask) & 0x02) ? 1 : (((mask) & 0x04) ? 2 : (((mask) & 0x08) ? 3 : (((mask) & 0x10) ? 4 : (((mask) & 0x20) ? 5 : (((mask) & 0x40) ? 6 : 7 )))))))
#define bit_get_mask_field8(byte, mask) (((byte) & (mask))>>bit_to_shift8(mask))
//#define bit_to_shift8(mask) (((mask) & 0x01) ? 0 : (((mask) & 0x02) ? 1 : (((mask) & 0x04) ? 2 : (((mask) & 0x08) ? 3 : (((mask) & 0x10) ? 4 : (((mask) & 0x20) ? 5 : (((mask) & 0x40) ? 6 : (((mask) & 0x80) ? 7 : 8 ))))))))
//#define bit_get_mask_field8(byte, mask) (((byte) & (mask))>>bit_to_shift8(mask))
#define bit_hi_byte(x)	((uint8_t)(((uint16_t)(x)) >> 8))
#define bit_lo_byte(x)	((uint8_t)((uint16_t)(x)))
#define bit_byte_to_word(high, low) ((uint16_t)(((uint16_t)high << 8) | low))

// this is a permanent option but it can be made conditional
void onboard_led_enable(void);
void onboard_led_on(void);
void onboard_led_off(void);
void onboard_led_toggle(void);
//--
#ifdef ENABLE_MILLIS
#ifdef ENABLE_NB_DELAYS
#error "Non blocking delays conflicts with millis"
#endif
void millis_init(void);
uint32_t millis(void);
#endif
//--
#ifdef ENABLE_NB_DELAYS
#ifdef ENABLE_MILLIS
#error "Millis conflicts with non blocking delays"
#endif
void timer0_isr_init(void);
uint8_t check_delay(uint8_t slot);
void set_delay(uint8_t slot, uint16_t ms_time);
#endif

#ifdef ENABLE_I2C_SOFTWARE
#define I2C_PORT_O	I2C_PORT
#define I2C_PORT_D	DDR(I2C_PORT)
#define I2C_PORT_I	PIN(I2C_PORT)

#define I2C_SDA_WR()	sbi(I2C_PORT_D,I2C_SDA)
#define I2C_SDA_RD()	cbi(I2C_PORT_D,I2C_SDA)
#define I2C_SCL_WR()	sbi(I2C_PORT_D,I2C_SCL)
#define I2C_SCL_RD()	cbi(I2C_PORT_D,I2C_SCL)

#define I2C_SDA_H()	sbi(I2C_PORT_O,I2C_SDA)
#define I2C_SDA_L()	cbi(I2C_PORT_O,I2C_SDA)
#define I2C_SCL_H()	sbi(I2C_PORT_O,I2C_SCL)
#define I2C_SCL_L()	cbi(I2C_PORT_O,I2C_SCL)

uint8_t I2C_write(uint8_t b);
uint8_t I2C_read(uint8_t ack);
void I2C_start(void);
void I2C_stop(void);
#endif // end I2C software
#ifdef ENABLE_TWI

void TWI_init(void);
uint8_t TWI_start(void);
uint8_t TWI_repeatStart(void);
uint8_t TWI_sendAddress(uint8_t);
uint8_t TWI_sendData(uint8_t);
uint8_t TWI_receiveData_ACK(void);
uint8_t TWI_receiveData_NACK(void);
void TWI_stop(void);

#endif // ENABLE_TWI
#ifdef ENABLE_SPI //In preparation of porting ArduinoISP sketch
#ifdef ENABLE_SPI_INT
volatile uint8_t SPI_TC;
#endif

//SETTING "pinout" depending on microcontroller
#if defined(__AVR_ATmega16__)    || \
	defined(__AVR_ATmega16A__)     || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
    defined(__AVR_ATmega32A__)     || \
    defined(__AVR_ATmega324P__)    || \
    defined(__AVR_ATmega324PA__)   || \
    defined(__AVR_ATmega644__)     || \
    defined(__AVR_ATmega644P__)    || \
    defined(__AVR_ATmega1284P__)

#define SCK      PB7
#define SCK_PORT PORTB
#define SCK_DDR  DDRB

#define MOSI      PB5
#define MOSI_PORT PORTB
#define MOSI_DDR  DDRB

#define MISO      PB6
#define MISO_PORT PORTB
#define MISO_PIN  PINB
#define MISO_DDR  DDRB
// no to set MISO_DDR on hardware SPI, as is set automatically as input

#define SS      PB4
#define SS_PORT PORTB
#define SS_DDR  DDRB

#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560P__) // Arduino Mega1280
#define SCK      PB1
#define SCK_PORT PORTB
#define SCK_DDR  DDRB

#define MOSI      PB2
#define MOSI_PORT PORTB
#define MOSI_DDR  DDRB

#define MISO      PB3
#define MISO_PORT PORTB
#define MISO_PIN  PINB
#define MISO_DDR  DDRB
// no need to set MISO_DDR on hardware SPI, as is set automatically as input

#define SS      PB0
#define SS_PORT PORTB
#define SS_DDR  DDRB

#elif defined(__AVR_ATmega48__)    || \
      defined(__AVR_ATmega88__)      || \
      defined(__AVR_ATmega88P__)     || \
      defined(__AVR_ATmega168__)     || \
      defined(__AVR_ATmega168P__)    || \
      defined(__AVR_ATmega328P__)    // Arduino 28 pins
#define SCK      PB5
#define SCK_PORT PORTB
#define SCK_DDR  DDRB

#define MOSI      PB3
#define MOSI_PORT PORTB
#define MOSI_DDR  DDRB

#define MISO      PB4
#define MISO_PORT PORTB
#define MISO_PIN  PINB
#define MISO_DDR  DDRB
// no need to set MISO_DDR on hardware SPI, as is set automatically as input

#define SS      PB2
#define SS_PORT PORTB
#define SS_DDR  DDRB

#endif // pinout SETTINGS
#define LSBFIRST 0
#define MSBFIRST 1

#define SPI_CLOCK_DIV4 0x00
#define SPI_CLOCK_DIV16 0x01
#define SPI_CLOCK_DIV64 0x02
#define SPI_CLOCK_DIV128 0x03
#define SPI_CLOCK_DIV2 0x04
#define SPI_CLOCK_DIV8 0x05
#define SPI_CLOCK_DIV32 0x06
#define SPI_CLOCK_2DIV64 0x07

#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

#define SPI_MODE_MASK 0x0C  // CPOL = bit 3, CPHA = bit 2 on SPCR
#define SPI_CLOCK_MASK 0x03  // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define SPI_2XCLOCK_MASK 0x01  // SPI2X = bit 0 on SPSR
void SPI_master_setBitOrder(uint8_t bitOrder);
void SPI_master_setDataMode(uint8_t mode);
void SPI_master_setClockDivider(uint8_t rate);
void SPI_master_stop(void);

// Dharmani settings
// Master mode, MSB first, SCK phase low, SCK idle low, Low speed (fosc/64)
// SPCR register:
// | SPIE | SPE | DORD | MSTR | CPOL | CPHA | SPR1 | SPR0 |
// |  0   |  1  |  0   |  1   |  0   |  0   |  1   |  0   | = 0x52
#define SPI_LOW_SPEED        SPCR = 0x52; SPSR = 0
// Master mode, MSB first, SCK phase low, SCK idle low, Max 2 x speed (fosc/2)
// SPCR register:
// | SPIE | SPE | DORD | MSTR | CPOL | CPHA | SPR1 | SPR0 |
// |  0   |  1  |  0   |  1   |  0   |  0   |  0   |  0   | = 0x50
#define SPI_HIGH_SPEED       SPCR = 0x50; SPSR |= (1<<SPI2X)

void SPI_master_init(void);
uint8_t SPI_master_transmit(uint8_t);
uint8_t SPI_master_receive(void);
uint8_t SPI_master_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d);

#endif // ENABLE_SPI
//--
#ifdef ENABLE_SD_CARD
//SD commands, many of these are not used ...
#define GO_IDLE_STATE            0
#define SEND_OP_COND             1
#define SEND_IF_COND             8
#define SEND_CSD                 9
#define STOP_TRANSMISSION        12
#define SEND_STATUS              13
#define SET_BLOCK_LEN            16
#define READ_SINGLE_BLOCK        17
#define READ_MULTIPLE_BLOCKS     18
#define WRITE_SINGLE_BLOCK       24
#define WRITE_MULTIPLE_BLOCKS    25
#define ERASE_BLOCK_START_ADDR   32
#define ERASE_BLOCK_END_ADDR     33
#define ERASE_SELECTED_BLOCKS    38
#define SD_SEND_OP_COND          41   //ACMD
#define APP_CMD                  55
#define READ_OCR                 58
#define CRC_ON_OFF               59

#define ON     1
#define OFF    0

volatile uint32_t SD_startBlock, SD_totalBlocks;
volatile uint8_t SDHC_flag, SD_cardType, SD_buffer[512];

// select/deselect the sd-card - sorry, is Dharmani's naming choice...
// the direction(input/output) of this pin will be set inside SD_init() function.
#define SD_CS_ASSERT     cbi(SD_CS_PORT, SD_CS_PIN)
#define SD_CS_DEASSERT   sbi(SD_CS_PORT, SD_CS_PIN)

uint8_t SD_init(void);
uint8_t SD_card_type(void);
uint8_t SD_sendCommand(uint8_t cmd, uint32_t arg);
uint8_t SD_readSingleBlock(uint32_t SD_startBlock);
uint8_t SD_writeSingleBlock(uint32_t SD_startBlock);
uint8_t SD_erase(uint32_t SD_startBlock, uint32_t SD_totalBlocks);
#endif // ENABLE_SD_CARD
#ifdef ENABLE_FAT32

//Structure to access Master Boot Record for getting info about partitions
struct MBRinfo_Structure {
	uint8_t nothing[446]; //ignore, placed here to fill the gap in the structure
	uint8_t partitionData[64];//partition records (16x4)
	uint16_t signature;//0xaa55
};

//Structure to access info of the first partition of the disk
struct partitionInfo_Structure {
	uint8_t status; //0x80 - active partition
	uint8_t headStart;//starting head
	uint16_t cylSectStart;//starting cylinder and sector
	uint8_t type;//partition type
	uint8_t headEnd;//ending head of the partition
	uint16_t cylSectEnd;//ending cylinder and sector
	uint32_t firstSector;//total sectors between MBR & the first sector of the partition
	uint32_t sectorsTotal;//size of this partition in sectors
};

//Structure to access boot sector data
struct BS_Structure {
	uint8_t jumpBoot[3]; //default: 0x009000EB
	uint8_t OEMName[8];
	uint16_t bytesPerSector;//default: 512
	uint8_t sectorPerCluster;
	uint16_t reservedSectorCount;
	uint8_t numberofFATs;
	uint16_t rootEntryCount;
	uint16_t totalSectors_F16;//must be 0 for FAT32
	uint8_t mediaType;
	uint16_t FATsize_F16;//must be 0 for FAT32
	uint16_t sectorsPerTrack;
	uint16_t numberofHeads;
	uint32_t hiddenSectors;
	uint32_t totalSectors_F32;
	uint32_t FATsize_F32;//count of sectors occupied by one FAT
	uint16_t extFlags;
	uint16_t FSversion;//0x0000 (defines version 0.0)
	uint32_t rootCluster;//first cluster of root directory (=2)
	uint16_t FSinfo;//sector number of FSinfo structure (=1)
	uint16_t BackupBootSector;
	uint8_t reserved[12];
	uint8_t driveNumber;
	uint8_t reserved1;
	uint8_t bootSignature;
	uint32_t volumeID;
	uint8_t volumeLabel[11];//"NO NAME "
	uint8_t fileSystemType[8];//"FAT32"
	uint8_t bootData[420];
	uint16_t bootEndSignature;//0xaa55
};

//Structure to access FSinfo sector data
struct FSInfo_Structure {
	uint32_t leadSignature; //0x41615252
	uint8_t reserved1[480];
	uint32_t structureSignature;//0x61417272
	uint32_t freeClusterCount;//initial: 0xffffffff
	uint32_t nextFreeCluster;//initial: 0xffffffff
	uint8_t reserved2[12];
	uint32_t trailSignature;//0xaa550000
};

//Structure to access Directory Entry in the FAT
struct dir_Structure {
	uint8_t name[11];
	uint8_t attrib; //file attributes
	uint8_t NTreserved;//always 0
	uint8_t timeTenth;//tenths of seconds, set to 0 here
	uint16_t createTime;//time file was created
	uint16_t createDate;//date file was created
	uint16_t lastAccessDate;
	uint16_t firstClusterHI;//higher word of the first cluster number
	uint16_t writeTime;//time of last write
	uint16_t writeDate;//date of last write
	uint16_t firstClusterLO;//lower word of the first cluster number
	uint32_t fileSize;//size of file in bytes
};

//Attribute definitions for file/directory
#define ATTR_READ_ONLY     0x01
#define ATTR_HIDDEN        0x02
#define ATTR_SYSTEM        0x04
#define ATTR_VOLUME_ID     0x08
#define ATTR_DIRECTORY     0x10
#define ATTR_ARCHIVE       0x20
#define ATTR_LONG_NAME     0x0f

#define DIR_ENTRY_SIZE     0x32
#define EMPTY              0x00
#define DELETED            0xe5
#define GET                   0
#define SET                   1
#define READ                  0
#define VERIFY                1
#define ADD                   0
#define REMOVE                1
#define LOW                   0
#define HIGH                  1
#define TOTAL_FREE            1
#define NEXT_FREE             2
#define GET_LIST              0
#define GET_FILE              1
#define DELETE                2
#define FAT32_EOF    0x0fffffff

//************* external variables *************
volatile uint32_t firstDataSector, rootCluster, totalClusters;
volatile uint16_t bytesPerSector, sectorPerCluster, reservedSectorCount;
uint32_t unusedSectors, appendFileSector, appendFileLocation, fileSize,
appendStartCluster;

//global flag to keep track of free cluster count updating in FSinfo sector
uint8_t freeClusterCountUpdated;

//data string where data is collected before sending to the card
volatile uint8_t dataString[MAX_STRING_SIZE];
uint16_t timeFAT, dateFAT;

//************* functions *************
uint8_t getDateTime_FAT(void);
uint8_t F32_getBootSectorData(void);
uint32_t F32_getFirstSector(uint32_t clusterNumber);
uint32_t F32_getSetFreeCluster(uint8_t totOrNext, uint8_t get_set,
		uint32_t FSEntry);
struct dir_Structure* F32_findFiles(uint8_t flag, uint8_t *fileName);
uint32_t F32_getSetNextCluster(uint32_t clusterNumber, uint8_t get_set,
		uint32_t clusterEntry);
uint8_t F32_readFile(uint8_t flag, uint8_t *fileName);
//uint8_t F32_convertFileName(uint8_t *fileName);
uint8_t F32_writeFile(uint8_t *fileName, uint8_t *dataString);
void F32_appendFile(void);
uint32_t F32_searchNextFreeCluster(uint32_t startCluster);
void F32_displayMemory(uint8_t flag, uint32_t memory);
void F32_deleteFile(uint8_t *fileName);
void F32_freeMemoryUpdate(uint8_t flag, uint32_t size);
#endif //ENABLE_FAT32
//--
#ifdef ENABLE_RFM12B
/* DOCUMENTATION
 *
 * 	- Hardware connections:

 RFM12B		|	ATMEGA328P	|	Other Hardware		|	ATMEGA1284P
 ------------------------------------------------------------------
 GND		|	   GND		|						|	GND
 VDD		|	   3.3V		|						|	3.3V

 FSK		|	   NC		|						|	NC
 CLK		|	   NC		|						|	NC
 DCLK	|	   NC		|						|	NC
 NIRQ	|	   NC		|						|	NC
 NRES	|	   NC		|						|	NC

 NSEL	|	   PB2		|						|	PB4
 SCK		|	   PB5		|						|	PB7
 SDI		|	   PB3		|						|	PB5
 SDO		|	   PB4		|						|	PB6

 |	   PB1		| 	RED LED circuit		|	PC4
 |	   PB0		| 	GREEN LED circuit	|	PC6
 *
 */

// use the following whenever read or send from/to RFM12B module,
// and especially when you have other modules connected to SPI and need
// different transmission speed (e.g., SD-Card)
#define RF_SPI_LOW_SPEED SPCR = 0x50; SPSR = 0

unsigned char volatile DATA_RDY2SND = 0; // flag for indicating when to transmit data
unsigned char volatile rf_buff[256], rf_rxbuff[256];//provide some buffer for data

int volatile rf_buffindex = 0;// index for keeping place in buffer
unsigned char volatile rf_rxbuffindex = 0;// index for keeping place in buffer

/*inline void timeout_init(void);
 inline uint8_t timeout(void);
 uint8_t rf12_is_ready(void);
 uint16_t rf12_trans(uint16_t to_send);
 void rf12_txbyte(uint8_t b);
 uint8_t rf12_rxbyte(void);*/
void radio_config(void);
void radio_send(uint8_t volatile * buffer, uint8_t len);
int16_t radio_rcv(uint8_t volatile * buffer, uint8_t max_len);

#endif
//--
#ifdef ENABLE_GPL_RFM12B
/************************
 * VARIOUS RFM RELATED DEFINES FOR INTERNAL USE
 *(defines which shall be visible to the user are located in rfm12.h)
 */

//default preamble (altrernating 1s and 0s)
#define PREAMBLE 0xAA

//default synchronization pattern
#define SYNC_MSB 0x2D
#define SYNC_LSB 0xD4

//these are the states for the receive/transmit state machine
#define STATE_RX_IDLE 0
#define STATE_RX_ACTIVE 1
#define STATE_TX 2
#define STATE_POWER_DOWN 3

//packet header length in bytes
#define PACKET_OVERHEAD 3

/************************
 * LIBRARY DEFAULT SETTINGS
 */

#ifdef PWRMGT_DEFAULT
#warning "You are using the PWRMGT_DEFAULT makro directly in your rfm12_config.h - this is no longer supported."
#warning "RFM12_USE_WAKEUP_TIMER and RFM12_LOW_BATT_DETECTOR care about this on their own now. just remove PWRMGT_DEFAULT if you needed it for that."
#warning "if you need the clock output, use the new RFM12_USE_CLOCK_OUTPUT instead!"
#undef PWRMGT_DEFAULT
#endif

//if notreturns is not defined, we won't use this feature
#ifndef RFM12_NORETURNS
#define RFM12_NORETURNS 0
#endif

//if transmit only is not defined, we won't use this feature
#ifndef RFM12_TRANSMIT_ONLY
#define RFM12_TRANSMIT_ONLY 0
#endif

//if transmit only is on, we need to turn of collision detection
#if RFM12_TRANSMIT_ONLY
//disable collision detection, as we won't be able to receive data
#ifdef RFM12_NOCOLLISIONDETECTION
#undef RFM12_NOCOLLISIONDETECTION
#endif
#define RFM12_NOCOLLISIONDETECTION 1
#endif

//if nocollisiondetection is not defined, we won't use this feature
#ifndef RFM12_NOCOLLISIONDETECTION
#define RFM12_NOCOLLISIONDETECTION 0
#endif

//if livectrl is not defined, we won't use this feature
#ifndef RFM12_LIVECTRL
#define RFM12_LIVECTRL 0
#endif

//if low battery detector is not defined, we won't use this feature
#ifndef RFM12_LOW_BATT_DETECTOR
#define RFM12_LOW_BATT_DETECTOR 0
#endif

#ifndef RFM12_USE_CLOCK_OUTPUT
#define RFM12_USE_CLOCK_OUTPUT 0
#endif

#if RFM12_USE_CLOCK_OUTPUT
//Enable Xtal oscillator
#define PWRMGMT_CLCKOUT (RFM12_PWRMGT_EX)
#else
//Disable Clock output
#define PWRMGMT_CLCKOUT (RFM12_PWRMGT_DC)
#endif

//if the low battery detector feature is used, we will set some extra pwrmgmt options
#if RFM12_LOW_BATT_DETECTOR
//define PWRMGMT_LOW_BATT  with low batt detector
//it will be used later
#define PWRMGMT_LOW_BATT (RFM12_PWRMGT_EB)

//check if the default power management setting has the LB bit set
//and warn the user if it's not
#ifdef PWRMGT_DEFAULT
#if !((PWRMGT_DEFAULT) & RFM12_PWRMGT_EB)
#warning "You are using the RFM12 low battery detector, but PWRMGT_DEFAULT has the low battery detector bit unset."
#endif
#endif
#else
#define PWRMGMT_LOW_BATT 0
#endif /* RFM12_LOW_BATT_DETECTOR */

//if wakeuptimer is not defined, we won't use this feature
#ifndef RFM12_USE_WAKEUP_TIMER
#define RFM12_USE_WAKEUP_TIMER 0
#endif

//if wakeuptimer is on, we will set the default power management to use the wakeup timer
#if RFM12_USE_WAKEUP_TIMER
//define PWRMGMT_LOW_BATT  with low batt detector
//it will be used later
#define PWRMGMT_WKUP (RFM12_PWRMGT_EW)

//check if the default power management setting has the EW bit set
//and warn the user if it's not
#ifdef PWRMGT_DEFAULT
#if !((PWRMGT_DEFAULT) & RFM12_PWRMGT_EW)
#warning "You are using the RFM12 wakeup timer, but PWRMGT_DEFAULT has the wakeup timer bit unset."
#endif
#endif

//enable powermanagement shadowing
#define RFM12_PWRMGT_SHADOW 1
#else
#define PWRMGMT_WKUP 0
#endif /* RFM12_USE_WAKEUP_TIMER */

//if ASK tx is not defined, we won't use this feature
#ifndef RFM12_TRANSMIT_ASK
#define RFM12_TRANSMIT_ASK 0
#endif

//if receive ASK is not defined, we won't use this feature
#ifndef RFM12_RECEIVE_ASK
#define RFM12_RECEIVE_ASK 0
#endif

//if software spi is not defined, we won't use this feature
#ifndef RFM12_SPI_SOFTWARE
#define RFM12_SPI_SOFTWARE 0
#endif

//if uart debug is not defined, we won't use this feature
#ifndef RFM12_UART_DEBUG
#define RFM12_UART_DEBUG 0
#endif

#ifndef RFM12_PWRMGT_SHADOW
#define RFM12_PWRMGT_SHADOW 0
#endif

//default value for powermanagement register
#ifndef PWRMGT_DEFAULT
#define PWRMGT_DEFAULT (PWRMGMT_CLCKOUT | PWRMGMT_WKUP | PWRMGMT_LOW_BATT)
#endif

//define a default receive power management mode
#if RFM12_TRANSMIT_ONLY
//the receiver is turned off by default in transmit only mode
#define PWRMGT_RECEIVE (RFM12_CMD_PWRMGT | PWRMGT_DEFAULT)
#else
//the receiver is turned on by default in normal mode
#define PWRMGT_RECEIVE (RFM12_CMD_PWRMGT | PWRMGT_DEFAULT | RFM12_PWRMGT_ER)
#endif

//default channel free time, if not defined elsewhere
#ifndef CHANNEL_FREE_TIME
#define CHANNEL_FREE_TIME 200
#endif

/*
 * backward compatibility for the spi stuff
 * these values weren't set in older revisions of this library
 * so they're now assumed to be on the same pin/port.
 * otherwise these defines serve to configure the software SPI ports
 */
/*
 #ifndef DDR_MOSI
 #define DDR_MOSI DDR_SPI
 #define PORT_MOSI PORT_SPI
 #endif

 #ifndef DDR_MISO
 #define DDR_MISO DDR_SPI
 #define PIN_MISO PIN_SPI
 #endif

 #ifndef DDR_SCK
 #define DDR_SCK DDR_SPI
 #define PORT_SCK PORT_SPI
 #endif

 #ifndef DDR_SPI_SS
 #define DDR_SPI_SS DDR_SPI
 #define PORT_SPI_SS PORT_SPI
 #endif
 */

/*
 * backward compatibility for rf channel settings
 * these values weren't set in the config in older revisions of this library
 * so we assume defaults here.
 */

#ifndef RFM12_XTAL_LOAD
#define                        RFM12_XTAL_LOAD RFM12_XTAL_11_5PF
#endif

#ifndef RFM12_POWER
#define RFM12_POWER            RFM12_TXCONF_POWER_0
#endif

#ifndef FSK_SHIFT
#define FSK_SHIFT 125000
#endif

#ifndef RFM12_RSSI_THRESHOLD
#define RFM12_RSSI_THRESHOLD   RFM12_RXCTRL_RSSI_79
#endif

#ifndef RFM12_FILTER_BW
#define RFM12_FILTER_BW        RFM12_RXCTRL_BW_400
#endif

#ifndef RFM12_LNA_GAIN
#define RFM12_LNA_GAIN         RFM12_RXCTRL_LNA_6
#endif

#ifndef RFM12_LBD_VOLTAGE
#define RFM12_LBD_VOLTAGE      RFM12_LBD_VOLTAGE_3V7
#endif

#ifndef RFM12_CLOCK_OUT_FREQUENCY
#define RFM12_CLOCK_OUT_FREQUENCY RFM12_CLOCK_OUT_FREQUENCY_1_00_MHz
#endif

#ifndef RFM12_FREQUENCY
#ifndef FREQ
#error "RFM12_FREQUENCY not defined."
#else
#define RFM12_FREQUENCY FREQ
#warning "using FREQ for RFM12_FREQUENCY. Please use RFM12_FREQUENCY in the future!"
#endif
#endif

//baseband selection
#if (RFM12_BASEBAND) == RFM12_BAND_433
#define RFM12_FREQUENCY_CALC(x) RFM12_FREQUENCY_CALC_433(x)
#elif (RFM12_BASEBAND) == RFM12_BAND_868
#define RFM12_FREQUENCY_CALC(x) RFM12_FREQUENCY_CALC_868(x)
#elif (RFM12_BASEBAND) == RFM12_BAND_915
#define RFM12_FREQUENCY_CALC(x) RFM12_FREQUENCY_CALC_915(x)
#else
#error "Unsupported RFM12 baseband selected."
#endif

/************************
 * HELPER MACROS
 */

//macros to turn the int on and off
//if polling is used, just define these macros as empty
#if !(RFM12_USE_POLLING)
#define RFM12_INT_ON() RFM12_INT_MSK |= (1<<RFM12_INT_BIT)
#define RFM12_INT_OFF() RFM12_INT_MSK &= ~(1<<RFM12_INT_BIT)
#else
#define RFM12_INT_ON()
#define RFM12_INT_OFF()
#endif /* !(RFM12_USE_POLLING) */

/*
 * the following macros help to manage the rfm12 fifo
 * default fiforeset is as follows:
 * 2 Byte Sync Pattern, disable sensitive reset, fifo filled interrupt at 8 bits
 */
//default fiforeset register value to accept data
#define ACCEPT_DATA (RFM12_CMD_FIFORESET | RFM12_FIFORESET_DR | (8<<4) | RFM12_FIFORESET_FF)
#define ACCEPT_DATA_INLINE (RFM12_FIFORESET_DR | (8<<4) | RFM12_FIFORESET_FF)
//default fiforeset register value to clear fifo
#define CLEAR_FIFO (RFM12_CMD_FIFORESET | RFM12_FIFORESET_DR | (8<<4))
#define CLEAR_FIFO_INLINE (RFM12_FIFORESET_DR | (8<<4))

//this macro helps to encapsulate the return values, when noreturn is set to on
#if (RFM12_NORETURNS)
#define TXRETURN(a)
#else
#define TXRETURN(a) (a)
#endif

/** \name States for rx and tx buffers
 * \anchor rxtx_states
 * \see rfm12_rx_status() and rfm12_control_t
 * @{
 */
//! Indicates that the buffer is free
#define STATUS_FREE 0
//! Indicates that the buffer is in use by the library
#define STATUS_OCCUPIED 1
//! Indicates that a receive buffer holds a complete transmission
#define STATUS_COMPLETE 2
//@}

/** \name  Return values for rfm12_tx() and rfm12_start_tx()
 * \anchor tx_retvals
 * \see rfm12_tx() and rfm12_start_tx()
 * @{
 */
//!  The packet data is longer than the internal buffer
#define RFM12_TX_ERROR 0x02
//! The transmit buffer is already occupied
#define RFM12_TX_OCCUPIED 0x03
//! The packet has been enqueued successfully
#define RFM12_TX_ENQUEUED 0x80
//@}

/************************
 * function protoypes
 */

//see rfm12.c for more documentation
void rfm12_init(void);
void rfm12_tick(void);

#if RFM12_USE_RX_CALLBACK
/* set the callback function pointer */
void rfm12_set_callback ((*in_func)(uint8_t, uint8_t *));
#endif
//if receive mode is not disabled (default)
#if !(RFM12_TRANSMIT_ONLY)
void rfm12_rx_clear(void);
#endif /* !(RFM12_TRANSMIT_ONLY) */

//FIXME: the tx function should return a status, do we need to do this also?
// uint8_t rfm12_tx_status();

#if (RFM12_NORETURNS)
//see rfm12.c for more documentation
void rfm12_start_tx(uint8_t type, uint8_t length);
#if !(RFM12_SMALLAPI)
void rfm12_tx(uint8_t len, uint8_t type, uint8_t *data);
#endif
#else
uint8_t rfm12_start_tx(uint8_t type, uint8_t length);
#if !(RFM12_SMALLAPI)
uint8_t rfm12_tx(uint8_t len, uint8_t type, uint8_t *data);
#endif
#endif

//if polling is used, define a polling function
#if RFM12_USE_POLLING
void rfm12_poll(void);
#endif

/************************
 * private control structs
 */

//! The transmission buffer structure.
/** \note Note that this complete buffer is transmitted sequentially,
 * beginning with the sync bytes.
 *
 * \see rfm12_start_tx(), rfm12_tx() and rf_tx_buffer
 */
typedef struct {
	//! Sync bytes for receiver to start filling fifo.
	uint8_t sync[2];

	//! Length byte - number of bytes in buffer.
	uint8_t len;

	//! Type field for the simple airlab protocol.
	uint8_t type;

	//! Checksum over the former two members.
	uint8_t checksum;

	//! Buffer for the raw bytes to be transmitted.
	uint8_t buffer[RFM12_TX_BUFFER_SIZE];
} rf_tx_buffer_t;

//if receive mode is not disabled (default)
#if !(RFM12_TRANSMIT_ONLY)
//! The receive buffer structure.
/** \note Note that there will be two receive buffers of this type,
 * as double buffering is being employed by this library.
 *
 * \see rfm12_rx_status(), rfm12_rx_len(), rfm12_rx_type(), rfm12_rx_buffer() , rfm12_rx_clear() and rf_rx_buffers
 */
typedef struct {
	//! Indicates if the buffer is free or completed.
	/** \see \ref rxtx_states "States for rx and tx buffers" */
	volatile uint8_t status;

	//! Length byte - number of bytes in buffer.
	uint8_t len;

	//! Type field for the simple airlab protocol.
	uint8_t type;

	//! Checksum over the type and length header fields
	uint8_t checksum;

	//! The actual receive buffer data
	uint8_t buffer[RFM12_RX_BUFFER_SIZE];
} rf_rx_buffer_t;
#endif /* !(RFM12_TRANSMIT_ONLY) */

//! Control and status structure.
/** This data structure keeps all control and some status related variables. \n
 * By using a central structure for all global variables, the compiler can
 * use smaller instructions and reduce the size of the binary.
 *
 * \note Some states are defined in the non-documented rfm12_core.h header file.
 * \see ISR(RFM12_INT_VECT, ISR_NOBLOCK), rfm12_tick() and ctrl
 */
typedef struct {
	//! This controls the library internal state machine.
	volatile uint8_t rfm12_state;

	//! Transmit buffer status.
	/** \see \ref rxtx_states "States for rx and tx buffers" */
	volatile uint8_t txstate;

	//! Number of bytes to transmit or receive.
	/** This refers to the overall data size, including header data and sync bytes. */
	uint8_t num_bytes;

	//! Counter for the bytes we are transmitting or receiving at the moment.
	uint8_t bytecount;

	//if receive mode is not disabled (default)
#if !(RFM12_TRANSMIT_ONLY)
	//! the number of the currently used in receive buffer.
	uint8_t buffer_in_num;

	//! the number of the currently used out receive buffer.
	uint8_t buffer_out_num;
#endif /* !(RFM12_TRANSMIT_ONLY) */

#if RFM12_PWRMGT_SHADOW
//! Power management shadow register.
/** The wakeup timer feature needs to buffer the current power management state. */
uint16_t pwrmgt_shadow;
#endif

#if RFM12_LOW_BATT_DETECTOR
//! Low battery detector status.
/** \see \ref batt_states "States for the low battery detection feature",
 * as well as rfm12_set_batt_detector() and rfm12_get_batt_status()
 */
volatile uint8_t low_batt;
#endif /* RFM12_LOW_BATT_DETECTOR */

#if RFM12_USE_WAKEUP_TIMER
//! Wakeup timer flag.
/** The wakeup timer feature sets this flag from the interrupt on it's ticks */
volatile uint8_t wkup_flag;
#endif

#if RFM12_LIVECTRL
uint16_t rxctrl_shadow;
uint16_t afc_shadow;
uint16_t txconf_shadow;
uint16_t cfg_shadow;
#endif
} rfm12_control_t;

/************************
 * GLOBALS
 */

//Buffer and status for the message to be transmitted
extern rf_tx_buffer_t rf_tx_buffer;

//if receive mode is not disabled (default)
#if !(RFM12_TRANSMIT_ONLY)
//buffers for storing incoming transmissions
extern rf_rx_buffer_t rf_rx_buffers[2];
#endif /* !(RFM12_TRANSMIT_ONLY) */

//the control struct
extern rfm12_control_t ctrl;

/************************
 * INLINE FUNCTIONS
 */

//if receive mode is not disabled (default)
#if !(RFM12_TRANSMIT_ONLY)
//! Inline function to return the rx buffer status byte.
/** \returns STATUS_FREE or STATUS_COMPLETE
 * \see \ref rxtx_states "rx buffer states", rfm12_rx_len(), rfm12_rx_type(), rfm12_rx_buffer(), rfm12_rx_clear() and rf_rx_buffer_t
 */
static inline uint8_t rfm12_rx_status(void) {
	return rf_rx_buffers[ctrl.buffer_out_num].status;
}

//! Inline function to return the rx buffer length field.
/** \returns The length of the data inside the buffer
 * \see rfm12_rx_status(), rfm12_rx_type(), rfm12_rx_buffer(), rfm12_rx_clear() and rf_rx_buffer_t
 */
static inline uint8_t rfm12_rx_len(void) {
	return rf_rx_buffers[ctrl.buffer_out_num].len;
}

//! Inline function to return the rx buffer type field.
/** \returns The packet type from the packet header type field
 * \see rfm12_rx_status(), rfm12_rx_len(), rfm12_rx_buffer(), rfm12_rx_clear() and rf_rx_buffer_t
 */
static inline uint8_t rfm12_rx_type(void) {
	return rf_rx_buffers[ctrl.buffer_out_num].type;
}

//! Inline function to retreive current rf buffer contents.
/** \returns A pointer to the current receive buffer contents
 * \see rfm12_rx_status(), rfm12_rx_len(), rfm12_rx_type(), rfm12_rx_clear() and rf_rx_buffer_t
 */
static inline uint8_t *rfm12_rx_buffer(void) {
	return (uint8_t*) rf_rx_buffers[ctrl.buffer_out_num].buffer;
}
#endif /* !(RFM12_TRANSMIT_ONLY) */

/************************
 * amplitude modulation receive mode
 */

#if RFM12_RECEIVE_ASK
/** \name States and buffer size for the amplitude modulated receive feature.
 * \anchor ask_defines
 * \note You need to define RFM12_RECEIVE_ASK as 1 to enable this.
 * \see rfrxbuf_t and ISR(ADC_vect, ISR_NOBLOCK)
 * @{
 */
//! The ASK receive buffer size.
#define RFM12_ASK_RFRXBUF_SIZE 55
//! The ASK receive buffer is empty.
#define RFM12_ASK_STATE_EMPTY 0
//! The ASK receive buffer is active.
#define RFM12_ASK_STATE_RECEIVING 1
//! The ASK receive buffer is full.
#define RFM12_ASK_STATE_FULL 2
//@}

//! The receive buffer structure for the amplitude modulated receive feature.
/** \note You need to define RFM12_RECEIVE_ASK as 1 to enable this.
 * \see ask_rxbuf (for further usage instructions) and ISR(ADC_vect, ISR_NOBLOCK)
 * \headerfile rfm12.h
 */
typedef struct {
	//! A pointer into the buffer, used while receiving.
	volatile uint8_t p;

	//! The buffer's state.
	/** \see See \ref ask_defines "ASK mode defines" for a list of possible states */
	volatile uint8_t state;

	//! The data buffer
	uint8_t buf[RFM12_ASK_RFRXBUF_SIZE];
}rfm12_rfrxbuf_t;

//see rfm12_extra.c for more documentation
extern rfm12_rfrxbuf_t ask_rxbuf;

//see rfm12_extra.c for more documentation
void adc_init(void);
#endif /* RFM12_RECEIVE_ASK */

/************************
 * amplitude modulated raw tx mode
 */

#if RFM12_TRANSMIT_ASK
//see rfm12_extra.c for more documentation
void rfm12_ask_tx_mode(uint8_t setting);

//see rfm12_extra.c for more documentation
inline void rfm12_tx_on(void);

//see rfm12_extra.c for more documentation
inline void rfm12_tx_off(void);
#endif /* RFM12_TRANSMIT_ASK  */

/************************
 * rfm12 wakeup timer mode
 */

#if RFM12_USE_WAKEUP_TIMER
//this function sets the wakeup timer register
//(see datasheet for values)
//see rfm12_extra.c for more documentation
void rfm12_set_wakeup_timer(uint16_t val);
#endif /* RFM12_USE_WAKEUP_TIMER */

#if RFM12_USE_POWER_CONTROL
void rfm12_power_down();
void rfm12_power_up();
#endif

/************************
 * rfm12 low battery detector mode
 */

#if RFM12_LOW_BATT_DETECTOR
/**\name States for the low battery detection feature.
 * \anchor batt_states
 * \see rfm12_set_batt_detector() and rfm12_get_batt_status()
 * @{
 */
//! Battery voltage is okay.
#define RFM12_BATT_OKAY 0
//! Low battery voltage detected.
#define RFM12_BATT_LOW 1
//@}

//this function sets the low battery detector and microcontroller clock divider register
//(see datasheet for values)
//see rfm12_extra.c for more documentation
void rfm12_set_batt_detector(uint16_t val);

//return the battery status
//see rfm12_extra.c for more documentation
uint8_t rfm12_get_batt_status(void);
#endif /* RFM12_LOW_BATT_DETECTOR */

typedef struct {
	uint16_t rfm12_hw_command; //actual SPI command for rfm12
	uint16_t rfm12_hw_parameter_mask; //mask that selects valid bits for this parameter
	uint16_t *shadow_register; //pointer to the shadow register to be used for this command
	uint16_t current_value;

#if RFM12_LIVECTRL_CLIENT
//these are for the client
uint16_t min_val;
uint16_t max_val;
int16_t step;
char *name;
void (*to_string)(char *str, uint16_t value);
#endif
} livectrl_cmd_t;

extern livectrl_cmd_t livectrl_cmds[];

//these constants have to be in the same order as the elemts in the table
//livectrl_cmds
#define RFM12_LIVECTRL_BASEBAND   0
#define RFM12_LIVECTRL_XTAL_LOAD  1
#define RFM12_LIVECTRL_FREQUENCY  2
#define RFM12_LIVECTRL_DATARATE   3
#define RFM12_LIVECTRL_TX_POWER   4
#define RFM12_LIVECTRL_FSK_SHIFT  5
#define RFM12_LIVECTRL_LNA        6
#define RFM12_LIVECTRL_RSSI       7
#define RFM12_LIVECTRL_FILTER_BW  8

#define NUM_LIVECTRL_CMDS         9

void rfm12_livectrl(uint8_t cmd, uint16_t value);

void rfm12_save_settings();
void rfm12_load_settings();

#if RFM12_LIVECTRL_CLIENT
void rfm12_livectrl_get_parameter_string(uint8_t cmd, char *str);
#endif

/* Configuration setting command
 Bit el enables the internal data register.
 Bit ef enables the FIFO mode. If ef=0 then DATA (pin 6) and DCLK (pin 7) are used for data and data clock output.
 x3 x2 x1 x0 Crystal Load Capacitance [pF]
 0 0 0 0 8.5
 0 0 0 1 9.0
 0 0 1 0 9.5
 0 0 1 1 10.0
 ....
 1 1 1 0 15.5
 1 1 1 1 16.0
 */
#	define RFM12_CMD_CFG 0x8000
#	define RFM12_CFG_EL 0x80
#	define RFM12_CFG_EF 0x40
#	define RFM12_CFG_BAND_MASK 0x30
#	define RFM12_BAND_315 0x00
#	define RFM12_BAND_433 0x10
#	define RFM12_BAND_868 0x20
#	define RFM12_BAND_915 0x30

#	define RFM12_CFG_XTAL_MASK 0x0F
#	define RFM12_XTAL_8_5PF  0x00
#	define RFM12_XTAL_9_0PF  0x01
#	define RFM12_XTAL_9_5PF  0x02
#	define RFM12_XTAL_10_0PF 0x03
#	define RFM12_XTAL_10_5PF 0x04
#	define RFM12_XTAL_11_0PF 0x05
#	define RFM12_XTAL_11_5PF 0x06
#	define RFM12_XTAL_12_0PF 0x07
#	define RFM12_XTAL_12_5PF 0x08
#	define RFM12_XTAL_13_0PF 0x09
#	define RFM12_XTAL_13_5PF 0x0A
#	define RFM12_XTAL_14_0PF 0x0B
#	define RFM12_XTAL_14_5PF 0x0C
#	define RFM12_XTAL_15_0PF 0x0D
#	define RFM12_XTAL_15_5PF 0x0E
#	define RFM12_XTAL_16_0PF 0x0F

/*
 2. Power Management Command
 Bit Function of the control bit Related blocks
 er Enables the whole receiver chain RF front end, baseband, synthesizer, oscillator
 ebb The receiver baseband circuit can be separately switched on Baseband
 et Switches on the PLL, the power amplifier, and starts the
 transmission (If TX register is enabled) Power amplifier, synthesizer, oscillator
 es Turns on the synthesizer Synthesizer
 ex Turns on the crystal oscillator Crystal oscillator
 eb Enables the low battery detector Low battery detector
 ew Enables the wake-up timer Wake-up timer
 dc Disables the clock output (pin 8) Clock output buffer
 */
#define RFM12_CMD_PWRMGT 0x8200
#define RFM12_PWRMGT_ER 0x80
#define RFM12_PWRMGT_EBB 0x40
#define RFM12_PWRMGT_ET 0x20
#define RFM12_PWRMGT_ES 0x10
#define RFM12_PWRMGT_EX 0x08
#define RFM12_PWRMGT_EB 0x04
#define RFM12_PWRMGT_EW 0x02
#define RFM12_PWRMGT_DC 0x01

/*
 3. Frequency Setting Command
 Bit 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0 POR
 1 0 1 0 f11 f10 f9 f8 f7 f6 f5 f4 f3 f2 f1 f0 A680h
 The 12-bit parameter F (bits f11 to f0) should be in the range
 of 96 and 3903. When F value sent is out of range, the
 previous value is kept. The synthesizer center frequency f0 can
 be calculated as:
 f0 = 10 * C1 * (C2 + F/4000) [MHz]
 The constants C1 and C2 are determined by
 the selected band as:
 Band [MHz] C1 C2
 433		1 43
 868		2 43
 915		3 30

 Frequency in 433 Band can be from 430.24MHz to 439.7575MHz in 2.5kHz increments.
 */

#define RFM12_CMD_FREQUENCY 0xA000
#define RFM12_FREQUENCY_MASK 0x0FFF
#define RFM12_FREQUENCY_CALC_433(f) (((f)-430000000UL)/2500UL)
#define RFM12_FREQUENCY_CALC_868(f) (((f)-860000000UL)/5000UL)
#define RFM12_FREQUENCY_CALC_915(f) (((f)-900000000UL)/7500UL)

/*
 4. Data Rate Command
 Bit 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0 POR
 1 1 0 0 0 1 1 0 cs r6 r5 r4 r3 r2 r1 r0 C623h
 The actual bit rate in transmit mode and the expected bit rate of the received data stream in receive mode is determined by the 7-bit
 parameter R (bits r6 to r0) and bit cs.
 BR = 10000 / 29 / (R+1) / (1+cs*7) [kbps]
 In the receiver set R according to the next function:
 R = (10000 / 29 / (1+cs*7) / BR) – 1, where BR is the expected bit rate in kbps.
 Apart from setting custom values, the standard bit rates from 600 bps to 115.2 kbps can be approximated with small error.
 Data rate accuracy requirements:
 Clock recovery in slow mode: ΔBR/BR < 1/(29*Nbit) Clock recovery in fast mode: ΔBR/BR < 3/(29*Nbit)
 BR is the bit rate set in the receiver and ΔBR is the bit rate difference between the transmitter and the receiver. Nbit is the maximum
 number of consecutive ones or zeros in the data stream. It is recommended for long data packets to include enough 1/0 and 0/1
 transitions, and to be careful to use the same division ratio in the receiver and in the transmitter.
 */

#define RFM12_CMD_DATARATE 0xC600
#define RFM12_DATARATE_CS 0x80
//calculate setting for datarates >= 2700 Baud
#define RFM12_DATARATE_CALC_HIGH(d) ((uint8_t)((10000000.0/29.0/d)-0.5))
//calculate setting for datarates < 2700 Baud
#define RFM12_DATARATE_CALC_LOW(d) (RFM12_DATARATE_CS|(uint8_t)((10000000.0/29.0/8.0/d)-0.5))
#define RFM12_DATARATE_MASK 0x00ff

/*
 5. Receiver Control Command
 Bit 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0 POR
 1 0 0 1 0 p16 d1 d0 i2 i1 i0 g1 g0 r2 r1 r0 9080h
 Bit 10 (p16): pin16 function select

 p16 Function of pin 16
 0 Interrupt input
 1 VDI output

 Bits 9-8 (d1 to d0): VDI (valid data indicator) signal response time setting:
 d1 d0 Response
 0 0 Fast
 0 1 Medium
 1 0 Slow
 1 1 Always on

 Bits 7-5 (i2 to i0): Receiver baseband bandwidth (BW) select:
 i2 i1 i0 BW [kHz]
 0 0 0 reserved
 0 0 1 400
 0 1 0 340
 0 1 1 270
 1 0 0 200
 1 0 1 134
 1 1 0 67
 1 1 1 reserved
 Bits 4-3 (g1 to g0): LNA gain select:
 g1 g0 relative to maximum [dB]
 0 0 0
 0 1 -6
 1 0 -14
 1 1 -20

 Bits 2-0 (r2 to r0): RSSI detector threshold:
 r2 r1 r0 RSSIsetth [dBm]
 0 0 0 -103
 0 0 1 -97
 0 1 0 -91
 0 1 1 -85
 1 0 0 -79
 1 0 1 -73
 1 1 0 Reserved
 1 1 1 Reserved
 The RSSI threshold depends on the LNA gain, the real RSSI threshold can be calculated:
 RSSIth=RSSIsetth+GLNA

 */

#define RFM12_CMD_RXCTRL 0x9000
#define RFM12_RXCTRL_P16_VDI 0x400
#define RFM12_RXCTRL_VDI_FAST 0x000
#define RFM12_RXCTRL_VDI_MEDIUM 0x100
#define RFM12_RXCTRL_VDI_SLOW 0x200
#define RFM12_RXCTRL_VDI_ALWAYS_ON 0x300

#define RFM12_RXCTRL_BW_MASK 0xE0
#define RFM12_RXCTRL_BW_400 0x20
#define RFM12_RXCTRL_BW_340 0x40
#define RFM12_RXCTRL_BW_270 0x60
#define RFM12_RXCTRL_BW_200 0x80
#define RFM12_RXCTRL_BW_134 0xA0
#define RFM12_RXCTRL_BW_67 0xC0

#define RFM12_RXCTRL_LNA_MASK 0x18
#define RFM12_RXCTRL_LNA_0 0x00
#define RFM12_RXCTRL_LNA_6 0x08
#define RFM12_RXCTRL_LNA_14 0x10
#define RFM12_RXCTRL_LNA_20 0x18

#define RFM12_RXCTRL_RSSI_103 0x00
#define RFM12_RXCTRL_RSSI_97 0x01
#define RFM12_RXCTRL_RSSI_91 0x02
#define RFM12_RXCTRL_RSSI_85 0x03
#define RFM12_RXCTRL_RSSI_79 0x04
#define RFM12_RXCTRL_RSSI_73 0x05
#define RFM12_RXCTRL_RSSI_67 0x06
#define RFM12_RXCTRL_RSSI_61 0x07
#define RFM12_RXCTRL_RSSI_MASK 0x07

/*
 6. Data Filter Command
 Bit 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0 POR
 1 1 0 0 0 0 1 0 al ml 1 s 1 f2 f1 f0 C22Ch

 Bit 7 (al): Clock recovery (CR) auto lock control, if set.
 CR will start in fast mode, then after locking it will automatically switch to slow mode.

 Bit 6 (ml): Clock recovery lock control
 1: fast mode, fast attack and fast release (4 to 8 bit preamble (1010...) is recommended)
 0: slow mode, slow attack and slow release (12 to 16 bit preamble is recommended)
 Using the slow mode requires more accurate bit timing (see Data Rate Command).

 Bits 4 (s): Select the type of the data filter:
 s Filter Type
 0 Digital filter
 1 Analog RC filter
 Digital: This is a digital realization of an analog RC filter followed by a comparator with hysteresis. The time constant is
 automatically adjusted to the bit rate defined by the Data Rate Command.
 Note: Bit rate can not exceed 115 kpbs in this mode.
 Analog RC filter: The demodulator output is fed to pin 7 over a 10 kOhm resistor. The filter cut-off frequency is set by the
 external capacitor connected to this pin and VSS.

 Bits 2-0 (f2 to f0): DQD threshold parameter.
 Note: To let the DQD report "good signal quality" the threshold parameter should be 4 in cases where the bitrate is close to the
 deviation. At higher deviation/bitrate settings, a higher threshold parameter can report "good signal quality" as well.
 */

#define RFM12_CMD_DATAFILTER 0xC228
#define RFM12_DATAFILTER_AL 0x80
#define RFM12_DATAFILTER_ML 0x40
#define RFM12_DATAFILTER_S 0x10

/*
 7. FIFO and Reset Mode Command
 Bit 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0 POR
 1 1 0 0 1 0 1 0 f3 f2 f1 f0 sp al ff dr CA80h

 Bits 7-4 (f3 to f0): FIFO IT level. The FIFO generates IT when the number of received data bits reaches this level.

 Bit 3 (sp): Select the length of the synchron pattern:
 sp		Byte1		Byte0 (POR) 	Synchron Pattern (Byte1+Byte0)
 0		2Dh			D4h				2DD4h
 1		Not used	D4h				D4h
 Note: Byte0 can be programmed by the Synchron Pattern Command.

 Bit 2 (al): Set the input of the FIFO fill start condition:
 al
 0 Synchron pattern
 1 Always fill

 Bit 1 (ff): FIFO fill will be enabled after synchron pattern reception. The FIFO fill stops when this bit is cleared.
 Bit 0 (dr): Disables the highly sensitive RESET mode.

 */
#define RFM12_CMD_FIFORESET 0xCA00
#define RFM12_FIFORESET_SP 0x08
#define RFM12_FIFORESET_AL 0x04
#define RFM12_FIFORESET_FF 0x02
#define RFM12_FIFORESET_DR 0x01

/*
 8. Synchron Pattern Command
 Bit 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0 POR
 1 1 0 0 1 1 1 0 b7 b6 b5 b4 b3 b2 b1 b0 CED4h
 The Byte0 used for synchron pattern detection can be reprogrammed by B <b7:b0>.
 */
#define RFM12_CMD_SYNCPATTERN 0xCE00

/*
 9. Receiver FIFO Read Command
 Bit 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0 POR
 1 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 B000h
 With this command, the controller can read 8 bits from the receiver FIFO. Bit 6 (ef) must be set in Configuration Setting Command.

 Note:: During FIFO access fSCK cannot be higher than fref /4, where fref is the crystal oscillator frequency. When the duty-cycle of the
 clock signal is not 50 % the shorter period of the clock pulse width should be at least 2/fref .
 */

#define RFM12_CMD_READ 0xB000

/*
 10. AFC Command
 Bit 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0 POR
 1 1 0 0 0 1 0 0 a1 a0 rl1 rl0 st fi oe en C4F7h

 Bit 7-6 (a1 to a0): Automatic operation mode selector:
 a1 a0
 0 0 Auto mode off (Strobe is controlled by microcontroller)
 0 1 Runs only once after each power-up
 1 0 Keep the foffset only during receiving (VDI=high)
 1 1 Keep the foffset value independently from the state of the VDI signal

 Bit 5-4 (rl1 to rl0): Range limit. Limits the value of the frequency offset register to the next values:
 rl1 rl0 Max deviation
 0 0 No restriction (+63 fres to -64 fres)
 0 1 +15 fres to -16 fres
 1 0 +7 fres to -8 fres
 1 1 +3 fres to -4 fres
 fres:
 433 MHz bands: 2.5 kHz
 868 MHz band: 5 kHz
 915 MHz band: 7.5 kHz

 Bit 3 (st): Strobe edge, when st goes to high, the actual latest calculated frequency error is stored into the offset register of the AFC
 block.
 Bit 2 (fi): Switches the circuit to high accuracy (fine) mode. In this case, the processing time is about twice as long, but the measurement
 uncertainty is about half.
 Bit 1 (oe): Enables the frequency offset register. It allows the addition of the offset register to the frequency control word of the PLL.
 Bit 0 (en): Enables the calculation of the offset frequency by the AFC circuit.

 In automatic operation mode (no strobe signal is needed from the microcontroller to update the output offset register) the AFC circuit
 is automatically enabled when the VDI indicates potential incoming signal during the whole measurement cycle and the circuit
 measures the same result in two subsequent cycles.
 There are three operation modes, examples from the possible application:
 1, (a1=0, a0=1) The circuit measures the frequency offset only once after power up. This way, extended TX-RX maximum distance
 can be achieved.
 Possible application:
 In the final application, when the user inserts the battery, the circuit measures and compensates for the frequency offset caused by
 the crystal tolerances. This method allows for the use of a cheaper quartz in the application and provides protection against tracking
 an interferer.
 2a, (a1=1, a0=0) The circuit automatically measures the frequency offset during an initial effective low data rate pattern –easier to
 receive- (i.e.: 00110011) of the package and changes the receiving frequency accordingly. The further part of the package can be
 received by the corrected frequency settings.
 2b, (a1=1, a0=0) The transmitter must transmit the first part of the packet with a step higher deviation and later there is a possibility
 of reducing it.
 In both cases (2a and 2b), when the VDI indicates poor receiving conditions (VDI goes low), the output register is automatically
 cleared. Use these settings when receiving signals from different transmitters transmitting in the same nominal frequencies.
 3, (a1=1, a0=1) It’s the same as 2a and 2b modes, but suggested to use when a receiver operates with only one transmitter. After a
 complete measuring cycle, the measured value is kept independently of the state of the VDI signal.
 */

#define RFM12_CMD_AFC 0xC400
#define RFM12_AFC_AUTO_OFF 0x00
#define RFM12_AFC_AUTO_ONCE 0x40
#define RFM12_AFC_AUTO_VDI 0x80
#define RFM12_AFC_AUTO_KEEP 0xC0
#define RFM12_AFC_LIMIT_OFF 0x00 /* 64 */
#define RFM12_AFC_LIMIT_16 0x10
#define RFM12_AFC_LIMIT_8 0x20
#define RFM12_AFC_LIMIT_4 0x30
#define RFM12_AFC_ST 0x08
#define RFM12_AFC_FI 0x04
#define RFM12_AFC_OE 0x02
#define RFM12_AFC_EN 0x01

/*
 11. TX Configuration Control Command
 Bit 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0 POR
 1 0 0 1 1 0 0 mp m3 m2 m1 m0 0 p2 p1 p0 9800h

 Bits 8-4 (mp, m3 to m0): FSK modulation parameters:
 The resulting output frequency can be calculated as:
 fout = f0 + (-1)SIGN * (M + 1) * (15 kHz)
 where:
 f0 is the channel center frequency (see the
 Frequency Setting Command)
 M is the four bit binary number <m3 : m0>
 SIGN = (mp) XOR FSK

 Bits 2-0 (p2 to p0): Output power:
 p2 p1 p0 Relative Output Power [dB]
 0 0 0 0
 0 0 1 -2.5
 0 1 0 -5
 0 1 1 -7.5
 1 0 0 -10
 1 0 1 -12.5
 1 1 0 -15
 1 1 1 -17.5

 */

#define RFM12_CMD_TXCONF 0x9800
#define RFM12_TXCONF_MP 0x100
#define RFM12_TXCONF_POWER_0 	0x00
#define RFM12_TXCONF_POWER_3 	0x01
#define RFM12_TXCONF_POWER_6 	0x02
#define RFM12_TXCONF_POWER_9 	0x03
#define RFM12_TXCONF_POWER_12 	0x04
#define RFM12_TXCONF_POWER_15   0x05
#define RFM12_TXCONF_POWER_18 	0x06
#define RFM12_TXCONF_POWER_21   0x07
#define RFM12_TXCONF_FSK_MASK   0xf0
#define RFM12_TXCONF_FS_CALC(f) (((f/15000UL)-1)<<4)
#define RFM12_TXCONF_MASK 0x01F7
#define RFM12_TXCONF_POWER_MASK 0x07

/*
 12. PLL Setting Command
 Bit 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0 POR
 1 1 0 0 1 1 0 0 0 ob1 ob0 0 ddy ddit 1 bw0 CC67h
 Note: POR default setting of the register carefully selected to cover almost all typical applications.
 Bit 6-5 (ob1-ob0): Microcontroller output clock buffer rise and fall time control.
 ob1 ob0 Selected uC CLK frequency
 0 0 5 or 10 MHz (recommended)
 0 1 3.3 MHz
 1 X 2.5 MHz or less

 (Typ conditions: Top = 27 oC; Vdd = Voc = 2.7 V, Crystal ESR = 30 Ohm)

 Bit 3 (ddy): Switches on the delay in the phase detector when this bit is set.
 Bit 2 (ddit): When set, disables the dithering in the PLL loop.
 Bit 0 (bw0): PLL bandwidth can be set for optimal TX RF performance.

 bw0		Max bit rate [kbps]		Phase noise at 1MHz offset [dBc/Hz]
 0		86.2					-107
 1		256						-102

 Note: Needed for optimization of the RF
 performance. Optimal settings can vary
 according to the external load capacitance.
 */

#define RFM12_CMD_PLL	0xCC02
#define RFM12_PLL_DDY	0x08
#define RFM12_PLL_DDIT	0x04
#define RFM12_PLL_BW0	0x01

/*
 13. Transmitter Register Write Command
 Bit 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0 POR
 1 0 1 1 1 0 0 0 t7 t6 t5 t4 t3 t2 t1 t0 B8AAh
 With this command, the controller can write 8 bits (t7 to t0) to the transmitter data register. Bit 7 (el) must be set in Configuration
 Setting Command.
 */

#define RFM12_CMD_TX 0xB800

/*
 14. Wake-Up Timer Command
 Bit 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0 POR
 1 1 1 r4 r3 r2 r1 r0 m7 m6 m5 m4 m3 m2 m1 m0 E196h
 The wake-up time period can be calculated by (m7 to m0) and (r4 to r0):
 Twake-up = 1.03 * M * 2R + 0.5 [ms]
 Note:
 • For continual operation the ew bit should be cleared and set at the end of every cycle.
 • For future compatibility, use R in a range of 0 and 29.
 */
#define RFM12_CMD_WAKEUP 0xE000

/*
 15. Low Duty-Cycle Command
 Bit 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0 POR
 1 1 0 0 1 0 0 0 d6 d5 d4 d3 d2 d1 d0 en C80Eh
 With this command, Low Duty-Cycle operation can be set in order to decrease the average power consumption in receiver mode.
 The time cycle is determined by the Wake-Up Timer Command.
 The Duty-Cycle can be calculated by using (d6 to d0) and M. (M is parameter in a Wake-Up Timer Command.)
 Duty-Cycle= (D * 2 +1) / M *100%
 The on-cycle is automatically extended while DQD indicates good received signal condition (FSK transmission is detected in the
 frequency range determined by Frequency Setting Command plus and minus the baseband filter bandwidth determined by the
 Receiver Control Command).
 */
#define RFM12_CMD_DUTYCYCLE 0xC800
#define RFM12_DUTYCYCLE_ENABLE 0x01

/*
 16. Low Battery Detector and Microcontroller Clock Divider Command
 Bit 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0 POR
 1 1 0 0 0 0 0 0 d2 d1 d0 0 v3 v2 v1 v0 C000h
 The 4 bit parameter (v3 to v0) represents the value V, which defines the threshold voltage Vlb of the detector:
 Vlb= 2.25 + V * 0.1 [V]
 Clock divider configuration:
 Clock Output
 Frequency [MHz]
 0 0 0 1
 0 0 1 1.25
 0 1 0 1.66
 0 1 1 2
 1 0 0 2.5
 1 0 1 3.33
 1 1 0 5
 1 1 1 10
 d2 d1 d0
 The low battery detector and the clock output can be enabled or disabled by bits eb and dc, respectively, using the Power
 Management Command.
 */
#define RFM12_CMD_LBDMCD 0xC000

#define RFM12_LBD_VOLTAGE_2V2 0
#define RFM12_LBD_VOLTAGE_2V3 1
#define RFM12_LBD_VOLTAGE_2V4 2
#define RFM12_LBD_VOLTAGE_2V5 3
#define RFM12_LBD_VOLTAGE_2V6 4
#define RFM12_LBD_VOLTAGE_2V7 5
#define RFM12_LBD_VOLTAGE_2V8 6
#define RFM12_LBD_VOLTAGE_2V9 7
#define RFM12_LBD_VOLTAGE_3V0 8
#define RFM12_LBD_VOLTAGE_3V1 9
#define RFM12_LBD_VOLTAGE_3V2 10
#define RFM12_LBD_VOLTAGE_3V3 11
#define RFM12_LBD_VOLTAGE_3V4 12
#define RFM12_LBD_VOLTAGE_3V5 13
#define RFM12_LBD_VOLTAGE_3V6 14
#define RFM12_LBD_VOLTAGE_3V7 15

#define RFM12_CLOCK_OUT_FREQUENCY_1_00_MHz  (0<<5)
#define RFM12_CLOCK_OUT_FREQUENCY_1_25_MHz  (1<<5)
#define RFM12_CLOCK_OUT_FREQUENCY_1_66_MHz  (2<<5)
#define RFM12_CLOCK_OUT_FREQUENCY_2_00_MHz  (3<<5)
#define RFM12_CLOCK_OUT_FREQUENCY_2_50_MHz  (4<<5)
#define RFM12_CLOCK_OUT_FREQUENCY_3_33_MHz  (5<<5)
#define RFM12_CLOCK_OUT_FREQUENCY_5_00_MHz  (6<<5)
#define RFM12_CLOCK_OUT_FREQUENCY_10_00_MHz (7<<5)

/*
 17. Status Read Command
 The read command starts with a zero, whereas all other control commands start with a one. If a read command is identified, the
 status bits will be clocked out on the SDO pin as follows:

 bitnumber
 15	RGIT TX register is ready to receive the next byte (Can be cleared by Transmitter Register Write Command)
 FFIT The number of data bits in the RX FIFO has reached the pre-programmed limit (Can be cleared by any of the
 FIFO read methods)
 14	POR Power-on reset (Cleared after Status Read Command)
 13	RGUR TX register under run, register over write (Cleared after Status Read Command)
 FFOV RX FIFO overflow (Cleared after Status Read Command)
 12	WKUP Wake-up timer overflow (Cleared after Status Read Command)
 11	EXT Logic level on interrupt pin (pin 16) changed to low (Cleared after Status Read Command)
 10	LBD Low battery detect, the power supply voltage is below the pre-programmed limit
 9	FFEM FIFO is empty
 8	ATS Antenna tuning circuit detected strong enough RF signal
 RSSI The strength of the incoming signal is above the pre-programmed limit
 7	DQD Data quality detector output
 6	CRL Clock recovery locked
 5	ATGL Toggling in each AFC cycle
 4	OFFS(6) MSB of the measured frequency offset (sign of the offset value)
 3	OFFS(3) -OFFS(0) Offset value to be added to the value of the frequency control parameter (Four LSB bits)
 2
 1
 0
 */

#define RFM12_CMD_STATUS	0x0000
#define RFM12_STATUS_RGIT	0x8000
#define RFM12_STATUS_FFIT	0x8000
#define RFM12_STATUS_POR	0x4000
#define RFM12_STATUS_RGUR	0x2000
#define RFM12_STATUS_FFOV	0x2000
#define RFM12_STATUS_WKUP	0x1000
#define RFM12_STATUS_EXT	0x0800
#define RFM12_STATUS_LBD	0x0400
#define RFM12_STATUS_FFEM	0x0200
#define RFM12_STATUS_ATS	0x0100
#define RFM12_STATUS_RSSI	0x0100
#define RFM12_STATUS_DQD	0x0080
#define RFM12_STATUS_CRL	0x0040
#define RFM12_STATUS_ATGL	0x0020

/* undocumented software reset command for the rf12
 */
#define RFM12_RESET		0xffff

#endif // ENABLE_GPL_RFM12B
//--
#ifdef ENABLE_MIRF24
#define mirf_csnHi        mirf_CSN_PORT |=  (1<<mirf_CSN_PIN);
#define mirf_csnLow       mirf_CSN_PORT &= ~(1<<mirf_CSN_PIN);
#define mirf_ceHi         mirf_CE_PORT  |=  (1<<mirf_CE_PIN);
#define mirf_ceLow        mirf_CE_PORT  &= ~(1<<mirf_CE_PIN);

uint8_t mirf_PTX;

void mirf_init();
void mirf_config();
void mirf_send(uint8_t *value);
void mirf_setRADDR(uint8_t * adr);
void mirf_setTADDR(uint8_t * adr);
uint8_t mirf_dataReady();
uint8_t mirf_isSending();
uint8_t mirf_rxFifoEmpty();
uint8_t mirf_txFifoEmpty();
void mirf_getData(uint8_t * data);
uint8_t mirf_getStatus();

void mirf_transmitSync(uint8_t *dataout,uint8_t len);
void mirf_transferSync(uint8_t *dataout,uint8_t *datain,uint8_t len);
void mirf_configRegister(uint8_t reg, uint8_t value);
void mirf_readRegister(uint8_t reg, uint8_t * value, uint8_t len);
void mirf_writeRegister(uint8_t reg, uint8_t * value, uint8_t len);
void mirf_powerUpRx();
void mirf_powerUpTx();
void mirf_powerDown();
void mirf_flushRx();

#endif // ENABLE_MIRF24
//--
#ifdef ENABLE_ONE_WIRE
#define OW_MATCH_ROM    0x55
#define OW_SKIP_ROM     0xCC
#define OW_SEARCH_ROM   0xF0

#define OW_SEARCH_FIRST 0xFF        // start new search
#define OW_PRESENCE_ERR 0xFF
#define OW_DATA_ERR     0xFE
#define OW_LAST_DEVICE  0x00        // last device found
// rom-code size including CRC
#define OW_ROMCODE_SIZE 8

uint8_t ow_reset(void);

uint8_t ow_bit_io(uint8_t b);
uint8_t ow_byte_wr(uint8_t b);
uint8_t ow_byte_rd(void);

uint8_t ow_rom_search(uint8_t diff, uint8_t *id);

void ow_command(uint8_t command, uint8_t *id);
void ow_command_with_parasite_enable(uint8_t command, uint8_t *id);

void ow_parasite_enable(void);
void ow_parasite_disable(void);
uint8_t ow_input_pin_state(void);

#ifndef OW_ONE_BUS
void ow_set_bus( volatile uint8_t* in,
		volatile uint8_t* out,
		volatile uint8_t* ddr,
		uint8_t pin );
#endif

#ifdef ENABLE_DS18_2_
uint8_t crc8(uint8_t* data, uint16_t number_of_bytes_in_data);
//

// return values
#define DS18X20_OK                0x00
#define DS18X20_ERROR             0x01
#define DS18X20_START_FAIL        0x02
#define DS18X20_ERROR_CRC         0x03

#define DS18X20_INVALID_DECICELSIUS  2000

#define DS18X20_POWER_PARASITE    0x00
#define DS18X20_POWER_EXTERN      0x01

#define DS18X20_CONVERSION_DONE   0x00
#define DS18X20_CONVERTING        0x01

// DS18X20 specific values (see datasheet)
#define DS18S20_FAMILY_CODE       0x10
#define DS18B20_FAMILY_CODE       0x28
#define DS1822_FAMILY_CODE        0x22

#define DS18X20_CONVERT_T         0x44
#define DS18X20_READ              0xBE
#define DS18X20_WRITE             0x4E
#define DS18X20_EE_WRITE          0x48
#define DS18X20_EE_RECALL         0xB8
#define DS18X20_READ_POWER_SUPPLY 0xB4

#define DS18B20_CONF_REG          4
#define DS18B20_9_BIT             0
#define DS18B20_10_BIT            (1<<5)
#define DS18B20_11_BIT            (1<<6)
#define DS18B20_12_BIT            ((1<<6)|(1<<5))
#define DS18B20_RES_MASK          ((1<<6)|(1<<5))

// undefined bits in LSB if 18B20 != 12bit
#define DS18B20_9_BIT_UNDF        ((1<<0)|(1<<1)|(1<<2))
#define DS18B20_10_BIT_UNDF       ((1<<0)|(1<<1))
#define DS18B20_11_BIT_UNDF       ((1<<0))
#define DS18B20_12_BIT_UNDF       0

// conversion times in milliseconds
#define DS18B20_TCONV_12BIT       750
#define DS18B20_TCONV_11BIT       DS18B20_TCONV_12_BIT/2
#define DS18B20_TCONV_10BIT       DS18B20_TCONV_12_BIT/4
#define DS18B20_TCONV_9BIT        DS18B20_TCONV_12_BIT/8
#define DS18S20_TCONV             DS18B20_TCONV_12_BIT

// constant to convert the fraction bits to cel*(10^-4)
#define DS18X20_FRACCONV          625

// scratchpad size in bytes
#define DS18X20_SP_SIZE           9

// DS18X20 EEPROM-Support
#define DS18X20_WRITE_SCRATCHPAD  0x4E
#define DS18X20_COPY_SCRATCHPAD   0x48
#define DS18X20_RECALL_E2         0xB8
#define DS18X20_COPYSP_DELAY      10 /* ms */
#define DS18X20_TH_REG            2
#define DS18X20_TL_REG            3

#define DS18X20_DECIMAL_CHAR      '.'

extern uint8_t DS18X20_find_sensor(uint8_t *diff, uint8_t id[]);
extern uint8_t DS18X20_get_power_status(uint8_t id[]);
extern uint8_t DS18X20_start_meas(uint8_t with_external, uint8_t id[]);
// returns 1 if conversion is in progress, 0 if finished
// not available when parasite powered
extern uint8_t DS18X20_conversion_in_progress(void);

#if DS18X20_DECICELSIUS
extern uint8_t DS18X20_read_decicelsius(uint8_t id[], int16_t *decicelsius);
extern uint8_t DS18X20_read_decicelsius_single(uint8_t familycode,
		int16_t *decicelsius);
extern uint8_t DS18X20_format_from_decicelsius(int16_t decicelsius, int8_t s[],
		uint8_t n);
#endif // DS18X20_DECICELSIUS
#if DS18X20_MAX_RESOLUTION
// temperature unit for max. resolution is °C * 10e-4
// examples: -250625 -> -25.0625°C, 1250000 -> 125.0000 °C
extern uint8_t DS18X20_read_maxres(uint8_t id[], int32_t *temperaturevalue);
extern uint8_t DS18X20_read_maxres_single(uint8_t familycode,
		int32_t *temperaturevalue);
extern uint8_t DS18X20_format_from_maxres(int32_t temperaturevalue, char s[],
		uint8_t n);
#endif // DS18X20_MAX_RESOLUTION
#if DS18X20_EEPROMSUPPORT
// write th, tl and config-register to scratchpad (config ignored on DS18S20)
uint8_t DS18X20_write_scratchpad(uint8_t id[], uint8_t th, uint8_t tl,
		uint8_t conf);
// read scratchpad into array SP
uint8_t DS18X20_read_scratchpad(uint8_t id[], uint8_t sp[], uint8_t n);
// copy values int scratchpad into DS18x20 eeprom
uint8_t DS18X20_scratchpad_to_eeprom(uint8_t with_power_extern, uint8_t id[]);
// copy values from DS18x20 eeprom into scratchpad
uint8_t DS18X20_eeprom_to_scratchpad(uint8_t id[]);
#endif // DS18X20_EEPROMSUPPORT
#if DS18X20_VERBOSE
extern void DS18X20_show_id_uart(uint8_t *id, size_t n);
extern uint8_t DS18X20_read_meas_all_verbose(void);
#endif // DS18X20_VERBOSE
#endif // ENABLE_DS18_20_
#endif // ENABLE_ONE_WIRE
//--
#ifdef ENABLE_CONVERSION // various conversion functions
uint8_t bcd2bin(uint8_t bcd);
uint8_t bin2bcd(uint8_t bin);
uint8_t nibble2hex(uint8_t val);
void byte2hex(uint8_t val, int8_t *s);
void word2hex(uint16_t val, int8_t *s);
void double2hex(uint32_t val, int8_t *s);
void byte2dec(uint8_t val, int8_t *s);
#endif // end Conversion
//--
#ifdef ENABLE_XPRINTF
#if _USE_XFUNC_OUT
#define xdev_out(func) xfunc_out = (void(*)(unsigned char))(func)
extern void (*xfunc_out)(unsigned char);
void xputc (char c);
void xputs (const char* str);
void xfputs (void (*func)(unsigned char), const char* str);
void xprintf (const char* fmt, ...);
void xprintf_P (const char* fmt, ...);
void xsprintf (char* buff, const char* fmt, ...);
void xsprintf_P (char* buff, const char* fmt, ...);
void xfprintf (void (*func)(unsigned char), const char* fmt, ...);
void put_dump (const void* buff, unsigned long addr, int len, int width);
#define DW_CHAR		sizeof(char)
#define DW_SHORT	sizeof(short)
#define DW_LONG		sizeof(long)
#endif

#if _USE_XFUNC_IN
#define xdev_in(func) xfunc_in = (unsigned char(*)(void))(func)
extern unsigned char (*xfunc_in)(void);
int xgets (char* buff, int len);
int xfgets (unsigned char (*func)(void), char* buff, int len);
int xatoi (char** str, long* res);
#endif
#endif
//--
#ifdef ENABLE_SERIAL // serial with interrupts...
#ifdef USART0_RX_vect // if uC with more than one Serial peripheral ...
#define UART0_ISR_VECT USART0_RX_vect
#elif defined(USART_RXC_vect)
#define UART0_ISR_VECT USART_RXC_vect
#else
#define UART0_ISR_VECT USART_RX_vect
#endif

void serial_init(void);
uint8_t serial_getchar(void);
uint8_t serial_available(void);
void serial_flush(void);
#endif // end Serial
#ifdef ENABLE_SERIAL_POLL
void serial_init(void);
uint8_t serial_getchar(void);
#endif // end Serial Poll
#if defined(ENABLE_SERIAL) || defined(ENABLE_SERIAL_POLL)

#if defined(USART_RXC_vect)
#define UART0_DATA	UDR
#else
#define UART0_DATA	UDR0
#endif

void serial_putchar(uint8_t data);
#define serial_putc(c__) serial_putchar((uint8_t)c__)
void serial_putstr(int8_t *s); // send a string from SRAM
#define serial_puts(s__) serial_putstr((int8_t *)s__)
void serial_putstr_f(int8_t *s); // send a string from Flash memory
#define serial_puts_f(s__) serial_putstr_f((int8_t *)PSTR(s__))
//void serial_putstr_e(uint8_t *s);   // send a string from EEPROM
void serial_putint(int value);
void serial_putU08(uint8_t value); // display a byte
void serial_puthexU08(uint8_t value); // display a byte in hex value
void serial_puthexU16(uint16_t value); // display a word in hex value
void serial_puthexU32(uint32_t value); // display a double in hex value

#define TX_NEWLINE {serial_putc(0x0d); serial_putc(0x0a);}
// required by Dharmani's SD_Card functions...
#endif // ENABLE_SERIAL_POLL
#ifdef ENABLE_IR
#define IR_BUFFER_SIZE		16
void ir_init(void);
uint8_t ir_get(void);
uint8_t ir_get_nonblock(void);
#endif // end IR
#ifdef ENABLE_PWMSERVO
void pwmservo_init(uint8_t pwmno);
void pwmservo_set(uint8_t servo, uint8_t pwmval); /* pwmval 0-255 */
/* 16bit timer counter values for PWM */
#define SERVO_MIN_POS16		1400
#define SERVO_MID_POS16		3000
#define SERVO_MAX_POS16		4600
/* 8bit timer counter values for PWM */
#define SERVO_MIN_POS8		10
#define SERVO_MID_POS8		25
#define SERVO_MAX_POS8		40
#endif

#ifdef ENABLE_PWM
void pwm_init(uint8_t pwmnum);
void pwm_set(uint8_t pwmnum, uint8_t pwmval); /* pwmval 0-255 */
#endif

#ifdef ENABLE_FREQMEASURE
#if defined(CAPTURE_USE_TIMER1)

static uint8_t saveTCCR1A, saveTCCR1B;

static inline void capture_init(void) {
	saveTCCR1A = TCCR1A;
	saveTCCR1B = TCCR1B;
	TCCR1B = 0;
	TCCR1A = 0;
	TCNT1 = 0;
	TIFR1 = (1 << ICF1) | (1 << TOV1);
	TIMSK1 = (1 << ICIE1) | (1 << TOIE1);
}

static inline void capture_start(void) {
	TCCR1B = (1 << ICNC1) | (1 << ICES1) | (1 << CS10);
}

static inline uint16_t capture_read(void) {
	return ICR1;
}

static inline uint8_t capture_overflow(void) {
	return TIFR1 & (1 << TOV1);
}

static inline uint8_t capture_overflow_reset(void) {
	return TIFR1 = (1 << TOV1);
}

static inline void capture_shutdown(void) {
	TCCR1B = 0;
	TIMSK1 = 0;
	TCCR1A = saveTCCR1A;
	TCCR1B = saveTCCR1B;
}

#define TIMER_OVERFLOW_VECTOR  TIMER1_OVF_vect
#define TIMER_CAPTURE_VECTOR   TIMER1_CAPT_vect

#elif defined(CAPTURE_USE_TIMER3)

static uint8_t saveTCCR3A, saveTCCR3B;

static inline void capture_init(void)
{
	saveTCCR3A = TCCR3A;
	saveTCCR3B = TCCR3B;
	TCCR3B = 0;
	TCCR3A = 0;
	TCNT3 = 0;
	TIFR3 = (1<<ICF3) | (1<<TOV3);
	TIMSK3 = (1<<ICIE3) | (1<<TOIE3);
}

static inline void capture_start(void)
{
	TCCR3B = (1<<ICNC3) | (1<<ICES3) | (1<<CS30);
}

static inline uint16_t capture_read(void)
{
	return ICR3;
}

static inline uint8_t capture_overflow(void)
{
	return TIFR3 & (1<<TOV3);
}

static inline uint8_t capture_overflow_reset(void)
{
	return TIFR3 = (1<<TOV3);
}

static inline void capture_shutdown(void)
{
	TCCR3B = 0;
	TIMSK3 = 0;
	TCCR3A = saveTCCR3A;
	TCCR3B = saveTCCR3B;
}

#define TIMER_OVERFLOW_VECTOR  TIMER3_OVF_vect
#define TIMER_CAPTURE_VECTOR   TIMER3_CAPT_vect

#elif defined(CAPTURE_USE_TIMER4)

static uint8_t saveTCCR4A, saveTCCR4B;

static inline void capture_init(void)
{
	saveTCCR4A = TCCR4A;
	saveTCCR4B = TCCR4B;
	TCCR4B = 0;
	TCCR4A = 0;
	TCNT4 = 0;
	TIFR4 = (1<<ICF4) | (1<<TOV4);
	TIMSK4 = (1<<ICIE4) | (1<<TOIE4);
}

static inline void capture_start(void)
{
	TCCR4B = (1<<ICNC4) | (1<<ICES4) | (1<<CS40);
}

static inline uint16_t capture_read(void)
{
	return ICR4;
}

static inline uint8_t capture_overflow(void)
{
	return TIFR4 & (1<<TOV4);
}

static inline uint8_t capture_overflow_reset(void)
{
	return TIFR4 = (1<<TOV4);
}

static inline void capture_shutdown(void)
{
	TCCR4B = 0;
	TIMSK4 = 0;
	TCCR4A = saveTCCR4A;
	TCCR4B = saveTCCR4B;
}

#define TIMER_OVERFLOW_VECTOR  TIMER4_OVF_vect
#define TIMER_CAPTURE_VECTOR   TIMER4_CAPT_vect

#elif defined(CAPTURE_USE_TIMER5)

static uint8_t saveTCCR5A, saveTCCR5B;

static inline void capture_init(void)
{
	saveTCCR5A = TCCR5A;
	saveTCCR5B = TCCR5B;
	TCCR5B = 0;
	TCCR5A = 0;
	TCNT5 = 0;
	TIFR5 = (1<<ICF5) | (1<<TOV5);
	TIMSK5 = (1<<ICIE5) | (1<<TOIE5);
}

static inline void capture_start(void)
{
	TCCR5B = (1<<ICNC5) | (1<<ICES5) | (1<<CS50);
}

static inline uint16_t capture_read(void)
{
	return ICR5;
}

static inline uint8_t capture_overflow(void)
{
	return TIFR5 & (1<<TOV5);
}

static inline uint8_t capture_overflow_reset(void)
{
	return TIFR5 = (1<<TOV5);
}

static inline void capture_shutdown(void)
{
	TCCR5B = 0;
	TIMSK5 = 0;
	TCCR5A = saveTCCR5A;
	TCCR5B = saveTCCR5B;
}

#define TIMER_OVERFLOW_VECTOR  TIMER5_OVF_vect
#define TIMER_CAPTURE_VECTOR   TIMER5_CAPT_vect
#endif // CAPTURE_USE_***
// Main functions
void FreqMeasure_begin(void);
uint8_t FreqMeasure_available(void);
uint32_t FreqMeasure_read(void);
void FreqMeasure_end(void);

#endif //ENABLE_FREQMEASURE
#ifdef ENABLE_ADC
// this is NOT user zone
//define adc voltage reference
#define VREF_OFF_INT_AREF       0
#define VREF_AVCC_CAP_AREF      1
#define VREF_INT_1_1V_CAP_AREF  2
#define VREF_INT_2_56V_CAP_AREF 3
// define adc prescaler
#define ADC_PRESCALER_0   0
#define ADC_PRESCALER_2   1
#define ADC_PRESCALER_4   2
#define ADC_PRESCALER_8   3
#define ADC_PRESCALER_16  4
#define ADC_PRESCALER_32  5
#define ADC_PRESCALER_64  6
#define ADC_PRESCALER_128 7

void adc_init(uint8_t adc_reference, uint8_t adc_prescaler);
uint16_t adc_get(uint8_t adcnum);
void adc_poweroff_digital_pinbuffer(uint8_t adcnum);
#endif //adc
#ifdef ENABLE_LCD
// this is NOT user zone
#ifdef LCD_1X8
#define LCD_COLUMN      8
#define LCD_LINE        1
#define LCD_LINE1       0x80
#endif

#ifdef LCD_1X16
#define LCD_COLUMN      16
#define LCD_LINE        1
#define LCD_LINE1       0x80
#endif

#ifdef LCD_1X20
#define LCD_COLUMN      20
#define LCD_LINE        1
#define LCD_LINE1       0x80
#endif

#ifdef LCD_1X40
#define LCD_COLUMN      40
#define LCD_LINE        1
#define LCD_LINE1       0x80
#endif

#ifdef LCD_2X8
#define LCD_COLUMN      8
#define LCD_LINE        2
#define LCD_LINE1       0x80
#define LCD_LINE2       (0x80 + 0x40)
#endif

#ifdef LCD_2X12
#define LCD_COLUMN      12
#define LCD_LINE        2
#define LCD_LINE1       0x80
#define LCD_LINE2       (0x80 + 0x40)
#endif

#ifdef LCD_2X16
#define LCD_COLUMN      16
#define LCD_LINE        2
#define LCD_LINE1       0x80
#define LCD_LINE2       (0x80 + 0x40)
#endif

#ifdef LCD_2X20
#define LCD_COLUMN      20
#define LCD_LINE        2
#define LCD_LINE1       0x80
#define LCD_LINE2       (0x80 + 0x40)
#endif

#ifdef LCD_2X24
#define LCD_COLUMN      24
#define LCD_LINE        2
#define LCD_LINE1       0x80
#define LCD_LINE2       (0x80 + 0x40)
#endif

#ifdef LCD_2X40
#define LCD_COLUMN      40
#define LCD_LINE        2
#define LCD_LINE1       0x80
#define LCD_LINE2       (0x80 + 0x40)
#endif

#ifdef LCD_4X16
#define LCD_COLUMN      16
#define LCD_LINE        4
#define LCD_LINE1       0x80
#define LCD_LINE2       (0x80 + 0x40)
#define LCD_LINE3       (0x80 + 0x10)
#define LCD_LINE4       (0x80 + 0x50)
#endif

#ifdef LCD_4X20
#define LCD_COLUMN      20
#define LCD_LINE        4
#define LCD_LINE1       0x80
#define LCD_LINE2       (0x80 + 0x40)
#define LCD_LINE3       (0x80 + 0x14)
#define LCD_LINE4       (0x80 + 0x54)
#endif

#define	LCD_TIME_ENA    1.0             // 1µs
#define LCD_TIME_DAT    50.0            // 50µs
#define LCD_TIME_CLR    2000.0          // 2ms
#ifdef LCD_LINE4
#define lcd_set_cursor(x, y)    lcd_command((x) + ((y==3) ? LCD_LINE4 : \
                                           (y==2) ? LCD_LINE3 : \
                                           (y==1) ? LCD_LINE2 : LCD_LINE1 ))
#else
#ifdef LCD_LINE3
#define lcd_set_cursor(x, y)    lcd_command((x) + ((y==2) ? LCD_LINE3 : \
                                           (y==1) ? LCD_LINE2 : LCD_LINE1 ))
#else
#ifdef LCD_LINE2
#define lcd_set_cursor(x, y)    lcd_command((x) + ((y==1) ? LCD_LINE2 : LCD_LINE1 ))
#else
#define lcd_set_cursor(x, y)    lcd_command((x) + LCD_LINE1 )
#endif
#endif
#endif

void lcd_init(void);
void lcd_clear(void);
void lcd_home();
void lcd_putchar(uint8_t d);
#define lcd_putc(c__) lcd_putchar((uint8_t)c__)
void lcd_putstr(int8_t *);
#define lcd_puts(s__) lcd_putstr((int8_t *)s__)
void lcd_putstr_f(int8_t *);
#define lcd_puts_f(s__) lcd_putstr_f((int8_t *)PSTR(s__))
void lcd_putint(int value);
void lcd_putU08(uint8_t value); // display a byte
void lcd_puthexU08(uint8_t value);// display a byte in hex value
void lcd_puthexU16(uint16_t value);// display a word in hex value
void lcd_blank(uint8_t len);// blank n digits
void lcd_cursor_on(void);
void lcd_cursor_off(void);
void lcd_blink_on(void);
void lcd_blink_off(void);
void lcd_display_on(void);
void lcd_display_off(void);

// Private
/*
 static void lcd_nibble( uint8_t d );
 static void lcd_byte( uint8_t d );
 */
void lcd_command(uint8_t d);

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

uint8_t _displayfunction;
uint8_t _displaycontrol;
uint8_t _displaymode;

#endif //ENABLE_LCD
#ifdef ENABLE_GLCD
// this is NOT user zone
// Chips
#define GLCD_CHIP1            0x00
#define GLCD_CHIP2            0x01

// Commands
#define GLCD_ON               0x3F
#define GLCD_OFF              0x3E
#define GLCD_SET_ADD          0x40
#define GLCD_SET_PAGE         0xB8
#define GLCD_DISP_START       0xC0

// Colors
#define GLCD_BLACK            0xFF
#define GLCD_WHITE            0x00

// Font Indices
#define GLCD_FONT_LENGTH         0
#define GLCD_FONT_FIXED_WIDTH    2
#define GLCD_FONT_HEIGHT         3
#define GLCD_FONT_FIRST_CHAR     4
#define GLCD_FONT_CHAR_COUNT     5
#define GLCD_FONT_WIDTH_TABLE    6

// Uncomment for slow drawing
// #define DEBUG

typedef struct {
	uint8_t x;
	uint8_t y;
	uint8_t page;
}glcdCoord;

typedef uint8_t (*ks0108FontCallback)(const uint8_t*);

//
// Function Prototypes
//

// Graphic Functions
void GLCD_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);
void GLCD_DrawRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color);
void GLCD_DrawRoundRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t radius, uint8_t color);
void GLCD_FillRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color);
void GLCD_InvertRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height);
void GLCD_SetInverted(uint8_t invert);
void GLCD_SetDot(uint8_t x, uint8_t y, uint8_t color);

#define GLCD_DrawVertLine(x, y, length, color) {GLCD_FillRect(x, y, 0, length, color);}
#define GLCD_DrawHoriLine(x, y, length, color) {GLCD_FillRect(x, y, length, 0, color);}
#define GLCD_DrawCircle(xCenter, yCenter, radius, color) {GLCD_DrawRoundRect(xCenter-radius, yCenter-radius, 2*radius, 2*radius, radius, color);}
#define GLCD_ClearScreen() {GLCD_FillRect(0, 0, 127, 63, GLCD_WHITE);}

// Font Functions
uint8_t GLCD_ReadFontData(const uint8_t* ptr);//Standard Read Callback
void GLCD_SelectFont(const uint8_t* font, ks0108FontCallback callback, uint8_t color);
int GLCD_PutChar(char c);
void GLCD_Puts(char* str);
void GLCD_Puts_P(PGM_P str);
uint8_t GLCD_CharWidth(char c);
uint16_t GLCD_StringWidth(char* str);
uint16_t GLCD_StringWidth_P(PGM_P str);

// Control Functions
void GLCD_GotoXY(uint8_t x, uint8_t y);
void GLCD_Init(uint8_t invert);
inline uint8_t GLCD_ReadData(void);
void GLCD_WriteCommand(uint8_t cmd, uint8_t chip);
void GLCD_WriteData(uint8_t data);

#endif //ENABLE_GLCD
//--
#ifdef ENABLE_7SEG
// this is NOT user zone
typedef enum mydigit {
	SELECT_DIGIT_ONE = 1,
	SELECT_DIGIT_TWO = 2,
	SELECT_DIGIT_THREE = 4,
	SELECT_DIGIT_FOUR = 8,
	SELECT_DIGIT_FIVE = 16,
	SELECT_DIGIT_SIX = 32,
	SELECT_DIGIT_SEVEN = 64,
	SELECT_DIGIT_EIGHT = 128
}MyDigit;

// Define the segments
#define SEG_A 0b00000001   //  1
#define SEG_B 0b00000010   //  2
#define SEG_C 0b00000100   //  4
#define SEG_D 0b00001000   //  8
#define SEG_E 0b00010000   // 16
#define SEG_F 0b00100000   // 32
#define SEG_G 0b01000000   // 64
#ifdef SEG_DP_PORT
#define SEG_DP 0b10000000  //128
#endif

void seg_init(void);
uint8_t seg_digit_from_array(uint8_t index);
void seg_nibble(uint8_t);
void seg_common_select(uint8_t);
void seg_select_digit(uint8_t, uint8_t);
#endif //7-seg
//--
#ifdef ENABLE_PCF8583 //it requires Hardware or Software I2C
#ifdef PCF8583_USE_TWI
#ifndef ENABLE_TWI
#error "This needs TWI so, uncomment ENABLE_TWI from library header"
#endif
#endif
uint8_t PCF8583_read(uint8_t address);
void PCF8583_write(uint8_t address, uint8_t data);
uint8_t PCF8583_read_bcd(uint8_t address);
void PCF8583_write_bcd(uint8_t address, uint8_t data);
uint8_t PCF8583_get_status(void);
void PCF8583_init(void);
void PCF8583_stop(void);
void PCF8583_start(void);
void PCF8583_hold_off(void);
void PCF8583_hold_on(void);
void PCF8583_alarm_off(void);
void PCF8583_alarm_on(void);
void PCF8583_write_word(uint8_t address, uint16_t data);
void PCF8583_write_date(uint8_t address, uint8_t day, uint16_t year);
void PCF8583_get_time(uint8_t *hour, uint8_t *min, uint8_t *sec, uint8_t *hsec);
void PCF8583_set_time(uint8_t hour, uint8_t min, uint8_t sec, uint8_t hsec);
void PCF8583_get_date(uint8_t *day, uint8_t *month, uint16_t *year);
void PCF8583_set_date(uint8_t day, uint8_t month, uint16_t year);
void PCF8583_get_alarm_time(uint8_t *hour, uint8_t *min, uint8_t *sec,
		uint8_t *hsec);
void PCF8583_set_alarm_time(uint8_t hour, uint8_t min, uint8_t sec,
		uint8_t hsec);
void PCF8583_get_alarm_date(uint8_t *day, uint8_t *month);
void PCF8583_set_alarm_date(uint8_t day, uint8_t month);
#endif //ENABLE_PCF8583
//Now, the following list of microcontrollers must meet some conditions:
// 1. Must be supported by avr-gcc 4.7.2 and avr-libc 1.8 (svn 2291 or newer)
// 2. Must be supported by avrdude (well, some definitions can be added
//    without problems - e.g., for ATmega88PA starting from ATmega88P).
#if	(defined(__AVR_ATmega48__)   || \
		defined(__AVR_ATmega48P__)     || \
		defined(__AVR_ATmega48PA__)    || \
		defined(__AVR_ATmega88__)      || \
		defined(__AVR_ATmega88P__)     || \
		defined(__AVR_ATmega88PA__)    || \
		defined(__AVR_ATmega168__)     || \
		defined(__AVR_ATmega168P__)    || \
		defined(__AVR_ATmega168PA__)   || \
		defined(__AVR_ATmega328P__)    || \
		defined(__AVR_ATmega16__)      || \
		defined(__AVR_ATmega16A__)     || \
		defined(__AVR_ATmega164P__)    || \
		defined(__AVR_ATmega32__)      || \
		defined(__AVR_ATmega32A__)     || \
		defined(__AVR_ATmega324P__)    || \
		defined(__AVR_ATmega324PA__)   || \
		defined(__AVR_ATmega644__)     || \
		defined(__AVR_ATmega644P__)    || \
		defined(__AVR_ATmega644PA__)   || \
		defined(__AVR_ATmega1284P__)   || \
		defined(__AVR_ATmega1284__)    || \
		defined(__AVR_ATmega2560__)    || \
		defined(__AVR_ATmega1280__))
// Tested with: 168P, 328P, 644P
// TODO: - add "complete" support for ATmega1280 - the last one supported
//         by OptiBoot bootloader. Not so soon, as I don't have yet an
//         Arduino Mega board for tests.
#else
// unsupported type
#error "Processor type not supported in atmegaclib2.c !"
#endif

#if !(F_CPU == 16000000)
#error "Processor speed not supported in atmegaclib2.c !"
#endif

#endif // end file
