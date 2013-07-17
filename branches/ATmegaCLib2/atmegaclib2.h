/* *****************************************************************************
 *  BSD License
 *  ATmega-CLib - a BSD library for using GNU toolchain mainly with EvB4.3
 *                board, but also Arduino, Sanguino, ...
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
 *  - (c) 2011 Martin Thomas, http://www.siwawi.arubi.uni-kl.de/avr-projects/
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

// ============== USER ZONE --- DO WHATEVER SETTINGS YOU NEED ==================

// *****************************************************************************
// Enabling/disabling additional functionality
// *****************************************************************************
#define UART_BAUD_RATE			19200 // default is 57600
#define UART_BAUD_SELECT		(F_CPU / (UART_BAUD_RATE * 16L) - 1)
#define ENABLE_SERIAL // Interrupt based, require CONVERSION, conflicts with SERIAL_POLL
//#define ENABLE_SERIAL_POLL // require CONVERSION, conflicts with SERIAL
//#define ENABLE_PWMSERVO    // servo control (conflicts with regular pwm)
#define ENABLE_PWM         // motor or led control (conflicts with pwmservo)
//#define ENABLE_IR          // infrared receiver, SONY protocol- it use TIMER0
//#define IR_DEBOUNCE        // uncomment to debounce IR with a delay
//#define ENABLE_ADC         // analog to digital converter
//#define ENABLE_TWI         // hardware I2C
//#define ENABLE_I2C_SOFTWARE // software I2C
#define ENABLE_CONVERSION    // useful for Serial, LCD and 7SEG Display
//#define ENABLE_PCF8583     // require CONVERSION and I2C/TWI
//#define ENABLE_ONE_WIRE    // one wire protocol
//#define ENABLE_DS18_2_ // Dallas temperature sensors, require ONE_WIRE
//#define ENABLE_NB_DELAYS // Non-blocking, slotted delays (instead of millis()) using Timer0
//#define ENABLE_LCD         // require CONVERSION
//#define ENABLE_7SEG        // starting from one digit, up to eight digits.
#define ENABLE_ISPPROG     // Use Arduino as ISP Programmer - require SPI, conflict SD_Card
#define ENABLE_SPI         // hardware SPI (master)
//#define ENABLE_SPI_INT     // hardware SPI use Interrupts
//#define ENABLE_SD_CARD_DEBUG // SD_ and F32_ functions send info on serial console
//#define ENABLE_SD_CARD       // raw SD Card operations; require SPI
//#define ENABLE_FAT32         // require PCF8583, SPI and SD_CARD
//#define OPTIMIZE_SPEED
// *****************************************************************************
// End block of "enable/disable" features
// *****************************************************************************

//Slotted delays
#ifdef ENABLE_NB_DELAYS
//the following define the number of non-blocking delays.
// set it to the required number of non-blocking delays you need.
#define DELAY_SLOTS  3 // the number of nb. delays
int16_t isr_countdowns[DELAY_SLOTS];
#endif

#ifdef ENABLE_ISPPROG
// DEFINE 3 LEDS INDICATORS FOR THE ISP PROGRAMMER
// TODO: To change the Heart Beat LED assignment because it must be a PWM channel!!!
#if defined(__AVR_ATmega16__)      || \
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
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
#if defined(ENABLE_SERIAL)// interrupt based
#define UART_BUFFER_SIZE  64 // buffer size for USART RX (receiving)
// change it to your needs (16, 32, 64, 128)
#endif

#ifdef ENABLE_SD_CARD
//define the pin used for CS (chip select) the SD Card
//it can be the SPI SS pin if you want (recommended if SD is the first SPI peripheral)
//default is set for atmega168p/328p (PB2) but it is an ongoing project,
//tested with different micro controllers so, the settings can vary from a
// SVN update to another...
#define SD_CS_DDR  DDRB
#define SD_CS_PORT PORTB
#define SD_CS_PIN  PB4
#endif //ENABLE_SD_CARD
#ifdef ENABLE_FAT32
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
#define OW_PIN  PD7
#define OW_IN   PIND
#define OW_DDR  DDRD
#define OW_OUT  PORTD
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
#ifdef ENABLE_7SEG
// Define delay in microseconds between digits selection to avoid ghost effect
#define SEG_DELAY 1500
// Define buffer for how many digits you have/are going to use
uint8_t SEG_DIGITS_BUFFER[4];
// Define the type of 7seg (common anode or common cathode)
#define SEG_COMMON_ANODE // comment this for common cathode type
//select the type of 7seg display (how many digits you have/use)
//must be the same as the buffer size (SEG_DIGITS_BUFFER[])
//#define SEG_DIGITS_1 1
//#define SEG_DIGITS_2 2
//#define SEG_DIGITS_3 3
#define SEG_DIGITS_4 4
//#define SEG_DIGITS_5 5
//#define SEG_DIGITS_6 6
//#define SEG_DIGITS_7 7
//#define SEG_DIGITS_8 8

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
#define SEG_G_PIN PB0

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

#define SEG_COMM_1_PORT PORTC
#define SEG_COMM_1_DDR DDRC
#define SEG_COMM_1_PIN PC3

#define SEG_COMM_2_PORT PORTC
#define SEG_COMM_2_DDR DDRC
#define SEG_COMM_2_PIN PC4

#define SEG_COMM_3_PORT PORTC
#define SEG_COMM_3_DDR DDRC
#define SEG_COMM_3_PIN PC5

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

// set the PCF8583 address and which I2C it use...
#ifdef ENABLE_PCF8583
//#define PCF8583_USE_TWI // disable if you want to use the software I2C
#define PCF8583_A0 0;          // relative base address.
#define Physical_Address 0xA2; // On EVB board, the address for PCF is 0xA2
// - set to 0xA0 if pin A0 of PCF8583 is connected to GND
// - set to 0xA2 if pin A0 of PCF8583 is connected to VCC
#endif

// ============ END USER ZONE --- NO EDITING ALLOWED! ==========================
// =============================================================================
// =============================================================================

#define FALSE 0
#define TRUE  1
#define HIGH  1
#define LOW   0

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

#ifdef ENABLE_NB_DELAYS
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
    defined(__AVR_ATmega164P__)    || \
    defined(__AVR_ATmega32__)      || \
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
//#define SPI_CLOCK_DIV64 0x07

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
	uint8_t partitionData[64]; //partition records (16x4)
	uint16_t signature; //0xaa55
};

//Structure to access info of the first partition of the disk
struct partitionInfo_Structure {
	uint8_t status; //0x80 - active partition
	uint8_t headStart; //starting head
	uint16_t cylSectStart; //starting cylinder and sector
	uint8_t type; //partition type
	uint8_t headEnd; //ending head of the partition
	uint16_t cylSectEnd; //ending cylinder and sector
	uint32_t firstSector; //total sectors between MBR & the first sector of the partition
	uint32_t sectorsTotal; //size of this partition in sectors
};

//Structure to access boot sector data
struct BS_Structure {
	uint8_t jumpBoot[3]; //default: 0x009000EB
	uint8_t OEMName[8];
	uint16_t bytesPerSector; //default: 512
	uint8_t sectorPerCluster;
	uint16_t reservedSectorCount;
	uint8_t numberofFATs;
	uint16_t rootEntryCount;
	uint16_t totalSectors_F16; //must be 0 for FAT32
	uint8_t mediaType;
	uint16_t FATsize_F16; //must be 0 for FAT32
	uint16_t sectorsPerTrack;
	uint16_t numberofHeads;
	uint32_t hiddenSectors;
	uint32_t totalSectors_F32;
	uint32_t FATsize_F32; //count of sectors occupied by one FAT
	uint16_t extFlags;
	uint16_t FSversion; //0x0000 (defines version 0.0)
	uint32_t rootCluster; //first cluster of root directory (=2)
	uint16_t FSinfo; //sector number of FSinfo structure (=1)
	uint16_t BackupBootSector;
	uint8_t reserved[12];
	uint8_t driveNumber;
	uint8_t reserved1;
	uint8_t bootSignature;
	uint32_t volumeID;
	uint8_t volumeLabel[11]; //"NO NAME "
	uint8_t fileSystemType[8]; //"FAT32"
	uint8_t bootData[420];
	uint16_t bootEndSignature; //0xaa55
};

//Structure to access FSinfo sector data
struct FSInfo_Structure {
	uint32_t leadSignature; //0x41615252
	uint8_t reserved1[480];
	uint32_t structureSignature; //0x61417272
	uint32_t freeClusterCount; //initial: 0xffffffff
	uint32_t nextFreeCluster; //initial: 0xffffffff
	uint8_t reserved2[12];
	uint32_t trailSignature; //0xaa550000
};

//Structure to access Directory Entry in the FAT
struct dir_Structure {
	uint8_t name[11];
	uint8_t attrib; //file attributes
	uint8_t NTreserved; //always 0
	uint8_t timeTenth; //tenths of seconds, set to 0 here
	uint16_t createTime; //time file was created
	uint16_t createDate; //date file was created
	uint16_t lastAccessDate;
	uint16_t firstClusterHI; //higher word of the first cluster number
	uint16_t writeTime; //time of last write
	uint16_t writeDate; //date of last write
	uint16_t firstClusterLO; //lower word of the first cluster number
	uint32_t fileSize; //size of file in bytes
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
//
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
#ifdef ENABLE_SERIAL // serial with interrupts...
#ifdef USART0_RX_vect // if uC with more than one Serial peripheral ...
#define UART0_ISR_VECT USART0_RX_vect
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
#define UART0_DATA	UDR0

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
// required by Dharmani's SD_Card functions... not sure, here would be nice to
// have setting for Linux and Mac, as now is only  for Windows termination line...
// Anyway, I think is ok, as long as is about a FAT32 file system.

#endif

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

#ifdef ENABLE_ADC
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
void lcd_puthexU08(uint8_t value); // display a byte in hex value
void lcd_puthexU16(uint16_t value); // display a word in hex value
void lcd_blank(uint8_t len); // blank n digits
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
#ifdef ENABLE_7SEG
typedef enum mydigit {
	SELECT_DIGIT_ONE = 1,
	SELECT_DIGIT_TWO = 2,
	SELECT_DIGIT_THREE = 4,
	SELECT_DIGIT_FOUR = 8,
	SELECT_DIGIT_FIVE = 16,
	SELECT_DIGIT_SIX = 32,
	SELECT_DIGIT_SEVEN = 64,
	SELECT_DIGIT_EIGHT = 128
} MyDigit;

void seg_init(void);
uint8_t seg_digit_from_array(uint8_t index);
void seg_nibble(uint8_t);
void seg_common_select(uint8_t);
void seg_select_digit(uint8_t, uint8_t);
#endif //7-seg
#ifdef ENABLE_PCF8583 //it requires Hardware or Software I2C
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
// 2. Must be supported by avrdude.
// It doesn't matter if there are more microcontrollers from the same family
//  supported in Atmel Studio 6 - the ATmega-CLib must be portable (Linux,
//  Mac OS X;  <rant>ATMEL, Visual Studio is your loss! But, if you want to program
//  the IDE in C++, then Qt library and Qt Designer are the solution</rant>).
// However, if the "user" will work exclusively in Atmel Studio 6, then is free
//  to add the required microcontrollers.
#if	(defined(__AVR_ATmega48__)   || \
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
  defined(__AVR_ATmega1284P__)   || \
  defined(__AVR_ATmega2560__)    || \
  defined(__AVR_ATmega1280__))
// Tested with: 168P, 328P, 644P
// TODO: - add "complete" support for ATmega1280 - the last one supported
//         by OptiBoot bootloader. Not so soon, as I don't have yet an
//         Arduino Mega board for tests.
#else
// unsupported type
#error "Processor type not supported in atmegaclib.c !"
#endif

#if !(F_CPU == 16000000)
#error "Processor speed not supported in atmegaclib.c !"
#endif

#endif // end file
