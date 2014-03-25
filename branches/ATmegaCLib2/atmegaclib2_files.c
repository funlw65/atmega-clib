/*
 * atmegaclib2_files.c
 *
 *  Created on: Mar 25, 2014
 *      Author: sotea
 */

#include "atmegaclib2.h"

#ifdef ENABLE_CONVERSION
#include <conversion.c>
#endif
//

#ifdef ENABLE_AD9850
#include <ad9850.c>
#endif
//

#ifdef ENABLE_XPRINTF
#include <xprintf.c>
#endif
//

#ifdef ENABLE_NB_DELAYS
#include <nbdelays.c>
#endif
//

#ifdef ENABLE_I2C_SOFTWARE
#include <i2c.c>
#endif
//

#ifdef ENABLE_TWI
#include <twi.c>
#endif
//

#ifdef ENABLE_PCF8583
#include <pcf8583.c>
#endif
//

#ifdef ENABLE_SERIAL
#include <serial.c>
#include <serial_common.c>
#endif
//

#ifdef ENABLE_SERIAL_POLL
#include <serial_poll.c>
#include <serial_common.c>
#endif
//

#ifdef ENABLE_FREQMEASURE
#include <freqmeasure.c>
#endif
//

#ifdef ENABLE_SPI
#include <spi.c>
#endif
//

#ifdef ENABLE_SD_CARD
#include <sd_card.c>
#endif
//

#ifdef ENABLE_FAT32
#include <fat32.c>
#endif
//

#ifdef ENABLE_ONE_WIRE
#include <one_wire.c>
#endif
//

#ifdef ENABLE_DS18_2_
#include <dallas_temp.c>
#endif
//

#ifdef ENABLE_LCD
#include <lcd.c>
#endif
//

#ifdef ENABLE_GLCD
#include <glcd.c>
#endif
//

#ifdef ENABLE_7SEG
#include <7seg.c>
#endif
//

#ifdef ENABLE_RFM12B
#include <rfm12b.c>
#endif
//

#ifdef ENABLE_MIRF24
#include <mirf24.c>
#endif
//

#ifdef ENABLE_MILLIS
#include <millis.c>
#endif
//

#ifdef ENABLE_IR
#include <ir.c>
#endif
//

#ifdef ENABLE_PWMSERVO
#include <pemservo.c>
#endif
//

#ifdef ENABLE_PWM
#include <pwm.c>
#endif
//

#ifdef ENABLE_ADC
#include <adc.c>
#endif
//

#ifdef ENABLE_ARDUINO_COMPAT
#include <arduinocompat.c>
#endif
//

#include <onboard.c>
