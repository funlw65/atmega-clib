/*
 * main.c
 *
 *  Created on: Jul 1, 2013
 *      Author: Vasile Guta Ciucur
 *  Description: Displaying the temperature and the id of a single temp.sensor
 *               on a single bus (specifically, DS18B20 sensor, external power)
 *  Hardware: Arduino UNO, DS18B20 sensor (on EvB4.3 board). The sensor
 *             is connected to the Analog pin 5 on Arduino (PC5 on any ATmega),
 *             is externally powered and have a 4k7 pull-up resistor on data
 *             line.
 *
 *******************************************************************************
 *  BSD License.
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

/*
//Scroll down the "atmegaclib2.h" header file and, still in the user zone, search for and
// change the following definitions, as bellow:

#define OW_ONE_BUS <-- uncomment this line.

// search for the following lines and change to the required pin if you need.
#define OW_PIN  PC5
#define OW_IN   PINC
#define OW_DDR  DDRC
#define OW_OUT  PORTC

// Search for the following and change them as follows:

#define OW_USE_INTERNAL_PULLUP    0  // 0=external, 1=internal

// DS18x20 EERPROM support disabled(0) or enabled(1) :
#define DS18X20_EEPROMSUPPORT     0
// decicelsius functions disabled(0) or enabled(1):
#define DS18X20_DECICELSIUS       1
// max. resolution functions disabled(0) or enabled(1):
#define DS18X20_MAX_RESOLUTION    0
// extended output via UART disabled(0) or enabled(1) :
#define DS18X20_VERBOSE           0 //require serial comm.

// Now you are ready to compile and upload your project.
*/

#ifndef F_CPU
#define F_CPU 16000000U // required by Atmel Studio 6
#endif

#include <avr/io.h>
//#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include "atmegaclib2.h"
#include <atmegaclib2_files.c>

void stop(void){
	for(;;){
		onboard_led_toggle();
		_delay_ms(250);
	}
}

void main(void) __attribute__((noreturn));
void main(void) {
	uint8_t id[OW_ROMCODE_SIZE]; // our ROM_id
	int16_t decicelsius;
	int8_t temp_str[7];
	uint8_t res, i;
	//--
	onboard_led_enable();
	onboard_led_off();
	serial_init();
	res = DS18X20_OK;
	res = ow_rom_search(OW_SEARCH_FIRST, &id[0]);
	if (res == OW_PRESENCE_ERR) {
		//
		serial_puts_f("No Sensor found!\r\n");
		stop();
	}
	if (res == OW_DATA_ERR) {
		//
		serial_puts_f("Bus Error!\r\n");
		stop();
	}
	while (1){
		// start measurement for all sensors or a single(unique) sensor
		res = DS18X20_start_meas(DS18X20_POWER_EXTERN, NULL);
		if(res != DS18X20_OK){
			//
			serial_puts_f( "DS18X20_start_meas: Short Circuit!\r\n");
			stop();
		}
		_delay_ms(DS18B20_TCONV_12BIT);
		// read the temperature of the unique sensor
		res=DS18X20_read_decicelsius_single(DS18B20_FAMILY_CODE, &decicelsius);
		if(res != DS18X20_OK){
			serial_puts_f("Error reading the temperature!\r\n");
			stop();
		}
		DS18X20_format_from_decicelsius(decicelsius, temp_str, 7);
		serial_puts(temp_str);
		serial_puts_f("_C ");
		// print the id code in hexadecimal
		for(i = 0; i < OW_ROMCODE_SIZE; i++){
			// showing the sensor id
			serial_puthexU08(id[i]);
		}
		serial_puts_f("\r\n");
	}
}

