/*
 *  See atmegaclib2.h for copyright and license
 */
#ifndef F_CPU
#define F_CPU 16000000UL //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <util/crc16.h>
#include "atmegaclib2.h"
#ifdef ENABLE_IR
#include "irkeys.h"
#endif
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#ifdef ENABLE_ONE_WIRE
#include <util/atomic.h>
#endif

int main(void){
	return 0;
}
