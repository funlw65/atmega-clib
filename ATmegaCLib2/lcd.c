/* *****************************************************************************
 * lcd.c
 * See atmegaclib2.h header for the copyrights...
 * *****************************************************************************
 */
#ifndef F_CPU
#define F_CPU 16000000UL //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include "atmegaclib2.h"
#include <stdlib.h>

//--
static void lcd_nibble(uint8_t d);
static void lcd_byte(uint8_t d);

uint8_t lcd_pos = LCD_LINE1;
void lcd_init(void) {

	// set LCD DDR pins to 1 for output
	LCD_D4_DDR |= (1 << LCD_D4_PIN);
	LCD_D5_DDR |= (1 << LCD_D5_PIN);
	LCD_D6_DDR |= (1 << LCD_D6_PIN);
	LCD_D7_DDR |= (1 << LCD_D7_PIN);
	LCD_E_DDR |= (1 << LCD_E_PIN);
	LCD_RS_DDR |= (1 << LCD_RS_PIN);

	/*// set LCD DDR pins to 1 for output
	 sbi(LCD_D4_DDR,LCD_D4_PIN);
	 sbi(LCD_D5_DDR,LCD_D5_PIN);
	 sbi(LCD_D6_DDR,LCD_D6_PIN);
	 sbi(LCD_D7_DDR,LCD_D7_PIN);
	 sbi(LCD_E_DDR,LCD_E_PIN);
	 */sbi(LCD_RS_DDR, LCD_RS_PIN);
	/**/

// set the E and RS PORT pins to 0
	LCD_E_PORT &= ~(1 << LCD_E_PIN);
	LCD_RS_PORT &= ~(1 << LCD_RS_PIN);

	/*
	 // set the E and RS PORT pins to 0
	 cbi(LCD_E_PORT,LCD_E_PIN);
	 cbi(LCD_RS_PORT,LCD_RS_PIN);*//**/

	_delay_ms(15);
	lcd_nibble(0x30);
	_delay_ms(4.1);
	lcd_nibble(0x30);
	_delay_us(100);
	lcd_nibble(0x30);
	_delay_us(LCD_TIME_DAT);
	lcd_nibble(0x20); // 4 bit mode
	_delay_us(LCD_TIME_DAT);
#if LCD_LINE == 1
	lcd_command( 0x20 ); // 1 line
#else
	lcd_command(0x28); // 2 lines 5*7
#endif
	lcd_command(0x08); // display off
	lcd_command(0x01); // display clear
	lcd_command(0x06); // cursor increment
	lcd_command(0x0C); // on, no cursor, no blink

	// Set initial display conditions
	_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;

	// Initialize to default text direction (for romance languages)
	_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
	// set the entry mode
	lcd_command(LCD_ENTRYMODESET | _displaymode);
}

void lcd_clear() {
	lcd_command(0x01);
}

void lcd_home() {
	lcd_set_cursor(0, 0);
}

void lcd_putchar(uint8_t d) {
	sbi(LCD_RS_PORT, LCD_RS_PIN);

	lcd_byte(d);

	switch (++lcd_pos) {
	case LCD_LINE1 + LCD_COLUMN:
#ifdef LCD_LINE2
		d = LCD_LINE2;
		break;
		case LCD_LINE2 + LCD_COLUMN:
#ifdef LCD_LINE3
		d = LCD_LINE3;
		break;
		case LCD_LINE3 + LCD_COLUMN:
#ifdef LCD_LINE4
		d = LCD_LINE4;
		break;
		case LCD_LINE4 + LCD_COLUMN:
#endif
#endif
#endif
		d = LCD_LINE1;
		break;
	default:
		return;
	}
	lcd_command(d);
}

void lcd_putstr(int8_t *s) // display string from SRAM
{
	for (int8_t *s1 = s; *s1; s1++) // until zero byte
		lcd_putchar((int8_t) *s1);
}

void lcd_putstr_f(int8_t *FlashString) {
	uint8_t i = 0;
	// Check for '\0' string terminator or maximum LCD width
	while (pgm_read_byte(&FlashString[i]) && (i < LCD_COLUMN)) {
		lcd_putchar(pgm_read_byte(&FlashString[i++]));
	}
}

void lcd_putint(int value) {
	int8_t string[18];
	itoa(value, (char *) string, 10);
	lcd_putstr((int8_t *) string);
}

void lcd_putU08(uint8_t value) {
	int8_t s[4];
	byte2dec(value, s);
	lcd_putstr(s);
}

void lcd_puthexU08(uint8_t value) {
	int8_t s[3];
	byte2hex(value, s);
	lcd_putstr(s);
}

void lcd_puthexU16(uint16_t value) {
	int8_t s[5];
	word2hex(value, s);
	lcd_putstr(s);
}

void lcd_blank(uint8_t len) // blank n digits
{
	while (len--)
		lcd_putchar(' ');
}

void lcd_cursor_on(void) {
	_displaycontrol |= LCD_CURSORON;
	lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void lcd_cursor_off(void) {
	_displaycontrol &= ~LCD_CURSORON;
	lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void lcd_blink_on(void) {
	_displaycontrol |= LCD_BLINKON;
	lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void lcd_blink_off(void) {
	_displaycontrol &= ~LCD_BLINKON;
	lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);

}
void lcd_display_on(void) {
	_displaycontrol |= LCD_DISPLAYON;
	lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);

}
void lcd_display_off(void) {
	_displaycontrol &= ~LCD_DISPLAYON;
	lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);

}

// Private functions
static void lcd_nibble(uint8_t d) {

	cbi(LCD_D7_PORT, LCD_D7_PIN);
	if (d & 1 << 7)
		sbi(LCD_D7_PORT, LCD_D7_PIN);
	cbi(LCD_D6_PORT, LCD_D6_PIN);
	if (d & 1 << 6)
		sbi(LCD_D6_PORT, LCD_D6_PIN);
	cbi(LCD_D5_PORT, LCD_D5_PIN);
	if (d & 1 << 5)
		sbi(LCD_D5_PORT, LCD_D5_PIN);
	cbi(LCD_D4_PORT, LCD_D4_PIN);
	if (d & 1 << 4)
		sbi(LCD_D4_PORT, LCD_D4_PIN);

	sbi(LCD_E_PORT, LCD_E_PIN);
	_delay_us(LCD_TIME_ENA);
	cbi(LCD_E_PORT, LCD_E_PIN);
}

static void lcd_byte(uint8_t d) {
	lcd_nibble(d);
	lcd_nibble(d << 4);
	_delay_us(LCD_TIME_DAT);
}

void lcd_command(uint8_t d) {
	cbi(LCD_RS_PORT, LCD_RS_PIN);
	lcd_byte(d);
	switch (d) {
	case 0 ... 3: // on longer commands
		_delay_us(LCD_TIME_CLR);
		d = LCD_LINE1;
	case 0x80 ... 0xFF: // set position
		lcd_pos = d;
		//break;
	}
}

