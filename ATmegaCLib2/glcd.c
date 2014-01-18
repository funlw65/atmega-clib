/* *****************************************************************************
 * glcd.c
 * See atmegaclib2.h header for the copyrights...
 * *****************************************************************************
 */
#ifndef F_CPU
#define F_CPU 16000000UL //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include "atmegaclib2.h"

//--

glcdCoord ks0108Coord;
uint8_t ks0108Inverted = 0;
ks0108FontCallback ks0108FontRead;
uint8_t ks0108FontColor;
const uint8_t* ks0108Font;

void GLCD_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2,
		uint8_t color) {
	uint8_t length, i, y, yAlt, xTmp, yTmp;
	int16_t m;

	//
	// vertical line
	//
	if (x1 == x2) {
		// x1|y1 must be the upper point
		if (y1 > y2) {
			yTmp = y1;
			y1 = y2;
			y2 = yTmp;
		}
		GLCD_DrawVertLine(x1, y1, y2 - y1, color);

		//
		// horizontal line
		//
	} else if (y1 == y2) {
		// x1|y1 must be the left point
		if (x1 > x2) {
			xTmp = x1;
			x1 = x2;
			x2 = xTmp;
		}
		GLCD_DrawHoriLine(x1, y1, x2 - x1, color);

		//
		// schiefe line :)
		//
	} else {
		// angle >= 45°
		if ((y2 - y1) >= (x2 - x1) || (y1 - y2) >= (x2 - x1)) {
			// x1 must be smaller than x2
			if (x1 > x2) {
				xTmp = x1;
				yTmp = y1;
				x1 = x2;
				y1 = y2;
				x2 = xTmp;
				y2 = yTmp;
			}

			length = x2 - x1; // not really the length :)
			m = ((y2 - y1) * 200) / length;
			yAlt = y1;

			for (i = 0; i <= length; i++) {
				y = ((m * i) / 200) + y1;

				if ((m * i) % 200 >= 100)
					y++;
				else if ((m * i) % 200 <= -100)
					y--;

				GLCD_DrawLine(x1 + i, yAlt, x1 + i, y, color);

				if (length <= (y2 - y1) && y1 < y2)
					yAlt = y + 1;
				else if (length <= (y1 - y2) && y1 > y2)
					yAlt = y - 1;
				else
					yAlt = y;
			}

			// angle < 45°
		} else {
			// y1 must be smaller than y2
			if (y1 > y2) {
				xTmp = x1;
				yTmp = y1;
				x1 = x2;
				y1 = y2;
				x2 = xTmp;
				y2 = yTmp;
			}

			length = y2 - y1;
			m = ((x2 - x1) * 200) / length;
			yAlt = x1;

			for (i = 0; i <= length; i++) {
				y = ((m * i) / 200) + x1;

				if ((m * i) % 200 >= 100)
					y++;
				else if ((m * i) % 200 <= -100)
					y--;

				GLCD_DrawLine(yAlt, y1 + i, y, y1 + i, color);
				if (length <= (x2 - x1) && x1 < x2)
					yAlt = y + 1;
				else if (length <= (x1 - x2) && x1 > x2)
					yAlt = y - 1;
				else
					yAlt = y;
			}
		}
	}
}

void GLCD_DrawRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height,
		uint8_t color) {
	GLCD_DrawHoriLine(x, y, width, color);
	// top
	GLCD_DrawHoriLine(x, y + height, width, color);
	// bottom
	GLCD_DrawVertLine(x, y, height, color);
	// left
	GLCD_DrawVertLine(x + width, y, height, color);
	// right
}

void GLCD_DrawRoundRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height,
		uint8_t radius, uint8_t color) {
	int16_t tSwitch, x1 = 0, y1 = radius;
	tSwitch = 3 - 2 * radius;

	while (x1 <= y1) {
		GLCD_SetDot(x + radius - x1, y + radius - y1, color);
		GLCD_SetDot(x + radius - y1, y + radius - x1, color);

		GLCD_SetDot(x + width - radius + x1, y + radius - y1, color);
		GLCD_SetDot(x + width - radius + y1, y + radius - x1, color);

		GLCD_SetDot(x + width - radius + x1, y + height - radius + y1, color);
		GLCD_SetDot(x + width - radius + y1, y + height - radius + x1, color);

		GLCD_SetDot(x + radius - x1, y + height - radius + y1, color);
		GLCD_SetDot(x + radius - y1, y + height - radius + x1, color);

		if (tSwitch < 0) {
			tSwitch += (4 * x1 + 6);
		} else {
			tSwitch += (4 * (x1 - y1) + 10);
			y1--;
		}
		x1++;
	}

	GLCD_DrawHoriLine(x + radius, y, width - (2 * radius), color);
	// top
	GLCD_DrawHoriLine(x + radius, y + height, width - (2 * radius), color);
	// bottom
	GLCD_DrawVertLine(x, y + radius, height - (2 * radius), color);
	// left
	GLCD_DrawVertLine(x + width, y + radius, height - (2 * radius), color);
	// right
}

/*
 * Hardware-Functions
 */
void GLCD_FillRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height,
		uint8_t color) {
	uint8_t mask, pageOffset, h, i, data;
	height++;

	pageOffset = y % 8;
	y -= pageOffset;
	mask = 0xFF;
	if (height < 8 - pageOffset) {
		mask >>= (8 - height);
		h = height;
	} else {
		h = 8 - pageOffset;
	}
	mask <<= pageOffset;

	GLCD_GotoXY(x, y);
	for (i = 0; i <= width; i++) {
		data = GLCD_ReadData();

		if (color == GLCD_BLACK) {
			data |= mask;
		} else {
			data &= ~mask;
		}

		GLCD_WriteData(data);
	}

	while (h + 8 <= height) {
		h += 8;
		y += 8;
		GLCD_GotoXY(x, y);

		for (i = 0; i <= width; i++) {
			GLCD_WriteData(color);
		}
	}

	if (h < height) {
		mask = ~(0xFF << (height - h));
		GLCD_GotoXY(x, y + 8);

		for (i = 0; i <= width; i++) {
			data = GLCD_ReadData();

			if (color == GLCD_BLACK) {
				data |= mask;
			} else {
				data &= ~mask;
			}

			GLCD_WriteData(data);
		}
	}
}

void GLCD_InvertRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height) {
	uint8_t mask, pageOffset, h, i, data, tmpData;
	height++;

	pageOffset = y % 8;
	y -= pageOffset;
	mask = 0xFF;
	if (height < 8 - pageOffset) {
		mask >>= (8 - height);
		h = height;
	} else {
		h = 8 - pageOffset;
	}
	mask <<= pageOffset;

	GLCD_GotoXY(x, y);
	for (i = 0; i <= width; i++) {
		data = GLCD_ReadData();
		tmpData = ~data;
		data = (tmpData & mask) | (data & ~mask);
		GLCD_WriteData(data);
	}

	while (h + 8 <= height) {
		h += 8;
		y += 8;
		GLCD_GotoXY(x, y);

		for (i = 0; i <= width; i++) {
			data = GLCD_ReadData();
			GLCD_WriteData(~data);
		}
	}

	if (h < height) {
		mask = ~(0xFF << (height - h));
		GLCD_GotoXY(x, y + 8);

		for (i = 0; i <= width; i++) {
			data = GLCD_ReadData();
			tmpData = ~data;
			data = (tmpData & mask) | (data & ~mask);
			GLCD_WriteData(data);
		}
	}
}

void GLCD_SetInverted(uint8_t invert) {
	if (ks0108Inverted != invert) {
		GLCD_InvertRect(0, 0, 127, 63);
		ks0108Inverted = invert;
	}
}

void GLCD_SetDot(uint8_t x, uint8_t y, uint8_t color) {
	uint8_t data;

	GLCD_GotoXY(x, y - y % 8); // read data from display memory
	data = GLCD_ReadData();

	if (color == GLCD_BLACK) {
		data |= 0x01 << (y % 8); // set dot
	} else {
		data &= ~(0x01 << (y % 8)); // clear dot
	}

	GLCD_WriteData(data); // write data back to display
}

//
// Font Functions
//

uint8_t GLCD_ReadFontData(const uint8_t* ptr) {
	return pgm_read_byte(ptr);
}

void GLCD_SelectFont(const uint8_t* font, ks0108FontCallback callback,
		uint8_t color) {
	ks0108Font = font;
	ks0108FontRead = callback;
	ks0108FontColor = color;
}

int GLCD_PutChar(char c) {
	uint8_t width = 0;
	uint8_t height = ks0108FontRead(ks0108Font + GLCD_FONT_HEIGHT);
	uint8_t bytes = (height + 7) / 8;

	uint8_t firstChar = ks0108FontRead(ks0108Font + GLCD_FONT_FIRST_CHAR);
	uint8_t charCount = ks0108FontRead(ks0108Font + GLCD_FONT_CHAR_COUNT);

	uint16_t index = 0;
	uint8_t x = ks0108Coord.x, y = ks0108Coord.y;

	if (c < firstChar || c >= (firstChar + charCount)) {
		return 1;
	}
	c -= firstChar;

	// read width data, to get the index
	for (uint8_t i = 0; i < c; i++) {
		index += ks0108FontRead(ks0108Font + GLCD_FONT_WIDTH_TABLE + i);
	}
	index = index * bytes + charCount + GLCD_FONT_WIDTH_TABLE;
	width = ks0108FontRead(ks0108Font + GLCD_FONT_WIDTH_TABLE + c);

	// last but not least, draw the character
	for (uint8_t i = 0; i < bytes; i++) {
		uint8_t page = i * width;
		for (uint8_t j = 0; j < width; j++) {
			uint8_t data = ks0108FontRead(ks0108Font + index + page + j);

			if (height < (i + 1) * 8) {
				data >>= (i + 1) * 8 - height;
			}

			if (ks0108FontColor == GLCD_BLACK) {
				GLCD_WriteData(data);
			} else {
				GLCD_WriteData(~data);
			}
		}
		// 1px gap between chars
		if (ks0108FontColor == GLCD_BLACK) {
			GLCD_WriteData(0x00);
		} else {
			GLCD_WriteData(0xFF);
		}
		GLCD_GotoXY(x, ks0108Coord.y + 8);
	}
	GLCD_GotoXY(x + width + 1, y);

	return 0;
}

void GLCD_Puts(char* str) {
	int x = ks0108Coord.x;
	while (*str != 0) {
		if (*str == '\n') {
			GLCD_GotoXY(x,
					ks0108Coord.y
							+ ks0108FontRead(ks0108Font + GLCD_FONT_HEIGHT));
		} else {
			GLCD_PutChar(*str);
		}
		str++;
	}
}

void GLCD_Puts_P(PGM_P str) {
	int x = ks0108Coord.x;
	while (pgm_read_byte(str) != 0) {
		if (pgm_read_byte(str) == '\n') {
			GLCD_GotoXY(x,
					ks0108Coord.y
							+ ks0108FontRead(ks0108Font + GLCD_FONT_HEIGHT));
		} else {
			GLCD_PutChar(pgm_read_byte(str));
		}
		str++;
	}
}

uint8_t GLCD_CharWidth(char c) {
	uint8_t width = 0;
	uint8_t firstChar = ks0108FontRead(ks0108Font + GLCD_FONT_FIRST_CHAR);
	uint8_t charCount = ks0108FontRead(ks0108Font + GLCD_FONT_CHAR_COUNT);

	// read width data
	if (c >= firstChar && c < (firstChar + charCount)) {
		c -= firstChar;
		width = ks0108FontRead(ks0108Font + GLCD_FONT_WIDTH_TABLE + c) + 1;
	}

	return width;
}

uint16_t GLCD_StringWidth(char* str) {
	uint16_t width = 0;

	while (*str != 0) {
		width += GLCD_CharWidth(*str++);
	}

	return width;
}

uint16_t GLCD_StringWidth_P(PGM_P str) {
	uint16_t width = 0;

	while (pgm_read_byte(str) != 0) {
		width += GLCD_CharWidth(pgm_read_byte(str++));
	}

	return width;
}

void GLCD_GotoXY(uint8_t x, uint8_t y) {
	uint8_t chip = GLCD_CHIP1, cmd;

	if (x > 127)
		x = 0; // ensure that coordinates are legal
	if (y > 63)
		y = 0;

	ks0108Coord.x = x; // save new coordinates
	ks0108Coord.y = y;
	ks0108Coord.page = y / 8;

	if (x >= 64) { // select the right chip
		x -= 64;
		chip = GLCD_CHIP2;
	}
	cmd = GLCD_SET_ADD | x;
	GLCD_WriteCommand(cmd, chip); // set x address on active chip

	cmd = GLCD_SET_PAGE | ks0108Coord.page; // set y address on both chips
	GLCD_WriteCommand(cmd, GLCD_CHIP1);
	GLCD_WriteCommand(cmd, GLCD_CHIP2);
}

void GLCD_Init(uint8_t invert) {
	ks0108Coord.x = 0;
	ks0108Coord.y = 0;
	ks0108Coord.page = 0;

	ks0108Inverted = invert;

	GLCD_CMD_DIR = 0xFF; // command port is output
	GLCD_WriteCommand(GLCD_ON, GLCD_CHIP1); // power on
	GLCD_WriteCommand(GLCD_ON, GLCD_CHIP2);

	GLCD_WriteCommand(GLCD_DISP_START, GLCD_CHIP1); // display start line = 0
	GLCD_WriteCommand(GLCD_DISP_START, GLCD_CHIP2);
	GLCD_ClearScreen(); // display clear
	GLCD_GotoXY(0, 0);
}

inline void GLCD_Enable(void) {
	GLCD_CMD_PORT |= 0x01 << GLCD_EN; // EN high level width: min. 450ns
	asm volatile("nop\n\t"
			"nop\n\t"
			"nop\n\t"
			::);
	GLCD_CMD_PORT &= ~(0x01 << GLCD_EN);
	for (volatile uint8_t i = 0; i < 8; i++)
		; // a little delay loop (faster than reading the busy flag)
}

uint8_t GLCD_DoReadData(uint8_t first) {
	uint8_t data;
	volatile uint8_t i;

	GLCD_DATA_OUT = 0x00;
	GLCD_DATA_DIR = 0x00; // data port is input

	if (ks0108Coord.x < 64) {
		GLCD_CMD_PORT &= ~(0x01 << GLCD_CSEL2); // deselect chip 2
		GLCD_CMD_PORT |= 0x01 << GLCD_CSEL1; // select chip 1
	} else if (ks0108Coord.x >= 64) {
		GLCD_CMD_PORT &= ~(0x01 << GLCD_CSEL1); // deselect chip 1
		GLCD_CMD_PORT |= 0x01 << GLCD_CSEL2; // select chip 2
	}
	if (ks0108Coord.x == 64 && first) { // chip2 X-address = 0
		GLCD_WriteCommand(GLCD_SET_ADD, GLCD_CHIP2); // wuff wuff
	}

	GLCD_CMD_PORT |= 0x01 << GLCD_D_I; // D/I = 1
	GLCD_CMD_PORT |= 0x01 << GLCD_R_W; // R/W = 1

	GLCD_CMD_PORT |= 0x01 << GLCD_EN; // EN high level width: min. 450ns
	asm volatile("nop\n\t"
			"nop\n\t"
			"nop\n\t"
			::);

	data = GLCD_DATA_IN; // read Data

	GLCD_CMD_PORT &= ~(0x01 << GLCD_EN);
	for (i = 0; i < 8; i++)
		; // a little delay loop (faster than reading the busy flag)

	GLCD_DATA_DIR = 0xFF;

	GLCD_GotoXY(ks0108Coord.x, ks0108Coord.y);

	if (ks0108Inverted)
		data = ~data;
	return data;
}

inline uint8_t GLCD_ReadData(void) {
	GLCD_DoReadData(1); // dummy read
	return GLCD_DoReadData(0); // "real" read
}

void GLCD_WriteCommand(uint8_t cmd, uint8_t chip) {
	if (chip == GLCD_CHIP1) {
		GLCD_CMD_PORT &= ~(0x01 << GLCD_CSEL2); // deselect chip 2
		GLCD_CMD_PORT |= 0x01 << GLCD_CSEL1; // select chip 1
	} else if (chip == GLCD_CHIP2) {
		GLCD_CMD_PORT &= ~(0x01 << GLCD_CSEL1); // deselect chip 1
		GLCD_CMD_PORT |= 0x01 << GLCD_CSEL2; // select chip 2
	}

	GLCD_CMD_PORT &= ~(0x01 << GLCD_D_I); // D/I = 0
	GLCD_CMD_PORT &= ~(0x01 << GLCD_R_W); // R/W = 0
	GLCD_DATA_DIR = 0xFF; // data port is output
	GLCD_DATA_OUT = cmd; // write command
	GLCD_Enable(); // enable
	GLCD_DATA_OUT = 0x00;
}

void GLCD_WriteData(uint8_t data) {
	uint8_t displayData, yOffset, cmdPort;

#ifdef DEBUG
	volatile uint16_t i;
	for(i=0; i<5000; i++);
#endif

	if (ks0108Coord.x >= 128)
		return;

	if (ks0108Coord.x < 64) {
		GLCD_CMD_PORT &= ~(0x01 << GLCD_CSEL2); // deselect chip 2
		GLCD_CMD_PORT |= 0x01 << GLCD_CSEL1; // select chip 1
	} else if (ks0108Coord.x >= 64) {
		GLCD_CMD_PORT &= ~(0x01 << GLCD_CSEL1); // deselect chip 1
		GLCD_CMD_PORT |= 0x01 << GLCD_CSEL2; // select chip 2
	}
	if (ks0108Coord.x == 64) // chip2 X-address = 0
		GLCD_WriteCommand(GLCD_SET_ADD, GLCD_CHIP2);

	GLCD_CMD_PORT |= 0x01 << GLCD_D_I; // D/I = 1
	GLCD_CMD_PORT &= ~(0x01 << GLCD_R_W); // R/W = 0
	GLCD_DATA_DIR = 0xFF; // data port is output

	yOffset = ks0108Coord.y % 8;
	if (yOffset != 0) {
		// first page
		cmdPort = GLCD_CMD_PORT; // save command port
		displayData = GLCD_ReadData();

		GLCD_CMD_PORT = cmdPort; // restore command port
		GLCD_DATA_DIR = 0xFF; // data port is output

		displayData |= data << yOffset;
		if (ks0108Inverted)
			displayData = ~displayData;
		GLCD_DATA_OUT = displayData; // write data
		GLCD_Enable(); // enable

		// second page
		GLCD_GotoXY(ks0108Coord.x, ks0108Coord.y + 8);

		displayData = GLCD_ReadData();

		GLCD_CMD_PORT = cmdPort; // restore command port
		GLCD_DATA_DIR = 0xFF; // data port is output

		displayData |= data >> (8 - yOffset);
		if (ks0108Inverted)
			displayData = ~displayData;
		GLCD_DATA_OUT = displayData; // write data
		GLCD_Enable(); // enable

		GLCD_GotoXY(ks0108Coord.x + 1, ks0108Coord.y - 8);
	} else {
		if (ks0108Inverted)
			data = ~data;
		GLCD_DATA_OUT = data; // write data
		GLCD_Enable(); // enable
		ks0108Coord.x++;
	}
	GLCD_DATA_OUT = 0x00;
}

