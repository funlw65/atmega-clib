/* *****************************************************************************
 * 7seg.c
 * See atmegaclib2.h header for the copyrights...
 * *****************************************************************************
 */
#ifndef F_CPU
#define F_CPU 16000000UL //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include "atmegaclib2.h"

//--
#ifdef SEG_COMMON_ANODE
const uint8_t seg_mask = 0xff;
#else
const uint8_t seg_mask = 0x00;
#endif

#ifdef SEG_DP_PORT
#define SEG_NR_CHARS 38
#else
#define SEG_NR_CHARS 37
#endif

const uint8_t PROGMEM seg_code[SEG_NR_CHARS] = {
// index 0 is character 0
		SEG_A + SEG_B + SEG_C + SEG_D + SEG_E + SEG_F,
		// index 1 is character 1
		SEG_B + SEG_C,
		// index 2 is character 2
		SEG_A + SEG_B + SEG_D + SEG_E + SEG_G,
		// index 3 is character 3
		SEG_A + SEG_B + SEG_C + SEG_D + SEG_G,
		// index 4 is character 4
		SEG_F + SEG_G + SEG_B + SEG_C,
		// index 5 is character 5
		SEG_A + SEG_C + SEG_D + SEG_F + SEG_G,
		// index 6 is character 6
		SEG_A + SEG_C + SEG_D + SEG_E + SEG_F + SEG_G,
		// index 7 is character 7
		SEG_A + SEG_B + SEG_C,
		// index 8 is character 8
		SEG_A + SEG_B + SEG_C + SEG_D + SEG_E + SEG_F + SEG_G,
		// index 9 is character 9
		SEG_A + SEG_B + SEG_C + SEG_D + SEG_F + SEG_G,
		// index 10 is character A
		SEG_A + SEG_B + SEG_C + SEG_E + SEG_F + SEG_G,
		// index 11 is character b
		SEG_C + SEG_D + SEG_E + SEG_F + SEG_G,
		// index 12 is character C
		SEG_A + SEG_D + SEG_E + SEG_F,
		// index 13 is character d
		SEG_B + SEG_C + SEG_D + SEG_E + SEG_G,
		// index 14 is character E
		SEG_A + SEG_D + SEG_E + SEG_F + SEG_G,
		// index 15 is character F
		SEG_A + SEG_E + SEG_F + SEG_G,
		// index 16 is character S
		SEG_A + SEG_F + SEG_G + SEG_C + SEG_D,
		// index 17 is character c
		SEG_G + SEG_E + SEG_D,
		// index 18 is character r
		SEG_G + SEG_E,
		// index 19 is character H
		SEG_F + SEG_E + SEG_G + SEG_B + SEG_C,
		// index 20 is character i
		SEG_C,
		// index 21 is character L
		SEG_F + SEG_E + SEG_D,
		// index 22 is character o
		SEG_G + SEG_C + SEG_D + SEG_E,
		// index 23 is character P
		SEG_A + SEG_B + SEG_G + SEG_F + SEG_E,
		// index 24 is character U
		SEG_F + SEG_E + SEG_D + SEG_C + SEG_B,
		// index 25 is character u
		SEG_E + SEG_D + SEG_C,
		// index 26 is character h
		SEG_F + SEG_E + SEG_G + SEG_C,
		// index 27 is character Y
		SEG_F + SEG_G + SEG_B + SEG_C + SEG_D,
		// index 28 is character J
		SEG_B + SEG_C + SEG_D,
		// index 29 is character N
		SEG_E + SEG_F + SEG_A + SEG_B + SEG_C,
		// index 30 is character n
		SEG_C + SEG_G + SEG_E,
		// index 31 is character T
		SEG_A + SEG_F + SEG_E,
		// index 32 is character = (equal)
		SEG_G + SEG_D,
		// index 33 is character - (minus)
		SEG_G,
		// index 34 is character _ (underline)
		SEG_D,
		// index 35 is character G
		SEG_A + SEG_C + SEG_D + SEG_E + SEG_F,
		// index 36 is character space
		0
#ifdef SEG_DP_PORT
		// index 37 is character DP (dot) only if you enabled it in atmegaclib.h header...
		// Otherwise, your array ends at index 36.
		, SEG_DP
#endif
}		;
void seg_init(void) {
#ifdef SEG_DIGITS_8
	SEG_DIGITS_BUFFER[0] = 36;
	SEG_DIGITS_BUFFER[1] = 36;
	SEG_DIGITS_BUFFER[2] = 36;
	SEG_DIGITS_BUFFER[3] = 36;
	SEG_DIGITS_BUFFER[4] = 36;
	SEG_DIGITS_BUFFER[5] = 36;
	SEG_DIGITS_BUFFER[6] = 36;
	SEG_DIGITS_BUFFER[7] = 36;

#elif  defined(SEG_DIGITS_7)
	SEG_DIGITS_BUFFER[0] = 36;
	SEG_DIGITS_BUFFER[1] = 36;
	SEG_DIGITS_BUFFER[2] = 36;
	SEG_DIGITS_BUFFER[3] = 36;
	SEG_DIGITS_BUFFER[4] = 36;
	SEG_DIGITS_BUFFER[5] = 36;
	SEG_DIGITS_BUFFER[6] = 36;
#elif  defined(SEG_DIGITS_6)
	SEG_DIGITS_BUFFER[0] = 36;
	SEG_DIGITS_BUFFER[1] = 36;
	SEG_DIGITS_BUFFER[2] = 36;
	SEG_DIGITS_BUFFER[3] = 36;
	SEG_DIGITS_BUFFER[4] = 36;
	SEG_DIGITS_BUFFER[5] = 36;
#elif  defined(SEG_DIGITS_5)
	SEG_DIGITS_BUFFER[0] = 36;
	SEG_DIGITS_BUFFER[1] = 36;
	SEG_DIGITS_BUFFER[2] = 36;
	SEG_DIGITS_BUFFER[3] = 36;
	SEG_DIGITS_BUFFER[4] = 36;
#elif  defined(SEG_DIGITS_4)
	SEG_DIGITS_BUFFER[0] = 36;
	SEG_DIGITS_BUFFER[1] = 36;
	SEG_DIGITS_BUFFER[2] = 36;
	SEG_DIGITS_BUFFER[3] = 36;
#elif  defined(SEG_DIGITS_3)
	SEG_DIGITS_BUFFER[0] = 36;
	SEG_DIGITS_BUFFER[1] = 36;
	SEG_DIGITS_BUFFER[2] = 36;
#elif  defined(SEG_DIGITS_2)
	SEG_DIGITS_BUFFER[0] = 36;
	SEG_DIGITS_BUFFER[1] = 36;
#else
	SEG_DIGITS_BUFFER[0] = 36;
#endif
	// set direction for segment pins
#ifdef SEG_DP_DDR
	sbi(SEG_DP_DDR, SEG_DP_PIN);
#endif
	sbi(SEG_A_DDR, SEG_A_PIN);
	sbi(SEG_B_DDR, SEG_B_PIN);
	sbi(SEG_C_DDR, SEG_C_PIN);
	sbi(SEG_D_DDR, SEG_D_PIN);
	sbi(SEG_E_DDR, SEG_E_PIN);
	sbi(SEG_F_DDR, SEG_F_PIN);
	sbi(SEG_G_DDR, SEG_G_PIN);

	// set direction for common pins
#ifdef SEG_COMM_0_DDR
	sbi(SEG_COMM_0_DDR, SEG_COMM_0_PIN);
#endif
#ifdef SEG_COMM_1_DDR
	sbi(SEG_COMM_1_DDR, SEG_COMM_1_PIN);
#endif
#ifdef SEG_COMM_2_DDR
	sbi(SEG_COMM_2_DDR, SEG_COMM_2_PIN);
#endif
#ifdef SEG_COMM_3_DDR
	sbi(SEG_COMM_3_DDR, SEG_COMM_3_PIN);
#endif
#ifdef SEG_COMM_4_DDR
	sbi(SEG_COMM_4_DDR, SEG_COMM_4_PIN);
#endif
#ifdef SEG_COMM_5_DDR
	sbi(SEG_COMM_5_DDR, SEG_COMM_5_PIN);
#endif
#ifdef SEG_COMM_6_DDR
	sbi(SEG_COMM_6_DDR, SEG_COMM_6_PIN);
#endif
#ifdef SEG_COMM_7_DDR
	sbi(SEG_COMM_7_DDR, SEG_COMM_7_PIN);
#endif

}

uint8_t seg_digit_from_array(uint8_t index) {
	return seg_mask ^ pgm_read_byte(&seg_code[index]);
}

void seg_nibble(uint8_t segments) {
#ifdef SEG_DP_DDR
	cbi(SEG_DP_PORT, SEG_DP_PIN);
	if (bit_isset(segments,7))
	sbi(SEG_DP_PORT, SEG_DP_PIN);
#endif
	cbi(SEG_G_PORT, SEG_G_PIN);
	if (bit_isset(segments,6))
		sbi(SEG_G_PORT, SEG_G_PIN);
	cbi(SEG_F_PORT, SEG_F_PIN);
	if (bit_isset(segments,5))
		sbi(SEG_F_PORT, SEG_F_PIN);
	cbi(SEG_E_PORT, SEG_E_PIN);
	if (bit_isset(segments,4))
		sbi(SEG_E_PORT, SEG_E_PIN);
	cbi(SEG_D_PORT, SEG_D_PIN);
	if (bit_isset(segments,3))
		sbi(SEG_D_PORT, SEG_D_PIN);
	cbi(SEG_C_PORT, SEG_C_PIN);
	if (bit_isset(segments,2))
		sbi(SEG_C_PORT, SEG_C_PIN);
	cbi(SEG_B_PORT, SEG_B_PIN);
	if (bit_isset(segments,1))
		sbi(SEG_B_PORT, SEG_B_PIN);
	cbi(SEG_A_PORT, SEG_A_PIN);
	if (bit_isset(segments,0))
		sbi(SEG_A_PORT, SEG_A_PIN);
}

void seg_common_select(uint8_t digit) {
#ifdef SEG_COMM_7_PORT
	cbi(SEG_COMM_7_PORT,SEG_COMM_7_PIN);
	if(bit_isset(digit,7)) sbi(SEG_COMM_7_PORT,SEG_COMM_7_PIN);
#endif
#ifdef SEG_COMM_6_PORT
	cbi(SEG_COMM_6_PORT,SEG_COMM_6_PIN);
	if(bit_isset(digit,6)) sbi(SEG_COMM_6_PORT,SEG_COMM_6_PIN);
#endif
#ifdef SEG_COMM_5_PORT
	cbi(SEG_COMM_5_PORT,SEG_COMM_5_PIN);
	if(bit_isset(digit,5)) sbi(SEG_COMM_5_PORT,SEG_COMM_5_PIN);
#endif
#ifdef SEG_COMM_4_PORT
	cbi(SEG_COMM_4_PORT,SEG_COMM_4_PIN);
	if(bit_isset(digit,4)) sbi(SEG_COMM_4_PORT,SEG_COMM_4_PIN);
#endif
#ifdef SEG_COMM_3_PORT
	cbi(SEG_COMM_3_PORT, SEG_COMM_3_PIN);
	if (bit_isset(digit,3))
	sbi(SEG_COMM_3_PORT, SEG_COMM_3_PIN);
#endif
#ifdef SEG_COMM_2_PORT
	cbi(SEG_COMM_2_PORT, SEG_COMM_2_PIN);
	if (bit_isset(digit,2))
	sbi(SEG_COMM_2_PORT, SEG_COMM_2_PIN);
#endif
#ifdef SEG_COMM_1_PORT
	cbi(SEG_COMM_1_PORT, SEG_COMM_1_PIN);
	if (bit_isset(digit,1))
	sbi(SEG_COMM_1_PORT, SEG_COMM_1_PIN);
#endif
	// At least one pin must be set for selecting the digit
	// (minimum 1 digit, maximum 8 digits)
	cbi(SEG_COMM_0_PORT, SEG_COMM_0_PIN);
	if (bit_isset(digit,0))
		sbi(SEG_COMM_0_PORT, SEG_COMM_0_PIN);
}

// Usage if common pin is active in 0 logic and you want to select the digit nr.4:
// seg_select_digit(SELECT_DIGIT_FOUR, 0);
void seg_select_digit(MyDigit digit, uint8_t active_logic) {
	uint8_t digit_mask;
	if (active_logic == 0)
		digit_mask = 0xFF;
	else
		digit_mask = 0x00;
	seg_common_select(digit_mask ^ digit);
}

