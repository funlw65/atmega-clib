/* *****************************************************************************
 * ad9850.c
 * See atmegaclib2.h header for the copyrights...
 * *****************************************************************************
 */
#include "atmegaclib2.h"
uint32_t AD9850_frequency; // delta phase
uint8_t AD9850_phase; // phase offset

//#define pulse(pin) {digitalWrite(pin, HIGH); digitalWrite(pin, LOW);}

void AD9850_init(void){
    AD9850_frequency = 0;
    AD9850_phase = 0;
    //pinMode(W_CLK, OUTPUT);
    sbi(AD9850_W_CLK_DDR, AD9850_W_CLK); //output
    //pinMode(FQ_UD, OUTPUT);
    sbi(AD9850_FQ_UD_DDR, AD9850_FQ_UD); //output
    //pinMode(D7, OUTPUT);
    sbi(AD9850_D7_DDR, AD9850_D7_DDR);   //output
    //pulse(W_CLK);
    sbi(AD9850_W_CLK_PORT, AD9850_W_CLK); //1
    cbi(AD9850_W_CLK_PORT, AD9850_W_CLK); //0
    //pulse(FQ_UD);
    sbi(AD9850_FQ_UD_PORT, AD9850_FQ_UD); //1
    cbi(AD9850_FQ_UD_PORT, AD9850_FQ_UD); //0
}

void AD9850_update(void) {
    uint32_t f = AD9850_frequency;
    uint32_t t;
    for (int i = 0; i < 32; i++, f >>= 1) {
        //digitalWrite(D7, f & (uint32_t)0x00000001);
    	t = f & (uint32_t)0x00000001;
    	if(t > 0) sbi(AD9850_D7_PORT, AD9850_D7);
    	else cbi(AD9850_D7_PORT, AD9850_D7);
        //pulse(W_CLK);
        sbi(AD9850_W_CLK_PORT, AD9850_W_CLK); //1
        cbi(AD9850_W_CLK_PORT, AD9850_W_CLK); //0
    }
    uint8_t p = AD9850_phase;
    uint8_t t2;
    for (int i = 0; i < 8; i++, p >>= 1) {
        //digitalWrite(D7, p & (uint8_t)0x01);
    	t2 = p & (uint8_t)0x01;
    	if(t2 > 0) sbi(AD9850_D7_PORT, AD9850_D7);
    	else cbi(AD9850_D7_PORT, AD9850_D7);
       // pulse(W_CLK);
        sbi(AD9850_W_CLK_PORT, AD9850_W_CLK); //1
        cbi(AD9850_W_CLK_PORT, AD9850_W_CLK); //0
    }
    //pulse(FQ_UD);
    sbi(AD9850_FQ_UD_PORT, AD9850_FQ_UD); //1
    cbi(AD9850_FQ_UD_PORT, AD9850_FQ_UD); //0
}

void AD9850_setfreq(double f) {
	AD9850_frequency = f * 4294967296.0 / AD9850_EX_CLK;
	AD9850_update();
}
void AD9850_setphase(uint8_t p) {
	AD9850_phase = p << 3;
	AD9850_update();
}

void AD9850_down(void) {
    //pulse(FQ_UD);
    sbi(AD9850_FQ_UD_PORT, AD9850_FQ_UD); //1
    cbi(AD9850_FQ_UD_PORT, AD9850_FQ_UD); //0
    uint8_t p = 0x04;
    uint8_t t;
    for (int i = 0; i < 8; i++, p >>= 1) {
        //digitalWrite(D7, p & (uint8_t)0x01);
    	t = p & (uint8_t)0x01;
    	if(t > 0) sbi(AD9850_D7_PORT, AD9850_D7);
    	else cbi(AD9850_D7_PORT, AD9850_D7);
        //pulse(W_CLK);
        sbi(AD9850_W_CLK_PORT, AD9850_W_CLK); //1
        cbi(AD9850_W_CLK_PORT, AD9850_W_CLK); //0
    }
    //pulse(FQ_UD);
    sbi(AD9850_FQ_UD_PORT, AD9850_FQ_UD); //1
    cbi(AD9850_FQ_UD_PORT, AD9850_FQ_UD); //0
}

void AD9850_up(void) { AD9850_update(); }
