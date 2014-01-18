/* *****************************************************************************
 * gpl_rfm12b.c
 * See atmegaclib2.h header for the copyrights...
 * *****************************************************************************
 */
#ifndef F_CPU
#define F_CPU 16000000UL //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include "atmegaclib2.h"
//--
#if RFM12_USE_RX_CALLBACK
	volatile static (*rfm12_rx_callback_func)(uint8_t, uint8_t *) = (void *)0x0000;
	void rfm12_set_callback ((*in_func)(uint8_t, uint8_t *)) {
		rfm12_rx_callback_func = in_func;
	}
#endif


/************************
 * library internal globals
*/

//! Buffer and status for packet transmission.
rf_tx_buffer_t rf_tx_buffer;

//if receive mode is not disabled (default)
#if !(RFM12_TRANSMIT_ONLY)
	//! Buffers and status to receive packets.
	rf_rx_buffer_t rf_rx_buffers[2];
#endif /* RFM12_TRANSMIT_ONLY */

//! Global control and status.
rfm12_control_t ctrl;


/************************
 * load other core and external components
 * (putting them directly into here allows GCC to optimize better)
*/

/* include spi functions into here */
//hardware spi helper macros
#define SS_ASSERT() RF_PORT_SS &= ~(1<<RF_BIT_SS)
#define SS_RELEASE() RF_PORT_SS |= (1<<RF_BIT_SS)


#if RFM12_SPI_SOFTWARE
/* @description Actual sending function to send raw data to the Module
 * @note do NOT call this function directly, unless you know what you're doing.
 */
static uint8_t spi_data(uint8_t c) {
	uint8_t x, d = d;
	for (x = 0; x < 8; x++) {
		if (c & 0x80) {
			MOSI_PORT |= (1<<MOSI);
		} else {
			MOSI_PORT &= ~(1<<MOSI);
		}
		SCK_PORT |= (1<<SCK);
		d <<= 1;
		if (MISO_PIN & (1<<MISO)) {
			d |= 1;
		}
		SCK_PORT &= ~(1<<SCK);
		c <<= 1;
	}
	return d;
}
#endif


//non-inlined version of rfm12_data
//warning: without the attribute, gcc will inline this even if -Os is set
static void __attribute__ ((noinline)) rfm12_data(uint16_t d) {
	SS_ASSERT();
	#if !(RFM12_SPI_SOFTWARE)
		SPDR = d >> 8;
		while (!(SPSR & (1<<SPIF)));

		SPDR = d & 0xff;
		while (!(SPSR & (1<<SPIF)));
	#else
		spi_data(d >> 8);
		spi_data(d & 0xff);
	#endif
	SS_RELEASE();
}


//non-inlined version of rfm12_read
//warning: without the attribute, gcc will inline this even if -Os is set
static uint16_t __attribute__ ((noinline)) rfm12_read(uint16_t c) {
	uint16_t retval;
	SS_ASSERT();

	#if !(RFM12_SPI_SOFTWARE)
		SPDR = c >> 8;
		while (!(SPSR & (1<<SPIF)));
		retval = SPDR << 8;
		SPDR = c & 0xff;
		while (!(SPSR & (1<<SPIF)));
		retval |= SPDR;
	#else
		retval = spi_data(c >> 8);
		retval <<= 8;
		retval |= spi_data(c & 0xff);
	#endif
	SS_RELEASE();
	return retval;
}


/* @description reads the upper 8 bits of the status
 * register (the interrupt flags)
 */
static uint8_t rfm12_read_int_flags_inline(void) {
	#if !(RFM12_SPI_SOFTWARE)
		SS_ASSERT();
		SPDR = 0;
		while (!(SPSR & (1<<SPIF)));
		SS_RELEASE();
		return SPDR;
	#else
		SS_ASSERT();
		unsigned char x, d = d;
		MOSI_PORT &= ~(1<<MOSI);
		for (x = 0; x < 8; x++) {
			SCK_PORT |= (1<<SCK);
			d <<= 1;
			if (MISO_PIN & (1<<MISO)) {
				d |= 1;
			}
			SCK_PORT &= ~(1<<SCK);
		}
		SS_RELEASE();
		return d;
	#endif
}

static void spi_init(void) {
#define PORT_SPI PORTB
#define DDR_SPI  DDRB
	MOSI_DDR |= (_BV(MOSI));
	SCK_DDR  |= (_BV(SCK));
	#if !(RFM12_SPI_SOFTWARE)
		PORT_SPI |= (_BV(SS));
		DDR_SPI  |= (_BV(SS));
	#endif

	MISO_DDR &= ~(_BV(MISO));

	#if !(RFM12_SPI_SOFTWARE)
		SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0); //SPI Master, clk/16
	#endif
}

/*
 * include control / init functions into here
 * all of the stuff in there is optional, so there's no code-bloat.
*/
//#define RFM12_LIVECTRL_HOST 1//if we are building for the microcontroller, we are the host.
//#include "include/rfm12_livectrl.c"
#if RFM12_LIVECTRL

#if RFM12_LIVECTRL_CLIENT

	#if __AVR__
		//#include <avr/pgmspace.h>
		//yes, we include the c file because of ease of configuration this way
		//the C preprocessor can decide wether we need the file or not.
		//#include "../xprintf/xprintf.c"
	#else
		#define PSTR(s)    s
		#define strcpy_P   strcpy
		#define xsprintf_P sprintf
	#endif

	void baseband_to_string(char *s, uint16_t var) {
		switch (var) {
			case RFM12_BAND_315:
				strcpy_P(s, PSTR("315MHz"));
				break;
			case RFM12_BAND_433:
				strcpy_P(s, PSTR("433MHz"));
				break;
			case RFM12_BAND_868:
				strcpy_P(s, PSTR("868MHz"));
				break;
			case RFM12_BAND_915:
				strcpy_P(s, PSTR("915MHz"));
				break;
			default:
				*s = 0;
				break;
		}
	}


	void frequency_to_string(char *s, uint16_t val) {
		//Band [MHz] C1 C2
		//315         1 31
		//433         1 43
		//868         2 43
		//915         3 30
		//f0 = 10 * C1 * (C2 + F/4000) [MHz]
		//f0 = 10 * C1 * (C2*1000 + F/4)    [kHz]

		//433:
		//f0 = 430 + F/400  [MHz]
		//f0 = 430000 + F * 2.5 [kHz]

		//915:
		//f0 = 900 + F * (3 /400)  [MHz]
		//f0 = 900000 + F * 7.5 [kHz]

		//868:
		//f0 = 860 + F * (2 /400)  [MHz]
		//f0 = 860000 + F * 5 [kHz]


		uint16_t mhz;
		uint16_t khz;
		uint16_t band_setting = livectrl_cmds[RFM12_LIVECTRL_BASEBAND].current_value;

		if (band_setting == RFM12_BAND_433) {
			mhz = 430 + val / 400;
			khz = ((val % 400) * 5) / 2;
		} else if (band_setting == RFM12_BAND_915) {
			val *= 3;
			mhz = 900 + val / 400;
			khz = ((val % 400) * 5) / 2;
		} else if (band_setting == RFM12_BAND_868) {
			val *= 2;
			mhz = 860 + val / 400;
			khz = ((val % 400) * 5) / 2;
		} else {
			mhz = 0;
			khz = 0;
		}
		#if (DISP_LEN == 8)
			xsprintf_P(s, PSTR("%3d.%03d"), mhz, khz);
		#else
			xsprintf_P(s, PSTR("%3d.%03d MHz"), mhz, khz);
		#endif
	}

	void datarate_to_string(char *s, uint16_t val) {
		/*
			4. Data Rate Command
			Bit 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0 POR
			1 1 0 0 0 1 1 0 cs r6 r5 r4 r3 r2 r1 r0 C623h
			The actual bit rate in transmit mode and the expected bit rate of the received data stream in receive mode is determined by the 7-bit
			parameter R (bits r6 to r0) and bit cs.
			BR = 10000 / 29 / (R+1) / (1+cs*7) [kbps]
		*/

		uint16_t n;

		if (val & 0x80) {
			//low bitrate
			n = 29 * 8;
		} else {
			//high bitrate
			n = 29;
		}
		val &= 0x7f;

		n *= val;

		uint32_t bitrate = 10000000ul / n;

		xsprintf_P(s, PSTR("%4ld bps"), bitrate);
	}


	#define FULLSCALE_TX_POWER 8;

	void tx_power_to_string(char *s, uint16_t var) {
		int16_t val = var;
		val *= -3;
		val += FULLSCALE_TX_POWER;

		xsprintf_P(s, PSTR("%3d dBm"), val);
	}

	void fsk_shift_to_string(char *s, uint16_t val) {
		val >>= 4; //right adjust

		val = (val + 1) * 15;

		xsprintf_P(s, PSTR("+-%d kHz"), val);
	}

	void lna_to_string(char *s, uint16_t var) {
		uint8_t val = var;
		val >>= 3; //right adjust

		switch (val) {
			case 1: val = 6; break;
			case 2: val = 14; break;
			case 3: val = 20; break;
		}

		xsprintf_P(s, PSTR("%3d dB"), -val);
	}

	void rssi_to_string(char *s, uint16_t val) {
		xsprintf_P(s, PSTR("%4d dBm"), -61 - (7 - val) * 6);
	}

	void filter_bw_to_string(char *s, uint16_t val) {
		val >>= 5; //right adjust

		val = ((7 - val) * 200) / 3;

		xsprintf_P(s, PSTR("%3d kHz"), val);
	}

	void xtal_load_to_string(char *s, uint16_t val) {
		val += 1;
		uint8_t pf = val / 2 + 8;
		uint8_t n = 0;
		if(val & 0x01) n = 5;
		xsprintf_P(s, PSTR("%2d.%dpF"), pf, n);
	}


	#define IFCLIENT(a,b,c,d,e) a,b,c,d,e
#else // RFM12_LIVECTRL_CLIENT
	#define IFCLIENT(a,b,c,d,e)
#endif // RFM12_LIVECTRL_CLIENT

#if RFM12_LIVECTRL_HOST
	#define IFHOST(a) a
#else
	#define IFHOST(a) 0
#endif

livectrl_cmd_t livectrl_cmds[] = {
	{ RFM12_CMD_CFG,       RFM12_CFG_BAND_MASK,     IFHOST(&ctrl.cfg_shadow),    RFM12_BASEBAND,                        IFCLIENT(0x00,   0x30, 0x10, "Baseband"  , baseband_to_string  )},
	{ RFM12_CMD_CFG,       RFM12_CFG_XTAL_MASK,     IFHOST(&ctrl.cfg_shadow),    RFM12_XTAL_LOAD,                       IFCLIENT(0x00,   0x0f,    1, "Xtal Load" , xtal_load_to_string  )},
	{ RFM12_CMD_FREQUENCY, RFM12_FREQUENCY_MASK,    0,                           RFM12_FREQUENCY_CALC(RFM12_FREQUENCY), IFCLIENT(0x00, 0x0fff,    4, "Frequency" , frequency_to_string )},
	{ RFM12_CMD_DATARATE,  RFM12_DATARATE_MASK,     0,                           DATARATE_VALUE,                        IFCLIENT(0x03,   0xff,    1, "Data rate" , datarate_to_string  )},
	{ RFM12_CMD_TXCONF,    RFM12_TXCONF_POWER_MASK, IFHOST(&ctrl.txconf_shadow), RFM12_POWER,                           IFCLIENT(0x00,   0x07,    1, "TX Power"  , tx_power_to_string  )},
	{ RFM12_CMD_TXCONF,    RFM12_TXCONF_FSK_MASK,   IFHOST(&ctrl.txconf_shadow), RFM12_TXCONF_FS_CALC(FSK_SHIFT),       IFCLIENT(0x00,   0xf0, 0x10, "FSK Shift" , fsk_shift_to_string )},
	{ RFM12_CMD_RXCTRL,    RFM12_RXCTRL_LNA_MASK,   IFHOST(&ctrl.rxctrl_shadow), RFM12_LNA_GAIN,                        IFCLIENT(0x00,   0x18, 0x08, "LNA"       , lna_to_string      )},
	{ RFM12_CMD_RXCTRL,    RFM12_RXCTRL_RSSI_MASK,  IFHOST(&ctrl.rxctrl_shadow), RFM12_RSSI_THRESHOLD,                  IFCLIENT(0x00,   0x07,    1, "RSSI"      , rssi_to_string      )},
	{ RFM12_CMD_RXCTRL,    RFM12_RXCTRL_BW_MASK,    IFHOST(&ctrl.rxctrl_shadow), RFM12_FILTER_BW,                       IFCLIENT(0x20,   0xC0, 0x20, "Filter BW" , filter_bw_to_string )},
};


#if RFM12_LIVECTRL_LOAD_SAVE_SETTINGS
	#include <avr/eeprom.h>

	void rfm12_save_settings() {
		uint8_t x;
		uint16_t checksumm = 0;

		for (x = 0; x < NUM_LIVECTRL_CMDS; x++) {
			uint16_t val = livectrl_cmds[x].current_value;
			checksumm += val;
			eeprom_write_word((void*)(2 * x), val);
		}

		eeprom_write_word((void*)(2 * x), checksumm);
	}

	void rfm12_load_settings() {

		uint8_t x;
		uint16_t val;
		uint16_t checksumm = 0;

		for (x = 0; x < NUM_LIVECTRL_CMDS; x++) {
			val = eeprom_read_word((void*)(2 * x));
			checksumm += val;
		}

		val = eeprom_read_word((void*)(2 * x));
		if (val != checksumm) return; //eeprom invalid, keep default values from array

		//set the settings if eeprom valid
		for (x = 0; x < NUM_LIVECTRL_CMDS; x++) {
			val = eeprom_read_word((void*)(2 * x));
			rfm12_livectrl(x, val);
		}
	}
#endif

#if RFM12_LIVECTRL_HOST
	void rfm12_data_safe(uint16_t d) {
		//disable the interrupt (as we're working directly with the transceiver now)
		RFM12_INT_OFF();
		rfm12_data(d);
		RFM12_INT_ON();
	}


	void rfm12_livectrl(uint8_t cmd, uint16_t value) {
		uint16_t tmp = 0;
		livectrl_cmd_t  *livectrl_cmd = &livectrl_cmds[cmd];

		livectrl_cmd->current_value = value; //update current value

		//the shadow register is somewhat redundant with the current value,
		//but it makes sense never the less:
		//the current_value only saves the bits for this one setting (for menu,saving,loding settings)
		//while the shadow register keeps track of ALL bits the rfm12 has in that register.
		//the shadow will also be used from rfm12_tick or maybe the interrupt

		if (livectrl_cmd->shadow_register) {
			tmp = *livectrl_cmd->shadow_register;         //load shadow value if any
			tmp &= ~livectrl_cmd->rfm12_hw_parameter_mask;//clear parameter bits
		}
		tmp |= livectrl_cmd->rfm12_hw_command | (livectrl_cmd->rfm12_hw_parameter_mask & value);

		*livectrl_cmd->shadow_register = tmp;

		rfm12_data_safe(tmp);
	}
#endif // RFM12_LIVECTRL_HOST

#if RFM12_LIVECTRL_CLIENT
	void rfm12_livectrl_get_parameter_string(uint8_t cmd, char *str) {
		livectrl_cmd_t *livectrl_cmd = &livectrl_cmds[cmd];

		uint16_t var = livectrl_cmd->current_value;
		livectrl_cmd->to_string(str, var);
	}
#endif // RFM12_LIVECTRL_CLIENT


#endif // RFM12_LIVECTRL
// end rfm12_livectrl.c

/*
 * include extra features here
 * all of the stuff in there is optional, so there's no code-bloat..
*/
//#include "include/rfm12_extra.c"
/************************
 * amplitude modulation receive mode
*/

#if RFM12_RECEIVE_ASK
	//! The ASK mode receive buffer structure.
	/** You will need to poll the state field of this structure to determine
	* if data is available, see \ref ask_defines "ASK mode defines". \n
	* Received data can be read from the buf field.
	* It is necessary to reset the state field  to RFM12_ASK_STATE_EMPTY after reading.
	*
	* \note You need to define RFM12_RECEIVE_ASK as 1 to enable this.
	*/
	rfm12_rfrxbuf_t ask_rxbuf;


	//! ASK mode ADC interrupt.
	/** This interrupt function directly measures the receive signal strength
	* on an analog output pin of the rf12 ic.
	*
	* You will need to solder something onto your rf12 module to make this to work.
	*
	* \note You need to define RFM12_RECEIVE_ASK as 1 to enable this.
	* \see adc_init() and rfm12_rfrxbuf_t
	*/
	ISR(ADC_vect, ISR_NOBLOCK) {
		static uint16_t adc_average;
		static uint8_t pulse_timer;
		uint8_t value;
		static uint8_t oldvalue;
		static uint8_t ignore;
		uint16_t adc;


		ADCSRA = (1<<ADEN) | (1<<ADFR) | (0<<ADIE) //start free running mode
				| (1<<ADPS2) | (1<<ADPS1);  //preescaler to clk/64
											//samplerate = 16MHz/(64*13) = 19231 Hz


		adc = ADC;

		adc_average -= adc_average / 64;
		adc_average +=adc;

		value = (ADC > ((adc_average / 64) + 50)) ? 1 : 0;

		if (value) {
			PORTD |= (1<<PD7);
		} else {
			PORTD &= ~(1<<PD7);
		}


		if (TCNT0 > 0xE0) {
			ignore = 0;
		}

		if (ask_rxbuf.state == RFM12_ASK_STATE_EMPTY) {
			if (value && (!ignore)) {
				//pulse_timer = 0;
				TCNT0 = 0;
				ask_rxbuf.p = 0;
				ask_rxbuf.state = RFM12_ASK_STATE_RECEIVING;
			}
		} else if (ask_rxbuf.state == RFM12_ASK_STATE_FULL) {
			if (value) {
				TCNT0 = 0;
				ignore = 1;
			}

		} else if (ask_rxbuf.state == RFM12_ASK_STATE_RECEIVING) {
			if (value != oldvalue) {

				ask_rxbuf.buf[ask_rxbuf.p] = TCNT0;
				TCNT0 = 0;
				//pulse_timer = 0;
				if (ask_rxbuf.p != (RFM12_ASK_RFRXBUF_SIZE - 1) ) {
					ask_rxbuf.p++;
				}
			} else if (TCNT0 > 0xe0) {
				//if( !value ){
				//PORTD |= (1<<PD6);
					ask_rxbuf.state = RFM12_ASK_STATE_FULL;
				//}else{
				//	ask_rxbuf.state = STATE_EMPTY;
				//}
			}
		}

		oldvalue = value;

		ADCSRA = (1<<ADEN) | (1<<ADFR) | (1<<ADIE) //start free running mode
				| (1<<ADPS2) | (1<<ADPS1);  //preescaler to clk/64
											//samplerate = 16MHz/(64*13) = 19231 Hz

	}


	//! ASK mode ADC interrupt setup.
	/** This will setup the ADC interrupt to receive ASK modulated signals.
	* rfm12_init() calls this function automatically if ASK receive mode is enabled.
	*
	* \note You need to define RFM12_RECEIVE_ASK as 1 to enable this.
	* \see ISR(ADC_vect, ISR_NOBLOCK) and rfm12_rfrxbuf_t
	*/
	void adc_init(void) {
		ADMUX  = (1<<REFS0) | (1<<REFS1); //Internal 2.56V Reference, MUX0

		ADCSRA = (1<<ADEN) | (1<<ADSC) | (1<<ADFR) | (1<<ADIE) //start free running mode
				| (1<<ADPS2) | (1<<ADPS1);  //preescaler to clk/64
											//samplerate = 16MHz/(64*13) = 19231 Hz

	}
#endif /* RFM12_RECEIVE_ASK */


/************************
 * ASK modulated raw tx mode
*/

#if RFM12_TRANSMIT_ASK
	//! En- or disable ASK transmissions.
	/** When enabling ASK tx mode, this function puts the internal state machine
	* into transmit mode and disables the interrupt.
	* Otherwise it will restore normale operation.
	*
	* \param [setting] Pass 1 to enable the raw mode, 0 to disable it.
	* \note You need to define RFM12_TRANSMIT_ASK as 1 to enable this.
	* \warning This will interfere with the wakeup timer feature.
	* \todo Use power management shadow register if the wakeup timer feature is enabled.
	* \see rfm12_tx_on() and rfm12_tx_off()
	*/
	void rfm12_ask_tx_mode(uint8_t setting) {
		if (setting)
		{
		#if 0
			/* disable the receiver */
			rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT);

			/* fill preamble into buffer */
			rfm12_data(RFM12_CMD_TX | PREAMBLE);
			rfm12_data(RFM12_CMD_TX | PREAMBLE);
		#endif
			ctrl.rfm12_state = STATE_TX;
			RFM12_INT_OFF();
		} else
		{
			/* re-enable the receiver */
			rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT | RFM12_PWRMGT_ER);
			RFM12_INT_ON();
			ctrl.rfm12_state = STATE_RX_IDLE;
		}
	}


	//! Enable the transmitter immediately (ASK transmission mode).
	/** This will send out the current buffer contents.
	* This function is used to emulate amplitude modulated signals.
	*
	* \note You need to define RFM12_TRANSMIT_ASK as 1 to enable this.
	* \warning This will interfere with the wakeup timer feature.
	* \todo Use power management shadow register if the wakeup timer feature is enabled.
	* \see rfm12_tx_off() and rfm12_ask_tx_mode()
	*/
	inline void rfm12_tx_on(void) {
		/* set enable transmission bit now. */
		rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT | RFM12_PWRMGT_ET | RFM12_PWRMGT_ES | RFM12_PWRMGT_EX);
	}


	//! Set default power mode (usually transmitter off, receiver on).
	/** This will usually stop a transmission.
	* This function is used to emulate amplitude modulated signals.
	*
	* \note You need to define RFM12_TRANSMIT_ASK as 1 to enable this.
	* \warning This will interfere with the wakeup timer feature.
	* \todo Use power management shadow register if the wakeup timer feature is enabled.
	* \see rfm12_tx_on() and rfm12_ask_tx_mode()
	*/
	inline void rfm12_tx_off(void) {
		/* turn off everything. */
		rfm12_data(RFM12_CMD_PWRMGT);
	}
#endif /* RFM12_TRANSMIT_ASK */


/************************
 * rfm12 wakeup timer mode
*/

#if RFM12_USE_WAKEUP_TIMER
	//! This function sets the wakeup timer register.
	/** \param [val]  The wakeup timer period value to be passed to the rf12. \n
	* See the rf12 datasheet for valid values.
	*/
	void rfm12_set_wakeup_timer(uint16_t val) {
		//disable the interrupt (as we're working directly with the transceiver now)
		//we won't loose interrupts, as the AVR caches them in the int flag
		RFM12_INT_OFF();

		//set wakeup timer
		rfm12_data (RFM12_CMD_WAKEUP | (val & 0x1FFF));

		//restart the wakeup timer by toggling the bit on and off
		rfm12_data(ctrl.pwrmgt_shadow & ~RFM12_PWRMGT_EW);
		rfm12_data(ctrl.pwrmgt_shadow);

		RFM12_INT_ON();
	}
#endif /* RFM12_USE_WAKEUP_TIMER */


/************************
 * rfm12 power up / power down
*/

#if RFM12_USE_POWER_CONTROL
	//! This function powers down the rfm12 modules receiver to save power.
	/**
	 * It can not receive in that state.
	 */
	void rfm12_power_down() {
		//wait for rfm12 to get to state STATE_RX_IDLE
		//before turning of the receiver.
		//reason: this way transmissions that have been triggered before
		//can be completed before we power down the rfm12.

		while (ctrl.rfm12_state != STATE_RX_IDLE);

		//disable the interrupt (as we're working directly with the transceiver now)
		//we won't loose interrupts, as the AVR caches them in the int flag
		RFM12_INT_OFF();

		//disable receiver
		ctrl.pwrmgt_shadow &= ~RFM12_PWRMGT_ER;
		rfm12_data(ctrl.pwrmgt_shadow);

		ctrl.rfm12_state = STATE_POWER_DOWN;

		RFM12_INT_ON();
	}

	//! This function powers the rfm12 modules receiver back up again
	/**
	 * Should only be called after rfm12_power_down()
	 */
	void rfm12_power_up() {
		//disable the interrupt (as we're working directly with the transceiver now)
		//we won't loose interrupts, as the AVR caches them in the int flag
		RFM12_INT_OFF();

		ctrl.rfm12_state = STATE_RX_IDLE;

		//enable receiver
		ctrl.pwrmgt_shadow |= RFM12_PWRMGT_ER;
		rfm12_data(ctrl.pwrmgt_shadow);

		RFM12_INT_ON();
	}

#endif

/************************
 * rfm12 low battery detector mode
*/

#if RFM12_LOW_BATT_DETECTOR
	//! This function sets the low battery detector and microcontroller clock divider register.
	/** \param [val]  The register value to be passed to the rf12. \n
	* See the rf12 datasheet for valid values.
	* \see rfm12_get_batt_status()
	*/
	void rfm12_set_batt_detector(uint16_t val) {
		//disable the interrupt (as we're working directly with the transceiver now)
		//we won't loose interrupts, as the AVR caches them in the int flag
		RFM12_INT_OFF();

		//set the low battery detector and microcontroller clock divider register
		rfm12_data (RFM12_CMD_LBDMCD | (val & 0x01FF));

		RFM12_INT_ON();
	}

	//! Return the current low battery detector status.
	/** \returns One of these \ref batt_states "battery states" .
	* \see rfm12_set_batt_detector() and the \ref batt_states "battery state" defines
	*/
	uint8_t rfm12_get_batt_status(void) {
		return ctrl.low_batt;
	}
#endif /* RFM12_LOW_BATT_DETECTOR */

// end rfm12_extra.c

/************************
 * Begin of library
*/


//! Interrupt handler to handle all transmit and receive data transfers to the rfm12.
/** The receiver will generate an interrupt request (IT) for the
* microcontroller - by pulling the nIRQ pin low - on the following events:
* - The TX register is ready to receive the next byte (RGIT)
* - The FIFO has received the preprogrammed amount of bits (FFIT)
* - Power-on reset (POR)
* - FIFO overflow (FFOV) / TX register underrun (RGUR)
* - Wake-up timer timeout (WKUP)
* - Negative pulse on the interrupt input pin nINT (EXT)
* - Supply voltage below the preprogrammed value is detected (LBD)
*
* The rfm12 status register is read to determine which event has occured.
* Reading the status register will clear the event flags.
*
* The interrupt handles the RGIT and FFIT events by default.
* Upon specific configuration of the library the WKUP and LBD events
* are handled additionally.
*
* \see rfm12_control_t, rf_rx_buffer_t and rf_tx_buffer_t
*/
//if polling is used, do not define an interrupt handler, but a polling function
#if (RFM12_USE_POLLING)
void rfm12_poll(void)
#else
ISR(RFM12_INT_VECT, ISR_NOBLOCK)
#endif
{
	RFM12_INT_OFF();
	uint8_t status;
	uint8_t recheck_interrupt;
	//if receive mode is not disabled (default)
	#if !(RFM12_TRANSMIT_ONLY)
		static uint8_t checksum; //static local variables produce smaller code size than globals
	#endif /* !(RFM12_TRANSMIT_ONLY) */

	do {
		//clear AVR int flag
		RFM12_INT_FLAG = (1<<RFM12_FLAG_BIT);

		//first we read the first byte of the status register
		//to get the interrupt flags
		status = rfm12_read_int_flags_inline();

		//if we use at least one of the status bits, we need to check the status again
		//for the case in which another interrupt condition occured while we were handeling
		//the first one.
		recheck_interrupt = 0;

		#if RFM12_UART_DEBUG >= 2
			serial_putc('S');
			serial_putc(status);
		#endif

		//low battery detector feature
		#if RFM12_LOW_BATT_DETECTOR
			if (status & (RFM12_STATUS_LBD >> 8)) {
				//debug
				#if RFM12_UART_DEBUG >= 2
					serial_putc('L');
				#endif

				//set status variable to low battery
				ctrl.low_batt = RFM12_BATT_LOW;
				recheck_interrupt = 1;
			}
		#endif /* RFM12_LOW_BATT_DETECTOR */

		//wakeup timer feature
		#if RFM12_USE_WAKEUP_TIMER
			if (status & (RFM12_STATUS_WKUP >> 8)) {
				//debug
				#if RFM12_UART_DEBUG >= 2
					serial_putc('W');
				#endif

				ctrl.wkup_flag = 1;
				recheck_interrupt = 1;
			}
			if (status & ((RFM12_STATUS_WKUP | RFM12_STATUS_FFIT) >> 8) ) {
				//restart the wakeup timer by toggling the bit on and off
				rfm12_data(ctrl.pwrmgt_shadow & ~RFM12_PWRMGT_EW);
				rfm12_data(ctrl.pwrmgt_shadow);
			}
		#endif /* RFM12_USE_WAKEUP_TIMER */

		//check if the fifo interrupt occurred
		if (status & (RFM12_STATUS_FFIT>>8)) {
			//yes
			recheck_interrupt = 1;
			//see what we have to do (start rx, rx or tx)
			switch (ctrl.rfm12_state) {
				case STATE_RX_IDLE: {
					//if receive mode is not disabled (default)
					#if !(RFM12_TRANSMIT_ONLY)
						uint8_t data;

						//init the bytecounter - remember, we will read the length byte, so this must be 1
						ctrl.bytecount = 1;

						//read the length byte,  and write it to the checksum
						//remember, the first byte is the length byte
						data = rfm12_read(RFM12_CMD_READ);
						checksum = data;

						//add the packet overhead and store it into a working variable
						ctrl.num_bytes = data + PACKET_OVERHEAD;

						//debug
						#if RFM12_UART_DEBUG >= 2
							serial_putc('I');
							serial_putc(data);
						#endif

						//see whether our buffer is free
						//FIXME: put this into global statekeeping struct, the free state can be set by the function which pulls the packet, i guess
						if (rf_rx_buffers[ctrl.buffer_in_num].status == STATUS_FREE) {
							//the current receive buffer is empty, so we start receiving
							ctrl.rfm12_state = STATE_RX_ACTIVE;

							//store the received length into the packet buffer
							//this length field will be used by application reading the
							//buffer.
							rf_rx_buffers[ctrl.buffer_in_num].len = data;

							//end the interrupt without resetting the fifo
							goto no_fifo_reset;
						}

						/* if we're here, the buffer is full, so we ignore this transmission by resetting the fifo (at the end of the function)  */
					#endif /* !(RFM12_TRANSMIT_ONLY) */

					} break;

				case STATE_RX_ACTIVE: {
					//if receive mode is not disabled (default)
					#if !(RFM12_TRANSMIT_ONLY)
						uint8_t data;
						//read a byte
						data = rfm12_read(RFM12_CMD_READ);

						//check if transmission is complete
						if (ctrl.bytecount < ctrl.num_bytes) {
							//debug
							#if RFM12_UART_DEBUG >= 2
								uart_putc('R');
								uart_putc(data);
							#endif

							//xor the remaining bytes onto the checksum
							//note: only the header will be effectively checked
							checksum ^= data;

							//put next byte into buffer, if there is enough space
							if (ctrl.bytecount < (RFM12_RX_BUFFER_SIZE + 3)) {
								//hackhack: begin writing to struct at offsetof len
								(& rf_rx_buffers[ctrl.buffer_in_num].len)[ctrl.bytecount] = data;
							}
#ifndef DISABLE_CHECKSUMM
							//check header against checksum
							if (ctrl.bytecount == 2 && checksum != 0xff) {
								//if the checksum does not match, reset the fifo
								break;
							}
#endif

							//increment bytecount
							ctrl.bytecount++;

							//end the interrupt without resetting the fifo
							goto no_fifo_reset;
						}

						/* if we're here, receiving is done */
						/* the fifo will be reset at the end of the function */

						//debug
						#if RFM12_UART_DEBUG >= 2
							uart_putc('D');
						#endif

						//indicate that the buffer is ready to be used
						rf_rx_buffers[ctrl.buffer_in_num].status = STATUS_COMPLETE;

						#if RFM12_USE_RX_CALLBACK
							if (rfm12_rx_callback_func != 0x0000) {
								rfm12_rx_callback_func (ctrl.rf_buffer_in->len, ctrl.rf_buffer_in.buffer);
							}
						#endif

						//switch to other buffer
						ctrl.buffer_in_num ^= 1;

						#if RFM12_USE_RX_CALLBACK
							rfm12_rx_clear(); /* clear immediately since the data has been processed by the callback func */
						#endif
					#endif /* !(RFM12_TRANSMIT_ONLY) */
					} break;

				case STATE_TX:
					//debug
					#if RFM12_UART_DEBUG >= 2
						uart_putc('T');
					#endif

					if (ctrl.bytecount < ctrl.num_bytes) {
						//load the next byte from our buffer struct.
						rfm12_data( RFM12_CMD_TX | rf_tx_buffer.sync[ctrl.bytecount++]);

						//end the interrupt without resetting the fifo
						goto no_fifo_reset;
					}

					/* if we're here, we're finished transmitting the bytes */
					/* the fifo will be reset at the end of the function */

					//Transmitter on RFM12BP off
					#ifdef TX_LEAVE_HOOK
						TX_LEAVE_HOOK;
					#endif

					//flag the buffer as free again
					ctrl.txstate = STATUS_FREE;

					//turn off the transmitter and enable receiver
					//the receiver is not enabled in transmit only mode (by PWRMGT_RECEIVE makro)
					#if RFM12_PWRMGT_SHADOW
						ctrl.pwrmgt_shadow &= ~(RFM12_PWRMGT_ET); /* disable transmitter */
						ctrl.pwrmgt_shadow |= (PWRMGT_RECEIVE);   /* activate predefined receive mode */
						rfm12_data(ctrl.pwrmgt_shadow);
					#else /* no RFM12_PWRMGT_SHADOW */
						rfm12_data( PWRMGT_RECEIVE );
					#endif /* RFM12_PWRMGT_SHADOW */

					//Receiver on RFM12BP on
					#ifdef RX_ENTER_HOOK
						RX_ENTER_HOOK;
					#endif

					//load a dummy byte to clear int status
					rfm12_data( RFM12_CMD_TX | 0xaa);
					break;
					#if RFM12_USE_POWER_CONTROL
						case STATE_POWER_DOWN:
							//load a dummy byte to clear int status
							rfm12_data( RFM12_CMD_TX | 0xaa);
							break;
					#endif
			}//end of switch

			//set the state machine to idle
			ctrl.rfm12_state = STATE_RX_IDLE;

			//reset the receiver fifo, if receive mode is not disabled (default)
			#if !(RFM12_TRANSMIT_ONLY)
				#if RFM12_UART_DEBUG >= 2
					uart_putc('F');
				#endif
				rfm12_data( RFM12_CMD_FIFORESET | CLEAR_FIFO_INLINE);
				rfm12_data( RFM12_CMD_FIFORESET | ACCEPT_DATA_INLINE);
			#endif /* !(RFM12_TRANSMIT_ONLY) */

			uint8_t b;
			no_fifo_reset:
			b = b;
		}
	} while (recheck_interrupt);

	#if RFM12_UART_DEBUG >= 2
		uart_putc('E');
	#endif

	//turn the int back on
	RFM12_INT_ON();
}


//! The tick function implements collision avoidance and initiates transmissions.
/** This function has to be called periodically.
* It will read the rfm12 status register to check if a carrier is being received,
* which would indicate activity on the chosen radio channel. \n
* If there has been no activity for long enough, the channel is believed to be free.
*
* When there is a packet waiting for transmission and the collision avoidance
* algorithm indicates that the air is free, then the interrupt control variables are
* setup for packet transmission and the rfm12 is switched to transmit mode.
* This function also fills the rfm12 tx fifo with a preamble.
*
* \warning Warning, if you do not call this function periodically, then no packet will get transmitted.
* \see rfm12_tx() and rfm12_start_tx()
*/
void rfm12_tick(void) {
	//collision detection is enabled by default
	#if !(RFM12_NOCOLLISIONDETECTION)
		uint16_t status;

		//start with a channel free count of 16, this is necessary for the ASK receive feature to work
		static uint8_t channel_free_count = 16; //static local variables produce smaller code size than globals
	#endif

	//debug
	#if RFM12_UART_DEBUG
		static uint8_t oldstate;
		uint8_t state = ctrl.rfm12_state;
		if (oldstate != state) {
			uart_putstr ("mode change: ");
			switch (state) {
				case STATE_RX_IDLE:
					uart_putc ('i');
					break;
				case STATE_RX_ACTIVE:
					uart_putc ('r');
					break;
				case STATE_TX:
					uart_putc ('t');
					break;
				default:
					uart_putc ('?');
			}
			uart_putstr ("\r\n");
			oldstate = state;
		}
	#endif

	//don't disturb RFM12 if transmitting or receiving
	if (ctrl.rfm12_state != STATE_RX_IDLE) {
		return;
	}

	//collision detection is enabled by default
	#if !(RFM12_NOCOLLISIONDETECTION)
		//disable the interrupt (as we're working directly with the transceiver now)
		//hint: we could be losing an interrupt here, because we read the status register.
		//this applys for the Wakeup timer, as it's flag is reset by reading.
		RFM12_INT_OFF();
		status = rfm12_read(RFM12_CMD_STATUS);
		RFM12_INT_ON();

		//wakeup timer workaround (if we don't restart the timer after timeout, it will stay off.)
		#if RFM12_USE_WAKEUP_TIMER
			if (status & (RFM12_STATUS_WKUP)) {
				ctrl.wkup_flag = 1;

				RFM12_INT_OFF();
				//restart the wakeup timer by toggling the bit on and off
				rfm12_data(ctrl.pwrmgt_shadow & ~RFM12_PWRMGT_EW);
				rfm12_data(ctrl.pwrmgt_shadow);
				RFM12_INT_ON();
			}
		#endif /* RFM12_USE_WAKEUP_TIMER */

		//check if we see a carrier
		if (status & RFM12_STATUS_RSSI) {
			//yes: reset free counter and return
			channel_free_count = CHANNEL_FREE_TIME;
			return;
		}
		//no

		//is the channel free long enough ?
		if (channel_free_count != 0) {
			//no:
			channel_free_count--; // decrement counter
			return;
		}
		//yes: we can begin transmitting
	#endif

	//do we have something to transmit?
	if (ctrl.txstate == STATUS_OCCUPIED) { //yes: start transmitting
		//disable the interrupt (as we're working directly with the transceiver now)
		//we won't loose interrupts, as the AVR caches them in the int flag.
		//we could disturb an ongoing reception,
		//if it has just started some cpu cycles ago
		//(as the check for this case is some lines (cpu cycles) above)
		//anyhow, we MUST transmit at some point...
		RFM12_INT_OFF();

		//disable receiver - if you don't do this, tx packets will get lost
		//as the fifo seems to be in use by the receiver

		#if RFM12_PWRMGT_SHADOW
			ctrl.pwrmgt_shadow &= ~(RFM12_PWRMGT_ER); /* disable receiver */
			rfm12_data(ctrl.pwrmgt_shadow);
		#else
			rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT ); /* disable receiver */
		#endif

		//RFM12BP receiver off
		#ifdef RX_LEAVE_HOOK
			RX_LEAVE_HOOK;
		#endif

		//calculate number of bytes to be sent by ISR
		//2 sync bytes + len byte + type byte + checksum + message length + 1 dummy byte
		ctrl.num_bytes = rf_tx_buffer.len + 6;

		//reset byte sent counter
		ctrl.bytecount = 0;

		//set mode for interrupt handler
		ctrl.rfm12_state = STATE_TX;

		//RFM12BP transmitter on
		#ifdef TX_ENTER_HOOK
			TX_ENTER_HOOK;
		#endif

		//fill 2byte 0xAA preamble into data register
		//the preamble helps the receivers AFC circuit to lock onto the exact frequency
		//(hint: the tx FIFO [if el is enabled] is two staged, so we can safely write 2 bytes before starting)
		rfm12_data(RFM12_CMD_TX | PREAMBLE);
		rfm12_data(RFM12_CMD_TX | PREAMBLE);

		//set ET in power register to enable transmission (hint: TX starts now)
		#if RFM12_PWRMGT_SHADOW
			ctrl.pwrmgt_shadow |= RFM12_PWRMGT_ET;
			rfm12_data (ctrl.pwrmgt_shadow);
		#else
			rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT | RFM12_PWRMGT_ET);
		#endif

		//enable the interrupt to continue the transmission
		RFM12_INT_ON();
	}
}


//! Enqueue an already buffered packet for transmission
/** If there is no active transmission, the packet header is written to the
* transmission control buffer and the packet will be enqueued for transmission. \n
* This function is not responsible for buffering the actual packet data.
* The data has to be copied into the transmit buffer beforehand,
* which can be accomplished by the rfm12_tx() function.
*
* \note Note that this function does not start the transmission, it merely enqueues the packet. \n
* Transmissions are started by rfm12_tick().
* \param [type] The packet header type field
* \param [length] The packet data length
* \returns One of these defines: \ref tx_retvals "TX return values"
* \see rfm12_tx() and rfm12_tick()
*/
#if (RFM12_NORETURNS)
void
#else
uint8_t
#endif
rfm12_start_tx(uint8_t type, uint8_t length) {
	//exit if the buffer isn't free
	if (ctrl.txstate != STATUS_FREE)
		return TXRETURN(RFM12_TX_OCCUPIED);

	//write airlab header to buffer
	rf_tx_buffer.len = length;
	rf_tx_buffer.type = type;
	rf_tx_buffer.checksum = length ^ type ^ 0xff;

	//schedule packet for transmission
	ctrl.txstate = STATUS_OCCUPIED;

	return TXRETURN(RFM12_TX_ENQUEUED);
}


//! Copy a packet to the buffer and call rfm12_start_tx() to enqueue it for transmission.
/** If there is no active transmission, the buffer contents will be copied to the
* internal transmission buffer. Finally the buffered packet is going to be enqueued by
* calling rfm12_start_tx(). If automatic buffering of packet data is not necessary,
* which is the case when the packet data does not change while the packet is enqueued
* for transmission, then one could directly store the data in \ref rf_tx_buffer
* (see rf_tx_buffer_t) and use the rfm12_start_tx() function.
*
* \note Note that this function does not start the transmission, it merely enqueues the packet. \n
* Transmissions are started by rfm12_tick().
* \param [len] The packet data length
* \param [type] The packet header type field
* \param [data] Pointer to the packet data
* \returns One of these defines: \ref tx_retvals "TX return values"
* \see rfm12_start_tx() and rfm12_tick()
*/
#if !(RFM12_SMALLAPI)
	#if (RFM12_NORETURNS)
	void
	#else
	uint8_t
	#endif
	rfm12_tx(uint8_t len, uint8_t type, uint8_t *data) {
		#if RFM12_UART_DEBUG
			uart_putstr ("sending packet\r\n");
		#endif

		if (len > RFM12_TX_BUFFER_SIZE) return TXRETURN(RFM12_TX_ERROR);

		//exit if the buffer isn't free
		if (ctrl.txstate != STATUS_FREE)
			return TXRETURN(RFM12_TX_OCCUPIED);

		memcpy(rf_tx_buffer.buffer, data, len);

		#if (!(RFM12_NORETURNS))
		return rfm12_start_tx(type, len);
		#else
		rfm12_start_tx(type, len);
		#endif
	}
#endif /* RFM12_SMALLAPI */


//if receive mode is not disabled (default)
#if !(RFM12_TRANSMIT_ONLY)
	//! Function to clear buffer complete/occupied status.
	/** This function will set the current receive buffer status to free and switch
	* to the other buffer, which can then be read using rfm12_rx_buffer().
	*
	* \see rfm12_rx_status(), rfm12_rx_len(), rfm12_rx_type(), rfm12_rx_buffer() and rf_rx_buffers
	*/
	//warning: without the attribute, gcc will inline this even if -Os is set
	void __attribute__((noinline)) rfm12_rx_clear(void) {
			//mark the current buffer as empty
			rf_rx_buffers[ctrl.buffer_out_num].status = STATUS_FREE;

			//switch to the other buffer
			ctrl.buffer_out_num ^= 1;

	}
#endif /* !(RFM12_TRANSMIT_ONLY) */


//enable internal data register and fifo
//setup selected band
#define RFM12_CMD_CFG_DEFAULT   (RFM12_CMD_CFG | RFM12_CFG_EL | RFM12_CFG_EF | RFM12_BASEBAND | RFM12_XTAL_LOAD)

//set rx parameters: int-in/vdi-out pin is vdi-out,
//Bandwith, LNA, RSSI
#define RFM12_CMD_RXCTRL_DEFAULT (RFM12_CMD_RXCTRL | RFM12_RXCTRL_P16_VDI | RFM12_RXCTRL_VDI_FAST | RFM12_FILTER_BW | RFM12_LNA_GAIN | RFM12_RSSI_THRESHOLD )

//set AFC to automatic, (+4 or -3)*2.5kHz Limit, fine mode, active and enabled
#define RFM12_CMD_AFC_DEFAULT  (RFM12_CMD_AFC | RFM12_AFC_AUTO_KEEP | RFM12_AFC_LIMIT_4 | RFM12_AFC_FI | RFM12_AFC_OE | RFM12_AFC_EN)

//set TX Power, frequency shift
#define RFM12_CMD_TXCONF_DEFAULT  (RFM12_CMD_TXCONF | RFM12_POWER | RFM12_TXCONF_FS_CALC(FSK_SHIFT) )

static const uint16_t init_cmds[] PROGMEM = {
	//defined above (so shadow register is inited with same value)
	RFM12_CMD_CFG_DEFAULT,

	//set power default state (usually disable clock output)
	//do not write the power register two times in a short time
	//as it seems to need some recovery
	(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT),

	//set frequency
	(RFM12_CMD_FREQUENCY | RFM12_FREQUENCY_CALC(RFM12_FREQUENCY) ),

	//set data rate
	(RFM12_CMD_DATARATE | DATARATE_VALUE ),

	//defined above (so shadow register is inited with same value)
	RFM12_CMD_RXCTRL_DEFAULT,

	//automatic clock lock control(AL), digital Filter(!S),
	//Data quality detector value 3, slow clock recovery lock
	(RFM12_CMD_DATAFILTER | RFM12_DATAFILTER_AL | 3),

	//2 Byte Sync Pattern, Start fifo fill when sychron pattern received,
	//disable sensitive reset, Fifo filled interrupt at 8 bits
	(RFM12_CMD_FIFORESET | RFM12_FIFORESET_DR | (8 << 4)),

	//defined above (so shadow register is inited with same value)
	RFM12_CMD_AFC_DEFAULT,

	//defined above (so shadow register is inited with same value)
	RFM12_CMD_TXCONF_DEFAULT,

	//disable low dutycycle mode
	(RFM12_CMD_DUTYCYCLE),

	//disable wakeup timer
	(RFM12_CMD_WAKEUP),

	//enable rf receiver chain, if receiving is not disabled (default)
	//the magic is done via defines
	(RFM12_CMD_PWRMGT | PWRMGT_RECEIVE),
};

//! This is the main library initialization function
/**This function takes care of all module initialization, including:
* - Setup of the used frequency band and external capacitor
* - Setting the exact frequency (channel)
* - Setting the transmission data rate
* - Configuring various module related rx parameters, including the amplification
* - Enabling the digital data filter
* - Enabling the use of the modules fifo, as well as enabling sync pattern detection
* - Configuring the automatic frequency correction
* - Setting the transmit power
*
* This initialization function also sets up various library internal configuration structs and
* puts the module into receive mode before returning.
*
* \note Please note that the transmit power and receive amplification values are currently hard coded.
* Have a look into rfm12_hw.h for possible settings.
*/
void rfm12_init(void) {
	//initialize spi
	SS_RELEASE();
	RF_DDR_SS |= (1<<RF_BIT_SS);
	spi_init();

	//typically sets DDR registers for RFM12BP TX/RX pin
	#ifdef TX_INIT_HOOK
		TX_INIT_HOOK;
	#endif

	#ifdef RX_INIT_HOOK
		RX_INIT_HOOK;
	#endif

	//store the syncronization pattern to the transmission buffer
	//the sync pattern is used by the receiver to distinguish noise from real transmissions
	//the sync pattern is hardcoded into the receiver
	rf_tx_buffer.sync[0] = SYNC_MSB;
	rf_tx_buffer.sync[1] = SYNC_LSB;

	//if receive mode is not disabled (default)
	#if !(RFM12_TRANSMIT_ONLY)
		//init buffer pointers
		ctrl.buffer_in_num = 0;
		ctrl.buffer_out_num = 0;
	#endif /* !(RFM12_TRANSMIT_ONLY) */

	//low battery detector feature initialization
	#if RFM12_LOW_BATT_DETECTOR
		ctrl.low_batt = RFM12_BATT_OKAY;
	#endif /* RFM12_LOW_BATT_DETECTOR */

	#if RFM12_PWRMGT_SHADOW
		//set power management shadow register to receiver chain enabled or disabled
		//the define correctly handles the transmit only mode
		ctrl.pwrmgt_shadow = (RFM12_CMD_PWRMGT | PWRMGT_RECEIVE);
	#endif


	#if RFM12_LIVECTRL
		//init shadow registers with values about to be written to rfm12
		ctrl.rxctrl_shadow = RFM12_CMD_RXCTRL_DEFAULT;
		ctrl.afc_shadow = RFM12_CMD_AFC_DEFAULT;
		ctrl.txconf_shadow = RFM12_CMD_TXCONF_DEFAULT;
		ctrl.cfg_shadow =    RFM12_CMD_CFG_DEFAULT;
	#endif

	//write all the initialisation values to rfm12
	uint8_t x;
	for (x = 0; x < ( sizeof(init_cmds) / 2) ; x++) {
		rfm12_data(pgm_read_word(&init_cmds[x]));
	}

	#ifdef RX_ENTER_HOOK
		RX_ENTER_HOOK;
	#endif

	#if RFM12_USE_CLOCK_OUTPUT || RFM12_LOW_BATT_DETECTOR
		rfm12_data(RFM12_CMD_LBDMCD | RFM12_LBD_VOLTAGE | RFM12_CLOCK_OUT_FREQUENCY ); //set low battery detect, clock output
	#endif

	//ASK receive mode feature initialization
	#if RFM12_RECEIVE_ASK
		adc_init();
	#endif

	//setup interrupt for falling edge trigger
	RFM12_INT_SETUP();

	//clear int flag
	rfm12_read(RFM12_CMD_STATUS);
	RFM12_INT_FLAG = (1<<RFM12_FLAG_BIT);

	//init receiver fifo, we now begin receiving.
	rfm12_data(CLEAR_FIFO);
	rfm12_data(ACCEPT_DATA);

	//activate the interrupt
	RFM12_INT_ON();
}





