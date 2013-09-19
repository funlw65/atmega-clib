/*
    Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use, copy,
    modify, merge, publish, distribute, sublicense, and/or sell copies
    of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.

    $Id$
*/
#ifndef NRF24L01_H_
#define NRF24L01_H_

/* The registers are prefixed with "RF24_" to avoid an eventual conflict in our
 * multi-library so, when you read the documentation,
 * remove the prefix in your mind (ignore it)...
 */

/* Memory Map */
#define RF24_CONFIG      0x00
#define RF24_EN_AA       0x01
#define RF24_EN_RXADDR   0x02
#define RF24_SETUP_AW    0x03
#define RF24_SETUP_RETR  0x04
#define RF24_RF_CH       0x05
#define RF24_RF_SETUP    0x06
#define RF24_STATUS      0x07
#define RF24_OBSERVE_TX  0x08
#define RF24_CD          0x09
#define RF24_RX_ADDR_P0  0x0A
#define RF24_RX_ADDR_P1  0x0B
#define RF24_RX_ADDR_P2  0x0C
#define RF24_RX_ADDR_P3  0x0D
#define RF24_RX_ADDR_P4  0x0E
#define RF24_RX_ADDR_P5  0x0F
#define RF24_TX_ADDR     0x10
#define RF24_RX_PW_P0    0x11
#define RF24_RX_PW_P1    0x12
#define RF24_RX_PW_P2    0x13
#define RF24_RX_PW_P3    0x14
#define RF24_RX_PW_P4    0x15
#define RF24_RX_PW_P5    0x16
#define RF24_FIFO_STATUS 0x17

/* Bit Mnemonics */
#define RF24_MASK_RX_DR  6
#define RF24_MASK_TX_DS  5
#define RF24_MASK_MAX_RT 4
#define RF24_EN_CRC      3
#define RF24_CRCO        2
#define RF24_PWR_UP      1
#define RF24_PRIM_RX     0
#define RF24_ENAA_P5     5
#define RF24_ENAA_P4     4
#define RF24_ENAA_P3     3
#define RF24_ENAA_P2     2
#define RF24_ENAA_P1     1
#define RF24_ENAA_P0     0
#define RF24_ERX_P5      5
#define RF24_ERX_P4      4
#define RF24_ERX_P3      3
#define RF24_ERX_P2      2
#define RF24_ERX_P1      1
#define RF24_ERX_P0      0
#define RF24_AW          0
#define RF24_ARD         4
#define RF24_ARC         0
#define RF24_PLL_LOCK    4
#define RF24_RF_DR       3
#define RF24_RF_PWR      1
#define RF24_LNA_HCURR   0
#define RF24_RX_DR       6
#define RF24_TX_DS       5
#define RF24_MAX_RT      4
#define RF24_RX_P_NO     1
#define RF24_TX_FULL     0
#define RF24_PLOS_CNT    4
#define RF24_ARC_CNT     0
#define RF24_TX_REUSE    6
#define RF24_FIFO_FULL   5
#define RF24_TX_EMPTY    4
#define RF24_RX_FULL     1
#define RF24_RX_EMPTY    0

/* Instruction Mnemonics */
#define RF24_R_REGISTER    0x00
#define RF24_W_REGISTER    0x20
#define RF24_REGISTER_MASK 0x1F
#define RF24_R_RX_PAYLOAD  0x61
#define RF24_W_TX_PAYLOAD  0xA0
#define RF24_FLUSH_TX      0xE1
#define RF24_FLUSH_RX      0xE2
#define RF24_REUSE_TX_PL   0xE3
#define RF24_NOP           0xFF


#endif /* NRF24L01_H_ */
