/*
 * ltc6804.h
 *
 *  Created on: Jul 30, 2025
 *      Author: Jordi
 */

#ifndef INC_LTC6804_H_
#define INC_LTC6804_H_

#include <stdint.h>

/* ---------- Fixed (no parameters) ---------- */
#define WRCFG      0b00000000001
#define RDCFG      0b00000000010

#define RDCVA      0b00000000100
#define RDCVB      0b00000000110
#define RDCVC      0b00000001000
#define RDCVD      0b00000001010

#define RDAUXA     0b00000001100
#define RDAUXB     0b00000001110

#define RDSTATA    0b00000010000
#define RDSTATB    0b00000010010

#define CLRCELL    0b11100010001
#define CLRAUX     0b11100010010
#define CLRSTAT    0b11100010011

#define PLADC      0b11100010100
#define DIAGN      0b11100010101

#define WRCOMM     0b11100100001
#define RDCOMM     0b11100100010
#define STCOMM     0b11100100011


/* ---------- Parameterized (MD, DCP, CH, etc.) ----------
   Bit positions (MSB..LSB): [10:0]
   - MD is 2 bits at [8:7]
   - DCP is 1 bit at [4]
   - CH/CHG/CHST are 3-bit fields at [2:0]
   - PUP is 1 bit at [6]
   - ST is 2 bits at [6:5]
*/

/* Start Cell Voltage ADC Conversion and Poll Status */
#define ADCV(MD,DCP,CH) \
  ( 0b01001100000 \
  | (((MD) & 0x3) << 7) \
  | (((DCP) & 0x1) << 4) \
  |  ((CH) & 0x7) )

/* Start Open-Wire ADC Conversion and Poll Status */
#define ADOW(MD,PUP,DCP,CH) \
  ( 0b01000101000 \
  | (((MD)  & 0x3) << 7) \
  | (((PUP) & 0x1) << 6) \
  | (((DCP) & 0x1) << 4) \
  |  ((CH)  & 0x7) )

/* Start Self-Test Cell Voltage Conversion and Poll Status */
#define CVST(MD,ST) \
  ( 0b01000000111 \
  | (((MD) & 0x3) << 7) \
  | (((ST) & 0x3) << 5) )

/* Start GPIOs ADC Conversion and Poll Status */
#define ADAX(MD,CHG) \
  ( 0b10001100000 \
  | (((MD)  & 0x3) << 7) \
  |  ((CHG) & 0x7) )

/* Start Self-Test GPIOs Conversion and Poll Status */
#define AXST(MD,ST) \
  ( 0b10000000111 \
  | (((MD) & 0x3) << 7) \
  | (((ST) & 0x3) << 5) )

/* Start Status Group ADC Conversion and Poll Status */
#define ADSTAT(MD,CHST) \
  ( 0b10001101000 \
  | (((MD)   & 0x3) << 7) \
  |  ((CHST) & 0x7) )

/* Start Self-Test Status Group Conversion and Poll Status */
#define STATST(MD,ST) \
  ( 0b10000001111 \
  | (((MD) & 0x3) << 7) \
  | (((ST) & 0x3) << 5) )

/* Start Combined Cell Voltage and GPIO1/GPIO2 Conversion and Poll Status */
#define ADCVAX(MD,DCP) \
  ( 0b10001101111 \
  | (((MD)  & 0x3) << 7) \
  | (((DCP) & 0x1) << 4) )

/* Make Address Command */
#define MAKEADDRCMD(ADDR, CMD) \
  ( 0b100000000000000 \
  | ((ADDR) & 0b1111) << 3 \
  | ((CMD >> 8) & 0b111) \
  |  (CMD & 0xFF) )

/* Make Broadcast Command */
#define MAKEBROADCASTCMD(CMD) \
  ((CMD >> 8) & 0b111) \
  |(CMD & 0xFF)

/* ADC Mode */
#define MD_FAST 0b01
#define MD_NORMAL 0b10
#define MD_FILTERED 0b11

/* Discharge permitted */
#define DCP_PERMITTED 0b1
#define DCP_NOT_PERMITTED 0b0

/* Cell selection for ADC conversion */
#define CH_ALL_CELLS 0b000
// TODO: Add other channel definitions as needed

/* Pull up for Open-Wire ADC */
#define PUP_PULL_UP 0b1
#define PUP_PULL_DOWN 0b0

/* Self-Test Mode */
#define ST_TEST1 0b01
#define ST_TEST2 0b10

/* GPIOs for ADC conversion */
#define CHG_ALL GPIOS 0b000
// TODO: Add other GPIO definitions as needed

/* Status Group selection */
#define CHST_ALL_STATUS 0b000
// TODO: Add other status group definitions as needed


/* PEC error detection — pre-computed CRC15 Table */
extern const uint16_t crc15Table[256];

uint16_t pec15_calc(uint8_t len, uint8_t *data);
#endif /* INC_LTC6804_H_ */


