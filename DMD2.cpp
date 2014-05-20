/*
  DMD2 Implementation of SPIDMD, SoftDMD.

 Copyright (C) 2014 Freetronics, Inc. (info <at> freetronics <dot> com)

 Updated by Angus Gratton, based on DMD by Marc Alexander.

---

 This program is free software: you can redistribute it and/or modify it under the terms
 of the version 3 GNU General Public License as published by the Free Software Foundation.

 This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the GNU General Public License for more details.

 You should have received a copy of the GNU General Public License along with this program.
 If not, see <http://www.gnu.org/licenses/>.
*/
#include "DMD2.h"

// Port registers are same size as a pointer (16-bit on AVR, 32-bit on ARM)
typedef intptr_t port_reg_t;

#ifdef SPARK_CORE
// NOTE: default pins may be changed - the nOE pin must support PWM (on Spark: A0-A7, DO, D1)
SPIDMD::SPIDMD(byte panelsWide, byte panelsHigh)
  : BaseDMD(panelsWide, panelsHigh, A0, D4, D3, D2)		// nOE=A0, A=D4, B=D3, SCK=D2
{
}
#else // Arduino
SPIDMD::SPIDMD(byte panelsWide, byte panelsHigh)
  : BaseDMD(panelsWide, panelsHigh, 9, 6, 7, 8)
{
}
#endif



/* Create a DMD display using a custom pinout for all the non-SPI pins (SPI pins set by hardware) */
SPIDMD::SPIDMD(byte panelsWide, byte panelsHigh, byte pin_noe, byte pin_a, byte pin_b, byte pin_sck)
  : BaseDMD(panelsWide, panelsHigh, pin_noe, pin_a, pin_b, pin_sck)
{
}

void SPIDMD::beginNoTimer()
{
  // Configure SPI before initialising the base DMD
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);	// CPOL=0, CPHA=0

#ifdef SPARK_CORE
  SPI.setClockDivider(SPI_CLOCK_DIV16); // 4.5MHz clock
#else
#ifdef __AVR__
  SPI.setClockDivider(SPI_CLOCK_DIV4); // 4MHz clock
#else
  SPI.setClockDivider(20); // 4.2MHz on Due
#endif //AVR
#endif //SPARK

  BaseDMD::beginNoTimer();
}

void SPIDMD::writeSPIData(volatile uint8_t *rows[4], const int rowsize)
{
  for(int i = 0; i < rowsize; i++) {
    SPI.transfer(*(rows[3]++));
    SPI.transfer(*(rows[2]++));
    SPI.transfer(*(rows[1]++));
    SPI.transfer(*(rows[0]++));
  }
}

void BaseDMD::scanDisplay()
{
  if(pin_other_cs >= 0 && digitalRead(pin_other_cs) != HIGH)
    return;
  // Rows are send out in 4 blocks of 4 (interleaved), across all panels

  int rowsize = unified_width() / 8; // in bytes

  volatile uint8_t *bmp = bitmap;

  volatile uint8_t *rows[4] = { // Scanning out 4 interleaved rows
    bmp + (scan_row + 0) * rowsize,
    bmp + (scan_row + 4) * rowsize,
    bmp + (scan_row + 8) * rowsize,
    bmp + (scan_row + 12) * rowsize,
  };

  writeSPIData(rows, rowsize);

  digitalWrite(pin_noe, LOW);
  digitalWrite(pin_sck, HIGH); // Latch DMD shift register output
  digitalWrite(pin_sck, LOW); // (Deliberately left as digitalWrite to ensure decent latching time)

  // A, B determine which set of interleaved rows we are multiplexing on
  // 0 = 1,5,9,13
  // 1 = 2,6,10,14
  // 2 = 3,7,11,15
  // 3 = 4,8,12,16
  digitalWrite(pin_a, scan_row & 0x01);
  digitalWrite(pin_b, scan_row & 0x02);
  scan_row = (scan_row + 1) % 4;

  if(brightness == 255)
    digitalWrite(pin_noe, HIGH);
  else
    analogWrite(pin_noe, brightness);
}

#ifdef SPARK_CORE
// Spark Core version
SoftDMD::SoftDMD(byte panelsWide, byte panelsHigh)
  : BaseDMD(panelsWide, panelsHigh, A0, D4, D3, D2),
    pin_clk(SCK),		// default to h/w SPI SCK pin
    pin_r_data(MOSI)	// default to h/w SPI MOSI pin
{
}
#else
// Arduino version
SoftDMD::SoftDMD(byte panelsWide, byte panelsHigh)
  : BaseDMD(panelsWide, panelsHigh, 9, 6, 7, 8),
    pin_clk(13),
    pin_r_data(11)
{
}
#endif

SoftDMD::SoftDMD(byte panelsWide, byte panelsHigh, byte pin_noe, byte pin_a, byte pin_b, byte pin_sck,
          byte pin_clk, byte pin_r_data)
  : BaseDMD(panelsWide, panelsHigh, pin_noe, pin_a, pin_b, pin_sck),
    pin_clk(pin_clk),
    pin_r_data(pin_r_data)
{
}

void SoftDMD::beginNoTimer()
{
  digitalWrite(pin_clk, LOW);
  pinMode(pin_clk, OUTPUT);

  digitalWrite(pin_r_data, LOW);
  pinMode(pin_r_data, OUTPUT);
  BaseDMD::beginNoTimer();
}

#ifdef SPARK_CORE
// Spark Core version
// Software SPI using Spark fast GPIO
// Uses specified SPI pins dataPin (MOSI) and clkPin (SCK).
static inline __attribute__((always_inline)) void softSPITransfer(uint8_t data, volatile byte clkPin, volatile byte dataPin) {
	uint8_t b=0;

	for (uint8_t bit = 0; bit < 8; bit++)  {
		if (data & (1 << (7-bit)))		// walks down mask from bit 7 to bit 0
			PIN_MAP[dataPin].gpio_peripheral->BSRR = PIN_MAP[dataPin].gpio_pin; // Data High
		else
			PIN_MAP[dataPin].gpio_peripheral->BRR = PIN_MAP[dataPin].gpio_pin; // Data Low
		
		PIN_MAP[clkPin].gpio_peripheral->BSRR = PIN_MAP[clkPin].gpio_pin; // Clock High

		// DUMMY READ OF MISO but do nothing with it - adds a bit of delay between clock high/low
		b <<= 1;
		if (PIN_MAP[MISO].gpio_peripheral->IDR & PIN_MAP[MISO].gpio_pin)
			b |= 1;

		PIN_MAP[clkPin].gpio_peripheral->BRR = PIN_MAP[clkPin].gpio_pin; // Clock Low
	}
}
#else
// Arduino version
static inline __attribute__((always_inline)) void softSPITransfer(uint8_t data, volatile port_reg_t *data_port, port_reg_t data_mask, volatile port_reg_t *clk_port, port_reg_t clk_mask) {
  // MSB first, data captured on rising edge
  for(uint8_t bit = 0; bit < 8; bit++) {
    if(data & (1<<7))
      *data_port |= data_mask;
    else
      *data_port &= ~data_mask;
    *clk_port |= clk_mask;
    data <<= 1;
    *clk_port &= ~clk_mask;
  }
}
#endif


void SoftDMD::writeSPIData(volatile uint8_t *rows[4], const int rowsize)
{
#ifdef SPARK_CORE
  // Spark Core version
  for(int i = 0; i < rowsize; i++) {
    softSPITransfer(*(rows[3]++), pin_clk, pin_r_data);
    softSPITransfer(*(rows[2]++), pin_clk, pin_r_data);
    softSPITransfer(*(rows[1]++), pin_clk, pin_r_data);
    softSPITransfer(*(rows[0]++), pin_clk, pin_r_data);
  }
#else
  // Arduino version
  volatile port_reg_t *port_clk = (volatile port_reg_t *)portOutputRegister(digitalPinToPort(pin_clk));
  port_reg_t mask_clk = digitalPinToBitMask(pin_clk);
  volatile port_reg_t *port_r_data = (volatile port_reg_t *) portOutputRegister(digitalPinToPort(pin_r_data));
  port_reg_t mask_r_data = digitalPinToBitMask(pin_r_data);

  for(int i = 0; i < rowsize; i++) {
    softSPITransfer(*(rows[3]++), port_r_data, mask_r_data, port_clk, mask_clk);
    softSPITransfer(*(rows[2]++), port_r_data, mask_r_data, port_clk, mask_clk);
    softSPITransfer(*(rows[1]++), port_r_data, mask_r_data, port_clk, mask_clk);
    softSPITransfer(*(rows[0]++), port_r_data, mask_r_data, port_clk, mask_clk);
  }
#endif
}

BaseDMD::BaseDMD(byte panelsWide, byte panelsHigh, byte pin_noe, byte pin_a, byte pin_b, byte pin_sck)
  :
  DMDFrame(panelsWide, panelsHigh),
  scan_row(0),
  pin_noe(pin_noe),
  pin_a(pin_a),
  pin_b(pin_b),
  pin_sck(pin_sck),
#ifdef SPARK_CORE
  // Spark Core version (nOE=A0, A=D4, B=D3, SCK=D2)
  default_pins(pin_noe == A0 && pin_a == D4 && pin_b == D3 && pin_sck == D2),
#else
  // Arduino version
  default_pins(pin_noe == 9 && pin_a == 6 && pin_b == 7 && pin_sck == 8),
#endif
  pin_other_cs(-1),
  brightness(255)
{
}

void BaseDMD::beginNoTimer()
{
  digitalWrite(pin_noe, LOW);
  pinMode(pin_noe, OUTPUT);

  digitalWrite(pin_a, LOW);
  pinMode(pin_a, OUTPUT);

  digitalWrite(pin_b, LOW);
  pinMode(pin_b, OUTPUT);

  digitalWrite(pin_sck, LOW);
  pinMode(pin_sck, OUTPUT);

  clearScreen();
  scanDisplay();
}
