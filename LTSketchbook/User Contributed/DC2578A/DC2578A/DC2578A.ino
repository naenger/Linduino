/*!
Linear Technology DC2578A Demonstration Board
LTC4281, LTC3886, LTC2975, LTC3887: Power Management Solution for Hot-Plug Systems

Demonstrate bus functions for PSM ICs

@verbatim

NOTES
  Setup:
   Set the terminal baud rate to 115200 and select the newline terminator.

@endverbatim

http://www.linear.com/product/LTC4281

http://www.linear.com/product/LTC3886

http://www.linear.com/product/LTC2975

http://www.linear.com/product/LTC3887

REVISION HISTORY
$Revision: 3033 $
$Date: 2017-08-04 14:40:30 -0800 (Thu, 04 Aug 2017) $

Copyright (c) 2016, Linear Technology Corp.(LTC)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of Linear Technology Corp.

The Linear Technology Linduino is not affiliated with the official Arduino team.
However, the Linduino is only possible because of the Arduino team's commitment
to the open-source community.  Please, visit http://www.arduino.cc and
http://store.arduino.cc , and consider a purchase that will help fund their
ongoing work.
*/

/*! @file
    @ingroup DC2578
*/

#include <Arduino.h>
#include <stdint.h>
#include <Linduino.h>
#include <UserInterface.h>
#include <LT_Wire.h>
#include <LT_PMBus.h>
#include <LT_SMBusPec.h>
#include <LT_PMBusMath.h>
#include <LT_SMBus.h>
#include <LT_I2CBus.h>
#include <LT_SMBusGroup.h>
//#include <LT_FaultLog.h>
#include <LT_SMBusNoPec.h>
#include <LT_SMBusBase.h>

#define LTC4281_I2C_ADDRESS 0x46
#define LTC3886_I2C_ADDRESS 0x4F
#define LTC2975_I2C_ADDRESS 0x47
#define LTC3887_I2C_ADDRESS 0x40

// Global variables
static uint8_t ltc4281_i2c_address;
static uint8_t ltc3886_i2c_address;
static uint8_t ltc2975_i2c_address;
static uint8_t ltc3887_i2c_address;

static LT_SMBus *smbus = new LT_SMBusNoPec();
static LT_PMBus *pmbus = new LT_PMBus(smbus);


//! Initialize Linduino
//! @return void
void setup()
{
  Serial.begin(115200);         //! Initialize the serial port to the PC
  print_title();
  ltc4281_i2c_address = LTC4281_I2C_ADDRESS;
  ltc3886_i2c_address = LTC3886_I2C_ADDRESS;
  ltc2975_i2c_address = LTC2975_I2C_ADDRESS;
  ltc3887_i2c_address = LTC3887_I2C_ADDRESS;
  print_prompt();
}

//! Repeats Linduino loop
//! @return void
void loop()
{
  uint8_t user_command;

  if (Serial.available())                          //! Checks for user input
    {
      user_command = read_int();                     //! Reads the user command
      if (user_command != 'm')
	Serial.println(user_command);
      
      switch (user_command)                          //! Prints the appropriate submenu
	{

	case 1:
	  //  1-Read all voltages
	  print_all_voltages();
	  break;
	case 2:
	  // 2-Read all currents
	  print_all_currents();
	  break;
	case 3:
	  // 3-Read all status
	  print_all_status();
	  break;
	case 4:
	  // 4-Sequence off and on
	  sequence_off_on();
	  break;
	case 5:
	  // 5-Margin POLs high
	  margin_high();
	  break;
	case 6:
	  // 6-Margin POLs low
	  margin_low();
	  break;
	case 7:
	  // 7-Margin POLs nominal
	  margin_off();
	  break;
	case 8:
	  // 8-Clear faults
	  smbus->writeByte(ltc4281_i2c_address, 0x04, 0x00); // 4281 FAULT register
	  pmbus->clearAllFaults(ltc3886_i2c_address);
	  pmbus->clearAllFaults(ltc3887_i2c_address);
	  pmbus->clearAllFaults(ltc2975_i2c_address);

	  break;
	case 9:
	  // 9-
	  Serial.println(F("NOOP"));
	  break;
	  
	default:
	  Serial.println(F("Incorrect Option"));
	  break;
	}
      print_prompt(); 
    }
}

//! Prints the title block when program first starts.
//! @return void
void print_title()
{
  Serial.print(F("\n*****************************************************************\n"));
  Serial.print(F("* DC2578 Hello World Demonstration Program                      *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("* This program demonstrates how to send and receive data from   *\n"));
  Serial.print(F("* the DC2578 demo board.                                        *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("* Set the baud rate to 115200 and select the newline terminator.*\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("*****************************************************************\n"));
}

//! Prints main menu.
//! @return void
void print_prompt()
{
  Serial.print(F("\n  1-Read all voltages     \n"));
  Serial.print(F(  "  2-Read all currents     \n"));
  Serial.print(F(  "  3-Read all status       \n"));
  Serial.print(F(  "  4-Sequence off and on   \n"));
  Serial.print(F(  "  5-Margin high           \n"));
  Serial.print(F(  "  6-Margin low            \n"));
  Serial.print(F(  "  7-Margin nominal        \n"));
  Serial.print(F(  "  8-Clear faults          \n"));
  //  Serial.print(F(  "  9-     \n"));
  Serial.print(F("\nEnter a command: "));
}

//! Print all voltages
//! @return void
void print_all_voltages()
{
  float   voltage;
  uint8_t page;
  uint8_t r1, r2;
  uint16_t readval;

  // LTC4281 hot swap voltages
  Serial.println(F("LTC4281 HOT SWAP VOLTAGES: "));
  //  readval = smbus->readWord(ltc4281_i2c_address, 0x3A); // 4281 VSOURCE register
  r1 = smbus->readWord(ltc4281_i2c_address, 0x3A); // 4281 VSOURCE register
  r2 = smbus->readWord(ltc4281_i2c_address, 0x3B); // 4281 VSOURCE register
  Serial.print(F("LTC4281 VSOURCE "));
  readval = (uint16_t)r2 | (uint16_t)r1<<8;
  voltage = (float)readval * 254e-6;
  Serial.println(voltage, DEC);
  

  // LTC3886 intermediate bus voltages
  Serial.println(F("LTC3886 INTERMEDIATE BUS VOLTAGES: "));
  for (page = 0; page < 2; page++)
  {
    pmbus->setPage(ltc3886_i2c_address, page);
    voltage = pmbus->readVout(ltc3886_i2c_address, false);
    Serial.print(F("LTC3886 VOUT "));
    Serial.println(voltage, DEC);
  }

  // LTC3887 POL voltages
  Serial.println(F("LTC3887 POINT OF LOAD VOLTAGES: "));
  for (page = 0; page < 2; page++)
  {
    pmbus->setPage(ltc3887_i2c_address, page);
    voltage = pmbus->readVout(ltc3887_i2c_address, false);
    Serial.print(F("LTC3887 CH"));
    Serial.print(page, DEC);
    Serial.print(F(": "));
    Serial.println(voltage, DEC);
  }

  // LTC2975 POL voltages
  Serial.println(F("LTC2975 POINT OF LOAD VOLTAGES: "));
  for (page = 0; page < 4; page++)
  {
    pmbus->setPage(ltc2975_i2c_address, page);
    voltage = pmbus->readVout(ltc2975_i2c_address, false);
    Serial.print(F("LTC2975 CH"));
    Serial.print((page+2), DEC);
    Serial.print(F(": "));
    Serial.println(voltage, DEC);
  }

}

//! Print all currents
//! @return void
void print_all_currents()
{
  float   current;
  uint8_t page;
  uint8_t r1, r2;
  uint16_t readval;

  // LTC4281 hot swap current
  Serial.println(F("LTC4281 HOT SWAP CURRENT: "));
  //  readval = smbus->readWord(ltc4281_i2c_address, 0x40); // 4281 VSENSE register
  r1 = smbus->readWord(ltc4281_i2c_address, 0x40); // 4281 VSOURCE register
  r2 = smbus->readWord(ltc4281_i2c_address, 0x41); // 4281 VSOURCE register
  Serial.print(F("LTC4281 ISNS "));
  //  current = (float)readval * (127.45e-9 / 0.02);
  readval = (uint16_t)r2 | (uint16_t)r1<<8;
  current = (float)readval * (610e-9 / 0.02);
  Serial.println(current, DEC);
  

  // LTC3886 intermediate bus currents
  Serial.println(F("LTC3886 INTERMEDIATE BUS CURRENTS: "));
  for (page = 0; page < 2; page++)
  {
    pmbus->setPage(ltc3886_i2c_address, page);
    current = pmbus->readIout(ltc3886_i2c_address, false);
    Serial.print(F("LTC3886 IOUT "));
    Serial.println(current, DEC);
  }

  // LTC3887 POL currents
  Serial.println(F("LTC3887 POINT OF LOAD CURRENTS: "));
  for (page = 0; page < 2; page++)
  {
    pmbus->setPage(ltc3887_i2c_address, page);
    current = pmbus->readIout(ltc3887_i2c_address, false);
    Serial.print(F("LTC3887 CH"));
    Serial.print(page, DEC);
    Serial.print(F(": "));
    Serial.println(current, DEC);
  }

  // LTC2975 POL currents
  Serial.println(F("LTC2975 POINT OF LOAD CURRENTS: "));
  for (page = 0; page < 4; page++)
  {
    pmbus->setPage(ltc2975_i2c_address, page);
    current = pmbus->readIout(ltc2975_i2c_address, false);
    Serial.print(F("LTC2975 CH"));
    Serial.print((page+2), DEC);
    Serial.print(F(": "));
    Serial.println(current, DEC);
  }


}

//! Print all status bytes and words
//! @return void
void print_all_status()
{
  uint8_t b;
  uint16_t w;
  uint8_t page;

  // LTC4281 hot swap status
  Serial.println(F("LTC4281 HOT SWAP STATUS: "));
  b = smbus->readByte(ltc4281_i2c_address, 0x1E); // 4281 STATUS register
  Serial.print(F("LTC4281 STATUS BYTE LOWER 0x"));
  Serial.println(b, HEX);
  b = smbus->readByte(ltc4281_i2c_address, 0x1F); // 4281 STATUS register
  Serial.print(F("LTC4281 STATUS BYTE UPPER 0x"));
  Serial.println(b, HEX);

  // LTC3886 intermediate bus status
  Serial.println(F("LTC3886 INTERMEDIATE BUS STATUS: "));
  for (page = 0; page < 2; page++)
  {
    Serial.print(F("PAGE "));
    Serial.println(page, DEC);
    pmbus->setPage(ltc3886_i2c_address, page);
    b = pmbus->readStatusByte(ltc3886_i2c_address);
    Serial.print(F("LTC3886 STATUS BYTE 0x"));
    Serial.println(b, HEX);
    w = pmbus->readStatusWord(ltc3886_i2c_address);
    Serial.print(F("LTC3886 STATUS WORD 0x"));
    Serial.println(w, HEX);
  }

  // LTC3887 POL currents
  Serial.println(F("LTC3887 POINT OF LOAD STATUS: "));
  for (page = 0; page < 2; page++)
  {
    Serial.print(F("PAGE "));
    Serial.println(page, DEC);
    pmbus->setPage(ltc3887_i2c_address, page);
    b = pmbus->readStatusByte(ltc3887_i2c_address);
    Serial.print(F("LTC3887 STATUS BYTE 0x"));
    Serial.println(b, HEX);
    w = pmbus->readStatusWord(ltc3887_i2c_address);
    Serial.print(F("LTC3887 STATUS WORD 0x"));
    Serial.println(w, HEX);
  }


  // LTC2975 POL status
  Serial.println(F("LTC2975 POINT OF LOAD STATUS: "));
  for (page = 0; page < 4; page++)
  {
    Serial.print(F("PAGE "));
    Serial.println(page, DEC);
    pmbus->setPage(ltc2975_i2c_address, page);
    b = pmbus->readStatusByte(ltc2975_i2c_address);
    Serial.print(F("LTC2975 STATUS BYTE 0x"));
    Serial.println(b, HEX);
    w = pmbus->readStatusWord(ltc2975_i2c_address);
    Serial.print(F("LTC2975 STATUS WORD 0x"));
    Serial.println(w, HEX);
  }
}

//! Sequence off then on
//! @return void
void sequence_off_on()
{
  // sequence down manually to preserve order
  //LTC3887 & LTC2975
  pmbus->clearAllFaults(ltc3887_i2c_address);
  pmbus->clearAllFaults(ltc2975_i2c_address);
  smbus->writeByte(ltc3887_i2c_address, 0x00, 0xFF); // page register
  smbus->writeByte(ltc2975_i2c_address, 0x00, 0xFF); // page register
  pmbus->sequenceOff(&ltc3887_i2c_address, 1); // turn off the part
  pmbus->sequenceOff(&ltc2975_i2c_address, 1); // turn off the part
  delay(600);
  //LTC3886
  pmbus->clearAllFaults(ltc3886_i2c_address);
  smbus->writeByte(ltc3886_i2c_address, 0x00, 0xFF); // page register
  pmbus->sequenceOff(&ltc3886_i2c_address, 1); // turn off the page
  delay(800);
  // now all of the supplies are off, turn off the hot-swap
  smbus->writeWord(ltc4281_i2c_address, 0x00, 0x6002); // 4281 CONTROL register -> OFF
  delay (2000);
  smbus->writeByte(ltc4281_i2c_address, 0x1D, 0x80); // 4281 reset
  // all supplies come up automagically when the hot-swap comes up
}

//! Margin high
//! @return void
void margin_high()
{
  pmbus->marginHighGlobal();
}

//! Margin low
//! @return void
void margin_low()
{
  pmbus->marginLowGlobal();
}

//! Go to nominal
//! @return void
void margin_off()
{
  pmbus->sequenceOnGlobal();
}
