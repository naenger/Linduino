/*!
Linear Technology DC2578A Demonstration Board
LTC4281, LTC3886, LTC2975, LTC3887: Power Management Solution for Hot-Plug Systems

Toggle the hot-swap on and off, demonstrating sequencing

@verbatim

NOTES
  Setup:
   Set the terminal baud rate to 115200 and select the newline terminator.

@endverbatim

http://www.linear.com/product/LTC4281

http://www.linear.com/product/LTC3880

http://www.linear.com/product/LTC2975

http://www.linear.com/product/LTC3887

REVISION HISTORY
$Revision: 3033 $
$Date: 2016-10-19 14:58:30 -0800 (Tue, 19 Oct 2016) $

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

  //  if (Serial.available())                          //! Checks for user input
    if (1)                          //! Checks for user input
    {
      //      user_command = read_int();                     //! Reads the user command
      //      user_command = random(1,9);
      user_command = 1;
      if (user_command != 'm')
	Serial.println(user_command);
      
      // sequence down manually
      //LTC3887 & LTC2975
      //      pmbus->clearAllFaults(ltc3887_i2c_address);
      //      pmbus->clearAllFaults(ltc2975_i2c_address);
      //      smbus->writeByte(ltc3887_i2c_address, 0x00, 0xFF); // page register
      //      smbus->writeByte(ltc2975_i2c_address, 0x00, 0xFF); // page register
      //      pmbus->sequenceOff(&ltc3887_i2c_address, 1); // turn off the part
      //      pmbus->sequenceOff(&ltc2975_i2c_address, 1); // turn off the part
      delay(10);

      switch (user_command)                          //! Prints the appropriate submenu
	{
	  
	case 1:
	  // sequence down manually
	  //LTC3887 & LTC2975
	  //	  pmbus->clearAllFaults(ltc3887_i2c_address);
	  //	  pmbus->clearAllFaults(ltc2975_i2c_address);
	  //	  smbus->writeByte(ltc3887_i2c_address, 0x00, 0xFF); // page register
	  //	  smbus->writeByte(ltc2975_i2c_address, 0x00, 0xFF); // page register
	  //	  pmbus->sequenceOff(&ltc3887_i2c_address, 1); // turn off the part
	  //	  pmbus->sequenceOff(&ltc2975_i2c_address, 1); // turn off the part

	  //	  delay(10);

	  //LTC3886
	  pmbus->clearAllFaults(ltc3886_i2c_address);
	  //	  smbus->writeByte(ltc3886_i2c_address, 0x00, 0xFF); // page register
	  smbus->writeByte(ltc3886_i2c_address, 0x00, 0x01); // page register
	  pmbus->sequenceOff(&ltc3886_i2c_address, 1); // turn off the page

	  //	  delay(600);

	  //LTC4281
	  //	  smbus->writeByte(ltc4281_i2c_address, 0x00, 0xC0); // 4281 CONTROL register

	  //	  delay(1000);

	  //	  smbus->writeByte(ltc4281_i2c_address, 0x00, 0xE8); // 4281 CONTROL register

	  //	  delay(1000);     

	  // EXTRA COMMANDS TO CYCLE INDIVIDUAL SUPPLIES
	  delay(1000);
	  //	  pmbus->sequenceOn(&ltc2975_i2c_address, 1); // turn on the part
	  //	  pmbus->sequenceOn(&ltc3887_i2c_address, 1); // turn on the part
	  smbus->writeByte(ltc3886_i2c_address, 0x00, 0x01); // page register
	  pmbus->sequenceOn(&ltc3886_i2c_address, 1); // turn off the page
	  delay(1000);
	  break;
	  
	default:
	  Serial.println(F("Incorrect Option"));
	  break;
	}
      //      print_prompt(); 
    }
}


//! Prints the title block when program first starts.
//! @return void
void print_title()
{
  Serial.print(F("\n*****************************************************************\n"));
  Serial.print(F("* DC2578 Hello World Demonstration Program                     *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("* This program demonstrates how to send and receive data from   *\n"));
  Serial.print(F("* the DC2578   demo board.                                      *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("* Set the baud rate to 115200 and select the newline terminator.*\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("*****************************************************************\n"));
}

//! Prints main menu.
//! @return void
void print_prompt()
{
  Serial.print(F("\n  1-sequence up\n"));
  Serial.print(F("  2-sequence down\n"));
  Serial.print(F("\nEnter a command:"));
}
