/*!
Linear Technology DC2313 Demonstration Board
LTC2937: Power Management Solution for Application Processors

This sketch has only been tested on a Mega 2560. It is known to fail
on Aarduino Uno and Linduino due to size of RAM.

@verbatim

NOTES
  Setup:
   Set the terminal baud rate to 115200 and select the newline terminator.

@endverbatim

http://www.linear.com/product/LTC3880

http://www.linear.com/product/LTC2974

http://www.linear.com/product/LTC2977

http://www.linear.com/demo/DC1962C

REVISION HISTORY
$Revision: 4080 $
$Date: 2015-11-30 12:35:30 -0600 (Mon, 30 Nov 2015) $

Copyright (c) 2014, Linear Technology Corp.(LTC)
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
    @ingroup DC1962
*/

#include <Arduino.h>
#include <Linduino.h>
#include <UserInterface.h>
#include <LT_PMBus.h>
#include <nvm.h>
#include <avr/boot.h>
#include "LTC2937.h"
#include "data.h"

#define SIGRD 5
#define LTC2937_I2C_ADDRESS 0x37

// Global variables
static uint8_t ltc2937_i2c_address;
// function return values for LTC2937
enum return_values {SUCCEED=0, // the function succeeded without errors
                    NOT_DOWN, // the LTC2937 is not sequenced-down
                    WRITE_PROTECTED, // the LTC2937 is write-protected
                    FAIL  // general failure to do the right thing
                   };
// delay times in milliseconds
#define LTC2937_RESTORE_DELAY   10
#define LTC2937_STORE_DELAY   120

static LT_SMBusNoPec *smbusNoPec = new LT_SMBusNoPec();
static LT_SMBusPec *smbusPec = new LT_SMBusPec();
static NVM *nvm = new NVM(smbusNoPec, smbusPec);

static LT_SMBus *smbus = smbusNoPec;
static LT_PMBus *pmbus = new LT_PMBus(smbus);

static bool pec = false;

static uint8_t dc1613_addresses[] = { LTC2937_I2C_ADDRESS };

//! Wait for nvm operation to complete
//! @return void
void wait_for_nvm()
{
  delay(2000); // Allow time for action to start.

  //  pmbus->waitForNotBusy(ltc2937_i2c_address);
}

//! Program the nvm
//! @return void
void program_nvm ()
{
  bool worked;

  Serial.println(F("Please wait for programming EEPROM..."));
  worked = nvm->programWithData(isp_data);
  //  ltc2937_store();  
  wait_for_nvm();
  // Programming creates a fault when a Poll on Ack generates a Busy Fault just after
  // a Clear Fault Log.
  //  pmbus->clearFaults(ltc2937_i2c_address);
  Serial.println(F("Programming Complete! "));
}

//! Verify the nvm
//! @return void
void verify_nvm ()
{
  bool worked;

  Serial.println(F("Please wait for verification of EEPROM..."));
  //  ltc2937_restore();
  wait_for_nvm();
  worked = nvm->verifyWithData(isp_data);
  

  if (worked == 0)
    Serial.println(F("Verification complete: Invalid EEPROM data"));
  else
  {
    Serial.println(F("Verification complete: Valid EEPROM data"));
  }

  // Ensure there is no write protect so that the clear faults option works.
  //  pmbus->disableWriteProtectGlobal();
}

//! Restore nvm to ram
//! @return void
void restore_nvm ()
{
  //  smbus->writeByte(LTC3880_I2C_ADDRESS, MFR_EEPROM_STATUS, 0x00);

  Serial.println(F("Restore User All"));
  //  pmbus->restoreFromNvmGlobal();
  //  wait_for_nvm();
  Serial.println(F("Restore Complete (EEPROM written to RAM)"));
}

//! Reset all devices
//! @return void
void reset_all_devices ()
{
  Serial.println(F("Reseting all devices"));
  //  pmbus->startGroupProtocol();
  //  pmbus->reset(ltc3880_i2c_address);
  //  pmbus->restoreFromNvm(ltc2974_i2c_address);
  //  pmbus->restoreFromNvm(ltc2977_i2c_address);
  //  pmbus->executeGroupProtocol();

  //  smbus->waitForAck(ltc3880_i2c_address, 0x00);
  //  pmbus->waitForNotBusy(ltc3880_i2c_address);

  //  pmbus->waitForNotBusy(ltc2974_i2c_address);

  //  pmbus->waitForNotBusy(ltc2977_i2c_address);
  Serial.println(F("All devices reset (RAM == EEPROM)"));
}

//! Initialize Linduino
//! @return void
void setup()
{
  Serial.begin(115200);         //! Initialize the serial port to the PC
  /*
  byte sig0;
  byte sig2;
  byte sig4;
  sig0 = boot_signature_byte_get (0);
  sig2 = boot_signature_byte_get (2);
  sig4 = boot_signature_byte_get (4);
  if (sig0 != 0x1E || sig2 != 0x98 | sig4 != 0x01)
  {
    Serial.println("Sketch only runs on Mega 2560");
    return;
  }
  */
  print_title();
  ltc2937_i2c_address = LTC2937_I2C_ADDRESS;
  print_prompt();
}

//! Repeats Linduino loop
//! @return void
void loop()
{
  uint8_t user_command;
  uint8_t res;
  uint8_t model[7];
  uint8_t revision[10];
  uint8_t *addresses = NULL;

  if (Serial.available())                          //! Checks for user input
  {
    user_command = read_int();                     //! Reads the user command
    if (user_command != 'm')
      Serial.println(user_command);

    switch (user_command)                          //! Prints the appropriate submenu
    {
      case 1:
        program_nvm();
        break;
      case 2:
        verify_nvm();
        // Lock the NVM in case the verify failed.
	//        pmbus->smbus()->writeByte(0x5b, MFR_EE_UNLOCK, 0x00);
        break;
      case 3:
        restore_nvm();
        break;
      case 4:
        program_nvm();
        reset_all_devices();
        break;
      case 5:
	//        pmbus->clearFaultsGlobal();
// need an equivalent 2937 clear faults procedure
        break;
      case 6:
	//        pmbus->enablePec(ltc3880_i2c_address);
	//        pmbus->enablePec(ltc2974_i2c_address);
	//        pmbus->enablePec(ltc2977_i2c_address);
	//        smbus = smbusPec;
	//        pmbus->smbus(smbus);
	//        pec = true;
        break;
      case 7:
	//        pmbus->disablePec(ltc3880_i2c_address);
	//        pmbus->disablePec(ltc2974_i2c_address);
	//        pmbus->disablePec(ltc2977_i2c_address);
	//        smbus = smbusNoPec;
	//        pmbus->smbus(smbus);
	//        pec = false;
        break;
      case 8:
        addresses = smbus->probe(0);
        while (*addresses != 0)
        {
          Serial.print(F("ADDR 0x"));
          Serial.println(*addresses++, HEX);
        }
        break;
      case 9:
        reset_all_devices();
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
  Serial.print(F("\n********************************************************************\n"));
  Serial.print(F("* DC2313A In Flight Update Demonstration Program (MEGA 2560 Only)  *\n"));
  Serial.print(F("*                                                                  *\n"));
  Serial.print(F("* This program demonstrates how to program EEPROM from hex data.   *\n"));
  Serial.print(F("*                                                                  *\n"));
  Serial.print(F("* Set the baud rate to 115200 and select the newline terminator.   *\n"));
  Serial.print(F("*                                                                  *\n"));
  Serial.print(F("********************************************************************\n"));
}

//! Prints main menu.
//! @return void
void print_prompt()
{
  Serial.print(F("\n  1-Program\n"));
  Serial.print(F("  2-Verify\n"));
  Serial.print(F("  3-Restore\n"));
  Serial.print(F("  4-Program and Apply\n"));
  Serial.print(F("  5-Clear Faults -- not operational\n"));
  Serial.print(F("  6-PEC On -- not operational\n"));
  Serial.print(F("  7-PEC Off -- not operational\n"));
  Serial.print(F("  8-Bus Probe\n"));
  Serial.print(F("  9-Reset\n"));
  Serial.print(F("\nEnter a command:"));
}

///////////////////////////////////////////////////////////////////////////////////////////

//! store the LTC2937 settings into EEPROM
//! @return int for success
int ltc2937_store()
{
  uint16_t return_val;

  if (ltc2937_is_down() == SUCCEED)
  {
    smbus->sendByte(ltc2937_i2c_address, LTC2937_STORE);
    //    smbus->writeWord(ltc2937_i2c_address, LTC2937_STORE, 0xFFFF);
    //    return_val = smbus->readWord(ltc2937_i2c_address, LTC2937_STORE);
    Serial.print (F("STORED LTC2937 CONFIGURATION TO EEPROM.\n"));
    delay(LTC2937_STORE_DELAY);
    return SUCCEED;
  }
  else
  {
    Serial.print(F("\n IT IS A BAD IDEA TO STORE WHILE THE SUPPLIES ARE SEQUENCED-UP!\n"));
    Serial.print(F("   --SEQUENCE-DOWN FIRST. \n"));
    return NOT_DOWN;
  }
}


//! indicate if the LTC2937 is in the sequenced-down state
//! @return int for part staus
int ltc2937_is_down()
{
  uint16_t return_val,
           on_off,
           status;

  // This function indicates when it is safe to issue a CLEAR command

  // must meet the following conditions:
  // ON_OFF_CONTROL[7] == 1'b0 (commanded down)
  // STATUS_INFORMATION[11:10] == 2'b00 (local seq down complete)
  // STATUS_INFORMATION[9:8] == 2'b00 (global seq down complete)
  // STATUS_INFORMATION[2] == 1'b1 (supplies discharged)

  // may also want:
  // BREAK_POINT[10] == 1'b0 (not in BP mode)

  return_val = smbus->readWord(ltc2937_i2c_address, LTC2937_STATUS_INFORMATION);
  status = (return_val & 0x0F04); // mask the bits
  return_val = smbus->readWord(ltc2937_i2c_address, LTC2937_ON_OFF_CONTROL);
  on_off = (return_val & 0x0080); // mask the bits
  if ((status == 0x0004) && (on_off == 0x0000))
  {
    return SUCCEED;
  }
  else
  {
    Serial.print(F("\n LTC2937 IS NOT DOWN \n"));
    return NOT_DOWN;
  }
}

//! restore the LTC2937 settings from EEPROM values
//! @return int for success
int ltc2937_restore()
{
  uint16_t return_val;

  if (ltc2937_is_down() == SUCCEED)
  {
    // send the RESTORE command
    smbus->sendByte(ltc2937_i2c_address, LTC2937_RESTORE);
    //    return_val = smbus->readWord(ltc2937_i2c_address, LTC2937_RESTORE);
    Serial.print (F("\nRESTORED LTC2937 CONFIGURATION FROM EEPROM.\n"));
    delay(LTC2937_RESTORE_DELAY);
    return SUCCEED;
  }
  else
  {
    Serial.print(F("\n IT IS A BAD IDEA TO RESTORE WHILE THE SUPPLIES ARE SEQUENCED-UP!\n"));
    Serial.print(F("   --SEQUENCE-DOWN FIRST. \n"));
    return NOT_DOWN;
  }
}
