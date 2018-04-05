/*
Linear Technology High Reliability Demonstration Board
LTC3882, LTC2977, LTC2978: Power Management Solution for Application Processors

@verbatim

NOTES
  Setup:
   Set the terminal baud rate to 115200 and select the newline terminator.

@endverbatim

http://www.linear.com/product/LTC3882
http://www.linear.com/product/LTC2977
http://www.linear.com/product/LTC2978

REVISION HISTORY
$Revision: 3033 $
$Date: 2014-12-05 14:58:30 -0800 (Fri, 05 Dec 2014) $

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

#include <Arduino.h>
#include <stdint.h>
#include <Linduino.h>
#include <UserInterface.h>
#include <LT_Wire.h>
#include <LT_twi.h>
#include <LT_PMBus.h>
#include <LT_SMBusPec.h>
#include <LT_PMBusMath.h>
#include <LT_SMBus.h>
#include <LT_I2CBus.h>
#include <LT_SMBusGroup.h>
#include <LT_FaultLog.h>
#include <LT_SMBusNoPec.h>
#include <LT_SMBusBase.h>

#define LTC3882_I2C_ADDRESS 0x4F
#define LTC2977_I2C_ADDRESS 0x5D
#define LTC2978_I2C_ADDRESS 0x5C

// Global variables
static uint8_t ltc3882_i2c_address;
static uint8_t ltc2977_i2c_address;
static uint8_t ltc2978_i2c_address;
static LT_SMBus *smbus = new LT_SMBusPec();
static LT_PMBus *pmbus = new LT_PMBus(smbus);

static uint8_t all_addresses[] = { LTC3882_I2C_ADDRESS, LTC2977_I2C_ADDRESS, LTC2978_I2C_ADDRESS };

//! Initialize Linduino
void setup()
{
  Serial.begin(115200);         //! Initialize the serial port to the PC
  print_title();
  ltc3882_i2c_address = LTC3882_I2C_ADDRESS;
  ltc2977_i2c_address = LTC2977_I2C_ADDRESS;
  ltc2978_i2c_address = LTC2978_I2C_ADDRESS;
  clear_faults();
  print_prompt();
}

//! Repeats Linduino loop
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
        menu_1_basic_commands();                 // Print single-ended voltage menu
        break;
      case 2:
        pmbus->enablePec(ltc3882_i2c_address);
        pmbus->enablePec(ltc2977_i2c_address);
        pmbus->enablePec(ltc2978_i2c_address);
        delete smbus;
        delete pmbus;
        smbus = new LT_SMBusPec();
        pmbus = new LT_PMBus(smbus);
        break;
      case 3:
        pmbus->disablePec(ltc3882_i2c_address);
        pmbus->disablePec(ltc2977_i2c_address);
        pmbus->disablePec(ltc2978_i2c_address);
        delete smbus;
        delete pmbus;
        smbus = new LT_SMBusNoPec();
        pmbus = new LT_PMBus(smbus);
        break;
      case 4:
        addresses = smbus->probe(0);
        while (*addresses != 0)
        {
          Serial.print(F("ADDR 0x"));
          Serial.println(*addresses++, HEX);
        }
        break;
      case 5 :
        pmbus->startGroupProtocol();
        pmbus->reset(ltc3882_i2c_address);
        pmbus->restoreFromNvm(ltc2977_i2c_address);
        pmbus->restoreFromNvm(ltc2978_i2c_address);
        pmbus->executeGroupProtocol();
        break;
      default:
        Serial.println(F("Incorrect Option"));
        break;
    }
    print_prompt();
  }

}

// Function Definitions

//! Prints the title block when program first starts.
void print_title()
{
  Serial.print(F("\n*****************************************************************\n"));
  Serial.print(F("* High Reliability Demonstration Program                        *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("* Set the baud rate to 115200 and select the newline terminator.*\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("*****************************************************************\n"));
}

//! Prints main menu.
void print_prompt()
{
  Serial.print(F("\n  1-Basic Commands\n"));
  Serial.print(F("  2-PEC On\n"));
  Serial.print(F("  3-PEC Off\n"));
  Serial.print(F("  4-Bus Probe\n"));
  Serial.print(F("  5-Reset\n"));
  Serial.print(F("\nEnter a command:"));
}

//! Prints a warning if the demo board is not detected.
void print_warning_prompt()
{
  Serial.println(F("\nWarning: Demo board not detected. Linduino will attempt to proceed."));
}

void print_all_voltages()
{
  float   voltage;
  uint8_t page;

  for (page = 0; page < 2; page++)
  {
    pmbus->setPage(ltc3882_i2c_address, page);
    voltage = pmbus->readVout(ltc3882_i2c_address, false);
    Serial.print(F("LTC3882 VOUT "));
    Serial.println(voltage, DEC);
  }

  for (page = 0; page < 8; page++)
  {
    pmbus->setPage(ltc2977_i2c_address, page);
    voltage = pmbus->readVout(ltc2977_i2c_address, false);
    Serial.print(F("LTC2977 VOUT "));
    Serial.println(voltage, DEC);
  }

  for (page = 0; page < 8; page++)
  {
    pmbus->setPage(ltc2978_i2c_address, page);
    voltage = pmbus->readVout(ltc2978_i2c_address, false);
    Serial.print(F("LTC2978 VOUT "));
    Serial.println(voltage, DEC);
  }

}

void print_all_currents()
{
  float   current;
  uint8_t page;

  for (page = 0; page < 2; page++)
  {
    pmbus->setPage(ltc3882_i2c_address, page);
    current = pmbus->readIout(ltc3882_i2c_address, false);
    Serial.print(F("LTC3880 IOUT "));
    Serial.println(current, DEC);
  }

}

void print_all_status()
{
  uint8_t b;
  uint16_t w;
  uint8_t page;

  for (page = 0; page < 2; page++)
  {
    Serial.print(F("PAGE "));
    Serial.println(page, DEC);
    pmbus->setPage(ltc3882_i2c_address, page);
    b = pmbus->readStatusByte(ltc3882_i2c_address);
    Serial.print(F("LTC3882 STATUS BYTE 0x"));
    Serial.println(b, HEX);
    w = pmbus->readStatusWord(ltc3882_i2c_address);
    Serial.print(F("LTC3882 STATUS WORD 0x"));
    Serial.println(w, HEX);
  }

  for (page = 0; page < 8; page++)
  {
    Serial.print(F("PAGE "));
    Serial.println(page, DEC);
    pmbus->setPage(ltc2977_i2c_address, page);
    b = pmbus->readStatusByte(ltc2977_i2c_address);
    Serial.print(F("LTC2977 STATUS BYTE 0x"));
    Serial.println(b, HEX);
    w = pmbus->readStatusWord(ltc2977_i2c_address);
    Serial.print(F("LTC2977 STATUS WORD 0x"));
    Serial.println(w, HEX);
  }

  for (page = 0; page < 8; page++)
  {
    Serial.print(F("PAGE "));
    Serial.println(page, DEC);
    pmbus->setPage(ltc2978_i2c_address, page);
    b = pmbus->readStatusByte(ltc2978_i2c_address);
    Serial.print(F("LTC2978 STATUS BYTE 0x"));
    Serial.println(b, HEX);
    w = pmbus->readStatusWord(ltc2978_i2c_address);
    Serial.print(F("LTC2978 STATUS WORD 0x"));
    Serial.println(w, HEX);
  }
}

void sequence_off_on()
{
  pmbus->sequenceOffGlobal();
  delay (2000);
  pmbus->sequenceOnGlobal();
}

void margin_high()
{
  pmbus->marginHighGlobal();
}

void margin_low()
{
  pmbus->marginLowGlobal();
}

void margin_off()
{
  pmbus->sequenceOnGlobal();
}

void wait_for_not_busy()
{
  pmbus->waitForNotBusy(ltc3882_i2c_address);
  smbus->waitForAck(ltc2978_i2c_address, 0x00);
  pmbus->waitForNotBusy(LTC2977_I2C_ADDRESS);
}


void wait_for_nvm()
{
  delay(2); // Allow time for action to start.

  // No poll on Ack required for LTC3882/4/6/7
  pmbus->waitForNotBusy(ltc3882_i2c_address);
  // Device not busy, but NVM still copying
  // NVM copy complete after this completes.
  pmbus->waitForNvmDone(ltc3882_i2c_address);

  // This will cause CML, but there is no choice for LTC2978
  // NVM will be done after it ACKs
  // The CML will need clearing
  smbus->waitForAck(ltc2978_i2c_address, 0x00);

  // NVM will be done after it is not busy
  pmbus->waitForNotBusy(ltc2977_i2c_address);

}

void store_nvm_all ()
{
  Serial.println(F("Store User All"));

  // Clear before polling
  smbus->writeByte(ltc3882_i2c_address, MFR_EEPROM_STATUS, 0x00);

  // Store all devices at same time
  pmbus->storeToNvmGlobal();

  // Wait for all to complete
  wait_for_nvm();

  // Clear CML from wait for Ack
  pmbus->clearAllFaults(ltc2978_i2c_address);

  Serial.println(F("Store User All Complete"));
}

void store_nvm_ltc2978 ()
{

  pmbus->storeToNvm(ltc2978_i2c_address);

  // Wait for all to complete
  wait_for_nvm();

  // Clear CML from wait for Ack
  pmbus->clearAllFaults(ltc2978_i2c_address);

  Serial.println(F("Store User All Complete"));
}

void restore_nvm ()
{
  Serial.println(F("Restore User All"));

  // Restore all devices at same time
  pmbus->restoreFromNvmGlobal();

  Serial.println(F("Restore User All Complete"));
}


bool check_nvm_checksum(uint8_t address)
{
  uint16_t w;
  uint8_t bs[32];
  uint16_t nwords;
  uint8_t bpos = 0, wpos;
  bool nok = false;

  // Unlock
  nok = pmbus->unlockNVM(address) || nok;

  if (nok)
  {
    Serial.println(F("NVM Unlock Failed before Read"));
    return nok;
  }

  // Read
  pmbus->waitForNotBusy(address);
  w = smbus->readWord(address, MFR_EE_DATA);
  pmbus->waitForNotBusy(address);
  // Second word is number of 16 bit words to read
  nwords = w = smbus->readWord(address, MFR_EE_DATA);

  for (wpos = 0; wpos < nwords/16; wpos++)
  {
    while (bpos < 32)
    {
      pmbus->waitForNotBusy(address);
      w = smbus->readWord(address, MFR_EE_DATA);
      bs[bpos++] = w & 0xFF;
      bs[bpos++] = (w >> 8) & 0xFF;
    }
    // Compute CRC on 32 byte boundary
    nok = smbus->checkCRC(bs) || nok;
    bpos = 0;
  }

  if (pmbus->lockNVM(address))
  {
    Serial.print(F("Lock failed after comparing checksum on address "));
    Serial.println(address, HEX);
  }

  return nok;
}

bool read_write_nvm(uint8_t address)
{
  uint16_t w;
  uint8_t bs[32];
  uint8_t *ds, *dsp;
  uint16_t nwords;
  uint8_t bpos = 0, wpos;
  uint16_t dpos = 0;
  bool nok = false;

  nok = pmbus->unlockNVM(address) || nok;

  if (nok)
  {
    Serial.println(F("NVM Unlock Failed before Read"));
    return nok;
  }

  // Read
  pmbus->waitForNotBusy(address);
  w = smbus->readWord(address, MFR_EE_DATA);
  pmbus->waitForNotBusy(address);
  // Second word is number of 16 bit words to read
  nwords = w = smbus->readWord(address, MFR_EE_DATA);

  ds = (uint8_t *) malloc(nwords * 2);
  if (ds == NULL)
  {
    Serial.println(F("Failed to malloc when reading nvm"));
    return true;
  }

  for (wpos = 0; wpos < nwords/16; wpos++)
  {
    while (bpos < 32)
    {
      pmbus->waitForNotBusy(address);
      w = smbus->readWord(address, MFR_EE_DATA);
      bs[bpos++] = w & 0xFF;
      bs[bpos++] = (w >> 8) & 0xFF;
    }
    // Compute CRC on 32 byte boundary
    nok = smbus->checkCRC(bs) || nok;
    // Copy the data to the buffer
    memcpy(ds + dpos, bs, 32);
    dpos += 32;
    bpos = 0;
  }

  if (pmbus->lockNVM(address))
  {
    Serial.print(F("Lock failed after reading data on address "));
    Serial.println(address, HEX);
  }

  // If we failed CRC, return and be done.
  if (nok)
  {
    Serial.println(F("CRC Failed reading data"));
    free (ds);
    return nok;
  }

  // Unlock
  nok = pmbus->unlockNVM(address) || nok;

  if (nok)
  {
    Serial.println(F("NVM Unlock Failed before Erase"));
    free(ds);
    return nok;
  }

  // Erase
  pmbus->eraseNVM(address);

  // Unlock
  nok = pmbus->unlockNVM(address) || nok;

  if (nok)
  {
    Serial.println(F("NVM Unlock Failed before Write"));
    free(ds);
    return nok;
  }

  // Write
  dsp = ds;
  while (dsp < ds + dpos) // dpos is the size from the read
  {
    pmbus->waitForNotBusy(address);
    smbus->writeWord(address, MFR_EE_DATA, (dsp[1] << 8) | dsp[0]);
    dsp += 2;
  }

  if (pmbus->lockNVM(address))
  {
    Serial.print(F("Unlock failed after comparing checksum on address "));
    Serial.println(address, HEX);
  }


  free(ds);

  return nok;
}

void check_nvm ()
{
  bool nok;
  uint8_t status;

  Serial.println(F("Check NVM"));

  // If ok, RAM matches NVM and NVM has no CRC mismatches
  nok = pmbus->compareRamWithNvm(ltc3882_i2c_address);
  // If ok, NVM has no CRC mismatches
  nok = check_nvm_checksum(ltc3882_i2c_address) || nok;

  if (nok)
    Serial.println(F("LTC3882 not Ok"));
  else
    Serial.println(F("LTC3882 Ok"));

  // The only possible check without outputs power cycling is to check
  // for CML. Lack of CML proves the command executed without failure.
  status = pmbus->readStatusWord(ltc2978_i2c_address);
  nok = ((status & 0x02) != 0x00);
  if (nok)
    Serial.println(F("LTC2978 not Ok"));
  else
    Serial.println(F("LTC2978 Ok"));

  nok = check_nvm_checksum(ltc2977_i2c_address);
  if (nok)
    Serial.println(F("LTC2977 not Ok"));
  else
    Serial.println(F("LTC2977 Ok"));

}

void clear_faults ()
{
  pmbus->clearAllFaultsGlobal();
  // LTC2978 does not respond to global clear faults
  for (int page = 0; page < 8; page++)
  {
    pmbus->setPage(ltc2978_i2c_address, page);
    pmbus->clearAllFaults(ltc2978_i2c_address);
  }
}

void menu_1_basic_commands()
{
  uint8_t user_command;

  do
  {
    //! Displays the Read/Write menu
    Serial.print(F("  1-Read All Voltages\n"));
    Serial.print(F("  2-Read All Currents\n"));
    Serial.print(F("  3-Read All Status\n"));
    Serial.print(F("  4-Sequence Off/On\n"));
    Serial.print(F("  5-Store to NVM (STORE_USER_ALL)\n"));
    Serial.print(F("  6-Store to NVM (READ/WRTE)\n"));
    Serial.print(F("  7-Check NVM\n"));
    Serial.print(F("  8-Restore from NVM\n"));
    Serial.print(F("  9-Clear Faults\n"));
    Serial.print(F("  m-Main Menu\n"));
    Serial.print(F("\nEnter a command: "));

    user_command = read_int();                              //! Reads the user command
    if (user_command == 'm')                                // Print m if it is entered
    {
      Serial.print(F("m\n"));
    }
    else
      Serial.println(user_command);                         // Print user command

    switch (user_command)
    {
      case 1:
        wait_for_not_busy();
        print_all_voltages();
        break;
      case 2:
        wait_for_not_busy();
        print_all_currents();
        break;
      case 3:
        wait_for_not_busy();
        print_all_status();
        break;
      case 4:
        wait_for_not_busy();
        sequence_off_on();
        break;
      case 5:
        wait_for_not_busy();
        store_nvm_all();
        break;
      case 6:
        wait_for_not_busy();
        read_write_nvm(ltc3882_i2c_address);
        read_write_nvm(ltc2977_i2c_address);
        store_nvm_ltc2978();
        break;
      case 7:
        wait_for_not_busy();
        check_nvm();
        break;
      case 8:
        wait_for_not_busy();
        restore_nvm();
        break;
      case 9:
        wait_for_not_busy();
        clear_faults();
        break;

      default:
        if (user_command != 'm')
          Serial.println(F("Invalid Selection"));
        break;
    }
  }
  while (user_command != 'm');
}


