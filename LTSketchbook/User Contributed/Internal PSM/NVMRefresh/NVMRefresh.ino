/*
Linear Technology DC1962A/C Demonstration Board
LTC3880, LTC2974, LTC2977/8: Power Management Solution for Application Processors
Example application of NVM Refreshing

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
$Revision:  $
$Date:  $

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
#include <Linduino.h>
#include <UserInterface.h>
#include <avr/boot.h>

#include <LT_SMBus.h>
#include <LT_SMBusPec.h>
#include <LT_SMBusNoPec.h>
#include <LT_PMBus.h>
#include <LT_NVMRefresh.h>

#define LTC3880_I2C_ADDRESS 0x30
#define LTC2974_I2C_ADDRESS 0x32
// (Covers DC1962A/C. A has LTC2978, C has LTC2977)
#define LTC2977_8_I2C_ADDRESS 0x33


// Used for reading and writing NVM.
// Read will allocate or reallocate data.
// Write will free it and set the pointer to NULL.
static uint16_t *ltc3880data;
static uint16_t ltc3880dataCount;
static uint16_t *ltc2974data;
static uint16_t ltc2974dataCount;
static uint16_t *ltc2977data;
static uint16_t ltc2977dataCount;

void setup()
{
  Serial.begin(115200);         //! Initialize the serial port to the PC

  // There is not enough memory in an Uno, so make noise if necessary.
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

  print_title();
  print_prompt();
}

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
        Serial.println(F("Check LTC3880 NVM"));
        check_health(LTC3880_I2C_ADDRESS);
        Serial.println(F("Check LTC2974 NVM"));
        check_health(LTC2974_I2C_ADDRESS);
        Serial.println(F("Check LTC2977/8 NVM"));
        check_health(LTC2977_8_I2C_ADDRESS);
        break;
      case 2:
        Serial.println(F("Store All Fault Logs"));
        store_fault_logs();
        wait_for_ready(LTC3880_I2C_ADDRESS);
        wait_for_ready(LTC2974_I2C_ADDRESS);
        wait_for_ready(LTC2977_8_I2C_ADDRESS);
        break;
      case 3:
        Serial.println(F("Bump LTC3880 Counter"));
        increment_counter(LTC3880_I2C_ADDRESS);
        Serial.println(F("Bump LTC2974 Counter"));
        increment_counter(LTC2974_I2C_ADDRESS);
        Serial.println(F("Bump LTC2977/8 Counter"));
        increment_counter(LTC2977_8_I2C_ADDRESS);
        break;
      case 4: // (technique does not work with LTC2978/A)
        read_bytes(LTC3880_I2C_ADDRESS, &ltc3880data, &ltc3880dataCount, true);
        Serial.println(F("Read LTC3880 Data"));
        read_bytes(LTC2974_I2C_ADDRESS, &ltc2974data, &ltc2974dataCount, true);
        Serial.println(F("Read LTC2974 Data"));
        if (is_ltc2978(LTC2977_8_I2C_ADDRESS))
          Serial.println("LTC2978/A data not read (LTC2978/A does not support direct read of NVM)");
        else
        {
          read_bytes(LTC2977_8_I2C_ADDRESS, &ltc2977data, &ltc2977dataCount, true);
          Serial.println(F("Read LTC2977 Data"));
        }
        break;
      case 5: // (technique does not work with LTC2978/A)
        write_bytes(LTC3880_I2C_ADDRESS, &ltc3880data, &ltc3880dataCount, true);
        Serial.println(F("LTC3880 Data written to NVM"));
        write_bytes(LTC2974_I2C_ADDRESS, &ltc2974data, &ltc2974dataCount, true);
        Serial.println(F("LTC2974 Data written to NVM"));
        if (is_ltc2978(LTC2977_8_I2C_ADDRESS))
          Serial.println("LTC2978/A data not written (LTC2978/A does not support direct read of NVM)");
        else
        {
          write_bytes(LTC2977_8_I2C_ADDRESS, &ltc2977data, &ltc2977dataCount, true);
          Serial.println(F("LTC2977 Data written to NVM"));
        }
        // Uncomment to free memory
        //free(ltc3880data);
        //ltc3880data = NULL;
        //free(ltc2974data);
        //ltc2974data = NULL;
        //free(ltc2977data);
        //ltc2977data = NULL;
        break;
      case 6: // (technique works on all PSM devices)
        // This clears the status register so that tools observing on the
        // bus can observe restore behavior. It is not neccessary
        // for correct operation.
        smbus->writeByte(LTC3880_I2C_ADDRESS, MFR_EEPROM_STATUS, 0x00);

        store();

        wait_for_ready(LTC3880_I2C_ADDRESS);
        wait_for_nvm_done(LTC3880_I2C_ADDRESS);
        wait_for_ready(LTC2974_I2C_ADDRESS);
        wait_for_ready(LTC2977_8_I2C_ADDRESS);
        break;
      case 7:
        Serial.println(F("Clear All Fault Logs"));
        clear_fault_logs();
        wait_for_ready(LTC3880_I2C_ADDRESS);
        wait_for_ready(LTC2974_I2C_ADDRESS);
        wait_for_ready(LTC2977_8_I2C_ADDRESS);
        break;
      case 8:
        clear_faults_global();
        break;
      case 9:
        addresses = smbus->probe(0);
        while (*addresses != 0)
        {
          Serial.print(F("ADDR 0x"));
          Serial.println(*addresses++, HEX);
        }
        break;
      case 10:
        // Group protocol is a way to send multiple devices in one command as an atomic transaction.
        // It begins with a start, follows with address/command transactions, and ends in a stop.
        // At the stop, all devices respond to their command at the same time. It is not legal
        // or defined how things behave if it sends more than one command to a device withing one
        // group protocol transaction..

        // The reason for using this is that the managers do not have a reset command like the controllers,
        // so the code next to mix restore commands to managers with reset commands to controllers.

        // Tell the PMBus object to start saving commands, but don't do anything on PMBus.
        smbus->beginStoring();
        // Add commands to the list
        smbus->sendByte(LTC3880_I2C_ADDRESS, MFR_RESET);
        smbus->sendByte(LTC2974_I2C_ADDRESS, RESTORE_USER_ALL);
        smbus->sendByte(LTC2977_8_I2C_ADDRESS, RESTORE_USER_ALL);
        // Tell the PMBus object to send all the comands atomically as a Group Command Protocol transaction.
        // At the stop the controller will reset, and the managers will do a restore. This effectivly restarts
        // all devices at the same time so that proper sequencing takes place.
        smbus->execute();

        // Wait for each device to finish.
        wait_for_ready(LTC3880_I2C_ADDRESS);
        wait_for_ready(LTC2974_I2C_ADDRESS);
        wait_for_ready(LTC2977_8_I2C_ADDRESS);

        break;
      case 11:
        // Example of something like a real application without retry,
        // but no policy, such as protection of over programming or
        // scheduling randomness.

        // Make sure the world is sane. This does not allow
        // storing with NVM/EEPROM CRC errors, but you can
        // change the code to do that.
        if (check_health(LTC3880_I2C_ADDRESS) && check_health(LTC2974_I2C_ADDRESS) && check_health(LTC2977_8_I2C_ADDRESS))
        {
          // Store fault logs so the devices cannot be busy with a fault log.
          store_fault_logs();
          wait_for_ready(LTC3880_I2C_ADDRESS);
          wait_for_ready(LTC2974_I2C_ADDRESS);
          wait_for_ready(LTC2977_8_I2C_ADDRESS);

          // Bump the counters.
          increment_counter(LTC3880_I2C_ADDRESS);
          increment_counter(LTC2974_I2C_ADDRESS);
          increment_counter(LTC2977_8_I2C_ADDRESS);
          // Store all data in parallel.
          smbus->sendByte(0x5B, STORE_USER_ALL);
          wait_for_ready(LTC3880_I2C_ADDRESS);
          wait_for_nvm_done(LTC3880_I2C_ADDRESS);
          wait_for_ready(LTC2974_I2C_ADDRESS);
          wait_for_ready(LTC2977_8_I2C_ADDRESS);

          // All is ok, so clear the fault logs so they can be used if enabled.
          clear_fault_logs();
          wait_for_ready(LTC3880_I2C_ADDRESS);
          wait_for_ready(LTC2974_I2C_ADDRESS);
          wait_for_ready(LTC2977_8_I2C_ADDRESS);

          // Do not restore or reset here or there will be
          // a power cycle and rails with turn off and on.
          if (!check_health(LTC3880_I2C_ADDRESS))
          {
            Serial.println(F("LTC3880 failed validation"));
            // Implement code to try again if CRC fails, etc.
          }
          if (!check_health(LTC2974_I2C_ADDRESS))
          {
            Serial.println(F("LTC2974 failed validation"));
            // Implement code to try again if CRC fails, etc.
          }
          if (!check_health(LTC2977_8_I2C_ADDRESS))
          {
            Serial.println(F("LTC2977/8 failed validation"));
            // Implement code to try again if CRC fails, etc.
          }

        }
        Serial.println(F("NVM (EEPROM) is Updated"));
        break;
      default:
        Serial.println(F("Incorrect Option"));
        break;
    }
    print_prompt();
  }
}

//! Prints the title block when program first starts.
void print_title()
{
  Serial.print(F("\n********************************************************************\n"));
  Serial.print(F("* DC1962C Store/Restore User All                                   *\n"));
  Serial.print(F("*                                                                  *\n"));
  Serial.print(F("* This program demonstrates how to store and restore EEPROM.       *\n"));
  Serial.print(F("*                                                                  *\n"));
  Serial.print(F("* Set the baud rate to 115200 and select the newline terminator.   *\n"));
  Serial.print(F("*                                                                  *\n"));
  Serial.print(F("********************************************************************\n"));
}

//! Prints main menu.
void print_prompt()
{
  Serial.print(F("\n  1 -Check NVM (EEPROM) is Ok to Program\n"));
  Serial.print(F("  2 -Force Fault Logs\n"));
  Serial.print(F("  3 -Increment Counter\n"));
  Serial.print(F("  4 -Bulk Read NVM (EEPROM)\n"));
  Serial.print(F("  5 -Bulk Write NVM (EEPROM)\n"));
  Serial.print(F("  6 -Store\n"));
  Serial.print(F("  7 -Clear/Erase Fault Logs\n"));
  Serial.print(F("  8 -Clear Faults\n"));
  Serial.print(F("  9 -Bus Probe\n"));
  Serial.print(F(" 10 -Reset (Will Power Cycle)\n"));
  Serial.print(F(" 11 -Example \"In System\" Safe Store\n"));
  Serial.print(F("\nEnter a command:"));
}
