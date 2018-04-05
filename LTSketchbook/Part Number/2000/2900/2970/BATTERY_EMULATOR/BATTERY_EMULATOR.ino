/*!
Linear Technology DC980A/DC2240A board combo battery emulator.

LTC2970: Dual I2C Power Supply Monitor and Margining Controller
LT8714: Bipolar Output Synchronous Controller with Seamless 
        Four Quadrant Operation

@verbatim
  Setup:
   Set the terminal baud rate to 115200 and select the newline terminator.

@endverbatim

http://www.linear.com/product/LTC2970

http://www.linear.com/demo/#demoboards

REVISION HISTORY
$Revision: 4037 $
$Date: 2016-05-03 10:20:48 -0600 (Tue, 3 May 2016) $

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
!*/

/*! @file
    @ingroup LTC2970
*/

#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "UserInterface.h"
#include "LT_SMBusNoPec.h"
#include "LTC2970.h"

#define LTC2970_I2C_ADDRESS 0x5B //global 7-bit address
//#define LTC2970_I2C_ADDRESS 0x6F //SLAVE_HH 7-bit address


/****************************************************************************/
// Global variables
static uint8_t ltc2970_i2c_address;

static LT_SMBusNoPec *smbus = new LT_SMBusNoPec();

/*
NEED:
     open-circuit voltage curve vs charge capacity
     short-circuit resistance vs. ???
     voltage behavior curve vs load current (resistance) (two time constants)
     charge capacity vs charge/discharge time
     charge capacity vs temperature
*/

/****************************************************************************/
//! Initialize Linduino
//! @return void
void setup()
{
  uint16_t return_val;

  // initialize the i2c port
  Serial.begin(115200);         //! Initialize the serial port to the PC
  print_title();
  print_prompt();

  ltc2970_i2c_address = LTC2970_I2C_ADDRESS;

  servo0_value_nom = 0x2733;
  servo1_value_nom = 0x1A24;
  servo0_value_marg = 0x2347; // 10% low
  servo1_value_marg = 0x1786; // 10% low

  //************************  init_voltage_transition();
}

//! Main Linduino loop
//! @return void
void loop()
{
  uint8_t user_command;
  uint16_t return_val;

  int i = 0;

  if (Serial.available())                //! Checks for user input
  {
    user_command = read_int();         //! Reads the user command
    if (user_command != 'm')
      Serial.println(user_command);

    switch (user_command)              //! Prints the appropriate submenu
    {

      case 1 :
        Serial.print(F("\n****INITIALIZING THE LTC2970****\n"));
        ltc2970_configure();
        break;

      case 2 :
        Serial.print(F("\n****ENTER BATTERY PARAMETERS****\n"));
	

	/*
        ltc2970_dac_disconnect(smbus, ltc2970_i2c_address, 0);
        ltc2970_gpio_up(smbus, ltc2970_i2c_address, 0);

        ltc2970_dac_disconnect(smbus, ltc2970_i2c_address, 1);
        ltc2970_gpio_up(smbus, ltc2970_i2c_address, 1);
	*/
        break;

      case 3 :
        Serial.print(F("\n****ENABLE BATTERY OUTPUT****\n"));

	/*
        ltc2970_soft_connect_dac(smbus, ltc2970_i2c_address, 0);
        ltc2970_soft_connect_dac(smbus, ltc2970_i2c_address, 1);
	*/
        break;
	/*
      case 4 :
        Serial.print(F("\n****SERVO CHANNEL 0 and 1 VOLTAGES 10% LOW****\n"));
        ltc2970_servo_to_adc_val(smbus, ltc2970_i2c_address, 0, servo0_value_marg);
        ltc2970_servo_to_adc_val(smbus, ltc2970_i2c_address, 1, servo1_value_marg);
        break;

      case 5 :
        Serial.print(F("\n****SERVO CHANNEL 0 and 1 VOLTAGES TO NOMINAL****\n"));
        ltc2970_servo_to_adc_val(smbus, ltc2970_i2c_address, 0, servo0_value_nom);
        ltc2970_servo_to_adc_val(smbus, ltc2970_i2c_address, 1, servo1_value_nom);
        break;

      case 6 :
        Serial.print(F("\n****ADC CH_0 VOLTAGE =   (HEX VALUE)\n"));
        return_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_ADC);
        Serial.println(((return_val & 0x7FFF)*500e-6), DEC);
        Serial.println(return_val, HEX);

        Serial.print(F("\n****ADC CH_1 VOLTAGE =   (HEX VALUE)\n"));
        return_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_A_ADC);
        Serial.println(((return_val & 0x7FFF)*500e-6), DEC);
        Serial.println(return_val, HEX);
        break;

      case 7 :
        Serial.print(F("\n****ADC CH_0 CURRENT =   (HEX VALUE)\n"));
        return_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_B_ADC);
        Serial.println((((return_val & 0x7FFF)*500e-6)/0.007), DEC);
        Serial.println(return_val, HEX);

        Serial.print(F("\n****ADC CH_1 CURRENT =   (HEX VALUE)\n"));
        return_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_B_ADC);
        Serial.println((((return_val & 0x7FFF)*500e-6)/0.008), DEC);
        Serial.println(return_val, HEX);
        break;

      case 8 :
        Serial.print(F("\n****PRINT FAULTS, CLEAR LATCHED FAULTS \n"));
        ltc2970_read_faults(smbus, ltc2970_i2c_address);
        break;

      case 9 :
        Serial.print(F("\n****PRINT DIE TEMPERATURE \n"));
        ltc2970_print_die_temp (smbus, ltc2970_i2c_address);
        break;
	*/
      default:
        Serial.println(F("Incorrect Option"));
        break;
    }
    print_prompt();
  }
}


/************************************************************************/
// Function Definitions

//! Prints the title block when program first starts.
//! @return void
void print_title()
{
  Serial.print(F("\n***************************************************************\n"));
  Serial.print(F("* BATTERY EMULATOR PROGRAM                                      *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("* This program provides a simple interface to control the       *\n"));
  Serial.print(F("* the LT8714 regulator through the LTC2970                      *\n"));
  Serial.print(F("* Functions are intended to make the LT8714 behave as a battery *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("* Set the baud rate to 115200 and select the newline terminator.*\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("*****************************************************************\n"));
}

//! Prints main menu.
//! @return void
void print_prompt()
{
  Serial.print(F("\n"));
  Serial.print(F("  1  - Reset the LTC2970, Disable Servo\n"));
  Serial.print(F("  2  - Enter Battery Parameters\n"));
  Serial.print(F("  3  - Enable Battery Output\n"));
  Serial.print(F("  4  - \n"));
  Serial.print(F("  5  - \n"));
  Serial.print(F("  6  - \n"));
  Serial.print(F("  7  - \n"));
  Serial.print(F("  8  - \n"));
  Serial.print(F("  9  - \n"));
  Serial.print(F("\nEnter a command number:"));
}


//! Writes configuration values to the LTC2970 registers
//! @return void
void ltc2970_configure()
{
  uint16_t return_val;
  //start the 2970 by configuring all of its registers for this application
  // use SMbus commands
  smbus->writeWord(ltc2970_i2c_address, LTC2970_FAULT_EN, 0xFFFF);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_IO, 0x003A);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_ADC_MON, 0x007F);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_VDD_OV, 0x2EE0);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_VDD_UV, 0x2328);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_V12_OV, 0x2475);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_V12_UV, 0x1A0B);

  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_OV, 0x2AF8);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_UV, 0x00CA);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_SERVO, 0x09CC);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_IDAC, 0x0489);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_B_OV, 0x0640);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_B_UV, 0x03E8);

  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_OV, 0x2EE0);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_UV, 0x0001);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_SERVO, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, 0x0080);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_B_OV, 0x2EE0);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_B_UV, 0x0000);
}

