/*!
Linear Technology DC2467 Demonstration Board (Linduino Shield)

LTC2970: Dual I2C Power Supply Monitor and Margining Controller

@verbatim
  Setup:
   Set the terminal baud rate to 115200 and select the newline terminator.

@endverbatim

http://www.linear.com/product/LTC2970

http://www.linear.com/demo/#demoboards

REVISION HISTORY
$Revision: 4037 $
$Date: 2016-01-27 10:20:48 -0600 (Wed, 27 Jan 2016) $

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
// NOTE THAT THE LTC2970.h FILE CONTAINS REGISTER DEFINITIONS FOR THE LTC2970


#define LTC2970_I2C_ADDRESS 0x5B //global 7-bit address
//#define LTC2970_I2C_ADDRESS 0x5C //SLAVE_LL 7-bit address
//#define LTC2970_I2C_ADDRESS 0x6F //SLAVE_HH 7-bit address


/****************************************************************************/
// Global variables
static uint8_t ltc2970_i2c_address;

static LT_SMBusNoPec *smbus = new LT_SMBusNoPec();

// DEFINE GLOBAL VARIABLES HERE

// NOTE THAT "some_var" IS A VARIABLE NAME USED BY CODE GENERATED FROM LTPOWERPLAY
// YOU CAN USE THE SAME VARIABLE OR DEFINE YOUR OWN
uint16_t some_var;


/****************************************************************************/
//! Initialize Linduino
//! @return void
void setup()
{
  // CODE IN THIS SECTION RUNS ONCE AT THE BEGINNING OF EXECUTION.
  // PLACE INITIALIZATION HERE
  
  // NOTE: we do NOT initialize the LTC2970 here (though we could with the ltc2970_configure() function)
  //  Assume that there might be another bus master talking to it on the I2C bus.

  // Initialize the i2c port
  // Begin communication with the PC
  Serial.begin(115200);
  print_title();
  print_prompt();

  ltc2970_i2c_address = LTC2970_I2C_ADDRESS;

  // THIS IS A GOOD PLACE TO INITIALIZE GLOBAL VARIABLES
}

//! Main Linduino loop
//! @return void
void loop()
{
  // CODE IN THIS SECTION RUNS REPEATEDLY, FOREVER
  // USUALLY THIS IS A MENU-DRIVEN LIST OF OPTIONS COMMUNICATED OVER THE SERIAL PORT
  // THE USER CAN PLACE CODE INTO THE "SWITCH" STATEMENT BELOW

  uint8_t user_command;

  // DEFINE LOCAL VARIABLES HERE

  if (Serial.available())              //! Waits for user input from the serial port
  {
    user_command = read_int();         //! Reads the user command
    Serial.println(user_command);
    
    // CUSTOMIZE THIS SWITCH STATEMENT TO SUIT.
    // INSERT CODE OR FUNCTION CALLS TO EXECUTE ANY FUNCTIONALITY
    // REMOVE UNUSED CASES.
    switch (user_command)              //! Executes the appropriate option
    {
    case 1 :
      Serial.print(F("\n****INITIALIZING THE LTC2970****\n"));
      ltc2970_configure();
      break;
      
    case 2 :
      Serial.print(F("\n****ENABLE LTC2970 CHANNEL 0 AND CHANNEL 1****\n"));
      ltc2970_dac_disconnect(smbus, ltc2970_i2c_address, 0);
      ltc2970_gpio_up(smbus, ltc2970_i2c_address, 0);
      
      ltc2970_dac_disconnect(smbus, ltc2970_i2c_address, 1);
      ltc2970_gpio_up(smbus, ltc2970_i2c_address, 1);
      break;
      
    case 3 :
      Serial.print(F("\n****USER DEFINED****\n"));
      // INSERT YOUR CODE HERE
      break;
      
    case 4 :
      Serial.print(F("\n****USER DEFINED****\n"));
      // INSERT YOUR CODE HERE
      break;
      
    case 5 :
      Serial.print(F("\n****USER DEFINED****\n"));
      // INSERT YOUR CODE HERE
      break;
      
    case 6 :
      Serial.print(F("\n****USER DEFINED****\n"));
      // INSERT YOUR CODE HERE
      break;
      
    case 7 :
      Serial.print(F("\n****USER DEFINED****\n"));
      // INSERT YOUR CODE HERE
      break;
      
    case 8 :
      Serial.print(F("\n****USER DEFINED****\n"));
      // INSERT YOUR CODE HERE
      break;
      
    case 9 :
      Serial.print(F("\n****USER DEFINED****\n"));
      // INSERT YOUR CODE HERE
      break;
      
    default:
      Serial.println(F("Incorrect Option"));
      break;
    }
    print_prompt();
  }
}


/************************************************************************/
// Function Definitions

// PLACE USER-DEFINED FUNCTIONS HERE
// THIS IS NORMAL C CODE THAT CAN BE CALLED FROM THE loop() FUNCTION


//! Prints the title block when program first starts.
//! @return void
void print_title()
{
  Serial.print(F("\n***************************************************************\n"));
  Serial.print(F("* DC2467 Control Program                                        *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("* This program provides a simple interface to control the       *\n"));
  Serial.print(F("* the DC2467 regulators through the LTC2970                     *\n"));
  Serial.print(F("* REQUIRES +12V POWER TO THE LINDUINO                           *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("* Set the baud rate to 115200 and select the newline terminator.*\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("*****************************************************************\n"));
}

//! Prints main menu.
//! @return void
// THE USER SHOULD CUSTOMIZE THE TEXT OF THIS MENU TO EXPLAIN THE OPTIONS AVAILABLE
void print_prompt()
{
  Serial.print(F("\n"));
  Serial.print(F("  1  - Reset the LTC2970, Disable Regulators\n"));
  Serial.print(F("  2  - Enable Channel 0 and Channel 1; DACs disconnected\n"));
  Serial.print(F("  3  - User Defined. Insert your own code.\n"));
  Serial.print(F("  4  - User Defined. Insert your own code.\n"));
  Serial.print(F("  5  - User Defined. Insert your own code.\n"));
  Serial.print(F("  6  - User Defined. Insert your own code.\n"));
  Serial.print(F("  7  - User Defined. Insert your own code.\n"));
  Serial.print(F("  8  - User Defined. Insert your own code.\n"));
  Serial.print(F("  9  - User Defined. Insert your own code.\n"));
  Serial.print(F("\nEnter a command number:"));
}


//! Writes configuration values to the LTC2970 registers
//! @return void
void ltc2970_configure()
{
  // INSERT YOUR OWN REGISTER VALUES HERE
  // NOTE THAT LTPOWERPLAY CAN HELP WITH GENERATING THE CORRECT WRITE STATEMENTS
  // SEE THE "REG INFO" TAB IN LTPOWERPLAY
  
  //start the 2970 by configuring all of its registers for this application
  // use SMbus commands
  smbus->writeWord(ltc2970_i2c_address, LTC2970_FAULT_EN, 0x0DEF);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_IO, 0x00CA);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_ADC_MON, 0x007F);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_VDD_OV, 0x2CEC);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_VDD_UV, 0x2328);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_V12_OV, 0x2CEC);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_V12_UV, 0x2328);

  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_OV, 0x2AF8);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_UV, 0x2328);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_SERVO, 0x2710); 
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_IDAC, 0x0880);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_B_OV, 0x0078);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_B_UV, 0x7FD8);

  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_OV, 0x1068);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_UV, 0x0BB8);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_SERVO, 0x0FA0);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, 0x0480);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_B_OV, 0x0960);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_B_UV, 0x7F38);
}
