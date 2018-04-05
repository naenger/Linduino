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

#define LTC2970_I2C_ADDRESS 0x5B //global 7-bit address
//#define LTC2970_I2C_ADDRESS 0x5C //SLAVE_LL 7-bit address
//#define LTC2970_I2C_ADDRESS 0x6F //SLAVE_HH 7-bit address

#define GATE_REG_TIME   10 // millisecond wait for gate bias regulator start-up
#define DRAIN_UP_TIME   1 // millisecond wait for drain voltage to stabilize
#define DAC_CONN_TIME   1 // millisecond wait for DAC (hard) connect
#define SERVO_UP_TIME   2000 // millisecond wait for servo to reach the commanded current
#define SERVO_DWN_TIME  2000 // millisecond wait for servo to reach the commanded current
#define DAC_DISCONN_TIME 2 // millisecond wait for DAC disconnect
#define DRAIN_DOWN_TIME 1 // millisecond wait for drain switches to disconnect
#define GATE_REG_DWN_TIME 1 // millisecond wait for gate bias regulator to disable

/****************************************************************************/
// Global variables
static uint8_t ltc2970_i2c_address;

static LT_SMBusNoPec *smbus = new LT_SMBusNoPec(400000);

// variables used for keeping track of the LTC2970 state
uint16_t servo0_value_low; // servo 0 controls amp drain current
uint16_t servo0_value_hi;
uint16_t servo0_value_nom;
uint16_t servo1_value_low; //  servo 1 controls the +3.3V supply voltage
uint16_t servo1_value_hi;
uint16_t servo1_value_nom;


/****************************************************************************/
//! Initialize Linduino
//! @return void
void setup()
{

  // NOTE: we do NOT initialize the LTC2970 here (though we could with the ltc2970_configure() function)
  //  Assume that there might be another bus master talking to it on the I2C bus.
  //  ltc2970_configure();

  // initialize the i2c port
  Serial.begin(115200);         //! Initialize the serial port to the PC
  print_title();
  print_prompt();

  ltc2970_i2c_address = LTC2970_I2C_ADDRESS;

  servo0_value_nom = 0x00DE; 
  servo1_value_nom = 0x19C8; 
  servo0_value_low = 0x0080; 
  servo1_value_low = 0x1900; 
  servo0_value_hi = 0x0106; 
  servo1_value_hi = 0x1A90; 

}

//! Main Linduino loop
//! @return void
void loop()
{
  uint8_t user_command;
  uint16_t return_val;

  if (Serial.available())                //! Checks for user input
    {
      user_command = read_int();         //! Reads the user command

      switch (user_command)              //! Prints the appropriate submenu
	{

	case 1 :
	  Serial.print(F("\n****INITIALIZING THE LTC2970****\n"));
	  ltc2970_configure();
	  break;
	  
	case 2 :
	  Serial.print(F("\n****SEQUENCE UP THE RF PA VOLTAGES AND CURRENTS****\n"));
	  rfpa_sequence_up();
	  break;
	  
	case 3 :
	  rfpa_sequence_down();
	  break;
	  
	case 4 :
	  Serial.print(F("\n****SERVO AMPLIFIER CURRENT LOW****\n"));
	  ltc2970_servo_to_adc_val(smbus, ltc2970_i2c_address, 0, servo0_value_low);
	  ltc2970_servo_to_adc_val(smbus, ltc2970_i2c_address, 1, servo1_value_low);
	  break;
	  
	case 5 :
	  Serial.print(F("\n****SERVO AMPLIFIER CURRENT TO NOMINAL****\n"));
	  ltc2970_servo_to_adc_val(smbus, ltc2970_i2c_address, 0, servo0_value_nom);
	  ltc2970_servo_to_adc_val(smbus, ltc2970_i2c_address, 1, servo1_value_nom);
	  break;
	  
	case 6 :
	  Serial.print(F("\n****SERVO AMPLIFIER CURRENT HIGH****\n"));
	  ltc2970_servo_to_adc_val(smbus, ltc2970_i2c_address, 0, servo0_value_hi);
	  ltc2970_servo_to_adc_val(smbus, ltc2970_i2c_address, 1, servo1_value_hi);
	  break;
	  
	case 7 :
	  Serial.print(F("\n****ADC CH_0 VOLTAGE  (HEX VALUE)\n"));
	  return_val = smbus->readWord(ltc2970_i2c_address,LTC2970_CH0_A_ADC);
	  Serial.print(((return_val & 0x7FFF)*500e-6), DEC);
	  Serial.print(F("\t(0x"));
	  Serial.print(return_val, HEX);
	  Serial.println(F(")"));
	  
	  Serial.print(F("\n****ADC CH_1 VOLTAGE  (HEX VALUE)\n"));
	  return_val = smbus->readWord(ltc2970_i2c_address,LTC2970_CH1_A_ADC);
	  // interpret the hex code as a negative voltage
	  Serial.print(5.035 - 5.025*((return_val & 0x7FFF)*500e-6), DEC);
	  Serial.print(F("\t(0x"));
	  Serial.print(return_val, HEX);
	  Serial.println(F(")"));
	  
	  Serial.print(F("\n****ADC CH_0 CURRENT  (HEX VALUE)\n"));
	  return_val = smbus->readWord(ltc2970_i2c_address,LTC2970_CH0_B_ADC);
	  // interpret the hex value as a current through the sense resistor
	  // 0.02 ohms adjusted for inaccuracies
	  Serial.print((((return_val & 0x7FFF)*500e-6)/0.0215), DEC);
	  Serial.print(F("\t(0x"));
	  Serial.print(return_val, HEX);
	  Serial.println(F(")"));
	  
	  Serial.print(F("\n****ADC CH_1 CURRENT  (HEX VALUE)\n"));
	  return_val = smbus->readWord(ltc2970_i2c_address,LTC2970_CH1_B_ADC);
	  // filter out negative values to avoid confusion
	  return_val = ((return_val & 0x4000) == 0x4000) ? 0x0000 : return_val;
	  // hex voltage is 1V/A, so no interpretation necessary
	  Serial.print((0.965*(return_val & 0x7FFF)*500e-6), DEC);
	  Serial.print(F("\t(0x"));
	  Serial.print(return_val, HEX);
	  Serial.println(F(")"));
	  break;

	case 8 :
	  Serial.print(F("\n****PRINT FAULTS, CLEAR LATCHED FAULTS \n"));
	  ltc2970_read_faults(smbus, ltc2970_i2c_address);
	  break;
	  
	case 9 :
	  Serial.print(F("\n****PRINT DIE TEMPERATURE \n"));
	  ltc2970_print_die_temp (smbus, ltc2970_i2c_address);
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

//! Prints the title block when program first starts.
//! @return void
void print_title()
{
  Serial.print(F("\n***************************************************************\n"));
  Serial.print(F("* RF ABC Linduino Control Program                               *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("* This program provides a simple interface to control the       *\n"));
  Serial.print(F("* the board power amplifier through the LTC2970                 *\n"));
  Serial.print(F("* REQUIRES +12V POWER TO THE LINDUINO                           *\n"));
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
  Serial.print(F("  1  - Reset the LTC2970, Disable the amplifier\n"));
  Serial.print(F("  2  - Sequence-up the power amplifier to nominal gain\n"));
  Serial.print(F("  3  - Sequence-down the power amplifier\n"));
  Serial.print(F("  4  - Servo amplifier current low\n"));
  Serial.print(F("  5  - Servo amplifier current to nominal\n"));
  Serial.print(F("  6  - Servo amplifier current high\n"));
  Serial.print(F("  7  - Print all voltages and currents\n"));
  Serial.print(F("  8  - Print Fault Register Contents\n"));
  Serial.print(F("  9  - Print LTC2970 Temperature\n"));
  Serial.print(F("\nEnter a command number:"));
}


//! Writes configuration values to the LTC2970 registers
//! @return void
void ltc2970_configure()
{
  //configure all of the LTC2970 registers for this application
  // use SMbus commands
  //  smbus->writeWord(ltc2970_i2c_address, LTC2970_FAULT_EN, 0xFFFF);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_FAULT_EN, 0xFD29);  // mute UV faults
  //  smbus->writeWord(ltc2970_i2c_address, LTC2970_IO, 0x003A); // with IOs active
  smbus->writeWord(ltc2970_i2c_address, LTC2970_IO, 0x000A); // with IOs disabled
  smbus->writeWord(ltc2970_i2c_address, LTC2970_ADC_MON, 0x007F);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_VDD_OV, 0x2EE0);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_VDD_UV, 0x1F40);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_V12_OV, 0x2278); 
  smbus->writeWord(ltc2970_i2c_address, LTC2970_V12_UV, 0x0A6B);
  
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_OV, 0x0147);
  //  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_UV, 0x0080);
    smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_UV, 0x0000);
  //  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_SERVO, 0x8080); // servo enabled 
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_SERVO, 0x0080); // servo disabled 
  //  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_IDAC, 0x033E); // DAC connected
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_IDAC, 0x003E); // DAC disconnected
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_B_OV, 0x36B0);
  //  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_B_UV, 0x0FA0);
    smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_B_UV, 0x0000);

  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_OV, 0x1F40);
  //  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_UV, 0x1770);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_UV, 0x0000);
  //  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_SERVO, 0x99C8); // servo enabled 
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_SERVO, 0x19C8); // servo disabled 
  //  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, 0x0B1D); // DAC connected
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, 0x081D); // DAC disconnected
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_B_OV, 0x0FA0);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_B_UV, 0x0000);
}

//! Executes a controlled up-sequence for the RFPA voltages and currents on the demo board
//! @return void
void rfpa_sequence_up()
{
  uint16_t return_val0, return_val1, return_val2;

  return_val0 = smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_IDAC);
  return_val1 = smbus->readWord(ltc2970_i2c_address, LTC2970_IO);
  return_val2 = smbus->readWord(ltc2970_i2c_address, LTC2970_FAULT_LA_INDEX);
  //  if (((return_val0 & 0x0300) == 0x0000) &&
  //      ((return_val1 & 0x01FF) == 0x000A))
  if(1)
  {
      // the DAC is not connected; GPIOs are down; no latched faults
      Serial.println(F("\nSEQUENCING UP..."));
      // enable gate bias first (regulator enable)
      ltc2970_gpio_up(smbus, ltc2970_i2c_address, 0);
      delay(GATE_REG_TIME);
      // enable drain switch
      ltc2970_gpio_up(smbus, ltc2970_i2c_address, 1);
      delay(DRAIN_UP_TIME);
      // start the drain current servo
      if((return_val2 & 0x0001) == 0x0000)
	{
	  // can't soft-connect because the DAC doesn't have enough range (on this board)
	  //	  ltc2970_soft_connect_dac(smbus,ltc2970_i2c_address, 0);
	  ltc2970_hard_connect_dac(smbus,ltc2970_i2c_address, 0, 0x01);
	  delay(DAC_CONN_TIME);
	  ltc2970_servo_to_adc_val(smbus,ltc2970_i2c_address, 0, servo0_value_nom);
	  delay(SERVO_UP_TIME);
	  // un-mute UV faults
	  smbus->writeWord(ltc2970_i2c_address, LTC2970_FAULT_EN, 0xFFFF);
	}
      else
	{
	  ltc2970_gpio_up(smbus, ltc2970_i2c_address, 1);
	  delay(DRAIN_DOWN_TIME);
	  ltc2970_gpio_up(smbus, ltc2970_i2c_address, 0);
	  delay(GATE_REG_DWN_TIME);
	  Serial.println(F("\nCANNOT SEQUENCE UP. MAKE SURE THERE ARE NO LATCHED FAULTS."));
	}
      Serial.println(F("\nDONE."));
    }
    else 
      {
	// the DAC is enabled and connected, so probably already sequenced-up
	// DO NOTHING
	Serial.println(F("\nCANNOT SEQUENCE UP. SEQUENCE DOWN FIRST. MAKE SURE THERE ARE NO LATCHED FAULTS."));
      }
}

//! Executes a controlled up-sequence for the RFPA voltages and currents on the demo board
//! @return void
void rfpa_sequence_down()
{
  uint16_t return_val0, return_val1;
  
  return_val0 = smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_IDAC);
  return_val1 = smbus->readWord(ltc2970_i2c_address, LTC2970_IO);
  //  if (((return_val0 & 0x0300) == 0x0300) && ((return_val1 & 0x01FF) == 0x003A))
  if(1)
    {
      // the DAC is connected; GPIOs are up
      Serial.println(F("\nSEQUENCING DOWN..."));
      // mute UV faults
      smbus->writeWord(ltc2970_i2c_address, LTC2970_FAULT_EN, 0xFD29);
      // servo drain current to 0
      ltc2970_servo_to_adc_val(smbus,ltc2970_i2c_address, 0, 0x0001);
      delay(SERVO_DWN_TIME);
      ltc2970_dac_disconnect(smbus,ltc2970_i2c_address, 0);
      delay(DAC_DISCONN_TIME);
      // disable drain switch
      ltc2970_gpio_down(smbus, ltc2970_i2c_address, 1);
      delay(DRAIN_DOWN_TIME);
      // disable gate bias regulator
      ltc2970_gpio_down(smbus, ltc2970_i2c_address, 0);
      delay(GATE_REG_DWN_TIME);
      Serial.println(F("\nDONE."));
    }
    else 
      {
	// the DAC is enabled and connected, so probably already sequenced-up
	// DO NOTHING
	Serial.println(F("\nDAC ALREADY CONNECTED. SEQUENCE DOWN FIRST."));
      }
}

