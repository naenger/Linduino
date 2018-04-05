/*!
Linear Technology DC2467 Demonstration Board (Linduino Shield)

LTC2970: Dual I2C Power Supply Monitor and Margining Controller

This sketch demonstrates how to create a look-up table of voltage vs. DAC code
and use it to quickly step to a voltage, rather than using the servo loop, 
which is very slow by comparison.

Note that Linduino has no way to store this look-up table to a file and retrieve 
it later. The user must re-create the table at power-up/reset.

@verbatim
  Setup:
   Set the terminal baud rate to 115200 and select the newline terminator.

@endverbatim

http://www.linear.com/product/LTC2970

http://www.linear.com/demo/#demoboards

REVISION HISTORY
$Revision: 4037 $
$Date: 2016-04-18 10:20:48 -0600 (Mon, 18 Apr 2016) $

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
#include <math.h>
#include "Linduino.h"
#include "UserInterface.h"
#include "LT_SMBusNoPec.h"
#include "LTC2970.h"

#define LTC2970_I2C_ADDRESS 0x5B //global 7-bit address
//#define LTC2970_I2C_ADDRESS 0x5C //SLAVE_LL 7-bit address
//#define LTC2970_I2C_ADDRESS 0x6F //SLAVE_HH 7-bit address

#define DELAY_VAL 100 // wait between DAC code steps before reading voltage


/****************************************************************************/
// Global variables
static uint8_t ltc2970_i2c_address;

static LT_SMBusNoPec *smbus = new LT_SMBusNoPec();

// variables used for keeping track of the LTC2970 state
uint16_t servo0_value_low;
uint16_t servo0_value_hi;
uint16_t servo0_value_nom;
uint16_t servo1_value_low;
uint16_t servo1_value_hi;
uint16_t servo1_value_nom;

// NOTE: because Linduino is so limited, we choose the most memory-efficient storage for the LUT.
// We could choose a more computationally efficient technique on a larger system.

static uint8_t dac_code[256]; // array in which to store DAC codes
static uint16_t adc_code[256]; // corresponding array for ADC values (smaller than float)
static int dac_sign; // is the DAC code-to-voltage curve sloping up or down?

/****************************************************************************/
//! Initialize Linduino
//! @return void
void setup()
{

  // NOTE: we do NOT initialize the LTC2970 here (though we could with the ltc2970_configure() function)
  //  Assume that there might be another bus master talking to it on the I2C bus.

  // initialize the i2c port
  Serial.begin(115200);         //! Initialize the serial port to the PC
  print_title();
  print_prompt();

  ltc2970_i2c_address = LTC2970_I2C_ADDRESS;

  servo0_value_nom = 0x2710; // 5.0V
  servo1_value_nom = 0x0F9C; // 2.0V -> -5.0V
  servo0_value_low = 0x2328; // 10% low
  servo1_value_low = 0x0ED4; // 10% low
  servo0_value_hi = 0x2AF8; // 10% high
  servo1_value_hi = 0x1063; // 10% high

}

//! Main Linduino loop
//! @return void
void loop()
{
  uint8_t user_command;
  uint16_t return_val;
  float input_v;
  
  if (Serial.available())                //! Checks for user input
    {
      user_command = read_int();         //! Reads the user command

      switch (user_command)              //! Prints the appropriate submenu
	{

	case 1 :
	  Serial.print(F("\n****INITIALIZING THE LTC2970****\n"));
	  ltc2970_configure();
	  
	  Serial.print(F("\n****ENABLE LTC2970 CHANNEL 0 AND CHANNEL 1****\n"));
	  ltc2970_dac_disconnect(smbus, ltc2970_i2c_address, 0);
	  ltc2970_gpio_up(smbus, ltc2970_i2c_address, 0);
	  
	  ltc2970_dac_disconnect(smbus, ltc2970_i2c_address, 1);
	  ltc2970_gpio_up(smbus, ltc2970_i2c_address, 1);

	  delay(200);
	  Serial.print(F("\n****SOFT CONNECT LTC2970 DAC0 and DAC1****\n"));
	  ltc2970_soft_connect_dac(smbus, ltc2970_i2c_address, 0);
	  ltc2970_soft_connect_dac(smbus, ltc2970_i2c_address, 1);
	  break;
	  
	case 2 :
	  Serial.print(F("\n****CONSTRUCT LUT BY STEPPING THROUGH ALL DAC CODES****\n"));
	  ltc2970_build_lut(smbus, ltc2970_i2c_address, 0);
	  break;
	  
	case 3 :
	  Serial.print(F("\n****STEP TO A GIVEN VOLTAGE USING LUT****\n"));
	  Serial.print(F("\nENTER DESIRED VOLTAGE: "));
	  input_v = read_float();
	  Serial.println(input_v);

	  // write the DAC code to LTC2970
	  // NOTE: taking large steps may cause bad things to happen to the analog loop
	  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_IDAC, (0x0300 | (ltc2970_search_lut_1(input_v))));
	  break;

	case 4 :
	  print_lut();
	  break;
	case 5 :
	  break;
	case 6 :
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
	  // 0.02 ohms, possibly adjusted for inaccuracies
	  //	  Serial.print((((return_val & 0x7FFF)*500e-6)/0.0215), DEC);
	  Serial.print((((return_val & 0x7FFF)*500e-6)/0.02), DEC);
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
void print_prompt()
{
  Serial.print(F("\n"));
  Serial.print(F("  1  - Reset the LTC2970, Enable Regulators, connect DACs\n"));
  Serial.print(F("  2  - Construct CH0 Look-up Table\n"));
  Serial.print(F("  3  - Step to CH0 Voltage Using Look-up Table\n"));
  Serial.print(F("  4  - Print the CH0 Look-up Table\n"));
  Serial.print(F("  5  - NOOP\n"));
  Serial.print(F("  6  - NOOP\n"));
  Serial.print(F("  7  - Print Channel 0 & 1 Voltages and Currents\n"));
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
  smbus->writeWord(ltc2970_i2c_address, LTC2970_FAULT_EN, 0x0DEF);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_IO, 0x00CA);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_ADC_MON, 0x007F);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_VDD_OV, 0x2CEC);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_VDD_UV, 0x2328);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_V12_OV, 0x0FA0); 
  smbus->writeWord(ltc2970_i2c_address, LTC2970_V12_UV, 0x0A6B);
  
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_OV, 0x2AF8);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_UV, 0x2328);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_SERVO, 0x2710); 
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_IDAC, 0x0880); // IDAC set by the servo
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_B_OV, 0x0078);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_B_UV, 0x7FD8);

  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_OV, 0x1068);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_UV, 0x0BB8);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_SERVO, 0x0FA0); // 0x0F88 is servo to -5V
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, 0x0480); // IDAC set by the servo
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_B_OV, 0x0960);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_B_UV, 0x7F38);
}


//! Step through DAC codes and measure the resulting voltage at each
//! Build a look-up table of code/voltage pairs
//! Assume that the LTC2970 is configured properly before starting
void ltc2970_build_lut(LT_SMBusNoPec *smbus, uint8_t ltc2970_i2c_address, int channel)
{
  int i;
  uint16_t return_val, store_val;
  
  // loop through DAC codes 0 to 255 (0 and 255 cause DAC faults)
  // there is one slot in the array for each code
  // no guarantee of unique voltage at each code
  if (channel == 0)
    {
      store_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_IDAC); // preserve the IDAC reg value
      smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_IDAC, 0x0301); // set IDAC
    }
  else if (channel == 1)
    {
      store_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC); // preserve the IDAC reg value
      smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, 0x0301); // set IDAC
    }
  else
    Serial.println(F("Invalid channel number."));

  delay(2000); // wait for settling (big steps take a long time)

  for(i = 0; i <= 255; i++) {
    if (channel == 0)
      {
	smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_IDAC, (0x0300 + (uint8_t)i)); // set IDAC
	delay(DELAY_VAL); // wait for settling
	return_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_ADC);
	// could take multiple samples and average here, but averaging a fixed-point number may not help
      }
    else if (channel == 1)
      {
	smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, (0x0300 + (uint8_t)i)); // set IDAC
	delay(DELAY_VAL); // wait for settling
	return_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_A_ADC);
	// could take multiple samples and average here, but averaging a fixed-point number may not help
      }
    else
      {
	Serial.println(F("Invalid channel number."));
      }

    // Print the table as we build it
    Serial.print(i, DEC);
    Serial.print(F("\t"));
    Serial.print(((return_val & 0x7FFF)*500e-6), DEC);
    Serial.print(F("\t(0x"));
    Serial.print(return_val, HEX);
    Serial.println(F(")"));


    // make sure that the voltages returned are reasonable
    // the value is 2s complement (signed)
    // bit 15 is the 'new' bit; ignore it because of the delay above
    if ((return_val & 0x7FFF) < 0x3FFF)
      {
	// return_val is greater than or equal to 0V
	dac_code[i] = (uint8_t)i;
	adc_code[i] = 0x0000;
	adc_code[i] = return_val;
      }
    else
      {
	// return_val is less than 0V
	// for the purposes of building the LUT it doesn't matter
	// as long as we understand 2s complement in the voltage search algorithm
	dac_code[i] = (uint8_t)i;
	adc_code[i] = 0x0000;
	adc_code[i] = return_val;
      }
  } // for ...

  if (adc_code[254] > adc_code[1])
    {
      dac_sign = 1; // positive slope
      Serial.println(F("POSITIVE SLOPE"));
    }
  else if (adc_code[254] < adc_code[1])
    {
      Serial.println(F("NEGATIVE SLOPE"));
      dac_sign = 0; // negative slope
    }
  // return when all DAC codes have been measured

  if (channel == 0)
    smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_IDAC, store_val); // restore IDAC
  else if (channel == 0)
    smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, store_val); // restore IDAC
  else
    Serial.println(F("Invalid channel number."));


}

void print_lut()
{
  int i;

  Serial.println(F("\nLOOK-UP TABLE:\nDAC\tADC VOLTS\t(ADC CODE)"));
  for (i = 0; i < 255; i++)
    {
      Serial.print(dac_code[i]);
      Serial.print(F("\t"));
      Serial.print(((adc_code[i] & 0x7FFF)*500e-6), DEC);
      Serial.print(F("\t(0x"));
      Serial.print(adc_code[i]);
      Serial.println(F(")"));
    }
}

//! Search through the LUT for the DAC code closest to the desired voltage
//! Return the DAC code
//! Only manipulates the LUT; does not talk to the LTC2970
//! Assumes that the code-to-voltage curve is monotonic (may have flat spots, but no slope sign changes)
//!  A more sophisticated search algorithm would be needed to find voltages on a non-monotonic curve
uint8_t ltc2970_search_lut_1(float voltage)
{
  int i = 0;

  float adc1, adc2,
    difference1, difference2,
    difference3;

  do {
    adc1 = convert_adc(adc_code[i]);
    Serial.println(adc1, DEC);
    adc2 = convert_adc(adc_code[i+1]);
    Serial.println(adc2, DEC);
    difference1 = voltage - adc1;
    Serial.println(difference1, DEC);
    difference2 = voltage - adc2;
    Serial.println(difference2, DEC);
    difference3 = fabs(difference2) - fabs(difference1); // want the difference to be shrinking as we search
  } while ((i++ < 255) && (difference3 <= 1e-3) ); // give it a little noise immunity

  if (i < 1)
    return dac_code[i]; // return the dac code corresponding the the smallest voltage error
  else
    return dac_code[i-1]; // return the dac code corresponding the the smallest voltage error
    
}

//! convert uint16_t adc_code into a float voltage
//! bit 15 is a flag, not part of the number
//! bit 14 is the sign bit
//! ADC LSB size is 500uV
float convert_adc(uint16_t adc_code)
{
  float ret_val;
  
  if ((adc_code & 0x7FFF) <= 0x3FFF) // voltage >= 0
    {
      ret_val = 500e-6 * (float)(adc_code & 0x7FFF);
    }
  else // voltage < 0
    {
      ret_val = -1.0 * 500e-6 * (float)((~adc_code | 0x7FFF) + 0x0001);
    }
  return ret_val;
}

