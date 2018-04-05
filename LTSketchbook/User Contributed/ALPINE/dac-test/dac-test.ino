/*
Linear Technology Audio H-class amp power Demonstration Project
LTC3787, LTC2970: Power Management Solution

@verbatim
http://www.linear.com/demo/DC1262A
http://www.linear.com/demo/DC1411A

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
#include <stdint.h>
#include "Linduino.h"
#include "UserInterface.h"
//#include "LT_I2CBus.h"
#include "LT_SMBusNoPec.h"
//#include "LT_SMBusPec.h"
//#include "LT_PMBUS.h"
//#include "LT_I2C.h"
#include "LTC2970.h"
#include <math.h>

#define LTC2970_I2C_ADDRESS 0x5C //global 7-bit address

/****************************************************************************/
// Global variables
static uint8_t ltc2970_i2c_address;

//static LT_I2CBus *i2cbus = new LT_I2CBus();
//static LT_I2CBus *i2cbus = new LT_I2CBus();
static LT_SMBusNoPec *smbus = new LT_SMBusNoPec(400000);

uint16_t dac_value, dac_ctrl;
uint16_t servo_value;
uint16_t idac_reg;

// definitions of information about the system
static float    dac_step_size = 0.190; // boost volts per DAC step
//static float    dac_v0 = 12.12;  //voltage reference point
//static uint16_t dac_code0 = 0x0049;  // code at reference point
static float    dac_v0 = 12.29;  //voltage reference point
static uint16_t dac_code0 = 0x004D;  // code at reference point
static uint16_t dac_max_code = 0x0040; // maximum allowed DAC code
static uint16_t dac_min_code = 0x0000; // minimum allowed DAC code

static float    adc_step_size = 0.003980; // boost volts per ADC step
static uint16_t adc_max_code = 0x18B7; // maximum allowed ADC reading
static uint16_t adc_min_code = 0x0AE0; // minimum allowed ADC reading

uint16_t dac_mid_code, dac_code_ampl;
uint16_t dac_code = dac_min_code;

int time = 0;

/****************************************************************************/
//! Initialize Linduino
void setup()
{
  uint16_t return_val;

  // initialize the i2c port
  //  i2c_enable();

  Serial.begin(115200);         //! Initialize the serial port to the PC
  //  print_title();
  //  print_prompt();

  ltc2970_i2c_address = LTC2970_I2C_ADDRESS;

  dac_value = 0x0030;
  dac_ctrl = 0x0300;
  idac_reg = dac_ctrl + dac_value;

  servo_value = 0x11CB; // 18v

  ltc2970_configure();
  // make sure servoing is disabled
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_SERVO,  0x0BDF);

  hard_connect_dac();

  dac_mid_code = (uint16_t)((dac_max_code + dac_min_code) / 2);
  dac_code_ampl = (uint16_t)((dac_max_code - dac_min_code - 10) / 2);

  Serial.print(F("\nDAC MAX = "));
  Serial.println(dac_max_code, HEX);
  Serial.print(F("\nDAC MID = "));
  Serial.println(dac_mid_code, HEX);
  Serial.print(F("\nDAC MIN = "));
  Serial.println(dac_min_code, HEX);
}

uint16_t increment = 0x000A;
int dir = 1;
const int period = 28;
float foo = 0.0;
int bar = 0;

void loop() {
  //  uint16_t dac_code;
  
  // sinusoid wave
  foo = sin(2.0*3.1416*(float)time/(float)period);
  dac_code = (uint16_t)((float)foo*dac_code_ampl + (float)dac_mid_code);
  time = (time < period) ? time+1 : 0 ;
  
  //  Serial.print(time, DEC);
  //  Serial.print(F(" : "));
  //  Serial.print(foo, DEC);
  //  Serial.print(F(" : "));
  //  Serial.println(dac_code, HEX);
    
  // sawtooth wave
  //  dac_code = (dac_code < dac_max_code) ? dac_code + 1 : dac_min_code;

  //triangle wave
  //  dac_code += dir*increment;
  //  if (dac_code > dac_max_code) {
  //    dac_code = dac_max_code;
  //    dir = -1;
  //  }
  //  else if (dac_code < dac_min_code) {
  //    dac_code = dac_min_code;
  //    dir = 1;
  //  }

  // update the DAC in the LTC2970
  set_dac_code(dac_code);
  delay(1);
}


/////////////////////////////////////////////////////////////////////////////////

//! Writes configuration values to the LTC2970 registers
void ltc2970_configure()
{
  uint16_t return_val;
  //start the 2970 by configuring all of its registers for this application
  // use SMbus commands
  smbus->writeWord(ltc2970_i2c_address, LTC2970_FAULT_EN, 0x0168);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_IO, 0x002A);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_ADC_MON, 0x007F);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_VDD_OV, 0xFFFF);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_VDD_UV, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_V12_OV, 0xFFFF);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_V12_UV, 0x00000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_OV, 0x2710);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_UV, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_SERVO, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_IDAC, 0x0080);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_B_OV, 0x0654);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_B_UV, 0x0000);

  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_OV, 0x186A);
//  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_UV, 0x09C4);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_UV, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_SERVO, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, 0x0087);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_B_OV, 0x0654);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_B_UV, 0x0000);
}

///!test function to step through DAC codes
void test_dac_steps()
{
  dac_value = dac_min_code;
  set_dac_code(dac_value);
  delay(150);
  for (dac_value =  dac_min_code; dac_value <= dac_max_code; dac_value++)
  {
    set_dac_code(dac_value);
    delay(2);
  }
}

//! Unceremoniously connect the DAC to the boost control node
//!  no attempt to equalize voltages
void hard_connect_dac()
{
  // use the global DAC variables
  //  dac_value = 0x0087;
  dac_ctrl = 0x0300;
  idac_reg = dac_ctrl + dac_value;

  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, idac_reg);
}

//!set DAC code to the defined value (ignore other bits)
void set_dac_code(uint16_t code)
{
  // use the global DAC variables
  dac_value = (0x00FF & code);
  idac_reg = dac_ctrl + dac_value;

  // make sure servoing is disabled
  //  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_SERVO,  0x0BDF);

  // set DAC code
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, idac_reg);

}
