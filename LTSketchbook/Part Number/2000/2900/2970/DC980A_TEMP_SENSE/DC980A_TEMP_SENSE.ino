/*
LTC2970: Dual I2C Power Supply Monitor and Margining Controller

Linear Technology DC980A Demonstration Board Temperature Sense with External BJT
Attach an external 2N3906 BJT to IDAC0 and measure voltage across it on VIN0B
This sketch reports temperature at the BJT
Requires that the IDAC is not terminated with a resistor to GND
Requires that the BJT be installed with force and sense lines separate

@verbatim

NOTES
  Setup:
   Set the terminal baud rate to 115200 and select the newline terminator.

@endverbatim

http://www.linear.com/product/LTC2970
http://www.linear.com/demo/DC980A

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

/*! @file
    @ingroup LTC2970
*/

#include <math.h>
#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "UserInterface.h"
#include "LT_SMBusNoPec.h"
#include "LTC2970.h"

#define LTC2970_I2C_ADDRESS 0x5C //global 7-bit address
//#define LTC2970_I2C_ADDRESS 0x6F //SLAVE_HH 7-bit address


/****************************************************************************/
// Global variables
static uint8_t ltc2970_i2c_address;

static LT_SMBusNoPec *smbus = new LT_SMBusNoPec();

uint16_t dac0_value, dac0_ctrl;
uint16_t servo0_value;
uint16_t servo0_value_marg;
uint16_t servo0_value_nom;
uint16_t idac0_reg;
uint16_t dac1_value, dac1_ctrl;
uint16_t servo1_value;
uint16_t servo1_value_marg;
uint16_t servo1_value_nom;
uint16_t idac1_reg;


// definitions of information about the system
static float    dac_step_size = 0.193; // volts per DAC step
static uint16_t dac_max_code = 0x0087; // maximum allowed DAC code
static uint16_t dac_min_code = 0x0049; // minimum allowed DAC code

static float    adc_step_size = 0.003980; // volts per ADC step
static uint16_t adc_max_code = 0x18B7; // maximum allowed ADC reading
static uint16_t adc_min_code = 0x0AE0; // minimum allowed ADC reading

static uint16_t SOFT_CONNECT_DELAY = 1000; // milliseconds to wait for soft connect

// Values for the external temperature sensor routine
static uint16_t dac_current_low = 0x0714;
static uint16_t dac_current_high = 0x07FF;

float bjt_n = 1.016;



/****************************************************************************/
//! Initialize Linduino
void setup()
{
  uint16_t return_val;

  // initialize the i2c port
  //  i2c_enable();

  Serial.begin(115200);         //! Initialize the serial port to the PC

  ltc2970_i2c_address = LTC2970_I2C_ADDRESS;

  dac0_value = 0x0084;
  dac0_ctrl = 0x0000;
  idac0_reg = dac0_ctrl + dac0_value;
  dac1_value = 0x0084;
  dac1_ctrl = 0x0000;
  idac1_reg = dac1_ctrl + dac1_value;

  servo0_value = 0x2733;
  servo1_value = 0x1A24;
  servo0_value_nom = 0x2733;
  servo1_value_nom = 0x1A24;
  servo0_value_marg = 0x2347; // 10% low
  servo1_value_marg = 0x1786; // 10% low

  
  print_title();
  Serial.print(F("\n****INITIALIZING THE LTC2970****\n"));
  ltc2970_configure();

}

void loop()
{
  uint8_t user_command;
  float ext_temp = -273.15;

  if (Serial.available())                //! Checks for user input
    {
      user_command = read_int();         //! Reads the user command
      switch (user_command)              //! Prints the appropriate submenu
	{

	case 1 :
	  Serial.print(F("\n****CALIBRATING THE EXTERNAL DIODE****\n"));
	  Serial.print(F("\n*****ENTER CENTIGRADE TEMPERATURE:"));
	  ext_temp = read_float();
	  ltc2970_print_bjt_n(ext_temp);
	  break;
	  
	case 2 :
	  ltc2970_print_external_temp();
	  ltc2970_print_die_temp ();
	  break;

	default :	  
	  Serial.println(F("Incorrect Option"));
	  break;
	}
      print_prompt();
    }
}



/************************************************************************/
// Function Definitions

//! Prints the title block when program first starts.
void print_title()
{
  Serial.print(F("\n***************************************************************\n"));
  Serial.print(F("* DC980 Temperature Measurement Program                         *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("* This program provides a simple interface to measure           *\n"));
  Serial.print(F("* temperature using a BJT connected to the LTC2970              *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("* Set the baud rate to 115200 and select the newline terminator.*\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("*****************************************************************\n"));
}

//! Prints main menu.
void print_prompt()
{
  Serial.print(F("\n"));
  Serial.print(F("  1  - Calibrate at a known temperature\n"));
  Serial.print(F("  2  - Measure temperature in a calibrated diode\n"));
  Serial.print(F("\nEnter a command number:"));
}




//! Writes configuration values to the LTC2970 registers
void ltc2970_configure()
{
  uint16_t return_val;
  //start the 2970 by configuring all of its registers for this application
  // use SMbus commands
  smbus->writeWord(ltc2970_i2c_address, LTC2970_FAULT_EN, 0x0168);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_IO, 0x000A);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_ADC_MON, 0x007F);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_VDD_OV, 0x2CEC);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_VDD_UV, 0x2328);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_V12_OV, 0x3FFF);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_V12_UV, 0x00000);

  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_OV, 0x2AF8);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_UV, 0x2328);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_SERVO, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, 0x0700);

  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_B_OV, 0x3FFF);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_B_UV, 0x0000);

  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_OV, 0x1C5D);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_UV, 0x1770);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_SERVO, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_IDAC, 0x0084);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_B_OV, 0x3FFF);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_B_UV, 0x0000);
}


//! Prints die temperature on the LTC2970
void ltc2970_print_die_temp ()
{
  static float temp_scale = 4;
  static float temp_offset = 1093; //adc codes

  float temperature;
  uint16_t return_val;
  //print the on-die temperature for the LTC2970
  return_val =
    smbus->readWord(ltc2970_i2c_address, LTC2970_TEMP_ADC);
  return_val = return_val & 0x7FFF; // drop bit 15

  temperature = ((float(return_val) - temp_offset) / temp_scale);

  Serial.print(F("\n LTC_2970 DIE TEMP: "));
  Serial.println(temperature, DEC);
  Serial.println(return_val, HEX);
}

//! return the kelvin temperature of an external BJT hooked-up to IDAC0 and VIN0B
float ltc2970_print_external_temp ()
{
  static int count = 10;  // number of samples per measurement (for averaging)
  int i; // loop counter
  uint16_t reg_vbe_low[count], 
    reg_i_low[count], 
    reg_vbe_high[count], 
    reg_i_high[count];

  float voltage_low[count], 
    voltage_high[count], //voltage measurements at low and high DAC current
    current_low[count], 
    current_high[count], //current measurements at low and high DAC current
    // the current measurement is just the voltage across a 10k resistor
    avg_voltage_low = 0.0,
    avg_voltage_high = 0.0,
    avg_current_low = 0.0,
    avg_current_high = 0.0,
    sdev_voltage_low = 0.0,
    sdev_voltage_high = 0.0,
    sdev_current_low = 0.0,
    sdev_current_high = 0.0,
    avg_current_ratio = 0.0,
    ln_avg_current_ratio = 0.0,
    vbe_delta = 0.0;

  float temperature;
  
  // BJT properties
  //  static float n = 1.016;
  static float q_by_k = 11.60452e3;
  float q_by_n_k = (q_by_k / bjt_n);
  
  //! Initialize the DAC
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, 0x0700);
  delay(500);
  
  //! Measure voltage and current at low current
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, dac_current_low);
  delay(500);
  for (i = 0; i < 10; i++) {
    reg_i_low[i] =
      (0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_B_ADC));
    reg_vbe_low[i] =
      (0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_A_ADC));
    voltage_low[i] = ((float)(reg_vbe_low[i] & 0x7FFF))*500e-6;
    current_low[i] = ((float)(reg_i_low[i] & 0x7FFF))*500e-6;
    avg_voltage_low += voltage_low[i];
    avg_current_low += current_low[i];
    //    Serial.println(voltage_low[i], DEC);
    //    Serial.println(current_low[i], DEC);
  }

  //! Measure voltage and current at high current
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, dac_current_high);
  delay(500);
  for (i = 0; i < 10; i++) {
    reg_i_high[i] =
      (0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_B_ADC));
    reg_vbe_high[i] =
      (0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_A_ADC));
    voltage_high[i] = ((float)(reg_vbe_high[i] & 0x7FFF))*500e-6; 
    current_high[i] = ((float)(reg_i_high[i] & 0x7FFF))*500e-6; 
    avg_voltage_high += voltage_high[i];
    avg_current_high += current_high[i];
    //    Serial.println(voltage_high[i], DEC);
    //    Serial.println(current_high[i], DEC);
  }
  //! Calculate averages, etc.
  avg_voltage_high = avg_voltage_high / count;
  avg_current_high = avg_current_high / count;
  avg_voltage_low = avg_voltage_low / count;
  avg_current_low = avg_current_low / count;
  for (i = 0; i < 10; i++) {
    // accumulate the squared differences
    sdev_voltage_low += pow((voltage_low[i] - avg_voltage_low), 2);
    sdev_current_low += pow((current_low[i] - avg_current_low), 2);
    sdev_voltage_high += pow((voltage_high[i] - avg_voltage_high), 2);
    sdev_current_high += pow((current_high[i] - avg_current_high), 2);
  }
  // divide by number of samples
  sdev_voltage_low = sdev_voltage_low / count;
  sdev_current_low = sdev_current_low / count;
  sdev_voltage_high = sdev_voltage_high / count;
  sdev_current_high = sdev_current_high / count;
  // square root for std dev
  sdev_voltage_low = sqrt(sdev_voltage_low);
  sdev_current_low = sqrt(sdev_current_low);
  sdev_voltage_high = sqrt(sdev_voltage_high);
  sdev_current_high = sqrt(sdev_current_high);
  

  //! Print statistics
  Serial.print(F("AVERAGE LOW VOLTAGE = "));
  Serial.println(avg_voltage_low, DEC);
  Serial.print(F("STD DEV LOW VOLTAGE = "));
  Serial.println(sdev_voltage_low, DEC);

  Serial.print(F("AVERAGE LOW CURRENT = "));
  Serial.println(avg_current_low, DEC);
  Serial.print(F("STD DEV LOW CURRENT = "));
  Serial.println(sdev_current_low, DEC);

  Serial.print(F("AVERAGE HIGH VOLTAGE = "));
  Serial.println(avg_voltage_high, DEC);
  Serial.print(F("STD DEV HIGH VOLTAGE = "));
  Serial.println(sdev_voltage_high, DEC);

  Serial.print(F("AVERAGE HIGH CURRENT = "));
  Serial.println(avg_current_high, DEC);
  Serial.print(F("STD DEV HIGH CURRENT = "));
  Serial.println(sdev_current_high, DEC);



  //! Set DAC current to zero between measurements
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, 0x0700);
  delay(500);

  //! Calculations
  vbe_delta = avg_voltage_high - avg_voltage_low;
  avg_current_ratio = avg_current_low / avg_current_high;
  ln_avg_current_ratio = log(avg_current_ratio); // natural log

  temperature = 
    (vbe_delta * (-1 / ln_avg_current_ratio) * q_by_n_k) - 273.15;
  Serial.print(F("\nEXTERNAL TEMPERATURE = "));
  Serial.println(temperature);

  return temperature;
}


//! return the BJT ideality factor of an external BJT hooked-up to IDAC0 and VIN0B at a give temperature
//!  This is almost the same as the temperature measurement, but calculate n instead of T
float ltc2970_print_bjt_n (float temperature)
{
  static int count = 10;  // number of samples per measurement (for averaging)
  int i; // loop counter
  uint16_t reg_vbe_low[count], 
    reg_i_low[count], 
    reg_vbe_high[count], 
    reg_i_high[count];

  float voltage_low[count], 
    voltage_high[count], //voltage measurements at low and high DAC current
    current_low[count], 
    current_high[count], //current measurements at low and high DAC current
    // the current measurement is just the voltage across a 10k resistor
    avg_voltage_low = 0.0,
    avg_voltage_high = 0.0,
    avg_current_low = 0.0,
    avg_current_high = 0.0,
    sdev_voltage_low = 0.0,
    sdev_voltage_high = 0.0,
    sdev_current_low = 0.0,
    sdev_current_high = 0.0,
    avg_current_ratio = 0.0,
    ln_avg_current_ratio = 0.0,
    vbe_delta = 0.0;

  
  // BJT properties
  float n = 0.0;
  static float q_by_k = 11.60452e3;
  float q_by_n_k = (q_by_k / n);
  
  //! Initialize the DAC
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, 0x0700);
  delay(500);
  
  //! Measure voltage and current at low current
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, dac_current_low);
  delay(500);
  for (i = 0; i < 10; i++) {
    reg_i_low[i] =
      (0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_B_ADC));
    reg_vbe_low[i] =
      (0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_A_ADC));
    voltage_low[i] = ((float)(reg_vbe_low[i] & 0x7FFF))*500e-6;
    current_low[i] = ((float)(reg_i_low[i] & 0x7FFF))*500e-6;
    avg_voltage_low += voltage_low[i];
    avg_current_low += current_low[i];
    //    Serial.println(voltage_low[i], DEC);
    //    Serial.println(current_low[i], DEC);
  }

  //! Measure voltage and current at high current
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, dac_current_high);
  delay(500);
  for (i = 0; i < 10; i++) {
    reg_i_high[i] =
      (0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_B_ADC));
    reg_vbe_high[i] =
      (0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_A_ADC));
    voltage_high[i] = ((float)(reg_vbe_high[i] & 0x7FFF))*500e-6; 
    current_high[i] = ((float)(reg_i_high[i] & 0x7FFF))*500e-6; 
    avg_voltage_high += voltage_high[i];
    avg_current_high += current_high[i];
    //    Serial.println(voltage_high[i], DEC);
    //    Serial.println(current_high[i], DEC);
  }
  //! Calculate averages, etc.
  avg_voltage_high = avg_voltage_high / count;
  avg_current_high = avg_current_high / count;
  avg_voltage_low = avg_voltage_low / count;
  avg_current_low = avg_current_low / count;
  for (i = 0; i < 10; i++) {
    // accumulate the squared differences
    sdev_voltage_low += pow((voltage_low[i] - avg_voltage_low), 2);
    sdev_current_low += pow((current_low[i] - avg_current_low), 2);
    sdev_voltage_high += pow((voltage_high[i] - avg_voltage_high), 2);
    sdev_current_high += pow((current_high[i] - avg_current_high), 2);
  }
  // divide by number of samples
  sdev_voltage_low = sdev_voltage_low / count;
  sdev_current_low = sdev_current_low / count;
  sdev_voltage_high = sdev_voltage_high / count;
  sdev_current_high = sdev_current_high / count;
  // square root for std dev
  sdev_voltage_low = sqrt(sdev_voltage_low);
  sdev_current_low = sqrt(sdev_current_low);
  sdev_voltage_high = sqrt(sdev_voltage_high);
  sdev_current_high = sqrt(sdev_current_high);
  

  //! Print statistics
  //  Serial.print(F("AVERAGE LOW VOLTAGE = "));
  //  Serial.println(avg_voltage_low, DEC);
  //  Serial.print(F("STD DEV LOW VOLTAGE = "));
  //  Serial.println(sdev_voltage_low, DEC);

  //  Serial.print(F("AVERAGE LOW CURRENT = "));
  //  Serial.println(avg_current_low, DEC);
  //  Serial.print(F("STD DEV LOW CURRENT = "));
  //  Serial.println(sdev_current_low, DEC);

  //  Serial.print(F("AVERAGE HIGH VOLTAGE = "));
  //  Serial.println(avg_voltage_high, DEC);
  //  Serial.print(F("STD DEV HIGH VOLTAGE = "));
  //  Serial.println(sdev_voltage_high, DEC);

  //  Serial.print(F("AVERAGE HIGH CURRENT = "));
  //  Serial.println(avg_current_high, DEC);
  //  Serial.print(F("STD DEV HIGH CURRENT = "));
  //  Serial.println(sdev_current_high, DEC);



  //! Set DAC current to zero between measurements
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, 0x0700);
  delay(500);

  //! Calculations
  vbe_delta = avg_voltage_high - avg_voltage_low;
  avg_current_ratio = avg_current_low / avg_current_high;
  ln_avg_current_ratio = log(avg_current_ratio); // natural log

  n = (vbe_delta * (-1 / ln_avg_current_ratio) * q_by_k * (1 / (temperature + 273.15))) ;
  Serial.print(F("\nBJT IDEALITY FACTOR n = "));
  Serial.println(n, DEC);

  bjt_n = n; // set the global n variable
  return n;
}


