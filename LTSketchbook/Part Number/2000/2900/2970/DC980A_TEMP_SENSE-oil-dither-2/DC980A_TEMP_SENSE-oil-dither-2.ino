/*
LTC2970: Dual I2C Power Supply Monitor and Margining Controller

Oil Bath Temperature Sense Board 8x LTC2970s with External BJT
This sketch reports temperature at each BJT
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

// there are 8 LTC2970s on the board, so we define them individually below
//#define LTC2970_I2C_ADDRESS 0x5B //global 7-bit address
//#define LTC2970_I2C_ADDRESS 0x6F //SLAVE_HH 7-bit address


/****************************************************************************/
// Global variables
uint8_t ltc2970_i2c_address[8];

static LT_SMBusNoPec *smbus = new LT_SMBusNoPec();


// Values for the external temperature sensor routine
static uint16_t dac_current_low = 0x0714;
static uint16_t dac_current_high = 0x07FD;
// array of DAC code offsets to add to the above dac values over successive samples to dither
//  elements must be zero-mean to preserve the DC value of the DAC output
// dithering this way assumes good DAC linearity so that +1 and -1 cancel exactly
//static int dac_current_dither[10] = {0,0,0,0,0,0,0,0,0,0}; // low-frequency tone, +/-0ma
static int dac_current_dither[20] = {0,-1,2,1,0,-1,1,-2,-1,0,1,-1,2,-1,0,-1,1,-2,1,1}; // low-frequency tone, +/-2u
//static int dac_current_dither[8] = {2,-2,2,-2,2,-2,2,-2}; // high-frequency tone +/-1ma

float bjt_n[8]; // = 1.016;

int meas_delay = 350; //number of milliseconds to wait for voltage settling between set and measure

/****************************************************************************/
//! Initialize Linduino
void setup()
{
  uint16_t return_val;
  int i;
  
  Serial.begin(115200);         //! Initialize the serial port to the PC

  // define individual chip addresses
  ltc2970_i2c_address[0] = 0x5C;
  ltc2970_i2c_address[1] = 0x5D;
  ltc2970_i2c_address[2] = 0x5E;
  ltc2970_i2c_address[3] = 0x5F;
  ltc2970_i2c_address[4] = 0x6B;
  ltc2970_i2c_address[5] = 0x6C;
  ltc2970_i2c_address[6] = 0x6D;
  ltc2970_i2c_address[7] = 0x6E;

  
  print_title();
  Serial.println(F("\n****INITIALIZING THE LTC2970****\n"));

  for(i = 0; i < 8; i++) {
    ltc2970_configure(ltc2970_i2c_address[i]);
    bjt_n[i] = 1.016;
  }
}

void loop()
{
  uint8_t user_command;
  float ext_temp = -273.15;
  int i;
  
  if (Serial.available())                //! Checks for user input
    {
      print_prompt();
      user_command = read_int();         //! Reads the user command
      switch (user_command)              //! Prints the appropriate submenu
	{

	case 1 :
	  Serial.print(F("\n****CALIBRATING EXTERNAL DIODES****\n"));
	  Serial.print(F("\n*****ENTER CENTIGRADE TEMPERATURE:"));
	  ext_temp = read_float();
	  Serial.println(F("\n"));
	  Serial.println(F("\n******* A *******:\n"));
	  ltc2970_print_bjt_n_dithered(ltc2970_i2c_address, 0, ext_temp);
	  Serial.println(F("\n******* B *******:\n"));
	  ltc2970_print_bjt_n_dithered(ltc2970_i2c_address, 1, ext_temp);
	  Serial.println(F("\n******* C *******:\n"));
	  ltc2970_print_bjt_n_dithered(ltc2970_i2c_address, 2, ext_temp);
	  Serial.println(F("\n******* D *******:\n"));
	  ltc2970_print_bjt_n_dithered(ltc2970_i2c_address, 3, ext_temp);
	  Serial.println(F("\n******* E *******:\n"));
	  ltc2970_print_bjt_n_dithered(ltc2970_i2c_address, 4, ext_temp);
	  Serial.println(F("\n******* F *******:\n"));
	  ltc2970_print_bjt_n_dithered(ltc2970_i2c_address, 5, ext_temp);
	  Serial.println(F("\n******* G *******:\n"));
	  ltc2970_print_bjt_n_dithered(ltc2970_i2c_address, 6, ext_temp);
	  Serial.println(F("\n******* H *******:\n"));
	  ltc2970_print_bjt_n_dithered(ltc2970_i2c_address, 7, ext_temp);
	  Serial.println(F("\n*****************:\n"));
	  break;
	  
	case 2 :
	  Serial.println(F("\n******* A *******:\n"));
	  ltc2970_print_external_temp_dithered(ltc2970_i2c_address, 0);
	  ltc2970_print_die_temp (ltc2970_i2c_address, 0);
	  Serial.println(F("\n******* B *******:\n"));
	  ltc2970_print_external_temp_dithered(ltc2970_i2c_address, 1);
	  ltc2970_print_die_temp (ltc2970_i2c_address, 1);
	  Serial.println(F("\n******* C *******:\n"));
	  ltc2970_print_external_temp_dithered(ltc2970_i2c_address, 2);
	  ltc2970_print_die_temp (ltc2970_i2c_address, 2);
	  Serial.println(F("\n******* D *******:\n"));
	  ltc2970_print_external_temp_dithered(ltc2970_i2c_address, 3);
	  ltc2970_print_die_temp (ltc2970_i2c_address, 3);
	  Serial.println(F("\n******* E *******:\n"));
	  ltc2970_print_external_temp_dithered(ltc2970_i2c_address, 4);
	  ltc2970_print_die_temp (ltc2970_i2c_address, 4);
	  Serial.println(F("\n******* F *******:\n"));
	  ltc2970_print_external_temp_dithered(ltc2970_i2c_address, 5);
	  ltc2970_print_die_temp (ltc2970_i2c_address, 5);
	  Serial.println(F("\n******* G *******:\n"));
	  ltc2970_print_external_temp_dithered(ltc2970_i2c_address, 6);
	  ltc2970_print_die_temp (ltc2970_i2c_address, 6);
	  Serial.println(F("\n******* H *******:\n"));
	  ltc2970_print_external_temp_dithered(ltc2970_i2c_address, 7);
	  ltc2970_print_die_temp (ltc2970_i2c_address, 7);
	  Serial.println(F("\n*****************:\n"));
	  break;

	case 3 :
	  Serial.print(F("\n****CLEARING BJT IDEALITY FACTORS****\n"));
	  Serial.print(F("\n*****ENTER BJT N VALUE: (nom = 1.016)"));
	  ext_temp = read_float();
	  for(i = 0; i < 8; i++) {
	    bjt_n[i] = ext_temp;
	  }
	  
	  break;
	  
	case 4 :
	  Serial.print(F("\n****RUNNING REPEATED MEASUREMENTS****\n"));
	  Serial.print(F("\n  ENTER CHANNEL NUMBER: \n"));
	  user_command = read_int();
	  Serial.print(F("TEMPERATURE, AVG_LOW_V, STD_DEV_LOW_V, AVG_LOW I, STD_DEV_LOW_I, AVG_HIGH_V, STD_DEV_HIGH_V, AVG_HIGH_I, STD_DEV_HIGH_I\n"));
	  for(i = 0; i < 50; i++) {
	    ltc2970_print_external_temp_dithered_csv(ltc2970_i2c_address, user_command);	    
	  }
	  break;
	  
	case 5 :
	  Serial.print(F("\n****RUNNING REPEATED MEASUREMENTS ALL CHANNELS****\n"));
	  Serial.println(F("EXT_TEMP[0],INT_TEMP[0],IDEALITY[0],EXT_TEMP[1],INT_TEMP[1],IDEALITY[1],EXT_TEMP[2],INT_TEMP[2],IDEALITY[2],EXT_TEMP[3],INT_TEMP[3],IDEALITY[3],EXT_TEMP[4],INT_TEMP[4],IDEALITY[4],EXT_TEMP[5],INT_TEMP[5],IDEALITY[5],EXT_TEMP[6],INT_TEMP[6],IDEALITY[6],EXT_TEMP[7],INT_TEMP[7],IDEALITY[7]"));
	  for(i = 0; i < 50; i++) {
	    ltc2970_print_external_temp_dithered_all_channels_csv (ltc2970_i2c_address);
	  }
	  Serial.print(F("\n****DONE****\n"));
	  break;

	case 6 :
	  Serial.print(F("\n****RUNNING REPEATED MEASUREMENTS ALL CHANNELS****\n"));
	  Serial.println(F("EXT_TEMP[0],INT_TEMP[0],IDEALITY[0],EXT_TEMP[1],INT_TEMP[1],IDEALITY[1],EXT_TEMP[2],INT_TEMP[2],IDEALITY[2],EXT_TEMP[3],INT_TEMP[3],IDEALITY[3],EXT_TEMP[4],INT_TEMP[4],IDEALITY[4],EXT_TEMP[5],INT_TEMP[5],IDEALITY[5],EXT_TEMP[6],INT_TEMP[6],IDEALITY[6],EXT_TEMP[7],INT_TEMP[7],IDEALITY[7]"));
	  for(i = 0; i < 50; i++) {
	    ltc2970_print_external_temp_single_all_channels_csv (ltc2970_i2c_address);
	  }
	  Serial.print(F("\n****DONE****\n"));
	  break;

	    
	default :	  
	  Serial.println(F("Incorrect Option"));
	  break;
	}
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
  Serial.println(F("\n"));
}

//! Prints main menu.
void print_prompt()
{
  Serial.println(F("\n"));
  Serial.print(F("  1  - Calibrate at a known temperature\n"));
  Serial.print(F("  2  - Measure temperature in a calibrated diode\n"));
  Serial.print(F("  3  - Set all diode ideality factors to a known value.\n"));
  Serial.print(F("  4  - Run 50 measurements on a designated channel (1 - 8). Dump CSV data.\n"));
  Serial.print(F("  5  - Run 50 measurements on all channels (1 - 8). Dump CSV data.\n"));
  Serial.print(F("  6  - Run 50 single measurements on all channels (1 - 8) NO AVERAGING. Dump CSV data.\n"));
  Serial.println(F("\nEnter a command number:\n"));
}




//! Writes configuration values to the LTC2970 registers
void ltc2970_configure(uint8_t local_i2c_address)
{
  uint16_t return_val;
  //start the 2970 by configuring all of its registers for this application
  // use SMbus commands
  smbus->writeWord(local_i2c_address, LTC2970_FAULT_EN, 0x0168);
  smbus->writeWord(local_i2c_address, LTC2970_IO, 0x000A);
  smbus->writeWord(local_i2c_address, LTC2970_ADC_MON, 0x007F);
  smbus->writeWord(local_i2c_address, LTC2970_VDD_OV, 0x2CEC);
  smbus->writeWord(local_i2c_address, LTC2970_VDD_UV, 0x2328);
  smbus->writeWord(local_i2c_address, LTC2970_V12_OV, 0x3FFF);
  smbus->writeWord(local_i2c_address, LTC2970_V12_UV, 0x00000);

  smbus->writeWord(local_i2c_address, LTC2970_CH1_A_OV, 0x2AF8);
  smbus->writeWord(local_i2c_address, LTC2970_CH1_A_UV, 0x2328);
  smbus->writeWord(local_i2c_address, LTC2970_CH1_A_SERVO, 0x0000);
  smbus->writeWord(local_i2c_address, LTC2970_CH1_A_IDAC, 0x0700);

  smbus->writeWord(local_i2c_address, LTC2970_CH1_B_OV, 0x3FFF);
  smbus->writeWord(local_i2c_address, LTC2970_CH1_B_UV, 0x0000);

  smbus->writeWord(local_i2c_address, LTC2970_CH0_A_OV, 0x1C5D);
  smbus->writeWord(local_i2c_address, LTC2970_CH0_A_UV, 0x1770);
  smbus->writeWord(local_i2c_address, LTC2970_CH0_A_SERVO, 0x0000);
  smbus->writeWord(local_i2c_address, LTC2970_CH0_A_IDAC, 0x0084);
  smbus->writeWord(local_i2c_address, LTC2970_CH0_B_OV, 0x3FFF);
  smbus->writeWord(local_i2c_address, LTC2970_CH0_B_UV, 0x0000);
}


//! Prints die temperature on the LTC2970
float ltc2970_print_die_temp (uint8_t local_i2c_address[], int n)
{
  static float temp_scale = 4;
  static float temp_offset = 1093; //adc codes

  float temperature;
  uint16_t return_val;
  //print the on-die temperature for the LTC2970
  return_val =
    smbus->readWord(local_i2c_address[n], LTC2970_TEMP_ADC);
  return_val = return_val & 0x7FFF; // drop bit 15

  temperature = ((float(return_val) - temp_offset) / temp_scale);

  Serial.print(F("\n LTC_2970 DIE TEMP: "));
  Serial.println(temperature, DEC);
  Serial.println(return_val, HEX);

  return temperature;
}


//! Returns die temperature on the LTC2970
float ltc2970_die_temp (uint8_t local_i2c_address[], int n)
{
  static float temp_scale = 4;
  static float temp_offset = 1093; //adc codes

  float temperature;
  uint16_t return_val;
  //print the on-die temperature for the LTC2970
  return_val =
    smbus->readWord(local_i2c_address[n], LTC2970_TEMP_ADC);
  return_val = return_val & 0x7FFF; // drop bit 15

  temperature = ((float(return_val) - temp_offset) / temp_scale);

  return temperature;
}

//! return the kelvin temperature of an external BJT hooked-up to IDAC0 and VIN0B
float ltc2970_print_external_temp_dithered (uint8_t local_i2c_address[], int n)
{
  static int count = 20;  // number of samples per measurement (for averaging)
  int i; // loop counter
  //  int meas_delay = 350; //number of milliseconds to wait for voltage settling between set and measure

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
  float q_by_n_k = (q_by_k / bjt_n[n]);
  
  //! Initialize the DAC
  smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, 0x0700);
  //  delay(meas_delay);
  
  //! Measure voltage and current at low current
  //! dither the DAC
  smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, dac_current_low);
  //  delay(meas_delay);
  for (i = 0; i < count; i++) {
    //    smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, (dac_current_low+dac_current_dither[i]));
        smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, dac_current_low);
    delay(meas_delay);
    reg_i_low[i] =
      (0x7FFF & smbus->readWord(local_i2c_address[n], LTC2970_CH1_B_ADC));
    reg_vbe_low[i] =
      (0x7FFF & smbus->readWord(local_i2c_address[n], LTC2970_CH1_A_ADC));
    voltage_low[i] = ((float)(reg_vbe_low[i] & 0x7FFF))*500e-6;
    current_low[i] = ((float)(reg_i_low[i] & 0x7FFF))*500e-6;
    avg_voltage_low += voltage_low[i];
    avg_current_low += current_low[i];
    //    Serial.println(voltage_low[i], DEC);
    //    Serial.println(current_low[i], DEC);
  }

  //! Measure voltage and current at high current
  smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, dac_current_high);
  //  delay(meas_delay);
  for (i = 0; i < count; i++) {
    smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, (dac_current_high+dac_current_dither[i]));
    delay(meas_delay);
    reg_i_high[i] =
      (0x7FFF & smbus->readWord(local_i2c_address[n], LTC2970_CH1_B_ADC));
    reg_vbe_high[i] =
      (0x7FFF & smbus->readWord(local_i2c_address[n], LTC2970_CH1_A_ADC));
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
  for (i = 0; i < count; i++) {
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
  smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, 0x0700);
  //  delay(meas_delay);

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


//! return the kelvin temperature of an external BJT hooked-up to IDAC0 and VIN0B
//!  print a comma-separated list of values calculated here:
//!  TEMPERATURE, AVG_LOW_V, STD_DEV_LOW_V, AVG_LOW I, STD_DEV_LOW_I, AVG_HIGH_V, STD_DEV_HIGH_V, AVG_HIGH_I, STD_DEV_HIGH_I

float ltc2970_print_external_temp_dithered_csv (uint8_t local_i2c_address[], int n)
{
  static int count = 20;  // number of samples per measurement (for averaging)
  int i; // loop counter
  //  int meas_delay = 350; //number of milliseconds to wait for voltage settling between set and measure

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
  float q_by_n_k = (q_by_k / bjt_n[n]);
  
  //! Initialize the DAC
  smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, 0x0700);
  //  delay(meas_delay);
  
  //! Measure voltage and current at low current
  //! dither the DAC
  smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, dac_current_low);
  //  delay(meas_delay);
  for (i = 0; i < count; i++) {
    //    smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, dac_current_low);
    delay(meas_delay);
    reg_i_low[i] =
      (0x7FFF & smbus->readWord(local_i2c_address[n], LTC2970_CH1_B_ADC));
    reg_vbe_low[i] =
      (0x7FFF & smbus->readWord(local_i2c_address[n], LTC2970_CH1_A_ADC));
    voltage_low[i] = ((float)(reg_vbe_low[i] & 0x7FFF))*500e-6;
    current_low[i] = ((float)(reg_i_low[i] & 0x7FFF))*500e-6;
    avg_voltage_low += voltage_low[i];
    avg_current_low += current_low[i];
    //    Serial.println(voltage_low[i], DEC);
    //    Serial.println(current_low[i], DEC);
  }

  //! Measure voltage and current at high current
  smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, dac_current_high);
  //  delay(meas_delay);
  for (i = 0; i < count; i++) {
    smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, (dac_current_high+dac_current_dither[i]));
    delay(meas_delay);
    reg_i_high[i] =
      (0x7FFF & smbus->readWord(local_i2c_address[n], LTC2970_CH1_B_ADC));
    reg_vbe_high[i] =
      (0x7FFF & smbus->readWord(local_i2c_address[n], LTC2970_CH1_A_ADC));
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
  for (i = 0; i < count; i++) {
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
  
  //! Calculations
  vbe_delta = avg_voltage_high - avg_voltage_low;
  avg_current_ratio = avg_current_low / avg_current_high;
  ln_avg_current_ratio = log(avg_current_ratio); // natural log

  temperature = 
    (vbe_delta * (-1 / ln_avg_current_ratio) * q_by_n_k) - 273.15;

  //! Print statistics
  //!  TEMPERATURE, AVG_LOW_V, STD_DEV_LOW_V, AVG_LOW I, STD_DEV_LOW_I, AVG_HIGH_V, STD_DEV_HIGH_V, AVG_HIGH_I, STD_DEV_HIGH_I
  //  Serial.print(F("\n"));
  Serial.print(temperature);
  Serial.print(F(","));
  Serial.print(avg_voltage_low, DEC);
  Serial.print(F(","));
  Serial.print(sdev_voltage_low, DEC);
  Serial.print(F(","));
  Serial.print(avg_current_low, DEC);
  Serial.print(F(","));
  Serial.print(sdev_current_low, DEC);
  Serial.print(F(","));
  Serial.print(avg_voltage_high, DEC);
  Serial.print(F(","));
  Serial.print(sdev_voltage_high, DEC);
  Serial.print(F(","));
  Serial.print(avg_current_high, DEC);
  Serial.print(F(","));
  Serial.print(sdev_current_high, DEC);
  Serial.print(F("\n"));

  //! Set DAC current to zero between measurements
  smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, 0x0700);
  //  delay(meas_delay);


  return temperature;
}


//! return the kelvin temperature of an external BJT hooked-up to IDAC0 and VIN0B
//!  print a comma-separated list of values calculated here:
//!  TEMPERATURE, AVG_LOW_V, STD_DEV_LOW_V, AVG_LOW I, STD_DEV_LOW_I, AVG_HIGH_V, STD_DEV_HIGH_V, AVG_HIGH_I, STD_DEV_HIGH_I

float ltc2970_print_external_temp_dithered_all_channels_csv (uint8_t local_i2c_address[])
{
  static int count = 20;  // number of samples per measurement (for averaging)
  int i, n; // loop counter
  //  int meas_delay = 350; //number of milliseconds to wait for voltage settling between set and measure

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

  float ext_temperature[8];
  float int_temperature[8];
  
  for(n = 0; n < 8; n++) { // loop through all 8 LTC2970 devices

    // BJT properties
    //  static float n = 1.016;
    static float q_by_k = 11.60452e3;
    float q_by_n_k = (q_by_k / bjt_n[n]);

    avg_voltage_low = 0.0;
    avg_voltage_high = 0.0;
    avg_current_low = 0.0;
    avg_current_high = 0.0;
    sdev_voltage_low = 0.0;
    sdev_voltage_high = 0.0;
    sdev_current_low = 0.0;
    sdev_current_high = 0.0;
    avg_current_ratio = 0.0;
    ln_avg_current_ratio = 0.0;
    vbe_delta = 0.0;

    
    //! Initialize the DAC
    smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, 0x0700);
    //delay(meas_delay);
    
    //! Measure voltage and current at low current
    //! dither the DAC
    smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, dac_current_low);
    delay(meas_delay);
    for (i = 0; i < count; i++) {
      //      smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, dac_current_low);
      delay(dac_current_dither[i]);
      reg_i_low[i] =
	(0x7FFF & smbus->readWord(local_i2c_address[n], LTC2970_CH1_B_ADC));
      reg_vbe_low[i] =
	(0x7FFF & smbus->readWord(local_i2c_address[n], LTC2970_CH1_A_ADC));
      voltage_low[i] = ((float)(reg_vbe_low[i] & 0x7FFF))*500e-6;
      current_low[i] = ((float)(reg_i_low[i] & 0x7FFF))*500e-6;
      avg_voltage_low += voltage_low[i];
      avg_current_low += current_low[i];
      //    Serial.println(voltage_low[i], DEC);
      //    Serial.println(current_low[i], DEC);
    }

    //! Measure voltage and current at high current
    smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, dac_current_high);
    delay(meas_delay);
    for (i = 0; i < count; i++) {
      //      smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, (dac_current_high+dac_current_dither[i]));
      //            smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, dac_current_high);
      delay(dac_current_dither[i]);
      reg_i_high[i] =
	(0x7FFF & smbus->readWord(local_i2c_address[n], LTC2970_CH1_B_ADC));
      reg_vbe_high[i] =
	(0x7FFF & smbus->readWord(local_i2c_address[n], LTC2970_CH1_A_ADC));
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
    for (i = 0; i < count; i++) {
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
    
    //! Calculations
    vbe_delta = avg_voltage_high - avg_voltage_low;
    avg_current_ratio = avg_current_low / avg_current_high;
    ln_avg_current_ratio = log(avg_current_ratio); // natural log
    
    ext_temperature[n] = 
      (vbe_delta * (-1 / ln_avg_current_ratio) * q_by_n_k) - 273.15;

    int_temperature[n] = ltc2970_die_temp(ltc2970_i2c_address, n);
      
    //  }
  
    //! Print statistics
    //!  EXT_TEMPERATURE[0], INT_TEMPERATURE[0], IDEALITY_FACTOR[0], ..., EXT_TEMPERATURE[7], INT_TEMPERATURE[7], IDEALITY_FACTOR[7]
    //  Serial.print(F("\n"));
    //  for(n = 0; n < 8; n++) { // loop through all 8 LTC2970 devices
    Serial.print(ext_temperature[n]);
    Serial.print(F(","));
    Serial.print(int_temperature[n]);
    Serial.print(F(","));
    Serial.print(bjt_n[n], DEC);
    Serial.print(F(","));

    //! Set DAC current to zero between measurements
    smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, 0x0700);
    //  delay(meas_delay);
    
  }
  Serial.print(F("\n"));


  return ext_temperature[0];
}

//! print the celsius temperature of 8 external BJTs hooked-up to IDAC0 and VIN0B
//!  print a comma-separated list of values calculated here:
//!  EXT_TEMPERATURE[n], INT_TEMP[n], BJT_IDEALITY[n],...

float ltc2970_print_external_temp_single_all_channels_csv (uint8_t local_i2c_address[])
{
  static int count = 20;  // number of samples per measurement (for averaging)
  int n, i; // loop counters
  
  uint16_t reg_vbe_low,
    reg_i_low,
    reg_vbe_high,
    reg_i_high;

  float voltage_low, voltage_high, //voltage measurements at low and high DAC current
    current_low, current_high, //current measurements at low and high DAC current
    // the current measurement is just the voltage across a 10k resistor
    current_ratio = 0.0,
    ln_current_ratio = 0.0,
    vbe_delta = 0.0;

  float ext_temperature[8];
  float int_temperature[8];
  
  // BJT properties
  //  static float n = 1.016;
  static float q_by_k = 11.60452e3;
  float q_by_n_k = (q_by_k / bjt_n[n]);
  
  
  //HACKED TO DO ONLY THE FIRST DEVICE!!!
  for(n = 0; n < 1; n++) { // loop through all 8 LTC2970 devices

    ext_temperature[n] = 0.0;
    current_ratio = 0.0;
    ln_current_ratio = 0.0;
    vbe_delta = 0.0;

    for (i = 0; i < count; i++) { // perform the average over <count> temperature measurements with dither
      //! Initialize the DAC
      smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, 0x0700);
      //delay(meas_delay);
      
      //! Measure voltage and current at low current
      //! dither the DAC
      //      smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, (dac_current_low+dac_current_dither[i]));
      smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, dac_current_low);
      delay(meas_delay+dac_current_dither[i]);
      reg_i_low =
	(0x7FFF & smbus->readWord(local_i2c_address[n], LTC2970_CH1_B_ADC));
      reg_vbe_low =
	(0x7FFF & smbus->readWord(local_i2c_address[n], LTC2970_CH1_A_ADC));
      voltage_low = ((float)(reg_vbe_low & 0x7FFF))*500e-6;
      current_low = ((float)(reg_i_low & 0x7FFF))*500e-6;
      
      //! Measure voltage and current at high current
      //      smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, (dac_current_high+dac_current_dither[i]));
      smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, dac_current_high);
      delay(meas_delay+dac_current_dither[i]);
      reg_i_high =
	(0x7FFF & smbus->readWord(local_i2c_address[n], LTC2970_CH1_B_ADC));
      reg_vbe_high =
	(0x7FFF & smbus->readWord(local_i2c_address[n], LTC2970_CH1_A_ADC));
      voltage_high = ((float)(reg_vbe_high & 0x7FFF))*500e-6; 
      current_high = ((float)(reg_i_high & 0x7FFF))*500e-6; 
      
    
      //! Calculations
      vbe_delta = voltage_high - voltage_low;
      current_ratio = current_low / current_high;
      ln_current_ratio = log(current_ratio); // natural log
    
      // Accumulate temperature measurements
      ext_temperature[n] += 
	(vbe_delta * (-1 / ln_current_ratio) * q_by_n_k) - 273.15;
    } // for i

    ext_temperature[n] = ext_temperature[n] / count; // divide the accumulated total to get the mean average
    
    int_temperature[n] = ltc2970_die_temp(ltc2970_i2c_address, n);
      
  
    //! Print statistics
    //!  EXT_TEMPERATURE[0], INT_TEMPERATURE[0], IDEALITY_FACTOR[0], ..., EXT_TEMPERATURE[7], INT_TEMPERATURE[7], IDEALITY_FACTOR[7]
    //  Serial.print(F("\n"));
    //  for(n = 0; n < 8; n++) { // loop through all 8 LTC2970 devices
    Serial.print(ext_temperature[n]);
    Serial.print(F(","));
    Serial.print(int_temperature[n]);
    Serial.print(F(","));
    Serial.print(bjt_n[n], DEC);
    Serial.print(F(","));

    //! Set DAC current to zero between measurements
    smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, 0x0700);
    //  delay(meas_delay);
    
  } // for n
  
  Serial.print(F("\n"));

  return ext_temperature[0];
}




//! return the BJT ideality factor of an external BJT hooked-up to IDAC0 and VIN0B at a give temperature
//!  This is almost the same as the temperature measurement, but calculate n instead of T
float ltc2970_print_bjt_n_dithered (uint8_t local_i2c_address[], int n, float temperature)
{
  static int count = 20;  // number of samples per measurement (for averaging)
  int i; // loop counter
  //  int meas_delay = 350; //number of milliseconds to wait for voltage settling between set and measure

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
  float calc_n = 0.0;
  static float q_by_k = 11.60452e3;
  float q_by_n_k = (q_by_k / n);
  
  //! Initialize the DAC
  smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, 0x0700);
  //  delay(meas_delay);
  
  //! Measure voltage and current at low current
  //! Dither the DAC
  smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, dac_current_low);
  //  delay(meas_delay);
  for (i = 0; i < count; i++) {
    smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, dac_current_low);
    delay(meas_delay);
    reg_i_low[i] =
      (0x7FFF & smbus->readWord(local_i2c_address[n], LTC2970_CH1_B_ADC));
    reg_vbe_low[i] =
      (0x7FFF & smbus->readWord(local_i2c_address[n], LTC2970_CH1_A_ADC));
    voltage_low[i] = ((float)(reg_vbe_low[i] & 0x7FFF))*500e-6;
    current_low[i] = ((float)(reg_i_low[i] & 0x7FFF))*500e-6;
    avg_voltage_low += voltage_low[i];
    avg_current_low += current_low[i];
    //    Serial.println(voltage_low[i], DEC);
    //    Serial.println(current_low[i], DEC);
  }

  //! Measure voltage and current at high current
  smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, dac_current_high);
  //  delay(meas_delay);
  for (i = 0; i < count; i++) {
    smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, (dac_current_high+dac_current_dither[i]));
    delay(meas_delay);
    reg_i_high[i] =
      (0x7FFF & smbus->readWord(local_i2c_address[n], LTC2970_CH1_B_ADC));
    reg_vbe_high[i] =
      (0x7FFF & smbus->readWord(local_i2c_address[n], LTC2970_CH1_A_ADC));
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
  for (i = 0; i < count; i++) {
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
  smbus->writeWord(local_i2c_address[n], LTC2970_CH1_A_IDAC, 0x0700);
  //  delay(meas_delay);

  //! Calculations
  vbe_delta = avg_voltage_high - avg_voltage_low;
  avg_current_ratio = avg_current_low / avg_current_high;
  ln_avg_current_ratio = log(avg_current_ratio); // natural log

  calc_n = (vbe_delta * (-1 / ln_avg_current_ratio) * q_by_k * (1 / (temperature + 273.15))) ;
  Serial.print(F("\nBJT IDEALITY FACTOR n = "));
  Serial.println(calc_n, DEC);

  bjt_n[n] = calc_n; // set the global n variable
  return n;
}


