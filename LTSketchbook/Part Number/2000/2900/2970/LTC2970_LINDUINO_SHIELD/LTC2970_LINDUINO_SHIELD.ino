/*
LTC2970: Dual I2C Power Supply Monitor and Margining Controller

Linduino shield containing one switching regulator and one external temperature sensor.
The LTC2970 reads the switching regulator through CH0 (A=voltage, B=current).
The LTC2970 reads the temperature sensor through CH1 (A=current, B=Vbe).

The temperature sensing algorithm uses delta-Vbe measurements across the diode, 

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
#define LTC2970_I2C_ADDRESS 0x5B //global 7-bit address
//#define LTC2970_I2C_ADDRESS 0x6F //SLAVE_HH 7-bit address


/****************************************************************************/
// Global variables
uint8_t ltc2970_i2c_address[1];

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


// Values for the external temperature sensor routine
const uint16_t dac_current_low = 0x0714;
const uint16_t dac_current_high = 0x07FF;

float bjt_n[1]; // = 1.016;

int meas_delay = 350; //number of milliseconds to wait for voltage settling between set and measure

const uint16_t SOFT_CONNECT_DELAY = 1000; // milliseconds to wait for soft connect

const float IMON_RES_VAL = 0.02;  // current sense resistor value

/****************************************************************************/
//! Initialize Linduino
void setup()
{
  uint16_t return_val;
  int i;
  
  Serial.begin(115200);         //! Initialize the serial port to the PC

  // define individual chip addresses
  ltc2970_i2c_address[0] = 0x5C;

  
  print_title();
  print_prompt();
  
  Serial.println(F("\n****INITIALIZING THE LTC2970****\n"));
  dac0_value = 0x0084;
  dac0_ctrl = 0x0000;
  idac0_reg = dac0_ctrl + dac0_value;
  dac1_value = 0x0084;
  dac1_ctrl = 0x0000;
  idac1_reg = dac1_ctrl + dac1_value;

  servo0_value = 0x0001;
  servo1_value = 0xEFFF;
  servo0_value_nom = 0x2733;
  servo1_value_nom = 0x1A24;
  servo0_value_marg = 0x0001; // 10% low
  servo1_value_marg = 0xEFFF; // 10% low


  ltc2970_configure(ltc2970_i2c_address[0]);
  bjt_n[0] = 1.016;
}

void loop()
{
  uint8_t user_command;
  float ext_temp = -273.15;
  uint16_t return_val;
  int i;
  
  if (Serial.available())                //! Checks for user input
    {
      user_command = read_int();         //! Reads the user command
      switch (user_command)              //! Prints the appropriate submenu
	{
	case 0 :
	  // temperature sub-menu
	  print_temperature_sub_menu();
	  do_temperature_sub_menu();
	  break;
	    
	case 1 :
	  Serial.print(F("\n****INITIALIZING THE LTC2970****\n"));
	  ltc2970_configure(ltc2970_i2c_address[0]);
	  break;
	  
	case 2 :
	  Serial.print(F("\n****ENABLE LTC2970 CHANNEL 0 ****\n"));
	  ltc2970_dac_disconnect(0);
	  ltc2970_gpio_up(0);
	  break;
	  
	case 3 :
	  Serial.print(F("\n****SOFT CONNECT LTC2970 DAC0 ****\n"));
	  ltc2970_soft_connect_dac(0);
	  break;
	  
	case 4 :
	  Serial.print(F("\n****SERVO CHANNEL 0 VOLTAGE 10% LOW****\n"));
	  ltc2970_servo_to_adc_val(0, servo0_value_marg);
	  break;
	  
	case 5 :
	  Serial.print(F("\n****SERVO CHANNEL 0 VOLTAGE TO NOMINAL****\n"));
	  ltc2970_servo_to_adc_val(0, servo0_value_nom);
	  break;
	  
	case 6 :
	  Serial.print(F("\n****ADC CH_0 VOLTAGE = "));
	  return_val = smbus->readWord(ltc2970_i2c_address[0],LTC2970_CH0_A_ADC);
	  Serial.println(((return_val & 0x7FFF)*500e-6), DEC);
	  break;
	    
	case 7 :
	  Serial.print(F("\n****ADC CH_0 CURRENT = "));
	  return_val = smbus->readWord(ltc2970_i2c_address[0],LTC2970_CH0_B_ADC);
	  Serial.println((((return_val & 0x7FFF)*500e-6)/IMON_RES_VAL), DEC);
	  break;
	  
	case 8 :
	  Serial.print(F("\n****PRINT FAULTS, CLEAR LATCHED FAULTS \n"));
	  ltc2970_read_faults();
	  break;
	    
	case 9 :
	  Serial.print(F("\n****PRINT DIE TEMPERATURE \n"));
	  ltc2970_print_die_temp (ltc2970_i2c_address, 0);
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
  Serial.print(F("\n"));
  Serial.print(F("  0  - Temperature Menu\n"));
  Serial.print(F("  1  - Reset the LTC2970, Disable Regulator\n"));
  Serial.print(F("  2  - Enable Channel 0 ; DACs disconnected\n"));
  Serial.print(F("  3  - Soft-Connect DAC0, and Confirm Connection\n"));
  Serial.print(F("  4  - Servo Channel 0 Voltage 10% low\n"));
  Serial.print(F("  5  - Servo Channel 0 Voltage to nominal\n"));
  Serial.print(F("  6  - Print Channel 0 Voltage\n"));
  Serial.print(F("  7  - Print Channel 0 Current\n"));
  Serial.print(F("  8  - Print Fault Register Contents\n"));
  Serial.print(F("  9  - Print LTC2970 Temperature\n"));
  Serial.print(F("\nEnter a command number:"));
}


//! Prints temperature sub-menu menu.
void print_temperature_sub_menu()
{
  Serial.println(F("\n"));
  Serial.print(F("  1  - Measure temperature in the external diode\n"));
  Serial.print(F("  2  - Run 50 external temperature measurements. Dump CSV data.\n"));
  Serial.println(F("\nEnter a command number:\n"));
  //  Serial.flush();
}

void do_temperature_sub_menu()
{
  uint8_t foo;
  int i;
  float ext_temp;
  
  foo = read_int();         //! Reads the user command
  
  switch (foo)
    {  
    case 1 :
      Serial.print(F("\n****SINGLE TEMPERATURE MEASUREMENT****\n"));
      Serial.print(F("EXTERNAL TEMPERATURE: "));
      Serial.print(ltc2970_external_temp(ltc2970_i2c_address[0]));
      ltc2970_print_die_temp (ltc2970_i2c_address, 0);
      break;
      
    case 2 :
      Serial.print(F("\n****RUNNING REPEATED MEASUREMENTS****\n"));
      Serial.print(F("TEMPERATURE, LOW_VBE, LOW I,HIGH_VBE, HIGH_I\n"));
      for(i = 0; i < 50; i++) {
	ltc2970_print_external_temp_csv(ltc2970_i2c_address[0]);
      }
      break;
      
    default :	  
      Serial.println(F("Incorrect Option."));
      break;
    }
}


//! Writes configuration values to the LTC2970 registers
void ltc2970_configure(uint8_t local_i2c_address)
{
  uint16_t return_val;
  //start the 2970 by configuring all of its registers for this application
  // use SMbus commands
  //only care about faults on channel 0
  smbus->writeWord(local_i2c_address, LTC2970_FAULT_EN, 0x001F);
  smbus->writeWord(local_i2c_address, LTC2970_IO, 0x000A);
  smbus->writeWord(local_i2c_address, LTC2970_ADC_MON, 0x007F);
  smbus->writeWord(local_i2c_address, LTC2970_VDD_OV, 0x2CEC);
  smbus->writeWord(local_i2c_address, LTC2970_VDD_UV, 0x2328);
  smbus->writeWord(local_i2c_address, LTC2970_V12_OV, 0x3FFF);
  smbus->writeWord(local_i2c_address, LTC2970_V12_UV, 0x00000);

  smbus->writeWord(local_i2c_address, LTC2970_CH1_A_OV, 0xEFFF);
  smbus->writeWord(local_i2c_address, LTC2970_CH1_A_UV, 0x0001);
  smbus->writeWord(local_i2c_address, LTC2970_CH1_A_SERVO, 0x0000);
  smbus->writeWord(local_i2c_address, LTC2970_CH1_A_IDAC, 0x0700);
  smbus->writeWord(local_i2c_address, LTC2970_CH1_B_OV, 0x3FFF);
  smbus->writeWord(local_i2c_address, LTC2970_CH1_B_UV, 0x0000);

  smbus->writeWord(local_i2c_address, LTC2970_CH0_A_OV, 0x0A28);
  smbus->writeWord(local_i2c_address, LTC2970_CH0_A_UV, 0x0898);
  smbus->writeWord(local_i2c_address, LTC2970_CH0_A_SERVO, 0x0000);
  smbus->writeWord(local_i2c_address, LTC2970_CH0_A_IDAC, 0x0084);
  smbus->writeWord(local_i2c_address, LTC2970_CH0_B_OV, 0x3FFF);
  smbus->writeWord(local_i2c_address, LTC2970_CH0_B_UV, 0x0000);
}


//! Read FAULT, FAULT_LA, and FAULT_LA_INDEX registers
void ltc2970_read_faults()
{
  uint16_t return_val;

  return_val =
    smbus->readWord(ltc2970_i2c_address[0],LTC2970_FAULT);
  Serial.print(F("\n LTC2970_FAULT: "));
  Serial.println(return_val, HEX);
  return_val =
    smbus->readWord(ltc2970_i2c_address[0],LTC2970_FAULT_LA);
  Serial.print(F("\n LTC2970_FAULT_LA: "));
  Serial.println(return_val, HEX);
  return_val =
    smbus->readWord(ltc2970_i2c_address[0],LTC2970_FAULT_LA_INDEX);
  Serial.print(F("\n LTC2970_FAULT_LA_INDEX: "));
  Serial.println(return_val, HEX);

}

//! Set GPIO_n high
void ltc2970_gpio_up(int gpio_number)
{
  uint16_t return_val;
  if (gpio_number == 0)
  {
    return_val = smbus->readWord(ltc2970_i2c_address[0],LTC2970_IO);
    return_val = (return_val | 0xFEEC) | 0x0012;
    smbus->writeWord(ltc2970_i2c_address[0], LTC2970_IO, return_val);
  }
  else if (gpio_number == 1)
  {
    return_val = smbus->readWord(ltc2970_i2c_address[0],LTC2970_IO);
    return_val = (return_val | 0xFED3) | 0x0028;
    smbus->writeWord(ltc2970_i2c_address[0], LTC2970_IO, return_val);
  }
  else
  {
    // error, no such GPIO
  }
}

//! Set GPIO_n low
void ltc2970_gpio_down(int gpio_number)
{
  uint16_t return_val;
  if (gpio_number == 0)
  {
    return_val = smbus->readWord(ltc2970_i2c_address[0],LTC2970_IO);
    return_val = (return_val | 0xFEEC) | 0x0010;
    smbus->writeWord(ltc2970_i2c_address[0], LTC2970_IO, return_val);
  }
  else if (gpio_number == 1)
  {
    return_val = smbus->readWord(ltc2970_i2c_address[0],LTC2970_IO);
    return_val = (return_val | 0xFED3) | 0x0020;
    smbus->writeWord(ltc2970_i2c_address[0], LTC2970_IO, return_val);
  }
  else
  {
    // error, no such GPIO
  }
}

//! Unceremoniously connect DAC0 to the control node
//!  no attempt to equalize voltages
void ltc2970_hard_connect_dac(int dac_number)
{
  uint16_t return_val;
  if (dac_number == 0)
  {
    // use the global DAC variables
    Serial.print(F("\nHARD CONNECT CHANNEL 0 : "));
    dac0_ctrl = 0x0300;
    idac0_reg = dac0_ctrl + (0x00FF | dac0_value);

    smbus->writeWord(ltc2970_i2c_address[0], LTC2970_CH0_A_IDAC, idac0_reg);
    return_val = smbus->readWord(ltc2970_i2c_address[0],LTC2970_CH0_A_IDAC);
    Serial.println(return_val, HEX);
  }
  else if (dac_number == 1)
  {
    // use the global DAC variables
    Serial.print(F("\nHARD CONNECT CHANNEL 1 : "));
    dac1_ctrl = 0x0300;
    idac1_reg = dac1_ctrl + (0x00FF | dac1_value);

    smbus->writeWord(ltc2970_i2c_address[0], LTC2970_CH1_A_IDAC, idac1_reg);
  }
  else
  {
    Serial.print(F("\nERROR CANNOT HARD CONNECT NON-EXISTANT CHANNEL"));
    // error, no such DAC
  }
}

//! soft-connect DACn to its controlled node
int ltc2970_soft_connect_dac(int dac_number)
{
  uint16_t return_val;
  if (dac_number == 0)
  {
    // check for existing faults
    return_val = smbus->readWord(ltc2970_i2c_address[0],LTC2970_FAULT);
    //    if ((return_val & 0x001F) == 0x0000) {
    if ((return_val & 0x001B) == 0x0000)
    {
      // make sure that the channel is not already connected
      return_val = smbus->readWord(ltc2970_i2c_address[0],LTC2970_CH0_A_IDAC);
      if ((return_val & 0x0300) == 0x0000)
      {
        // the soft-connect operation can succeed with no faults, setting IDAC[9] = 1
        // or it can fail, and not set IDAC[9]
        // we wait a safe amount of time, then check for results
        Serial.print(F("\nSOFT CONNECT CHANNEL 0 : "));
        dac0_ctrl = 0x0100;
        dac0_value = 0x0080;
        idac0_reg = dac0_ctrl + (0x00FF | dac0_value);
        smbus->writeWord(ltc2970_i2c_address[0], LTC2970_CH0_A_IDAC, idac0_reg);
        delay(SOFT_CONNECT_DELAY);
        return_val = smbus->readWord(ltc2970_i2c_address[0],LTC2970_CH0_A_IDAC);
        Serial.println(return_val, HEX);
        if ((return_val & 0x0300) == 0x0000)
        {
          Serial.print(F("\nCHANNEL 0 FAILED TO CONNECT"));
          Serial.print(F("\n  FAULT REGISTER: "));
          return_val = smbus->readWord(ltc2970_i2c_address[0],LTC2970_FAULT);
          Serial.println(return_val, HEX);
        }
        else
        {
          Serial.print(F("\nCHANNEL 0 SOFT CONNECT SUCCESS"));
          return_val = smbus->readWord(ltc2970_i2c_address[0],LTC2970_CH0_A_ADC);
          servo0_value = (return_val & 0x7FFF);
        }
      }
      else
      {
        Serial.print(F("\nCHANNEL 0 ALREADY CONNECTED"));
      }
    }
    else
    {
      Serial.print(F("\nERROR: CANNOT SOFT-CONNECT WITH FAULTS ON CHANNEL 0: "));
      Serial.println(return_val, HEX);
    }
  }
  else if (dac_number == 1)
  {
    // check for existing faults
    return_val = smbus->readWord(ltc2970_i2c_address[0],LTC2970_FAULT);
    //    if ((return_val & 0x03E0) == 0x0000) {
    if ((return_val & 0x0360) == 0x0000)
    {
      // make sure that the channel is not already connected
      return_val = smbus->readWord(ltc2970_i2c_address[0],LTC2970_CH1_A_IDAC);
      if ((return_val & 0x0300) == 0x0000)
      {
        // the soft-connect operation can succeed with no faults, setting IDAC[9] = 1
        // or it can fail, and not set IDAC[9]
        // we wait a safe amount of time, then check for results
        Serial.print(F("\nSOFT CONNECT CHANNEL 1 : "));
        dac1_ctrl = 0x0100;
        dac1_value = 0x0080;
        idac1_reg = dac1_ctrl + (0x00FF | dac1_value);
        smbus->writeWord(ltc2970_i2c_address[0], LTC2970_CH1_A_IDAC, idac1_reg);
        delay(SOFT_CONNECT_DELAY);
        return_val = smbus->readWord(ltc2970_i2c_address[0],LTC2970_CH1_A_IDAC);
        Serial.println(return_val, HEX);
        if ((return_val & 0x0300) == 0x0000)
        {
          Serial.print(F("\nCHANNEL 1 FAILED TO CONNECT"));
          Serial.print(F("\n  FAULT REGISTER: "));
          return_val = smbus->readWord(ltc2970_i2c_address[0],LTC2970_FAULT);
          Serial.println(return_val, HEX);
        }
        else
        {
          Serial.print(F("\nCHANNEL 1 SOFT CONNECT SUCCESS"));
          return_val = smbus->readWord(ltc2970_i2c_address[0],LTC2970_CH1_A_ADC);
          servo1_value = (return_val & 0x7FFF);
        }
      }
      else
      {
        Serial.print(F("\nCHANNEL 1 ALREADY CONNECTED"));
      }
    }
    else
    {
      Serial.print(F("\nERROR: CANNOT SOFT-CONNECT WITH FAULTS ON CHANNEL 1: "));
      Serial.println(return_val, HEX);
    }
  }
  else
  {
    Serial.print(F("\nERROR: CANNOT HARD CONNECT NON-EXISTANT CHANNEL"));
    // error, no such DAC
  }
  Serial.print(F("\n\n"));
}


//! Disconnect a DAC from its channel
void ltc2970_dac_disconnect(int dac_number)
{
  uint16_t return_val;
  if (dac_number == 0)
  {
    Serial.print(F("\nDISCONNECT CHANNEL 0 : "));
    dac0_ctrl = 0x0000;
    idac0_reg = dac0_ctrl + (0x00FF | dac0_value);

    smbus->writeWord(ltc2970_i2c_address[0], LTC2970_CH0_A_IDAC, idac0_reg);
  }
  else if (dac_number == 1)
  {
    Serial.print(F("\nDISCONNECT CHANNEL 1 : "));
    dac1_ctrl = 0x0000;
    idac1_reg = dac1_ctrl + (0x00FF | dac1_value);

    smbus->writeWord(ltc2970_i2c_address[0], LTC2970_CH1_A_IDAC, idac1_reg);
  }
  else
  {
    Serial.print(F("\nERROR CANNOT DISCONNECT NON-EXISTANT CHANNEL"));
    // error, no such DAC
  }
}





//! Servo once to a given ADC value
void ltc2970_servo_to_adc_val(int channel_number, uint16_t code)
{
  uint16_t return_val;

  uint16_t code_in;
  uint16_t code_max = 0x18B7,
           code_min = 0x0AE0;

  // get rid of bit 15 to make calculations easier
  code_in = (code & 0x7FFF);

  //  code_in = (code_in > code_max) ? code_max : code_in;
  //  code_in = (code_in < code_min) ? code_min : code_in;

  // ensure that bit 15 is high to enable servoing
  code_in = (code_in + 0x8000);

  if (channel_number == 0)
  {
    Serial.print(F("\nSERVO CHANNEL 0  "));
    servo0_value = code_in;
    smbus->writeWord(ltc2970_i2c_address[0], LTC2970_CH0_A_SERVO, code_in);

    return_val = smbus->readWord(ltc2970_i2c_address[0],LTC2970_CH0_A_SERVO);
    Serial.println(return_val, HEX);
  }
  else if (channel_number == 1)
  {
    Serial.print(F("\nSERVO CHANNEL 1  "));
    Serial.println(code_in, HEX);
    servo1_value = code_in;
    smbus->writeWord(ltc2970_i2c_address[0], LTC2970_CH1_A_SERVO, code_in);
  }
  else
  {
    Serial.print(F("\nERROR CANNOT SERVO NON-EXISTANT CHANNEL"));
    // error, no such channel
  }
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


//! Returns die temperature on the LTC2970 (without printing)
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
//! Perform a single measurement, no averaging
//  local_i2c_address is an array of addresses, n is an index into that array
float ltc2970_external_temp (uint8_t local_i2c_address)
{     
  uint16_t reg_vbe_low,
    reg_i_low,
    reg_vbe_high,
    reg_i_high;

  float voltage_low, 
    voltage_high, //voltage measurements at low and high DAC current
    current_low, 
    current_high, //current measurements at low and high DAC current
    // the current measurement is just the voltage across a 10k resistor
    current_ratio = 0.0,
    ln_current_ratio = 0.0,
    vbe_delta = 0.0;
  float temperature;

  // BJT properties
  //  static float n = 1.016;
  static float q_by_k = 11.60452e3;
  float q_by_n_k = (q_by_k / bjt_n[0]);
  
  //! Initialize the DAC
  smbus->writeWord(local_i2c_address, LTC2970_CH1_A_IDAC, 0x0700);
  
  //! Measure voltage and current at low current
  smbus->writeWord(local_i2c_address, LTC2970_CH1_A_IDAC, dac_current_low);
  delay(meas_delay);
  reg_i_low =
    (0x7FFF & smbus->readWord(local_i2c_address, LTC2970_CH1_A_ADC));
  reg_vbe_low =
    (0x7FFF & smbus->readWord(local_i2c_address, LTC2970_CH1_B_ADC));
  voltage_low = ((float)(reg_vbe_low & 0x7FFF))*500e-6;
  current_low = ((float)(reg_i_low & 0x7FFF))*500e-6;

  //! Measure voltage and current at high current
  smbus->writeWord(local_i2c_address, LTC2970_CH1_A_IDAC, dac_current_high);
  delay(meas_delay);
  reg_i_high =
    (0x7FFF & smbus->readWord(local_i2c_address, LTC2970_CH1_A_ADC));
  reg_vbe_high =
    (0x7FFF & smbus->readWord(local_i2c_address, LTC2970_CH1_B_ADC));
  voltage_high = ((float)(reg_vbe_high & 0x7FFF))*500e-6; 
  current_high = ((float)(reg_i_high & 0x7FFF))*500e-6; 
  
  //! Set DAC current to zero between measurements
  smbus->writeWord(local_i2c_address, LTC2970_CH1_A_IDAC, 0x0700);
  
  //! Temperature calculations
  vbe_delta = voltage_high - voltage_low;
  current_ratio = current_low / current_high;
  ln_current_ratio = log(current_ratio); // natural log

  temperature = 
    (vbe_delta * (-1 / ln_current_ratio) * q_by_n_k) - 273.15;
  
  return temperature;
}


//! return the kelvin temperature of an external BJT hooked-up to IDAC0 and VIN0B
//!  print a comma-separated list of values calculated here:
float ltc2970_print_external_temp_csv (uint8_t local_i2c_address)
{     
  uint16_t reg_vbe_low,
    reg_i_low,
    reg_vbe_high,
    reg_i_high;

  float voltage_low, 
    voltage_high, //voltage measurements at low and high DAC current
    current_low, 
    current_high, //current measurements at low and high DAC current
    // the current measurement is just the voltage across a 10k resistor
    current_ratio = 0.0,
    ln_current_ratio = 0.0,
    vbe_delta = 0.0;
  float temperature;

  // BJT properties
  //  static float n = 1.016;
  static float q_by_k = 11.60452e3;
  float q_by_n_k = (q_by_k / bjt_n[0]);
  
  //! Initialize the DAC
  smbus->writeWord(local_i2c_address, LTC2970_CH1_A_IDAC, 0x0700);
  
  //! Measure voltage and current at low current
  smbus->writeWord(local_i2c_address, LTC2970_CH1_A_IDAC, dac_current_low);
  delay(meas_delay);
  reg_i_low =
    (0x7FFF & smbus->readWord(local_i2c_address, LTC2970_CH1_A_ADC));
  reg_vbe_low =
    (0x7FFF & smbus->readWord(local_i2c_address, LTC2970_CH1_B_ADC));
  voltage_low = ((float)(reg_vbe_low & 0x7FFF))*500e-6;
  current_low = ((float)(reg_i_low & 0x7FFF))*500e-6;

  //! Measure voltage and current at high current
  smbus->writeWord(local_i2c_address, LTC2970_CH1_A_IDAC, dac_current_high);
  delay(meas_delay);
  reg_i_high =
    (0x7FFF & smbus->readWord(local_i2c_address, LTC2970_CH1_A_ADC));
  reg_vbe_high =
    (0x7FFF & smbus->readWord(local_i2c_address, LTC2970_CH1_B_ADC));
  voltage_high = ((float)(reg_vbe_high & 0x7FFF))*500e-6; 
  current_high = ((float)(reg_i_high & 0x7FFF))*500e-6; 
  
  //! Set DAC current to zero between measurements
  smbus->writeWord(local_i2c_address, LTC2970_CH1_A_IDAC, 0x0700);
  
  //! Temperature calculations
  vbe_delta = voltage_high - voltage_low;
  current_ratio = current_low / current_high;
  ln_current_ratio = log(current_ratio); // natural log

  temperature = 
    (vbe_delta * (-1 / ln_current_ratio) * q_by_n_k) - 273.15;
  
  // TEMPERATURE, LOW_VBE, LOW_I, HIGH_VBE, HIGH_I
  Serial.print(temperature);
  Serial.print(F(", "));
  Serial.print(voltage_low, 4);
  Serial.print(F(", "));
  Serial.print(current_low, 4);
  Serial.print(F(", "));
  Serial.print(voltage_high, 4);
  Serial.print(F(", "));
  Serial.print(current_high, 4);
  Serial.print(F("\n"));

  return temperature;
}

