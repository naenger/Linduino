/*
Linear Technology Alpine Demonstration Project
LTC3787, LTC2970: Power Management Solution for Alpine

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

#define LTC2970_I2C_ADDRESS 0x5B //global 7-bit address
//#define LTC2970_I2C_ADDRESS 0x6F //SLAVE_HH 7-bit address

/********************************************************************************/
//LTC2970 command address definitions

#define LTC2970_FAULT   0x00
#define LTC2970_FAULT_EN  0x08
#define LTC2970_FAULT_LA_INDEX  0x10
#define LTC2970_FAULT_LA  0x11

#define LTC2970_IO    0x17

#define LTC2970_ADC_MON   0x18

#define LTC2970_VDD_ADC   0x28
#define LTC2970_VDD_OV    0x29
#define LTC2970_VDD_UV    0x2A

#define LTC2970_V12_ADC   0x38
#define LTC2970_V12_OV    0x39
#define LTC2970_V12_UV    0x3A

#define LTC2970_CH0_A_ADC 0x40
#define LTC2970_CH0_A_OV  0x41
#define LTC2970_CH0_A_UV  0x42
#define LTC2970_CH0_A_SERVO 0x43
#define LTC2970_CH0_A_IDAC  0x44

#define LTC2970_CH0_B_ADC 0x48
#define LTC2970_CH0_B_OV  0x49
#define LTC2970_CH0_B_UV  0x4A

#define LTC2970_CH1_A_ADC 0x50
#define LTC2970_CH1_A_OV  0x51
#define LTC2970_CH1_A_UV  0x52
#define LTC2970_CH1_A_SERVO 0x53
#define LTC2970_CH1_A_IDAC  0x54

#define LTC2970_CH1_B_ADC 0x58
#define LTC2970_CH1_B_OV  0x59
#define LTC2970_CH1_B_UV  0x5A

#define LTC2970_TEMP_ADC  0x68

/****************************************************************************/
// Global variables
static uint8_t ltc2970_i2c_address;

//static LT_I2CBus *i2cbus = new LT_I2CBus();
//static LT_I2CBus *i2cbus = new LT_I2CBus();
static LT_SMBusNoPec *smbus = new LT_SMBusNoPec();

uint16_t dac_value, dac_ctrl;
uint16_t servo_value;
uint16_t idac_reg;

// definitions of information about the system
static float    dac_step_size = 0.190; // boost volts per DAC step
//static float    dac_v0 = 12.12;  //voltage reference point
//static uint16_t dac_code0 = 0x0049;  // code at reference point
static float    dac_v0 = 12.29;  //voltage reference point
static uint16_t dac_code0 = 0x004D;  // code at reference point
static uint16_t dac_max_code = 0x009B; // maximum allowed DAC code
static uint16_t dac_min_code = 0x004A; // minimum allowed DAC code

static float    adc_step_size = 0.003980; // boost volts per ADC step
static uint16_t adc_max_code = 0x18B7; // maximum allowed ADC reading
static uint16_t adc_min_code = 0x0AE0; // minimum allowed ADC reading


// structure to hold information about the voltage transition
struct voltage_transition
{
  uint8_t dac_code_start; // begin the transition at this code
  uint8_t step_unit; // number of DAC LSBs per step
  uint8_t step_direction; // 1 = step upward; 0 = step downward
  uint8_t num_steps; // total number of DAC steps in the transition
  uint8_t step_time; // pause this number of milliseconds between steps
  // final_voltage = (dac_code_start + (step_unit*num_steps))*dac_step_size
  // transition time = (step_time*num_steps)
} v_trans;

/****************************************************************************/
//! Initialize Linduino
void setup()
{
  uint16_t return_val;

  // initialize the i2c port
  //  i2c_enable();

  Serial.begin(115200);         //! Initialize the serial port to the PC
  print_title();
  print_prompt();

  ltc2970_i2c_address = LTC2970_I2C_ADDRESS;

  dac_value = 0x0087;
  dac_ctrl = 0x0300;
  idac_reg = dac_ctrl + dac_value;

  servo_value = 0x11CB; // 18v

  init_voltage_transition();
}

void loop()
{
  uint8_t user_command;

  uint16_t dac_value_start, dac_value_end;

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
        Serial.print(F("\n****SERVO LTC2970 DAC TO 12v****\n"));
        enable_boost_converter();
        dac_value = 0x0049;
        hard_connect_dac();
        //    servo_to_adc_val(0x178C); // 24v
        //    servo_to_adc_val(0x11CB); // 18v
        servo_to_adc_val(0x0ADF); // 12v
        break;

      case 3 :
        Serial.print(F("\n****SERVO LTC2970 DAC TO 24v****\n"));
        enable_boost_converter();
        dac_value = 0x0049;
        hard_connect_dac();
        servo_to_adc_val(0x178C); // 24v
        // servo_to_adc_val(0x11CB); // 18v
        //servo_to_adc_val(0x0ADF); // 12v
        break;

      case 4:
        if (get_user_step_values() != 0)
        {
          Serial.print(F("\n*ABORTING*\n"));
          break; // only break if there was an error
        }
      case 5:
        // same as above, but don't ask for inputs, just use the same values
        dac_value = (uint16_t)v_trans.dac_code_start;
        enable_boost_converter();
        set_dac_code(dac_value);
        hard_connect_dac();
        Serial.print(F("\nHIT ENTER TO BEGIN\n"));
        read_int();
        for ( i = 0; i < v_trans.num_steps; i++)
        {
          delay(1.0*(v_trans.step_time-1));
          if (v_trans.step_direction)
          {
            dac_value += (v_trans.step_unit);
            if (dac_value > dac_max_code)
            {
              dac_value = dac_max_code;
            }
          }
          else
          {
            dac_value -= (v_trans.step_unit);
            if (dac_value < dac_min_code)
            {
              dac_value = dac_min_code;
            }
          }
          set_dac_code(dac_value);
        }
        break;

      case 6 :
        Serial.print(F("\n****SERVO LTC2970 DAC DOWN****\n"));
        servo_value = servo_value - 0x007E; // -0.5v
        servo_to_adc_val(servo_value);
        break;

      case 7 :
        Serial.print(F("\n****SERVO LTC2970 DAC UP****\n"));
        servo_value = servo_value + 0x007E; // +0.5v
        servo_to_adc_val(servo_value);
        break;

      case 8 :
        Serial.print(F("\n****DISCONNECT LTC2970 DAC****\n"));
        disconnect_dac();
        Serial.print(F("\n****DISABLE LTC3787 BOOST CONVERTER****\n"));
        disable_boost_converter();
        break;

      case 9:
        Serial.print(F("\n****READ VOUT BOOST VOLTAGE****\n"));
        print_boost_voltage();
        Serial.println();
        break;

      case 10:
        Serial.print(F("\n****READ BOOST INDUCTOR CURRENTS****\n"));
        print_boost_currents();
        Serial.println();
        break;

      case 11:
        Serial.print(F("\n****READ TEMPERATURES****\n"));
        ltc2970_print_die_temp();
        //print_temp_sensor();
        break;

      case 12:
        Serial.print(F("\n****READ ALL REGISTERS****\n"));
        ltc2970_print_all_registers();
        break;

      case 13:
        Serial.print(F("\n****STEP DAC TO 12v****\n"));
        set_dac_code_12v();
        break;

      case 14:
        Serial.print(F("\n****STEP DAC TO 24v****\n"));
        set_dac_code_24v();
        break;

      case 15:
        Serial.print(F("\n****CLEAR FAULTS, READ FAULT_LA REGISTER****\n"));
        ltc2970_clear_faults();
        break;

      case 16:
        test_dac_steps();
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
  Serial.print(F("\n*******************************************************\n"));
  Serial.print(F("* Alpine Boost Regulator Control Program                                 *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("* This program provides a simple interface to control the *\n"));
  Serial.print(F("* the LTC3787 through the LTC2970                         *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("* Set the baud rate to 115200 and select the newline terminator.*\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("*********************************************************\n"));
}

//! Prints main menu.
void print_prompt()
{
  Serial.print(F("\n"));
  Serial.print(F("  1  - Reset the LTC2970, Disable Boost Converter\n"));
  Serial.print(F("  2  - Servo Boost Voltage to 12v\n"));
  Serial.print(F("  3  - Servo Boost Voltage to 24v\n"));
  Serial.print(F("  4  - Custom Voltage Transition - Enter Parameters\n"));
  Serial.print(F("  5  - Custom Voltage Transition - Use Same Parameters\n"));
  Serial.print(F("  6  - Decrement Boost Voltage Down\n"));
  Serial.print(F("  7  - Increment Boost Voltage Up\n"));
  Serial.print(F("  8  - Disconnect DAC and Disable Boost Converter\n"));
  Serial.print(F("  9  - Read Voltages\n"));
  Serial.print(F("  10 - Read Currents\n"));
  Serial.print(F("  11 - Read Temperatures\n"));
  Serial.print(F("  12 - Read All Registers\n"));
  Serial.print(F("  13 - Step Boost Voltage to 12v (no servo)\n"));
  Serial.print(F("  14 - Step Boost Voltage to 24v (no servo)\n"));
  Serial.print(F("  15 - Read FAULT_LA Register and Clear Faults\n"));
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

///!a test function to step through DAC codes
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

//! Print the FAULT_LA register contents and clear faults on the LTC2970
void ltc2970_clear_faults ()
{
  uint16_t return_val;

  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_FAULT_LA);
  Serial.print(F("\n LTC2970_FAULT_LA: "));
  Serial.println(return_val, HEX);
}

//! Prints all register contents on the LTC2970
void ltc2970_print_all_registers ()
{
  uint16_t return_val;

  //LTC2970_FAULT
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_FAULT);
  Serial.print(F("\n LTC2970_FAULT: "));
  Serial.println(return_val, HEX);


  //LTC2970_FAULT_EN
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_FAULT_EN);
  Serial.print(F("\n LTC2970_FAULT_EN: "));
  Serial.println(return_val, HEX);

  //LTC2970_FAULT_LA_INDEX
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_FAULT_LA_INDEX);
  Serial.print(F("\n LTC2970_FAULT_LA_INDEX: "));
  Serial.println(return_val, HEX);

  //LTC2970_FAULT_LA
  // NOTE: reading this register clears faults
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_FAULT_LA);
  Serial.print(F("\n LTC2970_FAULT_LA: "));
  Serial.println(return_val, HEX);

  //LTC2970_IO
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_IO);
  Serial.print(F("\n LTC2970_IO: "));
  Serial.println(return_val, HEX);

  //LTC2970_ADC_MON
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_ADC_MON);
  Serial.print(F("\n LTC2970_ADC_MON: "));
  Serial.println(return_val, HEX);

  //LTC2970_VDD_ADC
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_VDD_ADC);
  Serial.print(F("\n LTC2970_VDD_ADC: "));
  Serial.println(return_val, HEX);

  //LTC2970_VDD_OV
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_VDD_OV);
  Serial.print(F("\n LTC2970_VDD_OV: "));
  Serial.println(return_val, HEX);

  //LTC2970_VDD_UV
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_VDD_UV);
  Serial.print(F("\n LTC2970_VDD_UV: "));
  Serial.println(return_val, HEX);

  //LTC2970_V12_ADC
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_V12_ADC);
  Serial.print(F("\n LTC2970_V12_ADC: "));
  Serial.println(return_val, HEX);

  //LTC2970_V12_OV
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_V12_OV);
  Serial.print(F("\n LTC2970_V12_OV: "));
  Serial.println(return_val, HEX);

//LTC2970_V12_UV
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_V12_UV);
  Serial.print(F("\n LTC2970_V12_UV: "));
  Serial.println(return_val, HEX);

  //LTC2970_CH0_A_ADC
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_CH0_A_ADC);
  Serial.print(F("\n LTC2970_CH0_A_ADC: "));
  Serial.println(return_val, HEX);

  //LTC2970_CH0_A_OV
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_CH0_A_OV);
  Serial.print(F("\n LTC2970_CH0_A_OV: "));
  Serial.println(return_val, HEX);

  //LTC2970_CH0_A_UV
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_CH0_A_UV);
  Serial.print(F("\n LTC2970_CH0_A_UV: "));
  Serial.println(return_val, HEX);

  //LTC2970_CH0_A_SERVO
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_CH0_A_SERVO);
  Serial.print(F("\n LTC2970_CH0_A_SERVO: "));
  Serial.println(return_val, HEX);

  //LTC2970_CH0_A_IDAC
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_CH0_A_IDAC);
  Serial.print(F("\n LTC2970_CH0_A_IDAC: "));
  Serial.println(return_val, HEX);

  //LTC2970_CH0_B_ADC
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_CH0_B_ADC);
  Serial.print(F("\n LTC2970_CH0_B_ADC: "));
  Serial.println(return_val, HEX);

  //LTC2970_CH0_B_OV
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_CH0_B_OV);
  Serial.print(F("\n LTC2970_CH0_B_OV: "));
  Serial.println(return_val, HEX);

  //LTC2970_CH0_B_UV
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_CH0_B_UV);
  Serial.print(F("\n LTC2970_CH0_B_UV: "));
  Serial.println(return_val, HEX);

  //LTC2970_CH1_A_ADC
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_CH1_A_ADC);
  Serial.print(F("\n LTC2970_CH1_A_ADC: "));
  Serial.println(return_val, HEX);

  //LTC2970_CH1_A_OV
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_CH1_A_OV);
  Serial.print(F("\n LTC2970_CH1_A_OV: "));
  Serial.println(return_val, HEX);

  //LTC2970_CH1_A_UV
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_CH1_A_UV);
  Serial.print(F("\n LTC2970_CH1_A_UV: "));
  Serial.println(return_val, HEX);

  //LTC2970_CH1_A_SERVO
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_CH1_A_SERVO);
  Serial.print(F("\n LTC2970_CH1_A_SERVO: "));
  Serial.println(return_val, HEX);

  //LTC2970_CH1_A_IDAC
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_CH1_A_IDAC);
  Serial.print(F("\n LTC2970_CH1_A_IDAC: "));
  Serial.println(return_val, HEX);

  //LTC2970_CH1_B_ADC
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_CH1_B_ADC);
  Serial.print(F("\n LTC2970_CH1_B_ADC: "));
  Serial.println(return_val, HEX);

  //LTC2970_CH1_B_OV
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_CH1_B_OV);
  Serial.print(F("\n LTC2970_CH1_B_OV: "));
  Serial.println(return_val, HEX);

  //LTC2970_CH1_B_UV
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_CH1_B_UV);
  Serial.print(F("\n LTC2970_CH1_B_UV: "));
  Serial.println(return_val, HEX);

  //LTC2970_TEMP_ADC
  return_val =
    smbus->readWord(ltc2970_i2c_address,LTC2970_TEMP_ADC);
  Serial.print(F("\n LTC2970_TEMP_ADC: "));
  Serial.println(return_val, HEX);

  Serial.print(F("\n"));

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
}

//! Prints currents as measured in the inductors on the DC1411A
void print_boost_currents()
{
  static float current_scale = 200;
  static float current_offset = 0;
  float current0, current1;
  uint16_t return_val_0, return_val_1;

  return_val_0 =
    smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_B_ADC);
  return_val_0 = return_val_0 & 0x7FFF; // get rid of bit 15
  return_val_1 =
    smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_B_ADC);
  return_val_1 = return_val_1 & 0x7FFF; // get rid of bit 15

  //convert the returned DAC code to a current
  current0 = ((float(return_val_0) - current_offset) / current_scale);
  current1 = ((float(return_val_1) - current_offset)  / current_scale);

  // print results
  Serial.print(F("I_L0 (amps) "));
  Serial.println(current0, DEC);
  Serial.print(F("I_L1 (amps) "));
  Serial.println(current1, DEC);
  Serial.print(F("TOTAL = IOUT0 + IOUT1 = "));
  Serial.println((current0+current1), DEC);
}


//! Prints voltages on the DC1411A board
void print_boost_voltage()
{
  //static float voltage_scale = 253.1;
  static float voltage_scale = 251.2;
  static float voltage_offset = 0;
  float voltage0;
  uint16_t return_val_0;

  return_val_0 =
    smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_A_ADC);
  return_val_0 = return_val_0 & 0x7FFF; // get rid of bit 15

  // convert the returned DAC code to a voltage measurement
  voltage0 = ((float(return_val_0) - voltage_offset) / voltage_scale);
  //  voltage0 = ((float(return_val_0) - voltage_offset) * adc_step_size);

  // print results
  Serial.print(F("VBOOST "));
  Serial.println(voltage0, DEC);
}

//! Print temperature reading from DCLTC2997
void print_temp_sensor()
{
  static float temp_scale = 2;
  static float temp_offset = 2356;
  float temperature;
  uint16_t return_val_0;

  return_val_0 =
    smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_ADC);
  return_val_0 = return_val_0 & 0x7FFF; // get rid of bit 15

  // convert the returned DAC code to a voltage measurement
  temperature = (float(return_val_0) - temp_offset) / temp_scale;

  // print results
  Serial.print(F(" INDUCTOR TEMPERATURE "));
  Serial.println(temperature, DEC);
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

//! Gracefully connect the DAC without disturbing the control node
void soft_connect_dac()
{
  uint16_t return_val_0;
  uint16_t con_en_bits;

  // use the global DAC variables
  dac_value = 0x0087;
  dac_ctrl = 0x0100;
  idac_reg = dac_ctrl + dac_value;

  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, idac_reg);

  delay(1000); // not sure how long a soft-connect takes

  return_val_0 =
    smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC);

  // retrieve the DAC value
  dac_value = (0x00FF & return_val_0);
  con_en_bits = (0x0300 & return_val_0);

  if (con_en_bits == 0x0300)
  {
    Serial.print(F("DAC connected successfully\n"));
    Serial.print(F("IDAC code = "));
    Serial.println(dac_value, HEX);
  }
  else
  {
    Serial.print(F("DAC failed to connect."));
  }
}

//! Disconnect the DAC from the feedback node of the boost
void disconnect_dac()
{
  // use the global DAC variables
  //  dac_value = 0x0087;
  dac_ctrl = 0x0000;
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
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_SERVO,  0x0BDF);

  // set DAC code
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, idac_reg);

}


//!set boost voltage to 12v (near VBATT)
void set_dac_code_12v()
{
  // use the global DAC variables
  dac_value = 0x0049;
  //dac_ctrl = 0x0000;
  idac_reg = dac_ctrl + dac_value;

  // make sure servoing is disabled
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_SERVO,  0x0BDF);

  // set DAC code
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, idac_reg);

}

//! set boost voltage to mid-range (near un-controlled boost voltage)
void set_dac_code_24v()
{
  // use the global DAC variables
  dac_value = 0x0087;
  //dac_ctrl = 0x0000;
  idac_reg = dac_ctrl + dac_value;

  // make sure servoing is disabled
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_SERVO,  0x0BDF);

  // set DAC code
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, idac_reg);

}



//! Increase DAC code by 3
void increase_boost_voltage()
{
  // use the global DAC variables
  dac_value = dac_value + 0x0003;
  if (dac_value > 0x00FF)
  {
    dac_value = 0x00FF;
    Serial.print(F("****DAC AT UPPER LIMIT**** "));
  }
  idac_reg = dac_ctrl + dac_value;

  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, idac_reg);

  delay(1000);

  print_boost_voltage();

}


//! Decrease DAC code by 3
void decrease_boost_voltage()
{
  // use the global DAC variables
  if (dac_value > 0x0004)
  {
    dac_value = dac_value - 0x0003;
  }
  else
  {
    dac_value = 0x0000;
    Serial.print(F("****DAC AT LOWER LIMIT**** "));
  }

  //dac_ctrl = 0x0000;
  idac_reg = dac_ctrl + dac_value;

  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, idac_reg);

  delay(1000);

  print_boost_voltage();

}

//! Servo once to a given ADC value
void servo_to_adc_val(uint16_t code)
{
  uint16_t code_in;
  uint16_t code_max = 0x18B7,
           code_min = 0x0AE0;

  // get rid of bit 15
  code_in = (code & 0x7FFF);

  code_in = (code_in > code_max) ? code_max : code_in;
  code_in = (code_in < code_min) ? code_min : code_in;
  servo_value = code_in;

  // ensure that bit 15 is high to enable servoing
  code_in = code_in + 0x8000;

  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_SERVO, code_in);
}

//! Set GPIO_0 and GPIO_1 high
void enable_boost_converter()
{
  smbus->writeWord(ltc2970_i2c_address, LTC2970_IO, 0x003A);
}

//! Set GPIO_0 and GPIO_1 low
void disable_boost_converter()
{
  smbus->writeWord(ltc2970_i2c_address, LTC2970_IO, 0x000A);
}


