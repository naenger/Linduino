/*
Linear Technology 
LTC2975 + LTC3765/66 Isolated Forward Converter + Down-stream regulators
Use LTC2945 to measure up-stream V and I

Search for the most efficient operating voltage at a given load current
Report efficiency to the LCD screen
Respond to buttons on the LCD shield

@verbatim

NOTES
  Setup:
   Set the terminal baud rate to 115200 and select the newline terminator.

@endverbatim

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

#include "LT_SMBusPec.h"
#include "LT_PMBus.h"
#include "LT_PMBusMath.h"

#include "LTC2945.h"

#include <Adafruit_RGBLCDShield.h>
//#include <utility/Adafruit_MCP23017.h>
#define LCD_ON 0x07
#define LCD_OFF 0x00


#define LTC2975_I2C_ADDRESS 0x5C
#define LTM4676_M1_ADDRESS 0x41
#define LTM4676_M2_ADDRESS 0x42
#define LTM4676_M3_ADDRESS 0x43


// System Parameters
#define IBC_MIN_VOLTAGE 8.4
#define IBC_MAX_VOLTAGE 14.0
#define SYS_MIN_VOLTAGE 2.7
#define SYS_MAX_VOLTAGE 4.96
#define SAMPLE_DELAY 1500


// LTC2945 LSB Weights
const float LTC2945_ADIN_lsb = 5.001221E-04;                //!< Typical ADIN lsb weight in volts
const float LTC2945_DELTA_SENSE_lsb = 2.5006105E-05;        //!< Typical Delta lsb weight in volts
const float LTC2945_VIN_lsb = 2.5006105E-02;                //!< Typical VIN lsb weight in volts
const float LTC2945_Power_lsb = 6.25305E-07;                //!< Typical POWER lsb weight in V^2
const float LTC2945_ADIN_DELTA_SENSE_lsb = 1.25061E-08;     //!< Typical sense lsb weight in V^2  *ADIN_lsb * DELTA_SENSE_lsb

// Global variables
static uint8_t ltc2975_i2c_address, ltc2945_i2c_address, ltm4676_rail_address, ltm4676_m1_address, ltm4676_m2_address, ltm4676_m3_address;
static LT_PMBusMath *math = new LT_PMBusMath();
static LT_SMBus *smbus = new LT_SMBusPec();
static LT_PMBus *pmbus = new LT_PMBus(smbus);
static uint8_t pages[2];

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

static int state; // state machine state
#define SM_WAITING 0
#define SM_SEARCHING 1
#define SM_DONE 2

//! Initialize Linduino
void setup()
{
  state = SM_WAITING; //! Initialize the state machine
  
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  // fire it up
  lcd.setBacklight(LCD_ON);
  lcd.clear();
  lcd.print(F("VOLTAGE: 0.0")); 
  lcd.setCursor(0,1);
  lcd.print(F("CURRENT: 0.0")); 
  
  ltc2975_i2c_address = LTC2975_I2C_ADDRESS;
  ltc2945_i2c_address = LTC2945_I2C_ADDRESS;
  ltm4676_m1_address = LTM4676_M1_ADDRESS;
  ltm4676_m2_address = LTM4676_M2_ADDRESS;
  ltm4676_m3_address = LTM4676_M3_ADDRESS;

  pages[0] = 0x00;
  pages[1] = 0x01;

}
uint8_t buttons;

// vid voltages are what the LTC2975 commands (including the divided-down 12V rail)
float vid_value = 0.0,
  ibc_step = 0.0,  // voltage step size
  ibc_mi_voltage = 0.0,
  ibc_mx_voltage = 0.0;
float ibc_eff_voltage = 0.0;
float ibc_cmd_voltage = IBC_MIN_VOLTAGE;

int i = 0;

void loop ()
{
  
  // vin is the 48V supply to the forward converter (measured by LTC2945)
  float vin_voltage = 0.0,
    vin_current = 0.0;
  float power_in = 0.0;
  
  // ibc is the 12V output of the forward converter (measured by the LTC2975)
  float ibc_voltage = 0.0,
    ibc_current = 0.0;
  float power_ibc = 0.0;
  
  // out is the 3.3V output rail (controlled by the LTM4676)
  float out_voltage = 0.0,
    out_current = 0.0;
  float power_out = 0.0;
  
  float int_current; //LTM4676 input current (should be the same as ibc_current)
  
  // efficiency variables
  float eff_forward = 0.0;
  float eff_ltm = 0.0;
  float eff_total = 0.0;
  float eff_max = 0.0;
  
  float ibc_to_vid = 3.063; //! ratio of IBC to VID voltage
  
  uint16_t ltc2945_vin_code, ltc2945_iin_code;
  
  int sample_delay = SAMPLE_DELAY;
  const int count = 8; // for averaging
    
    
  //  pmbus->setPage(ltc2975_i2c_address, 0x00);                // Set to page 0 (servo IBC channel)
  //  pmbus->setPage(ltm4676_m1_address, 0x00);                // Set to page 0
  //  pmbus->setPage(ltm4676_m2_address, 0x00);                // Set to page 0
  //  pmbus->setPage(ltm4676_m3_address, 0x00);                // Set to page 0
  
  ibc_mi_voltage = IBC_MIN_VOLTAGE;
  ibc_mx_voltage = IBC_MAX_VOLTAGE;
  ibc_step = (ibc_mx_voltage - ibc_mi_voltage) / 8;  
  
  if (++i > count) {
    i = 0;
    pmbus->setVout(ltc2975_i2c_address, vid_value);           // Set starting voltage
    //increment the command voltage
    ibc_cmd_voltage += ibc_step;
    ibc_cmd_voltage = (ibc_cmd_voltage > ibc_mx_voltage) ? ibc_mi_voltage : ibc_cmd_voltage;
    vid_value = ibc_cmd_voltage / ibc_to_vid; // IBC voltage is divide by 3 before sensing
  }
  
  delay(sample_delay);
  
  ibc_voltage = pmbus->readVout(ltc2975_i2c_address, false); // divided-down voltage (+12V / ibc_to_vid)
  
  // current and voltage at the LTC2945 on the 48V main bus
  vin_current = 0.0;
  vin_voltage = 0.0;
  LTC2945_read_12_bits(ltc2945_i2c_address, LTC2945_DELTA_SENSE_MSB_REG, &ltc2945_iin_code);
  LTC2945_read_12_bits(ltc2945_i2c_address, LTC2945_VIN_MSB_REG, &ltc2945_vin_code);
  vin_current += LTC2945_code_to_current(ltc2945_iin_code, 0.0048, LTC2945_DELTA_SENSE_lsb); // (code value, sense resistor, lsb value)
  vin_voltage += LTC2945_VIN_code_to_voltage(ltc2945_vin_code, LTC2945_VIN_lsb);
  power_in = vin_voltage * vin_current;
  
  lcd.setCursor(0,0);
  lcd.print(F("VOLTAGE ")); 
  lcd.println(vin_voltage, DEC); 
  lcd.setCursor(0,1);
  lcd.print(F("CURRENT ")); 
  lcd.println(vin_current, DEC); 
  delay(2);
  // blink colons for an active display
  lcd.setCursor(0,0);
  lcd.print(F("VOLTAGE: ")); 
  lcd.println(vin_voltage, DEC); 
  lcd.setCursor(0,1);
  lcd.print(F("CURRENT: ")); 
  lcd.println(vin_current, DEC); 
  
}

////////////////////////////////////////////////////////////////////
// Function Definitions


//! Initialize the LTC2945 registers
void LTC2945_configure()
{
  //all defaults seem sane...do nothing
}

