/*
Linear Technology 
LTC2975 + LTC3765/66 Isolated Forward Converter + Down-stream regulators
Use LTC2945 to measure up-stream V and I

Search for the most efficient operating voltage at a given load current
Take multiple samples to average-out noise (there is a lot of noise)
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
#define SAMPLE_DELAY 5000


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

  //  uint8_t *addresses = NULL;
  //  addresses = smbus->probe(0);
  //  while(*addresses != 0)
  //    {
  //      Serial.print(F("ADDR 0x"));
  //      Serial.println(*addresses++, HEX);
  //    }
  // the above probe should find the following:
  // ADDRESS    |   DESCRIPTION
  //-------------------------------------------
  // 0x30       | LTM4676 rail address (three LTM4676 parts)
  // 0x41       | LTM4676 part address
  // 0x42       | LTM4676 part address
  // 0x43       | LTM4676 part address
  // 0x50       | DC2382A demo board ID EEPROM (LTC2975)
  // 0x5A       | LTM4676 global address (page all)
  // 0x5B       | LTC2975 global address & LTM4676 global address (page active)
  // 0x5C       | LTC2975 primary address
  // 0x66       | LTC2945 global address
  // 0x6F       | LTC2945 primary address
  //////////////////////////////////////////////

  
  ltc2975_i2c_address = LTC2975_I2C_ADDRESS;
  ltc2945_i2c_address = LTC2945_I2C_ADDRESS;
  ltm4676_m1_address = LTM4676_M1_ADDRESS;
  ltm4676_m2_address = LTM4676_M2_ADDRESS;
  ltm4676_m3_address = LTM4676_M3_ADDRESS;

  pages[0] = 0x00;
  pages[1] = 0x01;

}
uint8_t buttons;

void loop ()
{
  
  if (state == SM_WAITING) {
    lcd.setCursor(0,0);
    lcd.print("PRESS \"SELECT\"  ");
    lcd.setCursor(0,1);
    lcd.print("TO BEGIN SEARCH ");
    // waiting for a button press
    buttons = lcd.readButtons(); //! read the buttons on the LDC shield
    if (buttons & BUTTON_SELECT)
      state = SM_SEARCHING;
    else
      state = SM_WAITING;
  }
  else if (state == SM_SEARCHING) {
    // vid voltages are what the LTC2975 commands (including the divided-down 12V rail)
    float vid_value = 0.0,
      ibc_cmd_voltage = 0.0,
      ibc_step = 0.0,  // voltage step size
      ibc_mi_voltage = 0.0,
      ibc_mx_voltage = 0.0;
    float ibc_eff_voltage = 0.0;
    
    // vin is the 40V supply to the forward converter (measured by LTC2945)
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

    float ibc_to_vid = 3.03; //! ratio of IBC to VID voltage

    uint16_t ltc2945_vin_code, ltc2945_iin_code;
  
    int i = 0;
    int j = 0;
    int sample_delay = SAMPLE_DELAY;
    const int count = 16; // number of samples for averaging
    
    
    pmbus->setPage(ltc2975_i2c_address, 0x00);                // Set to page 0 (servo IBC channel)
    pmbus->setPage(ltm4676_m1_address, 0x00);                // Set to page 0
    pmbus->setPage(ltm4676_m2_address, 0x00);                // Set to page 0
    pmbus->setPage(ltm4676_m3_address, 0x00);                // Set to page 0
    
    j = 0;
    ibc_mi_voltage = IBC_MIN_VOLTAGE;
    ibc_mx_voltage = IBC_MAX_VOLTAGE;
    ibc_step = (ibc_mx_voltage - ibc_mi_voltage) / 8;
    
    lcd.setCursor(0,0);
    lcd.print(F("RUNNING SEARCH.."));
    
    while (j < 2) {
      // run this loop with 2 different step sizes and limits (coarse and fine)
      // noise may limit the effectiveness of the fine sweep
      for (ibc_cmd_voltage = ibc_mi_voltage; ibc_cmd_voltage < ibc_mx_voltage; ibc_cmd_voltage += ibc_step) {
	
	lcd.setCursor(0,1);
	lcd.print(F("VOLTAGE: ")); 
	lcd.println(ibc_cmd_voltage, DEC); 
	lcd.setCursor(14,1);
	lcd.print(F("   ")); // clear unnecessary digits from the LCD
	
	vid_value = ibc_cmd_voltage / ibc_to_vid; // IBC voltage is divide by 3 before sensing
	pmbus->setVout(ltc2975_i2c_address, vid_value);           // Set starting voltage
	delay(sample_delay/2);
	// blink colons on display as a heartbeat
	lcd.setCursor(0,1);
	lcd.print(F("VOLTAGE ")); 
	delay(sample_delay/2);
	
	ibc_voltage = pmbus->readVout(ltc2975_i2c_address, false); // divided-down voltage (+12V / ibc_to_vid)
	
	// take AVERAGE current and voltage at the LTC2945 on the 48V main bus and on the LTC2975 +12V bus
	// also take AVERAGE current and voltage at the LTM4676 on the 3.3V output rail
	// Note that there are 3 LTM4676 modules on the board, and each module has 2 phases
	vin_current = 0.0;
	vin_voltage = 0.0;
	ibc_current = 0.0;
	ibc_voltage = 0.0;
	power_ibc = 0.0;
	int_current = 0.0;
	out_current = 0.0;
	out_voltage = 0.0;
	power_out = 0.0;

	for (i = 0; i < count; i++) {
	  LTC2945_read_12_bits(ltc2945_i2c_address, LTC2945_DELTA_SENSE_MSB_REG, &ltc2945_iin_code);
	  LTC2945_read_12_bits(ltc2945_i2c_address, LTC2945_VIN_MSB_REG, &ltc2945_vin_code);
	  vin_current += LTC2945_code_to_current(ltc2945_iin_code, 0.020, LTC2945_DELTA_SENSE_lsb); // (code value, sense resistor, lsb value)
	  vin_voltage += LTC2945_VIN_code_to_voltage(ltc2945_vin_code, LTC2945_VIN_lsb);

	  ibc_current += pmbus->readIin(ltc2975_i2c_address, false);
	  ibc_voltage += pmbus->readVin(ltc2975_i2c_address, false);
	  power_ibc += pmbus->readPin(ltc2975_i2c_address, false);

	  out_current += pmbus->readIout(ltm4676_m1_address, false);
	  out_current += pmbus->readIout(ltm4676_m2_address, false);
	  out_current += pmbus->readIout(ltm4676_m3_address, false);
	  power_out += pmbus->readPout(ltm4676_m1_address, false);
	  power_out += pmbus->readPout(ltm4676_m2_address, false);
	  power_out += pmbus->readPout(ltm4676_m3_address, false);
	  pmbus->setPage(ltm4676_m1_address, 0x01);
	  pmbus->setPage(ltm4676_m2_address, 0x01);
	  pmbus->setPage(ltm4676_m3_address, 0x01);
	  out_current += pmbus->readIout(ltm4676_m1_address, false);
	  out_current += pmbus->readIout(ltm4676_m2_address, false);
	  out_current += pmbus->readIout(ltm4676_m3_address, false);
	  power_out += pmbus->readPout(ltm4676_m1_address, false);
	  power_out += pmbus->readPout(ltm4676_m2_address, false);
	  power_out += pmbus->readPout(ltm4676_m3_address, false);      
	  pmbus->setPage(ltm4676_m1_address, 0x00);
	  pmbus->setPage(ltm4676_m2_address, 0x00);
	  pmbus->setPage(ltm4676_m3_address, 0x00);
	  delay(random(1,10));
	  delayMicroseconds(random(1,1000));
	}

	vin_current = vin_current / count; // calculate average
	vin_voltage = vin_voltage / count; // calculate average
	
	// calculated input power (V*I)
	// this could be done with one read of the LTC2945 power register instead
	power_in = vin_voltage * vin_current;
	
	ibc_current = ibc_current / count; // calculate average
	ibc_voltage = ibc_voltage / count; // calculate average
	power_ibc = power_ibc / count;
	
	out_voltage = pmbus->readVout(ltm4676_m1_address, false); // all outputs are tied together, so pick one
	int_current += pmbus->readIin(ltm4676_m1_address, false);
	int_current += pmbus->readIin(ltm4676_m2_address, false);
	int_current += pmbus->readIin(ltm4676_m3_address, false);
	out_current = out_current / count; // calculate average
	power_out = power_out / count;
	
	if (power_in > 0.001) {
	  eff_forward = power_ibc / power_in;
	  eff_total = power_out / power_in;
	}
	else {
	  eff_forward = 0.0;
	  eff_total = 0.0;
	}
	
	if (eff_total > eff_max) {
	  eff_max = eff_total;
	  ibc_eff_voltage = ibc_voltage;
	}
      } // for()
      // upon exiting the for loop we have the voltage at which efficiency was maximized
      // to within the coarse step size
      // now zoom-in with a finer step size and tighter limits
      j++;
      ibc_mi_voltage = ibc_eff_voltage - ibc_step;
      ibc_mi_voltage = (ibc_mi_voltage < IBC_MIN_VOLTAGE) ? IBC_MIN_VOLTAGE : ibc_mi_voltage;
      ibc_mx_voltage = ibc_eff_voltage + (ibc_step);
      ibc_mx_voltage = (ibc_mx_voltage > IBC_MAX_VOLTAGE) ? IBC_MAX_VOLTAGE : ibc_mx_voltage;
      ibc_step = ibc_step / 2;
      sample_delay = sample_delay;
    } // while()
    // upon exiting the while loop we have the voltage at which efficiency is maximized
    // to within the fine step size (coarse step / 8)
    
    // set the operating voltage to the most efficient point
    vid_value = ibc_eff_voltage / ibc_to_vid;
    pmbus->setVout(ltc2975_i2c_address, vid_value);
    delay (1000);

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("SEARCH DONE.    ")); 
    lcd.setCursor(0,1);
    lcd.print(F("                ")); 
    delay(1000);
    lcd.setCursor(0,0);
    lcd.print(F("MAX EFF  AT     ")); 
    lcd.setCursor(0,1);
    lcd.print(F("V = ")); 
    lcd.println(ibc_eff_voltage, DEC);
    lcd.setCursor(10,1);
    lcd.print(F("      ")); 
    delay(10000);

    //    read_int();
    state = SM_WAITING;

  } // SM_SEARCHING
  else {
    state = SM_WAITING;
  }
} 

////////////////////////////////////////////////////////////////////
// Function Definitions


//! Initialize the LTC2945 registers
void LTC2945_configure()
{
  //all defaults seem sane...do nothing
}

