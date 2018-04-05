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
#define IBC_MIN_VOLTAGE 8.2
#define IBC_MAX_VOLTAGE 14.9
#define SYS_MIN_VOLTAGE 2.7
#define SYS_MAX_VOLTAGE 4.96
#define SAMPLE_DELAY 800


// LTC2945 LSB Weights
const float LTC2945_ADIN_lsb = 5.001221E-04;                //!< Typical ADIN lsb weight in volts
const float LTC2945_DELTA_SENSE_lsb = 2.5006105E-05;        //!< Typical Delta lsb weight in volts
const float LTC2945_VIN_lsb = 2.5006105E-02;                //!< Typical VIN lsb weight in volts
const float LTC2945_Power_lsb = 6.25305E-07;                //!< Typical POWER lsb weight in V^2
const float LTC2945_ADIN_DELTA_SENSE_lsb = 1.25061E-08;     //!< Typical sense lsb weight in V^2  *ADIN_lsb * DELTA_SENSE_lsb

// Global variables
static uint8_t ltc2975_i2c_address, ltc2945_i2c_address, ltm4676_m1_address, ltm4676_m2_address, ltm4676_m3_address;
static LT_PMBusMath *math = new LT_PMBusMath();
static LT_SMBus *smbus = new LT_SMBusPec();
static LT_PMBus *pmbus = new LT_PMBus(smbus);
static uint8_t pages[2];

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

static int state; // state machine state
#define SM_WAITING 0
#define SM_BUTTON_SEL 1
#define SM_BUTTON_UP 2
#define SM_BUTTON_DOWN 3
#define SM_BUTTON_LEFT 4
#define SM_BUTTON_RIGHT 5
#define SM_BUTTON_RELEASE 6

uint8_t buttons;

//! Initialize Linduino
void setup()
{
  //  Serial.begin(115200);         //! Initialize the serial port to the PC

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

  pmbus->setPage(ltc2975_i2c_address, 0x00);                // Set to page 0 (servo IBC channel)
  pmbus->setPage(ltm4676_m1_address, 0x00);                // Set to page 0
  pmbus->setPage(ltm4676_m2_address, 0x00);                // Set to page 0
  pmbus->setPage(ltm4676_m3_address, 0x00);                // Set to page 0
  
}

// vid voltages are what the LTC2975 commands (including the divided-down 12V rail)
// ibc voltages are actual regulated voltage
float vid_value = 0.0,
  ibc_cmd_voltage = IBC_MIN_VOLTAGE,
  ibc_mi_voltage = IBC_MIN_VOLTAGE,
  ibc_mx_voltage = IBC_MAX_VOLTAGE,
  ibc_step = (ibc_mx_voltage - ibc_mi_voltage) / 12;

float ibc_to_vid = 3.07; //! ratio of IBC to VID voltage

void loop ()
{
  float new_voltage;
  
  if (state == SM_WAITING) {
    lcd.setCursor(0,0);
    lcd.print("V = ");
    lcd.println(ibc_cmd_voltage, DEC); 

    lcd.print("PRESS UP/DOWN  ");
    lcd.setCursor(0,1);
    // waiting for a button press
    buttons = lcd.readButtons(); //! read the buttons on the LDC shield
    if (buttons & BUTTON_SELECT)
      state = SM_BUTTON_SEL;
    else if (buttons & BUTTON_UP)
      state = SM_BUTTON_UP;
    else if (buttons & BUTTON_DOWN)
      state = SM_BUTTON_DOWN;
    else if (buttons & BUTTON_LEFT)
      state = SM_BUTTON_LEFT;
    else if (buttons & BUTTON_RIGHT)
      state = SM_BUTTON_RIGHT;
    else
      state = SM_WAITING;
  }

  else if (state == SM_BUTTON_UP) {
    // increase voltage by one step
    new_voltage = ibc_cmd_voltage + ibc_step;
    ibc_cmd_voltage = (new_voltage <= ibc_mx_voltage) ? new_voltage : ibc_mx_voltage;
    
    vid_value = ibc_cmd_voltage / ibc_to_vid; // IBC voltage is divide by 3 before sensing
    pmbus->setVout(ltc2975_i2c_address, vid_value); // Set new voltage

    state = SM_BUTTON_RELEASE;
  } // SM_BUTTON_UP

  else if (state == SM_BUTTON_DOWN) {
    // decrease voltage by one step
    new_voltage = ibc_cmd_voltage - ibc_step;
    ibc_cmd_voltage = (new_voltage >= ibc_mi_voltage) ? new_voltage : ibc_mi_voltage;
    
    vid_value = ibc_cmd_voltage / ibc_to_vid; // IBC voltage is divide by 3 before sensing
    pmbus->setVout(ltc2975_i2c_address, vid_value); // Set new voltage

    state = SM_BUTTON_RELEASE;
  } // SM_BUTTON_DOWN
  
  else if (state == SM_BUTTON_LEFT) {
    state = SM_BUTTON_RELEASE;
  } // SM_BUTTON_LEFT
  
  else if (state == SM_BUTTON_RIGHT) {
    state = SM_BUTTON_RELEASE;
  } // SM_BUTTON_RIGHT
  
  else if (state == SM_BUTTON_SEL) {
    // increase voltage by one step
    new_voltage = (ibc_mx_voltage - ibc_mi_voltage) / 2; // mid-range
    
    vid_value = ibc_cmd_voltage / ibc_to_vid; // IBC voltage is divide by 3 before sensing
    pmbus->setVout(ltc2975_i2c_address, vid_value); // Set new voltage

    state = SM_BUTTON_RELEASE;
  } // SM_BUTTON_SEL
  
  else if (state == SM_BUTTON_RELEASE) {
    // wait for button to be released before proceeding
    buttons = lcd.readButtons(); //! read the buttons on the LDC shield
    if (buttons)
      state = SM_BUTTON_RELEASE;
    else
      state = SM_WAITING;
  } // SM_BUTTON_RELEASE
  
  else {
    // unknown event
    state = SM_WAITING;
  }

} 
