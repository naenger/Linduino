/*
Linear Technology 
LTC2975 + LTC3765/66 Isolated Forward Converter + Down-stream regulators
Use LTC2945 to measure up-stream V and I

Generate efficiency curves

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

#include <stdint.h>
#include <Arduino.h>
#include <LiquidCrystal.h>

#include "Linduino.h"
#include "UserInterface.h"

#include "LT_I2CBus.h"
#include "LT_I2C.h"
#include "LT_SMBusNoPec.h"
#include "LT_SMBusPec.h"
#include "LT_PMBus.h"
#include "LT_PMBusMath.h"
#include "LT_PMBusRail.h"

#include "LT_I2CBus.h"
#include "LTC2945.h"
//#include <Wire.h>
#include "LT_Wire.h"

//#include "LCD.h"
//#include "Buttons.h"


#define LTC2975_I2C_ADDRESS 0x5C
#define LTM4676_RAIL_ADDRESS 0x30
#define LTM4676_M1_ADDRESS 0x41
#define LTM4676_M2_ADDRESS 0x42
#define LTM4676_M3_ADDRESS 0x43


// System Parameters
#define IBC_MIN_VOLTAGE 8.2
#define IBC_MAX_VOLTAGE 14.9
#define SYS_MIN_VOLTAGE 2.7
#define SYS_MAX_VOLTAGE 4.96
#define SAMPLE_DELAY 1000


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
static tRailDef railDef[4];
static tRailDef *railDefp = &railDef[0];
static uint8_t pages[2];
static LT_PMBusRail *rail;


//! Initialize Linduino
void setup()
{
  Serial.begin(115200);         //! Initialize the serial port to the PC

  uint8_t *addresses = NULL;
  addresses = smbus->probe(0);
  while(*addresses != 0)
  {
    Serial.print(F("ADDR 0x"));
    Serial.println(*addresses++, HEX);
  }
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
  // 0x5F       | LTC2975 primary address
  // 0x66       | LTC2945 global address
  // 0x6F       | LTC2945 primary address
  //////////////////////////////////////////////

  
  ltc2975_i2c_address = LTC2975_I2C_ADDRESS;
  ltc2945_i2c_address = LTC2945_I2C_ADDRESS;
  ltm4676_rail_address = LTM4676_RAIL_ADDRESS;
  ltm4676_m1_address = LTM4676_M1_ADDRESS;
  ltm4676_m2_address = LTM4676_M2_ADDRESS;
  ltm4676_m3_address = LTM4676_M3_ADDRESS;

  pages[0] = 0x00;
  pages[1] = 0x01;

  railDef[0].address = ltm4676_m1_address;
  railDef[0].pages = pages;
  railDef[0].noOfPages = sizeof(pages);

  railDef[1].address = ltm4676_m2_address;
  railDef[1].pages = pages;
  railDef[1].noOfPages = sizeof(pages);

  railDef[2].address = ltm4676_m3_address;
  railDef[2].pages = pages;
  railDef[2].noOfPages = sizeof(pages);

  railDef[3].address = 0x00; // Termination

  rail = new LT_PMBusRail(pmbus, ltm4676_rail_address, &railDefp);

}

void loop ()
{

  // vid voltages are what the LTC2975 commands (including the divided-down 12V rail)
  float vid_value, vid_voltage,
    vid_step = (IBC_MAX_VOLTAGE - IBC_MIN_VOLTAGE) / 8;  // voltage step size

  // vin is the 40V supply to the forward converter (measured by LTC2945)
  float vin_voltage, vin_current;
  float power_in;

  // ibc is the 12V output of the forward converter (measured by the LTC2975)
  float ibc_voltage, ibc_current;
  float power_ibc;

  // out is the 3.3V output rail (controlled by the LTM4676)
  float out_voltage, out_current;
  float power_out;

  float int_current; //LTM4676 input current (should be the same as ibc_current)
  
  float eff_forward;
  float eff_ltm;
  float eff_total;

  uint16_t ltc2945_vin_code, ltc2945_iin_code;
  
  int i;
  const int count = 1; // for averaging
  
  
  // Sweep intermediate bus voltage and output current, voltage, and efficiency
  // 
  pmbus->setPage(ltc2975_i2c_address, 0x00);                // Set to page 0 (servo IBC channel)
  pmbus->setPage(ltm4676_m1_address, 0x00);                // Set to page 0
  pmbus->setPage(ltm4676_m2_address, 0x00);                // Set to page 0
  pmbus->setPage(ltm4676_m3_address, 0x00);                // Set to page 0
  
  // print header to the serial port
  Serial.println(F("V_UPSTREAM    , I_UPSTREAM   ,   P_UPSTREAM ,   COMMANDED_V ,   VID_VALUE ,  V_IBC      ,   I_IBC      ,   P_IBC      ,   INT_CURRENT  ,    V_OUT     ,    I_OUT     ,    P_OUT     ,     EFF_FORWARD ,     EFF_TOTAL")); 

  for (vid_voltage = IBC_MIN_VOLTAGE; vid_voltage <= IBC_MAX_VOLTAGE; vid_voltage = vid_voltage + vid_step) {
    vid_value = vid_voltage / 3; // IBC voltage is divide by 3 before sensing
    //Serial.println(F("2975: set: ")); 
    pmbus->setVout(ltc2975_i2c_address, vid_value);           // Set starting voltage
    delay(SAMPLE_DELAY);
    
    //Serial.println(F("2975: read: ")); 
    ibc_voltage = pmbus->readVout(ltc2975_i2c_address, false); // divided-down voltage (+12V / 3)

    // AVERAGE current and voltage at the LTC2945 on the 48V main bus
    vin_current = 0.0;
    vin_voltage = 0.0;
    for (i = 0; i < count; i++) {
      //Serial.println(F("2945: read: ")); 
      LTC2945_read_12_bits(ltc2945_i2c_address, LTC2945_DELTA_SENSE_MSB_REG, &ltc2945_iin_code);
      LTC2945_read_12_bits(ltc2945_i2c_address, LTC2945_VIN_MSB_REG, &ltc2945_vin_code);
      vin_current += LTC2945_code_to_current(ltc2945_iin_code, 0.020, LTC2945_DELTA_SENSE_lsb); // (code value, sense resistor, lsb value)
      vin_voltage += LTC2945_VIN_code_to_voltage(ltc2945_vin_code, LTC2945_VIN_lsb);
      delay(100);
    }
    vin_current = vin_current / count; // calculate average
    vin_voltage = vin_voltage / count; //calculate average
    
    // calculated input power (V*I)
    // this can be done with one read of the LTC2945 power register instead
    power_in = vin_voltage * vin_current;
    

    // AVERAGE current and voltage at the LTC2975 on the 12V intermediate bus
    ibc_current = 0.0;
    ibc_voltage = 0.0;
    power_ibc = 0.0;
    for (i = 0; i < count; i++) {
      //Serial.println(F("2975: read: ")); 
      ibc_current += pmbus->readIin(ltc2975_i2c_address, false);
      //Serial.println(F("2975: read: ")); 
      ibc_voltage += pmbus->readVin(ltc2975_i2c_address, false);
      //Serial.println(F("2975: read: ")); 
      power_ibc += pmbus->readPin(ltc2975_i2c_address, false);
    }
    ibc_current = ibc_current / count; // calculate average
    ibc_voltage = ibc_voltage / count; //calculate average
    power_ibc = power_ibc / count;


    // AVERAGE current and voltage at the LTM4676 on the 3.3V output rail
    // Note that there are 3 LTM4676 modules on the board, and each module has 2 phases
    // we use the rail construct to talk to the whole board
    int_current = 0.0;
    out_current = 0.0;
    out_voltage = 0.0;
    power_out = 0.0;
    for (i = 0; i < count; i++) {
      //      out_current += rail->readIout(true);
      //      out_voltage += rail->readVout(true);
      //      int_current += rail->readIin(true);
      //      power_out   += rail->readPout(true);
      //Serial.println(F("4676: read: ")); 
      out_current += pmbus->readIout(ltm4676_m1_address, false);
      //Serial.println(F("4676: read: ")); 
      out_current += pmbus->readIout(ltm4676_m2_address, false);
      //Serial.println(F("4676: read: ")); 
      out_current += pmbus->readIout(ltm4676_m3_address, false);
      //Serial.println(F("4676: read: ")); 
      power_out += pmbus->readPout(ltm4676_m1_address, false);
      //Serial.println(F("4676: read: ")); 
      power_out += pmbus->readPout(ltm4676_m2_address, false);
      //Serial.println(F("4676: read: ")); 
      power_out += pmbus->readPout(ltm4676_m3_address, false);
      //Serial.println(F("4676: set page: ")); 
      pmbus->setPage(ltm4676_m1_address, 0x01);
      //Serial.println(F("4676: set page: ")); 
      pmbus->setPage(ltm4676_m2_address, 0x01);
      //Serial.println(F("4676: set page: ")); 
      pmbus->setPage(ltm4676_m3_address, 0x01);
      //Serial.println(F("4676: read: ")); 
      out_current += pmbus->readIout(ltm4676_m1_address, false);
      //Serial.println(F("4676: read: ")); 
      out_current += pmbus->readIout(ltm4676_m2_address, false);
      //Serial.println(F("4676: read: ")); 
      out_current += pmbus->readIout(ltm4676_m3_address, false);
      //Serial.println(F("4676: read: ")); 
      power_out += pmbus->readPout(ltm4676_m1_address, false);
      //Serial.println(F("4676: read: ")); 
      power_out += pmbus->readPout(ltm4676_m2_address, false);
      //Serial.println(F("4676: read: ")); 
      power_out += pmbus->readPout(ltm4676_m3_address, false);      
      //Serial.println(F("4676: set page: ")); 
      pmbus->setPage(ltm4676_m1_address, 0x00);
      //Serial.println(F("4676: set page: ")); 
      pmbus->setPage(ltm4676_m2_address, 0x00);
      //Serial.println(F("4676: set page: ")); 
      pmbus->setPage(ltm4676_m3_address, 0x00);
    }

    //Serial.println(F("4676: read: ")); 
    out_voltage = pmbus->readVout(ltm4676_m1_address, false); // all outputs are tied together, so pick one
    //Serial.println(F("4676: read: ")); 
    int_current += pmbus->readIin(ltm4676_m1_address, false);
    //Serial.println(F("4676: read: ")); 
    int_current += pmbus->readIin(ltm4676_m2_address, false);
    //Serial.println(F("4676: read: ")); 
    int_current += pmbus->readIin(ltm4676_m3_address, false);

    out_current = out_current / count; // calculate average
    power_out = power_out / count;

    if(power_in > 0.001) {
      eff_forward = power_ibc / power_in;
      eff_total = power_out / power_in;
    }
    else {
      eff_forward = 0.0;
      eff_total = 0.0;
    }


    // Print values to the serial port
    Serial.print(vin_voltage, DEC);
    Serial.print(F(" , ")); 
    Serial.print(vin_current, DEC);
    Serial.print(F(" , ")); 
    Serial.print(power_in, DEC);
    Serial.print(F(" , ")); 
    Serial.print(vid_voltage, DEC);
    Serial.print(F(" , ")); 
    Serial.print(vid_value, DEC);
    Serial.print(F(" , ")); 
    Serial.print(ibc_voltage, DEC);
    Serial.print(F(" , ")); 
    Serial.print(ibc_current, DEC);
    Serial.print(F(" , ")); 
    Serial.print(power_ibc, DEC);
    Serial.print(F(" , ")); 
    Serial.print(int_current, DEC);
    Serial.print(F(" , ")); 
    Serial.print(out_voltage, DEC);
    Serial.print(F(" , ")); 
    Serial.print(out_current, DEC);
    Serial.print(F(" , ")); 
    Serial.print(power_out, DEC);
    Serial.print(F(" , ")); 
    Serial.print(eff_forward, DEC);
    Serial.print(F(" , ")); 
    Serial.print(eff_total, DEC);
    Serial.print(F("\n")); 
  }
  
  //Serial.println(F("2975: set: ")); 
  pmbus->setVout(ltc2975_i2c_address, 4.0);           // Set starting voltage
  delay (1000);
  Serial.print(F("\n\n\n"));
  Serial.print(F("Hit any key to repeat...\n"));
  read_int();
}

////////////////////////////////////////////////////////////////////
// Function Definitions


//! Initialize the LTC2945 registers
void LTC2945_configure()
{
  //all defaults seem sane...do nothing
}

