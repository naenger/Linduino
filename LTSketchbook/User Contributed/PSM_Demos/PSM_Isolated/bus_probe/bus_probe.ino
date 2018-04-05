/*
just probe the bus repeatedly

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
//#include "LT_PMBusRail.h"

#include "LT_I2CBus.h"
#include "LTC2945.h"
//#include <Wire.h>
#include "LT_Wire.h"

//#include "LCD.h"
//#include "Buttons.h"


#define LTC2975_I2C_ADDRESS 0x64
//#define LTM4676_RAIL_ADDRESS 0x30
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
static uint8_t ltc2975_i2c_address, ltc2945_i2c_address, ltm4676_m1_address, ltm4676_m2_address, ltm4676_m3_address;
//static uint8_t ltm4676_rail_address;
static LT_PMBusMath *math = new LT_PMBusMath();
//LT_SMBus *smbus = new LT_SMBusPec();
LT_SMBus *smbus = new LT_SMBusNoPec();
LT_PMBus *pmbus = new LT_PMBus(smbus);
//static tRailDef railDef[4];
//static tRailDef *railDefp = &railDef[0];
//static uint8_t pages[2];
//static LT_PMBusRail *rail;


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

}

void loop ()
{

  uint8_t *addresses = NULL;
  addresses = smbus->probe(0);
  while(*addresses != 0)
    {
      Serial.print(F("ADDR 0x"));
      Serial.println(*addresses++, HEX);
    }
  delay(1000);
}

