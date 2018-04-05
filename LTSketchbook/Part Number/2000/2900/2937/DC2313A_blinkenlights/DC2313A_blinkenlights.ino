/*
Linear Technology DC2313A Demonstration
LTC2937: Six Channel Sequencer and Voltage Supervisor with EEPROM
Randomply blink the LEDs on the board

@verbatim9
http://www.linear.com/demo/DC2313A


REVISION HISTORY
$Revision:  $
$Date:  $

Copyright (c) 2015, Linear Technology Corp.(LTC)
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
#include "LT_I2CBus.h"
#include "LT_SMBusNoPec.h"
//#include "LT_SMBusPec.h"
//#include "LT_PMBUS.h"
#include "LT_I2C.h"

#define LTC2937_I2C_ADDRESS_GLOBAL 0x36 //GLOBAL 7-bit address
#define LTC2937_I2C_ADDRESS_37 0x37 //LLL 7-bit address
#define LTC2937_I2C_ADDRESS_38 0x38 //LLZ 7-bit address
#define LTC2937_I2C_ADDRESS_39 0x39 //LLH 7-bit address
#define LTC2937_I2C_ADDRESS_3A 0x3A //LZL 7-bit address
#define LTC2937_I2C_ADDRESS_3B 0x3B //LZZ 7-bit address
#define LTC2937_I2C_ADDRESS_3C 0x3C //LZH 7-bit address
#define LTC2937_I2C_ADDRESS_3D 0x3D //LHL 7-bit address
#define LTC2937_I2C_ADDRESS_3E 0x3E //LHZ 7-bit address
#define LTC2937_I2C_ADDRESS_3F 0x3F //LHH 7-bit address


/********************************************************************************/
//LTC2937 command address definitions

#define LTC2937_WRITE_PROTECTION  0x00
#define LTC2937_SPECIAL_LOT   0x01
#define LTC2937_ON_OFF_CONTROL    0x02
#define LTC2937_V_RANGE     0x03
#define LTC2937_V_THRESHOLD_1   0x04
#define LTC2937_V_THRESHOLD_2   0x05
#define LTC2937_V_THRESHOLD_3   0x06
#define LTC2937_V_THRESHOLD_4   0x07
#define LTC2937_V_THRESHOLD_5   0x08
#define LTC2937_V_THRESHOLD_6   0x09
#define LTC2937_TON_TIMERS_1    0x0A
#define LTC2937_TON_TIMERS_2    0x0B
#define LTC2937_TON_TIMERS_3    0x0C
#define LTC2937_TON_TIMERS_4    0x0D
#define LTC2937_TON_TIMERS_5    0x0E
#define LTC2937_TON_TIMERS_6    0x0F
#define LTC2937_TOFF_TIMERS_1   0x10
#define LTC2937_TOFF_TIMERS_2   0x11
#define LTC2937_TOFF_TIMERS_3   0x12
#define LTC2937_TOFF_TIMERS_4   0x13
#define LTC2937_TOFF_TIMERS_5   0x14
#define LTC2937_TOFF_TIMERS_6   0x15
#define LTC2937_SEQ_UP_POSITION_1 0x16
#define LTC2937_SEQ_UP_POSITION_2 0x17
#define LTC2937_SEQ_UP_POSITION_3 0x18
#define LTC2937_SEQ_UP_POSITION_4 0x19
#define LTC2937_SEQ_UP_POSITION_5 0x1A
#define LTC2937_SEQ_UP_POSITION_6 0x1B
#define LTC2937_SEQ_DOWN_POSITION_1 0x1C
#define LTC2937_SEQ_DOWN_POSITION_2 0x1D
#define LTC2937_SEQ_DOWN_POSITION_3 0x1E
#define LTC2937_SEQ_DOWN_POSITION_4 0x1F
#define LTC2937_SEQ_DOWN_POSITION_5 0x20
#define LTC2937_SEQ_DOWN_POSITION_6 0x21
#define LTC2937_RSTB_CONFIG   0x22
#define LTC2937_FAULT_RESPONSE    0x23
//          0x24
//          0x25
#define LTC2937_MONITOR_STATUS_HISTORY  0x26
//          0x27
#define LTC2937_CLEAR_ALERTB    0x28
#define LTC2937_STATUS_INFORMATION  0x29
#define LTC2937_BREAK_POINT         0x2A
#define LTC2937_SEQ_POSITION_COUNT      0x2B
#define LTC2937_STORE                   0x2C
#define LTC2937_RESTORE                 0x2D
#define LTC2937_CLEAR                   0x2E
#define LTC2937_MONITOR_BACKUP          0x2F
#define LTC2937_MONITOR_STATUS          0x30
#define LTC2937_DEVICE_ID         0x31

/****************************************************************************/
// Global variables
static int N=10;
uint8_t ltc2937_i2c_address[10];
int i2c_address_count; // how many i2c devices found on the bus (max of N)

//static LT_I2CBus *i2cbus = new LT_I2CBus();
//static LT_I2CBus *i2cbus = new LT_I2CBus();
//static LT_SMBusNoPec *smbus = new LT_SMBusNoPec(i2cbus);
static LT_SMBusNoPec *smbus = new LT_SMBusNoPec();


// function return values
// SUCCESS and FAIL are defined in the LT_PMBUS.h file, and cause problems here
//  so we undefine them locally
#undef SUCCESS
#undef FAIL
enum return_values {SUCCESS=0, // the function succeeded without errors
                    NOT_DOWN, // the LTC2937 is not sequenced-down
                    WRITE_PROTECTED, // the LTC2937 is write-protected
                    FAIL  // general failure to do the right thing
                   };

// loop counter variable
int i;

// random variable
uint16_t data1, data2, data3;

// delay times in milliseconds
#define LTC2937_RESTORE_DELAY   10
#define LTC2937_STORE_DELAY   120

/****************************************************************************/
//! Initialize Linduino
void setup()
{
  uint16_t return_val;

  Serial.begin(115200);         //! Initialize the serial port to the PC

  i2c_address_count = 0;
  i2c_address_count = i2c_bus_scan();
  //  ltc2937_i2c_address[0] = LTC2937_I2C_ADDRESS_37;
  //  ltc2937_i2c_address[1] = LTC2937_I2C_ADDRESS_38;
  //  ltc2937_i2c_address[2] = LTC2937_I2C_ADDRESS_39;
  //  ltc2937_i2c_address[3] = LTC2937_I2C_ADDRESS_3A;
  //  ltc2937_i2c_address[4] = LTC2937_I2C_ADDRESS_3B;
  if (i2c_address_count == 0)
  {
    Serial.println(F("ERROR! No I2C devices found!"));
  }
  else
  {
    i2c_address_count = (i2c_address_count < N) ? i2c_address_count : N;
    Serial.print(F("Number of I2C devices found: "));
    Serial.println(i2c_address_count, DEC);
    // initialize all parts on the I2C bus
    for (i = 0; i < i2c_address_count; i++)
    {
      Serial.print(F("Device at address: "));
      Serial.println(ltc2937_i2c_address[i], HEX);
      ltc2937_write_all_regs_blinkenlights(ltc2937_i2c_address[i]);
    }
  }
}

/****************************************************************************/
//! Main Linduino Loop
void loop()
{
  data1 = random(1,7);  // which channel to talk to
  data2 = random(0,N);  // which part to talk to
  data3 = random(0,2048);  // decide whether to turn on or off

  //    Serial.print(data1, HEX);
  //    Serial.print(F("  "));
  //    Serial.print(data2, HEX);
  //    Serial.print(F("  "));
  //    Serial.print(data3, HEX);
  //    Serial.print(F(" \n"));

  switch (data1)
  {
    case 1 :
      if (i2c_address_count != 0)
      {
        smbus->writeWord(ltc2937_i2c_address[data2], LTC2937_SEQ_UP_POSITION_1, (0x0400&data3));
      }
      else
      {
        // just use the global address to call any LTC2937 parts on the bus
        smbus->writeWord(LTC2937_I2C_ADDRESS_GLOBAL, LTC2937_SEQ_UP_POSITION_1, (0x0400&data3));
      }
      break;
    case 2 :
      if (i2c_address_count != 0)
      {
        smbus->writeWord(ltc2937_i2c_address[data2], LTC2937_SEQ_UP_POSITION_2, (0x0400&data3));
      }
      else
      {
        // just use the global address to call any LTC2937 parts on the bus
        smbus->writeWord(LTC2937_I2C_ADDRESS_GLOBAL, LTC2937_SEQ_UP_POSITION_2, (0x0400&data3));
      }
      break;
    case 3 :
      if (i2c_address_count != 0)
      {
        smbus->writeWord(ltc2937_i2c_address[data2], LTC2937_SEQ_UP_POSITION_3, (0x0400&data3));
      }
      else
      {
        // just use the global address to call any LTC2937 parts on the bus
        smbus->writeWord(LTC2937_I2C_ADDRESS_GLOBAL, LTC2937_SEQ_UP_POSITION_3, (0x0400&data3));
      }
      break;
    case 4 :
      if (i2c_address_count != 0)
      {
        smbus->writeWord(ltc2937_i2c_address[data2], LTC2937_SEQ_UP_POSITION_4, (0x0400&data3));
      }
      else
      {
        // just use the global address to call any LTC2937 parts on the bus
        smbus->writeWord(LTC2937_I2C_ADDRESS_GLOBAL, LTC2937_SEQ_UP_POSITION_4, (0x0400&data3));
      }
      break;
    case 5 :
      if (i2c_address_count != 0)
      {
        smbus->writeWord(ltc2937_i2c_address[data2], LTC2937_SEQ_UP_POSITION_5, (0x0400&data3));
      }
      else
      {
        // just use the global address to call any LTC2937 parts on the bus
        smbus->writeWord(LTC2937_I2C_ADDRESS_GLOBAL, LTC2937_SEQ_UP_POSITION_5, (0x0400&data3));
      }
      break;
    case 6 :
      if (i2c_address_count != 0)
      {
        smbus->writeWord(ltc2937_i2c_address[data2], LTC2937_SEQ_UP_POSITION_6, (0x0400&data3));
      }
      else
      {
        // just use the global address to call any LTC2937 parts on the bus
        smbus->writeWord(LTC2937_I2C_ADDRESS_GLOBAL, LTC2937_SEQ_UP_POSITION_6, (0x0400&data3));
      }
      break;
  }
  delay(100);
}



/************************************************************************/
// Function Definitions

//! scan the I2C bus and find all devices
// return the number of devices found on the bus
// addresses stored in an array
int i2c_bus_scan()
{
  unsigned char addr, ack, ack_count;
  ack_count = 0;
  for (addr = 0x36; addr < 0x50; addr++)   //limit to addresses that LTC2937 can achieve
  {
    i2c_start();
    ack = i2c_write(addr << 1);
    i2c_stop;
    if ((ack == 0) && (addr != 0x36))
    {
      Serial.print(F("ACK at addr: "));
      Serial.println(addr, HEX);
      if (ack_count < N)
      {
        ltc2937_i2c_address[ack_count] = addr; //don't overflow the array
      }
      ack_count++; //return the actual number of addresses that ACK. May be more than addresses in the array
    }
  }
  Serial.print(F("Number of I2C devices found: "));
  Serial.println(ack_count, DEC);
  return ack_count;
}


//! restore the LTC2937 settings from EEPROM values
int ltc2937_restore(uint8_t device_address)
{
  uint16_t return_val;

  if (ltc2937_is_down(device_address) == SUCCESS)
  {
    // send the RESTORE command
    smbus->sendByte(device_address, LTC2937_RESTORE);
    Serial.print (F("\nRESTORED LTC2937 CONFIGURATION FROM EEPROM.\n"));
    delay(LTC2937_RESTORE_DELAY);
    return SUCCESS;
  }
  else
  {
    Serial.print(F("\n IT IS A BAD IDEA TO RESTORE WHILE THE SUPPLIES ARE SEQUENCED-UP!\n"));
    Serial.print(F("   --SEQUENCE-DOWN FIRST. \n"));
    return NOT_DOWN;
  }
}

//! store the LTC2937 settings into EEPROM
int ltc2937_store(uint8_t device_address)
{
  uint16_t return_val;

  if (ltc2937_is_down(device_address) == SUCCESS)
  {
    smbus->sendByte(device_address, LTC2937_STORE);
    Serial.print (F("STORED LTC2937 CONFIGURATION TO EEPROM.\n"));
    delay(LTC2937_STORE_DELAY);
    return SUCCESS;
  }
  else
  {
    Serial.print(F("\n IT IS A BAD IDEA TO STORE WHILE THE SUPPLIES ARE SEQUENCED-UP!\n"));
    Serial.print(F("   --SEQUENCE-DOWN FIRST. \n"));
    return NOT_DOWN;
  }
}

//! sequence-up the LTC2937
int ltc2937_sequence_up(uint8_t device_address)
{
  uint16_t return_val;

  if ( ltc2937_is_write_protected(device_address) != WRITE_PROTECTED)
  {
    return_val = smbus->readWord(device_address, LTC2937_ON_OFF_CONTROL);
    return_val = (return_val & 0xFFEB); // mask the bits of interest
    return_val = (return_val | 0x0018);  // set the ON/OFF control to ON
    smbus->writeWord(device_address, LTC2937_ON_OFF_CONTROL, return_val);
    return SUCCESS;
  }
  else
  {
    Serial.print (F("FAIL: 2937_WRITE_PROTECTED.\n"));
    return WRITE_PROTECTED;
  }
}

//! sequence-down the LTC2937
int ltc2937_sequence_down(uint8_t device_address)
{
  uint16_t return_val;

  if ( ltc2937_is_write_protected(device_address) != WRITE_PROTECTED)
  {
    return_val = smbus->readWord(device_address, LTC2937_ON_OFF_CONTROL);
    return_val = (return_val & 0xFFEF); // mask the bits
    smbus->writeWord(device_address, LTC2937_ON_OFF_CONTROL, return_val);
    return SUCCESS;
  }
  else
  {
    Serial.print (F("FAIL: 2937_WRITE_PROTECTED.\n"));
    return WRITE_PROTECTED;
  }
}

int ltc2937_is_down(uint8_t device_address)
{
  uint16_t return_val,
           on_off,
           status;

  // This function indicates when it is safe to issue a CLEAR command

  // must meet the following conditions:
  // ON_OFF_CONTROL[7] == 1'b0 (commanded down)
  // STATUS_INFORMATION[11:10] == 2'b00 (local seq down complete)
  // STATUS_INFORMATION[9:8] == 2'b00 (global seq down complete)
  // STATUS_INFORMATION[2] == 1'b1 (supplies discharged)

  // may also want:
  // BREAK_POINT[10] == 1'b0 (not in BP mode)

  return_val = smbus->readWord(device_address, LTC2937_STATUS_INFORMATION);
  status = (return_val & 0x0F04); // mask the bits
  return_val = smbus->readWord(device_address, LTC2937_ON_OFF_CONTROL);
  on_off = (return_val & 0x0080); // mask the bits
  if ((status == 0x0004) && (on_off == 0x0000))
  {
    return SUCCESS;
  }
  else
  {
    Serial.print(F("\n LTC2937 IS NOT DOWN \n"));
    return NOT_DOWN;
  }
}

//! read all status registers
int ltc2937_read_all_status(uint8_t device_address)
{
  uint16_t return_val;

  return_val = smbus->readWord(device_address, LTC2937_WRITE_PROTECTION);
  Serial.print(F("\n LTC2937_WRITE_PROTECTION: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_SPECIAL_LOT);
  Serial.print(F("\n LTC2937_SPECIAL_LOT: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_ON_OFF_CONTROL);
  Serial.print(F("\n LTC2937_ON_OFF_CONTROL: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_MONITOR_STATUS_HISTORY);
  Serial.print(F("\n LTC2937_MONITOR_STATUS_HISTORY: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_STATUS_INFORMATION);
  Serial.print(F("\n LTC2937_STATUS_INFORMATION: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_BREAK_POINT);
  Serial.print(F("\n LTC2937_BREAK_POINT: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_FAULT_RESPONSE);
  Serial.print(F("\n LTC2937_FAULT_RESPONSE: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_SEQ_POSITION_COUNT);
  Serial.print(F("\n LTC2937_SEQ_POSITION_COUNT: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_MONITOR_BACKUP);
  Serial.print(F("\n LTC2937_MONITOR_BACKUP: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_MONITOR_STATUS);
  Serial.print(F("\n LTC2937_MONITOR_STATUS: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_DEVICE_ID);
  Serial.print(F("\n LTC2937_DEVICE_ID: "));
  Serial.println(return_val, HEX);
  return SUCCESS;
}



//! read all registers
int ltc2937_read_all_registers(uint8_t device_address)
{
  uint16_t return_val;

  return_val = smbus->readWord(device_address, LTC2937_WRITE_PROTECTION);
  Serial.print(F("\n LTC2937_WRITE_PROTECTION: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_SPECIAL_LOT);
  Serial.print(F("\n LTC2937_SPECIAL_LOT: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_ON_OFF_CONTROL);
  Serial.print(F("\n LTC2937_ON_OFF_CONTROL: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_V_RANGE);
  Serial.print(F("\n LTC2937_V_RANGE: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_V_THRESHOLD_1);
  Serial.print(F("\n LTC2937_V_THRESHOLD_1: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_V_THRESHOLD_2);
  Serial.print(F("\n LTC2937_V_THRESHOLD_2: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_V_THRESHOLD_3);
  Serial.print(F("\n LTC2937_V_THRESHOLD_3: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_V_THRESHOLD_4);
  Serial.print(F("\n LTC2937_V_THRESHOLD_4: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_V_THRESHOLD_5);
  Serial.print(F("\n LTC2937_V_THRESHOLD_5: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_V_THRESHOLD_6);
  Serial.print(F("\n LTC2937_V_THRESHOLD_6: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_TON_TIMERS_1);
  Serial.print(F("\n LTC2937_TON_TIMERS_1: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_TON_TIMERS_2);
  Serial.print(F("\n LTC2937_TON_TIMERS_2: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_TON_TIMERS_3);
  Serial.print(F("\n LTC2937_TON_TIMERS_3: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_TON_TIMERS_4);
  Serial.print(F("\n LTC2937_TON_TIMERS_4: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_TON_TIMERS_5);
  Serial.print(F("\n LTC2937_TON_TIMERS_5: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_TON_TIMERS_6);
  Serial.print(F("\n LTC2937_TON_TIMERS_6: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_TOFF_TIMERS_1);
  Serial.print(F("\n LTC2937_TOFF_TIMERS_1: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_TOFF_TIMERS_2);
  Serial.print(F("\n LTC2937_TOFF_TIMERS_2: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_TOFF_TIMERS_3);
  Serial.print(F("\n LTC2937_TOFF_TIMERS_3: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_TOFF_TIMERS_4);
  Serial.print(F("\n LTC2937_TOFF_TIMERS_4: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_TOFF_TIMERS_5);
  Serial.print(F("\n LTC2937_TOFF_TIMERS_5: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_TOFF_TIMERS_6);
  Serial.print(F("\n LTC2937_TOFF_TIMERS_6: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_SEQ_UP_POSITION_1);
  Serial.print(F("\n LTC2937_SEQ_UP_POSITION_1: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_SEQ_UP_POSITION_2);
  Serial.print(F("\n LTC2937_SEQ_UP_POSITION_2: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_SEQ_UP_POSITION_3);
  Serial.print(F("\n LTC2937_SEQ_UP_POSITION_3: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_SEQ_UP_POSITION_4);
  Serial.print(F("\n LTC2937_SEQ_UP_POSITION_4: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_SEQ_UP_POSITION_5);
  Serial.print(F("\n LTC2937_SEQ_UP_POSITION_5: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_SEQ_UP_POSITION_6);
  Serial.print(F("\n LTC2937_SEQ_UP_POSITION_6: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_SEQ_DOWN_POSITION_1);
  Serial.print(F("\n LTC2937_SEQ_DOWN_POSITION_1: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_SEQ_DOWN_POSITION_2);
  Serial.print(F("\n LTC2937_SEQ_DOWN_POSITION_2: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_SEQ_DOWN_POSITION_3);
  Serial.print(F("\n LTC2937_SEQ_DOWN_POSITION_3: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_SEQ_DOWN_POSITION_4);
  Serial.print(F("\n LTC2937_SEQ_DOWN_POSITION_4: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_SEQ_DOWN_POSITION_5);
  Serial.print(F("\n LTC2937_SEQ_DOWN_POSITION_5: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_SEQ_DOWN_POSITION_6);
  Serial.print(F("\n LTC2937_SEQ_DOWN_POSITION_6: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_RSTB_CONFIG);
  Serial.print(F("\n LTC2937_RSTB_CONFIG: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_FAULT_RESPONSE);
  Serial.print(F("\n LTC2937_FAULT_RESPONSE: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_MONITOR_STATUS_HISTORY);
  Serial.print(F("\n LTC2937_MONITOR_STATUS_HISTORY: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_STATUS_INFORMATION);
  Serial.print(F("\n LTC2937_STATUS_INFORMATION: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_BREAK_POINT);
  Serial.print(F("\n LTC2937_BREAK_POINT: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_SEQ_POSITION_COUNT);
  Serial.print(F("\n LTC2937_SEQ_POSITION_COUNT: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_MONITOR_BACKUP);
  Serial.print(F("\n LTC2937_MONITOR_BACKUP: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_MONITOR_STATUS);
  Serial.print(F("\n LTC2937_MONITOR_STATUS: "));
  Serial.println(return_val, HEX);

  return_val = smbus->readWord(device_address, LTC2937_DEVICE_ID);
  Serial.print(F("\n LTC2937_DEVICE_ID: "));
  Serial.println(return_val, HEX);
  return SUCCESS;
}

//! write all registers to demo board defaults
int ltc2937_write_all_regs_dc_default(uint8_t device_address)
{
  if ( ltc2937_is_write_protected(device_address) != WRITE_PROTECTED)
  {
    if (ltc2937_is_down(device_address) == SUCCESS)
    {
      smbus->writeWord(device_address, LTC2937_WRITE_PROTECTION, 0x3AA);
      smbus->writeWord(device_address, LTC2937_SPECIAL_LOT, 0x2313);
      smbus->writeWord(device_address, LTC2937_ON_OFF_CONTROL, 0x0005);
      smbus->writeWord(device_address, LTC2937_V_RANGE, 0x0554);
      smbus->writeWord(device_address, LTC2937_V_THRESHOLD_1, 0x8970);
      smbus->writeWord(device_address, LTC2937_V_THRESHOLD_2, 0xE6C1);
      smbus->writeWord(device_address, LTC2937_V_THRESHOLD_3, 0xaF91);
      smbus->writeWord(device_address, LTC2937_V_THRESHOLD_4, 0x997E);
      smbus->writeWord(device_address, LTC2937_V_THRESHOLD_5, 0x7662);
      smbus->writeWord(device_address, LTC2937_V_THRESHOLD_6, 0x5745);
      smbus->writeWord(device_address, LTC2937_TON_TIMERS_1, 0xF000);
      smbus->writeWord(device_address, LTC2937_TON_TIMERS_2, 0xF000);
      smbus->writeWord(device_address, LTC2937_TON_TIMERS_3, 0xF000);
      smbus->writeWord(device_address, LTC2937_TON_TIMERS_4, 0xF000);
      smbus->writeWord(device_address, LTC2937_TON_TIMERS_5, 0xF000);
      smbus->writeWord(device_address, LTC2937_TON_TIMERS_6, 0xF000);
      smbus->writeWord(device_address, LTC2937_TOFF_TIMERS_1, 0x8000);
      smbus->writeWord(device_address, LTC2937_TOFF_TIMERS_2, 0x8000);
      smbus->writeWord(device_address, LTC2937_TOFF_TIMERS_3, 0x8000);
      smbus->writeWord(device_address, LTC2937_TOFF_TIMERS_4, 0x8000);
      smbus->writeWord(device_address, LTC2937_TOFF_TIMERS_5, 0x8000);
      smbus->writeWord(device_address, LTC2937_TOFF_TIMERS_6, 0x8000);
      smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_1, 0x0001);
      smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_2, 0x0002);
      smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_3, 0x0003);
      smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_4, 0x0004);
      smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_5, 0x0005);
      smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_6, 0x0006);
      smbus->writeWord(device_address, LTC2937_SEQ_DOWN_POSITION_1, 0x0006);
      smbus->writeWord(device_address, LTC2937_SEQ_DOWN_POSITION_2, 0x0005);
      smbus->writeWord(device_address, LTC2937_SEQ_DOWN_POSITION_3, 0x0004);
      smbus->writeWord(device_address, LTC2937_SEQ_DOWN_POSITION_4, 0x0003);
      smbus->writeWord(device_address, LTC2937_SEQ_DOWN_POSITION_5, 0x0002);
      smbus->writeWord(device_address, LTC2937_SEQ_DOWN_POSITION_6, 0x0001);
      smbus->writeWord(device_address, LTC2937_RSTB_CONFIG, 0xBFFF);
      smbus->writeWord(device_address, LTC2937_FAULT_RESPONSE, 0x022F);
      smbus->writeWord(device_address, LTC2937_BREAK_POINT, 0x0000);
      smbus->writeWord(device_address, LTC2937_DEVICE_ID, 0x2937);

      return SUCCESS;
    }
    else
    {
      return NOT_DOWN;
    }
  }
  else
  {
    Serial.print (F("FAIL: 2937_WRITE_PROTECTED.\n"));
    return WRITE_PROTECTED;
  }
}

//! write all registers to demo board defaults
int ltc2937_write_all_regs_blinkenlights(uint8_t device_address)
{
  //  if( ltc2937_is_write_protected(device_address) != 2937_WRITE_PROTECTED) {
  //    if(ltc2937_is_down(device_address) == 2937_SUCCESS){
  smbus->writeWord(device_address, LTC2937_WRITE_PROTECTION, 0x3AA);
  smbus->writeWord(device_address, LTC2937_SPECIAL_LOT, 0x2313);
  smbus->writeWord(device_address, LTC2937_ON_OFF_CONTROL, 0x0000);
  smbus->writeWord(device_address, LTC2937_V_RANGE, 0x0AAA);
  smbus->writeWord(device_address, LTC2937_V_THRESHOLD_1, 0x8970);
  smbus->writeWord(device_address, LTC2937_V_THRESHOLD_2, 0xE6C1);
  smbus->writeWord(device_address, LTC2937_V_THRESHOLD_3, 0xaF91);
  smbus->writeWord(device_address, LTC2937_V_THRESHOLD_4, 0x997E);
  smbus->writeWord(device_address, LTC2937_V_THRESHOLD_5, 0x7662);
  smbus->writeWord(device_address, LTC2937_V_THRESHOLD_6, 0x5745);
  smbus->writeWord(device_address, LTC2937_TON_TIMERS_1, 0xF000);
  smbus->writeWord(device_address, LTC2937_TON_TIMERS_2, 0xF000);
  smbus->writeWord(device_address, LTC2937_TON_TIMERS_3, 0xF000);
  smbus->writeWord(device_address, LTC2937_TON_TIMERS_4, 0xF000);
  smbus->writeWord(device_address, LTC2937_TON_TIMERS_5, 0xF000);
  smbus->writeWord(device_address, LTC2937_TON_TIMERS_6, 0xF000);
  smbus->writeWord(device_address, LTC2937_TOFF_TIMERS_1, 0x8000);
  smbus->writeWord(device_address, LTC2937_TOFF_TIMERS_2, 0x8000);
  smbus->writeWord(device_address, LTC2937_TOFF_TIMERS_3, 0x8000);
  smbus->writeWord(device_address, LTC2937_TOFF_TIMERS_4, 0x8000);
  smbus->writeWord(device_address, LTC2937_TOFF_TIMERS_5, 0x8000);
  smbus->writeWord(device_address, LTC2937_TOFF_TIMERS_6, 0x8000);
  smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_1, 0x0000);
  smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_2, 0x0000);
  smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_3, 0x0000);
  smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_4, 0x0000);
  smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_5, 0x0000);
  smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_6, 0x0000);
  smbus->writeWord(device_address, LTC2937_SEQ_DOWN_POSITION_1, 0x0000);
  smbus->writeWord(device_address, LTC2937_SEQ_DOWN_POSITION_2, 0x0000);
  smbus->writeWord(device_address, LTC2937_SEQ_DOWN_POSITION_3, 0x0000);
  smbus->writeWord(device_address, LTC2937_SEQ_DOWN_POSITION_4, 0x0000);
  smbus->writeWord(device_address, LTC2937_SEQ_DOWN_POSITION_5, 0x0000);
  smbus->writeWord(device_address, LTC2937_SEQ_DOWN_POSITION_6, 0x0000);
  smbus->writeWord(device_address, LTC2937_RSTB_CONFIG, 0xBFFF);
  smbus->writeWord(device_address, LTC2937_FAULT_RESPONSE, 0x022F);
  smbus->writeWord(device_address, LTC2937_BREAK_POINT, 0x0000);
  smbus->writeWord(device_address, LTC2937_DEVICE_ID, 0x2937);

  return SUCCESS;
  //    }
  //    else {
  //      return NOT_DOWN;
  //    }
  //  }
  //  else {
  //    Serial.print (F("FAIL: 2937_WRITE_PROTECTED.\n"));
  //    return WRITE_PROTECTED;
  //  }
}


//! write sequencing registers to a user-specified pattern
int ltc2937_write_seq_regs(uint8_t device_address)
{
  int i;
  uint16_t return_val;
  uint16_t user_command;
  uint8_t dummy_reg_address;


  if ( ltc2937_is_write_protected(device_address) != WRITE_PROTECTED)
  {
    if (ltc2937_is_down(device_address) == SUCCESS)
    {

      Serial.print(F("\nCHANGING SEQUENCE-UP ORDER\n"));
      dummy_reg_address = LTC2937_SEQ_UP_POSITION_1;
      for (i = 1; i <= 6; i++)
      {
        Serial.print(F("\nENTER SEQUENCE-UP POSITION FOR CHANNEL "));
        Serial.println(i, DEC);

        user_command = read_int();
        if (user_command > 1023)
        {
          user_command = 1023;
        }
        return_val = 0x0000 + user_command;
        smbus->writeWord(device_address, dummy_reg_address, return_val);
        dummy_reg_address++;
      }

      Serial.print(F("\nCHANGING SEQUENCE-DOWN ORDER\n"));
      dummy_reg_address = LTC2937_SEQ_DOWN_POSITION_1;
      for (i = 1; i <= 6; i++)
      {
        Serial.print(F("\nENTER SEQUENCE-DOWN POSITION FOR CHANNEL "));
        Serial.println(i, DEC);

        user_command = read_int();
        if (user_command > 1023)
        {
          user_command = 1023;
        }
        return_val = 0x0000 + user_command;
        smbus->writeWord(device_address, dummy_reg_address, return_val);
        dummy_reg_address++;
      }
      return SUCCESS;
    }
    else
    {
      return NOT_DOWN;
    }
  }
  else
  {
    Serial.print (F("FAIL: 2937_WRITE_PROTECTED.\n"));
    return WRITE_PROTECTED;
  }
}


//! return 0 if the part is write enabled (hardware and software), 1 if it is protected
int ltc2937_is_write_protected(uint8_t device_address)
{
  uint16_t return_val;
  uint16_t wp_bits;

  return_val = smbus->readWord(device_address, LTC2937_WRITE_PROTECTION);
  wp_bits = (return_val & 0x0003);

  if (wp_bits == 0x0000)
  {
    // neither hardware nor software lock bits set
    return SUCCESS;
  }
  else
  {
    Serial.print(F("\n LTC2937 IS WRITE-PROTECTED. WRITING WILL FAIL\n"));
    return WRITE_PROTECTED;
  }

}

//! remove software write-protection
//  report if the part is hardware write-protected
int ltc2937_write_enable(uint8_t device_address)
{
  uint16_t return_val, write_val;
  uint16_t hardware_wp_bit;
  uint16_t software_wp_bit;

  return_val = smbus->readWord(device_address, LTC2937_WRITE_PROTECTION);
  hardware_wp_bit = (return_val & 0x0002);
  software_wp_bit = (return_val & 0x0001);
  if (hardware_wp_bit == 0x0000)
  {
    // hardware is not write-protected, write the software unlock bit
    Serial.print(F("\n UNLOCKING LTC2937 SOFTWARE WRITE-PROTECT BIT."));
    //    Serial.println(return_val, HEX);
    write_val = (return_val & 0xFFFE);
    smbus->writeWord(device_address, LTC2937_WRITE_PROTECTION, write_val);
  }
  else
  {
    Serial.print(F("\n LTC2937 IS HARDWARE WRITE-PROTECTED. WRITING WILL FAIL\n"));
    return WRITE_PROTECTED;
  }
}

//! set the breakpoint to the given value
int ltc2937_set_breakpoint(uint8_t device_address, uint16_t set_val)
{
  uint16_t return_val;
  //  uint16_t bp_en_val;
  // read the existing breakpoint
  //  return_val = smbus->readWord(device_address, LTC2937_BREAK_POINT);
  //  bp_en_val = (return_val & 0x0400); // mask the bits of interest: b[10]
  //  return_val = (return_val & 0x03FF); // mask the bits of interest: b[9:0]
  if ( ltc2937_is_write_protected(device_address) != WRITE_PROTECTED)
  {
    if ((set_val >=0) && (set_val < 1024))
    {
      return_val = 0x0400 + set_val;  // set the enable bit and break_point value
      smbus->writeWord(device_address, LTC2937_BREAK_POINT, return_val);
    }
    else
    {
      Serial.print(F("\n ERROR! BREAKPOINT VALUE OUT OF RANGE.\n"));
    }
    return SUCCESS;
  }
  else
  {
    Serial.print (F("FAIL: 2937_WRITE_PROTECTED.\n"));
    return WRITE_PROTECTED;
  }
}

//! increment the breakpoint by 1
int ltc2937_inc_breakpoint(uint8_t device_address)
{
  uint16_t return_val;
  uint16_t bp_en_val;
  uint16_t bp_ct_val;

  //  read the existing breakpoint
  if ( ltc2937_is_write_protected(device_address) != WRITE_PROTECTED)
  {
    return_val = smbus->readWord(device_address, LTC2937_BREAK_POINT);
    bp_en_val = (return_val & 0x0400); // mask the bits of interest: b[10]
    bp_ct_val = (return_val & 0x03FF); // mask the bits of interest: b[9:0]
    if (bp_ct_val < 1023)
    {
      return_val = (++bp_ct_val);
      return_val = (return_val | 0x0400); // ensure that the enable bit is set
      smbus->writeWord(device_address, LTC2937_BREAK_POINT, return_val);
      Serial.print(F("\n INCREMENTING BREAK_POINT TO VALUE : \n"));
      Serial.println(bp_ct_val);
      return SUCCESS;
    }
    else
    {
      Serial.print(F("\n ERROR! BREAKPOINT VALUE OUT OF RANGE.\n"));
      return FAIL;
    }
  }
  else
  {
    Serial.print (F("FAIL: 2937_WRITE_PROTECTED.\n"));
    return WRITE_PROTECTED;
  }
}


//! pretty-print the sequence_position_count register contents
int ltc2937_print_seq_pos_count(uint8_t device_address)
{
  uint16_t return_val;
  uint16_t sp_bp_test;
  uint16_t sp_count;

  return_val = smbus->readWord(device_address, LTC2937_SEQ_POSITION_COUNT);
  sp_bp_test = (return_val & 0x0400);
  sp_count   = (return_val & 0x3FF);

  Serial.println(return_val, HEX);
  Serial.print(F("\n SEQUENCE POSITION COUNT = "));
  Serial.println(sp_count, DEC);

  if (sp_bp_test == 0x0400)
  {
    return_val = smbus->readWord(device_address, LTC2937_BREAK_POINT);
    if ((return_val & 0x0400) == 0x0400)
    {
      Serial.print(F("\n WAITING AT BREAKPOINT.\n"));
    }
  }
  else
  {
    Serial.print(F("\n NOT AT BREAKPOINT."));
  }
  return SUCCESS;
}


//! clear breakpoint register
int ltc2937_clear_breakpoint(uint8_t device_address)
{
  if ( ltc2937_is_write_protected(device_address) != WRITE_PROTECTED)
  {
    smbus->writeWord(device_address, LTC2937_BREAK_POINT, 0x0000);
    return SUCCESS;
  }
  else
  {
    return WRITE_PROTECTED;
  }
}

//! issue a clear command
int ltc2937_clear(uint8_t device_address)
{
  uint16_t return_val;
  uint8_t user_command;

  if (ltc2937_is_down(device_address) == NOT_DOWN)
  {
    Serial.print(F("\n IT IS A BAD IDEA TO CLEAR WHILE THE SUPPLIES ARE SEQUENCED-UP!\n"));
    Serial.print(F("     DO YOU WISH TO CLEAR? (y/n)"));
    user_command = read_char();         //! Reads the user command
    if ((user_command == 'y') || (user_command == 'Y'))
    {
      smbus->readWord(device_address, LTC2937_CLEAR);
      Serial.print(F("\n ****CLEARING**** \n"));
      return SUCCESS;
    }
    else
    {
      Serial.print(F("\n ****NOT CLEARING**** \n"));
      return NOT_DOWN;
    }
  }
  else
  {
    smbus->readWord(device_address, LTC2937_CLEAR);
    Serial.print(F("\n ****CLEARING**** \n"));
    return SUCCESS;
  }
}

//! handle the backup word fault log (affects NVM and RAM)
int ltc2937_clear_fault_backup(uint8_t device_address)
{
  uint8_t user_command;

  // RECIPE:
  //  check for sequenced-down and writable
  //  warn the user and ask for permission to overwrite config registers
  //  write-enable
  //  restore to retrieve the backup word
  //  pretty-print fault information
  //  clear to clear all faults
  //  store to clear the NVM backup word
  //  restore (again) to clear the RAM backup word flag

  if ( ltc2937_is_write_protected(device_address) != WRITE_PROTECTED)
  {
    if (ltc2937_is_down(device_address) == SUCCESS)
    {
      Serial.print(F("\n THIS OPERATION WILL AFFECT BOTH RAM AND EEPROM!\n"));
      Serial.print(F("     DO YOU WISH TO CLEAR RAM CONFIGURATION SETTINGS? (y/n)"));
      user_command = read_char();         //! Reads the user command
      if ((user_command == 'y') || (user_command == 'Y'))
      {
        if (ltc2937_restore(device_address) == SUCCESS)
        {
          ltc2937_pretty_print_monitor_backup(device_address);
          ltc2937_clear(device_address);
          ltc2937_store(device_address);
          delay(100);
          ltc2937_restore(device_address);
          Serial.print (F("\nTHE BACKUP WORD IS NOW CLEAR.\n"));
        }
        else
        {
          // fail due to unsuccessful restore
          return FAIL;
        }
      }
    }
    else
    {
      return NOT_DOWN;
    }
  }
  else
  {
    Serial.print (F("FAIL: CANNOT MANIPULATE FAULT BACKUP WHILE WRITE-PROTECTED.\n"));
    return WRITE_PROTECTED;
  }


}


//! clear ALERTB pin
int ltc2937_clear_alertb(uint8_t device_address)
{
  smbus->readWord(device_address, LTC2937_CLEAR_ALERTB);
  return SUCCESS;
}

//! pretty-print MONITOR_BACKUP
int ltc2937_pretty_print_monitor_backup(uint8_t device_address)
{
  uint16_t return_val,
           masked_val;

  return_val = smbus->readWord(device_address, LTC2937_MONITOR_BACKUP);

  Serial.print(F("\n LTC2937 MONITOR_BACKUP CONTENTS: "));
  Serial.println(return_val, HEX);

  masked_val = (return_val & 0xE000);
  switch (masked_val)
  {
    case 0xE000 :
      Serial.print(F("\n**RESERVED SEQUENCE STATE (THIS IS BAD)"));
      break;
    case 0xC000 :
      Serial.print(F("\n**SEQUENCE FAULT ON CHANNEL 6"));
      break;
    case 0xA000 :
      Serial.print(F("\n**SEQUENCE FAULT ON CHANNEL 5"));
      break;
    case 0x8000 :
      Serial.print(F("\n**SEQUENCE FAULT ON CHANNEL 4"));
      break;
    case 0x6000 :
      Serial.print(F("\n**SEQUENCE FAULT ON CHANNEL 3"));
      break;
    case 0x4000 :
      Serial.print(F("\n**SEQUENCE FAULT ON CHANNEL 2"));
      break;
    case 0x2000 :
      Serial.print(F("\n**SEQUENCE FAULT ON CHANNEL 1"));
      break;
    case 0x0000 :
      Serial.print(F("\n  NO SEQUENCE FAULTS"));
      break;
    default :
      Serial.print(F("\n**UNDEFINED SEQUENCE FAULT STATE (THIS IS BAD)"));
      break;
  }
  masked_val = (return_val & 0x1000);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n  NO RESET FAULT"));
  }
  masked_val = (return_val & 0x0800);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V6 OV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n  NO V6 OV FAULT"));
  }
  masked_val = (return_val & 0x0400);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V6 UV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n  NO V6 UV FAULT"));
  }
  masked_val = (return_val & 0x0200);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V5 OV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n  NO V5 OV FAULT"));
  }
  masked_val = (return_val & 0x0100);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V5 UV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n  NO V5 UV FAULT"));
  }
  masked_val = (return_val & 0x0080);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V4 OV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n  NO V4 OV FAULT"));
  }
  masked_val = (return_val & 0x0040);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V4 UV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n  NO V4 UV FAULT"));
  }
  masked_val = (return_val & 0x0020);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V3 OV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n  NO V3 OV FAULT"));
  }
  masked_val = (return_val & 0x0010);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V3 UV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n  NO V3 UV FAULT"));
  }
  masked_val = (return_val & 0x0008);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V2 OV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n  NO V2 OV FAULT"));
  }
  masked_val = (return_val & 0x0004);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V2 UV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n  NO V2 UV FAULT"));
  }
  masked_val = (return_val & 0x0002);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V1 OV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n  NO V1 OV FAULT"));
  }
  masked_val = (return_val & 0x0001);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V1 UV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n  NO V1 UV FAULT"));
  }
  return SUCCESS;
}

//! pretty-print MONITOR_STATUS_HISTORY
int ltc2937_pretty_print_monitor_status_history(uint8_t device_address)
{
  uint16_t return_val,
           masked_val;

  return_val = smbus->readWord(device_address, LTC2937_MONITOR_STATUS_HISTORY);

  Serial.print(F("\n LTC2937 MONITOR_STATUS_HISTORY CONTENTS: "));
  Serial.println(return_val, HEX);

  masked_val = (return_val & 0xE000);
  switch (masked_val)
  {
    case 0xE000 :
      Serial.print(F("\n RESERVED SEQUENCE STATE (THIS IS BAD)"));
      break;
    case 0xC000 :
      Serial.print(F("\n SEQUENCE FAULT ON CHANNEL 6"));
      break;
    case 0xA000 :
      Serial.print(F("\n SEQUENCE FAULT ON CHANNEL 5"));
      break;
    case 0x8000 :
      Serial.print(F("\n SEQUENCE FAULT ON CHANNEL 4"));
      break;
    case 0x6000 :
      Serial.print(F("\n SEQUENCE FAULT ON CHANNEL 3"));
      break;
    case 0x4000 :
      Serial.print(F("\n SEQUENCE FAULT ON CHANNEL 2"));
      break;
    case 0x2000 :
      Serial.print(F("\n SEQUENCE FAULT ON CHANNEL 1"));
      break;
    case 0x0000 :
      Serial.print(F("\n NO SEQUENCE FAULTS"));
      break;
    default :
      Serial.print(F("\n UNDEFINED SEQUENCE FAULT STATE (THIS IS BAD)"));
      break;
  }
  masked_val = (return_val & 0x1000);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n NO RESET FAULT"));
  }
  masked_val = (return_val & 0x0800);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n V6 OV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n NO V6 OV FAULT"));
  }
  masked_val = (return_val & 0x0400);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n V6 UV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n NO V6 UV FAULT"));
  }
  masked_val = (return_val & 0x0200);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n V5 OV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n NO V5 OV FAULT"));
  }
  masked_val = (return_val & 0x0100);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n V5 UV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n NO V5 UV FAULT"));
  }
  masked_val = (return_val & 0x0080);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n V4 OV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n NO V4 OV FAULT"));
  }
  masked_val = (return_val & 0x0040);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n V4 UV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n NO V4 UV FAULT"));
  }
  masked_val = (return_val & 0x0020);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n V3 OV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n NO V3 OV FAULT"));
  }
  masked_val = (return_val & 0x0010);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n V3 UV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n NO V3 UV FAULT"));
  }
  masked_val = (return_val & 0x0008);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n V2 OV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n NO V2 OV FAULT"));
  }
  masked_val = (return_val & 0x0004);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n V2 UV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n NO V2 UV FAULT"));
  }
  masked_val = (return_val & 0x0002);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n V1 OV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n NO V1 OV FAULT"));
  }
  masked_val = (return_val & 0x0001);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n V1 UV RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n NO V1 UV FAULT"));
  }
  return SUCCESS;
}

//! pretty-print MONITOR_STATUS
int ltc2937_pretty_print_monitor_status(uint8_t device_address)
{
  uint16_t return_val,
           masked_val;

  return_val = smbus->readWord(device_address, LTC2937_MONITOR_STATUS);

  Serial.print(F("\n LTC2937 MONITOR_STATUS CONTENTS: "));
  Serial.println(return_val, HEX);

  masked_val = (return_val & 0x2000);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n MARGIN IS ACTIVE"));
  }
  else
  {
    Serial.print(F("\n MARGIN IS NOT ACTIVE"));
  }
  masked_val = (return_val & 0x1000);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n RSTB PIN IS LOW"));
  }
  else
  {
    Serial.print(F("\n RSTB PIN IS HIGH"));
  }
  masked_val = (return_val & 0x0800);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V6 OV COMPARATOR ASSERTED"));
  }
  else
  {
    Serial.print(F("\n  V6 OV COMPARATOR NOT ASSERTED"));
  }
  masked_val = (return_val & 0x0400);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V6 UV COMPARATOR ASSERTED"));
  }
  else
  {
    Serial.print(F("\n  V6 UV COMPARATOR NOT ASSERTED"));
  }
  masked_val = (return_val & 0x0200);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V5 OV COMPARATOR ASSERTED"));
  }
  else
  {
    Serial.print(F("\n  V5 OV COMPARATOR NOT ASSERTED"));
  }
  masked_val = (return_val & 0x0100);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V5 UV COMPARATOR ASSERTED"));
  }
  else
  {
    Serial.print(F("\n  V5 UV COMPARATOR NOT ASSERTED"));
  }
  masked_val = (return_val & 0x0080);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V4 OV COMPARATOR ASSERTED"));
  }
  else
  {
    Serial.print(F("\n  V4 OV COMPARATOR NOT ASSERTED"));
  }
  masked_val = (return_val & 0x0040);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V4 UV COMPARATOR ASSERTED"));
  }
  else
  {
    Serial.print(F("\n  V4 UV COMPARATOR NOT ASSERTED"));
  }
  masked_val = (return_val & 0x0020);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V3 OV COMPARATOR ASSERTED"));
  }
  else
  {
    Serial.print(F("\n  V3 OV COMPARATOR NOT ASSERTED"));
  }
  masked_val = (return_val & 0x0010);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V3 UV COMPARATOR ASSERTED"));
  }
  else
  {
    Serial.print(F("\n  V3 UV COMPARATOR NOT ASSERTED"));
  }
  masked_val = (return_val & 0x0008);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V2 OV COMPARATOR ASSERTED"));
  }
  else
  {
    Serial.print(F("\n  V2 OV COMPARATOR NOT ASSERTED"));
  }
  masked_val = (return_val & 0x0004);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V2 UV COMPARATOR ASSERTED"));
  }
  else
  {
    Serial.print(F("\n  V2 UV COMPARATOR NOT ASSERTED"));
  }
  masked_val = (return_val & 0x0002);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V1 OV COMPARATOR ASSERTED"));
  }
  else
  {
    Serial.print(F("\n  V1 OV COMPARATOR NOT ASSERTED"));
  }
  masked_val = (return_val & 0x0001);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**V1 UV COMPARATOR ASSERTED"));
  }
  else
  {
    Serial.print(F("\n  V1 UV COMPARATOR NOT ASSERTED"));
  }
  return SUCCESS;
}


//! pretty-print STATUS_INFORMATION
int ltc2937_pretty_print_status_information(uint8_t device_address)
{
  uint16_t return_val,
           masked_val;

  return_val = smbus->readWord(device_address, LTC2937_STATUS_INFORMATION);

  Serial.print(F("\n LTC2937 STATUS_INFORMATION CONTENTS: "));
  Serial.println(return_val, HEX);

  masked_val = (return_val & 0xE000);
  switch (masked_val)
  {
    case 0xE000 :
      Serial.print(F("\n***RESERVED SEQUENCE STATE (THIS IS BAD)"));
      break;
    case 0xC000 :
      Serial.print(F("\n**SEQUENCE FAULT ON CHANNEL 6"));
      break;
    case 0xA000 :
      Serial.print(F("\n**SEQUENCE FAULT ON CHANNEL 5"));
      break;
    case 0x8000 :
      Serial.print(F("\n**SEQUENCE FAULT ON CHANNEL 4"));
      break;
    case 0x6000 :
      Serial.print(F("\n**SEQUENCE FAULT ON CHANNEL 3"));
      break;
    case 0x4000 :
      Serial.print(F("\n**SEQUENCE FAULT ON CHANNEL 2"));
      break;
    case 0x2000 :
      Serial.print(F("\n**SEQUENCE FAULT ON CHANNEL 1"));
      break;
    case 0x0000 :
      Serial.print(F("\n  NO SEQUENCE FAULTS"));
      break;
    default :
      Serial.print(F("\n**UNDEFINED SEQUENCE FAULT STATE (THIS IS BAD)"));
      break;
  }
  masked_val = (return_val & 0x1000);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**MONITOR BACKUP IS STORED IN EEPROM"));
  }
  else
  {
    Serial.print(F("\n  NO MONITOR BACKUP IN EEPROM"));
  }

  masked_val = (return_val & 0x0C00);
  switch (masked_val)
  {
    case 0x0C00 :
      Serial.print(F("\n  PART SEQUENCE-UP COMPLETE"));
      break;
    case 0x0800 :
      Serial.print(F("\n  PART SEQUENCE-DOWN IN PROGRESS"));
      break;
    case 0x0400 :
      Serial.print(F("\n  PART SEQUENCE-UP IN PROGRESS"));
      break;
    case 0x0000 :
      Serial.print(F("\n  PART SEQUENCE-DOWN COMPLETE"));
      break;
    default :
      Serial.print(F("\n**UNDEFINED PART SEQUENCING STATE (THIS IS BAD)"));
      break;
  }

  masked_val = (return_val & 0x0300);
  switch (masked_val)
  {
    case 0x0300 :
      Serial.print(F("\n  GROUP SEQUENCE-UP COMPLETE"));
      break;
    case 0x0200 :
      Serial.print(F("\n  GROUP SEQUENCE-DOWN IN PROGRESS"));
      break;
    case 0x0100 :
      Serial.print(F("\n  GROUP SEQUENCE-UP IN PROGRESS"));
      break;
    case 0x0000 :
      Serial.print(F("\n  GROUP SEQUENCE-DOWN COMPLETE"));
      break;
    default :
      Serial.print(F("\n**UNDEFINED GROUP SEQUENCING STATE (THIS IS BAD)"));
      break;
  }

  masked_val = (return_val & 0x0080);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**SEQUENCE-UP FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n  NO SEQUENCE-UP FAULT"));
  }
  masked_val = (return_val & 0x0040);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**SEQUENCE-DOWN FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n  NO SEQUENCE-DOWN FAULT"));
  }
  masked_val = (return_val & 0x0200);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**OV FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n  NO OV FAULT"));
  }
  masked_val = (return_val & 0x0010);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**UV FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n  NO UV FAULT"));
  }
  masked_val = (return_val & 0x0008);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**RESET FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n  NO RESET FAULT"));
  }
  masked_val = (return_val & 0x0004);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n  ALL SEQUENCED SUPPLIES ARE BELOW THEIR DISCHARGE THRESHOLDS"));
  }
  else
  {
    Serial.print(F("\n**NOT ALL SEQUENCED SUPPLIES ARE DOWN"));
  }
  masked_val = (return_val & 0x0002);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**CONTROL FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n  NO CONTROL FAULT"));
  }
  masked_val = (return_val & 0x0001);
  if (masked_val != 0x0000)
  {
    Serial.print(F("\n**OTHER FAULT DETECTED"));
  }
  else
  {
    Serial.print(F("\n  NO OTHER FAULT"));
  }
  return SUCCESS;
}


