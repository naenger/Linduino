
/* Copyright (c) 2014, Linear Technology Corp.(LTC)
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
ongoing work. */

/*
*  This is an example program to demonstrate use of the API with the Linduino.
*  Please create a folder called LTC2937_example_arduino containing the following files:
*    LTC2937_example_arduino.ino
*    LTC2937.c
*    LTC2937.h
*    LTC2937_reg_defs.h
*/

#include "LTC2937.h"
#include <Arduino.h>
#include "Linduino.h"

#include "UserInterface.h"
#include "LT_I2CBus.h"
#include "LT_SMBusNoPec.h"
//#include "LT_SMBusPec.h"

static LT_SMBus *smbus = new LT_SMBusNoPec();
//static LT_SMBus *smbus = new LT_SMBusPec();

LTC2937_chip_cfg_t cfg =  {
  LTC2937_ADDR_44, //.addr (7-bit)
  read_register, //.read_register
  write_register, //.write_register
  0 //.port_configuration not used
};

LTC2937 chip;
uint16_t data;

void setup()
{
  Serial.begin(115200);         //! Initialize the serial port to the PC
  while (!Serial);              //! Wait for serial port to be opened in the case of Leonardo USB Serial
  Serial.println(F("Initializing LTC2937"));
  chip = LTC2937_init(&cfg);
  
  /* part setup for blink demo */
  
  LTC2937_read_register(chip, LTC2937_CLEAR_ALERTB_CMD, &data);  // clear any old alerts, returned data meaningless //
  
  /* unlock the part */
  
  LTC2937_read_register(chip, LTC2937_HW_LOCK_BIT, &data); 
  if (data == LTC2937_HW_LOCK_BIT_PRESET_LOCKED)
  {Serial.println(F("Move WP jumper to unlock"));
  }

  while (data == LTC2937_HW_LOCK_BIT_PRESET_LOCKED)
  {
    LTC2937_read_register(chip, LTC2937_HW_LOCK_BIT, &data); 
  }
  
  LTC2937_read_modify_write_register(chip, LTC2937_SW_LOCK_BIT, LTC2937_SW_LOCK_BIT_PRESET_UNLOCKED);
  
  LTC2937_read_register(chip, LTC2937_SW_LOCK_BIT, &data);
  
  while (data == LTC2937_SW_LOCK_BIT_PRESET_LOCKED)
  {
    LTC2937_read_modify_write_register(chip, LTC2937_SW_LOCK_BIT, LTC2937_SW_LOCK_BIT_PRESET_UNLOCKED);
  }
  /* part should now be unlocked */
  Serial.println(F("Device unlocked"));
  
  /* setup the demo */
  
  LTC2937_read_modify_write_register(chip, LTC2937_RSTB_CONFIG_CMD, 0);  // do not combine comparators into rstb response //
  
  LTC2937_read_modify_write_register(chip, LTC2937_FAULT_RESPONSE_CMD, 0); // clear old register settings //
  LTC2937_read_modify_write_register(chip, LTC2937_FAULT_RESPONSE_ACTION, LTC2937_FAULT_RESPONSE_ACTION_PRESET_DISCHARGED_RETRY);
  
  LTC2937_read_modify_write_register(chip, LTC2937_ON_OFF_CONTROL_CMD, 0); // clear the whole register //
  LTC2937_read_modify_write_register(chip, LTC2937_I2C_ON_OFF_MASK, LTC2937_I2C_ON_OFF_MASK_PRESET_I2C_LISTEN); // control sequence through I2C //
  LTC2937_read_modify_write_register(chip, LTC2937_SEQ_DOWN_QUAL, LTC2937_SEQ_DOWN_QUAL_PRESET_TIME_BASED); // advance sequence down positions by time //
  
  LTC2937_clear(); // clear the state machines //
  
  LTC2937_read_modify_write_register(chip, LTC2937_V6_RANGE, LTC2937_V6_RANGE_PRESET_POSITIVE_ADJUSTABLE); // 0.2V to 1.2V range //
  LTC2937_read_modify_write_register(chip, LTC2937_V5_RANGE, LTC2937_V5_RANGE_PRESET_POSITIVE_ADJUSTABLE); // 0.2V to 1.2V range //
  LTC2937_read_modify_write_register(chip, LTC2937_V4_RANGE, LTC2937_V4_RANGE_PRESET_POSITIVE_ADJUSTABLE); // 0.2V to 1.2V range //
  LTC2937_read_modify_write_register(chip, LTC2937_V3_RANGE, LTC2937_V3_RANGE_PRESET_POSITIVE_ADJUSTABLE); // 0.2V to 1.2V range //
  LTC2937_read_modify_write_register(chip, LTC2937_V2_RANGE, LTC2937_V2_RANGE_PRESET_POSITIVE_ADJUSTABLE); // 0.2V to 1.2V range //
  LTC2937_read_modify_write_register(chip, LTC2937_V1_RANGE, LTC2937_V1_RANGE_PRESET_POSITIVE_ADJUSTABLE); // 0.2V to 1.2V range //
  
  LTC2937_read_modify_write_register(chip, LTC2937_V_THRESHOLD_1_CMD, 0x6969); // set UV and OV comparators to 0.6V threshold //
  LTC2937_read_modify_write_register(chip, LTC2937_V_THRESHOLD_2_CMD, 0x6969); // set UV and OV comparators to 0.6V threshold //
  LTC2937_read_modify_write_register(chip, LTC2937_V_THRESHOLD_3_CMD, 0x6969); // set UV and OV comparators to 0.6V threshold //
  LTC2937_read_modify_write_register(chip, LTC2937_V_THRESHOLD_4_CMD, 0x6969); // set UV and OV comparators to 0.6V threshold //
  LTC2937_read_modify_write_register(chip, LTC2937_V_THRESHOLD_5_CMD, 0x6969); // set UV and OV comparators to 0.6V threshold //
  LTC2937_read_modify_write_register(chip, LTC2937_V_THRESHOLD_6_CMD, 0x6969); // set UV and OV comparators to 0.6V threshold //
  
  LTC2937_read_modify_write_register(chip, LTC2937_TON_MAX_1, LTC2937_TON_MAX_1_PRESET_INFINITY);
  LTC2937_read_modify_write_register(chip, LTC2937_TON_DELAY_1, 2500); // 200ms //
  LTC2937_read_modify_write_register(chip, LTC2937_TON_MAX_2, LTC2937_TON_MAX_2_PRESET_INFINITY);
  LTC2937_read_modify_write_register(chip, LTC2937_TON_DELAY_2, 2500); // 200ms //
  LTC2937_read_modify_write_register(chip, LTC2937_TON_MAX_3, LTC2937_TON_MAX_3_PRESET_INFINITY);
  LTC2937_read_modify_write_register(chip, LTC2937_TON_DELAY_3, 2500); // 200ms //
  LTC2937_read_modify_write_register(chip, LTC2937_TON_MAX_4, LTC2937_TON_MAX_4_PRESET_INFINITY);
  LTC2937_read_modify_write_register(chip, LTC2937_TON_DELAY_4, 2500); // 200ms //
  LTC2937_read_modify_write_register(chip, LTC2937_TON_MAX_5, LTC2937_TON_MAX_5_PRESET_INFINITY);
  LTC2937_read_modify_write_register(chip, LTC2937_TON_DELAY_5, 2500); // 200ms //
  LTC2937_read_modify_write_register(chip, LTC2937_TON_MAX_6, LTC2937_TON_MAX_6_PRESET_INFINITY);
  LTC2937_read_modify_write_register(chip, LTC2937_TON_DELAY_6, 2500); // 200ms //
  
  LTC2937_read_modify_write_register(chip, LTC2937_TOFF_MAX_1, LTC2937_TOFF_MAX_1_PRESET_164MS); 
  LTC2937_read_modify_write_register(chip, LTC2937_TOFF_DELAY_1, 450); // 36ms added to 164ms enable delay //		
  LTC2937_read_modify_write_register(chip, LTC2937_TOFF_MAX_2, LTC2937_TOFF_MAX_2_PRESET_164MS);
  LTC2937_read_modify_write_register(chip, LTC2937_TOFF_DELAY_2, 450); // 36ms added to 164ms enable delay //
  LTC2937_read_modify_write_register(chip, LTC2937_TOFF_MAX_3, LTC2937_TOFF_MAX_3_PRESET_164MS);
  LTC2937_read_modify_write_register(chip, LTC2937_TOFF_DELAY_3, 450); // 36ms added to 164ms enable delay //
  LTC2937_read_modify_write_register(chip, LTC2937_TOFF_MAX_4, LTC2937_TOFF_MAX_4_PRESET_164MS);
  LTC2937_read_modify_write_register(chip, LTC2937_TOFF_DELAY_4, 450); // 36ms added to 164ms enable delay //
  LTC2937_read_modify_write_register(chip, LTC2937_TOFF_MAX_5, LTC2937_TOFF_MAX_5_PRESET_164MS);
  LTC2937_read_modify_write_register(chip, LTC2937_TOFF_DELAY_5, 450); // 36ms added to 164ms enable delay //
  LTC2937_read_modify_write_register(chip, LTC2937_TOFF_MAX_6, LTC2937_TOFF_MAX_6_PRESET_164MS);
  LTC2937_read_modify_write_register(chip, LTC2937_TOFF_DELAY_6, 450); // 36ms added to 164ms enable delay //
  
  LTC2937_read_modify_write_register(chip, LTC2937_SEQ_UP_POSITION_1_CMD, 1); // sequence up position //
  LTC2937_read_modify_write_register(chip, LTC2937_SEQ_UP_POSITION_2_CMD, 2); // sequence up position //
  LTC2937_read_modify_write_register(chip, LTC2937_SEQ_UP_POSITION_3_CMD, 3); // sequence up position //
  LTC2937_read_modify_write_register(chip, LTC2937_SEQ_UP_POSITION_4_CMD, 3); // sequence up position //
  LTC2937_read_modify_write_register(chip, LTC2937_SEQ_UP_POSITION_5_CMD, 2); // sequence up position //
  LTC2937_read_modify_write_register(chip, LTC2937_SEQ_UP_POSITION_6_CMD, 1); // sequence up position //
  
  LTC2937_read_modify_write_register(chip, LTC2937_SEQ_DOWN_POSITION_1_CMD, 3); // sequence down position //
  LTC2937_read_modify_write_register(chip, LTC2937_SEQ_DOWN_POSITION_2_CMD, 2); // sequence down position //
  LTC2937_read_modify_write_register(chip, LTC2937_SEQ_DOWN_POSITION_3_CMD, 1); // sequence down position //
  LTC2937_read_modify_write_register(chip, LTC2937_SEQ_DOWN_POSITION_4_CMD, 1); // sequence down position //
  LTC2937_read_modify_write_register(chip, LTC2937_SEQ_DOWN_POSITION_5_CMD, 2); // sequence down position //
  LTC2937_read_modify_write_register(chip, LTC2937_SEQ_DOWN_POSITION_6_CMD, 3); // sequence down position //
  
  LTC2937_read_modify_write_register(chip, LTC2937_ACTIVE_PULL_DOWN_1, LTC2937_ACTIVE_PULL_DOWN_1_PRESET_PULL_DOWN_ENABLED);
  LTC2937_read_modify_write_register(chip, LTC2937_ACTIVE_PULL_DOWN_2, LTC2937_ACTIVE_PULL_DOWN_2_PRESET_PULL_DOWN_ENABLED);
  LTC2937_read_modify_write_register(chip, LTC2937_ACTIVE_PULL_DOWN_3, LTC2937_ACTIVE_PULL_DOWN_3_PRESET_PULL_DOWN_ENABLED);
  LTC2937_read_modify_write_register(chip, LTC2937_ACTIVE_PULL_DOWN_4, LTC2937_ACTIVE_PULL_DOWN_4_PRESET_PULL_DOWN_ENABLED);
  LTC2937_read_modify_write_register(chip, LTC2937_ACTIVE_PULL_DOWN_5, LTC2937_ACTIVE_PULL_DOWN_5_PRESET_PULL_DOWN_ENABLED);
  LTC2937_read_modify_write_register(chip, LTC2937_ACTIVE_PULL_DOWN_6, LTC2937_ACTIVE_PULL_DOWN_6_PRESET_PULL_DOWN_ENABLED);
  
}

void loop()
{
  LTC2937_read_register(chip, LTC2937_LOCAL_SEQ_STATUS, &data);
  
  if (data != LTC2937_LOCAL_SEQ_STATUS_PRESET_SEQUENCE_DOWN_DONE)
  {
    Serial.println(F("Waiting for sequence down"));
  }
  while (data != LTC2937_LOCAL_SEQ_STATUS_PRESET_SEQUENCE_DOWN_DONE)
  {
    LTC2937_read_register(chip, LTC2937_LOCAL_SEQ_STATUS, &data);
  }
  
  LTC2937_read_register(chip, LTC2937_DISCHARGE, &data); 
  if (data != LTC2937_DISCHARGE_PRESET_ALL_SEQUENCED_CHANNELS_ARE_BELOW_DISCHARGE_THRESHOLDS)
  {
    Serial.println(F("Waiting for discharge"));
  }
  while (data != LTC2937_DISCHARGE_PRESET_ALL_SEQUENCED_CHANNELS_ARE_BELOW_DISCHARGE_THRESHOLDS)
  {
    LTC2937_read_register(chip, LTC2937_DISCHARGE, &data);
  }

  /* sequence up */

  LTC2937_read_modify_write_register(chip, LTC2937_I2C_ON_OFF, LTC2937_I2C_ON_OFF_PRESET_SEQUENCE_UP);
  
  LTC2937_read_register(chip, LTC2937_LOCAL_SEQ_STATUS, &data);
  
  while (data != LTC2937_LOCAL_SEQ_STATUS_PRESET_SEQUENCE_UP_DONE)
  {
    LTC2937_read_register(chip, LTC2937_LOCAL_SEQ_STATUS, &data);
  }

  /* sequence down */
  LTC2937_read_modify_write_register(chip, LTC2937_I2C_ON_OFF, LTC2937_I2C_ON_OFF_PRESET_SEQUENCE_DOWN);
  /* repeat */
}

/* read_register function wraps C++ method LT_SMBus::readWord and places the returned data in *data.*/
int read_register(uint8_t addr,uint8_t command_code, uint16_t *data, port_configuration_t *port_configuration) {
  //virtual uint8_t LT_SMBus::readByte(uint8_t address,uint8_t command);
  //virtual uint16_t LT_SMBus::readWord(uint8_t address,uint8_t command);
  *data = smbus->readWord(addr, command_code);
  return 0;
}

/* write_register function wraps C++ method LT_SMBus::writeWord.*/
int write_register(uint8_t addr,uint8_t command_code, uint16_t data, port_configuration_t *port_configuration) {
  //virtual void LT_SMBus::writeByte(uint8_t address,uint8_t command,uint8_t data);
  //virtual void LT_SMBus::writeWord(uint8_t address,uint8_t command,uint16_t data);
  smbus->writeWord(addr, command_code, data);
  return 0; 
}

uint8_t LTC2937_store()  // Write volatile memory into EEPROM //
{
  smbus->sendByte(LTC2937_ADDR_44, LTC2937_STORE_CMD);
}
uint8_t LTC2937_restore()  // Move EEPROM contents to volatile memory //
{
  smbus->sendByte(LTC2937_ADDR_44, LTC2937_RESTORE_CMD);
}
uint8_t LTC2937_clear()  // clear the state machines //
{
  smbus->sendByte(LTC2937_ADDR_44, LTC2937_CLEAR_CMD);
}
