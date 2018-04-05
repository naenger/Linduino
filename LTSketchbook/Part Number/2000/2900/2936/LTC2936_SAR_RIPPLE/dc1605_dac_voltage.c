/*! Utilities to talk to the LTC2637 DAC on the DC1633 and DC1605 demo boards
  Each DAC channel is tied to a channel on the demo board. The demo board has 6 channels, numbered 1 - 6.
  The DAC itself has 8 channels, labeled A - H. Here is the mapping:

  DAC CH | LTC2933 CH | LTC2936 CH
-----------------------------------
     A   |   N/C      |      1
     B   |    2       |      2 
     C   |    3       |      3 
     D   |    4       |      4 
     E   |    5       |      5 
     F   |    6       |      6 
     G   |   N/C      |     N/C
     H   |   N/C      |     N/C

 */
#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "UserInterface.h"
#include "LT_I2CBus.h"
#include "LT_I2CBus.c"
#include "LT_SMBusNoPec.h"
#include "dc1605_dac_voltage.h"

//! program the DAC on the DC1633B demo board to a voltage
//  refer to the LTC2637 datasheet
void dc1605_write_dac_voltage(uint8_t dac_address, int channel, float voltage)
{
  uint8_t *data = (uint8_t *)malloc(3*sizeof(uint8_t));
  uint8_t cmd, ch_addr;
  uint16_t v_data;
  float v;

  // pack the data bytes with the necessary bits
  // channel numbers 0 - 7 correspond to letters A - H in the datasheet
  if ((channel < 8) && (channel >= 0))
  {
    ch_addr = (uint8_t)channel;
  }
  else
  {
    //address all channels
    ch_addr = 0x0F;
  }
  cmd = 0x30; // the write to and update command
  data[0] = cmd | ch_addr;

  if ((voltage > 0) && (voltage < 4.096))
  {
    v = (voltage/4.096);
    v_data = (uint16_t)(v*4096);
  }
  else
  {
    //    Serial.println(F("\nERROR: Voltage out of DAC range"));
  }
  data[1] = (uint8_t)(v_data >> 4); // most significant bits
  data[2] = (uint8_t)(v_data << 4); // least significant bits

  //write the command and data to the DAC
  LT_Wire.beginTransmission(dac_address);
  LT_Wire.expectToWrite((uint16_t) 3);
  //  LT_Wire.write(command);
  LT_Wire.write(data[0]);
  LT_Wire.write(data[1]);
  LT_Wire.write(data[2]);
  LT_Wire.endTransmission(1);

  free(data);
  return;

}




//! write a random DAC voltage to ch_num on the DC1633B demo board
//! random voltages will be around mean and have maximum excursions of +/-ampl 
void dc1605_random_dac_voltage(uint8_t dac_address, int ch_num, float mean, float ampl) {
  float rand_voltage;

  rand_voltage = mean + ampl*2*(0.5 - (float)random(0,2048)/2048);
  //  Serial.print(F("RANDOM: "));
  //  Serial.println(rand_voltage);
  dc1633_write_dac_voltage(dac_address, ch_num, rand_voltage);


}
