/*
Demonstrate communication with the TI PCF8575C I2C-to-GPIO chip
 */

#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "UserInterface.h"
//#include "LT_I2CBus.h"
#include "LT_Wire.h"

#define PCF8575C_I2C_ADDRESS 0x20  //default 7-bit address

/********************************************************************************/
// Global Variables
uint8_t pcf8575_i2c_address;

uint8_t incoming_data[2];  //array of 2 bytes
uint8_t outgoing_data[2];  // array of 2 bytes
uint8_t io_data_mask[2];  // array of 2 bytes -- 1 = input, 0 = output


/****************************************************************************/
//! Initialize Linduino
void setup()
{

  uint16_t return_val;

  pcf8575_i2c_address = PCF8575C_I2C_ADDRESS;

  io_data_mask[0] = 0xFF;
  io_data_mask[1] = 0xFF;
  outgoing_data[1] = 0xAA;
  outgoing_data[0] = 0x00;



  Serial.begin(115200);         //! Initialize the serial port to the PC

  Serial.print (F("\n"));
  Serial.print (F("STARTING\n"));

}


/****************************************************************************/
//! Main Linduino Loop
void loop()
{
  // all inputs are assumed open-drain
  // so they are pulled-up externally
  // if somebody external wants them to be low, they are pulled low (actively)
  // in writing we write inputs to 1, which has no effect if the above is true
  // in reading we will see the logical state of all pins, both inputs and outputs

  outgoing_data[1] = outgoing_data[0];
  outgoing_data[0] = outgoing_data[0] + 0x01;


  send_pcf8575_pins(pcf8575_i2c_address, outgoing_data, io_data_mask, 2);

  get_pcf8575_pins(pcf8575_i2c_address, incoming_data);
  Serial.print(pcf8575_i2c_address, HEX);
  Serial.print(F("\t"));
  Serial.print(incoming_data[0], HEX);
  Serial.print(F("\t"));
  Serial.println(incoming_data[1], HEX);
  Serial.print(F("\n"));
  delay(1000);

}

//! Read 2 bytes as the two 8-bit input words, P07, P06, ..., P00, P17, ..., P10
int get_pcf8575_pins(uint8_t device_addr, uint8_t *data_array)
{
  LT_Wire.beginTransmission(device_addr);
  //  LT_Wire.expectToWrite((uint16_t) 1);
  //  LT_Wire.write(command);
  //  ret = LT_Wire.endTransmission(false);
  //  LT_Wire.beginTransmission(address);
  LT_Wire.requestFrom(device_addr, data_array, (uint16_t)2);

  return 0;
}

int send_pcf8575_pins(uint8_t device_addr, uint8_t *data_array, uint8_t *io_data_mask, int length)
{
  uint8_t *data = (uint8_t *)malloc(length*sizeof(uint8_t));

  int i;

  for (i = 0; i < length; i++)
  {
    data[i] = data_array[i] | io_data_mask[i];
  }
  //  twi_writeTo(device_addr, data, length, 0x00, 0x01);

  LT_Wire.beginTransmission(device_addr);
  LT_Wire.expectToWrite((uint16_t) 2);
  //  LT_Wire.write(command);
  LT_Wire.write(data_array[0]);
  LT_Wire.write(data_array[1]);
  LT_Wire.endTransmission(1);
  free(data);
  return 0;
}
