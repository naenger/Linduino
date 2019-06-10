/*!
LTC2947_65: LTC2947_65 a high-precision power and energy monitor with an internal sense resistor supporting up to 30A

@verbatim

The LTC2947_65 is a high-precision power and energy
monitor with an internal sense resistor supporting up
to 30A. Three internal No Latency delta sigma ADCs ensure
accurate measurement of voltage and current, while high-
bandwidth analog multiplication of voltage and current
provides accurate power measurement in a wide range of
applications. Internal or external clocking options enable
precise charge and energy measurements.
An internal 300 micro ohms, temperature-compensated sense
resistor minimizes efficiency loss and external compo-
nents, simplifying energy measurement applications while
enabling high accuracy current measurement over the full
temperature range. For more details see following URLs:

@endverbatim

http://www.linear.com/product/LTC2947_65

http://www.linear.com/product/LTC2947_65#demoboards


Copyright 2018(c) Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in
   the documentation and/or other materials provided with the
   distribution.
 - Neither the name of Analog Devices, Inc. nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.
 - The use of this software may or may not infringe the patent rights
   of one or more patent holders.  This license does not release you
   from the requirement that you obtain separate licenses from these
   patent holders to use this software.
 - Use of the software either in source or binary form, must be run
   on or directly connected to an Analog Devices Inc. component.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//! @ingroup Power_Monitors
//! @{
//! @defgroup LTC2947_65 LTC2947_65 a high-precision power and energy monitor with an internal sense resistor supporting up to 30A.
//! @}

/*! @file
  @ingroup LTC2947_65
  Library for LTC2947_65: A high-precision power and energy monitor with an internal sense resistor supporting up to 30A.
*/

#include <Arduino.h>
#include <Linduino.h>
#include <LT_I2C.h>
#include <LT_SPI.h>
#include "LTC2947_65.h"
#include <SPI.h>

#ifdef LTC2947_65_DEBUG
#include "UserInterface.h"
#endif

boolean LTC2947_65_SPI_Mode_Enabled = false;
uint8_t LTC2947_65_I2C_Slave_Addr = LTC2947_65_I2C_ADDR_LL;

void LTC2947_65_InitI2C(uint8_t slvAddr)
{
  LTC2947_65_SPI_Mode_Enabled = false;
  LTC2947_65_I2C_Slave_Addr = slvAddr;
}

void LTC2947_65_InitSPI()
{
  LTC2947_65_SPI_Mode_Enabled = true;
}

boolean LTC2947_65_Abs(uint8_t *bytes, uint8_t length)
{
  if (bitMaskClrChk(*bytes, 0x80))
    return false;// value is already positive

  // two's complement is generated by inverting all bits and add 1

  length--;
  bytes += length; // seek to LSB
  uint16_t cHelp = (~(*bytes)) & 0xFF; // invert LSB
  cHelp++; // add 1
  *bytes = cHelp; // store back to buffer

  while (length != 0)
  {
    // seek next byte (towards MSB)
    length--;
    bytes--;
    cHelp = cHelp >> 8; // restore carry from previous sum
    cHelp += (~(*bytes)) & 0xFF; // add inverted byte
    *bytes = cHelp; // store back
  }
  return true;// value inverted
}

double LTC2947_65_BytesToDouble(uint8_t *bytes, uint8_t length, boolean sig, double lsb)
{
  if (length == 0)
    return 0.0;
  else if (length == 1)
    return sig ? (int8_t)(bytes[0])*lsb : bytes[0] * lsb;
  else if (length == 2)
    return sig ? LTC2947_65_2BytesToInt16(bytes)*lsb : LTC2947_65_2BytesToUInt16(bytes)*lsb;
  else if (length == 3)
    return sig ? LTC2947_65_3BytesToInt32(bytes)*lsb : LTC2947_65_3BytesToUInt32(bytes)*lsb;
  else if (length == 4)
    return sig ? LTC2947_65_4BytesToInt32(bytes)*lsb : LTC2947_65_4BytesToUInt32(bytes)*lsb;
  else
    return sig ? LTC2947_65_SignedBytesToDouble(bytes, length, lsb) : LTC2947_65_UnsignedBytesToDouble(bytes, length, lsb);
}

double LTC2947_65_UnsignedBytesToDouble(uint8_t *unsignedBytes, uint8_t length, double lsb)
{
  // NOTE: On Arduino double is 32-bit and NOT 64-bit, thus the returned value
  // will not reflect the full precission of e.g. C1, C2... which are 48-bit values
  double ret = (*unsignedBytes); // MSB!

  while (length > 1)
  {
    unsignedBytes++; // go to next byte
    length--;
    ret = ret * 256.0 + (*unsignedBytes);
  }
  return ret*lsb;
}

void LTC2947_65_SerialPrint8hex(uint8_t val)
{
  if (val < 0x10) Serial.print("0");
  Serial.print(val, HEX);
}
void LTC2947_65_SerialPrint16hex(uint16_t val)
{
  for (uint16_t i = 0x1000L; i >= 0x10L; i = i >> 4)
    if (val < i) Serial.print("0");
  Serial.print(val, HEX);
}
void LTC2947_65_SerialPrint32hex(uint32_t val)
{
  for (uint32_t i = 0x10000000L; i >= 0x10L; i = i >> 4)
    if (val < i) Serial.print("0");
  Serial.print(val, HEX);
}
void LTC2947_65_SerialPrint64hex(uint64_t uint64Val)
{
  LTC2947_65_SerialPrint32hex((uint32_t)(uint64Val >> 32));
  LTC2947_65_SerialPrint32hex((uint32_t)(uint64Val));
}

void LTC2947_65_DoubleToBytes(double value, double lsb, uint8_t *bytes, uint8_t length)
{
  //! on Arduino Uno / Linduino the maximum is 8 bytes
  if (length > sizeof(int64_t)) //! sizeof(int64_t) = 8
    return;

  //! revert the scaling with LSB and convert to integer value
  int64_t int64Val = int64_t(value / lsb);

#ifdef LTC2947_65_DEBUG
  Serial.print(F("int64Val=0x"));
  LTC2947_65_SerialPrint64hex((uint64_t)(int64Val));
  Serial.println();
  Serial.println(F("bytes:"));
#endif

  //! convert the integer value to byte array
  for (int8_t i = length - 1; i >= 0; i--)
  {
    bytes[i] = int64Val & 0xFF;
    int64Val >>= 8;
#ifdef LTC2947_65_DEBUG
    Serial.print(i);
    Serial.print(F(":"));
    LTC2947_65_SerialPrint8hex(bytes[i]);
    if (i == 0)
      Serial.print(F(" (MSB)"));
    else if (i == length - 1)
      Serial.print(F(" (LSB)"));
    Serial.println();
#endif
  }
}

#ifdef LTC2947_65_DEBUG
//! conversion function test
void LTC2947_65_DoubleToBytes_Test()
{
  Serial.print(F("LSB:"));
  while (!Serial.available());
  double lsb = read_float();
  Serial.println(lsb, 8);

  Serial.print(F("LSBrshift:"));
  while (!Serial.available());
  int32_t shift = read_int();
  Serial.println(shift);
  while (shift-- > 0)
    lsb = lsb * 0.5;

  Serial.println(lsb, 16);

  Serial.print(F("VAL:"));
  while (!Serial.available());
  double val = read_float();
  Serial.println(val, 8);

  Serial.print(F("VALlshift:"));
  while (!Serial.available());
  shift = read_int();
  Serial.println(shift);
  while (shift-- > 0)
    val = val * 2.0;

  Serial.println(val, 16);

  byte bytes[8];
  LTC2947_65_DoubleToBytes(val, lsb, bytes, 8);
}
#endif


double LTC2947_65_SignedBytesToDouble(uint8_t *signedBytes, uint8_t length, double lsb)
{
  // reserve memory for unsigned bytes
  uint8_t *unsignedBytes = (uint8_t *)malloc(length);
  // copy signed bytes to unsigned bytes
  memcpy(unsignedBytes, signedBytes, length);
  // calculate absolute value of the signed bytes and store sign
  // this function will change the unsigned bytes, for this reason
  // we copied the original unsigned bytes to a new array
  boolean sign = LTC2947_65_Abs(unsignedBytes, length);
  // convert the unsigned bytes to a double value
  double absDouble = LTC2947_65_UnsignedBytesToDouble(unsignedBytes, length, lsb);
  // free the allocated memory of the copied array
  free(unsignedBytes);
  // recover the previously stored sign to return a signed value
  return sign ? -absDouble : absDouble;
}

int32_t LTC2947_65_4BytesToInt32(byte *bytes)
{
  int32_t ret;

  ret = *bytes;
  bytes++;
  ret = ret << 8;
  ret |= *bytes;
  bytes++;
  ret = ret << 8;
  ret |= *bytes;
  bytes++;
  ret = ret << 8;
  ret |= *bytes;
  return ret;
}

int32_t LTC2947_65_3BytesToInt32(byte *bytes)
{
  int32_t ret;
  // sign extension
  if (*bytes & 0x80)
    ret = 0xFF00;
  else
    ret = 0;

  ret |= *bytes;
  bytes++;
  ret = ret << 8;
  ret |= *bytes;
  bytes++;
  ret = ret << 8;
  ret |= *bytes;
  return ret;
}

int16_t LTC2947_65_2BytesToInt16(byte *bytes)
{
  int16_t ret;
  ret = *bytes;
  bytes++;
  ret = ret << 8;
  ret |= *bytes;
  return ret;
}

uint32_t LTC2947_65_4BytesToUInt32(byte *bytes)
{
  uint32_t ret;

  ret = *bytes;
  bytes++;
  ret = ret << 8;
  ret |= *bytes;
  bytes++;
  ret = ret << 8;
  ret |= *bytes;
  bytes++;
  ret = ret << 8;
  ret |= *bytes;
  return ret;
}

uint32_t LTC2947_65_3BytesToUInt32(byte *bytes)
{
  uint32_t ret;

  ret = *bytes;
  bytes++;
  ret = ret << 8;
  ret |= *bytes;
  bytes++;
  ret = ret << 8;
  ret |= *bytes;
  return ret;
}

uint16_t LTC2947_65_2BytesToUInt16(byte *bytes)
{
  uint16_t ret;
  ret = *bytes;
  bytes++;
  ret = ret << 8;
  ret |= *bytes;
  return ret;
}

/*
* Note on I2C/SPI functions:
* i2c_write / i2c_read / spi_write / spi_read block methods from LT_I2C / LT_SPI write / read byte arrays from last to first element
*  which is not compatible with LTC2947_65 library that expects the opposite order
*/

int8_t LTC2947_65_SpiWrBlock(uint8_t address, uint8_t length, uint8_t *values)
{
  int8_t i;

  output_low(LTC2947_65_CS);                 //! 1) Pull CS low

  SPI.transfer(LTC2947_65_SPI_WRITE_CMD); // write
  SPI.transfer(address); // reg addr

  for (i = 0; i < length; i++)
    SPI.transfer(values[i]);    //! 2) send byte array

  output_high(LTC2947_65_CS);                //! 3) Pull CS high
  return 0;
}

int8_t LTC2947_65_SpiRdBlock(uint8_t address, uint8_t length, uint8_t *values)
{
  int8_t i;

  output_low(LTC2947_65_CS);                 //! 1) Pull CS low

  SPI.transfer(LTC2947_65_SPI_READ_CMD); // read
  SPI.transfer(address); // reg addr

  for (i = 0; i < length; i++)
    values[i] = SPI.transfer(0x00);

  output_high(LTC2947_65_CS);                //! 3) Pull CS high
  return 0;
}

int8_t LTC2947_65_SpiWrByte(uint8_t address, uint8_t value)
{
  output_low(LTC2947_65_CS);                 //! 1) Pull CS low

  SPI.transfer(LTC2947_65_SPI_WRITE_CMD); // write
  SPI.transfer(address); // reg addr
  SPI.transfer(value);    //! 2) send byte

  output_high(LTC2947_65_CS);                //! 3) Pull CS high
  return 0;
}

int8_t LTC2947_65_SpiRdByte(uint8_t address, uint8_t *value)
{
  output_low(LTC2947_65_CS);                 //! 1) Pull CS low

  SPI.transfer(LTC2947_65_SPI_READ_CMD); // read
  SPI.transfer(address); // reg addr
  value[0] = SPI.transfer(0x00);    //! 2) read byte

  output_high(LTC2947_65_CS);                //! 3) Pull CS high
  return 0;
}

int8_t LTC2947_65_I2CWrBlock(uint8_t slvAddr, uint8_t regAddr, uint8_t length, uint8_t *values)
{
  int8_t ret = 0;

  if (i2c_start() != 0) //I2C START
    return 1;         //Stop and return 0 if START fail

  ret |= i2c_write((slvAddr << 1) | I2C_WRITE_BIT); // Write 7 bit address with W bit
  ret |= i2c_write(regAddr);                        // Set register address

  while (length > 0)
  {
    ret |= i2c_write(*values);     //Write Value
    length--;
    values++;
  }

  i2c_stop();                        // I2C STOP

  return ret != 0 ? 1 : 0;
}

int8_t LTC2947_65_I2CRdBlock(uint8_t slvAddr, uint8_t regAddr, uint8_t length, uint8_t *values)
{
  int8_t ret = 0;

  if (length == 0 || i2c_start() != 0) //I2C START
    return 1; //Stop and return 0 if START fail

  ret |= i2c_write((slvAddr << 1) | I2C_WRITE_BIT); // Write 7 bit address with W bit
  ret |= i2c_write(regAddr);                        // Set register address
  ret |= i2c_repeated_start();
  ret |= i2c_write((slvAddr << 1) | I2C_READ_BIT); // Write 7 bit address with R bit

  if (ret != 0)   //If NACK return 1
  {
    i2c_stop();                         //I2C STOP
    return 1;
  }

  length--;
  while (length > 0)
  {
    *values = i2c_read(WITH_ACK); //Read from bus with ACK
    values++;
    length--;
  }

  *values = i2c_read(WITH_NACK); //Read from bus with NACK for the last one;

  i2c_stop(); //I2C STOP

  return 0; // Success!
}

int8_t LTC2947_65_I2CWrByte(uint8_t slvAddr, uint8_t regAddr, uint8_t value)
{
  int8_t ret = 0;

  if (i2c_start() != 0) //I2C START
    return 1;        //Stop and return 0 if START fail

  ret |= i2c_write((slvAddr << 1) | I2C_WRITE_BIT); // Write 7 bit address with W bit
  ret |= i2c_write(regAddr);                        // Set register address
  ret |= i2c_write(value);     //Write Value

  i2c_stop();                  // I2C STOP

  return ret != 0 ? 1 : 0;
}

int8_t LTC2947_65_I2CRdByte(uint8_t slvAddr, uint8_t regAddr, uint8_t *value)
{
  int8_t ret = 0;

  if (i2c_start() != 0) //I2C START
    return 1; //Stop and return 0 if START fail

  ret |= i2c_write((slvAddr << 1) | I2C_WRITE_BIT); // Write 7 bit address with W bit
  ret |= i2c_write(regAddr);                        // Set register address
  ret |= i2c_repeated_start();
  ret |= i2c_write((slvAddr << 1) | I2C_READ_BIT); // Write 7 bit address with R bit

  if (ret != 0)   //If NACK return 1
  {
    i2c_stop();                         //I2C STOP
    return 1;
  }

  *value = i2c_read(WITH_NACK); //Read from bus with NACK for the last one;

  i2c_stop(); //I2C STOP

  return 0; // Success!
}

void LTC2947_65_GPIO_PinMode(uint8_t mode)
{
  uint8_t gpiostatcl;
  LTC2947_65_RD_BYTE(LTC2947_65_REG_GPIOSTATCL, &gpiostatcl);
  bitMaskSetClr(gpiostatcl, LTC2947_65_BM_GPIOSTATCL_GPOEN, mode != INPUT);
  LTC2947_65_WR_BYTE(LTC2947_65_REG_GPIOSTATCL, gpiostatcl);
}

void LTC2947_65_GPIO_SetPinState(uint8_t val)
{
  uint8_t gpiostatcl;
  LTC2947_65_RD_BYTE(LTC2947_65_REG_GPIOSTATCL, &gpiostatcl);
  bitMaskSetClr(gpiostatcl, LTC2947_65_BM_GPIOSTATCL_GPO, val != LOW);
  LTC2947_65_WR_BYTE(LTC2947_65_REG_GPIOSTATCL, gpiostatcl);
}

boolean LTC2947_65_GPIO_Read()
{
  uint8_t gpiostatcl;
  LTC2947_65_RD_BYTE(LTC2947_65_REG_GPIOSTATCL, &gpiostatcl);
  return bitMaskSetChk(gpiostatcl, LTC2947_65_BM_GPIOSTATCL_GPI);
}

uint8_t LTC2947_65_Ara(uint8_t *svlAddr)
{
  *svlAddr = 0;
  //! Send I2C start bit
  if (i2c_start() != 0)
    return LTC2947_65_ARA_ERROR;

  //! send the ALERT RESPONSE ADDRESS with read bit
  if (i2c_write((LTC2947_65_ALERT_RESP_ADDR << 1) | I2C_READ_BIT))
  {
    //! NACK means no device response!
    return LTC2947_65_ARA_NO_RESPONSE;
  }

  //! Read device address from the responding device
  *svlAddr = i2c_read(WITH_NACK); //! read with NACK
  i2c_stop(); //! I2C STOP
  //! check for the expected write bit of the response
  boolean response_wr_bit = bitMaskClrChk(*svlAddr, 0x1);
  //! right shift to get 7-bit slave address
  *svlAddr = ((*svlAddr) >> 1) & 0x7F;
  if (response_wr_bit)
  {
    //! got expected write bit, compare slave address
    //! with LTC2947_65's address
    return ((*svlAddr) == LTC2947_65_I2C_Slave_Addr)
           ? LTC2947_65_ARA_LTC2947_65_RESPONSE
           : LTC2947_65_ARA_OTHER_RESPONSE;
  }

  //! missing write bit within response!
  return LTC2947_65_ARA_RESPONSE_WO_WR;
}

int16_t LTC2947_65_wake_up()
{
  byte data[1];
  unsigned long wakeupStart = millis(), wakeupTime;
  LTC2947_65_WR_BYTE(LTC2947_65_REG_OPCTL, 0);//! any serial transaction will wakeup LTC2947_65
  do
  {
    delay(1);
    LTC2947_65_RD_BYTE(LTC2947_65_REG_OPCTL, data); //! wake up polling by reading OPCTL
    wakeupTime = millis() - wakeupStart;
    if (data[0] == 0) //! check if we are in idle mode
    {
      //! wake up successful, return wakeup time in milliseconds
      return wakeupTime;
    }
    if (wakeupTime > 200)
    {
      //! failed to wake up due to timeout, return -1
      return -1;
    }
  }
  while (true);
}

boolean LTC2947_65_GetCurrentPageSelect()
{
  uint8_t currentPageCtrl;
  LTC2947_65_RD_BYTE(LTC2947_65_REG_PGCTL, &currentPageCtrl);
  return bitMaskSetChk(currentPageCtrl, LTC2947_65_BM_PGCTL_PAGE);
}

void LTC2947_65_SetPageSelect(boolean page)
{
  LTC2947_65_WR_BYTE(LTC2947_65_REG_PGCTL, page ? LTC2947_65_BM_PGCTL_PAGE : 0); // switch page
}

void LTC2947_65_Read_I_P_V_TEMP_VCC(float *I, float *P, float *V, float *TEMP, float *VCC)
{
  // byte array to store register values
  byte bytes[12];

  // read measurement results from device
  LTC2947_65_RD_BYTES(LTC2947_65_VAL_I, 6, bytes);      // I[23:0] P[23:0] starting at data[0]
  LTC2947_65_RD_BYTES(LTC2947_65_VAL_V, 6, bytes + 6);  // V[15:0] TEMP[15:0] VDVCC[15:0] starting at data[6]

  // convert to floating point values
  // Note: definitions in LTC2947_65.h are given in mA, mW, mV
  // so they have to be mutliplied by 1e-3 to get A, W, V respectively
  *I = LTC2947_65_3BytesToInt32(bytes) * LTC2947_65_LSB_I * 1e-3;                            // calc current in amps
  *P = LTC2947_65_3BytesToInt32(bytes + 3) * LTC2947_65_LSB_P * 1e-3;                        // calc power in watts
  *V = LTC2947_65_2BytesToInt16(bytes + 6) * LTC2947_65_LSB_V * 1e-3;                        // calc voltage in volts
  *TEMP = LTC2947_65_2BytesToInt16(bytes + 6 + 2) * LTC2947_65_LSB_TEMP + LTC2947_65_OFFS_TEMP; // calc temperature in degree celcius
  *VCC = LTC2947_65_2BytesToInt16(bytes + 6 + 4) * LTC2947_65_LSB_VDVCC * 1e-3;              // calc supply voltage in volts
}

void LTC2947_65_Read_Abs_C_E_TB(boolean accuSet1, double *C, boolean *signC, double *E, boolean *signE, double *TB)
{
  // byte array to store register values
  byte bytes[16];

  // read measurement results from device
  if (accuSet1)
    // read accumulated quantities set 1: C1[47:0] E1[47:0] TB1[31:0]
    LTC2947_65_RD_BYTES(LTC2947_65_VAL_C1, 16, bytes);
  else
    // read accumulated quantities set 2: C2[47:0] E2[47:0] TB2[31:0]
    LTC2947_65_RD_BYTES(LTC2947_65_VAL_C2, 16, bytes);

  // calculate absolute value of Cx and store sign
  *signC = LTC2947_65_Abs(bytes, 6);
  // convert unsigned bytes to double value in As
  *C = LTC2947_65_UnsignedBytesToDouble(bytes, 6, LTC2947_65_LSB_C1);

  // calculate absolute value of Ex and store sign
  *signE = LTC2947_65_Abs(bytes + 6, 6);
  // convert unsigned bytes to double value in Ws
  *E = LTC2947_65_UnsignedBytesToDouble(bytes + 6, 6, LTC2947_65_LSB_E1);

  // calc time in seconds
  *TB = LTC2947_65_4BytesToUInt32(bytes + 12) * LTC2947_65_LSB_TB1;
}

void LTC2947_65_Read_C_E_TB(boolean accuSet1, double *C, double *E, double *TB)
{
  // byte array to store register values
  byte bytes[16];

  // read measurement results from device
  if (accuSet1)
    // read accumulated quantities set 1: C1[47:0] E1[47:0] TB1[31:0]
    LTC2947_65_RD_BYTES(LTC2947_65_VAL_C1, 16, bytes);
  else
    // read accumulated quantities set 2: C2[47:0] E2[47:0] TB2[31:0]
    LTC2947_65_RD_BYTES(LTC2947_65_VAL_C2, 16, bytes);

  // convert signed bytes to double value in As
  *C = LTC2947_65_SignedBytesToDouble(bytes, 6, LTC2947_65_LSB_C1);

  // convert signed bytes to double value in Ws
  *E = LTC2947_65_SignedBytesToDouble(bytes + 6, 6, LTC2947_65_LSB_E1);

  // calc time in seconds
  *TB = LTC2947_65_4BytesToUInt32(bytes + 6 + 6) * LTC2947_65_LSB_TB1;
}
