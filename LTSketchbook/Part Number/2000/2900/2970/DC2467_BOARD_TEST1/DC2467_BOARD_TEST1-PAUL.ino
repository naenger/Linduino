/*!
  --BOARD TEST PROGRAM--
  Linear Technology DC2467 Demonstration Board (Linduino Shield)

  LTC2970: Dual I2C Power Supply Monitor and Margining Controller

  @verbatim
  Setup:
   Set the terminal baud rate to 115200 and select the newline terminator.

  @endverbatim

  http://www.linear.com/product/LTC2970

  http://www.linear.com/demo/#demoboards

  REVISION HISTORY
  $Revision: 4037 $
  $Date: 2016-01-27 10:20:48 -0600 (Wed, 27 Jan 2016) $

  Copyright (c) 2016, Linear Technology Corp.(LTC)
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
  !*/

/*! @file
    @ingroup LTC2970
*/

#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "UserInterface.h"
#include "LT_SMBusNoPec.h"
#include "LTC2970.h"

// Added by Paul Phillips
#include <LT_I2C.h>

// this flag causes the program to exit when a test fails
#define STOP_ON_ERR

#define LTC2970_I2C_ADDRESS 0x5B //global 7-bit address
#define LTC2970_I2C_ADDRESS_ZZ 0x6B //SLAVE_LL 7-bit address
#define LTC2970_I2C_ADDRESS_LL 0x5C //SLAVE_LL 7-bit address
#define LTC2970_I2C_ADDRESS_HH 0x6F //SLAVE_HH 7-bit address

// Added by Paul Phillips
#define EEPROM_I2C_ADDRESS 0x50 // 7-Bit I2C Address
#define MAXBYTES 50

/****************************************************************************/
// Global variables
static uint8_t ltc2970_i2c_address;

static LT_SMBusNoPec *smbus = new LT_SMBusNoPec();

// variables used for keeping track of the LTC2970 state
uint16_t servo0_value_low;
uint16_t servo0_value_hi;
uint16_t servo0_value_nom;
uint16_t servo1_value_low;
uint16_t servo1_value_hi;
uint16_t servo1_value_nom;

/****************************************************************************/
//! Initialize Linduino
//! @return void
void setup()
{
  // NOTE: we do NOT initialize the LTC2970 here (though we could with the ltc2970_configure() function)
  //  Assume that there might be another bus master talking to it on the I2C bus.

  // initialize the i2c port
  Serial.begin(115200);         //! Initialize the serial port to the PC
  print_title();
  read_int();
  print_setup();
  //read_int();

  //  print_prompt();

  ltc2970_i2c_address = LTC2970_I2C_ADDRESS;

  servo0_value_nom = 0x2710; // 5.0V
  servo1_value_nom = 0x0F9C; // 2.0V -> -5.0V

  servo0_value_low = 0x2328; // 10% low
  //  servo1_value_low = 0x0ED3; // 10% low
  servo1_value_low = 0x0EB7; // 10% low

  servo0_value_hi = 0x2AF8; // 10% high
  //  servo1_value_hi = 0x1061; // 10% high
  servo1_value_hi = 0x1045; // 10% high
}

/*THINGS TO TEST (with Linduino):
  x  1) Do jumpers work? - check response at ZZ, HH, and global addresses
  x  1b) Does GPIO_CFG jumper cause initial GPIO state to change?
  x  2) Do LEDs work? - ON, OFF, not dim in either state
  x  3) Can we read and write the LTC2970 registers? - simple write and readback
  x  4) GPIOs work? - both LEDs and as enables to regulators
  x  5) Do regulators power-up? Able to sustain > 1A loads?
  x  6) Do current and voltage telemetry reading line-up with reality? - requires external instruments
  x  7) Can the 2970 servo CH1 and CH0? - use telemetry once it has been calibrated
*/

//! Main Linduino loop
//! @return void
void loop()
{
  uint8_t user_command;
  uint16_t return_val_a, return_val_b;
  int foo;

  if (Serial.available())                //! Checks for user input
  {
    Serial.read();

    print_prompt();
    read_int();


    // run this with just Linduino and the shield, no dongle, no +12V power
    // check if the Linduino can talk to the part on the bus

    // -------------------------------------------------
    // ------------------- G1:LED_5V -------------------
    // -------------------------------------------------
    Serial.println(F("\nQUESTION:"));
    Serial.println(F("VERIFY THAT THE VDD_5V LED IS LIT:"));
    Serial.println(F("AND THAT THE VDD_12V LED IS NOT LIT:"));
    Serial.println(F("PRESS 1 FOR YES, 0 FOR NO."));
    user_command = read_int();
    if (user_command != 1) {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("G1:LED_5V"));
#ifdef STOP_ON_ERR
      return;
#endif
    }
    else {
      Serial.print(F("\n*PASS: "));
      Serial.println(F("G1:LED_5V"));
    }

    // -------------------------------------------------
    // ------------------- G1:ASEL0,1 HI-Z -------------
    // -------------------------------------------------
    Serial.println(F("\n(G1:ASEL0,1) REMOVE JUMPERS ASEL0 and ASEL1."));
    Serial.println(F("PRESS ENTER WHEN READY..."));
    read_int();
    return_val_a = smbus->readWord(LTC2970_I2C_ADDRESS_ZZ, LTC2970_ADC_MON);
    smbus->writeWord(LTC2970_I2C_ADDRESS_ZZ, LTC2970_ADC_MON, ~return_val_a);
    return_val_b = smbus->readWord(LTC2970_I2C_ADDRESS_ZZ, LTC2970_ADC_MON);
    if ( return_val_a == return_val_b) {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("G1:ASEL0,1"));
#ifdef STOP_ON_ERR
      return;
#endif
    }
    else {
      Serial.print(F("\n*PASS..."));
      Serial.println(F("G1:ASEL0,1"));
    }

    // -------------------------------------------------
    // ------------------- G1:ASEL0,1 HI ---------------
    // -------------------------------------------------
    Serial.println(F("\nPLACE JUMPER ASEL0 = HI AND ASEL0 = HI ."));
    Serial.println(F("PRESS ENTER WHEN READY..."));
    read_int();

    return_val_a = smbus->readWord(LTC2970_I2C_ADDRESS_HH, LTC2970_ADC_MON);
    smbus->writeWord(LTC2970_I2C_ADDRESS_HH, LTC2970_ADC_MON, ~return_val_a);
    return_val_b = smbus->readWord(LTC2970_I2C_ADDRESS_HH, LTC2970_ADC_MON);
    if ( return_val_a == return_val_b) {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("G1:ASEL0,1"));
#ifdef STOP_ON_ERR
      return;
#endif
    }
    else {
      Serial.print(F("\n*PASS..."));
      Serial.println(F("G1:ASEL0,1"));
    }

    // -------------------------------------------------
    // ------------------- G1:ASEL0,1 LO ---------------
    // -------------------------------------------------
    Serial.println(F("\nPLACE JUMPER ASEL0 = LO AND ASEL0 = LO ."));
    Serial.println(F("PRESS ENTER WHEN READY..."));
    read_int();

    return_val_a = smbus->readWord(LTC2970_I2C_ADDRESS_LL, LTC2970_ADC_MON);
    smbus->writeWord(LTC2970_I2C_ADDRESS_LL, LTC2970_ADC_MON, ~return_val_a);
    return_val_b = smbus->readWord(LTC2970_I2C_ADDRESS_LL, LTC2970_ADC_MON);
    if ( return_val_a == return_val_b) {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("G1:ASEL0,1"));
#ifdef STOP_ON_ERR
      return;
#endif
    }
    else {
      Serial.print(F("\n*PASS "));
      Serial.println(F("G1:ASEL0,1"));
      ltc2970_i2c_address = LTC2970_I2C_ADDRESS_LL;
    }

    // -------------------------------------------------
    // ------------------- G1:GPIO_CFG HI --------------
    // -------------------------------------------------
    Serial.println(F("\nPLACE JUMPER GPIO_CFG = HI."));
    Serial.println(F("PRESS ENTER WHEN READY..."));
    read_int();
    return_val_b = smbus->readWord(ltc2970_i2c_address, LTC2970_IO);
    if ((return_val_b & 0x0200) != 0x0200) {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("G1:GPIO_CFG"));
#ifdef STOP_ON_ERR
      return;
#endif
    }
    else {
      Serial.print(F("\n*PASS..."));
      Serial.println(F("G1:GPIO_CFG"));
    }

    // -------------------------------------------------
    // ------------------- G1:GPIO_CFG LO --------------
    // -------------------------------------------------
    Serial.println(F("\nPLACE JUMPER GPIO_CFG = LO."));
    Serial.println(F("PRESS ENTER WHEN READY..."));
    read_int();
    return_val_a = smbus->readWord(ltc2970_i2c_address, LTC2970_IO);
    if ((return_val_a & 0x0200) != 0x0000) {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("G1:GPIO_CFG"));
#ifdef STOP_ON_ERR
      return;
#endif
    }
    else {
      Serial.print(F("\n*PASS "));
      Serial.println(F("G1:GPIO_CFG"));
    }

    // -------------------------------------------------
    // ------------------- G1:I2C_IO -------------------
    // -------------------------------------------------
    Serial.println(F("\n***PERFORMING WRITE/READ TEST."));
    Serial.println(F("PRESS ENTER WHEN READY..."));
    read_int();
    // clear all registers first
    Serial.println(F("***WRITING ZEROS..."));
    foo = ltc2970_write_verify_zero();
    // then write all registers to good values
    Serial.println(F("***WRITING FULL CONFIG..."));
    if (foo && ltc2970_configure_verify()) {
      Serial.print(F("\n*PASS "));
      Serial.println(F("G1:I2C_IO"));
    }
    else {
      Serial.print(F("\n*!*!*FAIL!*!*!"));
      Serial.println(F("G1:I2C_IO"));
#ifdef STOP_ON_ERR
      return;
#endif
    }

    // -------------------------------------------------
    // ------------------- G1:LED_ALL ------------------
    // -------------------------------------------------
    // Now apply +12V power and test regulators
    Serial.println(F("\nApply +12V power to the Linduino."));
    Serial.println(F("PRESS ENTER WHEN READY..."));
    read_int();

    Serial.println(F("\nQUESTION:"));
    Serial.println(F("VERIFY THAT *ALL* OF THE FOLLOWING LEDs ARE GREEN:"));
    Serial.println(F("VDD_5V, VDD_12V, GPIO_0, GPIO_1, PGOOD0, PGOOD1"));
    Serial.println(F("PRESS 1 FOR YES, 0 FOR NO."));
    user_command = read_int();
    if (user_command != 1) {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("G1:LED_ALL"));
#ifdef STOP_ON_ERR
      return;
#endif
    }
    else {
      Serial.print(F("\n*PASS "));
      Serial.println(F("G1:LED_ALL"));
    }

    // -------------------------------------------------
    // ------------------- G1:LED_SCL ------------------
    // -------------------------------------------------
    Serial.println(F("\nQUESTION:"));
    Serial.println(F("VERIFY THAT THE SCL LED FLICKERS GREEN:"));
    Serial.println(F("PRESS ENTER WHEN READY..."));
    read_int();
    Serial.println(F("PRESS 1 FOR YES, 0 FOR NO."));
    for (foo = 0; foo < 75; foo++) {
      smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_ADC);
      delay(10);
      smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_ADC);
      delay(10);
      smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_ADC);
      delay(40);
    }
    user_command = read_int();
    if (user_command != 1) {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("G1:LED_SCL"));
#ifdef STOP_ON_ERR
      return;
#endif
    }
    else {
      Serial.println(F("\n*PASS"));
      Serial.println(F("G1:LED_SCL"));
    }

    // -------------------------------------------------
    // ------------------- G1:CH0_VO -------------------
    // -------------------------------------------------
    // Check that the voltage readings make sense
    return_val_a = 0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_ADC);
    return_val_b = 0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_A_ADC);
    ////LIMITS
    //      if((return_val_a <= 0x2774) && (return_val_a >= 0x26AC)){
    if ((return_val_a <= 0x2904) && (return_val_a >= 0x251C)) {
      Serial.print(F("\n*PASS "));
      Serial.println(F("G1:CH0_V0"));
    }
    else {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("G1:CH0_V0"));
      Serial.print(F("\n****CH0 VOLTAGE READING OUT OF BOUNDS: "));
      Serial.println(return_val_a, HEX);
#ifdef STOP_ON_ERR
      return;
#endif
    }

    // -------------------------------------------------
    // ------------------- G1:CH1_VO -------------------
    // -------------------------------------------------
    ////LIMITS
    //      if((return_val_b <= 0x0FD0 ) && (return_val_b >= 0x0F75)){
    if ((return_val_b <= 0x1068 ) && (return_val_b >= 0x0ED8)) {
      Serial.print(F("\n*PASS "));
      Serial.println(F("G1:CH1_V0"));
    }
    else {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("G1:CH1_V0"));
      Serial.print(F("\n****CH1 VOLTAGE READING OUT OF BOUNDS: "));
      Serial.println(return_val_b, HEX);
#ifdef STOP_ON_ERR
      return;
#endif
    }

    // -------------------------------------------------
    // ------------------- G1:CH0_IO -------------------
    // -------------------------------------------------
    // Check that the current readings make sense (2's complement)
    return_val_a = 0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_B_ADC);
    return_val_b = 0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_B_ADC);
    ////LIMITS
    if ((return_val_a <= 0x0001) || (return_val_a >= 0x7FFE)) {
      Serial.print(F("\n*PASS "));
      Serial.println(F("G1:CH0_I0"));
    }
    else {
      Serial.print(F("\n*!*!*FAIL!*!*!"));
      Serial.println(F("G1:CH0_I0"));
      Serial.print(F("\n****CH0 CURRENT READING OUT OF BOUNDS: "));
      Serial.println(return_val_a, HEX);
#ifdef STOP_ON_ERR
      return;
#endif
    }

    // -------------------------------------------------
    // ------------------- G1:CH1_IO -------------------
    // -------------------------------------------------
    ////LIMITS
    if ((return_val_b <= 0x0003) || (return_val_b >= 0x7FFC)) {
      Serial.print(F("\n*PASS "));
      Serial.println(F("G1:CH1_I0"));
    }
    else {
      Serial.print(F("\n*!*!*FAIL!*!*!"));
      Serial.println(F("G1:CH1_I0"));
      Serial.print(F("\n****CH1 CURRENT READING OUT OF BOUNDS: "));
      Serial.println(return_val_b, HEX);
#ifdef STOP_ON_ERR
      return;
#endif
    }
    Serial.println(F("\n*PASSED VOLTAGE & CURRENT SANITY CHECK."));

    /////////////////////////

    // -------------------------------------------------
    // ------------------- G2:CH0_V1 -------------------
    // -------------------------------------------------
    // Add external instruments to test the regulators
    //Serial.println(F("\nConnect a DMM between VOUT_CH0 and GND."));
    //Serial.println(F("PRESS ENTER WHEN READY..."));
    //read_int();
    return_val_a = 0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_ADC);
    Serial.println(F("\nQUESTION:"));
    Serial.print(F("VERIFY THAT THE DMM READING FOR CH0 IS WITHIN: "));
    Serial.print((float)return_val_a * 500e-6 * 0.98, DEC);
    Serial.print(F(" AND "));
    Serial.println((float)return_val_a * 500e-6 * 1.02, DEC);
    Serial.println(F("PRESS 1 FOR YES, 0 FOR NO."));
    user_command = read_int();
    if (user_command != 1) {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("G2:CH0_V1"));
#ifdef STOP_ON_ERR
      return;
#endif
    }
    else {
      Serial.print(F("\n*PASS "));
      Serial.println(F("G2:CH0_V1"));
    }

    //////////////////////

    // -------------------------------------------------
    // ------------------- G2:CH1_V1 -------------------
    // -------------------------------------------------
    //Serial.println(F("\nConnect a DMM between VOUT_CH1 and GND."));
    //Serial.println(F("PRESS ENTER WHEN READY..."));
    //read_int();
    return_val_a = 0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_ADC);
    Serial.println(F("\nQUESTION:"));
    Serial.print(F("VERIFY THAT THE DMM READING FOR CH1 IS WITHIN: "));
    Serial.print((float)return_val_a * -500e-6 * 0.98, DEC);
    Serial.print(F(" AND "));
    Serial.println((float)return_val_a * -500e-6 * 1.02, DEC);
    Serial.println(F("PRESS 1 FOR YES, 0 FOR NO."));
    user_command = read_int();
    if (user_command != 1) {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("G2:CH1_V1"));
#ifdef STOP_ON_ERR
      return;
#endif
    }
    else {
      Serial.print(F("\n*PASS "));
      Serial.println(F("G2:CH1_V1"));
    }

    //Serial.println(F("\nRemove the DMM."));
    //Serial.println(F("PRESS ENTER WHEN READY..."));
    //read_int();

    // now that instruments confirm that the turret voltages correspond to register readings
    //  we can just use register readings and skip the external instruments

    // -------------------------------------------------
    // ------------------- G3:CH0_SERVO_LOW ------------
    // -------------------------------------------------
    Serial.println(F("\nSERVOING THE SUPPLIES TO -10%"));
    smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_SERVO, 0x2710);
    smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_IDAC, 0x0880);
    smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_SERVO, 0x0FA0);
    smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, 0x0480);
    ltc2970_soft_connect_dac(smbus, ltc2970_i2c_address, 0);
    ltc2970_soft_connect_dac(smbus, ltc2970_i2c_address, 1);
    ltc2970_servo_to_adc_val(smbus, ltc2970_i2c_address, 0, servo0_value_low);
    ltc2970_servo_to_adc_val(smbus, ltc2970_i2c_address, 1, servo1_value_low);
    delay(2000); // let the voltages settle
    return_val_a = 0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_ADC);
    return_val_b = 0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_A_ADC);
    ////LIMITS
    // 4.506V ... 4.495
    //      if((return_val_a <= 0x2334) && (return_val_a >= 0x231E)){
    //if ((return_val_a <= 0x2348) && (return_val_a >= 0x2308)) {
    if ((return_val_a <= 0x2355) && (return_val_a >= 0x22FB)) {
      Serial.print(F("\n*PASS "));
      Serial.println(F("G3:CH0_SERVO_LOW"));
    }
    else {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("G3:CH0_SERVO_LOW"));
      Serial.print(F("\n****CH0 VOLTAGE READING OUT OF BOUNDS: "));
      Serial.println(return_val_a, HEX);
#ifdef STOP_ON_ERR
      return;
#endif
    }

    // -------------------------------------------------
    // ------------------- G3:CH1_SERVO_LOW ------------
    // -------------------------------------------------
    ////LIMITS
    //      if((return_val_b <= 0x0EBA) && (return_val_b >= 0x0DB4)){
    if ((return_val_b <= 0x0EBA) && (return_val_b >= 0x0DB4)) {
      Serial.print(F("\n*PASS "));
      Serial.println(F("G3:CH1_SERVO_LOW"));
    }
    else {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("G3:CH1_SERVO_LOW"));
      Serial.print(F("\n****CH1 VOLTAGE READING OUT OF BOUNDS: "));
      Serial.println(return_val_b, HEX);
#ifdef STOP_ON_ERR
      return;
#endif
    }
    Serial.println(F("\nSERVOING THE SUPPLIES TO +10%"));
    ltc2970_servo_to_adc_val(smbus, ltc2970_i2c_address, 0, servo0_value_hi);
    ltc2970_servo_to_adc_val(smbus, ltc2970_i2c_address, 1, servo1_value_hi);
    delay(6000); // let the voltages settle
    return_val_a = 0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_ADC);
    return_val_b = 0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_A_ADC);

    // -------------------------------------------------
    // ------------------- G3:CH0_SERVO_HI -------------
    // -------------------------------------------------
    ////LIMITS
    //if ((return_val_a <= 0x2AFF) && (return_val_a >= 0x2AF0)) {
    if ((return_val_a <= 0x2B2F) && (return_val_a >= 0x2AC1)) {
      Serial.print(F("\n*PASS "));
      Serial.println(F("G3:CH0_SERVO_HI"));
    }
    else {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("G3:CH0_SERVO_HI"));
      Serial.print(F("\n****CH0 VOLTAGE READING OUT OF BOUNDS: "));
      Serial.println(return_val_a, HEX);
#ifdef STOP_ON_ERR
      return;
#endif
    }

    // -------------------------------------------------
    // ------------------- G3:CH1_SERVO_HI -------------
    // -------------------------------------------------
    ////LIMITS
    if ((return_val_b <= 0x1048) && (return_val_b >= 0x1042)) {
      Serial.print(F("\n*PASS "));
      Serial.println(F("G3:CH1_SERVO_HI"));
    }
    else {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("G3:CH1_SERVO_HI"));
      Serial.print(F("\n****CH1 VOLTAGE READING OUT OF BOUNDS: "));
      Serial.println(return_val_b, HEX);
#ifdef STOP_ON_ERR
      return;
#endif
    }
    Serial.println(F("\nSERVOING THE SUPPLIES TO NOMINAL"));
    ltc2970_servo_to_adc_val(smbus, ltc2970_i2c_address, 0, servo0_value_nom);
    ltc2970_servo_to_adc_val(smbus, ltc2970_i2c_address, 1, servo1_value_nom);
    delay(2000); // let the voltages settle
    return_val_a = 0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_ADC);
    return_val_b = 0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_A_ADC);


    // -------------------------------------------------
    // ------------------- G3:CH0_SERVO_NOM ------------
    // -------------------------------------------------
    ////LIMITS
    if ((return_val_a <= 0x271F) && (return_val_a >= 0x2702)) {
      Serial.print(F("\n*PASS "));
      Serial.println(F("G3:CH0_SERVO_NOM"));
    }
    else {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("G3:CH0_SERVO_NOM"));
      Serial.print(F("\n****CH0 VOLTAGE READING OUT OF BOUNDS: "));
      Serial.println(return_val_a, HEX);
#ifdef STOP_ON_ERR
      return;
#endif
    }

    // -------------------------------------------------
    // ------------------- G3:CH1_SERVO_NOM ------------
    // -------------------------------------------------
    ////LIMITS
    if ((return_val_b <= 0x0F9F) && (return_val_b >= 0x0F99)) {
      Serial.print(F("\n*PASS  "));
      Serial.println(F("G3:CH1_SERVO_NOM"));
    }
    else {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("G3:CH1_SERVO_NOM"));
      Serial.print(F("\n****CH1 VOLTAGE READING OUT OF BOUNDS: "));
      Serial.println(return_val_b, HEX);
#ifdef STOP_ON_ERR
      return;
#endif
    }

    // now load the outputs, make sure that the regulators can do at least 1A each
    Serial.println(F("\nConnect a load box between VOUT_CH0 (positive) and VOUT_CH1 (negative)."));
    Serial.println(F("Pull 1.10A +-10mA from VOUT_CH0 to VOUT_CH1."));
    Serial.println(F("PRESS ENTER WHEN READY..."));
    read_int();
    Serial.println(F("\n*TESTING VOLTAGE & CURRENT SANITY."));
    // disconnect the DACs to measure the raw regulator performance
    smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_IDAC, 0x0080);
    smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, 0x0080);
    // Check that the voltage readings make sense
    delay(2000);
    return_val_a = 0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_ADC);
    return_val_b = 0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_A_ADC);

    // -------------------------------------------------
    // ------------------- G4:V0_LOAD ------------------
    // -------------------------------------------------
    ////LIMITS
    if ((return_val_a <= 0x2774) && (return_val_a >= 0x26AC)) {
      Serial.print(F("\n*PASS "));
      Serial.println(F("G4:V0_LOAD"));
    }
    else {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("G4:V0_LOAD"));
      Serial.print(F("\n****CH0 VOLTAGE READING OUT OF BOUNDS: "));
      Serial.println(return_val_a, HEX);
#ifdef STOP_ON_ERR
      return;
#endif
    }

    // -------------------------------------------------
    // ------------------- G4:V1_LOAD ------------------
    // -------------------------------------------------
    ////LIMITS
    if ((return_val_b <= 0x0FD0 ) && (return_val_b >= 0x0F75)) {
      Serial.print(F("\n*PASS "));
      Serial.println(F("G4:V1_LOAD"));
    }
    else {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("G4:V1_LOAD"));
      Serial.print(F("\n****CH1 VOLTAGE READING OUT OF BOUNDS: "));
      Serial.println(return_val_b, HEX);
#ifdef STOP_ON_ERR
      return;
#endif
    }

    // Check that the current readings make sense
    return_val_a = 0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_B_ADC);
    return_val_b = 0x7FFF & smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_B_ADC);

    // -------------------------------------------------
    // ------------------- G4:IO_LOAD ------------------
    // -------------------------------------------------
    ////LIMITS
    if ((return_val_a <= 0x0030) && (return_val_a >= 0x002A)) {
      Serial.print(F("\n*PASS "));
      Serial.println(F("G4:I0_LOAD"));
    }
    else {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("G4:I0_LOAD"));
      Serial.print(F("\n****CH0 CURRENT READING OUT OF BOUNDS: "));
      Serial.println(return_val_a, HEX);
#ifdef STOP_ON_ERR
      return;
#endif
    }

    // -------------------------------------------------
    // ------------------- G4:I1_LOAD ------------------
    // -------------------------------------------------
    ////LIMITS
    if ((return_val_b <= 0x0906 ) && (return_val_b >= 0x082A)) {
      Serial.print(F("\n*PASS "));
      Serial.println(F("G4:I1_LOAD"));
    }
    else {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("G4:I1_LOAD"));
      Serial.print(F("\n****CH1 CURRENT READING OUT OF BOUNDS: "));
      Serial.println(return_val_b, HEX);
#ifdef STOP_ON_ERR
      return;
#endif
    }
    Serial.println(F("\n*PASSED VOLTAGE & CURRENT SANITY CHECK.\n"));
    Serial.println(F("\n*REMOVE LOAD CURRENT CONNECTIONS FROM CH0 and CH1.\n"));

    // ALERT LED
    Serial.println(F("\nQUESTION:"));
    Serial.println(F("VERIFY THAT THE ALERT LED IS ILLUMINATED RED:"));
    Serial.println(F("PRESS 1 FOR YES, 0 FOR NO."));
    //      smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_OV, 0x2AF8);
    smbus->writeWord(ltc2970_i2c_address, LTC2970_IO, 0x007A);
    smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_UV, 0x2900);

    // -------------------------------------------------
    // ------------------- G5:LED_ALERT ----------------
    // -------------------------------------------------
    user_command = read_int();
    if (user_command != 1) {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("G5:LED_ALERT"));
#ifdef STOP_ON_ERR
      return;
#endif
    }
    else {
      Serial.print(F("\n*PASS "));
      Serial.println(F("G5:LED_ALERT"));
    }

    // Value to write to EEPROM
    char valueToWrite[] = "LTC2970,Cls,D2970,01,01,DC,DC2467A,-------------\n\0";
    uint8_t result;
    result = EEPROM_Write(EEPROM_I2C_ADDRESS, 0, valueToWrite);

    // -------------------------------------------------
    // ------------------- D1:WRITE EEPROM -------------
    // -------------------------------------------------
    if (result != 0) {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("D1:WRITE EEPROM"));
#ifdef STOP_ON_ERR
      return;
#endif
    }
    else {
      Serial.print(F("\n*PASS "));
      Serial.println(F("D1:WRITE EEPROM"));
    }

    // -------------------------------------------------
    // ------------------- D1:READ EEPROM --------------
    // -------------------------------------------------
    // Holds the value returned from EEPROM_Read function
    char valueRead[MAXBYTES];
    result = EEPROM_Read(EEPROM_I2C_ADDRESS, 0, valueRead);
    if (result != 0) {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("D1:READ EEPROM"));
#ifdef STOP_ON_ERR
      return;
#endif
    }
    else {
      Serial.print(F("\n*PASS "));
      Serial.println(F("D1:READ EEPROM"));
    }

    // -------------------------------------------------
    // ------------------- D1:VERIFY EEPROM ------------
    // -------------------------------------------------
    result = EEPROM_Verify(valueToWrite, valueRead);
    if (result != 0) {
      Serial.print(F("\n*!*!*FAIL!*!*! "));
      Serial.println(F("D1:VERIFY EEPROM"));
#ifdef STOP_ON_ERR
      return;
#endif
    }
    else {
      Serial.print(F("\n*PASS "));
      Serial.println(F("D1:VERIFY EEPROM"));
    }

    smbus->writeWord(ltc2970_i2c_address, LTC2970_IO, 0x00FA);
    smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_UV, servo0_value_nom);

    // passed all tests. print a message
    print_success();
  }
}


/************************************************************************/
// Function Definitions

//! Prints the title block when program first starts.
//! @return void
void print_title()
{
  Serial.print(F("\n***************************************************************\n"));
  Serial.print(F("* DC2467 Test Program                                           *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("* This program provides a simple interface to test the          *\n"));
  Serial.print(F("* the DC2467 board in production                                *\n"));
  Serial.print(F("* REQUIRES EXTERNAL INSTRUMENTS                                 *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("* Set the baud rate to 115200 and select the newline terminator.*\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("*****************************************************************\n"));
  Serial.println(F("\nPRESS ENTER WHEN READY..."));
}

// Added by Paul Phillips
void print_setup()
{
  Serial.println(F("\nConnect a 14-pin ribbon cable between the Linduino and the 14 to 12 pin adapter"));
  Serial.println(F("Verify/install the two-pin jumper on the adapter"));
  Serial.println(F("Verify/set the jumpers on the Linduino are set as follows:"));
  Serial.println(F("  JP3 SET TO 5V AND ALL OTHER JUMPERS VACANT:"));
  Serial.println(F("\nSetup the hardware per the connection diagram"));
  Serial.println(F("\nPress ENTER to begin."));
}

//! Prints main menu.
//! @return void
void print_prompt()
{
  Serial.println(F("\n\n================================="));
  Serial.println(F("BEGIN TESTING A NEW DC2467 SHIELD"));
  Serial.println(F("Plug-in the DC2467 shield to the Linduino."));
  Serial.println(F("Connect a 12-pin ribbon cable between the DC2467A and the 14 to 12 pin adapter"));
  //Serial.print(F("\nPlug-in the USB cable from the PC to the Linduino."));
  Serial.println(F("Do not apply +12V power (yet)."));
  //Serial.print(F("\nDo not connect the DC1613 dongle (yet)."));
  Serial.println(F("Connect the DMM wires to CH0, GND, and CH1"));

  // Added by Paul Phillips
  // Have user set/verify that the jumpers on the DC2467A are set correctly
  Serial.println(F("\nVERIFY THAT THE JUMPERS ON THE DC2467A ARE:"));
  Serial.println(F("  ASEL0 = LO, ASEL1 = LO, and GPIO_CFL = LO:"));
  Serial.println(F("\nFollow on-screen prompts to test."));
  Serial.println(F("This program will terminate at any failure."));
  Serial.println(F("\nPress ENTER to begin."));
}

//! Prints main menu.
//! @return void
void print_success()
{
  Serial.print(F("\n"));
  Serial.print(F("\nTHIS DC2467 SHIELD PASSED ALL TESTS IN THIS PROGRAM!"));
  Serial.print(F("\nRemove ribbon cable from shield.  Plug in DC1613A ribbon cable and run LTpowerPlay"));
  Serial.print(F("\n\nStart LTpowerPlay.  Load the Example project"));
  Serial.print(F("\n  File -> Open Example Project File -> DC2467A -> DC2467A Defaults (which one?)"));
  Serial.print(F("\nSuccess Criteria follows"));
  Serial.print(F("\n  System Window shows U0 (7h5C)-LTC2970 with U0:0 and U0:1.  These are completely GREEN"));
  Serial.print(F("\n  Click CH0_A_ADC_LTC2970.  The value should be between __ and __"));
  Serial.print(F("\n  Click CH0_B_ADC_LTC2970.  The value should be between __ and __"));
  Serial.print(F("\n  Click CH1_A_ADC_LTC2970.  The value should be between __ and __"));
  Serial.print(F("\n  Click CH1_B_ADC_LTC2970.  The value should be between __ and __"));
  //Serial.print(F("\n\nProceed to other tests in the test procedure document."));
}

//! Writes configuration values to the LTC2970 registers
//! @return 1 for PASS 0 for FAIL
int ltc2970_configure_verify()
{
  uint16_t read_val;
  int i = 0;
  //configure all of the LTC2970 registers for this application
  // use SMbus commands
  smbus->writeWord(ltc2970_i2c_address, LTC2970_FAULT_EN, 0x0DEF);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_IO, 0x00FA);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_ADC_MON, 0x007F);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_VDD_OV, 0x2CEC);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_VDD_UV, 0x2328);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_V12_OV, 0x0FA0);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_V12_UV, 0x0A6B);

  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_OV, 0x2AFA);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_UV, 0x2300);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_SERVO, 0x2710);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_IDAC, 0x0880); // IDAC set by the servo
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_B_OV, 0x0078);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_B_UV, 0x7FD8);

  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_OV, 0x1068);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_UV, 0x0BB8);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_SERVO, 0x0FA0); // 0x0F88 is servo to -5V
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, 0x0480); // IDAC set by the servo
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_B_OV, 0x0960);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_B_UV, 0x7F38);

  // verify...

  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_FAULT_EN);
  if (((read_val & 0xFFFF) != 0x0DEF)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_IO);
  if (((read_val & 0xFDFF) != 0x00FA)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_ADC_MON);
  if (((read_val & 0xFFFF) != 0x007F)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_VDD_OV);
  if (((read_val & 0xFFFF) != 0x2CEC)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_VDD_UV);
  if (((read_val & 0xFFFF) != 0x2328)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_V12_OV);
  if (((read_val & 0xFFFF) != 0x0FA0)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_V12_UV);
  if (((read_val & 0xFFFF) != 0x0A6B)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_OV);
  if (((read_val & 0xFFFF) != 0x2AFA)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_UV);
  if (((read_val & 0xFFFF) != 0x2300)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_SERVO);
  if (((read_val & 0xFFFF) != 0x2710)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_IDAC);
  if (((read_val & 0xFFFF) != 0x0880)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_B_OV);
  if (((read_val & 0xFFFF) != 0x0078)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_B_UV);
  if (((read_val & 0xFFFF) != 0x7FD8)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_A_OV);
  if (((read_val & 0xFFFF) != 0x1068)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_A_UV);
  if (((read_val & 0xFFFF) != 0x0BB8)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_A_SERVO);
  if (((read_val & 0xFFFF) != 0x0FA0)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC);
  if (((read_val & 0xFFFF) != 0x0480)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_B_OV);
  if (((read_val & 0xFFFF) != 0x0960)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_B_UV);
  if (((read_val & 0xFFFF) != 0x7F38)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }

  return 1;
}

//! Writes zero values to the LTC2970 registers
//! Verifies register contents after writes
//! @return 1 for PASS 0 for FAIL
int ltc2970_write_verify_zero()
{
  uint16_t read_val;
  int i = 0;
  //write...

  smbus->writeWord(ltc2970_i2c_address, LTC2970_FAULT_EN, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_IO, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_ADC_MON, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_VDD_OV, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_VDD_UV, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_V12_OV, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_V12_UV, 0x0000);

  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_OV, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_UV, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_SERVO, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_A_IDAC, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_B_OV, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH0_B_UV, 0x0000);

  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_OV, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_UV, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_SERVO, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_B_OV, 0x0000);
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_B_UV, 0x0000);

  // verify...

  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_FAULT_EN);
  if (((read_val & 0xFFFF) != 0x0000)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_IO);
  if (((read_val & 0xFD8F) != 0x0000)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_ADC_MON);
  if (((read_val & 0xFFFF) != 0x0000)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_VDD_OV);
  if (((read_val & 0xFFFF) != 0x0000)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_VDD_UV);
  if (((read_val & 0xFFFF) != 0x0000)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_V12_OV);
  if (((read_val & 0xFFFF) != 0x0000)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_V12_UV);
  if (((read_val & 0xFFFF) != 0x0000)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_OV);
  if (((read_val & 0xFFFF) != 0x0000)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_UV);
  if (((read_val & 0xFFFF) != 0x0000)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_SERVO);
  if (((read_val & 0xFFFF) != 0x0000)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_A_IDAC);
  if (((read_val & 0xFFFF) != 0x0000)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_B_OV);
  if (((read_val & 0xFFFF) != 0x0000)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH0_B_UV);
  if (((read_val & 0xFFFF) != 0x0000)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_A_OV);
  if (((read_val & 0xFFFF) != 0x0000)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_A_UV);
  if (((read_val & 0xFFFF) != 0x0000)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_A_SERVO);
  if (((read_val & 0xFFFF) != 0x0000)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC);
  if (((read_val & 0xFFFF) != 0x0000)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_B_OV);
  if (((read_val & 0xFFFF) != 0x0000)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }
  i++;
  read_val = smbus->readWord(ltc2970_i2c_address, LTC2970_CH1_B_UV);
  if (((read_val & 0xFFFF) != 0x0000)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return 0;
  }

  return 1;
}

// Added by Paul Phillips
// Write to the EEPROM with the given I2C address, starting at the address
uint8_t EEPROM_Write(uint8_t i2cAddress, uint16_t address, char data[])
{
  uint8_t result;

  //! Poll the I2C port and look for an acknowledge
  //! @return Returns 0 if successful, 1 if not successful
  result = i2c_poll(i2cAddress);

  if (result == 0)
  {
    uint16_t addr;
    uint16_t index;
    uint16_t bytesToWrite = strlen(data) < MAXBYTES ? strlen(data) : MAXBYTES;
    for (addr = address, index = 0; addr < address + bytesToWrite; addr++, index++)
      WriteByte(i2cAddress, addr, data[index]);

    WriteByte(i2cAddress, addr, NULL);

    Serial.print(F("Wrote: "));
    Serial.println(data);
  }
  else
    Serial.println(F("Cannot locate EEPROM"));

  return result;
}

// Added by Paul Phillips
// Write the data byte to the EEPROM with the given I2C address at the address
void WriteByte(uint8_t i2cAddress, uint16_t addr, uint8_t data)
{
  //! Write start bit to the hardware I2C port
  //! @return 0 if successful, 1 if not successful
  i2c_start();

  //! Send a data byte to hardware I2C port
  //! @return 0 if successful, 1 if not successful
  // Write the EEPROM's I2C address
  i2c_write(i2cAddress << 1);

  // Write the EEPROM address (low)
  i2c_write(addr % 256);

  // Write the data that will be written
  i2c_write(data);

  //! Write stop bit to the hardware I2C port
  i2c_stop();

  // Write requirement for device (per datasheet)
  delay(5);
}

// Added by Paul Phillips
// Read from the EEPROM with the given I2C address, starting at the address
// up until NULL or MAXBYTES, whichever is less
uint8_t EEPROM_Read(uint8_t i2cAddress, uint16_t address, char* valueRead)
{
  //Serial.println(F("EEPROM Read"));

  uint8_t result;

  //! Poll the I2C port and look for an acknowledge
  //! @return Returns 0 if successful, 1 if not successful
  result = i2c_poll(i2cAddress);

  if (result == 0)
  {
    uint16_t index = 0;
    for (uint16_t addr = address; addr < address + MAXBYTES; addr++, index++)
    {
      //! Write start bit to the hardware I2C port
      //! @return 0 if successful, 1 if not successful
      i2c_start();

      //! Send a data byte to hardware I2C port
      //! @return 0 if successful, 1 if not successful
      // Write the EEPROM's I2C address
      i2c_write(i2cAddress << 1);

      // Write the EEPROM address (low)
      i2c_write(addr % 256);

      i2c_start();

      i2c_write((i2cAddress << 1) + 1);

      char dataRead;

      //! Send a data byte to hardware I2C port
      //! @return 0 if successful, 1 if not successful
      dataRead = i2c_read(WITH_NACK);
      valueRead[index] = dataRead;

      //! Write stop bit to the hardware I2C port
      i2c_stop();

      if (valueRead[index] == NULL)
        break;
    }

    // Terminate with NULL
    valueRead[index] = NULL;

    Serial.print(F("Read: "));
    Serial.println(valueRead);

    Serial.print(F("Length: "));
    Serial.println(strlen(valueRead));
  }
  else
    Serial.println(F("Cannot locate EEPROM"));

  return result;
}

// Added by Paul Phillips
// Compare the strings
uint8_t EEPROM_Verify(char* valWritten, char* valRead)
{
  Serial.print(F("Verified: "));
  int compare = strcmp(valWritten, valRead);
  if (compare == 0)
    Serial.println(F("TRUE"));
  else
    Serial.println(F("FALSE"));

  return (compare != 0);
}
