

/*
Whack-a-mole game (2937 Bop-It)
Linear Technology LTC2937 Demonstration Game
LTC2937: Six Channel Sequencer and Voltage Supervisor with EEPROM


@verbatim
http://www.linear.com/product/LTC2937
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
//#include "LT_I2CBus.h"
#include "LT_SMBusNoPec.h"
//#include "LT_SMBusPec.h"
//#include "LT_PMBUS.h"
//#include "LT_I2C.h"
#include "LT_Wire.h"

#define LTC2937_I2C_ADDRESS 0x36 //global 7-bit address
//#define LTC2937_I2C_ADDRESS 0x50 //HHZ 7-bit address

#define PCF8575C_I2C_ADDRESS 0x20 //default 7-bit address

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

// max time delay between events, in milliseconds
#define GAME_MAX_DELAY_TIME 1500
#define GAME_MIN_DELAY_TIME 200



#define GAME_NUM_CHANNELS 6

// delay times in milliseconds
#define LTC2937_RESTORE_DELAY   10
#define LTC2937_STORE_DELAY   120

#define tone_A_0 220
#define tone_G_0 196
#define tone_F_0 175
#define tone_E_0 165
#define tone_D_0 147
#define tone_C_0 131
#define tone_B_0 123
#define tone_A_1 440
#define tone_G_1 392
#define tone_F_1 349
#define tone_E_1 330
#define tone_D_1 294
#define tone_C_1 262
#define tone_B_1 247
#define tone_A_2 440
#define tone_G_2 392
#define tone_F_2 349
#define tone_E_2 330
#define tone_D_2 294
#define tone_C_2 262
#define tone_B_2 247



/****************************************************************************/
// Global variables
static uint8_t ltc2937_i2c_address;
static uint8_t pcf8575_i2c_address;

//static LT_I2CBus *i2cbus = new LT_I2CBus();
//static LT_I2CBus *i2cbus = new LT_I2CBus();
//static LT_SMBusNoPec *smbus = new LT_SMBusNoPec(i2cbus);
static LT_SMBusNoPec *smbus = new LT_SMBusNoPec();


// function return values
enum return_values {SUCCEED=0, // the function succeeded without errors
                    NOT_DOWN, // the LTC2937 is not sequenced-down
                    WRITE_PROTECTED, // the LTC2937 is write-protected
                    FAIL  // general failure to do the right thing
                   };

// a loop counter
int done = 0;
// a random delay variable
int game_rand_delay;
// the game score
int game_score;
// inverse likelihood of an impossible timeout
int game_easiness = 100;

//an array to hold channel addresses
uint8_t seq_position[GAME_NUM_CHANNELS] = {LTC2937_SEQ_UP_POSITION_1, LTC2937_SEQ_UP_POSITION_2, LTC2937_SEQ_UP_POSITION_3, LTC2937_SEQ_UP_POSITION_4, LTC2937_SEQ_UP_POSITION_5, LTC2937_SEQ_UP_POSITION_6};

//delay thresholds for calcualting score
float game_delay_0 = (GAME_MAX_DELAY_TIME - GAME_MIN_DELAY_TIME);
float game_delay_1 = (GAME_MIN_DELAY_TIME+(game_delay_0*1.0));
float game_delay_2 = (GAME_MIN_DELAY_TIME+(game_delay_0*0.8));
float game_delay_3 = (GAME_MIN_DELAY_TIME+(game_delay_0*0.6));
float game_delay_4 = (GAME_MIN_DELAY_TIME+(game_delay_0*0.4));
float game_delay_5 = (GAME_MIN_DELAY_TIME+(game_delay_0*0.2));


const int tune_1[] = {tone_A_0,tone_B_0,tone_C_0,tone_D_0,tone_E_0,tone_F_0,tone_G_0,tone_A_1,tone_B_1,tone_C_1,tone_D_1,tone_E_1,tone_F_1,tone_G_1,tone_A_2,tone_B_2,tone_C_2,tone_D_2,tone_E_2,tone_F_2,tone_G_2};
const int tune_1_max = 20; // 21 - 1

/****************************************************************************/
//! Initialize Linduino
void setup()
{
  uint16_t return_val;

  ltc2937_i2c_address = LTC2937_I2C_ADDRESS;
  pcf8575_i2c_address = PCF8575C_I2C_ADDRESS;

  Serial.begin(115200);         //! Initialize the serial port to the PC

  //TODO:
  //store a fault log in the part to prevent any EEPROM access during game play

  // initialize the LTC2937 to game default register settings
  game_write_all_regs_game_default(ltc2937_i2c_address);
  game_clear_fault();
  game_reset_score(0);
  //init the game state machine
  done = GAME_NUM_CHANNELS;

  game_rand_delay = 100;

  Serial.print (F("DELAY_5 = "));
  Serial.println (game_delay_5, DEC);
  Serial.print (F("DELAY_4 = "));
  Serial.println (game_delay_4, DEC);
  Serial.print (F("DELAY_3 = "));
  Serial.println (game_delay_3, DEC);
  Serial.print (F("DELAY_2 = "));
  Serial.println (game_delay_2, DEC);
  Serial.print (F("DELAY_1 = "));
  Serial.println (game_delay_1, DEC);
}

/****************************************************************************/
//! Main Linduino Loop
void loop()
{
  int x;
  game_write_all_regs_game_default(ltc2937_i2c_address);
  Serial.print (F("CLEAR FAULTS.\n"));
  game_clear_fault();
  game_reset_score(0);
  done = GAME_NUM_CHANNELS;

  //A**** play some random sequence of lights to confuse and excite the player before the game starts
  Serial.print (F("PLAY RANDOM LIGHTS.\n"));
  game_play_random_lights(ltc2937_i2c_address, 3000); //play for 3000ms
  //  game_chasing_score(3000); //play for 3000ms

  //B****choose random sequence-up positions (1-6) for channels 1-6
  //******set all channels to sequence-down pos 1 to expedite seq-dn
  Serial.print (F("ASSIGN RANDOM SEQUENCE POSITIONS.\n"));
  game_random_seq_up_pos(ltc2937_i2c_address);

  Serial.print (F("ASSIGN RANDOM SEQUENCE UP TIMEOUT.\n"));
  game_random_seq_up_timeout(ltc2937_i2c_address);

  //C****set the breakpoint at sequence-position 0, breakpoint enabled
  Serial.print (F("SET BREAKPOINT.\n"));
  game_set_breakpoint(ltc2937_i2c_address, 0x0000);

  //D****set the "press to start" indicator
  Serial.print (F("SET START LED.\n"));
  game_set_start_led_on();


  //E****wait for begin, start when pressed
  Serial.print (F("WAIT FOR USER INPUT.\n"));
  game_get_start(ltc2937_i2c_address); //wait until start button is pressed

  //Count down to begin
  game_count_down();
  
  game_set_start_led_off();
  game_reset_score(0);
  Serial.print (F("SEQUENCE-UP.\n"));
  game_sequence_up(ltc2937_i2c_address);

  do
  {
    while (done > 0)
    {
      //F****wait a random time before releasing breakpoint
      //******releasing breakpoint will advance, enabling the next channel
      game_rand_delay = random(GAME_MIN_DELAY_TIME,GAME_MAX_DELAY_TIME);
      delay(game_rand_delay);
      Serial.print (F("DONE WITH DELAY.\n"));

      //******set breakpoint to seq pos n+1
      Serial.print (F("INCREMENT BREAKPOINT.\n"));
      game_inc_breakpoint(ltc2937_i2c_address);

      //G****wait long enough, then read registers
      //****detect either the sequence fault (fail) or no fault (success)
      Serial.print (F("WAIT FOR RESULT.\n"));
      // note that if the an event is a fault then a fault log will be stored,
      // which takes 120ms
      switch (game_wait_for_result(ltc2937_i2c_address, 777) )
      {
        case 0 : // result was no fault
          Serial.print (F("SUCCESS!\n"));
          Serial.println(done, DEC);
          Serial.print (F("TURNING OFF LED. ADDR = "));
          Serial.println (seq_position[(GAME_NUM_CHANNELS-done)], HEX);
          smbus->writeWord(ltc2937_i2c_address, seq_position[(GAME_NUM_CHANNELS-done)], 0x0000);
          //Gy****if successfully advance to next button, play a suitably happy sound
	  drip(100);
          //******return to step F
          // increment score based upon delay value
          // (less delay = more points)
          Serial.print (F("DELAY = \n"));
          Serial.println (game_rand_delay, DEC);
          if (game_rand_delay < game_delay_5)
          {
            Serial.print (F("SCORE +5\n"));
            game_increment_score(5);
          }
          else if (game_rand_delay < game_delay_4)
          {
            Serial.print (F("SCORE +2\n"));
            game_increment_score(4);
          }
          else if (game_rand_delay < game_delay_3)
          {
            Serial.print (F("SCORE +3\n"));
            game_increment_score(3);
          }
          else if (game_rand_delay < game_delay_2)
          {
            Serial.print (F("SCORE +2\n"));
            game_increment_score(2);
          }
          else if (game_rand_delay < game_delay_1)
          {
            Serial.print (F("SCORE +1\n"));
            game_increment_score(1);
          }
          else
          {
            // add 0 points
            Serial.print (F("SCORE +0\n"));
          }
          Serial.print (F("SCORE = "));
          Serial.println (game_get_score(), DEC);

          done--;
          break;

      case 1 : // result was a fault
	buzz(2000);
	delay(1000);
	
	Serial.print (F("FAIL!\n"));
	Serial.println(done, DEC);
          Serial.print (F("ENDING THE GAME.\n"));
          Serial.print (F("FINAL SCORE = "));
          Serial.println (game_get_score(), DEC);

          done = -1;
          // exit the while loop and respond in disgrace
          break;
        default : //don't know what the result was
          Serial.print (F("OH OH! NO IDEA WHAT HAPPENED.\n"));
          done = -2;
          break;
      }
    }

    if (done == 0)
    {
      done = GAME_NUM_CHANNELS;  // reset for another go

      Serial.print (F("EXITED THE WHILE LOOP AFTER SUCCESSFULLY HITTING ALL BUTTONS.\n"));
      //increment breakpoint to finish the sequence
      game_inc_breakpoint(ltc2937_i2c_address);
      delay(500);
      //******sequence-down quickly
      game_sequence_down(ltc2937_i2c_address);
      //******return to step A

      //TODO:
      //****re-load new random values, shorten the time between breakpoints, and
      //******return to step F
      //****in this case, the player will ultimately lose due to insufficient reaction time (step Gx)
      Serial.print (F("ASSIGN NEW RANDOM SEQUENCE POSITIONS.\n"));
      game_random_seq_up_pos(ltc2937_i2c_address);

      game_easiness = (game_easiness > 10) ? (game_easiness - 10) : 0;
      Serial.print (F("ASSIGN RANDOM SEQUENCE UP TIMEOUT.\n"));
      game_random_seq_up_timeout(ltc2937_i2c_address) ;

      //C****set the breakpoint at sequence-position 0, breakpoint enabled
      Serial.print (F("SET BREAKPOINT TO 0.\n"));
      game_set_breakpoint(ltc2937_i2c_address, 0x0000);

      Serial.print (F("SEQUENCE-UP.\n"));
      game_sequence_up(ltc2937_i2c_address);

    }
    else if (done == -1)
    {
      Serial.print (F("EXITED THE WHILE LOOP IN DISMAL FAILURE.\n"));
      //****we exited the while loop with a failure
    }
    else
    {
      Serial.print (F("NO IDEA WHAT IS HAPPENING.\n"));
    }
  }
  while (done > 0);

  if (done == 0)
  {
    // it should be impossible to exit with done == 0

  }
  else if (done == -1)
  {
    Serial.print (F("EXITED THE WHILE LOOP IN DISMAL FAILURE.\n"));
    //****we exited the while loop with a failure

    //TODO:
    //Gx****if fault, play a wah wah wah sound for the loser
    //******blink all lights tauntingly
    //******reset for the next game, clear faults
    //******return to step A

  }
  else
  {
    Serial.print (F("NO IDEA WHAT IS HAPPENING.\n"));
  }
}

/************************************************************************/
// Function Definitions


//! write all registers to game defaults
int game_write_all_regs_game_default(uint8_t part_i2c_address)
{
  //  if( ltc2937_is_write_protected() != WRITE_PROTECTED) {
  //    if(ltc2937_is_down() == SUCCESS){
  Serial.print (F("INITIALIZING ALL REGISTERS.\n"));
  smbus->writeWord(part_i2c_address, LTC2937_WRITE_PROTECTION, 0x3AA);
  smbus->writeWord(part_i2c_address, LTC2937_SPECIAL_LOT, 0x2313);
  smbus->writeWord(part_i2c_address, LTC2937_ON_OFF_CONTROL, 0x0008);
  smbus->writeWord(part_i2c_address, LTC2937_V_RANGE, 0x0555);
  smbus->writeWord(part_i2c_address, LTC2937_V_THRESHOLD_1, 0x5A5A);
  smbus->writeWord(part_i2c_address, LTC2937_V_THRESHOLD_2, 0x5A5A);
  smbus->writeWord(part_i2c_address, LTC2937_V_THRESHOLD_3, 0x5A5A);
  smbus->writeWord(part_i2c_address, LTC2937_V_THRESHOLD_4, 0x5A5A);
  smbus->writeWord(part_i2c_address, LTC2937_V_THRESHOLD_5, 0x5A5A);
  smbus->writeWord(part_i2c_address, LTC2937_V_THRESHOLD_6, 0x5A5A);

  smbus->writeWord(part_i2c_address, LTC2937_TON_TIMERS_1, 0xE000);
  //smbus->writeWord(part_i2c_address, LTC2937_TON_TIMERS_1, 0x0000);
  smbus->writeWord(part_i2c_address, LTC2937_TON_TIMERS_2, 0xE000);
  //smbus->writeWord(part_i2c_address, LTC2937_TON_TIMERS_2, 0x0000);
  smbus->writeWord(part_i2c_address, LTC2937_TON_TIMERS_3, 0xE000);
  //smbus->writeWord(part_i2c_address, LTC2937_TON_TIMERS_3, 0x0000);
  smbus->writeWord(part_i2c_address, LTC2937_TON_TIMERS_4, 0xE000);
  //smbus->writeWord(part_i2c_address, LTC2937_TON_TIMERS_4, 0x0000);
  smbus->writeWord(part_i2c_address, LTC2937_TON_TIMERS_5, 0xE000);
  //smbus->writeWord(part_i2c_address, LTC2937_TON_TIMERS_5, 0x0000);
  smbus->writeWord(part_i2c_address, LTC2937_TON_TIMERS_6, 0xE000);
  //smbus->writeWord(part_i2c_address, LTC2937_TON_TIMERS_6, 0x0000);

  smbus->writeWord(part_i2c_address, LTC2937_TOFF_TIMERS_1, 0x8000);
  smbus->writeWord(part_i2c_address, LTC2937_TOFF_TIMERS_2, 0x8000);
  smbus->writeWord(part_i2c_address, LTC2937_TOFF_TIMERS_3, 0x8000);
  smbus->writeWord(part_i2c_address, LTC2937_TOFF_TIMERS_4, 0x8000);
  smbus->writeWord(part_i2c_address, LTC2937_TOFF_TIMERS_5, 0x8000);
  smbus->writeWord(part_i2c_address, LTC2937_TOFF_TIMERS_6, 0x8000);
  smbus->writeWord(part_i2c_address, LTC2937_SEQ_UP_POSITION_1, 0x0001);
  smbus->writeWord(part_i2c_address, LTC2937_SEQ_UP_POSITION_2, 0x0002);
  smbus->writeWord(part_i2c_address, LTC2937_SEQ_UP_POSITION_3, 0x0003);
  smbus->writeWord(part_i2c_address, LTC2937_SEQ_UP_POSITION_4, 0x0004);
  smbus->writeWord(part_i2c_address, LTC2937_SEQ_UP_POSITION_5, 0x0005);
  smbus->writeWord(part_i2c_address, LTC2937_SEQ_UP_POSITION_6, 0x0006);
  smbus->writeWord(part_i2c_address, LTC2937_SEQ_DOWN_POSITION_1, 0x0001);
  smbus->writeWord(part_i2c_address, LTC2937_SEQ_DOWN_POSITION_2, 0x0001);
  smbus->writeWord(part_i2c_address, LTC2937_SEQ_DOWN_POSITION_3, 0x0001);
  smbus->writeWord(part_i2c_address, LTC2937_SEQ_DOWN_POSITION_4, 0x0001);
  smbus->writeWord(part_i2c_address, LTC2937_SEQ_DOWN_POSITION_5, 0x0001);
  smbus->writeWord(part_i2c_address, LTC2937_SEQ_DOWN_POSITION_6, 0x0001);
  smbus->writeWord(part_i2c_address, LTC2937_RSTB_CONFIG, 0x0000);
  smbus->writeWord(part_i2c_address, LTC2937_FAULT_RESPONSE, 0x0010);
  smbus->writeWord(part_i2c_address, LTC2937_BREAK_POINT, 0x0001);
  smbus->writeWord(part_i2c_address, LTC2937_DEVICE_ID, 0x2937);

  return SUCCEED;
}



//!with some small probability, set the time-out register to 200ms (instead of 655ms)
int game_random_seq_up_timeout(uint8_t part_i2c_address)
{
  uint16_t ton_timers[6];

  // game_easiness should be a large positive integer (like 100) to determine the probability of short timeout
  // smaller is more likely

  ton_timers[0] = LTC2937_TON_TIMERS_1;
  ton_timers[1] = LTC2937_TON_TIMERS_2;
  ton_timers[2] = LTC2937_TON_TIMERS_3;
  ton_timers[3] = LTC2937_TON_TIMERS_4;
  ton_timers[4] = LTC2937_TON_TIMERS_5;
  ton_timers[5] = LTC2937_TON_TIMERS_6;

  int i, rnd;

  for (i = 0; i < 6; i++)
  {
    //    rnd = random(0,game_easiness);  // generate a random number
    rnd = 2;
    
    if (rnd < 2)
    {
      // use 164ms timeout (nearly certain user failure!)
      Serial.print(F("CH "));
      Serial.print(i, DEC);
      Serial.println(F(" 164ms timeout! "));
      smbus->writeWord(part_i2c_address, ton_timers[i], 0xC000);
    }
    else
    {
      //use 655ms timeout (normal)
      Serial.print(F("CH "));
      Serial.print(i, DEC);
      Serial.println(F(" 655ms timeout "));
      smbus->writeWord(part_i2c_address, ton_timers[i], 0xE000);
    }
  }
}


//!set random values into the sequence-up position registers
int game_random_seq_up_pos(uint8_t part_i2c_address)
{

  int i, x, rnd;
  uint8_t z;

  rnd = random(0,game_easiness);  // generate a random number

  // may only want to do this at game start
  // letting this evaolve might be more interesting
  //  seq_position[0] = LTC2937_SEQ_UP_POSITION_1;
  //  seq_position[1] = LTC2937_SEQ_UP_POSITION_2;
  //  seq_position[2] = LTC2937_SEQ_UP_POSITION_3;
  //  seq_position[3] = LTC2937_SEQ_UP_POSITION_4;
  //  seq_position[4] = LTC2937_SEQ_UP_POSITION_5;
  //  seq_position[5] = LTC2937_SEQ_UP_POSITION_6;

  for (i = 0; i < 6; i++)
  {
    x = random(0,6);  // generate a random sequence position
    //swap the locations i and x
    // keep track of the channels so we can address them later
    z = seq_position[i];
    seq_position[i] = seq_position[x];
    seq_position[x] = z;
  }
  // after this loop, each array location has been swapped with at least one other
  // the numbers should be fairly randomly-ordered, with no duplicates

  if (rnd < 2000)
  {
    // double-up two channels into one sequence position (the unused seq-pos remains empty)
    Serial.println(F("SEQUENCE ORDER.\nCHn\tSEQ_POS:"));
    smbus->writeWord(part_i2c_address, seq_position[0], (0x0000 | uint16_t(1)));
    for (i = 1; i < 6; i++)
    {
      // write the sequence order to the LTC2937
      smbus->writeWord(part_i2c_address, seq_position[i], (0x0000 | uint16_t(i+1)));
      Serial.print(seq_position[i], HEX);
      Serial.print(F("\t"));
      Serial.println(i, HEX);
    }
  }
  else
  {
    // don't change anything in the random order
    Serial.println(F("SEQUENCE ORDER.\nCHn\tSEQ_POS:"));
    for (i = 0; i < 6; i++)
    {
      // write the sequence order to the LTC2937
      smbus->writeWord(part_i2c_address, seq_position[i], (0x0000 | uint16_t(i+1)));
      Serial.print(seq_position[i], HEX);
      Serial.print(F("\t"));
      Serial.println(i, HEX);
    }
  }
}


//! set the breakpoint to the given value
int game_set_breakpoint(uint8_t device_address, uint16_t set_val)
{
  uint16_t return_val;

  if ((set_val >=0) && (set_val < 1024))
  {
    return_val = 0x0400 + set_val;  // set the enable bit and break_point value
    smbus->writeWord(device_address, LTC2937_BREAK_POINT, return_val);
    return SUCCEED;
  }
  else
  {
    return FAIL;
  }
}

//! increment the breakpoint by 1
int game_inc_breakpoint(uint8_t device_address)
{
  uint16_t return_val;
  uint16_t bp_en_val;
  uint16_t bp_ct_val;

  //  read the existing breakpoint
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
    return SUCCEED;
  }
  else
  {
    //    Serial.print(F("\n ERROR! BREAKPOINT VALUE OUT OF RANGE.\n"));
    return FAIL;
  }
}

//! light-up the START pushbutton
int  game_set_start_led_on()
{
  uint8_t data_array[2]; // an array of segments
  uint8_t data_mask[2];

  // assumes that digits read 00
  data_array[1] = game_int_to_7_segment(0);
  data_array[0] = game_int_to_7_segment(0) | 0x01;
  data_mask[0] = 0xFF;
  data_mask[1] = 0xFF;

  send_pcf8575_pins(pcf8575_i2c_address, data_array, data_mask);
  
  return SUCCEED;
}

//! turn-off the START pushbutton LED
int  game_set_start_led_off()
{
  uint8_t data_array[2]; // an array of segments
  uint8_t data_mask[2];

  // assumes that digits read 00
  data_array[1] = game_int_to_7_segment(0);
  data_array[0] = game_int_to_7_segment(0) & 0xFE;
  data_mask[0] = 0xFF;
  data_mask[1] = 0xFF;

  send_pcf8575_pins(pcf8575_i2c_address, data_array, data_mask);
  
  return SUCCEED;
}

//wait until start button is pressed
int game_get_start(uint8_t device_address)
{
  uint8_t dat_array[2];
  uint8_t data_mask[2];
  uint8_t data_cmp[2];
  int count = 0;
  
  // pause until the signal comes back from the start button
  do {
    //poll the start button
    //    Serial.print(F("Start button: "));
    //    Serial.println(analogRead(A0));
    delay(1);
    count++;
    if((count > 200)&&(count < 400)) {
      game_set_start_led_on();
    }
    if(count > 400) {
      game_set_start_led_off();
      count = 0;
    }
    //  } while (1);
  } while (analogRead(A0) < 500); // read analog voltage, wait until it is high

  //  return read_int();

  Serial.println (F("START BUTTON PRESSED."));
  return 1;
  
}

//! sequence-up the LTC2937
int game_sequence_up(uint8_t device_address)
{
  uint16_t return_val;
  Serial.print (F("\nSTARTING THE SEQUENCE.\n"));
  //  return_val = smbus->readWord(device_address, LTC2937_ON_OFF_CONTROL);
  //  return_val = (return_val & 0xFFEB); // mask the bits of interest
  //  return_val = (return_val | 0x0018);  // set the ON/OFF control to ON
  return_val = 0x0018;
  smbus->writeWord(device_address, LTC2937_ON_OFF_CONTROL, return_val);
  return SUCCEED;
}

//! sequence-down the LTC2937
int game_sequence_down(uint8_t device_address)
{
  uint16_t return_val;

  Serial.print (F("\nSEQUENCING DOWN.\n"));
  return_val = smbus->readWord(device_address, LTC2937_ON_OFF_CONTROL);
  return_val = (return_val & 0xFFEF); // mask the bits
  smbus->writeWord(device_address, LTC2937_ON_OFF_CONTROL, return_val);
  return SUCCEED;
}

//! wait for the time-out period, then check the LTC2937 for success or failure
int game_wait_for_result(uint8_t device_address, int timeout)
{
  uint16_t return_val1, return_val2;
  int ret;
  
  Serial.print (F("\nWAITING FOR TIMEOUT.\n"));
  delay(timeout);


  //STATUS_INFORMATION == seq-up in progres and  seq_up_fault == 1
  //MONITOR_STATUS_HISTORY[15:13] == which channel is currently sequencing
  return_val1 = smbus->readWord(device_address, LTC2937_STATUS_INFORMATION); // bit7
  return_val1 = smbus->readWord(device_address, LTC2937_STATUS_INFORMATION); // bit7
  return_val1 = smbus->readWord(device_address, LTC2937_STATUS_INFORMATION); // bit7
  return_val1 = smbus->readWord(device_address, LTC2937_STATUS_INFORMATION); // bit7
  //return_val2 = smbus->readWord(device_address, LTC2937_MONITOR_STATUS_HISTORY); // bit15:13
  Serial.print (F("DONE WAITING FOR TIMEOUT. STATUS_INFORMATION = "));
  Serial.println(return_val1, HEX);

  if ((return_val1 & 0x0080) == 0x0000)
  {
    ret = 0; // pass
    Serial.print (F("PASS\n"));
  }
  else   //
  {
    ret = 1; // fail
    Serial.print (F("FAIL\n"));
  }
  //  Serial.print (F("PRESS ANY KEY TO CONTINUE\n"));
  //  read_int();
  return ret;
}

//! blink lights randomly for a given time
int  game_play_random_lights(uint8_t device_address, int timeout)
{

  int i, j = 0;
  int del = 50; // how long to delay between changing LEDs
  int  freq = tune_1[0]; // what note to play
  uint8_t ch;
  uint16_t on_off;

  for (i = 0; i < (timeout/del); i++)
  {
    ch = random(0,6);
    //    freq = random(200, 1000);
    on_off = random(0,2048);

    smbus->writeWord(device_address, seq_position[ch], (0x0400&on_off));
    // play a tone on linduino pin 8
    tone(8, freq);
    delay(del/2);
    noTone(8);
    delay(del/2);
    freq = tune_1[j];
    j = (j < tune_1_max) ? j+1 : 0;
  }

  // turn-off all LEDs
  for (i = 0; i < GAME_NUM_CHANNELS; i++)
  {
    smbus->writeWord(device_address, seq_position[i], 0x0000);
    noTone(8);
  }
  return SUCCEED;
}

//! blink all LTC2937 lights together a few times
int game_blink_all_lights(uint8_t device_address)
{

  int i;

  uint16_t ch_val[GAME_NUM_CHANNELS];

  ch_val[0] = smbus->readWord(device_address, LTC2937_SEQ_UP_POSITION_1);
  ch_val[1] = smbus->readWord(device_address, LTC2937_SEQ_UP_POSITION_2);
  ch_val[2] = smbus->readWord(device_address, LTC2937_SEQ_UP_POSITION_3);
  ch_val[3] = smbus->readWord(device_address, LTC2937_SEQ_UP_POSITION_4);
  ch_val[4] = smbus->readWord(device_address, LTC2937_SEQ_UP_POSITION_5);
  ch_val[5] = smbus->readWord(device_address, LTC2937_SEQ_UP_POSITION_6);

  for (i = 0; i < 4; i++)
  {
    //set all channels to asynchronous
    //turn-on all lights
    //Serial.print (F("\nALL LEDs ON\n"));
    smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_1, 0x0400);
    smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_2, 0x0400);
    smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_3, 0x0400);
    smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_4, 0x0400);
    smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_5, 0x0400);
    smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_6, 0x0400);

    //wait
    delay (250);
    //turn-off all lights
    //Serial.print (F("ALL LEDs OFF\n"));
    smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_1, 0x0000);
    smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_2, 0x0000);
    smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_3, 0x0000);
    smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_4, 0x0000);
    smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_5, 0x0000);
    smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_6, 0x0000);
    delay (250);
  }
  //restore all channels to previous values
  smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_1, ch_val[0]);
  smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_2, ch_val[1]);
  smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_3, ch_val[2]);
  smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_4, ch_val[3]);
  smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_5, ch_val[4]);
  smbus->writeWord(device_address, LTC2937_SEQ_UP_POSITION_6, ch_val[5]);

  return SUCCEED;
}


//! issue a clear command
int game_clear_fault()
{
  //        smbus->readWord(ltc2937_i2c_address, LTC2937_CLEAR);
  smbus->sendByte(ltc2937_i2c_address, LTC2937_CLEAR);
  return SUCCEED;
}


//! reset the score to an initial value
int game_reset_score(int initial)
{
  // update the score display
  game_display_digits(initial);

  return SUCCEED;
}

//! add the increment value to the score
int game_increment_score(int increment)
{
  // update the score variable
  game_score += increment;

  // update the score display
  game_display_digits(game_score);

  return SUCCEED;
}

//! return the value of the score
int game_get_score()
{
  return game_score;
}

//! translate an integer between 0 and 9 into 7-segment encoding
// custom 7-segment encoding for the game
// 1 turns on a segment, 0 turns it off
uint8_t game_int_to_7_segment(int value)
{
  switch (value)
  {
    case 0 :
      return 0xF6;  // to invert the bits: (0xFF - 0xF6)
      break;
    case 1 :
      return 0x90;
      break;
    case 2 :
      return 0xCE;
      break;
    case 3 :
      return 0xDA;
      break;
    case 4 :
      return 0xB8;
      break;
    case 5 :
      return 0x7A;
      break;
    case 6 :
      return 0x7E;
      break;
    case 7 :
      return 0xD0;
      break;
    case 8 :
      return 0xFE;
      break;
    case 9 :
      return 0xF8;
      break;
    default :
      return 0x00; // cannot display integers > 9

  }
}

//! light-up some random LED segments on the scoreboard
// timeout is in milliseconds
int game_chasing_score(int timeout)
{
  int state = 0; //state machine variable
  int timer = 0;
  int step_time = 200; //milliseconds

  uint8_t data_array[2]; // an array of segments
  uint8_t data_mask[2];

  data_mask[0] = 0xFF;
  data_mask[1] = 0xFF;

  do    // this loop measures time
  {
    switch (state)
    {
      case 0 :
        data_array[1] = 0x02;
        data_array[0] = 0x02;
        state = 1;
        break;
      case 1 :
        data_array[1] = 0x04;
        data_array[0] = 0x10;
        state = 2;
        break;
      case 2 :
        data_array[1] = 0x20;
        data_array[0] = 0x80;
        state = 3;
        break;
      case 3 :
        data_array[1] = 0x40;
        data_array[0] = 0x40;
        state = 4;
        break;
      case 4 :
        data_array[1] = 0x80;
        data_array[0] = 0x20;
        state = 5;
        break;
      case 5 :
        data_array[1] = 0x10;
        data_array[0] = 0x04;
        state = 0;
        break;
      default :
        data_array[1] = 0xFF;
        data_array[0] = 0xFF;
        state = 0;
        break;
    }
    send_pcf8575_pins(pcf8575_i2c_address, data_array, data_mask);
    
    delay (step_time);
    timer += step_time;
  }
  while (timer < timeout);

  return 1;
}

//! display digits on the scoreboard
// input value should be between 00 and 99
int game_display_digits(int value)
{

  uint8_t data_array[2]; // an array of segments
  uint8_t data_mask[2];
  int upper, lower;

  if ((value >= 0) && (value < 100))
  {
    //translate the digits into 7-segment codes
    upper = (value > 9) ? (int)(value/10) : 0;
    lower = value - (10*upper);
    Serial.println(F("RESET SCORE TO:  "));
    Serial.print(upper, DEC);
    Serial.print(F("\t"));
    Serial.println(lower, DEC);
    data_array[1] = game_int_to_7_segment(upper);
    data_array[0] = game_int_to_7_segment(lower);
    data_mask[0] = 0xFF;
    data_mask[1] = 0xFF;

    send_pcf8575_pins(pcf8575_i2c_address, data_array, data_mask);

    return 0; //success
  }
  else
  {
    return 1; // out-of-range fail
  }
}


void game_count_down() {
  game_reset_score(99);
  //  tone(8,60);
  drip(100);
  delay(100);
  noTone(8);
  delay(400);
  game_reset_score(88);
  //  tone(8,60);
  bloop(100);
  delay(100);
  noTone(8);
  delay(400);
  game_reset_score(77);
  //  tone(8,60);
  drip(100);
  delay(100);
  noTone(8);
  delay(400);
  game_reset_score(66);
  //  tone(8,60);
  bloop(100);
  delay(100);
  noTone(8);
  delay(400);
  game_reset_score(55);
  //  tone(8,60);
  drip(100);
  delay(100);
  noTone(8);
  delay(400);
  game_reset_score(44);
  //  tone(8,60);
  bloop(100);
  delay(100);
  noTone(8);
  delay(400);
  game_reset_score(33);
  //  tone(8,60);
  drip(100);
  delay(100);
  noTone(8);
  delay(400);
  game_reset_score(22);
  //  tone(8,60);
  bloop(100);
  delay(100);
  noTone(8);
  delay(400);
  game_reset_score(11);
  //  tone(8,60);
  drip(100);
  delay(100);
  noTone(8);
  delay(400);
  game_reset_score(00);
  //  tone(8,60);
  bloop(100);
  delay(100);
  noTone(8);
  delay(400);
}


//! Read 2 bytes as the two 8-bit input words, P07, P06, ..., P00, P17, ..., P10
// data_array must be an array of length == 2
// note that actually reading a bit requires setting it high first, then readong to see if it is low
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

//! Write 2 bytes of data to the PCF8575
// data_array and io_data_mask must be arrays of length == 2
int send_pcf8575_pins(uint8_t device_addr, uint8_t *data_array, uint8_t *io_data_mask)
{
  uint8_t *data = (uint8_t *)malloc(2*sizeof(uint8_t));

  int i;

  for (i = 0; i < 2; i++)
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

//! poll a single bit (bit_num) by setting it high, then reading to see if it is pulled low
// return the value of the bit (1/0)
// 0 <= bit_num <= 7 or 10 <= bit_num <= 17  - per the device datasheet
int poll_pcf8575_bit(uint8_t device_addr, uint8_t *data_array, int bit_num)
{
  //????????????????????????????
  //  uint8_t data_array[2]; // an array of segments
    uint8_t data_mask[2];

    // first read to get the register contents
    get_pcf8575_pins(device_addr, data_array);
    
    // next set the desired bit high, leaving all other bits untouched
    send_pcf8575_pins(device_addr, data_array, data_mask);
    
}



////////////////////////////////////////////////////////////////////////////////////////////
// sound functions

//! Make a rising pitch sound covering the given duration
int drip(int duration)
{
  int freq_start = 60;
  int freq_stop = 2000;
  int num_steps = 10;
  int freq_step = (freq_stop - freq_start) / num_steps;
  
  int freq = freq_start;

  //divide-up the range into equal steps
  for (freq = freq_start; freq <= freq_stop; freq += freq_step)
    {
      tone(8,freq); 
      delay(duration / num_steps);
    }
  noTone(8);
}

//! Make a falling pitch sound covering the given duration
int bloop(int duration)
{
  int freq_start = 60;
  int freq_stop = 2000;
  int num_steps = 10;
  int i;
  int freq_step = (freq_stop - freq_start) / num_steps;
  
  int freq = freq_start;

  //divide-up the range into equal steps
  for (freq = freq_stop; freq >= freq_start; freq -= freq_step)
    {
      tone(8,freq); 
      delay(duration / num_steps);
    }
  noTone(8);
}

int buzz(int duration)
{
  int freq_a = 55;
  int freq_b = 100;
  int time = 60;
  int i;

  int bits = (int)((float)duration / (float)time); // how many "time" periods are there in duration - should be an integer

  bits = (int)((float)bits/2.0); // there are two tones in one cycle
  
  for(i = 0; i <= bits; i++) {
    tone(8,freq_a);
    delay(time);
    tone(8,freq_b);
    delay(time);
  }
  noTone(8);
  return 1;
}
