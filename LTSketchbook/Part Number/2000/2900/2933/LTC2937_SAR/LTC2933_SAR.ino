/*
  This code assumes that the channel is off-line, and not participating in supervision.
  The channel thresholds must be continuously controlled by the algorithm to detect ripple.
 */

#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "UserInterface.h"
#include "LT_I2CBus.h"
#include "LT_SMBusNoPec.h"
#include "LTC2933.h"

// LTC2637 DAC on the DC1633B board
#define DC1633_DAC_ADDRESS 0x22 //pin-strapped address for DAC

//LTC2933 I2C address on the board (selectable by a jumper)
#define LTC2933_I2C_ADDRESS LTC2933_I2C_GLOBAL_ADDRESS

// Global variables
static uint8_t ltc2933_i2c_address;
static uint8_t dc1633_dac_address;
static LT_SMBus *smbus = new LT_SMBusNoPec();

enum MACHINE_STATES {M_ACQUIRING = 0, M_TRACKING = 1};

static int MACHINE_STATE;
static int sample_count = 0;     // count number of samples taken by filter
const int max_sample_count = 48; // number of samples for filter to settle

static float thresholds[6] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0}; // state for the ripple detect function

void setup()
{
  MACHINE_STATE = M_ACQUIRING;

  Serial.begin(115200);         //! Initialize the serial port to the PC
  print_title();
  ltc2933_i2c_address = LTC2933_I2C_ADDRESS;
  dc1633_dac_address = DC1633_DAC_ADDRESS;
  
  ////
  //! write to the LTC2637 DACs on the DC1633 board to interesting voltages
  //!  other than the 2.0v DAC defaults
  dc1633_write_dac_voltage(dc1633_dac_address, 0, 3.3);
  dc1633_write_dac_voltage(dc1633_dac_address, 1, 2.5);
  dc1633_write_dac_voltage(dc1633_dac_address, 2, 1.8);
  dc1633_write_dac_voltage(dc1633_dac_address, 3, 1.5);
  dc1633_write_dac_voltage(dc1633_dac_address, 4, 1.2);
  dc1633_write_dac_voltage(dc1633_dac_address, 5, 1.0);
  //! write all LTC2933 registers with corresponding interesting voltage settings
  ltc2933_demo_board_demo_thresholds(ltc2933_i2c_address);
  ////
  Serial.println(F("SAR_VAL , FILTERED , THRESHOLD"));

}

void loop()
{
  float sar_value;  // output of a single SAR search
  float filtered_sar_value; // SAR values filtered by an IIR filter
  int ch_num = 2;
  
  // create a random voltage waveform
  dc1633_random_dac_voltage((ch_num-1), 2.5, 0.2);

  switch (MACHINE_STATE) {
  case M_ACQUIRING :
    sar_value = LTC2933_SAR_search(ltc2933_i2c_address, ch_num);
    filtered_sar_value = filter_2_2(sar_value);
    
    break;
  default :
    MACHINE_STATE = M_ACQUIRING;
    sample_count = 0;
    break;
  }
  // force the other DACs to output the captured and the filtered voltage
  dc1633_write_dac_voltage(dc1633_dac_address, 2, filtered_sar_value);
  dc1633_write_dac_voltage(dc1633_dac_address, 3, sar_value);

  Serial.print(sar_value, DEC);
  Serial.print(F(" , "));
  Serial.print(filtered_sar_value, DEC);
  //  Serial.print(F(" , "));
  //  Serial.print(thresholds[ch_num-1], DEC);
  Serial.println(F(" , "));

}

///////////////////////////////////////////////////////////////////////////////////////////////
// Function Definitions

//! Prints the title block when program first starts.
void print_title()
{
  Serial.print(F("\n*****************************************************************\n"));
  Serial.print(F("* DC1633B Demonstration Program                                 *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("* This program demonstrates how to implement a simple SAR ADC   *\n"));
  Serial.print(F("* using one channel of the LTC2937 on the DC1633B demo board.   *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("* Set the baud rate to 115200 and select the newline terminator.*\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("*****************************************************************\n"));
}


////////////////////////////////////////////////////////////////////////////////////////


//! Perform a SAR search for the voltage on channel <ch_num>
float LTC2933_SAR_search(uint8_t ltc2933_i2c_address, int ch_num)
{
  uint16_t save_vn_thr = 0x0000; //save the vn register state during the SAR search
  uint16_t save_vn_config = 0x0000; //save the vn register state during the SAR search
  uint16_t mod_vn_thr = 0x0000; //modified vn register state during the SAR search
  uint16_t mod_vn_config = 0x0000; //modified vn register state during the SAR search
  uint16_t ltc2933_status, ltc2933_status_mask;
  
  //  int bit = 7; // counter
  uint16_t sar = 0x8000; // sar register
  
  //  const int settling_time = 10; // number of milliseconds for settling between SAR steps
  const int settling_time = 1; // number of milliseconds for settling between SAR steps

  float result = 0.0;
    
  if ((ch_num > 1) && (ch_num <= 6)) {
    // CH1 is different from the others, so it doesn't play nicely with this function
    save_vn_thr = smbus->readWord(ltc2933_i2c_address, (LTC2933_V1_THR + ch_num-1));
    save_vn_config = smbus->readWord(ltc2933_i2c_address, (LTC2933_V1_CONFIG + ch_num-1));
    
    // modify CHn, leave the others alone
    mod_vn_config = (save_vn_config & 0xFF47) | 0x0080; // touch only the HI comparator
    
    mod_vn_thr = (save_vn_thr & 0x00FF) | 0x8000; // set HI comp threshold

    sar = 0x8000;

    //    Serial.print(F("CONVERTING CHANNEL "));
    //    Serial.println(ch_num, DEC);
    switch (ch_num)
      {
      case 1 :
	ltc2933_status_mask = 0x0004;
	break;
      case 2 :
	ltc2933_status_mask = 0x0010;
	break;
      case 3 :
	ltc2933_status_mask = 0x0040;
	break;
      case 4 :
	ltc2933_status_mask = 0x0100;
	break;
      case 5 :
	ltc2933_status_mask = 0x0400;
	break;
      case 6 :
	ltc2933_status_mask = 0x1000;
	break;
      default :
	ltc2933_status_mask = 0x0000;
	break;
      }
	
	  
    //set the config reg
    // leave the RANGE unchanged, but remove the channel from GPIO mapping
    smbus->writeWord(ltc2933_i2c_address, (LTC2933_V1_CONFIG + ch_num-1), mod_vn_config);
    
    
    while(sar > 0x00FF) {
      //set the threshold
      smbus->writeWord(ltc2933_i2c_address, (LTC2933_V1_THR + ch_num-1), mod_vn_thr);
      
      //wait for settling
      delay(settling_time);
      
      //read the status register, mask for the Vn_HI_FAULT bit
      ltc2933_status = (ltc2933_status_mask & smbus->readWord(ltc2933_i2c_address, LTC2933_STATUS_WORD));
      
      //interpret the result
      // if 1, then the voltage is above the threshold, if 0 then below
      if(ltc2933_status != 0x0000) {
	// keep the bit set and move-on to the next LSB
	sar = sar >> 1; // shift right
	mod_vn_thr = mod_vn_thr | sar;
	// Serial.println(F("HIGHER "));
      }
      else {
	// clear the bit and move-on to the next LSB
	mod_vn_thr = (mod_vn_thr & (~sar));
	sar = sar >> 1; // shift right
	mod_vn_thr = mod_vn_thr | sar;
	//  Serial.println(F("LOWER "));
      }
      // Serial.println(F("HIT RETURN TO CONTINUE..."));
      // read_int();
      result = 0.45+(0.01 * (int)(mod_vn_thr>>8));
      Serial.println(result, DEC);
      //      Serial.println(mod_vn_thr, HEX);
      dc1633_write_dac_voltage(dc1633_dac_address, 4, result);
    }
  
    // upon exiting the loop the mod_vn_thr register upper byte contains the
    //  DAC threshold nearest to the pin voltage, but not above it
    mod_vn_thr = mod_vn_thr>>8;
    //    Serial.print(F("SAR RESULT (HEX): "));
    //    Serial.println(mod_vn_thr, HEX);
    //    Serial.print(F("SAR RESULT (VOLTS): "));
    // convert register value to volts, depending upon threshold range setting
    // NOTE: this conversion is for CH2 - CH6, NOT CH1
    switch (save_vn_config & 0x0300)
      {
      case 0x0300 :
      case 0x0200 :
	// precision range
	result = 0.18+(0.004 * (int)mod_vn_thr);
	//	Serial.println(result, DEC);
	break;
      case 0x0100 :
	// low range
	result = 0.45+(0.01 * (int)mod_vn_thr);
	//	Serial.println(result, DEC);
	break;
      case 0x0000 :
	// medium range
	result = 0.90+(0.02 * (int)mod_vn_thr);
	//	Serial.println(result, DEC);
	break;
      default :
	Serial.println(F("ERROR: BOGUS RANGE CODE IN Vn_CONFIG!"));
	break; 
	// NOTE: high range is only used in CH1
	// high range
	//  Serial.println((2.25+(0.05 * int(mod_vn_thr))), DEC);
      }
    // NOTE: restoring the register settings is a bit touchy
    // if you try to change the range or threshold settings from what they were
    //  before the SAR algotithm you can cause a fault as the thresholds change
    smbus->writeWord(ltc2933_i2c_address, (LTC2933_V1_THR + ch_num-1), save_vn_thr); // restore Vn config
    smbus->writeWord(ltc2933_i2c_address, (LTC2933_V1_CONFIG + ch_num-1), save_vn_config); // restore Vn config
    
    return (result);
  }
  else {
    // invalid channel number
    Serial.println(F("ERROR: INVALID CHANNEL NUMBER!"));
    return (0.0);
  }
}



///
//! Filter a series of real numbers with an IIR function
//! Input is a real number
//! Output is the filtered output over all history
//! http://www.eas.uccs.edu/wickert/ece2610/lecture_notes/ece2610_chap8.pdf
float filter_2_2(float x_0)
{
  // Butterworth filter
  // 
  const int N = 2; // feed-back network
  const int M = 2; // feed-forward network
  static float state_x[M+1] = {0.0, 0.0, 0.0}; // feed-forward states
  static float state_y[N+1] = {0.0, 0.0, 0.0}; // feed-back states
  //  const  float coeff_a[N+1] = {0.0,0.9428, -0.3333}; // feed-back coefficients; 0th coeff is unused
  const  float coeff_a[N+1] = {1.0, 1.7954767, -0.8145336 }; // feed-back coefficients; 0th coeff is unused
  //  const  float coeff_b[M+1] = {0.0976, 0.1953, 0.0976}; // feed-forward coefficients
  const  float coeff_b[M+1] = {0.0047642, 0.0095284, 0.0047642}; // feed-forward coefficients

  float x_p = 0.0;  //output of the feed-forward filter
  float y_p = 0.0;
  
  int i;

  for (i = M; i >= 0; i--) {
    //calculate the feed-forward filter
    state_x[i] = (i == 0) ? x_0 : state_x[i-1]; // shift the value in the delay chain
    x_p += (state_x[i] * coeff_b[i]);
  }
  //  Serial.print(F("x = "));
  //  Serial.println(x_p);

  for (i = N; i > 0; i--) {
    //calculate the feed-back filter
    state_y[i] = state_y[i-1]; // shift the value in the delay chain
    y_p += (state_y[i] * coeff_a[i]);
  }
  //  Serial.print(F("y = "));
  //  Serial.println(x_p);

  state_y[0] = y_p + x_p; // state_y[0] is the output of the filter
  return (state_y[0]);
}


//! program the DAC on the DC1633B demo board to a voltage
//  refer to the LTC2637 datasheet
void dc1633_write_dac_voltage(uint8_t dac_address, int channel, float voltage)
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
    Serial.println(F("\nERROR: Voltage out of DAC range"));
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



//! Load different voltage threshold settings into RAM
void ltc2933_demo_board_demo_thresholds(uint8_t ltc2933_i2c_address)
{
  if (ltc2933_is_write_protected(ltc2933_i2c_address) != 1)
  {
    smbus->writeWord(ltc2933_i2c_address, LTC2933_WRITE_PROTECT, 0xAAA8);
    smbus->writeWord(ltc2933_i2c_address, LTC2933_GPI_CONFIG, 0x1040); // GPI2=MARG, not mapped; GPI1=MR, clear history, not mapped
    smbus->writeWord(ltc2933_i2c_address, LTC2933_GPIO1_CONFIG, 0x002E);
    //    smbus->writeWord(ltc2933_i2c_address, LTC2933_GPIO2_3_CONFIG, 0x2E07);
        smbus->writeWord(ltc2933_i2c_address, LTC2933_GPIO2_3_CONFIG, 0x2E06);
    ////SINCE WE ARE FORCING OTHER CHANNELS DACs TO OUTPUT STRANGE VOLTAGES, TURN OFF FAULTS FOR THOSE CHANNELS
    //    smbus->writeWord(ltc2933_i2c_address, LTC2933_V1_THR, 0x412D); // ov = 5.5, uv = 4.5
    smbus->writeWord(ltc2933_i2c_address, LTC2933_V1_THR, 0xFE00); 
    //    smbus->writeWord(ltc2933_i2c_address, LTC2933_V2_THR, 0x554B); // ov = 2.6, uv = 2.4
    smbus->writeWord(ltc2933_i2c_address, LTC2933_V2_THR, 0xFE00); 
    //    smbus->writeWord(ltc2933_i2c_address, LTC2933_V3_THR, 0x917D); // ov = 1.9, uv = 1.7
    smbus->writeWord(ltc2933_i2c_address, LTC2933_V3_THR, 0xFE00); 
    //    smbus->writeWord(ltc2933_i2c_address, LTC2933_V4_THR, 0x735F); // ov = 1.6, uv = 1.4
    smbus->writeWord(ltc2933_i2c_address, LTC2933_V4_THR, 0xFE00); 
    //    smbus->writeWord(ltc2933_i2c_address, LTC2933_V5_THR, 0x5541); // ov = 1.3, uv = 1.1
    smbus->writeWord(ltc2933_i2c_address, LTC2933_V5_THR, 0xFE00); 
    //    smbus->writeWord(ltc2933_i2c_address, LTC2933_V6_THR, 0x3C32); // ov = 1.05, uv = 0.95
    smbus->writeWord(ltc2933_i2c_address, LTC2933_V6_THR, 0xFE00); 
    smbus->writeWord(ltc2933_i2c_address, LTC2933_V1_CONFIG, 0x009C); // high range
    //    smbus->writeWord(ltc2933_i2c_address, LTC2933_V2_CONFIG, 0x009C); // medium range
    smbus->writeWord(ltc2933_i2c_address, LTC2933_V2_CONFIG, 0x0180); // low range
    smbus->writeWord(ltc2933_i2c_address, LTC2933_V3_CONFIG, 0x019C); // low range
    smbus->writeWord(ltc2933_i2c_address, LTC2933_V4_CONFIG, 0x019C); // low range
    smbus->writeWord(ltc2933_i2c_address, LTC2933_V5_CONFIG, 0x019C); // low range
    smbus->writeWord(ltc2933_i2c_address, LTC2933_V6_CONFIG, 0x019C); // low range
  }
  else
  {
    // error, LTC2933 is write-protected
    Serial.println(F("\nERROR: LTC2933 is write-protected. Cannot write to registers"));
  }
}

//! Clear ALERTB
void ltc2933_clear_alertb(uint8_t ltc2933_i2c_address)
{
  smbus->writeWord(ltc2933_i2c_address, LTC2933_CLEAR_HISTORY, 0x0000);
}


//! Return 1 if the LTC2933 is write-protected
//  0 otherwise
int ltc2933_is_write_protected(uint8_t ltc2933_i2c_address)
{
  uint16_t res;

  res = smbus->readWord(ltc2933_i2c_address, LTC2933_STATUS_WORD);

  //b[0] is the write-protect bit
  return ((res&0x0001) == 0x0001) ? 1 : 0;
}


//! write a random DAC voltage to ch_num on the DC1633B demo board
//! random voltages will be around mean and have maximum excursions of +/-ampl 
void dc1633_random_dac_voltage(int ch_num, float mean, float ampl) {
  float rand_voltage;

  rand_voltage = mean + ampl*2*(0.5 - (float)random(0,2048)/2048);
  //  Serial.print(F("RANDOM: "));
  //  Serial.println(rand_voltage);
  dc1633_write_dac_voltage(dc1633_dac_address, ch_num, rand_voltage);


}
