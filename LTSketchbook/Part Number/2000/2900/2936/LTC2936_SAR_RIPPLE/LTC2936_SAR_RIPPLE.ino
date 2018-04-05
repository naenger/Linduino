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
#include "LTC2936.h"
#include "butterworth_2_2.h"

// LTC2637 DAC on the DC1605B board
#define DC1605_DAC_ADDRESS 0x22 //pin-strapped address for DAC

//LTC2936 I2C address on the board (selectable by a jumper)
#define LTC2936_I2C_ADDRESS LTC2936_I2C_GLOBAL_ADDRESS

// Global variables
static uint8_t ltc2936_i2c_address;
static uint8_t dc1605_dac_address;
static LT_SMBus *smbus = new LT_SMBusNoPec();

enum MACHINE_STATES {M_ACQUIRING = 0, M_TRACKING = 1};

static int MACHINE_STATE;
static int sample_count = 0;     // count number of samples taken by filter
const int max_sample_count = 48; // number of samples for filter to settle

static float thresholds[6] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0}; // state for the ripple detect function

butterworth_2_2 filter_2_2(2.8);  // an instance of the butterworth filter

void setup()
{
  MACHINE_STATE = M_ACQUIRING;

  //  Serial.begin(115200);         //! Initialize the serial port to the PC
  //  print_title();
  ltc2936_i2c_address = LTC2936_I2C_ADDRESS;
  dc1605_dac_address = DC1605_DAC_ADDRESS;
  
  ////
  //! write to the LTC2637 DACs on the DC1615 board to interesting voltages
  //!  other than the 2.0v DAC defaults
  dc1605_write_dac_voltage(dc1605_dac_address, 0, 3.3);
  dc1605_write_dac_voltage(dc1605_dac_address, 1, 2.5);
  dc1605_write_dac_voltage(dc1605_dac_address, 2, 1.8);
  dc1605_write_dac_voltage(dc1605_dac_address, 3, 1.5);
  dc1605_write_dac_voltage(dc1605_dac_address, 4, 1.2);
  dc1605_write_dac_voltage(dc1605_dac_address, 5, 1.0);
  //! write all LTC2936 registers with corresponding interesting voltage settings
  ltc2936_demo_board_demo_thresholds(ltc2936_i2c_address);
  ////
  //  Serial.println(F("SAR_VAL , FILTERED , THRESHOLD"));

}

void loop()
//{
//  // a tight loop to see how fast we can drive the SAR
//  float sar_value;  // output of a single SAR search
//  int ch_num = 2;
//  sar_value = LTC2936_SAR_search(ltc2936_i2c_address, ch_num);
//  dc1605_write_dac_voltage(dc1605_dac_address, 3, sar_value);
//}
{
  float sar_value,  // output of a single SAR search
    ripple_val_low;
  float filtered_sar_value; // SAR values filtered by an IIR filter
  int ch_num = 2;
  
  // create a random voltage waveform
  //  dc1605_random_dac_voltage(dc1605_dac_address, (ch_num-1), 2.5, 0.1);
  // create a pulse train waveform
  dc1605_dac_voltage_pulse(dc1605_dac_address, (ch_num-1), 2.5, 0.1);
  delay (1);
  dc1605_dac_voltage_pulse(dc1605_dac_address, (ch_num-1), 2.5, 0.1);

  switch (MACHINE_STATE) {
  case M_ACQUIRING :
    // do not search for glitches while acquiring the average voltage
    sar_value = LTC2936_SAR_search(ltc2936_i2c_address, ch_num);
    filtered_sar_value = filter_2_2.increment(sar_value);
    
    if (sample_count < max_sample_count) {
      sample_count++;
      MACHINE_STATE = M_ACQUIRING;
    }
    else {
      MACHINE_STATE = M_TRACKING;
      thresholds[ch_num-1] = filtered_sar_value;
      LTC2936_init_ripple_thresh_low(ltc2936_i2c_address, ch_num, thresholds, filtered_sar_value);
      //      Serial.print(F("\n*********************************\n"));
      //      Serial.print(F("\nTHRESHOLD[ch_num-1] = "));
      //      Serial.println(thresholds[ch_num-1]);
    }
    break;
  case M_TRACKING :
    // search for glitches while tracking
    sar_value = LTC2936_SAR_search(ltc2936_i2c_address, ch_num);
    filtered_sar_value = filter_2_2.increment(sar_value);
    MACHINE_STATE = M_TRACKING;
    ripple_val_low = LTC2936_update_ripple_thresh_low(ltc2936_i2c_address, ch_num, thresholds);
//    Serial.print(F("\nTHRESHOLD[ch_num-1] = "));
//    Serial.println(thresholds[ch_num-1]);
    break;
  default :
    MACHINE_STATE = M_ACQUIRING;
    sample_count = 0;
    break;
  }
  // force the other DACs to output the voltages
  dc1605_write_dac_voltage(dc1605_dac_address, 2, filtered_sar_value);
  dc1605_write_dac_voltage(dc1605_dac_address, 3, sar_value);
  dc1605_write_dac_voltage(dc1605_dac_address, 4, ripple_val_low);

//  Serial.print(sar_value, DEC);
//  Serial.print(F(" , "));
//  Serial.print(filtered_sar_value, DEC);
//  Serial.print(F(" , "));
//  Serial.print(thresholds[ch_num-1], DEC);
//  Serial.println(F(" , "));
  
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Function Definitions

//! Prints the title block when program first starts.
void print_title()
{
  Serial.print(F("\n*****************************************************************\n"));
  Serial.print(F("* DC1605B Demonstration Program                                 *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("* This program demonstrates how to implement a simple SAR ADC   *\n"));
  Serial.print(F("* using one channel of the LTC2937 on the DC1605B demo board.   *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("* Set the baud rate to 115200 and select the newline terminator.*\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("*****************************************************************\n"));
}


////////////////////////////////////////////////////////////////////////////////////////


void LTC2936_init_ripple_thresh_low(uint8_t ltc2936_i2c_address, int ch_num, float *thresh, float voltage) {
  // initialize the ripple search algorithm with a sane initial value
  // assumes that the ch_num channel can be used for monitoring for UV faults on its LO comparator
  // assumes that ch_num can be querried for fault indication on the LO comp
  // assumes that ch_num Vn_THR_LO is not being modified by any other process
  // CLEARS the HISTORY_WORD if there is a fault on the LO comparator of ch_num, does not clear otherwise
  // updates the ch_num Vn_THR_LO reg according to the fault HISTORY_WORD
  // stores state in the thresh[] array. Each array element is a threshold voltage state
  // - returns a float representing the threshold voltage after modification
  
  uint16_t ltc2936_history, ltc2936_history_mask;
  uint16_t mod_vn_thr = 0x0000; //save the vn register state during the update
  uint16_t save_vn_config = 0x0000; //save the vn register state during the update
  uint16_t mod_vn_config = 0x0000; //save the vn register state during the update

  float result = 0.0;
    
  //  Serial.println(F("INIT_RIPPLE_THRESHOLD..."));
  if ((ch_num > 1) && (ch_num <= 6)) {
    // CH1 is different from the others, so it doesn't play nicely with this function
    //    save_vn_thr = smbus->readWord(ltc2936_i2c_address, (LTC2936_V1_THR + ch_num-1));
    save_vn_config = smbus->readWord(ltc2936_i2c_address, (LTC2936_V1_CONFIG + ch_num-1));

    switch (ch_num)
      {
      case 1 :
	ltc2936_history_mask = 0x0002;
	break;
      case 2 :
	ltc2936_history_mask = 0x0008;
	break;
      case 3 :
	ltc2936_history_mask = 0x0020;
	break;
      case 4 :
	ltc2936_history_mask = 0x0080;
	break;
      case 5 :
	ltc2936_history_mask = 0x0200;
	break;
      case 6 :
	ltc2936_history_mask = 0x0800;
	break;
      default :
	ltc2936_history_mask = 0x0000;
	break;
      }

    ////read HISTORY_WORD for ch_num
    //read the history register, mask for the Vn_LO_FAULT bit
    ltc2936_history = (ltc2936_history_mask & smbus->readWord(ltc2936_i2c_address, LTC2936_HISTORY_WORD));
    
    ////react to HISTORY_WORD for ch_num
    if(ltc2936_history != 0x0000) {
      // there was a fault
      //      Serial.print(F("VOLTAGE FAULT : "));
      //      Serial.println(ltc2936_history, HEX);
      
      // CLEAR_HISTORY faults
      //      Serial.print(F("CLEAR FAULT HISTORY..."));
      smbus->sendByte(ltc2936_i2c_address, LTC2936_CLEAR_HISTORY);
      //      Serial.println(F("DONE"));
    }


    switch (save_vn_config & 0x0300)
      {
      case 0x0300 :
      case 0x0200 :
	// precision range
	thresh[ch_num-1] = voltage;
	mod_vn_thr = 0x00FF & (uint16_t)((thresh[ch_num-1] - 0.18)/0.004);
	break;
      case 0x0100 :
	// low range
	thresh[ch_num-1] = voltage;
	mod_vn_thr = 0x00FF & (uint16_t)((thresh[ch_num-1] - 0.45)/0.01);
	break;
      case 0x0000 :
	// medium range
	thresh[ch_num-1] = voltage;
	mod_vn_thr = 0x00FF & (uint16_t)((thresh[ch_num-1] - 0.9)/0.02);
	break;
      default :
	//	Serial.println(F("ERROR: BOGUS RANGE CODE IN Vn_CONFIG!"));
	break; 
	// NOTE: high range is only used in CH1
	// high range
      }
    
    // write to the LTC2936 registers
    mod_vn_config = (save_vn_config & 0xFFB8) | 0x0000; // touch only the LO comparator
    delay(100);
    
    smbus->writeWord(ltc2936_i2c_address, (LTC2936_V1_THR + ch_num-1), mod_vn_thr);
    smbus->writeWord(ltc2936_i2c_address, (LTC2936_V1_CONFIG + ch_num-1), mod_vn_config);
  }
  else {
    // bad channel number
    //    Serial.print(F("BAD CHANNEL NUMBER: "));
    //    Serial.println(ch_num, DEC);
  }  
  //  Serial.println(F("DONE INIT_RIPPLE_THRESHOLD"));
}

////////////////////////////////////////////////////////////////////////////////////////

float LTC2936_update_ripple_thresh_low(uint8_t ltc2936_i2c_address, int ch_num, float thresh[]) {
  // assumes that the ch_num channel has been monitoring for UV faults on its LO comparator
  //  and can be configured to keep doing it
  // assumes that ch_num can be querried for fault indication on the LO comp
  // assumes that ch_num Vn_THR_LO is not being modified by any other process
  // CLEARS the HISTORY_WORD if there is a fault on the LO comparator of ch_num, does not clear otherwise
  // updates the ch_num Vn_THR_LO reg according to the fault HISTORY_WORD
  // stores state in the thresh[] array. Each array element is a threshold voltage state
  // - returns a float representing the threshold voltage after modification
  
  uint16_t ltc2936_history, ltc2936_history_mask;
  uint16_t mod_vn_thr = 0x0000; //save the vn register state during the update
  uint16_t save_vn_thr = 0x0000; //save the vn register state during the update
  uint16_t save_vn_config = 0x0000; //save the vn register state during the update
  uint16_t mod_vn_config = 0x0000; //save the vn register state during the update

  float result = 0.0;

  float gain = 0.0; // threshold register volts per bit
  float step = 0.0; // threshold register volts offset
  float offset = 0.0; // threshold register volts offset
  float max = 0.0; // threshold reg max voltage
  float min = 0.0; // threshold reg max voltage

  int fault = 0; // indicate if a fault was detected
  
  //  Serial.println(F("UPDATE_RIPPLE_THRESHOLD"));
  if ((ch_num > 1) && (ch_num <= 6)) {
    // CH1 is different from the others, so it doesn't play nicely with this function
    //    save_vn_thr = smbus->readWord(ltc2936_i2c_address, (LTC2936_V1_THR + ch_num-1));
    save_vn_config = smbus->readWord(ltc2936_i2c_address, (LTC2936_V1_CONFIG + ch_num-1));
    save_vn_thr = smbus->readWord(ltc2936_i2c_address, (LTC2936_V1_THR + ch_num-1));

    switch (ch_num)
      {
      case 1 :
	ltc2936_history_mask = 0x0002;
	break;
      case 2 :
	ltc2936_history_mask = 0x0008;
	break;
      case 3 :
	ltc2936_history_mask = 0x0020;
	break;
      case 4 :
	ltc2936_history_mask = 0x0080;
	break;
      case 5 :
	ltc2936_history_mask = 0x0200;
	break;
      case 6 :
	ltc2936_history_mask = 0x0800;
	break;
      default :
	ltc2936_history_mask = 0x0000;
	break;
      }

    ////read HISTORY_WORD for ch_num
    //read the history register, mask for the Vn_LO_FAULT bit
    ltc2936_history = (ltc2936_history_mask & smbus->readWord(ltc2936_i2c_address, LTC2936_HISTORY_WORD));

    ////react to HISTORY_WORD for ch_num
    if(ltc2936_history != 0x0000) {
      // there was a fault, indicating a threshold incursion
      //      Serial.print(F("F, ")); // indicate that a fault was detected
      switch (save_vn_config & 0x0300)
	{
	case 0x0300 :
	case 0x0200 :
	  // precision range
	  gain = 0.004;
	  step = -0.004;
	  offset = 0.18;
	  max = 1.2;
	  min = 0.2;
	  break;
	case 0x0100 :
	  // low range
	  gain = 0.01;
	  step = -0.01;
	  offset = 0.45;
	  max = 3.0;
	  min = 0.5;
	  break;
	case 0x0000 :
	  // medium range
	  gain = 0.02;
	  step = -0.02;
	  offset = 0.9;
	  max = 5.8;
	  min = 1.0;
	  break;
	default :
	  //	  Serial.println(F("ERROR: BOGUS RANGE CODE IN Vn_CONFIG!"));
	  break; 
	  // high range
	  // NOTE: high range is only used in CH1
	}

      // CLEAR_HISTORY faults
      fault = 1;
    }
    else {
      //no fault, no threshold incursion
      //      Serial.print(F(" , "));
      switch (save_vn_config & 0x0300)
	{
	case 0x0300 :
	case 0x0200 :
	  // precision range
	  gain = 0.004;
	  step = 0.0001;
	  //	  step = 0.004;
	  offset = 0.18;
	  max = 1.2;
	  min = 0.2;
	  break;
	case 0x0100 :
	  // low range
	  gain = 0.01;
	  step = 0.00025;
	  //	  step = 0.01;
	  offset = 0.45;
	  max = 3.0;
	  min = 0.5;
	  break;
	case 0x0000 :
	  // medium range
	  gain = 0.02;
	  step = 0.0005;
	  //	  step = 0.02;
	  offset = 0.9;
	  max = 5.8;
	  min = 1.0;
	  break;
	default :
	  //	  Serial.println(F("ERROR: BOGUS RANGE CODE IN Vn_CONFIG!"));
	  break; 
	  // high range
	  // NOTE: high range is only used in CH1
	}
    }
    
    //    delay(100); // in case there was a CLEAR_FAULT -- makes loop timing consistent, avoids NAC

    // update the LTC2936 registers
    thresh[ch_num-1] += step;
    if(thresh[ch_num-1] < min) {
      thresh[ch_num-1] = min;
    }
    else if(thresh[ch_num-1] > max) {
      thresh[ch_num-1] = max;
    }
      
    mod_vn_thr = (save_vn_thr & 0xFF00) | (0x00FF & (uint16_t)((thresh[ch_num-1] - offset)/gain));
    mod_vn_config = (save_vn_config & 0xFFB8) | 0x0000; // touch only the LO comparator

    smbus->writeWord(ltc2936_i2c_address, (LTC2936_V1_THR + ch_num-1), mod_vn_thr);
    smbus->writeWord(ltc2936_i2c_address, (LTC2936_V1_CONFIG + ch_num-1), mod_vn_config);
    if (fault != 0) {
      smbus->sendByte(ltc2936_i2c_address, LTC2936_CLEAR_HISTORY);
    }
  }
  else {
    // bad channel number
    //    Serial.print(F("BAD CHANNEL NUMBER: "));
    //    Serial.println(ch_num, DEC);
  }
  
  //  Serial.println(F("DONE UPDATE_RIPPLE_THRESHOLD"));
  delay(100); // in case there was a CLEAR_FAULT -- makes loop timing consistent, avoids NAC
  return (thresh[ch_num-1]);
}





//! Perform a SAR search for the voltage on channel <ch_num>
float LTC2936_SAR_search(uint8_t ltc2936_i2c_address, int ch_num)
{
  uint16_t save_vn_thr = 0x0000; //save the vn register state during the SAR search
  uint16_t save_vn_config = 0x0000; //save the vn register state during the SAR search
  uint16_t mod_vn_thr = 0x0000; //modified vn register state during the SAR search
  uint16_t mod_vn_config = 0x0000; //modified vn register state during the SAR search
  uint16_t ltc2936_status, ltc2936_status_mask;
  
  //  int bit = 7; // counter
  uint16_t sar = 0x8000; // sar register
  
  const int settling_time = 1; // number of milliseconds for settling between SAR steps

  float result = 0.0;
    
  if ((ch_num > 1) && (ch_num <= 6)) {
    // CH1 is different from the others, so it doesn't play nicely with this function
    save_vn_thr = smbus->readWord(ltc2936_i2c_address, (LTC2936_V1_THR + ch_num-1));
    save_vn_config = smbus->readWord(ltc2936_i2c_address, (LTC2936_V1_CONFIG + ch_num-1));
    
    // modify CHn, leave the others alone
    mod_vn_config = (save_vn_config & 0xFF47) | 0x0080; // touch only the HI comparator
    
    mod_vn_thr = (save_vn_thr & 0x00FF) | 0x8000; // set HI comp threshold

    sar = 0x8000;

    //    Serial.print(F("CONVERTING CHANNEL "));
    //    Serial.println(ch_num, DEC);
    switch (ch_num)
      {
      case 1 :
	ltc2936_status_mask = 0x0004;
	break;
      case 2 :
	ltc2936_status_mask = 0x0010;
	break;
      case 3 :
	ltc2936_status_mask = 0x0040;
	break;
      case 4 :
	ltc2936_status_mask = 0x0100;
	break;
      case 5 :
	ltc2936_status_mask = 0x0400;
	break;
      case 6 :
	ltc2936_status_mask = 0x1000;
	break;
      default :
	ltc2936_status_mask = 0x0000;
	break;
      }
	
	  
    //set the config reg
    // leave the RANGE unchanged, but remove the channel from GPIO mapping
    smbus->writeWord(ltc2936_i2c_address, (LTC2936_V1_CONFIG + ch_num-1), mod_vn_config);
    
    
    while(sar > 0x00FF) {
      //set the threshold
      smbus->writeWord(ltc2936_i2c_address, (LTC2936_V1_THR + ch_num-1), mod_vn_thr);
      
      //wait for settling
      delay(settling_time);
      
      //read the status register, mask for the Vn_HI_FAULT bit
      ltc2936_status = (ltc2936_status_mask & smbus->readWord(ltc2936_i2c_address, LTC2936_STATUS_WORD));
      
      //interpret the result
      // if 1, then the voltage is above the threshold, if 0 then below
      if(ltc2936_status != 0x0000) {
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
	//	Serial.println(F("ERROR: BOGUS RANGE CODE IN Vn_CONFIG!"));
	break; 
	// NOTE: high range is only used in CH1
	// high range
	//  Serial.println((2.25+(0.05 * int(mod_vn_thr))), DEC);
      }
    // NOTE: restoring the register settings is a bit touchy
    // if you try to change the range or threshold settings from what they were
    //  before the SAR algotithm you can cause a fault as the thresholds change
    smbus->writeWord(ltc2936_i2c_address, (LTC2936_V1_THR + ch_num-1), save_vn_thr); // restore Vn config
    smbus->writeWord(ltc2936_i2c_address, (LTC2936_V1_CONFIG + ch_num-1), save_vn_config); // restore Vn config
    
    return (result);
  }
  else {
    // invalid channel number
    //    Serial.println(F("ERROR: INVALID CHANNEL NUMBER!"));
    return (0.0);
  }
}







//! Load different voltage threshold settings into RAM
void ltc2936_demo_board_demo_thresholds(uint8_t ltc2936_i2c_address)
{
  if (ltc2936_is_write_protected(ltc2936_i2c_address) != 1)
  {
    smbus->writeWord(ltc2936_i2c_address, LTC2936_WRITE_PROTECT, 0xAAA8);
    smbus->writeWord(ltc2936_i2c_address, LTC2936_GPI_CONFIG, 0x1040); // GPI2=MARG, not mapped; GPI1=MR, clear history, not mapped
    smbus->writeWord(ltc2936_i2c_address, LTC2936_GPIO1_CONFIG, 0x002E);
    smbus->writeWord(ltc2936_i2c_address, LTC2936_GPIO2_3_CONFIG, 0x2E06);
    ////SINCE WE ARE FORCING OTHER CHANNELS DACs TO OUTPUT STRANGE VOLTAGES, TURN OFF FAULTS FOR THOSE CHANNELS
    //    smbus->writeWord(ltc2936_i2c_address, LTC2936_V1_THR, 0x412D); // ov = 5.5, uv = 4.5
    smbus->writeWord(ltc2936_i2c_address, LTC2936_V1_THR, 0xFE00); 
    //    smbus->writeWord(ltc2936_i2c_address, LTC2936_V2_THR, 0x554B); // ov = 2.6, uv = 2.4
    smbus->writeWord(ltc2936_i2c_address, LTC2936_V2_THR, 0xFE00); 
    //    smbus->writeWord(ltc2936_i2c_address, LTC2936_V3_THR, 0x917D); // ov = 1.9, uv = 1.7
    smbus->writeWord(ltc2936_i2c_address, LTC2936_V3_THR, 0xFE00); 
    //    smbus->writeWord(ltc2936_i2c_address, LTC2936_V4_THR, 0x735F); // ov = 1.6, uv = 1.4
    smbus->writeWord(ltc2936_i2c_address, LTC2936_V4_THR, 0xFE00); 
    //    smbus->writeWord(ltc2936_i2c_address, LTC2936_V5_THR, 0x5541); // ov = 1.3, uv = 1.1
    smbus->writeWord(ltc2936_i2c_address, LTC2936_V5_THR, 0xFE00); 
    //    smbus->writeWord(ltc2936_i2c_address, LTC2936_V6_THR, 0x3C32); // ov = 1.05, uv = 0.95
    smbus->writeWord(ltc2936_i2c_address, LTC2936_V6_THR, 0xFE00); 
    smbus->writeWord(ltc2936_i2c_address, LTC2936_V1_CONFIG, 0x009C); // high range
    //    smbus->writeWord(ltc2936_i2c_address, LTC2936_V2_CONFIG, 0x009C); // medium range
    smbus->writeWord(ltc2936_i2c_address, LTC2936_V2_CONFIG, 0x0180); // low range, no mappings
    smbus->writeWord(ltc2936_i2c_address, LTC2936_V3_CONFIG, 0x019C); // low range
    smbus->writeWord(ltc2936_i2c_address, LTC2936_V4_CONFIG, 0x019C); // low range
    smbus->writeWord(ltc2936_i2c_address, LTC2936_V5_CONFIG, 0x019C); // low range
    smbus->writeWord(ltc2936_i2c_address, LTC2936_V6_CONFIG, 0x019C); // low range
  }
  else
  {
    // error, LTC2936 is write-protected
    //    Serial.println(F("\nERROR: LTC2936 is write-protected. Cannot write to registers"));
  }
}

//! Clear ALERTB
void ltc2936_clear_alertb(uint8_t ltc2936_i2c_address)
{
  smbus->writeWord(ltc2936_i2c_address, LTC2936_CLEAR_HISTORY, 0x0000);
}


//! Return 1 if the LTC2936 is write-protected
//  0 otherwise
int ltc2936_is_write_protected(uint8_t ltc2936_i2c_address)
{
  uint16_t res;

  res = smbus->readWord(ltc2936_i2c_address, LTC2936_STATUS_WORD);

  //b[0] is the write-protect bit
  return ((res&0x0001) == 0x0001) ? 1 : 0;
}



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
  dc1605_write_dac_voltage(dac_address, ch_num, rand_voltage);


}

//! write a random DAC voltage to ch_num on the DC1633B demo board
//! random voltages will be around mean and have maximum excursions of +/-ampl 
void dc1605_dac_voltage_pulse(uint8_t dac_address, int ch_num, float mean, float pulse)
{
  // write the DAC to the pulse voltage, then quickly return it to the mean voltage
  dc1605_write_dac_voltage(dac_address, ch_num, (mean - pulse));
  dc1605_write_dac_voltage(dac_address, ch_num, mean);


}
