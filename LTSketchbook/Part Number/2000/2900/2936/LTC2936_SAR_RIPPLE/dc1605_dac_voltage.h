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

#ifndef DC1605_DAC
#define DC1605_DAC
#include <stdint.h>


// LTC2637 DAC on the DC1633B board
#define DC1633_DAC_ADDRESS 0x22 //pin-strapped address for DAC


//! program the DAC on the DC1633B demo board to a voltage
//  refer to the LTC2637 datasheet
void dc1605_write_dac_voltage(uint8_t dac_address, int channel, float voltage);



//! write a random DAC voltage to ch_num on the DC1633B demo board
//! random voltages will be around mean and have maximum excursions of +/-ampl 
void dc1605_random_dac_voltage(uint8_t dac_address, int ch_num, float mean, float ampl);

#endif
