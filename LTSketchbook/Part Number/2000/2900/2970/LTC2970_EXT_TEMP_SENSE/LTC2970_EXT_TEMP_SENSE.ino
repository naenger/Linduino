/*
LTC2970: Dual I2C Power Supply Monitor and Margining Controller

Oil Bath Temperature Sense Board 8x LTC2970s with External BJT
This sketch reports temperature at each BJT
Requires that the IDAC is not terminated with a resistor to GND
Requires that the BJT be installed with force and sense lines separate

Drives Petersen's VFD display to show temperature numerically in real-time

@verbatim

NOTES
  Setup:
   Set the terminal baud rate to 115200 and select the newline terminator.

@endverbatim

http://www.linear.com/product/LTC2970
http://www.linear.com/demo/DC980A

REVISION HISTORY
$Revision:  $
$Date:  $

Copyright (c) 2014, Linear Technology Corp.(LTC)
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

/*! @file
    @ingroup LTC2970
*/

#include <math.h>
#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "UserInterface.h"
#include "LT_SMBusNoPec.h"
#include "LTC2970.h"



/****************************************************************************/
// Global variables
uint8_t ltc2970_i2c_address[8];

//VFD PANEL STUFF
#define VFD_COLUMN 160
#define VFD_MAX_COL 312
#define VFD_MIN_COL 1
#define LTC_RED     132
#define LTC_GREEN   0
#define LTC_BLUE    22

// LTC2637 DAC on the DC1605B board
#define DC1605_DAC_ADDRESS 0x22 //pin-strapped address for DAC

static uint8_t dc1605_dac_address;

static LT_SMBusNoPec *smbus = new LT_SMBusNoPec();


// Values for the external temperature sensor routine
static uint16_t dac_current_low = 0x0714;
static uint16_t dac_current_high = 0x07FF;

//float bjt_n[8]; // = 1.016;

int meas_delay = 350; //number of milliseconds to wait for voltage settling and ADC sampling between set and measure

//! Circular buffers for running averages
//const int count = 40; // number of samples to average over
const int count = 1; // number of samples to average over
const int num_dev = 1; // number of LTC2970 devices on the bus
//const int dither[8] = {0,1,-2,1,0,-1,2,-1};
//const int dither[10] = {0,1,-2,1,-1,0,-1,2,-1,1};
const int dither[40] = {0,1,-2,1,-1,0,-1,2,-1,1,0,1,-2,1,-1,0,-1,2,-1,1,0,1,-2,1,-1,0,-1,2,-1,1,0,1,-2,1,-1,0,-1,2,-1,1};
//const int dither[40] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
// multi-dimentional arrays are [device][count]
float current_high[num_dev][count];
float current_low[num_dev][count];
float current_high_accum[num_dev];
float current_low_accum[num_dev];
float vbe_high[num_dev][count];
float vbe_low[num_dev][count];
float vbe_high_accum[num_dev];
float vbe_low_accum[num_dev];
int ptr, ptr2;


const float q_by_k = 11.60452e3;

float average = 0.0;

/****************************************************************************/
//! Initialize Linduino
void setup()
{
  uint16_t return_val;
  int i,j;
  
  //  Serial.begin(115200);         //! Initialize the serial port to the PC
    Serial.begin(38400);         //! Initialize the serial port to the VFD screen

  // define individual chip addresses
  ltc2970_i2c_address[0] = 0x5C;
  ltc2970_i2c_address[1] = 0x5D;
  ltc2970_i2c_address[2] = 0x5E;
  ltc2970_i2c_address[3] = 0x5F;
  ltc2970_i2c_address[4] = 0x6B;
  ltc2970_i2c_address[5] = 0x6C;
  ltc2970_i2c_address[6] = 0x6D;
  ltc2970_i2c_address[7] = 0x6E;



  for(i = 0; i < 8; i++) {
    // just in case we want to talk to a different device in the array, instead of  device A
    ltc2970_configure(ltc2970_i2c_address[i]);
    //! Initialize the DAC
    smbus->writeWord(ltc2970_i2c_address[i], LTC2970_CH1_A_IDAC, 0x0700);
  }
  
    // initialize the individual device arrays
  for(i = 0; i < num_dev; i++) {
    //    ltc2970_configure(ltc2970_i2c_address[i]);
    //! Initialize the DAC
    //smbus->writeWord(ltc2970_i2c_address[i], LTC2970_CH1_A_IDAC, 0x0700);
    //    bjt_n[i] = 1.016;
    vbe_low_accum[i] = 0.0;
    vbe_high_accum[i] = 0.0;
    current_low_accum[i] = 0.0;
    current_high_accum[i] = 0.0;

    // initialize the circular buffers and accumulators
    for(j = 0; j < count; j++) {
    current_high[i][j] = 0.0;
    current_low[i][j] = 0.0;
    vbe_high[i][j] = 0.0;
    vbe_low[i][j] = 0.0;
    }
  }

  ptr = 0;
  ptr2 = 0;


  // INITIALIZE THE VFD PANEL
  vfdNormalMode();
  vfdClearScreen();
  vfdZoom(2,2);
  
  vfdClearScreen();
  vfdSetCursor(VFD_COLUMN,1);

  printLogo();
  
  delay(meas_delay);
}


//! This loop takes no user input
//! It simply maintains a running average of measured Vbe and I values, and prints-out a new
//!   temperature reading at every measurement
//! Expect the first N readings to be noisy while the running averages are accumulating
void loop()
{
  uint16_t reg_val;

  int device; // count the number of LTC2970s on the bus
  int dev;
  int pos_text; // X position of VFD text on the screen
  int pos_backlight; // X position of VFD colors on the screen

  float p, x;
  float sens = 0.06;
  float q_by_n_k;
  
  float vbe_delta, current_ratio, ln_current_ratio, ext_temp;
  
  for (device = 0; device < num_dev; device++) {
    dev = device+1;
    //    q_by_n_k = (q_by_k / bjt_n[device]);
    q_by_n_k = q_by_k / 1.016;
    //! Measure voltage and current at low current
    smbus->writeWord(ltc2970_i2c_address[dev], LTC2970_CH1_A_IDAC, (dac_current_low+dither[ptr2]));
    delay(meas_delay);
    reg_val =
      (0x7FFF & LTC2970_readNewWord(ltc2970_i2c_address[dev], LTC2970_CH1_A_ADC));
    vbe_low_accum[device] -= vbe_low[device][ptr];
    vbe_low[device][ptr] = ((float)(reg_val & 0x7FFF))*500e-6;
    vbe_low_accum[device] += vbe_low[device][ptr];

    reg_val =
      (0x7FFF & LTC2970_readNewWord(ltc2970_i2c_address[dev], LTC2970_CH1_B_ADC));
    current_low_accum[device] -= current_low[device][ptr];
    current_low[device][ptr] = ((float)(reg_val & 0x7FFF))*500e-6;
    current_low_accum[device] += current_low[device][ptr];

    //! Measure voltage and current at high current
    smbus->writeWord(ltc2970_i2c_address[dev], LTC2970_CH1_A_IDAC, (dac_current_high+dither[ptr]));
    delay(meas_delay);
    reg_val =
      (0x7FFF & LTC2970_readNewWord(ltc2970_i2c_address[dev], LTC2970_CH1_A_ADC));
    vbe_high_accum[device] -= vbe_high[device][ptr];
    vbe_high[device][ptr] = ((float)(reg_val & 0x7FFF))*500e-6;
    vbe_high_accum[device] += vbe_high[device][ptr];

    reg_val =
      (0x7FFF & LTC2970_readNewWord(ltc2970_i2c_address[dev], LTC2970_CH1_B_ADC));
    current_high_accum[device] -= current_high[device][ptr];
    current_high[device][ptr] = ((float)(reg_val & 0x7FFF))*500e-6;
    current_high_accum[device] += current_high[device][ptr];

    vbe_delta = (vbe_high_accum[device] - vbe_low_accum[device]) / count;
    current_ratio = current_high_accum[device] / current_low_accum[device];
    ln_current_ratio = log(current_ratio); // natural log

    ext_temp = (vbe_delta * q_by_n_k / ln_current_ratio) - 273.15;
    average = filter_2_2(ext_temp);
  }

  // PRINT TO THE VFD PANEL
  x = sens*(average - 25.0);
  p = 1/(1+exp(-1*x));
  pos_text = (int)(VFD_MIN_COL + (VFD_MAX_COL - VFD_MIN_COL)*p);
  pos_backlight = (int)(0 + (32 - 0)*p);

  vfdClearScreen();
  vfdSetCursor(pos_text,1);
  Serial.print(average, 1);  
  Serial.print("C");  

  setBacklightBar(pos_backlight);

  ///////////////

  //! update the circular buffer pointer
  ++ptr;
  ++ptr2;
  ptr = (ptr >= count) ? 0 : ptr;
  ptr2 = (ptr2 >= 40) ? 0 : ptr2;

}


/************************************************************************/
// Function Definitions

//! Prints the title block when program first starts.
void print_title()
{
  Serial.print(F("\n***************************************************************\n"));
  Serial.print(F("* LTC2970 Temperature Measurement Program                       *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("* This program provides a simple interface to measure           *\n"));
  Serial.print(F("* temperature using a BJT connected to the LTC2970              *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("* Set the baud rate to 115200 and select the newline terminator.*\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("*****************************************************************\n"));
  Serial.println(F("\n"));
}


//! Writes configuration values to the LTC2970 registers
void ltc2970_configure(uint8_t local_i2c_address)
{
  uint16_t return_val;
  //start the 2970 by configuring all of its registers for this application
  // use SMbus commands
  smbus->writeWord(local_i2c_address, LTC2970_FAULT_EN, 0x0168);
  smbus->writeWord(local_i2c_address, LTC2970_IO, 0x000A);
  smbus->writeWord(local_i2c_address, LTC2970_ADC_MON, 0x007F);
  smbus->writeWord(local_i2c_address, LTC2970_VDD_OV, 0x2CEC);
  smbus->writeWord(local_i2c_address, LTC2970_VDD_UV, 0x2328);
  smbus->writeWord(local_i2c_address, LTC2970_V12_OV, 0x3FFF);
  smbus->writeWord(local_i2c_address, LTC2970_V12_UV, 0x00000);

  smbus->writeWord(local_i2c_address, LTC2970_CH1_A_OV, 0x2AF8);
  smbus->writeWord(local_i2c_address, LTC2970_CH1_A_UV, 0x2328);
  smbus->writeWord(local_i2c_address, LTC2970_CH1_A_SERVO, 0x0000);
  smbus->writeWord(local_i2c_address, LTC2970_CH1_A_IDAC, 0x0700);

  smbus->writeWord(local_i2c_address, LTC2970_CH1_B_OV, 0x3FFF);
  smbus->writeWord(local_i2c_address, LTC2970_CH1_B_UV, 0x0000);

  smbus->writeWord(local_i2c_address, LTC2970_CH0_A_OV, 0x1C5D);
  smbus->writeWord(local_i2c_address, LTC2970_CH0_A_UV, 0x1770);
  smbus->writeWord(local_i2c_address, LTC2970_CH0_A_SERVO, 0x0000);
  smbus->writeWord(local_i2c_address, LTC2970_CH0_A_IDAC, 0x0084);
  smbus->writeWord(local_i2c_address, LTC2970_CH0_B_OV, 0x3FFF);
  smbus->writeWord(local_i2c_address, LTC2970_CH0_B_UV, 0x0000);
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


///
//! Read one of the LTC2970 ADC registers and only return when the value is new, or timeout is reached
//!  New values are indicated by a 1 in bit 15 of the register
//!  Return the register value including bit 15 on success
//!  Return 0x0000 on failure (no new information)
//!  Perform some boundary checking to ensure that the register is an ADC reg.

uint16_t LTC2970_readNewWord(uint8_t ltc2970_i2c_address, uint8_t ltc2970_adc_reg)
{
  const int timeout = 1000; // give up after this many read attempts
  int i = timeout;
  int done = 0;
  uint16_t return_val = 0x0000;

  if((ltc2970_adc_reg == LTC2970_VDD_ADC) ||
     (ltc2970_adc_reg == LTC2970_V12_ADC) ||
     (ltc2970_adc_reg == LTC2970_CH0_A_ADC) ||
     (ltc2970_adc_reg == LTC2970_CH0_B_ADC) ||
     (ltc2970_adc_reg == LTC2970_CH1_A_ADC) ||
     (ltc2970_adc_reg == LTC2970_CH1_B_ADC) ||
     (ltc2970_adc_reg == LTC2970_TEMP_ADC)     ) {
    do {
      i--;
      return_val = smbus->readWord(ltc2970_i2c_address, ltc2970_adc_reg);
      if((return_val&0x8000) == 0x8000) {
	done = 1;
      }
      else if (i <= 0) {
	done = 1;
	return_val = 0x7FFF&return_val;
      }
    } while (!done);
  }
  return (return_val);
}


///////////////////////////////////////////////////////////////////
// VFD functions

void vfdClearScreen(void)
{
  	Serial.write(0x0C);
}

void vfdHome(void)
{
	Serial.write(0x0B);
}

void vfdNormalMode(void)
{
	Serial.write(0x1F);
	Serial.write(0x72);
	Serial.write(0x00);

}

void vfdZoom(uint8_t x, uint8_t y)
{
	Serial.write(0x1F);
	Serial.write(0x28);
	Serial.write(0x67);
	Serial.write(0x40);
	Serial.write(x);
	Serial.write(y);
	Serial.write(0x00);
}

void vfdSetCursor(uint16_t x, uint16_t y)
{
	Serial.write(0x1F);
	Serial.write(0x24);
	Serial.write((x & 0xFF));
	Serial.write((char)(x >> 8));
	Serial.write((y & 0xFF));
	Serial.write((y >> 8));
}


void setBacklightBar(int stop)
{
	uint8_t i;

	uint8_t red   = LTC_RED;
	uint8_t green = LTC_GREEN;
	uint8_t blue  = LTC_BLUE;

	// Set Backlight
	Serial.write(0x1F);
	Serial.write(0x4C);
	Serial.write(0x90);
	Serial.write(32);  // Number of specified areas

	for(i=0; i<(stop-1); i++)
	{
		Serial.write(i);  // Area number
		Serial.write(blue); Serial.write(green); Serial.write(red);  // Blue, Green, Red
	}
	uint8_t red_step = red / (stop - i);
	
	while(i < stop)
	{
		red -= red_step;
		if(red < 0x20)
			blue = 0x10;
		if(red < 0x10)
			blue = 0x00;
		Serial.write(i);  // Area number
		Serial.write(blue); Serial.write(green); Serial.write(red);  // Blue, Green, Red
		i++;
	}
	while(i < 32)
	{
		Serial.write(i);  // Area number
		Serial.write(127); Serial.write(0); Serial.write(0);  // Blue, Green, Red
		i++;
	}
}

void printLogo(void)
{
	uint8_t i;

	uint8_t red   = LTC_RED;
	uint8_t green = LTC_GREEN;
	uint8_t blue  = LTC_BLUE;

	//	setLED(red, green, blue, 0);
	
	uint8_t stop = 28;
	
	// Set Backlight
	Serial.write(0x1F);
	Serial.write(0x4C);
	Serial.write(0x90);
	Serial.write(32);  // Number of specified areas

	for(i=0; i<12; i++)
	{
		Serial.write(i);  // Area number
		Serial.write(blue); Serial.write(green); Serial.write(red);  // Blue, Green, Red
	}
	
	uint8_t red_step = red / (stop - i);
	
		while(i < stop)
		  {
		    red -= red_step;
		    if(red < 0x20)
		      blue = 0x10;
		    if(red < 0x10)
		      blue = 0x00;
		    Serial.write(i);  // Area number
		    Serial.write(blue); Serial.write(green); Serial.write(red);  // Blue, Green, Red
		    i++;
		  }

	while(i < 32)
	{
		Serial.write(i);  // Area number
		Serial.write(127); Serial.write(0); Serial.write(0);  // Blue, Green, Red
		i++;
	}
}


void setLED(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
	OCR1A = 255 - r;
	OCR1B = 255 - g;
	OCR2A = 255 - b;
	OCR2B = 255 - w;
}

