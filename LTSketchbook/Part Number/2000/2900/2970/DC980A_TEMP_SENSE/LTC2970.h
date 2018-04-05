/*
Linear Technology DC980A/B Demonstration Board Control
LTC2970: Dual I2C Power Supply Monitor and Margining Controller

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

//! @defgroup LTC2970 LTC2970: Dual I2C Power Supply Monitor and Margining Controller

/*! @file
    @ingroup LTC2970
*/

#ifndef LTC2970_h
#define LTC2970_h

/*DEVICE I2C ADDRESSES (7-bit)*/
// GLOBAL ADDRESS
#define LTC2970_I2C_GLOBAL_ADDRESS 0x5B
#define LTC2970_I2C_ARA_ADDRESS 0x0C

/********************************************************************************/
//! LTC2970 command address definitions
//!  NOTE: commands prefixed with LTC2970_1_ are for the LTC2970-1 only

#define LTC2970_FAULT		0x00
//	0x01
//	...
//	0x07
#define LTC2970_FAULT_EN	0x08
//	0x09
#define LTC2970_FAULT_LA_INDEX	0x10
#define LTC2970_FAULT_LA	0x11
//	0x12
//	...
//	0x16
#define LTC2970_IO		0x17
#define LTC2970_ADC_MON		0x18
//	0x19
//	...
//	0x1E
#define LTC2970_1_SYNC		0x1F
//	0x20
//	...
//	0x27
#define LTC2970_VDD_ADC		0x28
#define LTC2970_VDD_OV		0x29
#define LTC2970_VDD_UV		0x2A
//	0x2B
//	...
//	0x37
#define LTC2970_V12_ADC		0x38
#define LTC2970_V12_OV		0x39
#define LTC2970_V12_UV		0x3A
//	0x3B
//	...
//	0x3F
#define LTC2970_CH0_A_ADC	0x40
#define LTC2970_CH0_A_OV	0x41
#define LTC2970_CH0_A_UV	0x42
#define LTC2970_CH0_A_SERVO	0x43
#define LTC2970_CH0_A_IDAC	0x44
#define LTC2970_1_CH0_A_IDAC_TRACK 0x45
#define LTC2970_1_CH0_A_DELAY_TRACK 0x46
//	0x47
#define LTC2970_CH0_B_ADC	0x48
#define LTC2970_CH0_B_OV	0x49
#define LTC2970_CH0_B_UV	0x4A
//	0x4B
//	...
//	0x4F
#define LTC2970_CH1_A_ADC	0x50
#define LTC2970_CH1_A_OV	0x51
#define LTC2970_CH1_A_UV	0x52
#define LTC2970_CH1_A_SERVO	0x53
#define LTC2970_CH1_A_IDAC	0x54
#define LTC2970_1_CH1_A_IDAC_TRACK 0x55
#define LTC2970_1_CH1_A_DELAY_TRACK 0x56
//	0x55
//	...
//	0x57
#define LTC2970_CH1_B_ADC	0x58
#define LTC2970_CH1_B_OV	0x59
#define LTC2970_CH1_B_UV	0x5A
//	0x5B
//	...
//	0x67
#define LTC2970_TEMP_ADC	0x68

#endif
