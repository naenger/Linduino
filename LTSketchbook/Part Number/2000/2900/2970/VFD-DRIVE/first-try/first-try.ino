// first try at displaying text

#include <Arduino.h>
#include <stdint.h>
//#include "Linduino.h"
//#include "UserInterface.h"
//#include "LT_SMBusNoPec.h"


//#include <stdlib.h>
//#include <stdint.h>
//#include <stdio.h>
//#include <avr/io.h>
//#include <util/atomic.h>
//#include <util/delay.h>
//#include <string.h>
//#include <avr/eeprom.h>
//#include <avr/wdt.h>
//#include <avr/pgmspace.h>

//#include "serial.h"
//#include "pmbus_math.h"
//#include "avr-i2c-master.h"


#define LTC_RED     132
#define LTC_GREEN   0
#define LTC_BLUE    22

#define RESET_TIMEOUT     100

#define EEP_HIGH_ENERGY 0x10


/////////////////////////////////////////////////////////////////////////////////////

void setup() {
  //  Serial.begin(115200);         //! Initialize the serial port to the PC
    Serial.begin(38400);         //! Initialize the serial port to the PC
  int i;
	// Flush VFD Serial Buffer (in case reset mid command)
	for(i=0; i<32; i++)
	{
		// Send a bunch of something benign
		Serial.print(" ");
	}
	
 	// VFD Setup
	vfdNormalMode();

	printSplashScreen();

	vfdClearScreen();
	vfdZoom(2,2);

	vfdClearScreen();
	vfdSetCursor(78,1);
	Serial.print("Ready... ");
	delay(1000);
	Serial.print("Set... ");
	delay(1000);
	Serial.print("Go!");
	delay(1000);
	vfdClearScreen();

}

void loop() {
	vfdSetCursor(101,1);
	Serial.print("-");
	delay(1000);
	vfdSetCursor(101,1);
	Serial.print("/");
	delay(1000);
	vfdSetCursor(101,1);
	Serial.print("|");
	delay(1000);
	vfdSetCursor(101,1);
	Serial.print("\\");
	delay(1000);
}




///////////////////////////////////////////////////////////////////////////////////

void setLED(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
	OCR1A = 255 - r;
	OCR1B = 255 - g;
	OCR2A = 255 - b;
	OCR2B = 255 - w;
}

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

void printLogo(void)
{
	uint8_t i;

	uint8_t red   = LTC_RED;
	uint8_t green = LTC_GREEN;
	uint8_t blue  = LTC_BLUE;

	setLED(red, green, blue, 0);
	
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
		Serial.write(0); Serial.write(0); Serial.write(0);  // Blue, Green, Red
		i++;
	}

	// Draw Logo
	Serial.write(0x1F);
	Serial.write(0x28);
	Serial.write(0x66);
	Serial.write(0x11);

	Serial.write(65);  // X size
	Serial.write(0);

	Serial.write(4);   // Y size
	Serial.write(0);

	Serial.write(1);

	Serial.write(0x00); Serial.write(0x00); Serial.write(0x00); Serial.write(0x03);
	Serial.write(0x00); Serial.write(0x00); Serial.write(0x00); Serial.write(0x03);
	Serial.write(0x00); Serial.write(0x00); Serial.write(0x00); Serial.write(0x03);
	Serial.write(0x00); Serial.write(0x00); Serial.write(0x00); Serial.write(0x07);
	Serial.write(0x00); Serial.write(0x00); Serial.write(0x00); Serial.write(0x07);
	Serial.write(0x00); Serial.write(0x00); Serial.write(0x00); Serial.write(0x0F);
	Serial.write(0x00); Serial.write(0x00); Serial.write(0x00); Serial.write(0x1F);
	Serial.write(0x00); Serial.write(0x00); Serial.write(0x00); Serial.write(0x1F);

	Serial.write(0x00); Serial.write(0x00); Serial.write(0x00); Serial.write(0x7F);
	Serial.write(0x00); Serial.write(0x00); Serial.write(0x00); Serial.write(0xFF);
	Serial.write(0x00); Serial.write(0x00); Serial.write(0x03); Serial.write(0xFF);
	Serial.write(0x00); Serial.write(0x00); Serial.write(0x07); Serial.write(0xFF);
	Serial.write(0x00); Serial.write(0x00); Serial.write(0x1F); Serial.write(0xFF);
	Serial.write(0x00); Serial.write(0x00); Serial.write(0xFF); Serial.write(0xFF);
	Serial.write(0x00); Serial.write(0x07); Serial.write(0xFF); Serial.write(0xFF);
	Serial.write(0x00); Serial.write(0x1F); Serial.write(0xFF); Serial.write(0xFF);

	Serial.write(0x00); Serial.write(0x7F); Serial.write(0xFF); Serial.write(0xFF);
	Serial.write(0x01); Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFF);
	Serial.write(0x03); Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFF);
	Serial.write(0x07); Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFF);
	Serial.write(0x0F); Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFF);
	Serial.write(0x1F); Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFF);
	Serial.write(0x3F); Serial.write(0xFC); Serial.write(0x03); Serial.write(0xFF);
	Serial.write(0x7F); Serial.write(0xE0); Serial.write(0x01); Serial.write(0xFF);

	Serial.write(0x7F); Serial.write(0x00); Serial.write(0x00); Serial.write(0xFF);
	Serial.write(0xFE); Serial.write(0x00); Serial.write(0x00); Serial.write(0x7F);
	Serial.write(0xF8); Serial.write(0x00); Serial.write(0x00); Serial.write(0x7F);
	Serial.write(0xE0); Serial.write(0x00); Serial.write(0x00); Serial.write(0x3F);
	Serial.write(0xC0); Serial.write(0x00); Serial.write(0x00); Serial.write(0x3F);
	Serial.write(0x38); Serial.write(0x00); Serial.write(0x00); Serial.write(0x3F);
	Serial.write(0x78); Serial.write(0x00); Serial.write(0x00); Serial.write(0x1F);
	Serial.write(0xF8); Serial.write(0x00); Serial.write(0x00); Serial.write(0x1F);

	Serial.write(0xF8); Serial.write(0x00); Serial.write(0x00); Serial.write(0x1F);
	Serial.write(0xF8); Serial.write(0x00); Serial.write(0x00); Serial.write(0x1F);
	Serial.write(0xF8); Serial.write(0x00); Serial.write(0x00); Serial.write(0x1F);
	Serial.write(0xF8); Serial.write(0x00); Serial.write(0x00); Serial.write(0x1E);
	Serial.write(0xF8); Serial.write(0x00); Serial.write(0x00); Serial.write(0x1C);
	Serial.write(0xFC); Serial.write(0x00); Serial.write(0x00); Serial.write(0x03);
	Serial.write(0xFC); Serial.write(0x00); Serial.write(0x00); Serial.write(0x07);
	Serial.write(0xFC); Serial.write(0x00); Serial.write(0x00); Serial.write(0x1F);

	Serial.write(0xFE); Serial.write(0x00); Serial.write(0x00); Serial.write(0x7E);
	Serial.write(0xFF); Serial.write(0x00); Serial.write(0x01); Serial.write(0xFE);
	Serial.write(0xFF); Serial.write(0x00); Serial.write(0x0F); Serial.write(0xFC);
	Serial.write(0xFF); Serial.write(0xC0); Serial.write(0x3F); Serial.write(0xFC);
	Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xF8);
	Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xF0);
	Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xE0);
	Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xC0);

	Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFF); Serial.write(0x80);
	Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFE); Serial.write(0x00);
	Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xF8); Serial.write(0x00);
	Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xE0); Serial.write(0x00);
	Serial.write(0xFF); Serial.write(0xFF); Serial.write(0x00); Serial.write(0x00);
	Serial.write(0xFF); Serial.write(0xF8); Serial.write(0x00); Serial.write(0x00);
	Serial.write(0xFF); Serial.write(0xE0); Serial.write(0x00); Serial.write(0x00);
	Serial.write(0xFF); Serial.write(0xC0); Serial.write(0x00); Serial.write(0x00);

	Serial.write(0xFF); Serial.write(0x00); Serial.write(0x00); Serial.write(0x00);
	Serial.write(0xFE); Serial.write(0x00); Serial.write(0x00); Serial.write(0x00);
	Serial.write(0xFC); Serial.write(0x00); Serial.write(0x00); Serial.write(0x00);
	Serial.write(0xF8); Serial.write(0x00); Serial.write(0x00); Serial.write(0x00);
	Serial.write(0xF0); Serial.write(0x00); Serial.write(0x00); Serial.write(0x00);
	Serial.write(0xE0); Serial.write(0x00); Serial.write(0x00); Serial.write(0x00);
	Serial.write(0xE0); Serial.write(0x00); Serial.write(0x00); Serial.write(0x00);
	Serial.write(0xC0); Serial.write(0x00); Serial.write(0x00); Serial.write(0x00);

	Serial.write(0xC0); Serial.write(0x00); Serial.write(0x00); Serial.write(0x00);
}


void printSplashScreen(void)
{
	uint8_t i;

	vfdClearScreen();
	vfdZoom(2,2);

	//  Full Brightness
	Serial.write(0x1F);
	Serial.write(0x58);
	Serial.write(0x18);
	
	printLogo();
	
	vfdSetCursor(70,0);
	delay(300);
	Serial.write('L');
	delay(300);
	Serial.write('I');
	delay(300);
	Serial.write('N');
	delay(300);
	Serial.write('E');
	delay(300);
	Serial.write('A');
	delay(300);
	Serial.write('R');

	vfdSetCursor(70,2);
	delay(300);
	Serial.write('T');
	delay(300);
	Serial.write('E');
	delay(300);
	Serial.write('C');
	delay(300);
	Serial.write('H');
	delay(300);
	Serial.write('N');
	delay(300);
	Serial.write('O');
	delay(300);
	Serial.write('L');
	delay(300);
	Serial.write('O');
	delay(300);
	Serial.write('G');
	delay(300);
	Serial.write('Y');
	delay(300);

	delay(2000);
	//	while(!(PIND & _BV(PD7)))
	//	{
	//		if(resetCount > RESET_TIMEOUT)
	//		{
	//			vfdClearScreen();
	//			vfdZoom(1,1);
	//			Serial.println("Resetting Statistics...");
	//			eeprom_write_float((float *)EEP_HIGH_ENERGY, 0);
	//			for(i=0; i<sizeof(histogram)/sizeof(histogram[0]); i++)
	//			{
	//				histogram[i] = 0;
	//			}
	//			eeprom_write_block((void *)histogram, (void *)EEP_STATS, sizeof(histogram));
	//			delay(2000);
	//			cli();
	//			wdt_reset();
	//			MCUSR &= ~(_BV(WDRF));
	//			WDTCSR |= _BV(WDE) | _BV(WDCE); 
	//			WDTCSR = _BV(WDE);
	//			while(1);
	//		}
	//	}
	
	for(i=0x18; i>=0x10; i--)
	{
		Serial.write(0x1F);
		Serial.write(0x58);
		Serial.write(i);
		delay(25);
	}

	delay(1000);

	// Clear Backlight
	Serial.write(0x1F);
	Serial.write(0x4C);
	Serial.write(0x80);
	Serial.write(0); Serial.write(0); Serial.write(0);  // Blue, Green, Red

	vfdClearScreen();

	//  Restore Full Brightness
	Serial.write(0x1F);
	Serial.write(0x58);
	Serial.write(0x18);
}
