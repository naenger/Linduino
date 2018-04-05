/*************************************************** 
  This is a library for the MCP23017 i2c port expander

  These displays use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
//#include "LT_Wire.h"

#ifdef __AVR
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif
#include "Adafruit_MCP23017.h"
#ifdef __SAM3X8E__  // Arduino Due
 #define WIRE Wire1
#else
 #define WIRE Wire
//  #define WIRE LT_Wire
#endif

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

// minihelper
static inline void wiresend(uint8_t x) {
#if ARDUINO >= 100
  WIRE.write((uint8_t)x);
#else
  WIRE.send(x);
#endif
}

static inline uint8_t wirerecv(void) {
#if ARDUINO >= 100
  return WIRE.read();
#else
  return WIRE.receive();
#endif
}

////////////////////////////////////////////////////////////////////////////////

void Adafruit_MCP23017::begin(uint8_t addr) {
  if (addr > 7) {
    addr = 7;
  }
  i2caddr = addr;

  WIRE.begin();

  
  // set defaults!
  // modified to use LT_Wire library
  WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  //  WIRE.expectToWrite((uint16_t) 2);
  wiresend(MCP23017_IODIRA);
  //  WIRE.write(MCP23017_IODIRA);
  wiresend(0xFF);  // all inputs on port A
  //  WIRE.write(0xFF);
  WIRE.endTransmission();

  WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  //  WIRE.expectToWrite((uint16_t) 2);
  wiresend(MCP23017_IODIRB);
  //  WIRE.write(MCP23017_IODIRB);
  wiresend(0xFF);  // all inputs on port B
  //  WIRE.write(0xFF);
  WIRE.endTransmission();
}


void Adafruit_MCP23017::begin(void) {
  begin(0);
}

void Adafruit_MCP23017::pinMode(uint8_t p, uint8_t d) {
  uint8_t iodir;
  uint8_t iodiraddr;

  // only 16 bits!
  if (p > 15)
    return;

  if (p < 8)
    iodiraddr = MCP23017_IODIRA;
  else {
    iodiraddr = MCP23017_IODIRB;
    p -= 8;
  }

  // read the current IODIR
  WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  //  WIRE.expectToWrite((uint16_t) 1);
  wiresend(iodiraddr);	
  //  WIRE.write(iodiraddr);
  WIRE.endTransmission(false);
  
  WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  iodir = wirerecv();
  //  WIRE.requestFrom((MCP23017_ADDRESS | i2caddr), &iodir, 1);
  WIRE.endTransmission();
  
  // set the pin and direction
  if (d == INPUT) {
    iodir |= 1 << p; 
  } else {
    iodir &= ~(1 << p);
  }

  // write the new IODIR
  WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  //  WIRE.expectToWrite((uint16_t) 2);
  wiresend(iodiraddr);
  //  WIRE.write(iodiraddr);
  wiresend(iodir);	
  //  WIRE.write(iodir);
  WIRE.endTransmission();
  
}

uint16_t Adafruit_MCP23017::readGPIOAB() {
  uint16_t ba = 0;
  uint8_t a;

  // read the current GPIO output latches
  WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  //  WIRE.expectToWrite((uint16_t) 1);
  wiresend(MCP23017_GPIOA);	
  //  WIRE.write(MCP23017_GPIOA);
  WIRE.endTransmission(false);
  
  WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  //  uint8_t *tempHolder = new uint8_t[2];
  //  WIRE.requestFrom((MCP23017_ADDRESS | i2caddr), tempHolder, 2);
  WIRE.endTransmission();

  a = wirerecv();
  ba = wirerecv();
  ba <<= 8;
  ba |= a;
  //  ba = tempHolder[1] << 8;
  //  ba |= tempHolder[0];
  //  delete tempHolder;
  
  return ba;
}

void Adafruit_MCP23017::writeGPIOAB(uint16_t ba) {
  WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  //  LT_Wire.expectToWrite((uint16_t) 3);
  wiresend(MCP23017_GPIOA);	
  //  WIRE.write(MCP23017_GPIOA);
  wiresend(ba & 0xFF);
  //  WIRE.write(ba & 0xFF);
  wiresend(ba >> 8);
  //  WIRE.write(ba >> 8);
  WIRE.endTransmission();
}

void Adafruit_MCP23017::digitalWrite(uint8_t p, uint8_t d) {
  uint8_t gpio;
  uint8_t gpioaddr, olataddr;

  // only 16 bits!
  if (p > 15)
    return;

  if (p < 8) {
    olataddr = MCP23017_OLATA;
    gpioaddr = MCP23017_GPIOA;
  } else {
    olataddr = MCP23017_OLATB;
    gpioaddr = MCP23017_GPIOB;
    p -= 8;
  }

  // read the current GPIO output latches
  WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  //  LT_Wire.expectToWrite((uint16_t) 1);
  wiresend(olataddr);	
  //  WIRE.write(olataddr);
  WIRE.endTransmission(false);
  
  WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  //  WIRE.requestFrom((MCP23017_ADDRESS | i2caddr), &gpio, 1);
  gpio = wirerecv();
  WIRE.endTransmission();
  
  // set the pin and direction
  if (d == HIGH) {
    gpio |= 1 << p; 
  } else {
    gpio &= ~(1 << p);
  }

  // write the new GPIO
  WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  //  LT_Wire.expectToWrite((uint16_t) 2);
  wiresend(gpioaddr);
  //  WIRE.write(gpioaddr);
  wiresend(gpio);	
  //  WIRE.write(gpio);
  WIRE.endTransmission();
  
}

void Adafruit_MCP23017::pullUp(uint8_t p, uint8_t d) {
  uint8_t gppu;
  uint8_t gppuaddr;

  // only 16 bits!
  if (p > 15)
    return;

  if (p < 8)
    gppuaddr = MCP23017_GPPUA;
  else {
    gppuaddr = MCP23017_GPPUB;
    p -= 8;
  }


  // read the current pullup resistor set
  WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  //  LT_Wire.expectToWrite((uint16_t) 1);
  wiresend(gppuaddr);	
  //  WIRE.write(gppuaddr);
  WIRE.endTransmission(false);
  
  //  WIRE.requestFrom((MCP23017_ADDRESS | i2caddr), &gppu, 1);
  gppu = wirerecv();
  WIRE.endTransmission();

  // set the pin and direction
  if (d == HIGH) {
    gppu |= 1 << p; 
  } else {
    gppu &= ~(1 << p);
  }

  // write the new GPIO
  WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  //  LT_Wire.expectToWrite((uint16_t) 2);
  wiresend(gppuaddr);
  //  WIRE.write(gppuaddr);
  wiresend(gppu);	
  //  WIRE.write(gppu);
  WIRE.endTransmission();

}

uint8_t Adafruit_MCP23017::digitalRead(uint8_t p) {
  uint8_t gpioaddr;
  uint8_t foo;
  
  // only 16 bits!
  if (p > 15)
    return 0;

  if (p < 8)
    gpioaddr = MCP23017_GPIOA;
  else {
    gpioaddr = MCP23017_GPIOB;
    p -= 8;
  }

  // read the current GPIO
  WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  //  LT_Wire.expectToWrite((uint16_t) 2);
  wiresend(gpioaddr);	
  //  WIRE.write(gpioaddr);
  WIRE.endTransmission();
  
  //  WIRE.requestFrom((MCP23017_ADDRESS | i2caddr), &foo, 1);

  //  return (foo >> p) & 0x1;
  return (wirerecv() >> p) & 0x1;

}
