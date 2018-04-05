#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>

// These #defines make it easy to set the backlight color
#define OFF 0x0
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7 
#define ON 0x7 

static Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

void setup()
{
  Serial.begin(115200);
  Serial.println(F("BUTTONS AND LCD EXAMPLE"));
  lcd.begin(16,2);
  lcd.setBacklight(ON);
}

void loop()
{
  uint8_t buttons = lcd.readButtons();
    if (buttons) {
    Serial.print(F("BUTTON: "));
    Serial.println(buttons);
    lcd.clear();
    lcd.setCursor(0,0);
    if (buttons & BUTTON_UP) {
      lcd.print("UP ");
      Serial.println(F("BUTTON UP"));
      lcd.setBacklight(ON);
    }
    if (buttons & BUTTON_DOWN) {
      lcd.print("DOWN ");
      Serial.println(F("BUTTON DOWN"));
      lcd.setBacklight(OFF);
    }
    if (buttons & BUTTON_LEFT) {
      lcd.print("LEFT ");
      Serial.println(F("BUTTON LEFT"));
      lcd.setBacklight(ON);
    }
    if (buttons & BUTTON_RIGHT) {
      lcd.print("RIGHT ");
      Serial.println(F("BUTTON RIGHT"));
      lcd.setBacklight(OFF);
    }
  }
}

