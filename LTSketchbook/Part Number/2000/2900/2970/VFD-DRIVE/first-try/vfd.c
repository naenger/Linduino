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
