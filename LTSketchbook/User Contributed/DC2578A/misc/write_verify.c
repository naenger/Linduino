//! Write a value to a rergister
//! Verify the write with a mask
int write_verify_mask(uint16_t addr, uint16_t cmd, uint16_t value, uint16_t mask)
{
  uint16_t read_val;
  int i = 0;

  smbus->writeWord(addr, cmd, value);
  read_val = smbus->readWord(addr, cmd);
  if(((read_val & mask) != value)) {
    Serial.println(F("\n*!*!*FAIL!*!*!"));
    Serial.println(i);
    Serial.println(read_val, HEX);
    return -1;
  }
  else
    return 0;
}
