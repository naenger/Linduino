//a bunch of functions


////////////////////////////////////////////////////////////////////////////////////
//! Prints a warning if the demo board is not detected.
void print_warning_prompt()
{
  Serial.println(F("\nWarning: Demo board not detected. Linduino will attempt to proceed."));
}


////////////////////////////////////////////////////////////////////////////////////
//! Set boost voltage to maximum
void set_dac_code_48v()
{
  // use the global DAC variables
  dac_value = 0x00FF;
  //dac_ctrl = 0x0000;
  idac_reg = dac_ctrl + dac_value;

  // make sure servoing is disabled
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_SERVO,  0x0BDF);

  // set DAC code
  smbus->writeWord(ltc2970_i2c_address, LTC2970_CH1_A_IDAC, idac_reg);
}

////////////////////////////////////////////////////////////////////////////////////

void controlled_step()
{

}


int init_voltage_transition ()
{
  v_trans.dac_code_start = 0x49; // the code for 12v
  v_trans.step_unit = 1;
  v_trans.step_direction = 1;
  v_trans.num_steps = 1;
  v_trans.step_time = 1;
}

int get_user_step_values()
{
  // query the user for step parameters
  // store in a global struct

  uint8_t user_input;
  float user_input_f;
  float max_voltage, min_voltage, start_voltage, end_voltage;

  max_voltage = dac_v0 + dac_step_size*((float)dac_max_code - (float)dac_code0);
  min_voltage = dac_v0 + dac_step_size*((float)dac_min_code - (float)dac_code0);

  Serial.print(F("\n****VOLTAGE TRANSITION PARAMETERS****\n"));
  Serial.print(F("\n ENTER INITIAL BOOST VOLTAGE (12v <= V <= 24v): \n"));
  user_input_f = read_float();
  Serial.print(F("you entered: "));
  Serial.println(user_input_f);
  Serial.print(F("max_voltage: "));
  Serial.println(max_voltage);
  Serial.print(F("min_voltage: "));
  Serial.println(min_voltage);

  //process the floating-point number into a DAC code
  // the translation from DAC code to voltage has both gain and offset terms
  if ((user_input_f < max_voltage) &&
      (user_input_f > min_voltage))
  {
    //      dac_value = (uint16_t)((user_input_f + 2.1) / dac_step_size);
    dac_value = dac_code0 + ((user_input_f - dac_v0) / dac_step_size);
    start_voltage = dac_v0 + dac_step_size*((float)dac_value - (float)dac_code0);
    v_trans.dac_code_start = dac_value;
    Serial.print(F("calculated DAC value: "));
    Serial.println(v_trans.dac_code_start);
    Serial.print(F("calculated voltage at that DAC value: "));
    Serial.println(start_voltage);
  }
  else
  {
    Serial.print(F("\n****ERROR, voltage out of range\n"));
    Serial.print(F("UPPER LIMIT = "));
    Serial.println(max_voltage);
    Serial.print(F("LOWER LIMIT = "));
    Serial.println(min_voltage);
    Serial.print(F("NOTE THAT THE LOWER VOLTAGE IS ALSO LIMITED BY BOOST CONVERTER INPUT"));
    return 1;
  }

  Serial.print(F("\nTHE DAC WILL TAKE VOLTAGE STEPS OF SIZE = S*190mv: \n"));
  Serial.print(F("ENTER INTEGER S: \n"));
  Serial.print(F("POSITIVE FOR STEP-UP and NEGATIVE FOR STEP-DOWN.\n"));
  user_input_f = read_float();
  v_trans.step_direction = (user_input_f >= 0) ? 1 : 0;
  Serial.print(F("you entered: "));
  Serial.println(user_input_f);
  if (v_trans.step_direction == 1)
  {
    //positive voltage steps
    v_trans.step_unit = (uint8_t)user_input_f;
    Serial.print(F("voltage step size: "));
    Serial.println((v_trans.step_unit*dac_step_size));
  }
  else
  {
    //negative voltage steps
    v_trans.step_unit = (uint8_t)(-1*user_input_f);
    Serial.print(F("voltage step size: "));
    Serial.println((-1*v_trans.step_unit*dac_step_size));
  }

  Serial.print(F("\nTHE DAC WILL TAKE AN INTEGER NUMBER OF STEPS N \n"));
  Serial.print(F("ENTER A POSITIVE INTEGER NUMBER OF STEPS, N: \n"));
  user_input_f = read_float();
  Serial.print(F("you entered: "));
  Serial.println(user_input);
  if (user_input_f > 0)
  {
    v_trans.num_steps = (uint8_t)user_input_f;
    Serial.print(F("total voltage excursion (v) = "));
    Serial.println((v_trans.num_steps*v_trans.step_unit*dac_step_size));
  }
  else
  {
    Serial.print(F("\n****ERROR, number out of range\n"));
    return 1;
  }

  end_voltage = (v_trans.step_direction == 1) ?
                start_voltage + (dac_step_size*v_trans.step_unit*v_trans.num_steps) :
                start_voltage - (dac_step_size*v_trans.step_unit*v_trans.num_steps);

  if (end_voltage < min_voltage)
  {
    Serial.print(F("\n****ERROR, ending voltage is below minimum limit\n"));
    Serial.print(F("end voltage: "));
    Serial.println(end_voltage);
    return 1;
  }
  else if (end_voltage > max_voltage)
  {
    Serial.print(F("\n****ERROR, ending voltage is above maximum limit\n"));
    Serial.print(F("end voltage: "));
    Serial.println(end_voltage);
    return 1;
  }

  Serial.print(F("\nTHE DAC WILL WAIT BETWEEN STEPS (WAIT = D*1.0ms): \n"));
  Serial.print(F("ENTER AN INTEGER D: \n"));
  user_input = read_int();
  Serial.print(F("you entered: "));
  Serial.println(user_input);
  if (user_input > 0)
  {
    v_trans.step_time = user_input;
    Serial.print(F("time between steps (ms) = "));
    Serial.println((v_trans.step_time*1.0));
  }
  else
  {
    Serial.print(F("\n****ERROR, number out of range\n"));
    return 1;
  }

  //print summary information
  Serial.print(F("\n\nVOLTAGE EXCURSION INFORMATION:\n"));
  Serial.print(F("  INITIAL VOLTAGE (v) = "));
  //  Serial.println((v_trans.dac_code_start*0.17));
  Serial.println(start_voltage);
  Serial.print(F("  FINAL VOLTAGE (v) = "));
  Serial.println(end_voltage);
  //     (dac_step_size*(v_trans.dac_code_start+(v_trans.num_steps*v_trans.step_unit))-dac_offset) :
  //     (dac_step_size*(v_trans.dac_code_start-(v_trans.num_steps*v_trans.step_unit))-dac_offset));
  Serial.print(F("  TOTAL TRANSITION TIME (ms) = "));
  Serial.println(v_trans.num_steps*v_trans.step_time);



  Serial.print(F("ACCEPT THESE PARAMETERS (y/n)?\n"));
  user_input = read_char();
  if ((user_input == 'y') || (user_input == 'Y'))
  {
    Serial.print(F("ACCEPTED\n"));
    return 0;
  }
  else
  {
    Serial.print(F("NOT ACCEPTED\n"));
    return 1;
  }
}
