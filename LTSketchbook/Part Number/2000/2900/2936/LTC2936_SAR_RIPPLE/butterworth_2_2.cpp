/*! Filter a series of floating point numbers
  Assumes that there is only one instance, since it uses static variables to store state
*/

using namespace std;
#include "butterworth_2_2.h"


//! Constructor
//! Initialize internal states to zero
butterworth_2_2::butterworth_2_2()
{
  int i;
  for(i = 0; i <= B_N; i++) {
    state_x[i] = 0.0;
  }
  for(i = 0; i <= B_M; i++) {
    state_y[i] = 0.0;
  }
}

//! Constructor
//! Initialize internal states to a value
butterworth_2_2::butterworth_2_2(float initial_state)
{
  int i;
  for(i = 0; i <= B_N; i++) {
    state_x[i] = initial_state;
  }
  for(i = 0; i <= B_M; i++) {
    state_y[i] = initial_state;
  }
}


//! Update the filter state with a new input value
//! Return the filter output state
float butterworth_2_2::increment(float x_0)
{
  int i;

  x_p = 0.0;
  y_p = 0.0;

  for (i = B_M; i >= 0; i--) {
    //calculate the feed-forward filter
    state_x[i] = (i == 0) ? x_0 : state_x[i-1]; // shift the value in the delay chain
    x_p += (state_x[i] * coeff_b[i]);
  }
  //  Serial.print(F("x = "));
  //  Serial.println(x_p);
  
  for (i = B_N; i > 0; i--) {
    //calculate the feed-back filter
    state_y[i] = state_y[i-1]; // shift the value in the delay chain
    y_p += (state_y[i] * coeff_a[i]);
  }
  //  Serial.print(F("y = "));
  //  Serial.println(x_p);
  
  state_y[0] = y_p + x_p; // state_y[0] is the output of the filter
  return (state_y[0]);
}
