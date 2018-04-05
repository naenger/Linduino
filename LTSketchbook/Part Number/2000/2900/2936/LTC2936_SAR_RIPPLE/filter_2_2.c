/*! Filter a series of floating point numbers
  Assumes that there is only one instance, since it uses static variables to store state
  This is an obvious candidate for a c++ class.
 */

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

