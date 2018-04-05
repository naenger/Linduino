/*! Filter a series of floating point numbers
  Assumes that there is only one instance, since it uses static variables to store state
*/

#ifndef BUTTERWORTH
#define BUTTERWORTH
using namespace std;

///
//! Filter a series of real numbers with an IIR function
//! Input is a real number
//! Output is the filtered output over all history
//! http://www.eas.uccs.edu/wickert/ece2610/lecture_notes/ece2610_chap8.pdf
class butterworth_2_2 {
  // Butterworth filter by virtue of the coefficient vectors
  // 
#define B_N 2 // feed-back network
#define B_M 2 // feed-forward network
  float state_x[B_M+1] = {0.0, 0.0, 0.0}; // feed-forward states
  float state_y[B_N+1] = {0.0, 0.0, 0.0}; // feed-back states
  float coeff_a[B_N+1] = {1.0, 1.7954767, -0.8145336 }; // feed-back coefficients; 0th coeff is unused
  float coeff_b[B_M+1] = {0.0047642, 0.0095284, 0.0047642}; // feed-forward coefficients

  float x_p = 0.0;  //output of the feed-forward filter
  float y_p = 0.0;
  
public:
  //! Constructor
  //! Initialize internal states to zero
  butterworth_2_2();

  //! Constructor
  //! Initialize internal states to a value
  butterworth_2_2(float initial_state);
    
  //! Update the filter state with a new input value
  //! Return the filter output state
  float increment(float x_0);
};

#endif
