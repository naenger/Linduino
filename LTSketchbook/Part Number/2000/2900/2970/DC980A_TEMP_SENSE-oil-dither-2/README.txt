The other (older) temperature experiment takes the averages of the
various ADC measurements and performs one temperature calculation
using those averages. Dithering tries to scramble the noise so it can
be averaged out of the measurement, but the question remains if it is
valid to take the average of a dithered measuremnt on a non-linear
element (Vbe).

This sketch calculates temperature once for each set of samples, and
then averages the temperature measurements at the end. The goal is to
determine if this is better for measurement accuracy and noise.
