Use the LTC2933 CH6 comparators and implement a SAR algorithm to find
the voltage on the pin to within 1 DAC code (4mv over a range of 0.2v
to 1.2v)

GOAL: take CH6 off-line as a supervisor - disable fault
generation. Use the UV and/or OV comparator and the Linduino code to
find the pin voltage using a SAR algorithm, then place the comparators
back on-line as a supervisor.

NOTE: The DC1633B board has a calibrated DAC that can generate precise
pin voltages. We use the DAC to demonstrate the SAR algorithm, but it
is not necessary in the final implementation.

NOTE: It is possible to keep the unused OV/UV comparator on-line,
participating in supervision, while the SAR search is ongoing. It is
also possible to use the other comparator for a different kind of
search, like glitch or noise detection.
