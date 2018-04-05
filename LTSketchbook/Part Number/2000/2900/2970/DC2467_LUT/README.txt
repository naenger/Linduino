This sketch creates and uses a look-up table (LUT) with the DC2467
demo board. It drives the DAC, then measures output voltage, and
stores code/voltage pairs. It then can use these pairs to rapidly
drive the regulator output to a known voltage without using the servo
loop. This is good for modulating the regulator output, or simply
avoiding the very long servo loop settling time. Of course there are
limitations (like analog loop settling time, and slewing constraints),
but these must be comprehended by the user, since we are trying to go
faster than the LTC2970 servo loop.

Also, the Linduino has no way to store the LUT for later. WIth the
Arduino IDE software it cannot do file IO through the serial port, and
it has very small on-board memory in which it can store code/voltage
pairs. This limits us to a simple demonstration. It does not, however,
constrain a larger system, such as a beefier Linduino, larger
application processor, or Linux system.
