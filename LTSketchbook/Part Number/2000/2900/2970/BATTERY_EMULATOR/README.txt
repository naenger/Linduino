This sketch uses the LTC2970 combined with an LT8714 (4-quadrant
switching regulator) to emulate a battery. It discharges like a
battery by slowly degrading its output voltage over a programmed
pattern. It charges like a battery by slowly increasing its output
volage in response to charge (input) current.

The LT8714 does the work of keeping the voltge and current in line.
The LTC2970 manages the "battery" voltage and current.
The Linduino maintains the "state" of the battery by integrating the
charge added to or subtracted from it, and adjusting the voltage servo
target appropriately.
