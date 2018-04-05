This directory contains a sketch for collecting temperature data from
an appropriately-configured LTC2970 board in an oil bath (8 2970s
together). The code averages samples from the ADC, but does no
dithering, so the quantization noise is large (several degrees).
