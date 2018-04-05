NOTE: The original piece of hardware was a hand-soldered resistor and
bjt on a board with twisted wires going to a connector to the DC980A
board. This arrangement used the following connections:

CH0_IDAC : drive the top of the resistor
CH0_VDAC : not used
CH0_A : measure current (v across the resistor)
CH0_B : measure bjt vbe
CH1_IDAC : not used
CH1_VDAC : not used
CH1_A : not used
CH1_B : not used

The oil-bath board, produced by Michael Petersen is different. It has
8 copies of the same circuit. It has the same series-connected
resistor and bjt, but they are tied to different pins on the
LTC2970. It also doesn't use all of the wires and connectors. Here are
the connections:

CH0_IDAC : not used
CH0_VDAC : not used
CH0_A : not used
CH0_B : not used
CH1_IDAC : drive the top of the resistor
CH1_VDAC : not used
CH1_A : measure bjt vbe
CH1_B : measure current (v across the resistor)

