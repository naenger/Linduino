This sketch talks to an LTC2975 down-stream of an isolated forward
converter (LTC3765/66) and searches for the optimum forward converter
output voltage (CH0) to minimize power loss in the other channels (CH1
- CH3). This assumes a kind-of funny hook-up for the LTC2975 because
it is servoing its own input power supply voltage (over a limited
range). It does not have RUN control, though, so it can't accidentally
shut-down its own supply. 
