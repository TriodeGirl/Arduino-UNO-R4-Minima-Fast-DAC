# Arduino-UNO-R4-Minima-Fast-DAC
R4 Minima barebones direct-register code to use the DAC12 module in the RA4M1 processor.

Added DAC sine-wave example code using the RA4M1 hardware Floating Point Unit, which does speed things up!

Added test code for fast non-blocking ADC analog-read and DAC analog-write operation, with VERY-VERY-VERY fast sine calculation of frequency from ADC value.

Realtime sine calc with Paul Stoffregen's 11th order Taylor Series Approximation:
https://www.pjrc.com/high-precision-sine-wave-synthesis-using-taylor-series/
Note: I have condensed the Taylor Series code to a single set of inline asm calls

![Susan-Arduino-R4-DAC-loop()-with-ADC-Sine-calc-latancy-timing-1](https://github.com/TriodeGirl/Arduino-UNO-R4-Minima-Fast-DAC/assets/139503623/73a02f5d-e997-42cd-ab3c-00a2b63378ca)
