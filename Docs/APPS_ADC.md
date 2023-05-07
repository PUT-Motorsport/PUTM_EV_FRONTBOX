# Determining the sample time and oversampling for ADC 2

ADCs are clocked from AHB2, which has the clock speed of 168 MHz. The ADC has a prescaler of 4, which gives 42 MHz.

There are 5 channels, each of which is oversampled by 256 with sample time of 640.5 cycles.
Full measurement lasts for:
$$
5 * 256 * 640.5 =  794.220 cycles
$$

And with the frequency of 100 Hz, the sampling can take at most
$$
42 cycles / 100 = 480.000 cycles
$$