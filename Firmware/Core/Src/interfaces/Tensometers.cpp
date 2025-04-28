/*
 * Tensometers.cpp
 *
 *  Created on: Mar 17, 2025
 *      Author: pwlub
 */

#include "interfaces/Tensometers.hpp"

int16_t Tensometers::get_tens_1()
{

	float tens_1_sum = 0;
	float alpha = 0.5;
	float last_sample = 0;
	for(unsigned int i = 0; i < number_of_analog_samples; i++)
		{
			last_sample = alpha * tensometer_1_val_raw[i] +(1-alpha) * last_sample;
			tens_1_sum += last_sample;
		}

	int64_t tens_1_avg = (int)tens_1_sum/(int)number_of_analog_samples;
	int16_t tens_1 = std::round(tens_1_avg);
	return tens_1;
}
