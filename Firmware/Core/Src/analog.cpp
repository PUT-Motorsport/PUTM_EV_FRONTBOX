/*
 * analog.cpp
 *
 *  Created on: May 24, 2024
 *      Author: ketirange
 */

#include "AnalogsAbstract.hpp"

int16_t Analog::get_steering_position()
{
	int16_t steering_wheel_position_sum = 0;
	for(unsigned int i = 0; i < number_of_analog_samples; i++)
	{
		steering_wheel_position_sum += steering_position_val_raw[i];
	}
	steering_position_val_avg = steering_wheel_position_sum/number_of_analog_samples;
	return steering_position_val_avg;
}


