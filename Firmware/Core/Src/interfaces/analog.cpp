/*
 * analog.cpp
 *
 *  Created on: May 24, 2024
 *      Author: ketirange
 */

#include "interfaces/AnalogsAbstract.hpp"

uint16_t Analog::get_steering_position()
{
	float steering_wheel_position_sum = 0;
	float alpha = 0.5;
	float last_sample = 0;
	for(unsigned int i = 0; i < number_of_analog_samples; i++)
	{
		last_sample = alpha * steering_position_val_raw[i] +(1-alpha) * last_sample;
		steering_wheel_position_sum += current_sample;
	}

	uint64_t steering_position_val_avg = (int)(steering_wheel_position_sum / ((int)number_of_analog_samples));

    int steering_position_real_1 = (int)std::round(((float)steering_position_val_avg - (float)STEERING_OFFSETTED_MIN) / scale_factor_1);
    int steering_temp_1 = std::clamp(steering_position_real_1, STEERING_REAL_MIN, STEERING_REAL_MAX);

	return steering_temp_1;
}


