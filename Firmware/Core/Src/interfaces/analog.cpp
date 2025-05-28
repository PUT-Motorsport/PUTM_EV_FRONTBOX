/*
 * analog.cpp
 *
 *  Created on: May 24, 2024
 *      Author: ketirange
 */

#include "interfaces/AnalogsAbstract.hpp"

int16_t Analog::get_steering_position()
{
	float steering_wheel_position_sum = 0;
	float alpha = 0.5;
	float last_sample = 0;
	for(unsigned int i = 0; i < number_of_analog_samples; i++)
	{
		last_sample = alpha * steering_position_val_raw[i] +(1-alpha) * last_sample;
		steering_wheel_position_sum += last_sample;
	}

	uint64_t steering_position_val_avg = (int)(steering_wheel_position_sum / ((int)number_of_analog_samples));

	int16_t steering_position_calc =std::round(((steering_position_val_avg/scale_factor_1)-317)*0.9);

/*
	if(steering_position_val_avg<STEERING_RAW_MAX)
	{
		steering_position_val_avg-=STEERING_OFFSET_MIN;
	}

    //int steering_position_real_1 = (int)std::round(((float)steering_position_val_avg - (float)STEERING_OFFSETTED_MIN) / scale_factor_1);
	int steering_position_real_1 =(((int16_t)steering_position_val_avg % (int16_t)STEERING_RESET));//-(int)STEERING_OFFSETTED_MIN);///scale_factor_1;
    //int steering_temp_1 = std::clamp(steering_position_real_1, STEERING_REAL_MIN, STEERING_REAL_MAX);
    int steering_temp_1 = (int)std::round(((float)steering_position_real_1-(float)STEERING_RESET+(float)STEERING_RAW_MAX)/scale_factor_1);
    int16_t steering_temp_2=std::clamp(steering_temp_1,STEERING_REAL_MIN,STEERING_REAL_MAX);
*/
	return steering_position_calc;
}


