/*
 * brakes.cpp
 *
 *  Created on: Feb 10, 2024
 *      Author: ketirange
 */

#include "BrakesAbstract.hpp"

std::pair<int, int> Brakes::get_raw_avg_press_value()
{
	float press_temp_raw_sum_1 = 0;
	float press_temp_raw_sum_2 = 0;

	for(unsigned int i = 0; i < number_of_brake_pressure_sample; i = i + 2)
	{
		press_temp_raw_sum_1 += brake_pressure1_val_raw[i];
		press_temp_raw_sum_2 += brake_pressure2_val_raw[i + 1];
	}

	int press_temp_raw_avg_1 = (int)(press_temp_raw_sum_1 / ((float)number_of_brake_pressure_sample / 2.0f));
	int press_temp_raw_avg_2 = (int)(press_temp_raw_sum_2 / ((float)number_of_brake_pressure_sample / 2.0f));

	return std::make_pair(press_temp_raw_avg_1, press_temp_raw_avg_2);
}


