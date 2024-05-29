/*
 * brakes.cpp
 *
 *  Created on: Feb 10, 2024
 *      Author: ketirange
 */

#include "interfaces/BrakesAbstract.hpp"

std::pair<int, int> Brakes::get_raw_avg_press_value()
{
	float rear_press_temp_raw_sum_1 = 0;
	float front_press_temp_raw_sum_2 = 0;

	for(unsigned int i = 0; i < number_of_brake_pressure_sample; i = i + 2)
	{
		rear_press_temp_raw_sum_1 += brake_pressure_rear_val_raw[i];
		front_press_temp_raw_sum_2 += brake_pressure_front_val_raw[i + 1];
	}

	int rear_press_temp_raw_avg_1 = (int)(rear_press_temp_raw_sum_1 / ((float)number_of_brake_pressure_sample / 2.0f));
	int front_press_temp_raw_avg_2 = (int)(front_press_temp_raw_sum_2 / ((float)number_of_brake_pressure_sample / 2.0f));

	return std::make_pair(rear_press_temp_raw_avg_1, front_press_temp_raw_avg_2);
}


