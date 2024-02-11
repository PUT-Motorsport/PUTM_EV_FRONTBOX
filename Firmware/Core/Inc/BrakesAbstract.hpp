/*
 * BrakesAbstract.hpp
 *
 *  Created on: Feb 10, 2024
 *      Author: ketirange
 */

#ifndef INC_BRAKESABSTRACT_HPP_
#define INC_BRAKESABSTRACT_HPP_

#include <stdlib.h>
#include <algorithm>
#include <cstdint>
#include <stdbool.h>
#include <math.h>

class Brakes{
private:
	static const unsigned number_of_brake_pressure_sample = 50;
	bool sensor_plausibility_last = false;
public:
	uint16_t brake_pressure1_val_raw[number_of_brake_pressure_sample];
	uint16_t brake_pressure2_val_raw[number_of_brake_pressure_sample];
	uint16_t brake_position_val_raw [number_of_brake_pressure_sample];

	std::pair<int, int> get_raw_avg_press_value();
};



#endif /* INC_BRAKESABSTRACT_HPP_ */
