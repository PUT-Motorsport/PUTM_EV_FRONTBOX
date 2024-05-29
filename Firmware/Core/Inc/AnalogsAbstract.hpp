/*
 * AnalogsAbstract.hpp
 *
 *  Created on: Feb 10, 2024
 *      Author: ketirange
 */

#ifndef INC_ANALOGSABSTRACT_HPP_
#define INC_ANALOGSABSTRACT_HPP_

#include <stdlib.h>
#include <algorithm>
#include <cstdint>
#include <stdbool.h>
#include <math.h>

class Analog{
private:
	static const unsigned number_of_analog_samples = 50;
	static const uint16_t min_angle = 270;
	static const uint16_t max_angle = 2850;
	static constexpr uint16_t sensor_range = max_angle - min_angle;

public:
	uint16_t steering_position_val_raw[number_of_analog_samples];
	uint16_t steering_position_val_avg;
	uint16_t get_steering_position();
};

#endif /* INC_ANALOGSABSTRACT_HPP_ */
