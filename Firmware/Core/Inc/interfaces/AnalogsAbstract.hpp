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

	static const int STEERING_RAW_MIN = 330;
	static const int STEERING_RAW_MAX = 2800;

	const int STEERING_RAW_FULLSCALE = STEERING_RAW_MAX - STEERING_RAW_MIN;
	const int STEERING_OFFSETTED_MIN = (int)std::round(STEERING_RAW_FULLSCALE * 0 + STEERING_RAW_MIN);

	const int STEERING_REAL_MIN = 0;
	const int STEERING_REAL_MAX = 270;

	const int STEERING_REAL_SCALE = STEERING_REAL_MAX - STEERING_REAL_MIN;

	const float scale_factor_1 = (float)(((float)STEERING_RAW_MAX - (float)STEERING_OFFSETTED_MIN) / (float)STEERING_REAL_MAX);


public:
	uint16_t steering_position_val_raw[number_of_analog_samples];
	uint16_t steering_position_val_avg;
	uint16_t get_steering_position();
};

#endif /* INC_ANALOGSABSTRACT_HPP_ */
