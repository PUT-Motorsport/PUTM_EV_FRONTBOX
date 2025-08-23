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


	static const int STEERING_RAW_MIN = 0;
	static const int STEERING_RAW_MAX = 2690;
	//static const int STEERING_RESET = 3040;//3070;


	static const int STEERING_RAW_MIN2 = 1680;
	static const int STEERING_RAW_MAX2 = 230;

	const int STEERING_RAW_FULLSCALE = STEERING_RAW_MAX - STEERING_RAW_MIN;

	const int STEERING_RAW_FULLSCALE2 = STEERING_RAW_MAX2 - STEERING_RAW_MIN2;

	const int STEERING_OFFSET_MIN = 135;

	const int STEERING_REAL_MIN = -135;
	const int STEERING_REAL_MAX = 135;

	const int STEERING_REAL_SCALE = STEERING_REAL_MAX - STEERING_REAL_MIN;

	const float scale_factor_1 = STEERING_RAW_FULLSCALE/STEERING_REAL_SCALE;
	const float scale_factor_2 = STEERING_RAW_FULLSCALE2/STEERING_REAL_SCALE;

public:
	uint16_t steering_position_val_raw[number_of_analog_samples];
	uint16_t steering_position2_val_raw[number_of_analog_samples];
	//uint16_t steering_position_val_avg;
	//uint16_t steering_position2_val_avg;
	int16_t get_steering_position();
	int16_t get_steering_position2();
};

#endif /* INC_ANALOGSABSTRACT_HPP_ */
