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

	//static const int STEERING_RAW_MIN = 330;
	//static const int STEERING_RAW_MAX = 2800;

	static const int STEERING_RAW_MIN = 900;
	static const int STEERING_RAW_MAX = 2340;
	static const int STEERING_RESET = 3040;//3070;

	const int STEERING_RAW_FULLSCALE = STEERING_RAW_MAX - STEERING_RAW_MIN;
	//const int STEERING_OFFSETTED_MIN = (int)std::round(STEERING_RAW_FULLSCALE * 0 + STEERING_RAW_MIN);
	const int STEERING_OFFSET_MIN = 140;

	const int STEERING_REAL_MIN = -135;
	const int STEERING_REAL_MAX = 135;

	const int STEERING_REAL_SCALE = STEERING_REAL_MAX - STEERING_REAL_MIN;

//	const float scale_factor_1 = (float)(((float)STEERING_RAW_MAX - (float)STEERING_OFFSETTED_MIN) / (float)STEERING_REAL_MAX);
	//const float scale_factor_1 =(2*STEERING_RAW_MIN)/(2*STEERING_REAL_MAX);
	const float scale_factor_1 =(STEERING_RESET-STEERING_RAW_MAX+STEERING_RAW_MIN-STEERING_OFFSET_MIN)/(2*STEERING_REAL_MAX);//5,407
public:
	uint16_t steering_position_val_raw[number_of_analog_samples];
	uint16_t steering_position_val_avg;
	int16_t get_steering_position();
};

#endif /* INC_ANALOGSABSTRACT_HPP_ */
