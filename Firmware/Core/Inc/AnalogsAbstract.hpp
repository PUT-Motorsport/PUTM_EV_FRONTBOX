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
public:
	uint16_t steering_position_val_raw[number_of_analog_samples];
	uint16_t steering_position_val_avg;
	int16_t get_steering_position();
};

#endif /* INC_ANALOGSABSTRACT_HPP_ */
