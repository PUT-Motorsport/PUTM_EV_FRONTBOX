/*
 * Tensometers.h
 *
 *  Created on: Mar 17, 2025
 *      Author: pwlub
 */

#ifndef INC_INTERFACES_TENSOMETERS_HPP_
#define INC_INTERFACES_TENSOMETERS_HPP_

#include <stdlib.h>
#include <algorithm>
#include <cstdint>
#include <stdbool.h>
#include <math.h>

class Tensometers {
private:
	static const unsigned number_of_analog_samples = 50;
public:
	uint16_t tensometer_1_val_raw[number_of_analog_samples];
	int16_t get_tens_1();


};

#endif /* INC_INTERFACES_TENSOMETERS_HPP_ */
