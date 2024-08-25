/*
 * apps.hpp
 *
 *  Created on: Feb 10, 2024
 *      Author: ketirange
 */

#ifndef INC_APPSABSTRACT_HPP_
#define INC_APPSABSTRACT_HPP_

#include <stdlib.h>
#include <algorithm>
#include <cstdint>
#include <stdbool.h>
#include <math.h>

class Apps{
private:
	const float apps_dead_zone = 0.05;
	static const unsigned number_of_apps_sample = 50;
	bool sensor_plausibility_last = false;
	// const for apps sensor 1
	static const int APPS_1_RAW_MIN = 192;
	static const int APPS_1_RAW_MAX = 2840;
	static_assert(APPS_1_RAW_MIN < APPS_1_RAW_MAX);
	const int APPS_1_RAW_FULLSCALE = APPS_1_RAW_MAX - APPS_1_RAW_MIN;
	const int APPS_1_OFFSETTED_MIN = (int)std::round(APPS_1_RAW_FULLSCALE * apps_dead_zone + APPS_1_RAW_MIN);
	// const for apps sensor 2
	static const int APPS_2_RAW_MIN = 467;
	static const int APPS_2_RAW_MAX = 3550;
	static_assert(APPS_2_RAW_MIN < APPS_2_RAW_MAX);
	const int APPS_2_RAW_FULLSCALE = APPS_2_RAW_MAX - APPS_2_RAW_MIN;
	const int APPS_2_OFFSETTED_MIN = (int)std::round(APPS_2_RAW_FULLSCALE * apps_dead_zone + APPS_2_RAW_MIN);
	// const for apps scaled values
	const int APPS_REAL_MIN = 0;
	const int APPS_REAL_MAX = 500;
	const int APPS_REAL_SCALE = APPS_REAL_MAX - APPS_REAL_MIN;
	const float scale_factor_1 = (float)(((float)APPS_1_RAW_MAX - (float)APPS_1_OFFSETTED_MIN) / (float)APPS_REAL_MAX);
	const float scale_factor_2 = (float)(((float)APPS_2_RAW_MAX - (float)APPS_2_OFFSETTED_MIN) / (float)APPS_REAL_MAX);
	// FIXME sensor value to 10%
	const float sensor_implausibility_factor = 0.1;
	enum struct APPS_map_profile{
	    APPS_MAP_1_linear,
	    APPS_MAP_2,
	    APPS_MAP_3,
	    APPS_MAP_4
	} ;
public:
	uint16_t apps1_val_raw[number_of_apps_sample];
	uint16_t apps2_val_raw[number_of_apps_sample];

	std::pair<int, int> get_raw_avg_apps_value();
	std::pair<int, int> get_raw_avg_press_value();
	bool get_sensors_plausibility(int apps_raw_value_1, int apps_raw_value_2);
	uint16_t get_value_to_send();
	int apps_nonlinear_curve(int apps, APPS_map_profile map);
	float horner(const float *arry, unsigned int array_size, float x);
};


#endif /* INC_APPSABSTRACT_HPP_ */
