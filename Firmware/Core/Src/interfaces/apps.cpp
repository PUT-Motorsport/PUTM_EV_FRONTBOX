/*
 * apps.cpp
 *
 *  Created on: Feb 10, 2024
 *      Author: ketirange
 */

#include "interfaces/AppsAbstract.hpp"

extern bool sensor_plausibility_last;

std::pair<int, int> Apps::get_raw_avg_apps_value()
{
	float apps_temp_raw_sum_1 = 0;
	float apps_temp_raw_sum_2 = 0;

	for(unsigned int i = 0; i < number_of_apps_sample; i = i + 2)
	{
		apps_temp_raw_sum_1 += apps1_val_raw[i];
		apps_temp_raw_sum_2 += apps2_val_raw[i + 1];
	}

	int apps_temp_raw_avg_1 = (int)(apps_temp_raw_sum_1 / ((float)number_of_apps_sample / 2.0f));
	int apps_temp_raw_avg_2 = (int)(apps_temp_raw_sum_2 / ((float)number_of_apps_sample / 2.0f));

	return std::make_pair(apps_temp_raw_avg_1, apps_temp_raw_avg_2);
}

bool Apps::get_sensors_plausibility(int apps_raw_value_1, int apps_raw_value_2)
{

/* T 11.8.9 Implausibility is defined as a deviation of more than ten percentage points pedal travel
 between any of the used APPSs or any failure according to T 11.9.
 */

// fraction of full scale position
	float apps_scaled_1 = ((float)apps_raw_value_1 - (float)APPS_1_OFFSETTED_MIN) / (float)APPS_1_RAW_FULLSCALE;
	float apps_scaled_2 = ((float)apps_raw_value_2 - (float)APPS_2_OFFSETTED_MIN) / (float)APPS_2_RAW_FULLSCALE;
	float diff = fabsf(apps_scaled_1 - apps_scaled_2);
	//diff = fabsf(apps_scaled_1 - apps_scaled_2);

	if(diff > sensor_implausibility_factor) {
		return false;
	} else {
		return true;
	}
}

uint16_t Apps::get_value_to_send()
{
    auto [apps_avg_1, apps_avg_2] = get_raw_avg_apps_value();

    if(bool state = get_sensors_plausibility(apps_avg_1, apps_avg_2); !state && !sensor_plausibility_last)
    {
    	// turn led on
    	// HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PinState::GPIO_PIN_RESET);
    	apps_flag=1;
        return 0;
    }
    apps_flag=0;
    // range calculation
    int apps_real_1 = (int)std::round(((float)apps_avg_1 - (float)APPS_1_OFFSETTED_MIN) / scale_factor_1);
    int apps_real_2 = (int)std::round(((float)apps_avg_2 - (float)APPS_2_OFFSETTED_MIN) / scale_factor_2);

    // clamping real values
    int apps_temp_1 = std::clamp(apps_real_1, APPS_REAL_MIN, APPS_REAL_MAX);
    int apps_temp_2 = std::clamp(apps_real_2, APPS_REAL_MIN, APPS_REAL_MAX);

    // get average value of two sensors
    int apps_temp = (apps_temp_1 + apps_temp_2) / 2;

    // non linear curve
    return apps_nonlinear_curve(apps_temp, APPS_map_profile::APPS_MAP_1_linear);

}
float Apps::horner(const float *arry, unsigned int array_size, float x)
{
    float s = 0;
    for(unsigned int i = 0; i < array_size ; i++){
        s = s*(x) + arry[i];
    }
    return s;
}

int Apps::apps_nonlinear_curve(int apps, APPS_map_profile map){

    float apps_f = static_cast<float>(apps);

    const float pedal_map[4][5]  = {
        // linear #dziala
        { 0.00,  0.00,   0.00,   1.00,  0.00},
        // blue #dziala
        {-0.000000002291667,   0.000002986111111,  -0.000572916666667,   0.827579365079361,  -0.797619047618863},
        // green #dziala
        { 0.000000014583333,  -0.000009768518519,   0.002368055555556,   0.434391534391535,   0.198412698412663},
        // red  #dziala
        { 0.000000011458333,  -0.000006319444444,   0.001156250000000,   0.568253968253969,   0.297619047618997} };

    const uint16_t array_order = sizeof(pedal_map[0]) / sizeof(pedal_map[0][0]);
    static_assert(array_order == 5, "Array is using 4 order polynomial");

    std::size_t map_it = static_cast<std::size_t>(map);

    int torque = (int)(horner(&pedal_map[map_it][0],array_order,apps_f) + 0.5);

    torque = std::clamp(torque, APPS_REAL_MIN, APPS_REAL_MAX);

    return torque;
}


