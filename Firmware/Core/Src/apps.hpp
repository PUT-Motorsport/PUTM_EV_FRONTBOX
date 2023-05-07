#pragma once

#include <cstdint>

namespace apps {
    using apps_t = uint16_t;
    using adc_data_t = uint16_t;

    apps_t get_apps_value_from_raw(adc_data_t raw_value_1, adc_data_t raw_value_2);

    void set_nonlinearity_coefficient(float coefficient);
    float nonlinearity(float arg);
}