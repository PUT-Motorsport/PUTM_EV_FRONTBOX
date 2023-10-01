#pragma once

#include <cstdint>
#include <utility>
#include "PUTM_EV_CAN_LIBRARY/lib/CanHeaders/PM08-CANBUS-APPS.hpp"

namespace apps {
    using apps_t = uint16_t;
    using adc_data_t = uint16_t;

    std::pair<apps_t, PUTM_CAN::Apps_states> get_apps_value_from_raw(adc_data_t raw_value_1, adc_data_t raw_value_2) noexcept;

    void calibrate() noexcept;
    void set_nonlinearity_coefficient(float coefficient) noexcept;
    float nonlinearize(float arg) noexcept;
}
