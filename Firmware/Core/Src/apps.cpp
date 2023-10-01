#include "apps.hpp"

#include <cmath>
#include "main.h"

namespace apps {
    namespace {
        // todo: apps can be recalibrated, but these values should provide a good starting point
        adc_data_t apps_1_min_value;
        adc_data_t apps_2_min_value;

        adc_data_t apps_1_max_value;
        adc_data_t apps_2_max_value;

        uint32_t time_calibration_end{};
        constexpr uint32_t calibration_time{5000};
        constexpr float implausibility_coefficient{0.1};
        PUTM_CAN::Apps_states apps_state;
    }

    std::pair<apps_t, PUTM_CAN::Apps_states> get_apps_value_from_raw(adc_data_t raw_value_1, adc_data_t raw_value_2) noexcept {
        if (apps_state not_eq PUTM_CAN::Apps_states::Normal_operation) {
            return std::pair{0, apps_state};
        }
        if (HAL_GetTick() < time_calibration_end) {
            // register highest and lowest values
            if (raw_value_1 < apps_1_min_value) {
                apps_1_min_value = raw_value_1;
            } else if (raw_value_1 > apps_1_max_value) {
                apps_1_max_value = raw_value_1;
            }
            if (raw_value_2 < apps_2_min_value) {
                apps_2_min_value = raw_value_2;
            } else if (raw_value_2 > apps_2_max_value) {
                apps_2_max_value = raw_value_2;
            }

            return std::pair{0, PUTM_CAN::Apps_states::Normal_operation};
        }

        if (raw_value_1 > apps_1_max_value) {
            apps_state = PUTM_CAN::Apps_states::Left_sensor_out_of_range_upper_bound;
            return std::pair{0, apps_state};
        } else if (raw_value_1 < apps_1_min_value) {
            apps_state = PUTM_CAN::Apps_states::Left_sensor_out_of_range_lower_bound;
            return std::pair{0, apps_state};
        }
        if (raw_value_2 > apps_1_max_value) {
            apps_state = PUTM_CAN::Apps_states::Right_sensor_out_of_range_upper_bound;
            return std::pair{0, apps_state};
        } else if (raw_value_2 < apps_2_min_value) {
            apps_state = PUTM_CAN::Apps_states::Right_sensor_out_of_range_lower_bound;
            return std::pair{0, apps_state};
        }

        // rescale apps values to 0-1
        float apps_1 = (apps_1 - apps_1_min_value) / (apps_1_max_value - apps_1_min_value);
        float apps_2 = (apps_2 - apps_2_min_value) / (apps_2_max_value - apps_2_min_value);

        if ((apps_1 - apps_2) > implausibility_coefficient) {
            apps_state = PUTM_CAN::Apps_states::Sensor_Implausiblity;
            return std::pair{0, apps_state};
        }

        float apps = (apps_1 + apps_2) / 2;

        return std::pair{nonlinearize(apps), PUTM_CAN::Apps_states::Normal_operation};
    }

    void calibrate() noexcept {
        time_calibration_end = HAL_GetTick() + calibration_time;
    }

    namespace {
        float apps_coefficient{1.0f};
    }
    void set_coefficient(float coefficient) noexcept {
        if (coefficient >= 2.0f or coefficient <= 0.5f)
            return;
        
        apps_coefficient = coefficient;
    }

    // expects the argument to be 0-1
    float nonlinearize(float arg) noexcept {
        return 500 * std::pow(arg, apps_coefficient);        
    }

}
