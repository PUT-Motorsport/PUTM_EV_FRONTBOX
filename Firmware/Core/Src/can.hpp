#pragma once

#include "apps.hpp"

namespace can {
namespace {
	constexpr time_t frame_period_ms{100};
}

bool should_send_frame();

bool send_apps_frame(apps_t apps_value);

bool send_data_acquisition_card_main_frame();

bool send_data_acquisition_card_imu_frames();

}


