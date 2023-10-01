#include "can.hpp"

#include <array>

#define PUTM_CAN_FD

#include "PUTM_EV_CAN_LIBRARY/lib/can.hpp"

#include "brake_pressure.hpp"
#include "suspension.hpp"
#include "safety.hpp"
#include "apps.hpp"

PUTM_CAN::FDCAN fdcan;
constexpr time_t frame_period_ms{100};

FDCAN_FilterTypeDef null_filter = {
    .FilterIndex = 0,
    .FilterType = FDCAN_FILTER_MASK,
    .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
    .FilterID1 = 0,
    .FilterID2 = 0
};

void init(FDCAN_HandleTypeDef * fdCanHandle) noexcept
{
	fdcan = PUTM_CAN::FDCAN(fdCanHandle, null_filter, PUTM_CAN::CAN_FD_Format::CLASSIC_CAN);
}

bool should_send_frames() noexcept
{
	static time_t next_frame{HAL_GetTick() + frame_period_ms};

	if (HAL_GetTick() > next_frame) {
		next_frame += frame_period_ms;
		return true;
	}
	return false;
}

bool send_apps_frame(apps::apps_t apps_value, PUTM_CAN::Apps_states device_state) noexcept {
	static uint8_t counter{};
	static apps::apps_t last_value{};
	PUTM_CAN::Apps_main apps_data = {
		.pedal_position = apps_value,
		.counter = counter++,
		.position_diff = apps_value - last_value,
		.device_state = device_state,
	};
	last_value = apps_value;
	return fdcan.send(apps_data);
}

FDCAN_FilterTypeDef id(PUTM_CAN::can_id_t low, PUTM_CAN::can_id_t high)
{
	return FDCAN_FilterTypeDef {
    .FilterIndex = 0, .FilterType = FDCAN_FILTER_MASK,
    .FilterConfig = FDCAN_FILTER_TO_RXFIFO0, .FilterID1 = low << 21,
    .FilterID2 = high << 21
  };
}


