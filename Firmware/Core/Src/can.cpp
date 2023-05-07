#include "can.hpp"

namespace can {
bool should_send_frames() {
	static time_t next_frame{HAL_GetTick() + frame_period_ms};

	if (HAL_GetTick() > next_frame) {
		next_frame += frame_period_ms;
		return true;
	}
	return false;
}
}
