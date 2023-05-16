#include "can.hpp"

#include <array>

#define PUTM_CAN_FD

#include "PUTM_EV_CAN_LIBRARY/lib/can.hpp"

#include "brake_pressure.hpp"
#include "suspension.hpp"
#include "safety.hpp"
#include "apps.hpp"

namespace can {

namespace {
PUTM_CAN::FDCAN can;
constexpr time_t frame_period_ms{100};
}

void init(FDCAN_HandleTypeDef * can) noexcept {
	can = PUTM_CAN::FDCAN(can, PUTM_CAN::filter::null_filter, PUTM_CAN::CAN_FD_Format::CLASSIC_CAN);
}

bool should_send_frames() noexcept {
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
	PUTM_CAN::Apps_main apps_data {
		.pedal_position = apps_value,
		.counter = counter++,
		.position_diff = apps_value - last_value,
		.device_state = device_state,
	};
	last_value = apps_value;
	return can.send(apps_data);
}

bool send_data_acquisition_card_main_frame(BrakePressure brake_pressure, Suspension suspension, SafetyState safety_state) noexcept {
	PUTM_CAN::AQ_main aq_main {
		.brake_pressure_front = brake_pressure.front,
		.brake_pressure_back = brake_pressure.rear,
		.suspension_left = suspension.left,
		.suspension_right = suspension.right,
		.safety_byte = safety_state.safety_state_byte,
		.device_state = PUTM_CAN::AQ_states::Normal_operation,
	};
	aq_main.braking = braking(brake_pressure);

	return can.send(aq_main);
}

bool send_data_acquisition_card_imu_frames(std::array<IMU::IMUData_t, 3> imu_acc, std::array<IMU::IMUData_t, 3> imu_gyro) noexcept {
	const auto acc_sent = can.send<PUTM_CAN::AQ_acceleration>(*((PUTM_CAN::AQ_acceleration *)imu_acc.data()));
	const auto gyro_sent = can.send<PUTM_CAN::AQ_gyroscope>(*((PUTM_CAN::AQ_gyroscope*)imu_gyro.data()));

	return acc_sent and gyro_sent;
}

bool send_rtd_frame() noexcept {
	PUTM_CAN::AQ_ts_button ts_button{};
	return can.send(ts_button);
}

}
