#pragma once

#include "PUTM_EV_CAN_LIBRARY/lib/CanHeaders/can_headers.hpp"
#include <array>
#include "main.h"
#include <time.h>
#include "stm32g4xx_hal_fdcan.h"
#include "apps.hpp"
#include "imu.hpp"
#include "brake_pressure.hpp"
#include "suspension.hpp"
#include "safety.hpp"

namespace can {

void init(FDCAN_HandleTypeDef * can) noexcept;

bool should_send_frame() noexcept;

bool send_apps_frame(apps::apps_t apps_value, PUTM_CAN::Apps_states device_state) noexcept;

bool send_data_acquisition_card_main_frame(BrakePressure brake_pressure, Suspension suspension, SafetyState safety_state) noexcept;

bool send_data_acquisition_card_imu_frames(std::array<IMU::IMUData_t, 3> imu_acc, std::array<IMU::IMUData_t, 3> imu_gyro) noexcept;

bool send_rtd_frame() noexcept;

}


