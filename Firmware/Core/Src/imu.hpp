#pragma once

/*
 * This is the C++ wrapper of ISM330DHCX low-level functions.
 * The following code is heavily inspired by examples provided by ST.
 * https://github.com/STMicroelectronics/ism330dhcx-pid
 * */

#include <array>
#include <span>
#include "ISM330DHCX/ism330dhcx_reg.h"

namespace IMU {

using IMUData_t = int16_t;

bool initialize();
bool reinitialize(uint32_t timeout);

void updateSensorData();

std::array<IMUData_t, 3> get_acc_data();
std::array<IMUData_t, 3> get_gyro_data();

} //namespace IMU