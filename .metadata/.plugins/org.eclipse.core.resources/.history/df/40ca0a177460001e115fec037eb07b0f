#pragma once

#include "main.h"

#include "PM08-CANBUS-APPS.hpp"
#include "PM08-CANBUS-AQ_CARD.hpp"
#include "PM08-CANBUS-BMS_HV.hpp"
#include "PM08-CANBUS-BMS_LV.hpp"
#include "PM08-CANBUS-DASH.hpp"
#include "PM08-CANBUS-DV.hpp"
#include "PM08-CANBUS-LAP_TIMER.hpp"
#include "PM08-CANBUS-SF.hpp"
#include "PM08-CANBUS-STEERING_WHEEL.hpp"
#include "PM08-CANBUS-SW_SENSOR.hpp"
#include "PM08-CANBUS-TC.hpp"
#include "PM08-CANBUS-TELEMETRY.hpp"
#include "PM08-CANBUS-WHEELTEMP.hpp"
#include "PM08-CANBUS-YAWPROBE.hpp"

#include "../meta.hpp"

#include <limits>

namespace PUTM_CAN {

constexpr can_id_t invalid_can_id{std::numeric_limits<can_id_t>::max()};

template<typename frame_t>
constexpr can_id_t frame_id{invalid_can_id};

#define DECL_ID(type, id) template<> constexpr can_id_t frame_id<type>{id}

DECL_ID(Apps_main, APPS_MAIN_CAN_ID);
DECL_ID(AQ_main, AQ_MAIN_CAN_ID);
DECL_ID(AQ_acceleration, AQ_ACCELERATION_CAN_ID);
DECL_ID(AQ_gyroscope, AQ_GYROSCOPE_CAN_ID);
DECL_ID(AQ_ts_button, AQ_TS_BUTTON_CAN_ID);
DECL_ID(BMS_HV_main, BMS_HV_MAIN_CAN_ID);
DECL_ID(BMS_LV_main, BMS_LV_MAIN_CAN_ID);
DECL_ID(BMS_LV_temperature, BMS_LV_TEMPERATURE_CAN_ID);
DECL_ID(Dash_Main, DASH_MAIN_CAN_ID);
DECL_ID(DV_Ass, DV_ASS_CAN_ID);
DECL_ID(Lap_timer_Main, LAP_TIMER_MAIN_CAN_ID);
DECL_ID(Lap_timer_Sector, LAP_TIMER_SECTOR_CAN_ID);
DECL_ID(Lap_timer_Acc_time, LAP_TIMER_ACC_TIME_CAN_ID);
DECL_ID(Lap_timer_Skidpad_time, LAP_TIMER_SKIDPAD_TIME_CAN_ID);
DECL_ID(Lap_timer_Lap_time, LAP_TIMER_LAP_TIME_CAN_ID);
DECL_ID(SF_main, SF_MAIN_CAN_ID);
DECL_ID(SF_PassiveElements, SF_PASSIVEELEMENTS_CAN_ID);
DECL_ID(SF_LegendaryDVAndSupply, SF_LEGENDARYDVANDSUPPLY_CAN_ID);
DECL_ID(SF_Supply, SF_SUPPLY_CAN_ID);
DECL_ID(SF_safety, SF_SAFETY_CAN_ID);
DECL_ID(Steering_Wheel_main, STEERING_WHEEL_MAIN_CAN_ID);
DECL_ID(Steering_Wheel_event, STEERING_WHEEL_EVENT_CAN_ID);
DECL_ID(SWPS_main, SWPS_MAIN_CAN_ID);
DECL_ID(TC_main, TC_MAIN_CAN_ID);
DECL_ID(TC_rear_suspension, TC_REAR_SUSPENSION_CAN_ID);
DECL_ID(TC_wheel_velocities, TC_WHEEL_VELOCITIES_CAN_ID);
DECL_ID(TC_temperatures, TC_TEMPERATURES_CAN_ID);
DECL_ID(TC_imu_gyro, TC_IMU_GYRO_CAN_ID);
DECL_ID(TC_imu_acc, TC_IMU_ACC_CAN_ID);
DECL_ID(Telemetry_Main, TELEMETRY_MAIN_CAN_ID);
DECL_ID(WheelTemp_main, WHEELTEMP_MAIN_CAN_ID);
DECL_ID(YawProbe_air_flow, YAWPROBE_AIR_FLOW_CAN_ID);

} // namespace PUTM_CAN
