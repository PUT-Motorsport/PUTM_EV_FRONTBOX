#include "safety.hpp"

#include "main.h"

namespace safety {

    SafetyState get_safety_state() noexcept {
    	SafetyState state;
            state.ebs = not HAL_GPIO_ReadPin(SENSE_EBS_GPIO_Port, SENSE_EBS_Pin);
            state.inertia = not HAL_GPIO_ReadPin(SENSE_INERTIA_GPIO_Port, SENSE_INERTIA_Pin);
			state.driver_kill = not HAL_GPIO_ReadPin(SENSE_DRIVER_GPIO_Port, SENSE_DRIVER_Pin);
			state.bspd = not HAL_GPIO_ReadPin(SENSE_BSPD_GPIO_Port, SENSE_BSPD_Pin);
			state.right_kill = not HAL_GPIO_ReadPin(SENSE_RIGHT_GPIO_Port, SENSE_RIGHT_Pin);
			state.left_kill = not HAL_GPIO_ReadPin(SENSE_LEFT_GPIO_Port, SENSE_LEFT_Pin);
			state.overtravel = not HAL_GPIO_ReadPin(SENSE_OVERTRAVEL_GPIO_Port, SENSE_OVERTRAVEL_Pin);
			return state;
    } 
}
