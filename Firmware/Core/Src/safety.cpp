#include "safety.hpp"

namespace safety {

    SafetyState get_safety_state() {
        return SafetyState{
            .ebs = HAL_GPIO_Read(),
        };
    } 
}