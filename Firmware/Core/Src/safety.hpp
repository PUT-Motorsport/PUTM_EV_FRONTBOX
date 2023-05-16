#pragma once

#include <cstdint>

union SafetyState {
    struct __attribute__((packed)) {
    uint8_t ebs: 1;
    uint8_t inertia: 1;
    uint8_t driver_kill: 1;
    uint8_t bspd: 1;
    uint8_t right_kill: 1;
    uint8_t left_kill: 1;
    uint8_t overtravel: 1;
    };
    uint8_t safety_state_byte;
};

namespace safety {
    SafetyState get_safety_state() noexcept;
}