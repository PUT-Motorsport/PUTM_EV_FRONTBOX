#pragma once

using brake_pressure_t = uint16_t;

struct BrakePressure{
    brake_pressure_t front, rear;
};

inline BrakePressure get_brake_pressure_from_raw(uint16_t front, uint16_t rear) noexcept {
    return BrakePressure{front, rear};
}

inline bool braking(BrakePressure const brake_pressure) noexcept {
    constexpr brake_pressure_t braking_threshold{300};
    return (brake_pressure.front > braking_threshold or brake_pressure.rear > braking_threshold);
}
