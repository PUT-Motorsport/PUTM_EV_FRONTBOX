#pragma once

#include <cstdint>

struct Suspension {
    uint16_t left, right;
};

Suspension get_suspension_from_raw(uint16_t left, uint16_t right) {
	// todo: data processing
    return Suspension{left, right};
}
