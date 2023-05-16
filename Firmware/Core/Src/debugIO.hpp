#pragma once

#include "main.h"
#include <cstdint>

enum class State : uint8_t {
    OK = 1,
    CANMissedMsgWarning,	//can message not sent
	BadSensorRead,			//sensor read failure
	AssertionFailed,		//RUNTIME_ASSERT failed
	IOSPIError,				//SPI communications failed
	HALError,				//HAL routine called Error_Handler()
	HALAssertionFailed,		//HAL's internal assert_params failed
	InitFail,				//some part of the initialization routines failed
	BadInitSeq,				//attempted an operation without required initializations
	CANError,				//CAN bus communications failed
};

namespace {
State state_ = State::OK;
}

namespace Device {

inline void resetLEDS() {
    /*
     * The debug LED pins have inverted logic
     */

    HAL_GPIO_WritePin(DEBUG1_GPIO_Port, DEBUG1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DEBUG2_GPIO_Port, DEBUG2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DEBUG3_GPIO_Port, DEBUG3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DEBUG4_GPIO_Port, DEBUG4_Pin, GPIO_PIN_SET);
}

inline void setState(State state) {

    if (static_cast<uint8_t>(state) <= static_cast<uint8_t>(state_)) { //status level must persist unless a more important error has appeared
        return;
    }
    state_ = state;

    //display a 0-15 number using 4 available LEDs
    uint8_t state_value{static_cast<uint8_t>(state_)};

    /*
     * The debug LED pins have inverted logic
     */

    resetLEDS();

    if (state_value & (1u << 0)) {
    	HAL_GPIO_WritePin(DEBUG1_GPIO_Port, DEBUG1_Pin, GPIO_PIN_RESET);
    }
    if (state_value & (1u << 1)) {
    	HAL_GPIO_WritePin(DEBUG2_GPIO_Port, DEBUG2_Pin, GPIO_PIN_RESET);
    }
    if (state_value & (1u << 2)) {
    	HAL_GPIO_WritePin(DEBUG3_GPIO_Port, DEBUG3_Pin, GPIO_PIN_RESET);
    }
    if (state_value & (1u << 3)) {
    	HAL_GPIO_WritePin(DEBUG4_GPIO_Port, DEBUG4_Pin, GPIO_PIN_RESET);
    }
}

} //namespace Device

__attribute__((noreturn)) inline void unrecoverableError(State errorType) {
	/*
	 * This function is meant to be called when:
	 * 1. A failure encountered is so serious that it makes no sense to continue
	 * 2. A failure encountered should be eliminated in debug, so the call stack is preserved
	 * */
	Device::setState(errorType);

	__disable_irq();

	while(true);
}

#ifndef NDEBUG
#define RUNTIME_ASSERT(statement) {if (not (statement)) unrecoverableError(State::AssertionFailed);}
#else
#define RUNTIME_ASSERT(statement) ;
#endif
