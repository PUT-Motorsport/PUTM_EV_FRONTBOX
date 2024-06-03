/*
 * Amk.hpp
 *
 *  Created on: Jun 3, 2024
 *      Author: ketirange
 */

#ifndef INC_INTERFACES_AMK_HPP_
#define INC_INTERFACES_AMK_HPP_

#include <stdlib.h>
#include <algorithm>
#include <cstdint>
#include <stdbool.h>
#include <math.h>

#include "main.h"



class AMK{
private:


public:
	void AMK()
	{

	}
	void RunStateMachine(uint16_t pedalPosition, std::pair<uint16_t, uint16_t> BrakePressure, bool rtd_button);

};





#endif /* INC_INTERFACES_AMK_HPP_ */
