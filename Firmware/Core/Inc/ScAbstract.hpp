/*
 * ScAbstract.hpp
 *
 *  Created on: Feb 10, 2024
 *      Author: ketirange
 */

#ifndef INC_SCABSTRACT_HPP_
#define INC_SCABSTRACT_HPP_

#include <stdlib.h>
#include <algorithm>
#include <cstdint>
#include <stdbool.h>
#include "main.h"

class SC{
private:
	bool Sense_BSPD;
	bool Sense_Right;
	bool Sense_Overtravel;
	bool Sense_Right_Wheel;
	bool Sense_Left_Wheel;
	bool Sense_Driver;
	bool Sense_Left;
	bool Sense_EBS;
public:
bool SC_val[8];
void update_val();

};



#endif /* INC_SCABSTRACT_HPP_ */
