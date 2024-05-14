/*
 * AccelerometerAbstract.hpp
 *
 *  Created on: Apr 8, 2024
 *      Author: PW
 */

#ifndef INC_ACCELEROMETERABSTRACT_HPP_
#define INC_ACCELEROMETERABSTRACT_HPP_

#include "main.h"

#define WHO_AM_I 0x0F
#define acc_addres 0x6b
#define INT1_CTRL 0x0d
#define CTRL1_XL 0x10
#define OUTX_L_XL 0x28
#define OUTX_H_XL 0x29
#define OUTY_L_XL 0x2A
#define OUTY_H_XL 0x2B
#define OUTZ_L_XL 0x2C
#define OUTZ_H_XL 0x2D



class Accelerometer
{
private:
	uint8_t device_check;
	uint8_t Ctrl_1_init = 0b01100100;// configuration of device 416Hz High_Performance +/- 16g mode in CTRL1_XL
	uint8_t Int1_init = 0x01;//configuration device

public:
void acc_update_value(I2C_HandleTypeDef );
void acc_initial(I2C_HandleTypeDef);
int16_t acc_val [3];
};


#endif /* INC_ACCELEROMETERABSTRACT_HPP_ */
