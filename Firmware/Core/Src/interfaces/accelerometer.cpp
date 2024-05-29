/*
 * accelerometer.cpp
 *
 *  Created on: Apr 8, 2024
 *      Author: PW
 */

#include "interfaces/AccelerometerAbstract.hpp"
//void Accelerometer::acc_initial(I2C_HandleTypeDef I2C)
//{
//	HAL_I2C_Mem_Read(&I2C, acc_addres << 1, WHO_AM_I, 1, &device_check,sizeof(device_check), HAL_MAX_DELAY);
//	if(device_check==0x6A)
//	{
//		 HAL_I2C_Mem_Write(&I2C, acc_addres << 1, CTRL1_XL, 1 , &Ctrl_1_init, sizeof(Ctrl_1_init), HAL_MAX_DELAY);
//		 HAL_I2C_Mem_Write(&I2C, acc_addres << 1, INT1_CTRL, 1 , &Int1_init, sizeof(Int1_init), HAL_MAX_DELAY);
//	}
//}
//
//void Accelerometer::acc_update_value( I2C_HandleTypeDef I2C)
//{
//	uint8_t bufor [2];
//
//	HAL_I2C_Mem_Read(&I2C, acc_addres << 1, OUTX_L_XL, 1, &bufor[0], sizeof(bufor[0]), HAL_MAX_DELAY);
//	HAL_I2C_Mem_Read(&I2C, acc_addres << 1, OUTX_H_XL, 1, &bufor[1], sizeof(bufor[1]), HAL_MAX_DELAY);
//	acc_val[0]=(bufor[1]<<8)|bufor[0];
//	acc_val[0]=acc_val[0];//16383;//2^14
//
//	HAL_I2C_Mem_Read(&I2C, acc_addres << 1, OUTY_L_XL, 1, &bufor[0], sizeof(bufor[0]), HAL_MAX_DELAY);
//	HAL_I2C_Mem_Read(&I2C, acc_addres << 1, OUTY_H_XL, 1, &bufor[1], sizeof(bufor[1]), HAL_MAX_DELAY);
//	acc_val[1]=(bufor[1]<<8)|bufor[0];
//	acc_val[1]=acc_val[1];///16383;//2^14
//
//	HAL_I2C_Mem_Read(&I2C, acc_addres << 1, OUTZ_L_XL, 1, &bufor[0], sizeof(bufor[0]), HAL_MAX_DELAY);
//	HAL_I2C_Mem_Read(&I2C, acc_addres << 1, OUTZ_H_XL, 1, &bufor[1], sizeof(bufor[1]), HAL_MAX_DELAY);
//	acc_val[2]=(bufor[1]<<8)|bufor[0];
//	acc_val[2]=acc_val[2];///16383;//2^14
//}
