

/*
 * Sc.cpp
 *
 *  Created on: Feb 20, 2024
 *      Author: PW
 */

#include "ScAbstract.hpp"

void SC::update_val()
{
Sense_BSPD=HAL_GPIO_ReadPin(Sense_BSPD_GPIO_Port, Sense_BSPD_Pin);
SC_val[0]=Sense_BSPD;

Sense_Driver=HAL_GPIO_ReadPin(Sense_Driver_GPIO_Port, Sense_Driver_Pin);
SC_val[1]=Sense_Driver;

Sense_EBS=HAL_GPIO_ReadPin(Sense_EBS_GPIO_Port, Sense_EBS_Pin);
SC_val[2]=Sense_EBS;

Sense_Left=HAL_GPIO_ReadPin(Sense_Left_GPIO_Port, Sense_Left_Pin);
SC_val[3]=Sense_Left;

Sense_Left_Wheel=HAL_GPIO_ReadPin(Sense_Left_Wheel_GPIO_Port, Sense_Left_Wheel_Pin);
SC_val[4]=Sense_Left_Wheel;

Sense_Overtravel=HAL_GPIO_ReadPin(Sense_Overtravel_GPIO_Port, Sense_Overtravel_Pin);
SC_val[5]=Sense_Overtravel;

//Sense_Right=HAL_GPIO_ReadPin(Sense_Right_GPIO_Port, Sense_Right_Pin);
//SC_val[6]=Sense_Right;
SC_val[6]=0;

Sense_Right_Wheel=HAL_GPIO_ReadPin(Sense_Right_Wheel_GPIO_Port, Sense_Right_Wheel_Pin);
SC_val[7]=Sense_Right_Wheel;
}




