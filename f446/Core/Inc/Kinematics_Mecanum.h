/*
 * Kinematics_Mecanum.h
 *
 *  Created on: Jan 29, 2022
 *      Author: bohang
 */

#ifndef INC_KINEMATICS_MECANUM_H_
#define INC_KINEMATICS_MECANUM_H_
#include "stdint.h"

typedef struct
{
	float M1_RPM;	 // Real RPM FL
	float M2_RPM;	 // BL
	float M3_RPM;	 //	FR
	float M4_RPM;	 //	BR
	float M1_SetRPM; // Expected RPM
	float M2_SetRPM; //
	float M3_SetRPM; //
	float M4_SetRPM; //
}VelControl_Struct;

//typedef struct fourWheelPWMInfo {
//	uint16_t left_front_wheel, right_front_wheel, left_back_wheel, right_back_wheel;
//} fwInfo;

VelControl_Struct kenematics_mecanum_inverse (uint16_t x1, uint16_t y1, uint16_t x2);
#endif /* INC_KINEMATICS_MECANUM_H_ */
