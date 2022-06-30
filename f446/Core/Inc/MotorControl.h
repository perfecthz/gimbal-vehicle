/*
 * MotorControl.h
 *
 *  Created on: Jan 29, 2022
 *      Author: bohan
 */

#ifndef INC_MOTORCONTROL_H_
#define INC_MOTORCONTROL_H_

#include "main.h"


void MotorFL_Forward(void);
void MotorFL_Reverse(void);
void MotorFL_Stop(void);

void MotorFR_Forward(void);
void MotorFR_Reverse(void);
void MotorFR_Stop(void);

void MotorBL_Forward(void);
void MotorBL_Reverse(void);
void MotorBL_Stop(void);

void MotorBR_Forward(void);
void MotorBR_Reverse(void);
void MotorBR_Stop(void);

void MotorFL_Input(int speed);
void MotorFR_Input(int speed);
void MotorBL_Input(int speed);
void MotorBR_Input(int speed);

#endif /* INC_MOTORCONTROL_H_ */
