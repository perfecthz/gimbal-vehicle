/*
 * Kinematics_Mecanum.c
 *
 *  Created on: Jan 29, 2022
 *      Author: bohang
 */
#include <math.h>
#include "Kinematics_Mecanum.h"
#include "MotorControl.h"

VelControl_Struct kenematics_mecanum_inverse (uint16_t x1, uint16_t y1, uint16_t x2){
	VelControl_Struct expected_rpms;
	float scaled_x1, scaled_y1, scaled_x2;
	float temp_x1, temp_y1, temp_x2;
	int RL, FB, Turn;
	RL = 0;
	FB = 0;
	Turn = 0;
	float k;
	float move_sum;
	// our implementation decided that the activation trigger value for any slight movement starts at 1500 - 2500, total possible value for x1 (right and left movement), y1(front and back movement), x2(rotation) would be 0 - 4000
	if (abs(x1-2000) < 500){
		scaled_x1 = 0;
	}
	else {
		scaled_x1 = ((x1 - 2000)>= 0) ?  (float)(x1 - 2500) / 1500 * 100  : (float)(x1 - 1500) / 1500 * 100;
	}

	if (abs(y1-2000) < 500){
		scaled_y1 = 0;
	}
	else {
		scaled_y1 = ((y1 - 2000)>= 0) ?  (float)(y1 - 2500) / 1500 * 100  : (float)(y1 - 1500) / 1500 * 100;
	}
	if (abs(x2-2000) < 500){
		scaled_x2 = 0;
	}
	else {
		scaled_x2 = ((x2 - 2000)>= 0) ?  (float)(x2 - 2500) / 1500 * 100  : (float)(x2 - 1500) / 1500 * 100;
	}


//	scaled_x1 = ((x1 - 2000)>= 0) ?  (x1 - 2500) / 1500 * 100  : (x1 - 1500) / 1500 * 100;
//	scaled_y1 = ((x1 - 2000)>= 0) ?  (x1 - 2500) / 1500 * 100  : (x1 - 1500) / 1500 * 100;
//	scaled_x2 = ((x1 - 2000)>= 0) ?  (x1 - 2500) / 1500 * 100  : (x1 - 1500) / 1500 * 100;

	//	move_sum = abs(scaled_x1) + abs(scaled_y1) + abs(scaled_x2);
	temp_x1 = scaled_x1;
	temp_y1 = scaled_y1;
	temp_x2 = scaled_x2;
	if(scaled_x1 < 0) {
		temp_x1 = scaled_x1 * -1;
	}
	if(scaled_y1 < 0) {
		temp_y1 = scaled_y1 * -1;
	}
	if(scaled_x2 < 0) {
		temp_x2 = scaled_x2 * -1;
	}

	move_sum = temp_x1 + temp_y1 + temp_x2;
	if(move_sum == 0){
		RL = 0;
		FB = 0;
		Turn = 0;
	}
	else if(move_sum >=100){
		k = 100 / move_sum;
		FB = scaled_y1 * k;
		RL = scaled_x1 * k;
		Turn = scaled_x2 * k;
	}
	else{
		FB = scaled_y1;
		RL = scaled_x1;
		Turn = scaled_x2;
	}

	MotorFL_Input(FB + RL * 1 + Turn);
	MotorBL_Input(FB + RL * -1 + Turn);
	MotorFR_Input(FB + RL * -1 + Turn * -1);
	MotorBR_Input(FB + RL * 1 + Turn * -1);


	expected_rpms.M1_SetRPM = abs(FB + RL * 1 + Turn) < 40 ? 0: (abs(FB + RL * 1 + Turn)*6.5267 - 243.98);
	expected_rpms.M1_SetRPM = expected_rpms.M1_SetRPM > 330 ? 330: expected_rpms.M1_SetRPM;

	expected_rpms.M2_SetRPM = abs(FB + RL * -1 + Turn) < 40 ? 0: (abs(FB + RL * -1 + Turn)*6.5267 - 243.98);
	expected_rpms.M2_SetRPM = expected_rpms.M2_SetRPM > 330 ? 330: expected_rpms.M2_SetRPM;

	expected_rpms.M3_SetRPM = abs(FB + RL * -1 + Turn * -1) < 40 ? 0: (abs(FB + RL * -1 + Turn * -1)*6.5267 - 243.98);
	expected_rpms.M3_SetRPM = expected_rpms.M3_SetRPM > 330 ? 330: expected_rpms.M3_SetRPM;

	expected_rpms.M4_SetRPM = abs(FB + RL * 1 + Turn * -1) < 40 ? 0: (abs(FB + RL * 1 + Turn * -1)*6.5267 - 243.98);
	expected_rpms.M4_SetRPM = expected_rpms.M4_SetRPM > 330 ? 330: expected_rpms.M4_SetRPM;

	return expected_rpms;
//	  MotorFR_Forward();
//	  TIM2->CCR1 = 50;
//	  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
//	  MotorBR_Forward();
//	  TIM1->CCR4 = 50;
//	  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
//	  MotorFL_Forward();
//	  TIM1->CCR1 = 50;
////	  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
//	  MotorBL_Forward();
//	  TIM1->CCR3 = 50;
//	  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);

}
