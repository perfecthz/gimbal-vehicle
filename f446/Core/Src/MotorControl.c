/*
 * MotorControl.c
 *
 *  Created on: Jan 29, 2022
 *      Author: bohang
 */


#include "MotorControl.h"
#include "Kinematics_Mecanum.h"
#include "main.h"
//FL
void MotorFL_Forward(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,GPIO_PIN_RESET);
}

void MotorFL_Reverse(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,GPIO_PIN_SET);
}

void MotorFL_Stop(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,GPIO_PIN_RESET);
}
//FR
void MotorFR_Forward(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_RESET);
}

void MotorFR_Reverse(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_SET);
}

void MotorFR_Stop(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_RESET);
}
//BL
void MotorBL_Forward(void){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,GPIO_PIN_RESET);
}

void MotorBL_Reverse(void){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,GPIO_PIN_SET);
}

void MotorBL_Stop(void){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,GPIO_PIN_RESET);
}
//BR
void MotorBR_Forward(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);
}

void MotorBR_Reverse(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_SET);
}

void MotorBR_Stop(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);
}
//INPUT
void MotorFL_Input(int speed) {
	if(speed > 0 ){
		  MotorFL_Forward();
		  TIM1->CCR1 = speed;
//		  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	}
	else if(speed < 0){
		  MotorFL_Reverse();
		  TIM1->CCR1 = -1 * speed;
//		  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	}
	else{
		TIM1->CCR1 = 100;
		MotorFL_Stop();
	}
}

void MotorFR_Input(int speed) {
	if(speed > 0){
		  MotorFR_Forward();
		  TIM1->CCR2 = speed;
//		  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	}
	else if(speed < 0){
		  MotorFR_Reverse();
		  TIM1->CCR2 = -1 * speed;
//		  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	}
	else{
		TIM1->CCR2 = 100;
		MotorFR_Stop();
	}
}

void MotorBL_Input(int speed) {
	if(speed > 0 ){
		  MotorBL_Forward();
		  TIM1->CCR3 = speed;
//		  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	}
	else if(speed < 0){
		  MotorBL_Reverse();
		  TIM1->CCR3 = -1 * speed;
//		  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	}
	else{
		TIM1->CCR3 = 100;
		MotorBL_Stop();
	}
}

void MotorBR_Input(int speed) {
	if(speed > 0 ){
		  MotorBR_Forward();
		  TIM1->CCR4 = speed;
//		  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	}
	else if(speed < 0){
		  MotorBR_Reverse();
		  TIM1->CCR4 = -1 * speed;
//		  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	}
	else{
		TIM1->CCR4 = 100;
		MotorBR_Stop();
	}
}
