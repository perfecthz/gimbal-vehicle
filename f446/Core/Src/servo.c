#include "servo.h"

void adjust_servo_position_z(TIM_HandleTypeDef *HTIMx, MPU6050_t *DataStruct)
{

	if(DataStruct->KalmanAngleZ < 0 && DataStruct->KalmanAngleZ > -90) // When Yaw angle is between-90~0 deg
	{
		HTIMx->Instance->CCR2 = (DataStruct->KalmanAngleZ / -1.8) + 75;
	}
	else if(DataStruct->KalmanAngleZ <= -90) // When Yaw angle is less than -90 deg
	{
		HTIMx->Instance->CCR2 = 125;
	}
	else if(DataStruct->KalmanAngleZ > 0 && DataStruct->KalmanAngleZ < 90) // When Yaw angle is between 0~90 deg
	{
		HTIMx->Instance->CCR2 = (DataStruct->KalmanAngleZ / -1.8) + 75;
	}
	else if(DataStruct->KalmanAngleZ >= 90) // When Yaw angle is greater than deg
	{
		HTIMx->Instance->CCR2 = 25;
	}
	else if(DataStruct->KalmanAngleZ == 0) // When Yaw angle is 0 deg
	{
		HTIMx->Instance->CCR2 = 75;
	}
}

void adjust_servo_position_x(TIM_HandleTypeDef *HTIMx, MPU6050_t *DataStruct)
{
	if(DataStruct->KalmanAngleX > 0 && DataStruct->KalmanAngleX < 90) // When Pitch angle is between 0~90 deg
	{
		HTIMx->Instance->CCR1 = (DataStruct->KalmanAngleX / 1.8) + 75;
	}
	else if(DataStruct->KalmanAngleX >= 90) // When Pitch angle is greate than 90 deg
	{
		HTIMx->Instance->CCR1 = 125;
	}
	else if(DataStruct->KalmanAngleX < 0 && DataStruct->KalmanAngleX > -90) // When Pitch angle is between -90~0 deg
	{
		HTIMx->Instance->CCR1 = (DataStruct->KalmanAngleX / 1.8) + 75;
	}
	else if(DataStruct->KalmanAngleX <= -90) // When Pitch angle is less than-90 deg
	{
		HTIMx->Instance->CCR1 = 25;
	}
	else if(DataStruct->KalmanAngleX == 0) // When Pitch angle is 0 deg
	{
		HTIMx->Instance->CCR1 = 75;
	}
}

void manual_adjust_servo_position_x(TIM_HandleTypeDef *HTIMx, uint32_t x_value)
{
	if (x_value >= 1905 && x_value <= 2000) // If joystick reading is between 2900 and 3200
	{
		HTIMx->Instance->CCR1 = 75;
	}
	else if(x_value > 2000) // If joystick reading is greater than 3200
	{
		HTIMx->Instance->CCR1 = 75 + (float)((x_value - 2000) / (float)42);
	}
	else if(x_value < 1905) // If joystick reading is less than 2900
	{
		HTIMx->Instance->CCR1 = 75 - (float)((1905 - x_value) / (float)38);
	}
}


void manual_adjust_servo_position_z(TIM_HandleTypeDef *HTIMx, uint32_t y_value)
{
	if (y_value >= 1905 && y_value <= 2000) //If joystick reading is between 2900 and 3200
	{
		HTIMx->Instance->CCR2 = 75;
	}
	else if(y_value > 2000) // If joystick reading is greater than 3200
	{
		HTIMx->Instance->CCR2 = 75 + (float)((y_value - 2000) / (float)42);
	}
	else if(y_value < 1905) // If joystick reading is less than 2900
	{
		HTIMx->Instance->CCR2 = 75 - (float)((1905 - y_value) / (float)38);
	};
}
