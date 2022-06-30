#ifndef SERVO_H
#define SERVO_H

#include "stm32f4xx_hal.h"
#include "mpu.h"


void adjust_servo_position_x(TIM_HandleTypeDef *HTIMx, MPU6050_t *DataStruct); // Adjusting position of the servo motor according to MPU readings

void adjust_servo_position_z(TIM_HandleTypeDef *HTIMx, MPU6050_t *DataStruct); // Adjusting position of the servo motor according to MPU readings

void manual_adjust_servo_position_x(TIM_HandleTypeDef *HTIMx, uint32_t x_value); // Adjusting position of the servo motor according to joystick readings

void manual_adjust_servo_position_z(TIM_HandleTypeDef *HTIMx, uint32_t z_value); // Adjusting position of the servo motor according to MPU joystick readings


#endif
