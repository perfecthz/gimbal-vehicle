#ifndef MPU_H
#define MPU_H

#include "stm32f4xx_hal.h"

#define MPU6050_ADDR 0xD0

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

typedef struct // data structure holding values of the IMU
{
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    float Zero_Offset_X;
    float Zero_Offset_Y;
    float Zero_Offset_Z;

    double KalmanAngleX;
    double KalmanAngleY;
    double KalmanAngleZ;

} MPU6050_t;

typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct); // Read accelerometer data and store to MPU6050_t data structure

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct); // Read gyroscope data and store to MPU6050_t data structure

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct); // Read temperature sensor data and store to MPU6050_t data structure

void get_Zero_Offset(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct); // Use 1s to calculate the offset of the gyroscope reading and store to MPU6050_t data structure for future calibration

void get_Yaw(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct); // Calculate the yaw angle of the IMU and store it to MPU6050_t data structure

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct); // Read data from all sensors and calculated Roll angle and Pitch using Kalman Filter, then store to MPU6050_t data structure

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt); // Calculate the roll and pitch angle of the IMU


#endif
