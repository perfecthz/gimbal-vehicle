#include "mpu.h"
#include <math.h>
#include "stm32f4xx_hal_i2c.h"
#include "error.h"
#define RAD_TO_DEG 57.295779513082320876798154814105

const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;

uint32_t timer;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
	uint8_t check, Data;
	// check device ID WHO_AM_I
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

	if(check == 104) // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);
		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);
		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> +- 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);
		// Set Gyroscope configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> +- 250 deg/s
		Data = 0x00;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
		return 0;
	}else{

	}
	return 1;
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
	uint8_t Rec_Data[6];
	// Read 6 BYTES of data starting from ACCEL_XOUT_H register
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);
	DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
	//convert the RAW values into acceleration in 'g'
	//we have to divide according to the Full scale value set in FS_SEL
	//I have configured FS_SEL = 0. So I am dividing by 16384.0
	//for more details check ACCEL_CONFIG Register
	DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
	DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
	DataStruct->Az = DataStruct->Accel_Z_RAW / 16384.0;
}


void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];
    // Read 6 BYTES of data starting from GYRO_XOUT_H register
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);
    DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
    //convert the RAW values into dps
	//we have to divide according to the Full scale value set in FS_SEL
	//I have configured FS_SEL = 0. So I am dividing by 131.0
	//for more details check GYRO_CONFIG Register
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2];
    int16_t temp;
    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);
    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
}

void get_Zero_Offset(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
	static const float dt = 8.0 / 1000 ;  // 10ms time interval for sampling
	static const int ZERO_OFFSET_COUN = (1 / dt); // frequency of data sampling is 125 Hz
	static int g_GetZeroOffset = 0;
	static float gyroX_offset = 0.0f, gyroY_offset = 0.0f, gyroZ_offset = 0.0f;
	for (g_GetZeroOffset = 0; g_GetZeroOffset < ZERO_OFFSET_COUN; g_GetZeroOffset++)
	{
		MPU6050_Read_Gyro(I2Cx, DataStruct);
		gyroX_offset += DataStruct->Gx * dt;  // calculate the "angular distance traveled" in one second which is equal to the average angular velocity offset
		gyroY_offset += DataStruct->Gy * dt;
		gyroZ_offset += DataStruct->Gz * dt;
	}
	DataStruct->Zero_Offset_X = gyroX_offset;
	DataStruct->Zero_Offset_Y = gyroY_offset;
	DataStruct->Zero_Offset_Z = gyroZ_offset;
}

void get_Yaw(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
	static const float dt = 5.0 / 1000 ;  // time interval for sampling
	DataStruct->KalmanAngleZ += (DataStruct->Gz - DataStruct->Zero_Offset_Z) * dt * 2 * 7.5;
	if (DataStruct->KalmanAngleZ > 360)
		DataStruct->KalmanAngleZ -= 360;
	if (DataStruct->KalmanAngleZ < -360)
		DataStruct->KalmanAngleZ += 360;
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[14];
    int16_t temp;
    // Read 14 BYTES of data starting from ACCEL_XOUT_H register
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;

    // Kalman angle solve
    double dt = (double)(HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
    double roll;
    double roll_sqrt = sqrt(
        DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0)
    {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    }
    else
    {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    }
    else
    {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};
