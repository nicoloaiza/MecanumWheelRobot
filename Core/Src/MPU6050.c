
#include "MPU6050.h"

void MPU6050_Init (void)
{
	uint8_t check;
	uint8_t Data;

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)
	{
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}

}


struct Acceleration Read_Accel (void)
{
	struct Acceleration data;
	struct GAccelerationData gaccelerationData;
	uint8_t Rec_Data[6];
	int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW = 0;
	float Ax, Ay, Az;

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);


	Ax = Accel_X_RAW/16384.0;
	Ay = Accel_Y_RAW/16384.0;
	Az = Accel_Z_RAW/16384.0;
	data.Accel_X_RAW = Accel_X_RAW;
	data.Accel_Y_RAW = Accel_Y_RAW;
	data.Accel_Z_RAW = Accel_Z_RAW;
	gaccelerationData.Ax = Ax;
	gaccelerationData.Ay = Ay;
	gaccelerationData.Az = Az;
	data.GAccel = gaccelerationData;
	return data;
}


struct Gyro Read_Gyro (void)
{
	struct Gyro data;
	struct GGyroData gyroData;
	uint8_t Rec_Data[6];
	int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW = 0;
	float Gx, Gy, Gz;

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;
	data.Gyro_X_RAW = Gyro_X_RAW;
	data.Gyro_Y_RAW = Gyro_Y_RAW;
	data.Gyro_Z_RAW = Gyro_Z_RAW;
	gyroData.Gx = Gx;
	gyroData.Gy = Gy;
	gyroData.Gz = Gz;
	data.GGyro = gyroData;
	return data;
}
