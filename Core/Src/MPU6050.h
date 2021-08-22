
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "main.h"


#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

extern I2C_HandleTypeDef hi2c2;

struct GAccelerationData {
	float Ax;
	float Ay;
	float Az;
};

struct GGyroData {
	float Gx;
	float Gy;
	float Gz;
};

struct Acceleration {
	int16_t Accel_X_RAW;
	int16_t Accel_Y_RAW;
	int16_t Accel_Z_RAW;
	struct GAccelerationData GAccel;
};

struct Gyro {
	int16_t Gyro_X_RAW;
	int16_t Gyro_Y_RAW;
	int16_t Gyro_Z_RAW;
	struct GGyroData GGyro;
};


void MPU6050_Init();
struct Acceleration Read_Accel (void);
struct Gyro Read_Gyro (void);
