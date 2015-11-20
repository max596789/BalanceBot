/*
 * MPU9150.c
 *
 * Created: 1/12/2015 2:54:02 AM
 *  Author: Jacob
 */ 

#include "MPU6050.h"
#include "I2C.h"
#include <math.h>

void MPU6050_Init(uint8_t ad0)
{
	MPU6050Address = MPU6050_Address | ad0;
	initI2C();
}

//Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
//where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7), and 1kHz
//when the DLPF is enabled (see Register 26).
void MPU6050_SampleRateDivider(uint8_t division)
{
	uint8_t data[1] = {division};
	I2C_write(MPU6050Address,MPU6050_SMPLRT_DIV, data, 1,1);
}

void MPU6050_setConfiguration(uint8_t externalFSYNC, uint8_t DLPF)
{
	uint8_t data[1] = {DLPF | (externalFSYNC<<3)};
	I2C_write(MPU6050Address,MPU6050_CONFIG, data, 1,1);
}

//Upon power up, the MPU-9150 clock source defaults to the internal oscillator. However, it is highly
//recommended that the device be configured to use one of the gyroscopes (or an external clock
//source) as the clock reference for improved stability
void MPU6050_PowerManagment(uint8_t managmentReg1, uint8_t managmentReg2)
{
	uint8_t data[2] = {managmentReg1, managmentReg2};
	I2C_write(MPU6050Address,MPU6050_PWR_MGMT_1,data,2,1);
}

void MPU6050_GetAccelXYZ(int16_t *X, int16_t *Y, int16_t *Z)
{
	uint8_t data_H[1];
	uint8_t data_L[1];
	I2C_recieve(MPU6050Address, MPU6050_ACCEL_XOUT_H, data_H, 1);
	I2C_recieve(MPU6050Address, MPU6050_ACCEL_XOUT_L, data_L, 1);
	
	*X = convertFrom8To16(data_H[0],data_L[0]);
	I2C_recieve(MPU6050Address, MPU6050_ACCEL_YOUT_H, data_H, 1);
	I2C_recieve(MPU6050Address, MPU6050_ACCEL_YOUT_L, data_L, 1);
	
	*Y = convertFrom8To16(data_H[0],data_L[0]);
	I2C_recieve(MPU6050Address, MPU6050_ACCEL_ZOUT_H, data_H, 1);
	I2C_recieve(MPU6050Address, MPU6050_ACCEL_ZOUT_L, data_L, 1);
	*Z = convertFrom8To16(data_H[0],data_L[0]);

}

void MPU6050_GetGyroXYZ(int16_t *X, int16_t *Y, int16_t *Z)
{
	uint8_t data_H[1];
	uint8_t data_L[1];
	I2C_recieve(MPU6050Address, MPU6050_GYRO_XOUT_H, data_H, 1);
	I2C_recieve(MPU6050Address, MPU6050_GYRO_XOUT_L, data_L, 1);
	*X = convertFrom8To16(data_H[0],data_L[0]);
	I2C_recieve(MPU6050Address, MPU6050_GYRO_YOUT_H, data_H, 1);
	I2C_recieve(MPU6050Address, MPU6050_GYRO_YOUT_L, data_L, 1);
	*Y = convertFrom8To16(data_H[0],data_L[0]);
	I2C_recieve(MPU6050Address, MPU6050_GYRO_ZOUT_H, data_H, 1);
	I2C_recieve(MPU6050Address, MPU6050_GYRO_ZOUT_L, data_L, 1);
	*Z = convertFrom8To16(data_H[0],data_L[0]);

}

// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 28 and eq. 29
// atan2 outputs the value of -? to ? (radians)
angles getAngleFromAccel(float accelX, float accelY, float accelZ)
{
	//float roll  = atan2(accelY, accelZ) * RAD_TO_DEG;
	//float pitch = atan2(-accelX,sqrt(accelY * accelY + accelZ * accelZ)) * RAD_TO_DEG;
	float pitch  = atan2f(-accelX, accelZ) * RAD_TO_DEG;
	float roll = atan2f(accelY,sqrt(accelX * accelX + accelZ * accelZ)) * RAD_TO_DEG;
	angles data = {pitch, roll};
	return data;
}

uint16_t convertFrom8To16(uint8_t dataHigh, uint8_t dataLow)
{
	uint16_t dataBoth = 0x0000;
	dataBoth = dataHigh;
	dataBoth = dataBoth << 8;
	dataBoth |= dataLow;
	return dataBoth;
}


float getAccelerationValue(int16_t value, accelSensitivity sensitivity)
{
	float result;
	switch(sensitivity)
	{
		case two_g:
			result = ((float)value)/16384.0;
			break;
		case four_g:
			result = (float)value/8192.0;
			break;
		case eight_g:
			result = (float)value/4096.0;
			break;
		case sixteen_g:
			result = (float)value/2048.0;
			break;				
		default:
			result = (float)value/16384.0;
			break;
	}
	return result;
}

float getGyroValue(int16_t value, gyroSensitivity sensitivity)
{
	float result;
	switch(sensitivity)
	{
		case DegPerSec_250:
			result = value/131.0f;
			break;
		case DegPerSec_500:
			result = value/65.5f;
			break;
		case DegPerSec_1000:
			result = value/32.8f;
			break;
		case DegPerSec_2000:
			result = value/16.4f;
			break;
		default:
			result = value/131.0f;
			break;
	}
	return result;
} 
