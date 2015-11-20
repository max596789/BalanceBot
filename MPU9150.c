/*
 * MPU9150.c
 *
 * Created: 1/12/2015 2:54:02 AM
 *  Author: Jacob
 */ 

#include "MPU9150.h"
#include "I2C.h"
#include <math.h>

void MPU9150Init(uint8_t ad0)
{
	MPU9150Address = MPU9150_Address + ad0;
	initI2C();
}

//Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
//where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7), and 1kHz
//when the DLPF is enabled (see Register 26).
void MPU9150_SampleRateDivider(uint8_t division)
{
	uint8_t data[1] = {division};
	I2C_write(MPU9150Address,MPU9150_SMPLRT_DIV, data, 1);
}

void MPU9150_setConfiguration(uint8_t externalFSYNC, uint8_t DLPF)
{
	uint8_t data[1] = {DLPF | (externalFSYNC<<3)};
	I2C_write(MPU9150Address,MPU9150_CONFIG, data, 1);
}

//Upon power up, the MPU-9150 clock source defaults to the internal oscillator. However, it is highly
//recommended that the device be configured to use one of the gyroscopes (or an external clock
//source) as the clock reference for improved stability
void MPU9150_PowerManagment(uint8_t managmentReg1, uint8_t managmentReg2)
{
	uint8_t data[2] = {managmentReg1, managmentReg2};
	I2C_write(MPU9150Address,MPU9150_PWR_MGMT_1,data,2);
}

void MPU9150_GetAccelXYZ(int16_t *X, int16_t *Y, int16_t *Z)
{
	uint8_t data[6]; 
	I2C_recieve(MPU9150Address, MPU9150_ACCEL_XOUT_H, data, 6);
	
	*X = data[1] + ((uint16_t)data[0] << 8);
	*Y = data[3] + ((uint16_t)data[2] << 8);
	*Z = data[5] + ((uint16_t)data[4] << 8);
}

void MPU9150_GetGyroXYZ(int16_t *X, int16_t *Y, int16_t *Z)
{
	uint8_t data[6];
	I2C_recieve(MPU9150Address, MPU9150_GYRO_XOUT_H, data, 6);
	
	*X = data[1] + ((uint16_t)data[0] << 8);
	*Y = data[3] + ((uint16_t)data[2] << 8);
	*Z = data[5] + ((uint16_t)data[4] << 8);
}

// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
// atan2 outputs the value of -? to ? (radians)
angles getAngleFromAccel(double accelX, double accelY, double accelZ)
{
	double roll  = atan2(accelY, accelZ) * RAD_TO_DEG;
	double pitch = atan(-accelX / sqrt(accelY * accelY + accelZ * accelZ)) * RAD_TO_DEG;
	angles data = {pitch, roll};
	return data;
}