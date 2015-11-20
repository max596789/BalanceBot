/*
 * BalanceBot.c
 *
 * Created: 1/12/2015 2:39:17 AM
 *  Author: Jacob
 */ 

#define F_CPU 20000000UL
#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "MPU6050.h"
#include "I2C.h"
#include "motorDriver.h"
#include "clock_1.h"

volatile int sampleSensorFlag = 0;
#define SETPOINT -90

int main(void)
{
	sei();
	gyroSensitivity gyro_s = DegPerSec_250;
	accelSensitivity accel_s = two_g;
	int16_t X_raw_accel, Y_raw_accel, Z_raw_accel, X_raw_gyro, Y_raw_gyro, Z_raw_gyro;
	volatile float X_accel, Y_accel, Z_accel, X_gyro, Y_gyro, Z_gyro;
	volatile angles results;
	uint8_t data[1];
	uint16_t FS = 12;
	float dt = .04167;
	volatile float gyroangle = 0;
	volatile float angle = 0;
	volatile float previousAngle = -90;
	volatile float pulses;
	initMotorControl();
	initTimer_1();
	setClockPrescaler_1(pre_64_1);
	stopWatch(FS);
	Direction direction = forward;
	
	initPID((1/1.8),0.0, .25);
	
	MPU6050_Init(0);
	MPU6050_SampleRateDivider(0x00);
	MPU6050_setConfiguration(0x00, 0x06);
	MPU6050_PowerManagment(0x00, 0x00);
	I2C_recieve(MPU6050Address, MPU6050_WHO_AM_I, data, 1);
	
	enable_motors(0x00); //may want to change this to be on!
	
	//Add in delay for initialization of sensor
	
	while(1)
	{
		if (sampleSensorFlag == 1)
		{
			MPU6050_GetAccelXYZ(&X_raw_accel, &Y_raw_accel, &Z_raw_accel);
			X_accel = getAccelerationValue(X_raw_accel, accel_s);
			Y_accel = getAccelerationValue(Y_raw_accel, accel_s);
			Z_accel = getAccelerationValue(Z_raw_accel, accel_s);
			
			MPU6050_GetGyroXYZ(&X_raw_gyro, &Y_raw_gyro, &Z_raw_gyro);
			X_gyro = getGyroValue(X_raw_gyro, gyro_s);
			Y_gyro = getGyroValue(Y_raw_gyro, gyro_s);
			Z_gyro = getGyroValue(Z_raw_gyro, gyro_s);

			results = getAngleFromAccel(X_accel,Y_accel,Z_accel);
			
			gyroangle = previousAngle + Y_gyro*dt;
			previousAngle = angle;
		
			angle = 0.1*results.Pitch + 0.9*gyroangle;
			if((angle < -89.0) && (angle > -91.0))
			{
				enable_motors(0x00);//may want to change this to be on!
				direction = forward;
			}
			else if(angle < -91.0)
			{
				pulses = balancePID(SETPOINT, angle, dt);
				enable_motors(0x01);
				direction = forward;
				motorControl(floor(pulses), direction);
			}
			else if(angle > -89.0)
			{
				pulses = balancePID(SETPOINT, angle, dt);
				enable_motors(0x01);
				direction = backward;
				motorControl(floor(-pulses), direction);
			}
			sampleSensorFlag = 0;
		}
	}
}

ISR(TIMER1_COMPA_vect)
{
	sampleSensorFlag = 1;
}