/*
 * motorDriver.c
 *
 * Created: 1/12/2015 2:53:07 AM
 *  Author: Jacob
 *
 * 
 */ 

#include "motorDriver.h"
#include "clock_0.h"
#include "clock_2.h"
#include <stdint.h>
#include <avr/io.h>
#include <math.h>

//Pin relations for L297
#define rightMotorDirection 0
#define rightMotorEnable 1
#define rightMotorStepSize 2
#define leftMotorDirection 0
#define leftMotorEnable 1
#define leftMotorStepSize 2

 int MAX = 15;
 int NEG_MAX = -15;

void initMotorControl()
{
	DDRC |= (1<<leftMotorDirection) | (1<<leftMotorEnable) | (1<<leftMotorStepSize);
	DDRD |= (1<<rightMotorDirection) | (1<<rightMotorEnable) | (1<<rightMotorStepSize);
	PORTC &= ~(1<<leftMotorDirection) & ~(1<<leftMotorEnable) & ~(1<<leftMotorStepSize);
	PORTD &= ~(1<<rightMotorDirection) & ~(1<<rightMotorEnable) & ~(1<<rightMotorStepSize);	
	
	initOC0A(1);		//Outputs on pin D6
	initOC0B(0);
	setClockPrescaler_0(pre_1024_0);
	changeOC0AFrequency(350);
	
	initOC2A(0);
	initOC2B(1);		//Outputs on pin D3
	setClockPrescaler_2(pre_1024_2);
	changeOC2AFrequency(350);
}

void initPID(float P_gain, float I_gain, float D_gain)
{	
	Kp = P_gain;
	Ki = I_gain;
	Kd = D_gain;
	sumError = 0;
	lastError = 0;
}

float balancePID(int16_t wantedAngle, float measuredAngle, float dt)
{
	volatile float error, P, I, D, PID;
	error = wantedAngle - measuredAngle;
	sumError += error;
	P = Kp*error;
	if (dt == 0)
	{
		I = 0;
		D = 0;
	}
	else
	{
		I = Ki*sumError*dt;
		D = Kd*(error - lastError)/dt;
	}
	lastError = error;
	PID = P + I + D;
	if (PID > MAX)
	{
		PID = MAX;
	}
	if (PID < NEG_MAX)
	{
		PID = NEG_MAX;
	}
	return PID;
}

void motorControl(uint8_t pulses, Direction direction)
{
	if (direction == forward)
	{
		forwards();
		sendClockPulses_2(0,pulses);
		sendClockPulses_0(pulses,0);
	}
	else
	{
		backwards();
		sendClockPulses_2(0,pulses);
		sendClockPulses_0(pulses,0);
	}
}

void forwards()
{
	PORTC |= (1<<leftMotorDirection);
	PORTD |= (1<<rightMotorDirection);
}

void backwards()
{
	PORTC &= ~(1<<leftMotorDirection);
	PORTD &= ~(1<<rightMotorDirection);
}

void enable_motors(uint8_t on_off)
{
	if (on_off == 0)
	{
		PORTC &=  ~(1<<leftMotorEnable);
		PORTD &= ~(1<<rightMotorEnable);
	} 
	else
	{
		PORTC |=  (1<<leftMotorEnable);
		PORTD |= (1<<rightMotorEnable);		
	}

}

void RightMotorSpeed(uint8_t speed)
{
	//change clock frequency
}

void LeftMotorSpeed(uint8_t speed)
{
	
}