/*
 * motorDriver.h
 *
 * Created: 1/12/2015 2:53:22 AM
 *  Author: Jacob
 */ 


#ifndef MOTORDRIVER_H_
#define MOTORDRIVER_H_

#include <stdint.h>

typedef enum 
{
	forward,
	backward
}Direction;

//PID variables
float Kp;
float Ki;
float Kd;
float sumError;
float lastError;

void initMotorControl();
void initPID(float P_gain, float I_gain, float D_gain);
float balancePID(int16_t wantedAngle, float measuredAngle, float dt);
void motorControl(uint8_t powerValue, Direction direction);
void RightMotorSpeed(uint8_t speed);
void LeftMotorSpeed(uint8_t speed);
void forwards();
void backwards();
void enable_motors(uint8_t on_off);

#endif /* MOTORDRIVER_H_ */