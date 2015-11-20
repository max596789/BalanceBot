/*
 * clock_1.h
 *
 * Created: 4/19/2015 11:53:44 PM
 *  Author: ECE User
 */ 


#ifndef CLOCK_1_H_
#define CLOCK_1_H_

#include <stdint.h>

typedef enum
{
	no_pre_1,
	pre_8_1,
	pre_64_1,
	pre_256_1,
	pre_1024_1
}prescaler_1;

uint16_t prescaleValue; 
prescaler_1 prescaleSetting_1;

void initTimer_1();
void startTimer();
void stopTimer();
uint16_t getTime();
void stopWatch(uint32_t frequency);
void setClockPrescaler_1(prescaler_1 prescale);

#endif /* CLOCK_1_H_ */