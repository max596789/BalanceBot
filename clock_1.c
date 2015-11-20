/*
 * clock_1.c
 *
 * Created: 4/19/2015 11:53:28 PM
 *  Author: ECE User
 */ 

#include "clock_1.h"
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#define F_CPU 20000000UL

void initTimer_1()
{
	TCCR1B |= (1<<WGM12);
	TIMSK1 |= (1<<OCIE1A);
}

void startTimer()
{
	
}

void stopTimer()
{
	TCCR1B &= ~(1<<CS00) & ~(1<<CS01) & ~(1<<CS02);
}

void stopWatch(uint32_t frequency)
{
	uint16_t tempValue;
	tempValue = (F_CPU/(2*prescaleValue*frequency)) - 1;
	if (tempValue <= 65534)
	{
		OCR1A = tempValue;
	}
}

void setClockPrescaler_1(prescaler_1 prescale)
{
	prescaleSetting_1 = prescale;
	switch (prescale)
	{
		case no_pre_1:
			TCCR1B &= ~(1<<CS12) & ~(1<<CS11);
			TCCR1B |= (1<<CS10);
			prescaleValue = 1;
		break;
		case pre_8_1:
			TCCR1B &= ~(1<<CS12) & ~(1<<CS10);
			TCCR1B |= (1<<CS11);
			prescaleValue = 8;
		break;
		case pre_64_1:
			TCCR1B &= ~(1<<CS12);
			TCCR1B |= (1<<CS11) | (1<<CS10);
			prescaleValue = 64;
		break;
		case pre_256_1:
			TCCR1B &= ~(1<<CS11) & ~(1<<CS10);
			TCCR1B |= (1<<CS12);
			prescaleValue = 256;
		break;
		case pre_1024_1:
			TCCR1B &= ~(1<<CS11);
			TCCR1B |= (1<<CS12) | (1<<CS10);
			prescaleValue = 1024;
		break;
	}
}