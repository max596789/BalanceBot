/*
 * clock.c
 *
 * Created: 1/12/2015 2:57:22 AM
 *  Author: Jacob
 * Use the 8 bit clock OCA0 and OCB0  
 */ 

#include "clock_0.h"
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#define F_CPU 20000000UL

void initOC0A(int pin)
{
	if (pin >= 1)
	{	
		DDRD |= (1<<PIND6);
	}
	else
	{	disconnectOC0A();	}
	TCCR0A |= (1<<WGM01);
	TIMSK0 |= (1<<OCIE0A);
}

void initOC0B(int pin)
{
	if (pin >= 1)
	{	
		DDRD |= (1<<PIND5); 
	}
	else
	{	disconnectOC0B();	}
	TCCR0A |= (1<<WGM01);
	TIMSK0 |= (1<<OCIE0B);
}

void setClockPrescaler_0(prescaler prescale)
{
	prescaleSetting_0 = prescale;
	switch (prescale)
	{
		case no_pre_0:
			TCCR0B &= ~(1<<CS02) & ~(1<<CS01);
			TCCR0B |= (1<<CS00);
			prescaleValue = 1;
		break;
		case pre_8_0:
			TCCR0B &= ~(1<<CS02) & ~(1<<CS00);
			TCCR0B |= (1<<CS01);
			prescaleValue = 8;
		break;
		case pre_64_0:
			TCCR0B &= ~(1<<CS02);
			TCCR0B |= (1<<CS01) | (1<<CS00);
			prescaleValue = 64;
		break;
		case pre_256_0:
			TCCR0B &= ~(1<<CS01) & ~(1<<CS00);
			TCCR0B |= (1<<CS02);
			prescaleValue = 256;
		break;
		case pre_1024_0:
			TCCR0B &= ~(1<<CS01);
			TCCR0B |= (1<<CS02) | (1<<CS00);
			prescaleValue = 1024;
		break;
	}
}

void changeOC0AFrequency(uint32_t frequency) //Changes frequency 
{
	uint16_t tempValue;

	tempValue = (F_CPU/(2*prescaleValue*frequency)) - 1;
	if (tempValue <= 255)
	{
		OCR0A = tempValue;
	}
}

void changeOC0BDutyCycle(uint32_t frequency)	// Changes Duty Cycle
{
	uint16_t tempValue;

	tempValue = (F_CPU/(2*prescaleValue*frequency)) - 1;
	if (tempValue <= 255)
	{
		OCR0B = tempValue;
	}
}

void sendClockPulses_0(uint16_t pulsesOC0A, uint16_t pulsesOC0B)
{
	pulseWantedOC0A = pulsesOC0A;
	pulseWantedOC0B = pulsesOC0B;
	numberOfPulsesOC0A = 1;
	numberOfPulsesOC0B = 1;
	toggleOC0A();
	toggleOC0B();
}

void sendOC0AClockPulses(uint16_t numberOfPulses)
{
	pulseWantedOC0A = numberOfPulses;
	numberOfPulsesOC0A = 1;
	toggleOC0A();
}
void sendOC0BClockPulses(uint16_t numberOfPulses)
{
	pulseWantedOC0B = numberOfPulses;
	numberOfPulsesOC0B = 1;
	toggleOC0B();
}

void disconnectOC0A()
{	TCCR0A &= ~(1<<COM0A0) & ~(1<<COM0A1);}

void disconnectOC0B()
{	TCCR0A &= ~(1<<COM0B0) & ~(1<<COM0B1);}
	
void toggleOC0A()
{	TCCR0A |= (1<<COM0A0);}
	
void toggleOC0B()
{	TCCR0A |= (1<<COM0B0);}

void stopClocks()
{	TCCR0B &= ~(1<<CS00) & ~(1<<CS01) & ~(1<<CS02);}
	
ISR(TIMER0_COMPA_vect)
{
	if ((numberOfPulsesOC0A/2) != pulseWantedOC0A)
	{	numberOfPulsesOC0A++;}
	else
	{	disconnectOC0A();	}
}

ISR(TIMER0_COMPB_vect)
{
	if ((numberOfPulsesOC0B/2) != pulseWantedOC0B)
	{	numberOfPulsesOC0B++;}
	else
	{	disconnectOC0B();	}
}
