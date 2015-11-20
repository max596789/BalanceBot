/*
 * clock_2.c
 *
 * Created: 3/16/2015 7:09:08 PM
 *  Author: ECE User
 */ 

/*
 * clock.c
 *
 * Created: 1/12/2015 2:57:22 AM
 *  Author: Jacob
 * Use the 8 bit clock OCA0 and OCB0  
 */ 

#include "clock_2.h"
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#define F_CPU 20000000UL

void initOC2A(int pin)
{
	if (pin >= 1)
	{	DDRB |= (1<<PINB3); } 
	else
	{	disconnectOC2A();	}
	TCCR2A |= (1<<WGM21);
	TIMSK2 |= (1<<OCIE2A);
}

void initOC2B(int pin)
{
	if (pin >= 1)
	{	DDRD |= (1<<PIND3); }
	else
	{	disconnectOC2B();	}
	TCCR2A |= (1<<WGM21);
	TIMSK2 |= (1<<OCIE2B);
}

void setClockPrescaler_2(prescaler_2 prescale)
{
	prescaleSetting_2 = prescale;
	switch (prescale)
	{
		case no_pre_2:
			TCCR2B &= ~(1<<CS22) & ~(1<<CS21);
			TCCR2B |= (1<<CS20);
			prescaleValue = 1;
		break;
		case pre_8_2:
			TCCR2B &= ~(1<<CS22) & ~(1<<CS20);
			TCCR2B |= (1<<CS21);
			prescaleValue = 8;
		break;
		case pre_32_2:
			TCCR2B &= ~(1<<CS22);
			TCCR2B |= (1<<CS21) | (1<<CS20);
			prescaleValue = 32;
		break;
		case pre_64_2:
			TCCR2B &= ~(1<<CS21) & ~(1<<CS20);
			TCCR2B |= (1<<CS22);
			prescaleValue = 64;
		break;
		case pre_128_2:
			TCCR2B &= ~(1<<CS21);
			TCCR2B |= (1<<CS22) | (1<<CS20);
			prescaleValue = 128;
		break;
		case pre_256_2:
			TCCR2B &= ~(1<<CS20);
			TCCR2B |= (1<<CS22) | (1<<CS21);
			prescaleValue = 256;
		break;
		case pre_1024_2:
			TCCR2B |= (1<<CS22) | (1<<CS21) | (1<<CS20);
			prescaleValue = 1024;
		break;
	}
}

void changeOC2AFrequency(uint32_t frequency) //Changes frequency 
{
	uint16_t tempValue;

	tempValue = (F_CPU/(2*prescaleValue*frequency)) - 1;
	if (tempValue <= 255)
	{
		OCR2A = tempValue;
	}
}

void changeOC2BDutyCycle(uint32_t frequency)	// Changes Duty Cycle
{
	uint16_t tempValue;

	tempValue = (F_CPU/(2*prescaleValue*frequency)) - 1;
	if (tempValue <= 255)
	{
		OCR2B = tempValue;
	}
}

void sendClockPulses_2(uint16_t pulsesOC2A, uint16_t pulsesOC2B)
{
	pulseWantedOC2A = pulsesOC2A;
	pulseWantedOC2B = pulsesOC2B;
	numberOfpulsesOC2A = 1;
	numberOfpulsesOC2B = 1;
	toggleOC2A();
	toggleOC2B();
}

void sendOC2AClockPulses(uint16_t numberOfPulses)
{
	pulseWantedOC2A = numberOfPulses;
	numberOfpulsesOC2A = 1;
	toggleOC2A();
}
void sendOC2BClockPulses(uint16_t numberOfPulses)
{
	pulseWantedOC2B = numberOfPulses;
	numberOfpulsesOC2B = 1;
	toggleOC2B();
}

void disconnectOC2A()
{	TCCR2A &= ~(1<<COM2A0) & ~(1<<COM2A1);}

void disconnectOC2B()
{	TCCR2A &= ~(1<<COM2B0) & ~(1<<COM2B1);}
	
void toggleOC2A()
{	TCCR2A |= (1<<COM2A0);}
	
void toggleOC2B()
{	TCCR2A |= (1<<COM2B0);}

void stopClocks_2()
{	TCCR2B &= ~(1<<CS20) & ~(1<<CS21) & ~(1<<CS22);}
	
ISR(TIMER2_COMPA_vect)
{
	if ((numberOfpulsesOC2A/2) != pulseWantedOC2A)
	{	numberOfpulsesOC2A++;}
	else
	{	disconnectOC2A();	}
}

ISR(TIMER2_COMPB_vect)
{
	if ((numberOfpulsesOC2B/2) != pulseWantedOC2B)
	{	numberOfpulsesOC2B++;}
	else
	{	disconnectOC2B();	}
}
